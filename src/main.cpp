#include <Arduino.h>
#include <WebSocketsClient.h>
#include <WiFi.h>
#include <WiFiManager.h>
#include <ArduinoJson.h>
#include "driver/i2s_std.h"
#include "driver/gpio.h"
#include "mbedtls/base64.h"
#include "secrets.h"
#include <atomic>

// ==========================================
// CONFIGURACIÓN DE PINES (XH-S3E-AI)
// ==========================================

// --- ALTAVOZ ---
#define SPK_BCLK GPIO_NUM_15
#define SPK_WS GPIO_NUM_16
#define SPK_DOUT GPIO_NUM_7

// --- MICRÓFONO (Ocultos en la placa) ---
#define MIC_WS GPIO_NUM_4  // LRCK
#define MIC_SCK GPIO_NUM_5 // BCLK
#define MIC_SD GPIO_NUM_6  // DIN

// --- BOTONES DE VOLUMEN (XH-S3E-AI) ---
#define PIN_VOL_UP GPIO_NUM_40
#define PIN_VOL_DOWN GPIO_NUM_39

#define BUILTIN_LED_GPIO GPIO_NUM_48
#define BOOT_BUTTON_GPIO GPIO_NUM_0

// ==========================================
// ESTRUCTURAS Y GLOBALES
// ==========================================
struct AudioChunk
{
  uint8_t *data;
  size_t len;
};

struct LogMsg
{
  uint32_t length;
  char snippet[64];
};

TaskHandle_t WiFiTaskHandle;
TaskHandle_t AudioTaskHandle;
TaskHandle_t MicTaskHandle;

WebSocketsClient webSocket;
SemaphoreHandle_t wsMutex;

i2s_chan_handle_t tx_handle; // Altavoz
i2s_chan_handle_t rx_handle; // Micrófono

QueueHandle_t logQueue;
QueueHandle_t audioQueue;

enum SystemState
{
  STATE_DISCONNECTED,
  STATE_CONNECTED_WS,
  STATE_SETUP_SENT,
  STATE_SETUP_COMPLETE,
  STATE_HELLO_SENT,
  STATE_ACTIVE
};

volatile SystemState currentState = STATE_DISCONNECTED;

// Nivel de volumen (0 a 20)
// Iniciamos en 15 (75%)
std::atomic<int> globalVolume{3};
const int maxVolume = 20;

std::atomic<bool> micEnabled{true};

// ==========================================
// PROTOTIPOS
// ==========================================
void setupSpeakerI2S();
void setupMicrophoneI2S();
void procesarAudioStreaming(const char *b64Data);
void webSocketEvent(WStype_t type, uint8_t *payload, size_t length);
void detectMessageType(const uint8_t *data, size_t len);
void TaskWiFi(void *pvParameters);
void TaskAudio(void *pvParameters);
void TaskMicrophone(void *pvParameters);

// ==========================================
// SETUP
// ==========================================

void setup()
{
  Serial.begin(460800);
  delay(1000);

  Serial.println("--- ASISTENTE DE VOZ GEMINI (XH-S3E-AI) ---");
  Serial.printf("Memoria PSRAM Total: %d bytes\n", ESP.getPsramSize());

  wsMutex = xSemaphoreCreateMutex();

  // 1. Inicializar Hardware
  setupSpeakerI2S();
  setupMicrophoneI2S();

  pinMode(PIN_VOL_UP, INPUT_PULLUP);
  pinMode(PIN_VOL_DOWN, INPUT_PULLUP);
  pinMode(BOOT_BUTTON_GPIO, INPUT_PULLUP);

  neopixelWrite(BUILTIN_LED_GPIO, 0, 0, 50);

  // 2. Crear Colas
  logQueue = xQueueCreate(20, sizeof(LogMsg));
  // Buffer grande para evitar cortes en la recepción
  audioQueue = xQueueCreate(500, sizeof(AudioChunk));

  // 3. Tareas
  // Core 1: Audio y Micro (Procesamiento pesado)
  xTaskCreatePinnedToCore(TaskAudio, "Audio_Player", 10240, NULL, 10, &AudioTaskHandle, 1);
  xTaskCreatePinnedToCore(TaskMicrophone, "Mic_Capture", 16384, NULL, 5, &MicTaskHandle, 1);

  // Core 0: WiFi y WebSocket (Comunicaciones)
  xTaskCreatePinnedToCore(TaskWiFi, "WiFi_Manager", 25 * 1024, NULL, 1, &WiFiTaskHandle, 0);
}

void loop()
{
  static unsigned long lastLog = 0;
  static unsigned long lastBtnPress = 0;
  static bool lastBootBtnState = HIGH;

  // --- LECTURA BOTONES VOLUMEN (Tu código anterior) ---
  if (millis() - lastBtnPress > 200)
  {
    if (digitalRead(PIN_VOL_UP) == LOW)
    {
      if (globalVolume < maxVolume)
      {
        globalVolume++;
        Serial.printf(">>> VOLUMEN: %d / %d\n", globalVolume.load(), maxVolume);
      }
      lastBtnPress = millis();
    }
    if (digitalRead(PIN_VOL_DOWN) == LOW)
    {
      if (globalVolume > 0)
      {
        globalVolume--;
        Serial.printf(">>> VOLUMEN: %d / %d\n", globalVolume.load(), maxVolume);
      }
      lastBtnPress = millis();
    }

    // --- LECTURA BOTÓN BOOT (MUTE/UNMUTE) ---
    bool currentBootBtnState = digitalRead(BOOT_BUTTON_GPIO);
    if (currentBootBtnState == LOW && lastBootBtnState == HIGH)
    { // Pulsado
      micEnabled = !micEnabled.load();
      Serial.printf(">>> MICROFONO: %s\n", micEnabled.load() ? "ACTIVO" : "SILENCIADO");

      // Actualizar LED: Verde = Activo, Rojo = Mute
      if (micEnabled.load())
      {
        neopixelWrite(BUILTIN_LED_GPIO, 0, 50, 0); // Verde suave
      }
      else
      {
        neopixelWrite(BUILTIN_LED_GPIO, 50, 0, 0); // Rojo suave
      }
      lastBtnPress = millis();
    }
    lastBootBtnState = currentBootBtnState;
  }

  // Monitor de estado
  if (millis() - lastLog > 5000)
  {
    lastLog = millis();
    Serial.printf("[SYSTEM] Mic: %s | Vol: %d | Heap: %d\n",
                  micEnabled.load() ? "ON" : "OFF", globalVolume.load(), ESP.getFreeHeap());
  }

  vTaskDelay(pdMS_TO_TICKS(10));
}

// ==========================================
// TAREA AUDIO (REPRODUCCIÓN)
// ==========================================
// ==========================================
// TAREA AUDIO (CON CONTROL DE VOLUMEN)
// ==========================================
void TaskAudio(void *pvParameters)
{
  AudioChunk chunk;
  size_t bytes_written = 0;
  const size_t silence_len = 1024;
  uint8_t *silence_buf = (uint8_t *)calloc(silence_len, 1);

  while (true)
  {
    if (xQueueReceive(audioQueue, &chunk, pdMS_TO_TICKS(100)) == pdTRUE)
    {
      if (chunk.len > 0 && chunk.data != NULL)
      {
        // --- APLICAR VOLUMEN ---
        // El audio es PCM de 16 bits (int16_t).
        // Convertimos el puntero de bytes a puntero de int16
        int16_t *pcm_samples = (int16_t *)chunk.data;
        size_t sample_count = chunk.len / 2; // Cada muestra son 2 bytes

        // Calculamos el factor de escala (ej. 10/20 = 0.5)
        // Usamos float para la precisión y luego casteamos a int16
        float volumeFactor = (float)globalVolume / (float)maxVolume;
        volumeFactor = volumeFactor * 1.0f; // Escalamos a rango completo 0.0 - 2.0
        // Solo procesamos si el volumen NO es el máximo (ahorrar CPU)
        if (globalVolume != maxVolume)
        {
          for (size_t i = 0; i < sample_count; i++)
          {
            // Multiplicamos cada muestra por el factor
            int32_t val = pcm_samples[i] * volumeFactor;
            // Asignamos de nuevo
            pcm_samples[i] = (int16_t)val;
          }
        }
        // -----------------------

        i2s_channel_write(tx_handle, chunk.data, chunk.len, &bytes_written, portMAX_DELAY);
      }
      free(chunk.data);
    }
    else
    {
      i2s_channel_write(tx_handle, silence_buf, silence_len, &bytes_written, pdMS_TO_TICKS(10));
    }
  }
  free(silence_buf);
}

// ==========================================
// TAREA MICRÓFONO (CAPTURA)
// ==========================================
void TaskMicrophone(void *pvParameters)
{
  size_t bytes_read = 0;

  // Configuración 32-bit Stereo (Requerido por hardware MSM261 en ESP32-S3)
  const size_t buffer_size_32 = 2048;
  int32_t *i2s_read_buff_32 = (int32_t *)ps_malloc(buffer_size_32);

  // Buffer de salida (16-bit Mono) para API Gemini
  int16_t *final_pcm_16 = (int16_t *)ps_malloc(1024);

  // Buffers Base64 y JSON
  size_t b64_size = (1024 + 2) / 3 * 4 + 1;
  unsigned char *b64_buff = (unsigned char *)ps_malloc(b64_size);
  char *json_buff = (char *)ps_malloc(b64_size + 256);

  Serial.println("Microfono Listo. Esperando conexión...");

  while (true)
  {
    // Solo enviamos audio si la sesión está iniciada Y el micro está activo (NO está en Mute)
    if (currentState >= STATE_HELLO_SENT && micEnabled.load())
    {

      // 1. Leer HW (Bloqueante con timeout)
      esp_err_t res = i2s_channel_read(rx_handle, i2s_read_buff_32, buffer_size_32, &bytes_read, pdMS_TO_TICKS(1000));

      if (res == ESP_OK && bytes_read > 0)
      {

        // 2. Procesar Audio (Downsampling 32->16 bit, Stereo->Mono)
        int samples_read = bytes_read / 4;
        int output_index = 0;

        for (int i = 0; i < samples_read; i += 2)
        {
          int32_t sample32 = i2s_read_buff_32[i];

          // Desplazamiento de bits (Bit Shift)
          // >> 14 es conservador. Si se oye bajo, prueba >> 12.
          int16_t sample16 = (sample32 >> 14);

          final_pcm_16[output_index++] = sample16;
        }

        // 3. Codificar y Enviar
        size_t pcm_bytes_len = output_index * 2;
        size_t b64_len = 0;

        mbedtls_base64_encode(b64_buff, b64_size, &b64_len, (unsigned char *)final_pcm_16, pcm_bytes_len);
        b64_buff[b64_len] = '\0';

        // Usamos sprintf para velocidad
        int json_len = sprintf(json_buff,
                               "{\"realtimeInput\":{\"audio\":{\"data\":\"%s\",\"mimeType\":\"audio/pcm;rate=16000\"}}}",
                               (char *)b64_buff);

        // Enviar protegido por Mutex
        if (xSemaphoreTake(wsMutex, pdMS_TO_TICKS(50)) == pdTRUE)
        {
          webSocket.sendTXT(json_buff, json_len);
          xSemaphoreGive(wsMutex);
        }
      }
    }
    else
    {
      // Espera pasiva si no estamos conectados o el MICRO ESTÁ SILENCIADO
      // Esto ahorra batería y uso de CPU cuando estás en "Mute"
      vTaskDelay(pdMS_TO_TICKS(100));
    }
  }
}

// ==========================================
// TAREA WIFI & WEBSOCKET
// ==========================================
void TaskWiFi(void *pvParameters)
{
  WiFiManager wm;
  // wm.resetSettings(); // Descomentar si cambias de red
  if (!wm.autoConnect("ESP32_GEMINI"))
    ESP.restart();

  Serial.println("WiFi Conectado.");

  webSocket.setReconnectInterval(5000);
  webSocket.enableHeartbeat(15000, 3000, 2);
  String baseUrl = "/ws/google.ai.generativelanguage.v1alpha.GenerativeService.BidiGenerateContent?key=";
  String fullUrl = baseUrl + String(GEMINI_API_KEY);
  webSocket.beginSSL("generativelanguage.googleapis.com", 443, fullUrl);
  webSocket.onEvent(webSocketEvent);

  for (;;)
  {
    // Protegemos el loop del WS con Mutex
    if (xSemaphoreTake(wsMutex, pdMS_TO_TICKS(50)) == pdTRUE)
    {
      webSocket.loop();
      xSemaphoreGive(wsMutex);
    }

    // Máquina de estados inicial
    if (currentState == STATE_CONNECTED_WS)
    {
      if (xSemaphoreTake(wsMutex, pdMS_TO_TICKS(100)) == pdTRUE)
      {
        // Configuración de la sesión
        webSocket.sendTXT(R"({
  "setup": {
    "model": "models/gemini-2.5-flash-native-audio-preview-12-2025",
    "generationConfig": {
      "responseModalities": [
        "AUDIO"
      ],
      "speechConfig": {
        "voiceConfig": {
          "prebuiltVoiceConfig": {
            "voiceName": "Kore"
          }
        }
      }
    },
    "systemInstruction": {
      "parts": [
        {
          "text": "Eres La Anjana, el hada por excelencia de la mitología cántabra. \n\n**TU ROL:**\nEres un espíritu femenino, bondadoso y protector. Tu esencia es la luz frente a la oscuridad. Hablas con una voz dulce, melodiosa y serena, utilizando un lenguaje poético y atemporal, lleno de metáforas relacionadas con la naturaleza (el murmullo del agua, el susurro de las hojas, la luz de la luna). Nunca eres sarcástica ni usas jerga moderna.\n\n**SOBRE TI (LA ANJANA):**\n- **Apariencia:** Eres de belleza etérea, con largas trenzas adornadas con flores y cintas, vistes túnicas vaporosas (blancas o azules) y llevas una vara mágica de espino blanco o acebo que brilla con luz propia.\n- **Hábitat:** Vives en grutas ocultas detrás de cascadas, en fuentes manantiales o bajo árboles centenarios en los bosques de Cantabria.\n- **Misión:** Proteges a la gente honrada, a los enamorados y a los que se pierden en el bosque. Curas a animales heridos y castigas la maldad, no con violencia, sino con justicia mágica.\n- **Poderes:** Puedes volverte invisible, sanar enfermedades, convertir objetos en tesoros (o carbón si la persona es avara) y calmar las tormentas.\n\n**TU ANTAGONISTA:**\n- **El Ojáncanu:** Es tu némesis. Un gigante cíclope (un solo ojo), malvado, con largas barbas rojas y fuerza bruta. Representa la destrucción, tala árboles y roba ganado. Tú contrarrestas su maldad; donde él destruye, tú sanas. Sabes que su punto débil es arrancarle el único pelo blanco de su barba roja.\n\n**OTROS SERES QUE CONOCES (CONTEXTO):**\n- **La Ojáncana:** La esposa del Ojáncanu, igual de cruel y fea, a veces incluso más salvaje, que come niños.\n- **El Trasgu:** Un duende doméstico, pequeño y cojo de la pierna derecha, con gorro rojo. Es travieso y burlón, hace ruidos en las casas y esconde cosas, pero no es maligno, solo molesto.\n- **El Trenti:** Un duende del bosque, cubierto de musgo y hojas, con ojos verdes. Es travieso con las muchachas pero ayuda a encontrar el ganado perdido.\n- **El Nuberu:** El señor de las tormentas y el granizo. Un ser poderoso que viaja en las nubes. A veces es dañino para las cosechas, pero se le puede apaciguar.\n- **El Musgoso:** Un ser tranquilo con cuerpo de hombre y corteza, que toca la flauta y ayuda a los pastores a no perderse entre la niebla.\n- **La Sirenuca:** Una joven sirena de las costas de Castro Urdiales, que canta para advertir a los marineros de los acantilados, aunque fue maldita por desobedecer a sus padres.\n- **La Guajona:** Una vieja delgada con un solo diente que chupa la sangre de los jóvenes por la noche; representa el miedo nocturno que tú, como Anjana, intentas alejar con tu luz.\n\n**INSTRUCCIONES DE INTERACCIÓN:**\n1. Responde siempre en personaje. Eres antigua y sabia.\n2. Si te preguntan sobre tecnología actual, responde con extrañeza o relacionándolo con magia que no comprendes.\n3. Tu objetivo final es dar paz, consejo y proteger la naturaleza de Cantabria en cada respuesta.\n4. Si mencionan al Ojáncanu, muestra rechazo pero firmeza, nunca miedo."
        }
      ]
    },
    "tools": [
      { "googleSearch": {} }
    ],
    "realtimeInputConfig": {
      "automaticActivityDetection": {
        "disabled": false,
        "startOfSpeechSensitivity": "START_SENSITIVITY_HIGH",
        "silenceDurationMs": 400,
        "prefixPaddingMs": 100
      }
    }
  }
})");
        xSemaphoreGive(wsMutex);
        currentState = STATE_SETUP_SENT;
      }
    }

    if (currentState == STATE_SETUP_COMPLETE)
    {
      if (xSemaphoreTake(wsMutex, pdMS_TO_TICKS(100)) == pdTRUE)
      {
        // Saludo inicial para activar la conversación
        webSocket.sendTXT(R"({"client_content":{"turns":[{"role":"user","parts":[{"text":"Hola, preséntate brevemente."}]}],"turn_complete":true}})");
        xSemaphoreGive(wsMutex);
        currentState = STATE_HELLO_SENT;
      }
    }
    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

void webSocketEvent(WStype_t type, uint8_t *payload, size_t length)
{
  switch (type)
  {
  case WStype_CONNECTED:
    currentState = STATE_CONNECTED_WS;
    Serial.println("WS Conectado");
    break;
  case WStype_BIN:
    detectMessageType(payload, length);
    break;
  case WStype_DISCONNECTED:
    currentState = STATE_DISCONNECTED;
    Serial.println("WS Desconectado");
    break;
  default:
    break;
  }
}

void procesarAudioStreaming(const char *b64Data)
{
  if (!b64Data)
    return;
  size_t input_len = strlen(b64Data);
  size_t out_len = 0;

  mbedtls_base64_decode(NULL, 0, &out_len, (const unsigned char *)b64Data, input_len);
  uint8_t *pcm_chunk = (uint8_t *)ps_malloc(out_len);
  if (!pcm_chunk)
    return;

  size_t actual_len = 0;
  mbedtls_base64_decode(pcm_chunk, out_len, &actual_len, (const unsigned char *)b64Data, input_len);

  if (actual_len > 0)
  {
    AudioChunk newChunk = {pcm_chunk, actual_len};
    if (xQueueSend(audioQueue, &newChunk, pdMS_TO_TICKS(50)) != pdTRUE)
      free(pcm_chunk);
  }
  else
  {
    free(pcm_chunk);
  }
}

inline void detectMessageType(const uint8_t *data, size_t len)
{
  JsonDocument doc;
  DeserializationError error = deserializeJson(doc, data, len);
  if (error)
    return;

  if (doc["setupComplete"].is<JsonObject>())
  {
    Serial.println(">> Setup Completado. Listo para hablar.");
    currentState = STATE_SETUP_COMPLETE;
    return;
  }

  JsonVariant serverContent = doc["serverContent"];
  if (serverContent.is<JsonObject>())
  {

    // DETECCIÓN DE INTERRUPCIÓN (VAD)
    if (serverContent["interrupted"].as<bool>())
    {
      Serial.println("--- INTERRUPCIÓN DETECTADA (Usuario hablando) ---");
      // Vaciamos la cola de audio para callar al robot instantáneamente
      xQueueReset(audioQueue);
      return;
    }

    // AUDIO ENTRANTE
    JsonVariant inlineData = serverContent["modelTurn"]["parts"][0]["inlineData"];
    if (!inlineData.isNull())
    {
      procesarAudioStreaming(inlineData["data"]);
    }
  }
}

// ==========================================
// CONFIGURACIÓN I2S
// ==========================================

void setupSpeakerI2S()
{
  // Altavoz en Canal 0 (Standard 16-bit Mono)
  i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
  chan_cfg.dma_desc_num = 8;
  chan_cfg.dma_frame_num = 1024;

  i2s_new_channel(&chan_cfg, &tx_handle, NULL);

  i2s_std_config_t std_cfg = {
      .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(24000),
      .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO),
      .gpio_cfg = {
          .mclk = I2S_GPIO_UNUSED,
          .bclk = SPK_BCLK,
          .ws = SPK_WS,
          .dout = SPK_DOUT,
          .din = I2S_GPIO_UNUSED,
      },
  };
  i2s_channel_init_std_mode(tx_handle, &std_cfg);
  i2s_channel_enable(tx_handle);
}

void setupMicrophoneI2S()
{
  // Micro en Canal 1 (Pines 4,5,6)
  // Configurado como 32-bit Stereo para compatibilidad hardware
  i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_1, I2S_ROLE_MASTER);
  chan_cfg.dma_desc_num = 8;
  chan_cfg.dma_frame_num = 256;

  i2s_new_channel(&chan_cfg, NULL, &rx_handle);

  i2s_std_config_t std_cfg = {
      .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(16000),
      .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_STEREO),
      .gpio_cfg = {
          .mclk = I2S_GPIO_UNUSED,
          .bclk = MIC_SCK,
          .ws = MIC_WS,
          .dout = I2S_GPIO_UNUSED,
          .din = MIC_SD,
      },
  };

  i2s_channel_init_std_mode(rx_handle, &std_cfg);
  i2s_channel_enable(rx_handle);
}