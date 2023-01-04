/* Hello World Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include <string.h>
#include "esp_log.h"
#include "cJSON.h"
#include <string>
#include "driver/gpio.h"
#include "wetterstation-lora.h"
#include "cbor_tools.h"
#include "tools.h"
#ifdef USEDISPLAY
extern "C"
{
#include "u8g2_esp32_hal.h"
}
#endif

#define SENDING
//#define USEDISPLAY

static const char *TAG = "WS LORA";

static const char *TEMP_TAG = "TE";
static const char *HUM_TAG = "LF";
static const char *PRESS_TAG = "LD";
static const char *ILLU_TAG = "BS";
static const char *COOL_TAG = "LU";
static const char *POS_TAG = "Ort";
static const char *VAL_TAG = "W";
// Paket-Zähler
static const char *PC_TAG = "PC";

static const char *ORT_SCHUPPEN_SCHATTEN = "A";
static const char *ORT_SCHUPPEN_SONNE = "B";
static const char *ORT_SCHUPPEN_INNEN = "C";
static const char *ORT_CARPORT = "D";
static const char *ORT_GARTENHAUS = "E";
static const char *ORT_GEWAECHSHAUS = "F";

#define SENDER_ADDRESS 155
#define RECEIVER_ADDRESS 156

#define LED_PIN GPIO_NUM_25

// Display-SDA - GPIO4
#define PIN_SDA GPIO_NUM_4

// Display-SCL - GPIO15
#define PIN_SCL GPIO_NUM_15

// Display-Reset GPIO16
#define PIN_DISP_RESET GPIO_NUM_16

extern "C"
{
  void app_main()
  {
    gpio_pad_select_gpio(LED_PIN);
    gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);
#ifdef USEDISPLAY
    gpio_pad_select_gpio(PIN_DISP_RESET);
    gpio_set_direction(PIN_DISP_RESET, GPIO_MODE_OUTPUT);
    InitSSD1306();
#endif
    xTaskCreate(&task_tx, "task_tx", 4096, NULL, 5, NULL);
    for (;;)
    {
      vTaskDelay(pdMS_TO_TICKS(5000));
    }
  }
}
#ifdef USEDISPLAY
static u8g2_t u8g2; // a structure which will contain all the data for one display

void InitSSD1306()
{
  u8g2_esp32_hal_t u8g2_esp32_hal = U8G2_ESP32_HAL_DEFAULT;
  u8g2_esp32_hal.sda = PIN_SDA;
  u8g2_esp32_hal.scl = PIN_SCL;
  u8g2_esp32_hal_init(u8g2_esp32_hal);

  gpio_set_level(PIN_DISP_RESET, 0);
  vTaskDelay(pdMS_TO_TICKS(50));
  gpio_set_level(PIN_DISP_RESET, 1);

  u8g2_Setup_ssd1306_i2c_128x64_noname_f(
      &u8g2,
      U8G2_R0,
      u8g2_esp32_i2c_byte_cb,
      u8g2_esp32_gpio_and_delay_cb); // init u8g2 structure
  u8x8_SetI2CAddress(&u8g2.u8x8, 0x78);

  //ESP_LOGI(TAG, "u8g2_InitDisplay");
  u8g2_InitDisplay(&u8g2); // send init sequence to the display, display is in sleep mode after this,

  //ESP_LOGI(TAG, "u8g2_SetPowerSave");
  u8g2_SetPowerSave(&u8g2, 0); // wake up display
  //ESP_LOGI(TAG, "u8g2_ClearBuffer");
  u8g2_ClearBuffer(&u8g2);
  u8g2_SendBuffer(&u8g2);

  /*
  ESP_LOGI(TAG, "u8g2_DrawBox");
  u8g2_DrawBox(&u8g2, 0, 26, 80, 6);
  u8g2_DrawFrame(&u8g2, 0, 26, 100, 6);

  ESP_LOGI(TAG, "u8g2_SetFont");
  u8g2_SetFont(&u8g2, u8g2_font_ncenB14_tr);
  ESP_LOGI(TAG, "u8g2_DrawStr");
  u8g2_DrawStr(&u8g2, 2, 17, "Hi nkolban!");
  ESP_LOGI(TAG, "u8g2_SendBuffer");
  u8g2_SendBuffer(&u8g2);
*/
  ESP_LOGI(TAG, "SSD 1306 display initialized!");
}
#endif

#define MAX_VPBUFLEN 256
char vprintf_buffer[MAX_VPBUFLEN];

void error(const char *format, ...)
{
  va_list myargs;
  va_start(myargs, format);

  vsnprintf(vprintf_buffer, MAX_VPBUFLEN, format, myargs);
  ESP_LOGE(TAG, "%s", vprintf_buffer);
  va_end(myargs);
  int toggle = 0;
  for (;;)
  {
    toggle = 1 - toggle;
    gpio_set_level(LED_PIN, toggle);
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}



uint8_t lora_buf[256];

esp_err_t SendLoraMsg(SX1278_LoRa &aLoRa, LoraCommand aCmd, uint8_t *aBuf, uint16_t aSize, uint32_t aTag)
{
  gpio_set_level(LED_PIN, 1);
  uint8_t MaxPayloadPerPaketSize = 255 - sizeof(LoraPacketHeader);
  uint8_t NumPackets = aSize / MaxPayloadPerPaketSize + 1;
  uint8_t LastPacketSize = aSize - (NumPackets - 1) * MaxPayloadPerPaketSize;
  LoraPacketHeader ph;
  ph.Address = SENDER_ADDRESS;
  ph.Cmd = (uint8_t)aCmd;
  ph.NumPackets = NumPackets;
  ph.Tag=aTag;
  int BytesWritten = 0;
  esp_err_t ret;
  for (int i = 0; i < NumPackets; i++)
  {
    ph.PacketNumber = i;
    ph.PacketPayloadSize = i == (NumPackets - 1) ? LastPacketSize : MaxPayloadPerPaketSize;
    ph.TotalTransmissionSize = aSize;
    ph.PayloadCRC = compute_crc(aBuf + BytesWritten, ph.PacketPayloadSize);
    memcpy(lora_buf, &ph, sizeof(ph)); // Header kopieren
    memcpy(lora_buf + sizeof(ph), aBuf + BytesWritten, ph.PacketPayloadSize);
    ESP_LOGI(TAG, "Sending LoRa packet %d/%d, %d bytes", i + 1, NumPackets, sizeof(ph) + ph.PacketPayloadSize);
    ret = aLoRa.lora_send_packet(lora_buf, sizeof(ph) + ph.PacketPayloadSize);
    if (ret != ESP_OK)
    {
      gpio_set_level(LED_PIN, 0);
      return ret;
    }
    BytesWritten += ph.PacketPayloadSize;
  }
  gpio_set_level(LED_PIN, 0);
  return ESP_OK;
}

int64_t GetTime_us()
{
  struct timeval tv_now;
  gettimeofday(&tv_now, NULL);
  return (int64_t)tv_now.tv_sec * 1000000L + (int64_t)tv_now.tv_usec;
}

void task_tx(void *p)
{
  SX1278_LoRa LoRa;
  esp_err_t ret = LoRa.SetupModule();
  if (ret != ESP_OK)
    error(TAG, "Fehler beim Initialisieren des LoRa Moduls: %d", ret);

#ifdef SENDING

  uint8_t cbor_buf[255];

  int c = 0;
  //cJSON *tmp = NULL;
  double temp1 = 23.45;
  double temp2 = 31.86;
  double hum1 = 54.67;
  double hum2 = 43.21;
  double bright = 54680;
  double cool = 513;
  double press = 1006.8;

  #ifdef USEDISPLAY
  u8g2_SetFont(&u8g2, u8g2_font_lucasarts_scumm_subtitle_o_tf);
  char DisplayBuf[128];
  #endif
  for (;;)
  {
    // Achtung: In einer Map müssesn stets zwei Einträge paarweise stehen, sonst
    // schlägt der Encoder fehl!
    CborEncoder encoder, me0, me1, arr;
    cbor_encoder_init(&encoder, cbor_buf, sizeof(cbor_buf), 0);
    //cbor_encode_text_stringz(&encoder, "WS");

    // Paketzähler
    cbor_encoder_create_map(&encoder, &me0, CborIndefiniteLength);
    cbor_encode_text_stringz(&me0, PC_TAG);
    cbor_encode_int(&me0, c);

    // Temperaturen
    cbor_encode_text_stringz(&me0, TEMP_TAG);
    cbor_encoder_create_array(&me0, &arr, CborIndefiniteLength);

    cbor_encoder_create_map(&arr, &me1, CborIndefiniteLength);
    cbor_encode_text_stringz(&me1, POS_TAG);
    cbor_encode_text_stringz(&me1, ORT_SCHUPPEN_SCHATTEN);
    cbor_encode_text_stringz(&me1, VAL_TAG);
    cbor_encode_float(&me1, temp1);
    cbor_encoder_close_container(&arr, &me1);

    cbor_encoder_create_map(&arr, &me1, CborIndefiniteLength);
    cbor_encode_text_stringz(&me1, POS_TAG);
    cbor_encode_text_stringz(&me1, ORT_SCHUPPEN_SONNE);
    cbor_encode_text_stringz(&me1, VAL_TAG);
    cbor_encode_float(&me1, temp2);
    cbor_encoder_close_container(&arr, &me1);

    cbor_encoder_close_container(&me0, &arr);

    // Luftfeuchtigkeit
    cbor_encode_text_stringz(&me0, HUM_TAG);
    cbor_encoder_create_array(&me0, &arr, CborIndefiniteLength);

    cbor_encoder_create_map(&arr, &me1, CborIndefiniteLength);
    cbor_encode_text_stringz(&me1, POS_TAG);
    cbor_encode_text_stringz(&me1, ORT_SCHUPPEN_SCHATTEN);
    cbor_encode_text_stringz(&me1, VAL_TAG);
    cbor_encode_float(&me1, hum1);
    cbor_encoder_close_container(&arr, &me1);

    cbor_encoder_create_map(&arr, &me1, CborIndefiniteLength);
    cbor_encode_text_stringz(&me1, POS_TAG);
    cbor_encode_text_stringz(&me1, ORT_SCHUPPEN_SONNE);
    cbor_encode_text_stringz(&me1, VAL_TAG);
    cbor_encode_float(&me1, hum2);
    cbor_encoder_close_container(&arr, &me1);

    cbor_encoder_close_container(&me0, &arr);

    // Luftdruck
    cbor_encode_text_stringz(&me0, PRESS_TAG);
    cbor_encoder_create_map(&me0, &me1, CborIndefiniteLength);
    cbor_encode_text_stringz(&me1, POS_TAG);
    cbor_encode_text_stringz(&me1, ORT_SCHUPPEN_INNEN);
    cbor_encode_text_stringz(&me1, VAL_TAG);
    cbor_encode_float(&me1, press);
    cbor_encoder_close_container(&me0, &me1);

    // Beleuchtungsstaerke
    cbor_encode_text_stringz(&me0, ILLU_TAG);
    cbor_encoder_create_map(&me0, &me1, CborIndefiniteLength);
    cbor_encode_text_stringz(&me1, POS_TAG);
    cbor_encode_text_stringz(&me1, ORT_SCHUPPEN_SONNE);
    cbor_encode_text_stringz(&me1, VAL_TAG);
    cbor_encode_float(&me1, bright);
    cbor_encoder_close_container(&me0, &me1);

    // Lüftergeschwindigkeit
    cbor_encode_text_stringz(&me0, COOL_TAG);
    cbor_encoder_create_map(&me0, &me1, CborIndefiniteLength);
    cbor_encode_text_stringz(&me1, POS_TAG);
    cbor_encode_text_stringz(&me1, ORT_SCHUPPEN_SONNE);
    cbor_encode_text_stringz(&me1, VAL_TAG);
    cbor_encode_float(&me1, cool);
    cbor_encoder_close_container(&me0, &me1);

    cbor_encoder_close_container(&encoder, &me0);

    int len = cbor_encoder_get_buffer_size(&encoder, cbor_buf);
    //ESP_LOGI(TAG, "CBOR erstellt, Groesse: %d", len);
    //HexDump(cbor_buf, len);
    int RetryCounter=0;
    bool SendOK=true;
    do
    {
      /* code */
      int64_t ts = GetTime_us();
      ret = SendLoraMsg(LoRa, CMD_CBORDATA, cbor_buf, len, c);
      int64_t te = GetTime_us();
      //ESP_LOGI(TAG, "Zeit fuer LoRa: %.1f ms", double(te - ts) / 1000.0);
      if (ret != ESP_OK)
      {
        SendOK=false;
        RetryCounter++;
        ESP_LOGE(TAG, "Fehler beim Senden eines LoRa-Paketes: %d Versuch: %d", ret, RetryCounter);  
        ESP_LOGE(TAG, "Resette LORA-Modul...");  
        LoRa.lora_reset();  
        vTaskDelay(pdMS_TO_TICKS(100));  
        ret = LoRa.SetupModule();
        if (ret != ESP_OK)
          error(TAG, "Fehler beim Initialisieren des LoRa Moduls: %d", ret);
        else
          ESP_LOGI(TAG, "Re-Init LORA erfolgreich");
      }
      else
        ESP_LOGI(TAG, "LORA Paket Nummer: %d erfolgreich versendet.", c);
    } 
    while (!SendOK);
    
    
#ifdef USEDISPLAY
    u8g2_ClearBuffer(&u8g2);
    sprintf(DisplayBuf, "Paket: %d", c);
    u8g2_DrawStr(&u8g2, 2, 17, DisplayBuf);
    sprintf(DisplayBuf, "Zeit: %.1f ms", double(te - ts) / 1000.0);
    u8g2_DrawStr(&u8g2, 2, 34, DisplayBuf);
    u8g2_SendBuffer(&u8g2);
#endif
    vTaskDelay(pdMS_TO_TICKS(5000));
    c++;
  }
#else
  uint8_t BytesRead;
  uint8_t buf[255];
  ESP_LOGI(TAG, "LoRa-Lesethread startet jetzt...");
  for (;;)
  {
    lora_receive(); // put into receive mode
    while (lora_received())
    {
      gpio_set_level(LED_PIN, 1);
      ret = lora_receive_packet(buf, (uint8_t)sizeof(buf), &BytesRead);
      ESP_LOGI(TAG, "Gelesen: %d bytes", BytesRead);
      gpio_set_level(LED_PIN, 0);
      if (ret != ESP_OK)
      {
        ESP_LOGE(TAG, "Fehler beim Lesen eines LoRa-Paketes: %d", ret);
        lora_receive();
        continue;
      }

      ParseLoraPacket(buf, BytesRead);
      lora_receive();
    }
    vTaskDelay(1);
  }
#endif
}

void ParseLoraPacket(uint8_t *buf, uint8_t len)
{
  if (len < sizeof(LoraPacketHeader))
  {
    ESP_LOGE(TAG, "Length of packet shorter than packet header. Aborting");
    return;
  }

  CborParser root_parser;
  CborValue it;

  LoraPacketHeader *ph = (LoraPacketHeader *)buf;
  if (ph->Magic != PACKET_MAGIC)
  {
    ESP_LOGE(TAG, "Wrong package header. Header Magic: 0x%x, correct Magic: 0x%x", ph->Magic, PACKET_MAGIC);
  }
  uint8_t PacketSize = ph->PacketPayloadSize;
  uint8_t PacketOffset = sizeof(LoraPacketHeader);
  ESP_LOGI(TAG, "Packet payload size: %d", ph->PacketPayloadSize);
  uint16_t PayloadCRC = compute_crc(buf + PacketOffset, PacketSize);
  if (PayloadCRC != ph->PayloadCRC)
  {
    ESP_LOGE(TAG, "CRC-Error. CRC package header: 0x%x, calculated: 0x%x", ph->PayloadCRC, PayloadCRC);
    return;
  }
  else
    ESP_LOGI(TAG, "Package CRC OK!");

  // Initialize the cbor parser and the value iterator
  cbor_parser_init(buf + PacketOffset, PacketSize, 0, &root_parser, &it);

  ESP_LOGI(TAG, "convert CBOR to JSON");
  // Dump the values in JSON format
  cbor_value_to_json(stdout, &it, 0);
  puts("");

  //ESP_LOGI(TAG, "decode CBOR manually");
  // Decode CBOR data manully
  //example_dump_cbor_buffer(&it, 0);
}
