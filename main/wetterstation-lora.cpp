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

static const char *TAG = "WS LORA";

static const char *TEMP_TAG = "TE";
static const char *HUM_TAG = "LF";
static const char *PRESS_TAG = "LD";
static const char *ILLU_TAG = "BS";
static const char *COOL_TAG = "LU";
static const char *POS_TAG = "Ort";
static const char *VAL_TAG = "W";

static const char *ORT_SCHUPPEN_SCHATTEN = "A";
static const char *ORT_SCHUPPEN_SONNE = "B";
static const char *ORT_SCHUPPEN_INNEN = "C";

#define SENDER_ADDRESS 155
#define RECEIVER_ADDRESS 156

#define LED_PIN GPIO_NUM_25

extern "C"
{
  void app_main()
  {
    gpio_pad_select_gpio(LED_PIN);
    gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);
    InitLora();
    xTaskCreate(&task_tx, "task_tx", 4096, NULL, 5, NULL);
    for (;;)
    {
      vTaskDelay(pdMS_TO_TICKS(5000));
    }
  }
}

void InitLora()
{
  esp_err_t ret;
  ret = lora_init();
  if (ret != ESP_OK)
    error("lora_init failed: %d", ret);
  ret = lora_explicit_header_mode();
  if (ret != ESP_OK)
    error("lora_explicit_header_mode failed: %d", ret);
  ret = lora_set_frequency(433e6);//lora_set_frequency(434.54e6);
  if (ret != ESP_OK)
    error("lora_set_frequency failed: %d", ret);
  ret = lora_enable_crc();
  if (ret != ESP_OK)
    error("lora_enable_crc failed: %d", ret);
  
  ret = lora_set_preamble_length(14);
  if (ret != ESP_OK)
    error("lora_set_preamble_length failed: %d", ret);
  ret = lora_set_bandwidth(500E3);
  if (ret != ESP_OK)
    error("lora_set_bandwidth failed: %d", ret);
  ret = lora_set_sync_word(0x3d);
  if (ret != ESP_OK)
    error("lora_set_sync_word failed: %d", ret);
  ESP_LOGI(TAG, "Init LoRa OK");
}

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

#define SENDING

uint8_t lora_buf[256];

esp_err_t SendLoraMsg(LoraCommand aCmd, uint8_t *aBuf, uint16_t aSize)
{
  gpio_set_level(LED_PIN, 1);
  uint8_t MaxPayloadPerPaketSize = 255 - sizeof(LoraPacketHeader);
  uint8_t NumPackets = aSize / MaxPayloadPerPaketSize + 1;
  uint8_t LastPacketSize = aSize - (NumPackets - 1) * MaxPayloadPerPaketSize;
  LoraPacketHeader ph;
  ph.Address = SENDER_ADDRESS;
  ph.Cmd = (uint8_t)aCmd;
  ph.NumPackets = NumPackets;
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
    ret = lora_send_packet(lora_buf, sizeof(ph) + ph.PacketPayloadSize);
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

void task_tx(void *p)
{
#ifdef SENDING

  uint8_t cbor_buf[255];

  int c = 0;
  cJSON *tmp = NULL;
  double temp1 = 23.45;
  double temp2 = 31.86;
  double hum1 = 54.67;
  double hum2 = 43.21;
  double bright = 54680;
  double cool = 513;
  double press = 1006.8;

  for (;;)
  {
    // Achtung: In einer Map müssesn stets zwei Einträge paarweise stehen, sonst
    // schlägt der Encoder fehl!
    CborEncoder encoder, me0, me1, arr;
    cbor_encoder_init(&encoder, cbor_buf, sizeof(cbor_buf), 0);
    //cbor_encode_text_stringz(&encoder, "WS");

    cbor_encoder_create_map(&encoder, &me0, CborIndefiniteLength);
    cbor_encode_text_stringz(&me0, "TC");
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
    ESP_LOGI(TAG, "CBOR erstellt, Groesse: %d", len);
    //HexDump(cbor_buf, len);
    esp_err_t ret = SendLoraMsg(CMD_CBORDATA, cbor_buf, len);
    if (ret != ESP_OK)
      ESP_LOGE(TAG, "Fehler beim Senden eines LoRa-Paketes: %d", ret);
    vTaskDelay(pdMS_TO_TICKS(5000));

    /*
    cJSON *root;
    root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "TAG", "WS");
    cJSON_AddNumberToObject(root, "TC", c);

    cJSON *Temperatur = cJSON_CreateObject();
    cJSON *Luftfeuchtigkeit = cJSON_CreateObject();
    cJSON *Luftdruck = cJSON_CreateObject();
    cJSON *Beleuchtungsstaerke = cJSON_CreateObject();
    cJSON *Luefter = cJSON_CreateObject();

    cJSON_AddItemToObject(root, TEMP_TAG, Temperatur);
    cJSON_AddItemToObject(root, HUM_TAG, Luftfeuchtigkeit);
    cJSON_AddItemToObject(root, PRESS_TAG, Luftdruck);
    cJSON_AddItemToObject(root, ILLU_TAG, Beleuchtungsstaerke);
    cJSON_AddItemToObject(root, COOL_TAG, Luefter);

    // Temperaturen:
    cJSON_AddItemToObject(Temperatur, "T", tmp = cJSON_CreateObject());
    cJSON_AddStringToObject(tmp, "ORT", ORT_SCHUPPEN_SCHATTEN);
    cJSON_AddNumberToObject(tmp, "Wert", 23.6);
    cJSON_AddItemToObject(Temperatur, "T", tmp = cJSON_CreateObject());
    cJSON_AddStringToObject(tmp, "ORT", ORT_SCHUPPEN_SONNE);
    cJSON_AddNumberToObject(tmp, "Wert", 25.3);

    // Luftfeuchtigkeit:
    cJSON_AddItemToObject(Luftfeuchtigkeit, "H", tmp = cJSON_CreateObject());
    cJSON_AddStringToObject(tmp, "ORT", ORT_SCHUPPEN_SCHATTEN);
    cJSON_AddNumberToObject(tmp, "Wert", 54.9);
    cJSON_AddItemToObject(Luftfeuchtigkeit, "H", tmp = cJSON_CreateObject());
    cJSON_AddStringToObject(tmp, "ORT", ORT_SCHUPPEN_SONNE);
    cJSON_AddNumberToObject(tmp, "Wert", 45.2);

    // Luftdruck:
    cJSON_AddItemToObject(Luftfeuchtigkeit, "P", tmp = cJSON_CreateObject());
    cJSON_AddStringToObject(tmp, "ORT", ORT_SCHUPPEN_INNEN);
    cJSON_AddNumberToObject(tmp, "Wert", 1005.5);

    // Beleuchtungsstaerke
    cJSON_AddItemToObject(Beleuchtungsstaerke, "L", tmp = cJSON_CreateObject());
    cJSON_AddStringToObject(tmp, "ORT", ORT_SCHUPPEN_SONNE);
    cJSON_AddNumberToObject(tmp, "Wert", 56210);

    // Luefter
    cJSON_AddItemToObject(Luefter, "C", tmp = cJSON_CreateObject());
    cJSON_AddStringToObject(tmp, "ORT", ORT_SCHUPPEN_SONNE);
    cJSON_AddNumberToObject(tmp, "Wert", 512);

    char *my_json_string = cJSON_PrintUnformatted(root);

    ESP_LOGI(TAG, "my_json_string: %d \n%s", strlen(my_json_string), my_json_string);

    if (strlen(my_json_string) < 256)
    {
      lora_send_packet((uint8_t *)my_json_string, (uint8_t)strlen(my_json_string));
    }
    else
      ESP_LOGE(TAG, "my_json_string zu gross: %d ", strlen(my_json_string));

    cJSON_free(my_json_string);
    cJSON_Delete(root);
    */

    //vTaskDelay(pdMS_TO_TICKS(20000));
    c++;
  }
#else
  uint8_t BytesRead;
  uint8_t buf[255];
  esp_err_t ret;
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
