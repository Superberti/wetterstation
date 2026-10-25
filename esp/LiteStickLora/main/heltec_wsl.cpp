/* Wi-Fi Provisioning Manager Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <stdio.h>
#include <string.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/event_groups.h>

#include <esp_log.h>
#include <esp_wifi.h>
#include <esp_event.h>
#include <nvs_flash.h>

#include <network_provisioning/manager.h>
#include "esp_mac.h" // Wichtig für MACSTR, MAC2STR und esp_read_mac
#include <network_provisioning/scheme_ble.h>
#include "qrcode.h"
#include "driver/gpio.h"
#include "esp_check.h"
#include "esp_timer.h"
#include "lora/lorastructs.h"
#include "lora/lora.h"
#include "heltec_wsl.h"
#include "cbor.h"
#include "driver/rmt_tx.h"
#include "driver/spi_master.h"
// RadioLib einbinden
#include <RadioLib.h>

/* Signal Wi-Fi events on this event-group */
const int WIFI_CONNECTED_EVENT = BIT0;
static EventGroupHandle_t wifi_event_group;
static const char *TAG = "app";

// Inputs
#define USER_BUTTON GPIO_NUM_0
#define WHITE_LED GPIO_NUM_35

// SX1262 LoRa-Modul (SPI)
#define LORA_PIN_MISO GPIO_NUM_11
#define LORA_PIN_MOSI GPIO_NUM_10
#define LORA_PIN_CLK GPIO_NUM_9
#define LORA_PIN_CS GPIO_NUM_8
#define LORA_PIN_RESET GPIO_NUM_12
#define LORA_PIN_BUSY GPIO_NUM_13
#define LORA_PIN_DIO1 GPIO_NUM_14

// CC1101-Funkmodul (für Fielmann-Wetterstation)
#define CC_PIN_CSN GPIO_NUM_2
#define CC_PIN_SCK GPIO_NUM_18
#define CC_PIN_MISO GPIO_NUM_19
#define CC_PIN_MOSI GPIO_NUM_21
#define CC_PIN_GDO0 GPIO_NUM_3
#define PIN_RST RADIOLIB_NC
#define CC_PIN_GDO2 GPIO_NUM_1

extern "C"
{
  void app_main(void)
  {
    /* Initialize NVS partition */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
      /* NVS partition was truncated
       * and needs to be erased */
      ESP_ERROR_CHECK(nvs_flash_erase());

      /* Retry nvs_flash_init */
      ESP_ERROR_CHECK(nvs_flash_init());
    }

    HeltecStick wsl;
    wsl.Run();
  }
}

esp_err_t HeltecStick::Run()
{
  BaseType_t task_ret = xTaskCreate(LoRaTask, "LoRaTask", 4096, this, 10, &LoRaTaskHandle);
  if (task_ret != pdPASS)
  {
    ESP_LOGE(TAG, "Failed to create LoRaTask");
    // return ESP_FAIL;
  }

  task_ret = xTaskCreate(WifiTask, "WifiTask", 4096, this, 10, &WifiTaskHandle);
  if (task_ret != pdPASS)
  {
    ESP_LOGE(TAG, "Failed to create WifiTask");
    // return ESP_FAIL;
  }

  task_ret = xTaskCreate(CCTask, "CCTask", 4096, this, 10, &CCTaskHandle);
  if (task_ret != pdPASS)
  {
    ESP_LOGE(TAG, "Failed to create CCTask");
    // return ESP_FAIL;
  }

  InitGPIO();
  /* Start main application now */

  gpio_set_level(WHITE_LED, 0);

  int64_t StartTime_ms = GetTime_ms();
  // Abfrage User-Button. Ist der 5 Sekunden lang gedrückt, dann werden die WLAN-Einstellungen
  // vergessen und man kann das Gerät mit der BLE-Provisioning App von ESP wieder mit einem neuen WLAN verbinden
  while (1)
  {
    int key = gpio_get_level(USER_BUTTON);
    gpio_set_level(WHITE_LED, !key);
    // ESP_LOGI(TAG, "Key: %d",key);
    if (key == 0)
    {
      if (GetTime_ms() - StartTime_ms > 5000)
      {
        ESP_LOGI(TAG, "Setze WiFi zurück und reboote!");
        /* Netzwerkeinstellungen löschen */
        network_prov_mgr_reset_wifi_provisioning();
        for (int i = 0; i < 20; i++)
        {
          gpio_set_level(WHITE_LED, i % 2);
          vTaskDelay(100 / portTICK_PERIOD_MS);
        }
        gpio_set_level(WHITE_LED, 0);
        esp_restart();
      }
    }
    else
      StartTime_ms = GetTime_ms();

    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

spi_device_handle_t spi_handle;
rmt_channel_handle_t tx_channel = NULL;
rmt_encoder_handle_t copy_encoder = NULL;

// Korrektes Schreiben einzelner Register im ESP-IDF
void cc1101_write_reg(uint8_t reg, uint8_t val)
{
  spi_transaction_t t;
  memset(&t, 0, sizeof(t));
  t.flags = SPI_TRANS_USE_TXDATA;
  t.length = 16; // 16 Bit Gesamtübertragung (8 Bit Adresse + 8 Bit Daten)

  // Adresse (Bits 7=0 für Write, 6=0 für Single) in das erste Byte packen
  t.tx_data[0] = reg & 0x3F;
  t.tx_data[1] = val; // Der eigentliche Registerwert

  spi_device_transmit(spi_handle, &t);
}

// Senden von Befehls-Strobes (z. B. SRES, STX)
void cc1101_send_strobe(uint8_t strobe)
{
  spi_transaction_t t;
  memset(&t, 0, sizeof(t));
  t.flags = SPI_TRANS_USE_TXDATA;
  t.length = 8; // Ein Strobe ist genau 8 Bit lang
  t.tx_data[0] = strobe;

  spi_device_transmit(spi_handle, &t);
}

// Initialisiert das CC1101-Modul direkt über den ESP-IDF SPI-Treiber
void init_cc1101_hardware_direct()
{
  // 1. SPI-Bus auf den Heltec-Universal-GPIOs starten
  spi_bus_config_t buscfg = {};
  buscfg.mosi_io_num = CC_PIN_MOSI;
  buscfg.miso_io_num = CC_PIN_MISO;
  buscfg.sclk_io_num = CC_PIN_SCK;
  buscfg.quadwp_io_num = -1;
  buscfg.quadhd_io_num = -1;
  buscfg.max_transfer_sz = 0;
  ESP_ERROR_CHECK(spi_bus_initialize(SPI3_HOST, &buscfg, SPI_DMA_CH_AUTO));

  // 2. SPI-Gerät mit moderaten 1 MHz hinzufügen (extrem stabil)
  spi_device_interface_config_t devcfg = {};
  devcfg.mode = 0;
  devcfg.clock_speed_hz = 1000000;
  devcfg.spics_io_num = CC_PIN_CSN;
  devcfg.queue_size = 7;
  ESP_ERROR_CHECK(spi_bus_add_device(SPI3_HOST, &devcfg, &spi_handle));

  // 1. Reset ausführen (Zwingend erforderlich!)
  cc1101_send_strobe(0x30); // SRES
  vTaskDelay(pdMS_TO_TICKS(20));

  // 2. WICHTIG: Erst in den IDLE-Modus versetzen, bevor Register geschrieben werden
  cc1101_send_strobe(0x36); // SIDLE

  // 3. REGISTER BEFÜLLEN (Mit funktionierender SPI-Funktion)
  cc1101_write_reg(0x02, 0x0D); // IOCFG0: GDO0 als asynchronen Dateneingang definieren
  cc1101_write_reg(0x08, 0x30); // PKTCTRL0: Asynchroner Serial Mode (FIFO aus)

  // Automatische Kalibrierung beim Wechsel zu TX aktivieren!
  cc1101_write_reg(0x18, 0x18); // MCSM0: Auto-Kalibrierung einschalten

  // Frequenzeinstellung 433.92 MHz
  cc1101_write_reg(0x0D, 0x10); // FREQ2
  cc1101_write_reg(0x0E, 0xB1); // FREQ1
  cc1101_write_reg(0x0F, 0x3B); // FREQ0

  // OOK Modulation und Endstufe scharf schalten
  cc1101_write_reg(0x12, 0x30); // MDMCFG2: ASK/OOK Modulation
  cc1101_write_reg(0x22, 0x11); // FREND0: OOK-Leistungsstufe ein

  // 4. BUNDESWEITE SYNCHRONISIERUNG AUSLÖSEN (STX)
  cc1101_send_strobe(0x35); // STX Befehl absetzen -> Wechselt in TX und kalibriert

  ESP_LOGI(TAG, "CC1101 erfolgreich kalibriert und Modulator für GPIO 3 geöffnet!");
}

// Initialisiert den ESP32-S3 Hardware RMT-Treiber
void init_rmt_hardware_driver()
{
  rmt_tx_channel_config_t tx_chan_config = {};
  tx_chan_config.gpio_num = CC_PIN_GDO0;
  tx_chan_config.clk_src = RMT_CLK_SRC_DEFAULT;
  tx_chan_config.resolution_hz = 1000000; // 1 MHz = 1 Tick pro Mikrosekunde
  tx_chan_config.mem_block_symbols = 128; // Genug Speicher für das v1.0 Signal
  tx_chan_config.trans_queue_depth = 4;
  tx_chan_config.flags.invert_out = false;
  tx_chan_config.flags.with_dma = false;
  ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_chan_config, &tx_channel));

  rmt_copy_encoder_config_t copy_encoder_cfg = {};
  ESP_ERROR_CHECK(rmt_new_copy_encoder(&copy_encoder_cfg, &copy_encoder));
  ESP_ERROR_CHECK(rmt_enable(tx_channel));
}

#define T_PREAMBLE_SPACE_US 3000
#define T_SYNC_US 5500
#define T_BIT_US 1450

// Berechnet die originale Oregon v1 Nibble-CRC laut Erriez-Spezifikation
uint8_t calculate_thn128_crc(uint8_t b0, uint8_t b1, uint8_t b2)
{
  // Das Protokoll summiert alle 6 Daten-Nibbles einzeln auf
  uint8_t nibble_sum = 0;

  nibble_sum += (b0 & 0x0F);        // Nibble 0: Rolling Address
  nibble_sum += ((b0 >> 4) & 0x0F); // Nibble 1: Channel & Leerbits
  nibble_sum += (b1 & 0x0F);        // Nibble 2: TH3 (1/10 Grad)
  nibble_sum += ((b1 >> 4) & 0x0F); // Nibble 3: TH2 (1 Grad)
  nibble_sum += (b2 & 0x0F);        // Nibble 4: TH1 (10 Grad)
  nibble_sum += ((b2 >> 4) & 0x0F); // Nibble 5: Sign-Bit & Flags

  // Die finale CRC ist die invertierte Summe plus 1 (Zweierkomplement-Logik)
  return (uint8_t)(nibble_sum);
}

void send_fielmann_v1_packet(uint8_t channel, uint8_t rollingCode, float temperature)
{
  rmt_symbol_word_t raw_symbols[200]; // Als festes Array deklariert für ausreichend Speicher
  memset(raw_symbols, 0, sizeof(raw_symbols));
  size_t sym_idx = 0;

  // --- SCHRITT 1: DATA STRUKTUR IM ARDUINO-FORMAT BAUEN ---
  uint32_t rawData = 0;

  // Byte 0: Bit 0..3 Rolling Address, Bit 6..7 Channel (0=Ch1, 1=Ch2, 2=Ch3)
  uint8_t byte0 = (rollingCode & 0x0F);
  if (channel == 2)
    byte0 |= (1 << 6);
  if (channel == 3)
    byte0 |= (2 << 6);
  rawData |= ((uint32_t)byte0);

  // Temperatur in BCD wandeln
  bool isNegative = (temperature < 0);
  if (isNegative)
    temperature = -temperature;
  int tempInt = (int)(temperature * 10.0f + 0.5f);

  uint8_t th3 = tempInt % 10;         // 1/10 Grad
  uint8_t th2 = (tempInt / 10) % 10;  // 1 Grad
  uint8_t th1 = (tempInt / 100) % 10; // 10 Grad

  // Byte 1: Bit 0..3 = TH3, Bit 4..7 = TH2
  uint8_t byte1 = (th3 & 0x0F) | ((th2 & 0x0F) << 4);
  rawData |= ((uint32_t)byte1 << 8);

  // Byte 2: Bit 0..3 = TH1, Bit 5 = Sign (1=negativ)
  uint8_t byte2 = (th1 & 0x0F);
  if (isNegative)
    byte2 |= (1 << 5);
  rawData |= ((uint32_t)byte2 << 16);

  // Byte 3: CRC (Einfache Summe aus Byte 0 + Byte 1 + Byte 2)
  uint8_t crc = calculate_thn128_crc(byte0, byte1, byte2);
  rawData |= ((uint32_t)crc << 24);

  // --- SCHRITT 2: ZWEI DURCHGÄNGE CODIEREN (Doppel-Senden) ---
  for (int tx_run = 0; tx_run < 2; tx_run++)
  {

    // A) Preamble: 12x Bit 1 senden
    for (uint8_t i = 0; i < 12; i++)
    {
      raw_symbols[sym_idx].level0 = 1;
      raw_symbols[sym_idx].duration0 = T_BIT_US;
      raw_symbols[sym_idx].level1 = 0;
      raw_symbols[sym_idx].duration1 = T_BIT_US;
      sym_idx++;
    }
    // Preamble-Space: 3000 µs LOW
    raw_symbols[sym_idx].level0 = 0;
    raw_symbols[sym_idx].duration0 = T_PREAMBLE_SPACE_US;
    raw_symbols[sym_idx].level1 = 0;
    raw_symbols[sym_idx].duration1 = 10; // Minimal-Dummy
    sym_idx++;

    // B) Sync Pulse: 5500 µs HIGH, danach 5500 µs LOW
    raw_symbols[sym_idx].level0 = 1;
    raw_symbols[sym_idx].duration0 = T_SYNC_US;
    raw_symbols[sym_idx].level1 = 0;
    raw_symbols[sym_idx].duration1 = T_SYNC_US;
    sym_idx++;

    // C) 32 Datenbits senden (LSB-First)
    for (uint8_t i = 0; i < 32; i++)
    {
      if (rawData & (1UL << i))
      {
        // KORREKTUR: Bit 1 laut Erriez: HIGH -> LOW (Inverted Manchester)
        raw_symbols[sym_idx].level0 = 1;
        raw_symbols[sym_idx].duration0 = T_BIT_US;
        raw_symbols[sym_idx].level1 = 0;
        raw_symbols[sym_idx].duration1 = T_BIT_US;
      }
      else
      {
        // KORREKTUR: Bit 0 laut Erriez: LOW -> HIGH (Inverted Manchester)
        raw_symbols[sym_idx].level0 = 0;
        raw_symbols[sym_idx].duration0 = T_BIT_US;
        raw_symbols[sym_idx].level1 = 1;
        raw_symbols[sym_idx].duration1 = T_BIT_US;
      }
      sym_idx++;
    }

    // D) Disable Transmit: Pegel auf LOW ziehen
    raw_symbols[sym_idx].level0 = 0;
    raw_symbols[sym_idx].duration0 = T_BIT_US;
    raw_symbols[sym_idx].level1 = 0;
    raw_symbols[sym_idx].duration1 = 10;
    sym_idx++;

    // E) Überlaufsichere 100 ms Frame-Pause zwischen Paket 1 und Paket 2
    if (tx_run == 0)
    {
      for (int p = 0; p < 4; p++)
      {
        raw_symbols[sym_idx].level0 = 0;
        raw_symbols[sym_idx].duration0 = 25000; // 4x 25ms = 100ms
        raw_symbols[sym_idx].level1 = 0;
        raw_symbols[sym_idx].duration1 = 10;
        sym_idx++;
      }
    }
  }

  // Postamble zum finalen Erden des Pins
  raw_symbols[sym_idx].level0 = 0;
  raw_symbols[sym_idx].duration0 = T_BIT_US;
  raw_symbols[sym_idx].level1 = 0;
  raw_symbols[sym_idx].duration1 = T_BIT_US;
  sym_idx++;

  // --- ÜBERTRAGUNG AN DEN CC1101 ABFEUERN ---
  rmt_transmit_config_t tx_config = {};
  tx_config.loop_count = 0;
  tx_config.flags.eot_level = 0;
  ESP_ERROR_CHECK(rmt_transmit(tx_channel, copy_encoder, raw_symbols, sym_idx * sizeof(rmt_symbol_word_t), &tx_config));
}

void HeltecStick::CCTask(void *arg)
{
  HeltecStick *Instance = (HeltecStick *)arg;
  ESP_LOGI(TAG, "Starte CC Task...");

  init_cc1101_hardware_direct();

  // 2. Initialisiere die RMT Hardware-Einheit
  init_rmt_hardware_driver();

  ESP_LOGI(TAG, "System bereit. Starte Sendezyklus für Fielmann Wetterstation...");

  while (1)
  {
    // Kanal 1, ID 0x66, Simulation von 20.5 °C
    send_fielmann_v1_packet(1, 0x66, 20.5f);
    ESP_LOGI(TAG, "Paket abgesetzt. SDR sollte Signal zeigen.");

    // Exaktes v1.0 Sendeintervall (Kanal 1 = 39,25 Sek.)
    vTaskDelay(pdMS_TO_TICKS(39250));
  }

  vTaskDelete(NULL);
}

void HeltecStick::LoRaTask(void *arg)
{
  ESP_LOGI(TAG, "Starte LoRa Task...");
  // Globale Instanz des Funkchips anlegen
  // 1. Die richtige HAL für ESP-IDF instanziieren
  // Dieser Treiber übernimmt im Hintergrund automatisch die spi_master-Steuerung von ESP-IDF!
  EspHal *hal = new EspHal(LORA_PIN_CLK, LORA_PIN_MISO, LORA_PIN_MOSI, SPI2_HOST);
  Module *mod = new Module(hal, LORA_PIN_CS, LORA_PIN_DIO1, LORA_PIN_RESET, LORA_PIN_BUSY);

  // 4. Den SX1262 mit dem Modul verknüpfen
  SX1262 radio = mod;

  // Sendefrequenz: 434,54 MHz
  // Preambellänge: 14
  // Bandbreite 500 kHz
  // Achtung: Damit auch der neuere SX1262 verwendet wird, muss man sich über die Sync-Words ein paar mehr
  // Gedanken machen. Der SX1262 hat 2 Bytes Sync, nicht nur 1 Byte. Damit sich trotzdem beide verstehen, muss man
  // folgendes beachten:
  // SX1278: Jedes Nibble unterschiedlich von 1-7, also z.B. 0x37 => 0xYZ (Y!=Z, Z und Y <8 && >0)
  // SX1262: 0Y4Z4, also hier 0x3474
  // Sync-Byte: 0x37
  // Spreading-Factor: 8 = 256 Chips/symbol
  // Coding-Rate: 6 = 4/6 = 1,5-facher FEC-Overhead
  // Tx-Power 2-17

  // 3. Radio initialisieren (Frequenz, Bandbreite, SF, CR, SyncWord, Power, Preamble)
  int state = radio.begin(434.54, 500, 8, 6, 0x37, 10, 14);

  if (state == RADIOLIB_ERR_NONE)
  {
    ESP_LOGI(TAG, "SX1262 erfolgreich erkannt!\n");

    // Zwingend erforderlich bei Heltec V3 Boards (TCXO)
    radio.setTCXO(1.6);
  }
  else
  {
    ESP_LOGE(TAG, "Fehler bei SX1262 Initialisierung: %d\n", state);
  }
  radio.setSyncWord(0x37);

  // 1. Puffer für die maximal erlaubte LoRa-Paketgröße bereitstellen (max. 256 Bytes)
  uint8_t rx_buffer[256];
  while (1)
  {
    // 2. radio.receive() mit dem Buffer aufrufen.
    // Wenn die Länge auf 0 gesetzt wird, ermittelt RadioLib die Paketgröße automatisch.
    int rx_state = radio.receive(rx_buffer, 0, 1000);

    if (rx_state == RADIOLIB_ERR_NONE)
    {
      // Reale Anzahl der empfangenen Bytes auslesen
      size_t paket_laenge = radio.getPacketLength();

      // Nullterminator am Ende der echten Daten setzen, damit printf weiß, wo der Text aufhört
      if (paket_laenge < 256)
      {
        rx_buffer[paket_laenge] = '\0';
      }

      // Erfolg: Paket ist da und CRC ist korrekt!
      ESP_LOGI(TAG, "\n--- PAKET EMPFANGEN ---\n");
      ESP_LOGI(TAG, "Daten: %s", (char *)rx_buffer);
      ESP_LOGI(TAG, "Länge: %d Bytes", paket_laenge);
      ESP_LOGI(TAG, "RSSI:  %.2f dBm", radio.getRSSI());
      ESP_LOGI(TAG, "SNR:   %.2f dB", radio.getSNR());
      ESP_LOGI(TAG, "-----------------------");
    }
    else if (rx_state == RADIOLIB_ERR_CRC_MISMATCH)
    { // Code -3
      ESP_LOGE(TAG, "[SX1262] CRC-Fehler: Paket im Äther beschädigt oder Parameter-Fehlmatch.\n");
    }
    else
    {
      // Ein anderer Fehlercode (z.B. -6 = Timeout)
      if (rx_state != -6)
        ESP_LOGI(TAG, "[SX1262] Empfangsfehler. Code: %d\n", rx_state);
    }

    // Winzige Pause, um den FreeRTOS-Watchdog zu füttern
    vTaskDelay(pdMS_TO_TICKS(10));
  }

  vTaskDelete(NULL);
}

// Taktzeit für v1.0 Protokoll: 1 / 342Hz = ca. 2924 Mikrosekunden pro Gesamtbit
// Ein halbes Bit (Manchester-Flanke) dauert exakt 1462 Mikrosekunden.
#define DURATION_HALF_BIT_US 1462

// Funktion zur Initialisierung des präzisen RMT Hardware-Treiber auf GPIO 3
void HeltecStick::init_rmt_direct_drive()
{
  rmt_tx_channel_config_t tx_chan_config = {};
  tx_chan_config.gpio_num = CC_PIN_GDO0;
  tx_chan_config.clk_src = RMT_CLK_SRC_DEFAULT;
  tx_chan_config.resolution_hz = 1000000; // 1 MHz Auflösung = 1 Tock pro Mikrosekunde
  tx_chan_config.flags.with_dma = false;
  tx_chan_config.mem_block_symbols = 64; // Reicht für das kurze v1.0 Paket
  tx_chan_config.trans_queue_depth = 4;

  ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_chan_config, &tx_channel));

  // Wir installieren den offiziellen Copy-Encoder für Roh-Wellenformen
  rmt_copy_encoder_config_t copy_encoder_cfg = {};
  ESP_ERROR_CHECK(rmt_new_copy_encoder(&copy_encoder_cfg, &copy_encoder));

  ESP_ERROR_CHECK(rmt_enable(tx_channel));
}

// Sendet die rohe Wellenform über den RMT-Treiber direkt an den CC1101-Modulator
void HeltecStick::send_oregon_v1_packet(uint8_t channel, uint8_t rollingCode, float temperature)
{
  // Array groß genug dimensionieren und komplett nullen
  rmt_symbol_word_t raw_symbols[128];
  memset(raw_symbols, 0, sizeof(raw_symbols));
  size_t sym_idx = 0;

  // --- SCHRITT 1: PRÄAMBEL GENERIEREN ---
  // C++-konforme Zuweisung über die Instanzvariablen
  for (int i = 0; i < 16; i++)
  {
    raw_symbols[sym_idx].level0 = 1;
    raw_symbols[sym_idx].duration0 = DURATION_HALF_BIT_US;
    raw_symbols[sym_idx].level1 = 0;
    raw_symbols[sym_idx].duration1 = DURATION_HALF_BIT_US;
    sym_idx++;
  }

  // --- SCHRITT 2: DATEN-PROTOKOLL AUFBAUEN (4 Bytes) ---
  uint8_t packet[4] = {0};
  packet[0] = rollingCode;

  uint8_t chDigit = (channel == 2) ? 4 : ((channel == 3) ? 8 : 0);
  packet[1] = chDigit << 4;

  bool isNegative = (temperature < 0);
  if (isNegative)
    temperature = -temperature;
  int tempInt = (int)(temperature * 10.0f + 0.5f);
  uint8_t d0 = tempInt % 10;
  uint8_t d1 = (tempInt / 10) % 10;
  uint8_t d2 = (tempInt / 100) % 10;
  uint8_t sign = isNegative ? 0x8 : 0x0;

  packet[1] |= (d0 & 0x0F);
  packet[2] = d1 | (d2 << 4);
  packet[3] = (sign << 4) | ((rollingCode + packet[1] + packet[2]) & 0x0F);

  // --- SCHRITT 3: MANCHESTER-CODIERUNG IN HARDWARE-SYMBOLE ---
  for (int b = 0; b < 4; b++)
  {
    uint8_t dataByte = packet[b];
    // v1.0 verlangt MSB-first (Bit 7 bis Bit 0)
    for (int bit = 7; bit >= 0; bit--)
    {
      uint8_t currentBit = (dataByte >> bit) & 0x01;

      if (currentBit == 1)
      {
        // Logisch 1 -> High-to-Low Wechsel
        raw_symbols[sym_idx].level0 = 1;
        raw_symbols[sym_idx].duration0 = DURATION_HALF_BIT_US;
        raw_symbols[sym_idx].level1 = 0;
        raw_symbols[sym_idx].duration1 = DURATION_HALF_BIT_US;
      }
      else
      {
        // Logisch 0 -> Low-to-High Wechsel
        raw_symbols[sym_idx].level0 = 0;
        raw_symbols[sym_idx].duration0 = DURATION_HALF_BIT_US;
        raw_symbols[sym_idx].level1 = 1;
        raw_symbols[sym_idx].duration1 = DURATION_HALF_BIT_US;
      }
      sym_idx++;
    }
  }

  // Postamble: Letztes Bit mit einer Nullflanke abschließen
  raw_symbols[sym_idx].level0 = 0;
  raw_symbols[sym_idx].duration0 = DURATION_HALF_BIT_US;
  raw_symbols[sym_idx].level1 = 0;
  raw_symbols[sym_idx].duration1 = DURATION_HALF_BIT_US;
  sym_idx++;

  // --- SCHRITT 4: DIREKTES ABSETZEN ÜBER DIE RMT-HARDWARE ---
  rmt_transmit_config_t tx_config = {};
  tx_config.loop_count = 0;
  tx_config.flags.eot_level = 0; // Sorgt für definierten Pegel nach Ende

  ESP_ERROR_CHECK(rmt_transmit(tx_channel, copy_encoder, raw_symbols, sym_idx * sizeof(rmt_symbol_word_t), &tx_config));
}

void HeltecStick::WifiTask(void *arg)
{
  HeltecStick *Instance = (HeltecStick *)arg;
  const char *username = EXAMPLE_PROV_SEC2_USERNAME;
  const char *pop = EXAMPLE_PROV_SEC2_PWD;

  /* Initialize TCP/IP */
  ESP_ERROR_CHECK(esp_netif_init());

  /* Initialize the event loop */
  ESP_ERROR_CHECK(esp_event_loop_create_default());
  wifi_event_group = xEventGroupCreate();

  // 2. MAC-Adresse des Geräts auslesen
  uint8_t mac[6];
  // Holt die werkseitige Basis-MAC-Adresse für das Wi-Fi-Interface
  esp_err_t mac_err = esp_read_mac(mac, ESP_MAC_WIFI_STA);
  if (mac_err != ESP_OK)
  {
    ESP_LOGE("MAIN", "Fehler beim Lesen der MAC-Adresse!");
    // return ESP_ERR_INVALID_ARG;
  }

  // 3. Dynamischen Gerätenamen und optionales Passwort (PoP) generieren
  char device_name[32];
  char device_pop[16];

  // Beispiel: Name wird "PROV_3A4B5C" (die letzten 3 Bytes der MAC in Großbuchstaben)
  snprintf(device_name, sizeof(device_name), "PROV_%02X%02X%02X", mac[3], mac[4], mac[5]);

  // Optional: Auch das Passwort an die MAC koppeln (z.B. "pop_112233445566")
  snprintf(device_pop, sizeof(device_pop), "pop_%02X%02X%02X", mac[3], mac[4], mac[5]);

  ESP_LOGI("MAIN", "Generierter Gerätename: %s", device_name);
  ESP_LOGI("MAIN", "Generierter PoP-Code: %s", device_pop);

  /* Register our event handler for Wi-Fi, IP and Provisioning related events */
  ESP_ERROR_CHECK(esp_event_handler_register(NETWORK_PROV_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL));
  ESP_ERROR_CHECK(esp_event_handler_register(PROTOCOMM_TRANSPORT_BLE_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL));
  ESP_ERROR_CHECK(esp_event_handler_register(PROTOCOMM_SECURITY_SESSION_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL));
  ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL));

  /* Initialize Wi-Fi including netif with default config */
  esp_netif_create_default_wifi_sta();
  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&cfg));

  /* Configuration for the provisioning manager */
  network_prov_mgr_config_t config = {};
  config.network_prov_wifi_conn_cfg = {};
  config.network_prov_wifi_conn_cfg.wifi_conn_attempts = CONFIG_EXAMPLE_PROV_MGR_CONNECTION_CNT;

  /* What is the Provisioning Scheme that we want ?
   * network_prov_scheme_softap or network_prov_scheme_ble */
  config.scheme = network_prov_scheme_ble;
  // config.app_event_handler = wifi_prov_event_handler;

  /* Any default scheme specific event handler that you would
   * like to choose. Since our example application requires
   * neither BT nor BLE, we can choose to release the associated
   * memory once provisioning is complete, or not needed
   * (in case when device is already provisioned). Choosing
   * appropriate scheme specific event handler allows the manager
   * to take care of this automatically. This can be set to
   * NETWORK_PROV_EVENT_HANDLER_NONE when using network_prov_scheme_softap*/
  config.scheme_event_handler = NETWORK_PROV_SCHEME_BLE_EVENT_HANDLER_FREE_BTDM;

  /* Initialize provisioning manager with the
   * configuration parameters set above */
  ESP_ERROR_CHECK(network_prov_mgr_init(config));

  bool provisioned = false;

  /* Netzwerkeinstellungen löschen */
  // network_prov_mgr_reset_wifi_provisioning();

  /* Let's find out if the device is provisioned */
  ESP_ERROR_CHECK(network_prov_mgr_is_wifi_provisioned(&provisioned));

  /* If device is not yet provisioned start provisioning service */
  if (!provisioned)
  {
    ESP_LOGI(TAG, "Starting provisioning");

    /* What is the Device Service Name that we want
     * This translates to :
     *     - Wi-Fi SSID when scheme is network_prov_scheme_softap
     *     - device name when scheme is network_prov_scheme_ble
     */
    char service_name[12];
    Instance->get_device_service_name(service_name, sizeof(service_name));

    network_prov_security2_params_t sec2_params = {};

    ESP_ERROR_CHECK(Instance->example_get_sec2_salt(&sec2_params.salt, &sec2_params.salt_len));
    ESP_ERROR_CHECK(Instance->example_get_sec2_verifier(&sec2_params.verifier, &sec2_params.verifier_len));

    network_prov_security2_params_t *sec_params = &sec2_params;

    /* This step is only useful when scheme is network_prov_scheme_ble. This will
     * set a custom 128 bit UUID which will be included in the BLE advertisement
     * and will correspond to the primary GATT service that provides provisioning
     * endpoints as GATT characteristics. Each GATT characteristic will be
     * formed using the primary service UUID as base, with different auto assigned
     * 12th and 13th bytes (assume counting starts from 0th byte). The client side
     * applications must identify the endpoints by reading the User Characteristic
     * Description descriptor (0x2901) for each characteristic, which contains the
     * endpoint name of the characteristic */
    uint8_t custom_service_uuid[] = {
        /* LSB <---------------------------------------
         * ---------------------------------------> MSB */
        0xb4,
        0xdf,
        0x5a,
        0x1c,
        0x3f,
        0x6b,
        0xf4,
        0xbf,
        0xea,
        0x4a,
        0x82,
        0x03,
        0x04,
        0x90,
        0x1a,
        0x02,
    };

    /* If your build fails with linker errors at this point, then you may have
     * forgotten to enable the BT stack or BTDM BLE settings in the SDK (e.g. see
     * the sdkconfig.defaults in the example project) */
    network_prov_scheme_ble_set_service_uuid(custom_service_uuid);

    /* An optional endpoint that applications can create if they expect to
     * get some additional custom data during provisioning workflow.
     * The endpoint name can be anything of your choice.
     * This call must be made before starting the provisioning.
     */
    network_prov_mgr_endpoint_create("custom-data");

    /* Start provisioning service */
    ESP_ERROR_CHECK(network_prov_mgr_start_provisioning(NETWORK_PROV_SECURITY_2, (const void *)sec_params, service_name, NULL));

    // ESP_ERROR_CHECK(network_prov_mgr_start_provisioning(NETWORK_PROV_SECURITY_2, device_pop, device_name, NULL));

    /* The handler for the optional endpoint created above.
     * This call must be made after starting the provisioning, and only if the endpoint
     * has already been created above.
     */
    network_prov_mgr_endpoint_register("custom-data", custom_prov_data_handler, NULL);

    /* Uncomment the following to wait for the provisioning to finish and then release
     * the resources of the manager. Since in this case de-initialization is triggered
     * by the default event loop handler, we don't need to call the following */
    // network_prov_mgr_wait();
    // network_prov_mgr_deinit();
    /* Print QR code for provisioning */
    Instance->wifi_prov_print_qr(service_name, username, pop, PROV_TRANSPORT_BLE);
    // wifi_prov_print_qr(device_name, "egal", device_pop, PROV_TRANSPORT_BLE);
  }
  else
  {
    ESP_LOGI(TAG, "Already provisioned, starting Wi-Fi STA");

    /* We don't need the manager as device is already provisioned,
     * so let's release it's resources */
    ESP_ERROR_CHECK(network_prov_mgr_deinit());

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL));
    /* Start Wi-Fi station */
    Instance->wifi_init_sta();
  }
  /* Wait for Wi-Fi connection */
  xEventGroupWaitBits(wifi_event_group, WIFI_CONNECTED_EVENT, true, true, portMAX_DELAY);
  ESP_LOGI(TAG, "Wifi successfully started!");
  vTaskDelete(NULL);
}

esp_err_t HeltecStick::InitGPIO()
{
  // Outputs
  const uint64_t OutputBitMask = (1ULL << WHITE_LED);

  gpio_config_t ConfigOutput = {};
  ConfigOutput.pin_bit_mask = OutputBitMask;
  ConfigOutput.mode = GPIO_MODE_OUTPUT;
  ConfigOutput.pull_up_en = GPIO_PULLUP_DISABLE;
  ConfigOutput.pull_down_en = GPIO_PULLDOWN_DISABLE;
  ConfigOutput.intr_type = GPIO_INTR_DISABLE;
  ESP_RETURN_ON_ERROR(gpio_config(&ConfigOutput), TAG, "Fehler bei Init output GPIO");

  // Inputs
  const uint64_t InputBitMask = (1ULL << USER_BUTTON);
  gpio_config_t ConfigInput = {};
  ConfigInput.pin_bit_mask = InputBitMask;
  ConfigInput.mode = GPIO_MODE_INPUT;
  ConfigInput.pull_up_en = GPIO_PULLUP_DISABLE; // die Input-only Pins 34, 36, 39 haben keinen internen Pullup, dann braucht man den auch nicht einschalten!
  ConfigInput.pull_down_en = GPIO_PULLDOWN_DISABLE;
  ConfigInput.intr_type = GPIO_INTR_DISABLE;
  ESP_RETURN_ON_ERROR(gpio_config(&ConfigInput), TAG, "Fehler bei Init input GPIO");

  return ESP_OK;
}

/* Event handler for catching system events */
void HeltecStick::event_handler(void *arg, esp_event_base_t event_base,
                                int32_t event_id, void *event_data)
{
  if (event_base == NETWORK_PROV_EVENT)
  {
    switch (event_id)
    {
    case NETWORK_PROV_START:
      ESP_LOGI(TAG, "Provisioning started");
      break;
    case NETWORK_PROV_WIFI_CRED_RECV:
    {
      wifi_sta_config_t *wifi_sta_cfg = (wifi_sta_config_t *)event_data;
      ESP_LOGI(TAG, "Received Wi-Fi credentials"
                    "\n\tSSID     : %s\n\tPassword : %s",
               (const char *)wifi_sta_cfg->ssid,
               (const char *)wifi_sta_cfg->password);
      break;
    }
    case NETWORK_PROV_WIFI_CRED_FAIL:
    {
      network_prov_wifi_sta_fail_reason_t *reason = (network_prov_wifi_sta_fail_reason_t *)event_data;
      ESP_LOGE(TAG, "Provisioning failed!\n\tReason : %s"
                    "\n\tPlease reset to factory and retry provisioning",
               (*reason == NETWORK_PROV_WIFI_STA_AUTH_ERROR) ? "Wi-Fi station authentication failed" : "Wi-Fi access-point not found");

      network_prov_mgr_reset_wifi_sm_state_on_failure();
      break;
    }
    case NETWORK_PROV_WIFI_CRED_SUCCESS:
      ESP_LOGI(TAG, "Provisioning successful");
      break;
    case NETWORK_PROV_END:
    {
      /* De-initialize manager once provisioning is finished */
      esp_err_t err = network_prov_mgr_deinit();
      if (err != ESP_OK)
      {
        ESP_LOGE(TAG, "Failed to de-initialize provisioning manager: %s", esp_err_to_name(err));
      }
    }
    break;
    default:
      break;
    }
  }
  else if (event_base == WIFI_EVENT)
  {
    switch (event_id)
    {
    case WIFI_EVENT_STA_START:
      esp_wifi_connect();
      break;
    case WIFI_EVENT_STA_DISCONNECTED:
      ESP_LOGI(TAG, "Disconnected. Connecting to the AP again...");
      esp_wifi_connect();
      break;
    default:
      break;
    }
  }
  else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
  {
    ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
    ESP_LOGI(TAG, "Connected with IP Address:" IPSTR, IP2STR(&event->ip_info.ip));
    /* Signal main application to continue execution */
    xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_EVENT);
  }
  else if (event_base == PROTOCOMM_TRANSPORT_BLE_EVENT)
  {
    switch (event_id)
    {
    case PROTOCOMM_TRANSPORT_BLE_CONNECTED:
      ESP_LOGI(TAG, "BLE transport: Connected!");
      break;
    case PROTOCOMM_TRANSPORT_BLE_DISCONNECTED:
      ESP_LOGI(TAG, "BLE transport: Disconnected!");
      break;
    default:
      break;
    }
  }
  else if (event_base == PROTOCOMM_SECURITY_SESSION_EVENT)
  {
    switch (event_id)
    {
    case PROTOCOMM_SECURITY_SESSION_SETUP_OK:
      ESP_LOGI(TAG, "Secured session established!");
      break;
    case PROTOCOMM_SECURITY_SESSION_INVALID_SECURITY_PARAMS:
      ESP_LOGE(TAG, "Received invalid security parameters for establishing secure session!");
      break;
    case PROTOCOMM_SECURITY_SESSION_CREDENTIALS_MISMATCH:
      ESP_LOGE(TAG, "Received incorrect username and/or PoP for establishing secure session!");
      break;
    default:
      break;
    }
  }
}

void HeltecStick::wifi_init_sta(void)
{
  /* Start Wi-Fi in station mode */
  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
  ESP_ERROR_CHECK(esp_wifi_start());
}

void HeltecStick::get_device_service_name(char *service_name, size_t max)
{
  uint8_t eth_mac[6];
  const char *ssid_prefix = "PROV_";
  esp_wifi_get_mac(WIFI_IF_STA, eth_mac);
  snprintf(service_name, max, "%s%02X%02X%02X",
           ssid_prefix, eth_mac[3], eth_mac[4], eth_mac[5]);
}

/* Handler for the optional provisioning endpoint registered by the application.
 * The data format can be chosen by applications. Here, we are using plain ascii text.
 * Applications can choose to use other formats like protobuf, JSON, XML, etc.
 * Note that memory for the response buffer must be allocated using heap as this buffer
 * gets freed by the protocomm layer once it has been sent by the transport layer.
 */
esp_err_t HeltecStick::custom_prov_data_handler(uint32_t session_id, const uint8_t *inbuf, ssize_t inlen,
                                                uint8_t **outbuf, ssize_t *outlen, void *priv_data)
{
  if (inbuf)
  {
    ESP_LOGI(TAG, "Received data: %.*s", inlen, (char *)inbuf);
  }
  char response[] = "SUCCESS";
  *outbuf = (uint8_t *)strdup(response);
  if (*outbuf == NULL)
  {
    ESP_LOGE(TAG, "System out of memory");
    return ESP_ERR_NO_MEM;
  }
  *outlen = strlen(response) + 1; /* +1 for NULL terminating byte */

  return ESP_OK;
}

void HeltecStick::wifi_prov_print_qr(const char *name, const char *username, const char *pop, const char *transport)
{
  if (!name || !transport)
  {
    ESP_LOGW(TAG, "Cannot generate QR code payload. Data missing.");
    return;
  }
  char payload[150] = {0};
  if (pop)
  {

    snprintf(payload, sizeof(payload), "{\"ver\":\"%s\",\"name\":\"%s\""
                                       ",\"username\":\"%s\",\"pop\":\"%s\",\"transport\":\"%s\"}",
             PROV_QR_VERSION, name, username, pop, transport);
  }
  else
  {
    snprintf(payload, sizeof(payload), "{\"ver\":\"%s\",\"name\":\"%s\""
                                       ",\"transport\":\"%s\",\"network\":\"wifi\"}",
             PROV_QR_VERSION, name, transport);
  }
  ESP_LOGI(TAG, "Scan this QR code from the provisioning application for Provisioning.");
  esp_qrcode_config_t cfg = ESP_QRCODE_CONFIG_DEFAULT();
  esp_qrcode_generate(&cfg, payload);
  ESP_LOGI(TAG, "If QR code is not visible, copy paste the below URL in a browser.\n%s?data=%s", QRCODE_BASE_URL, payload);
}

#ifdef CONFIG_EXAMPLE_PROV_ENABLE_APP_CALLBACK
void HeltecStick::wifi_prov_app_callback(void *user_data, wifi_prov_cb_event_t event, void *event_data)
{
  /**
   * This is blocking callback, any configurations that needs to be set when a particular
   * provisioning event is triggered can be set here.
   */
  switch (event)
  {
  case WIFI_PROV_SET_STA_CONFIG:
  {
    /**
     * Wi-Fi configurations can be set here before the Wi-Fi is enabled in
     * STA mode.
     */
    wifi_config_t *wifi_config = (wifi_config_t *)event_data;
    (void)wifi_config;
    break;
  }
  default:
    break;
  }
}

const wifi_prov_event_handler_t wifi_prov_event_handler = {
    .event_cb = wifi_prov_app_callback,
    .user_data = NULL,
};
#endif /* EXAMPLE_PROV_ENABLE_APP_CALLBACK */

esp_err_t HeltecStick::example_get_sec2_salt(const char **salt, uint16_t *salt_len)
{
  ESP_LOGI(TAG, "Development mode: using hard coded salt");
  *salt = sec2_salt;
  *salt_len = sizeof(sec2_salt);
  return ESP_OK;
}

esp_err_t HeltecStick::example_get_sec2_verifier(const char **verifier, uint16_t *verifier_len)
{
  ESP_LOGI(TAG, "Development mode: using hard coded verifier");
  *verifier = sec2_verifier;
  *verifier_len = sizeof(sec2_verifier);
  return ESP_OK;
}

int64_t HeltecStick::GetTime_ms()
{
  // Zeit in Mikrosekunden abfragen und in Millisekunden umrechnen
  return esp_timer_get_time() / 1000;
}

void HeltecStick::ParseCborPayload(uint8_t *buffer, size_t len)
{
  CborParser parser;
  CborValue map_value;

  // Parser auf den Puffer ansetzen
  if (cbor_parser_init(buffer, len, 0, &parser, &map_value) != CborNoError)
    return;

  // Prüfen, ob das Wurzel-Element eine Map/Dictionary ist
  if (cbor_value_is_container(&map_value))
  {
    CborValue it;
    cbor_value_enter_container(&map_value, &it);

    float temperature = 0.0;
    int sender_id = 0;

    while (!cbor_value_at_end(&it))
    {
      // Key lesen (wird als String erwartet)
      char key[16];
      size_t key_len = sizeof(key);
      if (cbor_value_get_string_length(&it, &key_len) == CborNoError)
      {
        _cbor_value_copy_string(&it, key, &key_len, &it);
      }

      // Zum Value springen
      if (strcmp(key, "temp") == 0 && cbor_value_is_float(&it))
      {
        cbor_value_get_float(&it, &temperature);
      }
      else if (strcmp(key, "id") == 0 && cbor_value_is_integer(&it))
      {
        cbor_value_get_int(&it, &sender_id);
      }

      cbor_value_advance(&it); // Nächstes Element
    }

    printf("Geparst - Sender: %d, Temp: %.2f\n", sender_id, temperature);
    // Hier folgt jetzt die Weitergabe an InfluxDB...
  }
}
