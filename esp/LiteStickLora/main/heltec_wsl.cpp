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

#define LORA_ORT ORT_SYMPATEC

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
    ESP_LOGE(TAG, "Failed to create LoRaTask task");
    // return ESP_FAIL;
  }

  task_ret = xTaskCreate(WifiTask, "WifiTask", 4096, this, 10, &WifiTaskHandle);
  if (task_ret != pdPASS)
  {
    ESP_LOGE(TAG, "Failed to create WifiTask task");
    // return ESP_FAIL;
  }

  InitGPIO();
  /* Start main application now */

  gpio_set_level(WHITE_LED, 0);

  int64_t StartTime_ms = GetTime_ms();
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

void HeltecStick::LoRaTask(void *arg)
{
  // Globale Instanz des Funkchips anlegen
  // 1. Die richtige HAL für ESP-IDF instanziieren
  // Dieser Treiber übernimmt im Hintergrund automatisch die spi_master-Steuerung von ESP-IDF!
  EspHal *hal = new EspHal(LORA_PIN_CLK, LORA_PIN_MISO, LORA_PIN_MOSI);
  Module *mod = new Module(hal, LORA_PIN_CS, LORA_PIN_DIO1, LORA_PIN_RESET, LORA_PIN_BUSY);

  // 4. Den SX1262 mit dem Modul verknüpfen
  SX1262 radio = mod;

  printf("Starte RadioLib im ESP-IDF Modus...\n");

  // SetupModule(uint8_t aAddress, uint32_t aFrq, uint16_t aPreambleLength, LoRaBandwidth aBandwidth, uint16_t aSyncWord, SpreadingFactor aSpreadingFactor, LoRaCodingRate aCodingRate, int8_t aTxPower);
  // SetupModule(LORA_ADDR_GWHS, 434.54e6, 14, LoRaBase::LoRaBandwidth::LORA_BW_500, 0x3575, LoRaBase::SpreadingFactor::SF8, LoRaBase::LoRaCodingRate::LORA_CR_4_6, 15);
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
  /*
  HeltecStick *Instance = (HeltecStick *)arg;
  esp_err_t ret;
  LoRa_PinConfiguration PinConfig = {};
  PinConfig.ChipSelect = LORA_PIN_CS;
  PinConfig.Clock = LORA_PIN_CLK;
  PinConfig.Miso = LORA_PIN_MISO;
  PinConfig.Mosi = LORA_PIN_MOSI;
  PinConfig.Reset = LORA_PIN_RESET;
  PinConfig.Busy = LORA_PIN_BUSY;
  PinConfig.SPIChannel = SPI2_HOST;
  PinConfig.UseTXCO = 1;

  Instance->LoRa = new SX1262_LoRa(PinConfig);
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
  bool LoraOK = true;
  ret = Instance->LoRa->SetupModule(LORA_ADDR_GWHS, 434.54e6, 14, LoRaBase::LoRaBandwidth::LORA_BW_500, 0x3575, LoRaBase::SpreadingFactor::SF8, LoRaBase::LoRaCodingRate::LORA_CR_4_6, 15);

  //    LoRa->SetupModule(LORA_ADDR_GWHS, 434.54e6, 14, LoRaBase::LoRaBandwidth::LORA_BW_500, 0x37, LoRaBase::SpreadingFactor::SF8, LoRaBase::LoRaCodingRate::LORA_CR_4_6, 15);

  if (ret != ESP_OK)
  {
    LoraOK = false;
    ESP_LOGE(TAG, "Fehler beim Initialisieren vom LoRa-Modul: %d", ret);
  }
  else
  {
    ESP_LOGE(TAG, "Init SX1262 OK!");
  }
  int64_t StartTime_ms = Instance->GetTime_ms();
  uint8_t buf[255];
  uint8_t BytesRead = 0;
  ESP_LOGI(TAG, "Waiting for new LoRa message...");
  while (1)
  {
    ret = Instance->LoRa->ReceivePacket(buf, 255, &BytesRead, 1000);
    // Gibt einen Punkt aus, sofern das INFO-Level für diesen TAG aktiv ist
    esp_log_write(ESP_LOG_INFO, TAG, ".");
    fflush(stdout);
    if (ret == ESP_OK)
    {
      ESP_LOGI(TAG, "\r\nNew packet received with %d bytes", BytesRead);
    }
    else if (ret == ESP_ERR_TIMEOUT)
    {
      // ESP_LOGI(TAG, "Nothing received...");
    }
    else if (ret != ESP_OK)
    {
      ESP_LOGE(TAG, "\r\nFehler bei ReceivePacket: %d", ret);
      vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
  */
  vTaskDelete(NULL);
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