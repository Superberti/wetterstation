/* i2c - Example

   For other examples please check:
   https://github.com/espressif/esp-idf/tree/master/examples

   See README.md file to get detailed usage of this example.

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "esp_log.h"
#include "driver/i2c.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/event_groups.h>
#include "freertos/queue.h"

#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"

#include "esp_log.h"
#include "mqtt_client.h"

#include <esp_log.h>
#include <esp_wifi.h>
#include <esp_event.h>
#include <nvs_flash.h>
#include "esp_system.h"
#include "esp_netif.h"
#include "esp32_digital_led_lib.h"
#include <driver/gpio.h>

#include <wifi_provisioning/manager.h>
#include <wifi_provisioning/scheme_ble.h>

#include "sdkconfig.h"
#include "wetterstation.h"
#include "bmp280.h"
#include "veml7700.h"
#include "sht35.h"
#include "driver/ledc.h"
#include "driver/pcnt.h"
#include "tools.h"

#define NUM_LEDS 2

static const char *TAG = "Wetterstation";

// Pinbelegung:
// pin9=GPIO18=I2C SDA
// pin10=GPIO19=I2C SCL
// pin11=GPIO21=Data out für WS2812-LEDs
// pin12=GPIO22=CPU-Lüfter grün=Tachosignal
// pin13=GPIO23=CPU-Lüfter blau=PWM Lüftersteuerung

#define _I2C_NUMBER(num) I2C_NUM_##num
#define I2C_NUMBER(num) _I2C_NUMBER(num)

#define I2C_MASTER_SCL_IO 19               /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO 18               /*!< gpio number for I2C master data  */
#define I2C_MASTER_NUM I2C_NUMBER(0)       /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ 100000          /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE 0        /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0        /*!< I2C master doesn't need buffer */

#define HIGH 1
#define LOW 0
#define OUTPUT GPIO_MODE_OUTPUT
#define INPUT GPIO_MODE_INPUT

strand_t STRANDS[] =
{
  // Avoid using any of the strapping pins on the ESP32
  // ESP32 has 6 strapping pins:
  // MTDI/GPIO12: internal pull-down
  // GPIO0: internal pull-up
  // GPIO2: internal pull-down
  // GPIO4: internal pull-down
  // MTDO/GPIO15: internal pull-up
  // GPIO5: internal pull-up
  {
    .rmtChannel = 0,
    .gpioNum = 21,
    .ledType = LED_WS2813_V2,
    .brightLimit = 255,
    .numPixels = NUM_LEDS,
    .pixels = NULL,
    ._stateVars = NULL
  },
};

const int STRANDCNT = sizeof(STRANDS) / sizeof(STRANDS[0]);


/* Signal Wi-Fi events on this event-group */
const int WIFI_CONNECTED_EVENT = BIT0;
static EventGroupHandle_t wifi_event_group;
static bool smMQTTConnected = false;
static esp_mqtt_client_handle_t mqtt_client = NULL;
static volatile uint8_t smLEDPower=128;

extern "C"
{
  void app_main(void)
  {
    ESP_LOGI(TAG, "Starte ESP32-Wetterstation...");
    ESP_LOGI(TAG, "Startup..");
    ESP_LOGI(TAG, "Free memory: %d bytes", esp_get_free_heap_size());
    ESP_LOGI(TAG, "IDF version: %s", esp_get_idf_version());

    // rote LED auf dem ESP32
    gpioSetup(GPIO_NUM_2, OUTPUT, HIGH);
    digitalLeds_initDriver();

    for (int i = 0; i < STRANDCNT; i++)
    {
      gpioSetup(STRANDS[i].gpioNum, OUTPUT, LOW);
    }

    strand_t *MyStrand[] = {&STRANDS[0]};
    int rc = digitalLeds_addStrands(MyStrand, STRANDCNT);
    bool toggle = false;
    if (rc)
    {
      printf("digitalLeds_addStrands error code: %d. Halting\n", rc);
      while (true)
      {
        toggle = !toggle;
        gpio_set_level(GPIO_NUM_2, (uint32_t)toggle);
        vTaskDelay(100 / portTICK_RATE_MS);
      };
    }

    SetLEDColors(smLEDPower,0,0,smLEDPower,0,0);

    // TODO (pi#1#): Auf Konopfdruck (z.B. 10s) nvs_flash_erase() aufrufen und rebooten, damit wieder das WiFi konfiguriert werden kann.

    ESP_ERROR_CHECK(i2c_master_init());

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

    /* Initialize TCP/IP */
    ESP_ERROR_CHECK(esp_netif_init());

    /* Initialize the event loop */
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_event_group = xEventGroupCreate();

    /* Register our event handler for Wi-Fi, IP and Provisioning related events */
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_PROV_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL));

    /* Initialize Wi-Fi including netif with default config */
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmissing-field-initializers"

    /* Configuration for the provisioning manager */
    wifi_prov_mgr_config_t config =
    {
      .scheme = wifi_prov_scheme_ble,
      /* Any default scheme specific event handler that you would
       * like to choose. Since our example application requires
       * neither BT nor BLE, we can choose to release the associated
       * memory once provisioning is complete, or not needed
       * (in case when device is already provisioned). Choosing
       * appropriate scheme specific event handler allows the manager
       * to take care of this automatically. This can be set to
       * WIFI_PROV_EVENT_HANDLER_NONE when using wifi_prov_scheme_softap*/
      .scheme_event_handler = WIFI_PROV_SCHEME_BLE_EVENT_HANDLER_FREE_BTDM,
    };

#pragma GCC diagnostic pop

    /* Initialize provisioning manager with the
     * configuration parameters set above */
    ESP_ERROR_CHECK(wifi_prov_mgr_init(config));

    bool provisioned = false;
    /* Let's find out if the device is provisioned */
    ESP_ERROR_CHECK(wifi_prov_mgr_is_provisioned(&provisioned));

    /* If device is not yet provisioned start provisioning service */
    if (!provisioned)
    {
      ESP_LOGI(TAG, "Starting provisioning");

      /* What is the Device Service Name that we want
       * This translates to :
       *     - Wi-Fi SSID when scheme is wifi_prov_scheme_softap
       *     - device name when scheme is wifi_prov_scheme_ble
       */
      char service_name[12];
      get_device_service_name(service_name, sizeof(service_name));

      /* What is the security level that we want (0 or 1):
       *      - WIFI_PROV_SECURITY_0 is simply plain text communication.
       *      - WIFI_PROV_SECURITY_1 is secure communication which consists of secure handshake
       *          using X25519 key exchange and proof of possession (pop) and AES-CTR
       *          for encryption/decryption of messages.
       */
      wifi_prov_security_t security = WIFI_PROV_SECURITY_1;

      /* Do we want a proof-of-possession (ignored if Security 0 is selected):
       *      - this should be a string with length > 0
       *      - NULL if not used
       */
      const char *pop = "wetter_001";

      /* What is the service key (could be NULL)
       * This translates to :
       *     - Wi-Fi password when scheme is wifi_prov_scheme_softap
       *     - simply ignored when scheme is wifi_prov_scheme_ble
       */
      const char *service_key = NULL;


      /* This step is only useful when scheme is wifi_prov_scheme_ble. This will
       * set a custom 128 bit UUID which will be included in the BLE advertisement
       * and will correspond to the primary GATT service that provides provisioning
       * endpoints as GATT characteristics. Each GATT characteristic will be
       * formed using the primary service UUID as base, with different auto assigned
       * 12th and 13th bytes (assume counting starts from 0th byte). The client side
       * applications must identify the endpoints by reading the User Characteristic
       * Description descriptor (0x2901) for each characteristic, which contains the
       * endpoint name of the characteristic */
      uint8_t custom_service_uuid[] =
      {
        /* LSB <---------------------------------------
         * ---------------------------------------> MSB */
        0xb4, 0xdf, 0x5a, 0x1c, 0x3f, 0x6b, 0xf4, 0xbf,
        0xea, 0x4a, 0x82, 0x03, 0x04, 0x90, 0x1a, 0x02,
      };
      wifi_prov_scheme_ble_set_service_uuid(custom_service_uuid);


      /* An optional endpoint that applications can create if they expect to
       * get some additional custom data during provisioning workflow.
       * The endpoint name can be anything of your choice.
       * This call must be made before starting the provisioning.
       */
      wifi_prov_mgr_endpoint_create("custom-data");
      /* Start provisioning service */
      ESP_ERROR_CHECK(wifi_prov_mgr_start_provisioning(security, pop, service_name, service_key));

      /* The handler for the optional endpoint created above.
       * This call must be made after starting the provisioning, and only if the endpoint
       * has already been created above.
       */
      wifi_prov_mgr_endpoint_register("custom-data", custom_prov_data_handler, NULL);

      /* Uncomment the following to wait for the provisioning to finish and then release
       * the resources of the manager. Since in this case de-initialization is triggered
       * by the default event loop handler, we don't need to call the following */
      // wifi_prov_mgr_wait();
      // wifi_prov_mgr_deinit();
    }
    else
    {
      ESP_LOGI(TAG, "Already provisioned, starting Wi-Fi STA");

      /* We don't need the manager as device is already provisioned,
       * so let's release it's resources */
      wifi_prov_mgr_deinit();

      /* Start Wi-Fi station */
      wifi_init_sta();
    }

    /* Wait for Wi-Fi connection */
    xEventGroupWaitBits(wifi_event_group, WIFI_CONNECTED_EVENT, false, true, portMAX_DELAY);
    SetLEDColor(0, 0, smLEDPower, 0);

    mqtt_app_start();

    int c=0;
    double temp=0,hum=0;
    bool CrcErr=false;
    char TempStr[16]= {0};
    char HumStr[16]= {0};
    char PresStr[16]= {0};
    char LuxStr[16]= {0};
    double BMP_pres=0, BMP_pres_raw=0,VEML_lux=0;

    BMP280 bmp;
    VEML7700 veml;
    SHT35 sht;

    bool bmp_init_ok = bmp.init();
    if (!bmp_init_ok)
      ESP_LOGE(TAG, "Fehler bei der Initialisierung vom BMP280!");

    bool veml_init_ok=veml.init(VEML7700_GAIN_1_8,VEML7700_IT_25MS);
    if (!veml_init_ok)
      ESP_LOGE(TAG, "Fehler bei der Initialisierung vom VEML7700!");

    bool sht_init_ok=sht.init();
    if (!sht_init_ok)
      ESP_LOGE(TAG, "Fehler bei der Initialisierung vom SHT35!");

    if (bmp_init_ok && veml_init_ok && sht_init_ok)
      SetLEDColor(1, 0, smLEDPower, 0);

    esp_err_t SensorStatus=ESP_OK;
    bool SensorErr=false;

    // Ansteuerung CPU-Luefter
    ledc_timer_config_t ledc_timer =
    {
      .speed_mode = LEDC_HIGH_SPEED_MODE,   // timer mode
      .duty_resolution = LEDC_TIMER_10_BIT, // resolution of PWM duty
      .timer_num = LEDC_TIMER_0,            // timer index
      .freq_hz = 25000,                     // frequency of PWM signal
      .clk_cfg = LEDC_AUTO_CLK,             // Auto select the source clock
    };

    ledc_timer_config(&ledc_timer);

    ledc_channel_config_t ledc_channel =
    {
      .gpio_num   = 23,
      .speed_mode = LEDC_HIGH_SPEED_MODE,
      .channel    = LEDC_CHANNEL_0,
      .intr_type  = LEDC_INTR_DISABLE,
      .timer_sel  = LEDC_TIMER_0,
      .duty       = 0,
      .hpoint     = 0,
    };

    // Set LED Controller with previously prepared configuration

    ledc_channel_config(&ledc_channel);
    const int MaxDuty=1024;
    uint32_t CurrentDuty=640;
    ledc_set_duty(ledc_channel.speed_mode, ledc_channel.channel, CurrentDuty);
    ledc_update_duty(ledc_channel.speed_mode, ledc_channel.channel);


    // PCNT-Einheit zählt die Counterpulse vom CPU-Lüfter
    /* Prepare configuration for the PCNT unit */
    pcnt_config_t pcnt_config = {
        // Set PCNT input signal and control GPIOs
        .pulse_gpio_num = 22,
        .ctrl_gpio_num = -1,
        // What to do when control input is low or high?
        .lctrl_mode = PCNT_MODE_REVERSE, // Reverse counting direction if low
        .hctrl_mode = PCNT_MODE_KEEP,    // Keep the primary counter mode if high

        // What to do on the positive / negative edge of pulse input?
        .pos_mode = PCNT_COUNT_INC,   // Count up on the positive edge
        .neg_mode = PCNT_COUNT_DIS,   // Keep the counter value on the negative edge

        // Set the maximum and minimum limit values to watch
        .counter_h_lim = 32767,
        .counter_l_lim = 0,

        .unit = PCNT_UNIT_0,
        .channel = PCNT_CHANNEL_0,
    };
    /* Initialize PCNT unit */
    pcnt_unit_config(&pcnt_config);

    /* Configure and enable the input filter */
    pcnt_set_filter_value(PCNT_UNIT_0, 100);
    pcnt_filter_enable(PCNT_UNIT_0);
    /* Initialize PCNT's counter */
    pcnt_counter_pause(PCNT_UNIT_0);
    pcnt_counter_clear(PCNT_UNIT_0);
    pcnt_counter_resume(PCNT_UNIT_0);

    int16_t count = 0;
    std::string StatusStr="";
    for(;;)
    {
      SensorErr=false;
      StatusStr="";
      if (sht_init_ok)
      {
        SensorStatus=sht.ReadSHT35(temp, hum, CrcErr);
        if (CrcErr)
        {
          printf("WARNUNG: CRC-Datenfehler aufgetreten, Daten könnten falsch sein.\r\n");
          StatusStr+=strprintf("CRC-Fehler SHT35 ");
        }
        if (SensorStatus!=ESP_OK)
        {
          SensorErr=true;
          StatusStr+=strprintf("Fehler SHT35: %d ",SensorStatus);
          printf("Fehler SHT35: %d ",SensorStatus);
        }

        sprintf(TempStr,"%.2f",temp);
        sprintf(HumStr,"%.2f",hum);
        printf("SHT35 Temperatur: %s°C Luftfeuchte: %s %%\r\n",TempStr,HumStr);
      }

      if (bmp_init_ok)
      {
        //BMP_temp=bmp.ReadTemperature();
        SensorStatus=bmp.ReadPressure(BMP_pres_raw);
        if (SensorStatus!=ESP_OK)
        {
          SensorErr=true;
          StatusStr+=strprintf("Fehler BMP280: %d ",SensorStatus);
        }

        BMP_pres=bmp.seaLevelForAltitude(210, BMP_pres_raw);

        sprintf(PresStr,"%.1f",BMP_pres/100);
        printf("BMP280 Luftdruck: %s hPa [raw: %.1f]\r\n",PresStr,BMP_pres_raw/100);
      }
      if (veml_init_ok)
      {
        SensorStatus=veml.readLuxNormalized(VEML_lux);
        if (SensorStatus!=ESP_OK)
        {
          SensorErr=true;
          StatusStr+=strprintf("Fehler VEML7700: %d ",SensorStatus);
        }

        sprintf(LuxStr,"%.2f",VEML_lux);
        printf("VEML7700 Luxsensor: %s lux\r\n",LuxStr);
        // LED-Power je nach Helligkeit anpassen
        smLEDPower=10+VEML_lux/1000*2;
      }

      if (!SensorErr)
      {
        for (int i=0; i<3; i++)
        {
          SetLEDColor(0,0,0,smLEDPower);
          vTaskDelay(100 / portTICK_RATE_MS);
          SetLEDColor(0,0,0,0);
          vTaskDelay(100 / portTICK_RATE_MS);
        }
      }
      SetLEDColor(0,SensorErr ? smLEDPower : 0, 0,0);
      // Ablegen auf den MQTT-Server
      if (smMQTTConnected && mqtt_client!=NULL)
      {
        esp_mqtt_client_publish(mqtt_client, "/wetterstation/temperatur", TempStr, strlen(TempStr), 1,0);
        esp_mqtt_client_publish(mqtt_client, "/wetterstation/luftfeuchtigkeit", HumStr, strlen(HumStr), 1,0);
        esp_mqtt_client_publish(mqtt_client, "/wetterstation/luftdruck", PresStr, strlen(PresStr), 1,0);
        esp_mqtt_client_publish(mqtt_client, "/wetterstation/beleuchtungsstaerke", LuxStr, strlen(LuxStr), 1,0);

        /*
        for (int i=0; i<3; i++)
        {
          SetLEDColor(1,0,0,0);
          vTaskDelay(100 / portTICK_RATE_MS);
          SetLEDColor(1,0,0,smLEDPower);
          vTaskDelay(100 / portTICK_RATE_MS);
        }
        SetLEDColor(1,0,smLEDPower,0);*/
      }
      vTaskDelay(5000 / portTICK_RATE_MS);
      pcnt_get_counter_value(PCNT_UNIT_0, &count);
      uint16_t CoolerCount=abs(count);
      pcnt_counter_clear(PCNT_UNIT_0);
      printf("Counter CPU-Lüfter: %d PWM: %d\n",CoolerCount,CurrentDuty);

      if (CoolerCount<100)
      {
        // Lüfter ausgefallen?
        StatusStr+=strprintf("Empfange keine oder zu wenige Lüfter-Impulse: %d ",CoolerCount);
        printf("Empfange keine oder zu wenige Lüfter-Impulse: %d ",CoolerCount);
      }
      // Status erst am Ende veröffentlichen
      if (smMQTTConnected && mqtt_client!=NULL)
      {
        if (StatusStr.size()==0)
          StatusStr="Alles OK";
        esp_mqtt_client_publish(mqtt_client, "/wetterstation/status", StatusStr.c_str(), StatusStr.size(), 1,0);
      }
      /*
      CurrentDuty+=128;
      if (CurrentDuty>MaxDuty)
        CurrentDuty=0;
      ledc_set_duty(ledc_channel.speed_mode, ledc_channel.channel, CurrentDuty);
      ledc_update_duty(ledc_channel.speed_mode, ledc_channel.channel);*/
      //SetLEDColors(0, 0, c%2 ? smLEDPower : 0, 0, 0, c%2 ? 0: smLEDPower);
      c++;
    }
  }
}

// Farbei einer LED setzen
void SetLEDColor(uint8_t aLEDNum, uint8_t r, uint8_t g, uint8_t b)
{
  if (aLEDNum>1)
    return;
  strand_t *MyStrand[] = {&STRANDS[0]};
  strand_t *pStrand = &STRANDS[0];
  // Bei meinen 5mm-LEDs sind die Farben rot-gruen vertauscht...
  pixelColor_t Color = pixelFromRGB(g, r, b);
  pStrand->pixels[aLEDNum]=Color;
  digitalLeds_drawPixels(MyStrand, STRANDCNT);
}

// Farben beider LEDs gleichzeitig setzen
void SetLEDColors(uint8_t r1, uint8_t g1, uint8_t b1, uint8_t r2, uint8_t g2, uint8_t b2)
{
  strand_t *MyStrand[] = {&STRANDS[0]};
  strand_t *pStrand = &STRANDS[0];
  // Bei meinen 5mm-LEDs sind die Farben rot-gruen vertauscht...
  pStrand->pixels[0]=pixelFromRGB(g1, r1, b1);
  pStrand->pixels[1]=pixelFromRGB(g2, r2, b2);
  digitalLeds_drawPixels(MyStrand, STRANDCNT);
}

/**
 * @brief i2c master initialization
 */
esp_err_t i2c_master_init(void)
{
  int i2c_master_port = I2C_MASTER_NUM;
  i2c_config_t conf;
  conf.mode = I2C_MODE_MASTER;
  conf.sda_io_num = I2C_MASTER_SDA_IO;
  conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
  conf.scl_io_num = I2C_MASTER_SCL_IO;
  conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
  conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
  i2c_param_config(i2c_master_port, &conf);
  return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}


void disp_buf(uint8_t *buf, int len)
{
  int i;
  for (i = 0; i < len; i++)
  {
    printf("%02x ", buf[i]);
    if ((i + 1) % 16 == 0)
    {
      printf("\n");
    }
  }
  printf("\n");
}

/* Event handler for catching system events */
static void event_handler(void* arg, esp_event_base_t event_base,
                          int event_id, void* event_data)
{
  if (event_base == WIFI_PROV_EVENT)
  {
    switch (event_id)
    {
    case WIFI_PROV_START:
      ESP_LOGI(TAG, "Provisioning started");
      break;
    case WIFI_PROV_CRED_RECV:
    {
      wifi_sta_config_t *wifi_sta_cfg = (wifi_sta_config_t *)event_data;
      ESP_LOGI(TAG, "Received Wi-Fi credentials"
               "\n\tSSID     : %s\n\tPassword : %s",
               (const char *) wifi_sta_cfg->ssid,
               (const char *) wifi_sta_cfg->password);
      break;
    }
    case WIFI_PROV_CRED_FAIL:
    {
      wifi_prov_sta_fail_reason_t *reason = (wifi_prov_sta_fail_reason_t *)event_data;
      ESP_LOGE(TAG, "Provisioning failed!\n\tReason : %s"
               "\n\tPlease reset to factory and retry provisioning",
               (*reason == WIFI_PROV_STA_AUTH_ERROR) ?
               "Wi-Fi station authentication failed" : "Wi-Fi access-point not found");
      break;
    }
    case WIFI_PROV_CRED_SUCCESS:
      ESP_LOGI(TAG, "Provisioning successful");
      break;
    case WIFI_PROV_END:
      /* De-initialize manager once provisioning is finished */
      wifi_prov_mgr_deinit();
      break;
    default:
      break;
    }
  }
  else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START)
  {
    esp_wifi_connect();
  }
  else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
  {
    ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
    ESP_LOGI(TAG, "Connected with IP Address:" IPSTR, IP2STR(&event->ip_info.ip));
    /* Signal main application to continue execution */
    xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_EVENT);
  }
  else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
  {
    ESP_LOGI(TAG, "Disconnected. Connecting to the AP again...");
    esp_wifi_connect();
  }
}

static void wifi_init_sta(void)
{
  /* Start Wi-Fi in station mode */
  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
  ESP_ERROR_CHECK(esp_wifi_start());
}

static void get_device_service_name(char *service_name, size_t max)
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
 */
esp_err_t custom_prov_data_handler(uint32_t session_id, const uint8_t *inbuf, ssize_t inlen,
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

static esp_err_t mqtt_event_handler_cb(esp_mqtt_event_handle_t event)
{
  esp_mqtt_client_handle_t client = event->client;
  int msg_id=0;
  // your_context_t *context = event->context;
  switch (event->event_id)
  {
  case MQTT_EVENT_CONNECTED:
    smMQTTConnected = true;
    SetLEDColor(1, 0, smLEDPower, 0);
    ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
    //msg_id = esp_mqtt_client_publish(client, "/topic/qos1", "data_3", 0, 1, 0);
    //ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);

    msg_id = esp_mqtt_client_subscribe(client, "/wetterstation/commands", 0);
    ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);

    //msg_id = esp_mqtt_client_subscribe(client, "/topic/qos1", 1);
    //ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);

    //msg_id = esp_mqtt_client_unsubscribe(client, "/topic/qos1");
    //ESP_LOGI(TAG, "sent unsubscribe successful, msg_id=%d", msg_id);
    break;
  case MQTT_EVENT_DISCONNECTED:
    smMQTTConnected = false;
    SetLEDColor(1, smLEDPower, 0, 0);
    ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
    break;

  case MQTT_EVENT_SUBSCRIBED:
    ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
    //msg_id = esp_mqtt_client_publish(client, "/topic/qos0", "data", 0, 0, 0);
    ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);
    break;
  case MQTT_EVENT_UNSUBSCRIBED:
    ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
    break;
  case MQTT_EVENT_PUBLISHED:
    ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
    SetLEDColor(1,0,0,smLEDPower);
    vTaskDelay(100 / portTICK_RATE_MS);
    SetLEDColor(1,0,0,0);
    vTaskDelay(100 / portTICK_RATE_MS);
    break;
  case MQTT_EVENT_DATA:
    ESP_LOGI(TAG, "MQTT_EVENT_DATA");
    printf("TOPIC=%.*s\r\n", event->topic_len, event->topic);
    printf("DATA=%.*s\r\n", event->data_len, event->data);
    break;
  case MQTT_EVENT_ERROR:
    ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
    break;
  default:
    ESP_LOGI(TAG, "Other event id:%d", event->event_id);
    break;
  }
  return ESP_OK;
}

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
  ESP_LOGD(TAG, "Event dispatched from event loop base=%s, event_id=%d", base, event_id);
  mqtt_event_handler_cb((esp_mqtt_event_t*)event_data);
}

static void mqtt_app_start(void)
{
  // Raspi Zero-W mit Mosquitto
  const char * mqtt_uri="mqtt://raspiwetter";
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmissing-field-initializers"

  esp_mqtt_client_config_t mqtt_cfg =
  {
    .uri = mqtt_uri,
  };

#pragma GCC diagnostic pop

  mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
  esp_mqtt_client_register_event(mqtt_client, (esp_mqtt_event_id_t)ESP_EVENT_ANY_ID, mqtt_event_handler, mqtt_client);
  esp_mqtt_client_start(mqtt_client);
}
