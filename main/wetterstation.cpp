/* ESP32-Wetterstation
 * Wetterstation mit verschiedenen Sensoren, die ihre Daten über MQTT
 * in Netz liefern.
 * 03.02.2021 O.Rutsch
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
#include "esp_task_wdt.h"
#include "driver/uart.h"

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
#include <math.h>

#define NUM_LEDS 2

static const char *TAG = "Wetterstation";

// Pinbelegung:
// pin9=GPIO18=I2C SDA
// pin10=GPIO19=I2C SCL
// pin11=GPIO21=Data out für WS2812-LEDs
// pin12=GPIO22=CPU-Lüfter grün=Tachosignal
// pin13=GPIO23=CPU-Lüfter blau=PWM Lüftersteuerung
// pin07=GPIO17=NRF24 Set. Low=AT-Kommandos, High=Datentransfer

#define _I2C_NUMBER(num) I2C_NUM_##num
#define I2C_NUMBER(num) _I2C_NUMBER(num)

#define I2C_MASTER_SCL_IO 19               /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO 18               /*!< gpio number for I2C master data  */
#define I2C_MASTER_NUM I2C_NUMBER(0)       /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ 20000           /*!< I2C master clock frequency */
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
    .ledType = LED_WS2811_HS,
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
static xQueueHandle led_cmd_queue = NULL;
#define BUF_SIZE (1024)
std::string nrf_answer;
extern "C"
{
  void app_main(void)
  {
    ESP_LOGI(TAG, "Starte ESP32-Wetterstation...");
    ESP_LOGI(TAG, "Freier Heap: %d bytes", esp_get_free_heap_size());
    ESP_LOGI(TAG, "IDF version: %s", esp_get_idf_version());

    // GPIO17 schaltet Kommandomodus des NRF
    gpioSetup(GPIO_NUM_17, OUTPUT, LOW);

    uart_config_t uart_config =
    {
      .baud_rate = 9600,
      .data_bits = UART_DATA_8_BITS,
      .parity    = UART_PARITY_DISABLE,
      .stop_bits = UART_STOP_BITS_1,
      .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
      .rx_flow_ctrl_thresh = 0,
      .source_clk = UART_SCLK_APB,
    };
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_1, BUF_SIZE * 2, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_NUM_1, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM_1, 4, 5, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    // NRF24 einrichten
    bool NrfErr=false;
    nrf_answer=NRFCommand("AT+RFID2008\r\n");
    NrfErr|=nrf_answer!="OK";
    nrf_answer=NRFCommand("AT+DVID1374\r\n");
    NrfErr|=nrf_answer!="OK";
    nrf_answer=NRFCommand("AT+RFC079\r\n");
    NrfErr|=nrf_answer!="OK";
    nrf_answer=NRFCommand("AT+POWE8\r\n");
    NrfErr|=nrf_answer!="OK";
    nrf_answer=NRFCommand("AT+CLSSA0\r\n");
    NrfErr|=nrf_answer!="OK";

    if (NrfErr)
      ESP_LOGE(TAG,"Fehler beim initialisieren vom NRF24-Modul!");
    else
    {
      ESP_LOGI(TAG, "Init NRF24 OK.");
      //ESP_LOGI(TAG, "Splitte ESP_LOG-Ausgabe auf NRF24.");
      //esp_log_set_vprintf(&nrf_vprintf);
    }

    led_task_init();
    SetLEDColor(0,128,0,0);
    SetLEDColor(1,128,0,0);
    ESP_LOGI(TAG, "Digital LEDs OK");

    // TODO (pi#1#): Auf Konopfdruck (z.B. 10s) nvs_flash_erase() aufrufen und rebooten, damit wieder das WiFi konfiguriert werden kann.

    ESP_ERROR_CHECK(i2c_master_init());
    ESP_LOGI(TAG, "I2C OK");

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
    ESP_LOGI(TAG, "NVS OK");

    /* Initialize TCP/IP */
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_LOGI(TAG, "TCP/IP OK");

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
    ESP_LOGI(TAG, "Wifi init OK");

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

    ESP_LOGI(TAG,"Warte auf WLAN...");
    /* Wait for Wi-Fi connection */
    xEventGroupWaitBits(wifi_event_group, WIFI_CONNECTED_EVENT, false, true, portMAX_DELAY);
    ESP_LOGI(TAG,"WLAN eingeloggt.");
    SetLEDColor(0, 0, smLEDPower, 0);

    mqtt_app_start();
    ESP_LOGI(TAG, "MQTT OK");

    int c=0;
    double temp=0,hum=0;
    bool CrcErr=false;
    char TempStr0[16]= {0};
    char HumStr0[16]= {0};
    char TempStr1[16]= {0};
    char HumStr1[16]= {0};
    char PresStr[16]= {0};
    char LuxStr[16]= {0};
    char CoolerStr[16]= {0};
    double BMP_pres=0, BMP_pres_raw=0,VEML_lux=0;

    BMP280 bmp;
    VEML7700 veml;
    SHT35 sht0(0);
    SHT35 sht1(1);

    bool bmp_init_ok = bmp.init();
    if (!bmp_init_ok)
      ESP_LOGE(TAG, "Fehler bei der Initialisierung vom BMP280!");

    bool veml_init_ok=veml.init(VEML7700_GAIN_1_8,VEML7700_IT_25MS);
    if (!veml_init_ok)
      ESP_LOGE(TAG, "Fehler bei der Initialisierung vom VEML7700!");

    bool sht0_init_ok=sht0.init();
    if (!sht0_init_ok)
      ESP_LOGE(TAG, "Fehler bei der Initialisierung vom SHT35-0!");

    bool sht1_init_ok=sht1.init();
    if (!sht1_init_ok)
      ESP_LOGE(TAG, "Fehler bei der Initialisierung vom SHT35-1!");

    if (bmp_init_ok && veml_init_ok && sht0_init_ok && sht1_init_ok)
      SetLEDColor(1, 0, smLEDPower, 0);

    esp_err_t SensorStatus=ESP_OK;

    // Bitfeld mit möglichen Sensorfehlern
    uint16_t SensorErr=0;

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
    //const int MaxDuty=1024;
    uint32_t CurrentDuty=800;
    ledc_set_duty(ledc_channel.speed_mode, ledc_channel.channel, CurrentDuty);
    ledc_update_duty(ledc_channel.speed_mode, ledc_channel.channel);


    // PCNT-Einheit zählt die Counterpulse vom CPU-Lüfter
    // Prepare configuration for the PCNT unit
    pcnt_config_t pcnt_config =
    {
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
    // Initialize PCNT unit
    pcnt_unit_config(&pcnt_config);

    // Configure and enable the input filter
    pcnt_set_filter_value(PCNT_UNIT_0, 100);
    pcnt_filter_enable(PCNT_UNIT_0);
    // Initialize PCNT's counter
    pcnt_counter_pause(PCNT_UNIT_0);
    pcnt_counter_clear(PCNT_UNIT_0);
    pcnt_counter_resume(PCNT_UNIT_0);

    int16_t count = 0;
    std::string StatusStr="";
    std::string LastErrorString="";

    // Gleitende Mittelwerte der Messdaten
    double temp0_mean=0.0;
    double temp1_mean=0.0;
    double press_mean=0.0;
    double lux_mean=0.0;
    double hum0_mean=0.0;
    double hum1_mean=0.0;
    // Maximale Sprunggrößen zwischen zwei Messwerten bevor der Wert als suspekt eingestuft wird
    double temp_diff_max=1;     // Grad
    double press_diff_max=5;    // Millibar
    double lux_diff_max=50000;  // lux
    double hum_diff_max=5;      // Prozent

    // Wie oft soll bei einem I2C-Fehler das Kommando wiederholt werden?
    const int ErrorRetry=5;
    int RetryCounter=0;
    // Messdaten alle 10 Sekunden abfragen
    const int LoopDelayTime_s=10;
    // Watchdog auf 30 s, panic-handler bei timeout auslösen
    ESP_ERROR_CHECK(esp_task_wdt_init(LoopDelayTime_s*3, true));
    ESP_ERROR_CHECK(esp_task_wdt_add(NULL));
    bool RetrySuspectData=false;
    bool ErrorLogged=false;
    for(;;)
    {
      esp_task_wdt_reset();
      SensorErr=0;
      StatusStr="";

      if (sht0_init_ok)
      {
        RetryCounter=0;
        do
        {
          RetrySuspectData=false;
          SensorStatus=sht0.ReadSHT35(temp, hum, CrcErr);
          if (c==0)
          {
            // Startwerte
            temp0_mean=temp;
            hum0_mean= hum;
          }

          if (SensorStatus!=ESP_OK)
          {
            SetSensorErr(SensorErr,eSHT35_0,true);
            StatusStr+=strprintf("Fehler SHT35-0: %d ",SensorStatus);
            ESP_LOGE(TAG,"Fehler SHT35-0: %d, Versuch %d ",SensorStatus, RetryCounter+1);
            // I2C-Reset
            i2c_master_reset();
            vTaskDelay(1000 / portTICK_RATE_MS);
          }
          else if (CrcErr)
          {
            ESP_LOGW(TAG,"WARNUNG: SHT35-0 CRC-Datenfehler aufgetreten, Daten könnten falsch sein.");
            StatusStr+=strprintf("CRC-Fehler SHT35-0 Versuch %d ",RetryCounter+1);
            RetrySuspectData=true;
          }
          else
          {
            SetSensorErr(SensorErr,eSHT35_0,false);
            // Daten plausibel? (sollte durch CRC eigentlich nicht auftreten können...)
            if (fabs(temp-temp0_mean)>temp_diff_max)
            {
              ESP_LOGW(TAG,"WARNUNG: SHT35-0 Temperatur suspekt.");
              StatusStr+=strprintf("Suspekte Temperatur SHT35-0: Aktuell: %.2f°C Glt.Mitt.: %.2f°C ",temp,temp0_mean);
              RetrySuspectData=true;
            }
            else
              temp0_mean = temp0_mean*0.8+0.2*temp;

            if (fabs(hum-hum0_mean)>hum_diff_max)
            {
              ESP_LOGW(TAG,"WARNUNG: SHT35-0 Luftfeuchtigkeit suspekt.");
              StatusStr+=strprintf("Suspekte Luftfeuchtigkeit SHT35-0: Aktuell: %.2f %% Glt.Mitt.: %.2f %% ",hum,hum0_mean);
              RetrySuspectData=true;
            }
            else
              hum0_mean= hum0_mean*0.8+0.2*hum;

          }

          RetryCounter++;
        }
        while ((GetSensorErr(SensorErr,eSHT35_0) || RetrySuspectData) && RetryCounter<=ErrorRetry);
        if (GetSensorErr(SensorErr,eSHT35_0))
        {
          ESP_LOGE(TAG,"SHT35-0 Fehler: Gebe auf nach %d Versuchen!",ErrorRetry);
        }
        else
        {
          if (RetrySuspectData)
          {
            // Auch nach mehrmaligem Neuversuchen weichen die Daten zu sehr vom
            // Mittelwert ab. Damit sich die Werte nicht "festfahren" initialisieren
            // wir den gleitenden Mittelwert neu...
            ESP_LOGW(TAG,"WARNUNG: Neuzuweisung gleitender Mittelwert SHT35-0.");
            StatusStr+=strprintf("WARNUNG: Neuzuweisung gleitender Mittelwert SHT35-0.");
            temp0_mean=temp;
            hum0_mean=hum;
          }
          sprintf(TempStr0,"%.2f",temp0_mean);
          sprintf(HumStr0,"%.2f",hum0_mean);
          ESP_LOGI(TAG,"SHT35-0 Temperatur: %s°C Luftfeuchte: %s %%",TempStr0,HumStr0);
        }
      }

      if (sht1_init_ok)
      {
        RetryCounter=0;
        do
        {
          RetrySuspectData=false;
          SensorStatus=sht1.ReadSHT35(temp, hum, CrcErr);
          if (c==0)
          {
            // Startwerte
            temp1_mean=temp;
            hum1_mean= hum;
          }

          if (SensorStatus!=ESP_OK)
          {
            SetSensorErr(SensorErr,eSHT35_1,true);
            StatusStr+=strprintf("Fehler SHT35-1: %d ",SensorStatus);
            ESP_LOGE(TAG,"Fehler SHT35-1: %d, Versuch %d ",SensorStatus, RetryCounter+1);
            // I2C-Reset
            i2c_master_reset();
            vTaskDelay(1000 / portTICK_RATE_MS);
          }
          else if (CrcErr)
          {
            ESP_LOGW(TAG,"WARNUNG: SHT35-1 CRC-Datenfehler aufgetreten, Daten könnten falsch sein.");
            StatusStr+=strprintf("CRC-Fehler SHT35-1 Versuch %d ",RetryCounter+1);
            RetrySuspectData=true;
          }
          else
          {
            SetSensorErr(SensorErr,eSHT35_1,false);
            // Daten plausibel? (sollte durch CRC eigentlich nicht auftreten können...)
            if (fabs(temp-temp1_mean)>temp_diff_max)
            {
              ESP_LOGW(TAG,"WARNUNG: SHT35-1 Temperatur suspekt.");
              StatusStr+=strprintf("Suspekte Temperatur SHT35-1: Aktuell: %.2f°C Glt.Mitt.: %.2f°C ",temp,temp1_mean);
              RetrySuspectData=true;
            }
            else
              temp1_mean = temp1_mean*0.8+0.2*temp;

            if (fabs(hum-hum1_mean)>hum_diff_max)
            {
              ESP_LOGW(TAG,"WARNUNG: SHT35-1 Luftfeuchtigkeit suspekt.");
              StatusStr+=strprintf("Suspekte Luftfeuchtigkeit SHT35-1: Aktuell: %.2f %% Glt.Mitt.: %.2f %% ",hum,hum1_mean);
              RetrySuspectData=true;
            }
            else
              hum1_mean= hum1_mean*0.8+0.2*hum;

          }

          RetryCounter++;
        }
        while ((GetSensorErr(SensorErr,eSHT35_1) || RetrySuspectData) && RetryCounter<=ErrorRetry);
        if (GetSensorErr(SensorErr,eSHT35_1))
        {
          ESP_LOGE(TAG,"SHT35-1 Fehler: Gebe auf nach %d Versuchen!",ErrorRetry);
        }
        else
        {
          if (RetrySuspectData)
          {
            // Auch nach mehrmaligem Neuversuchen weichen die Daten zu sehr vom
            // Mittelwert ab. Damit sich die Werte nicht "festfahren" initialisieren
            // wir den gleitenden Mittelwert neu...
            ESP_LOGW(TAG,"WARNUNG: Neuzuweisung gleitender Mittelwert SHT35-1.");
            StatusStr+=strprintf("WARNUNG: Neuzuweisung gleitender Mittelwert SHT35-1.");
            temp1_mean=temp;
            hum1_mean=hum;
          }
          sprintf(TempStr1,"%.2f",temp1_mean);
          sprintf(HumStr1,"%.2f",hum1_mean);
          ESP_LOGI(TAG,"SHT35-1 Temperatur: %s°C Luftfeuchte: %s %%",TempStr1,HumStr1);
        }
      }

      if (bmp_init_ok)
      {
        RetryCounter=0;
        do
        {
          RetrySuspectData=false;
          SensorStatus=bmp.ReadPressure(BMP_pres_raw);
          BMP_pres=bmp.seaLevelForAltitude(210, BMP_pres_raw)/100.0; // in mbar=hPa
          if (c==0)
            press_mean=BMP_pres;
          if (SensorStatus!=ESP_OK)
          {
            SetSensorErr(SensorErr,eBMP280,true);
            StatusStr+=strprintf("Fehler BMP280: %d ",SensorStatus);
            ESP_LOGE(TAG,"Fehler BMP280: %d, Versuch %d ",SensorStatus, RetryCounter+1);
            // I2C-Reset
            i2c_master_reset();
            vTaskDelay(1000 / portTICK_RATE_MS);
          }
          else
          {
            SetSensorErr(SensorErr,eBMP280,false);
            // Daten plausibel?
            if (fabs(BMP_pres-press_mean)>press_diff_max)
            {
              ESP_LOGW(TAG,"WARNUNG: BMP280 Luftdruck suspekt.");
              StatusStr+=strprintf("Suspekter Luftdruck BMP280: Aktuell: %.2f hPa Glt.Mitt.: %.2f hPa ",BMP_pres,press_mean);
              RetrySuspectData=true;
            }
            else
              press_mean = press_mean*0.8+0.2*BMP_pres;
          }

          RetryCounter++;
        }
        while ((GetSensorErr(SensorErr,eBMP280) | RetrySuspectData) && RetryCounter<=ErrorRetry);
        if (GetSensorErr(SensorErr,eBMP280))
        {
          ESP_LOGE(TAG,"BMP280 Fehler: Gebe auf nach %d Versuchen!",ErrorRetry);
        }
        else
        {
          if (RetrySuspectData)
          {
            // Auch nach mehrmaligem Neuversuchen weichen die Daten zu sehr vom
            // Mittelwert ab. Damit sich die Werte nicht "festfahren" initialisieren
            // wir den gleitenden Mittelwert neu...
            ESP_LOGW(TAG,"WARNUNG: Neuzuweisung gleitender Mittelwert BMP280.");
            StatusStr+=strprintf("WARNUNG: Neuzuweisung gleitender Mittelwert BMP280.");
            press_mean=BMP_pres;
          }
          sprintf(PresStr,"%.1f",press_mean);
          ESP_LOGI(TAG,"BMP280 Luftdruck: %s hPa [raw: %.1f]",PresStr,BMP_pres_raw/100);
        }
      }

      if (veml_init_ok)
      {
        RetryCounter=0;
        do
        {
          RetrySuspectData=false;
          SensorStatus=veml.readLuxNormalized(VEML_lux);
          if (c==0)
            lux_mean=VEML_lux;
          if (SensorStatus!=ESP_OK)
          {
            SetSensorErr(SensorErr,eVEML7700,true);
            StatusStr+=strprintf("Fehler VEML7700: %d ",SensorStatus);
            ESP_LOGE(TAG,"Fehler VEML7700: %d, Versuch %d ",SensorStatus, RetryCounter+1);
            // I2C-Reset
            i2c_master_reset();
            vTaskDelay(1000 / portTICK_RATE_MS);
          }
          else
          {
            SetSensorErr(SensorErr,eVEML7700,false);
            // Daten plausibel?
            if (fabs(VEML_lux-lux_mean)>lux_diff_max || VEML_lux>300000)
            {
              ESP_LOGW(TAG,"WARNUNG: VEML7700 Bel.Stärke suspekt.");
              StatusStr+=strprintf("Suspekte Bel.Stärke VEML7700: Aktuell: %.2f lux Glt.Mitt.: %.2f lux ",VEML_lux,lux_mean);
              RetrySuspectData=true;
            }
            else
              lux_mean = lux_mean*0.8+0.2*VEML_lux;
          }

          RetryCounter++;
        }
        while ((GetSensorErr(SensorErr,eVEML7700) | RetrySuspectData) && RetryCounter<=ErrorRetry);
        if (GetSensorErr(SensorErr,eVEML7700))
        {
          ESP_LOGE(TAG,"VEML7700 Fehler: Gebe auf nach %d Versuchen!",ErrorRetry);
        }
        else
        {
          if (RetrySuspectData)
          {
            if (VEML_lux<300000)
            {
              // Auch nach mehrmaligem Neuversuchen weichen die Daten zu sehr vom
              // Mittelwert ab. Damit sich die Werte nicht "festfahren" initialisieren
              // wir den gleitenden Mittelwert neu...
              ESP_LOGW(TAG,"WARNUNG: Neuzuweisung gleitender Mittelwert VEML7700.");
              StatusStr+=strprintf("WARNUNG: Neuzuweisung gleitender Mittelwert VEML7700.");
              lux_mean=VEML_lux;
            }
          }
          smLEDPower=std::max((int)255,(int)(10+VEML_lux/20));
          sprintf(LuxStr,"%.2f",lux_mean);
          ESP_LOGI(TAG,"VEML7700 Luxsensor: %s lux",LuxStr);
        }
      }

      if (c)
      {
        pcnt_get_counter_value(PCNT_UNIT_0, &count);
        uint16_t CoolerCount=abs(count);
        pcnt_counter_clear(PCNT_UNIT_0);
        ESP_LOGI(TAG,"Counter CPU-Lüfter: %d PWM: %d",CoolerCount,CurrentDuty);

        sprintf(CoolerStr,"%d",CoolerCount);
        if (CoolerCount<100)
        {
          // Lüfter ausgefallen?
          StatusStr+=strprintf("Empfange keine oder zu wenige Lüfter-Impulse: %d ",CoolerCount);
          ESP_LOGE(TAG,"Empfange keine oder zu wenige Lüfter-Impulse: %d ",CoolerCount);
        }
      }

      if (SensorErr==0)
      {
        for (int i=0; i<3; i++)
        {
          SetLEDColor(0,0,0,smLEDPower);
          gpio_set_level(GPIO_NUM_2, 1);
          vTaskDelay(100 / portTICK_RATE_MS);
          SetLEDColor(0,0,0,0);
          gpio_set_level(GPIO_NUM_2, 0);
          vTaskDelay(100 / portTICK_RATE_MS);
        }
      }
      SetLEDColor(0,SensorErr ? smLEDPower : 0, 0,0);
      gpio_set_level(GPIO_NUM_2, SensorErr ? 1 : 0);
      // Ablegen auf den MQTT-Server
      if (smMQTTConnected && mqtt_client!=NULL)
      {
        if (GetSensorErr(SensorErr,eSHT35_1)==false)
        {
          esp_mqtt_client_publish(mqtt_client, "/wetterstation/aussen/temperatur", TempStr1, strlen(TempStr1), 1,0);
          esp_mqtt_client_publish(mqtt_client, "/wetterstation/aussen/luftfeuchtigkeit", HumStr1, strlen(HumStr1), 1,0);
        }
        if (GetSensorErr(SensorErr,eSHT35_0)==false)
        {
          esp_mqtt_client_publish(mqtt_client, "/wetterstation/aussen/temperatur_top", TempStr0, strlen(TempStr0), 1,0);
          esp_mqtt_client_publish(mqtt_client, "/wetterstation/aussen/luftfeuchtigkeit_top", HumStr0, strlen(HumStr0), 1,0);
        }
        if (GetSensorErr(SensorErr,eBMP280)==false)
          esp_mqtt_client_publish(mqtt_client, "/wetterstation/aussen/luftdruck", PresStr, strlen(PresStr), 1,0);
        if (GetSensorErr(SensorErr,eVEML7700)==false)
          esp_mqtt_client_publish(mqtt_client, "/wetterstation/aussen/beleuchtungsstaerke", LuxStr, strlen(LuxStr), 1,0);
        esp_mqtt_client_publish(mqtt_client, "/wetterstation/aussen/luefterdrehzahl", CoolerStr, strlen(CoolerStr), 1,0);
        if (StatusStr.size()==0)
          StatusStr="Alles OK";
        else
        {
          LastErrorString=StatusStr;
          ErrorLogged=false;
        }

        if (c==0)
        {
          StatusStr="Wetterstation neu gestartet!";
          esp_mqtt_client_publish(mqtt_client, "/wetterstation/aussen/status", StatusStr.c_str(), StatusStr.size(), 1,0);
        }
        else
          esp_mqtt_client_publish(mqtt_client, "/wetterstation/aussen/status", StatusStr.c_str(), StatusStr.size(), 1,0);
        if (!ErrorLogged)
        {
          ErrorLogged=true;
          esp_mqtt_client_publish(mqtt_client, "/wetterstation/error/status", LastErrorString.c_str(), LastErrorString.size(), 1,0);
        }
      }
      vTaskDelay(LoopDelayTime_s*1000 / portTICK_RATE_MS);
      c++;
    }
    esp_task_wdt_deinit();
  }
}

std::string NRFCommand(std::string aCmd)
{
  char nrf_answer[20];
  gpio_set_level(GPIO_NUM_17, LOW); // Kommandomodus einschalten
  vTaskDelay(100 / portTICK_RATE_MS);  // etwas warten
  uart_write_bytes(UART_NUM_1, aCmd.c_str(), aCmd.size());
  int len = uart_read_bytes(UART_NUM_1, (uint8_t*)nrf_answer, sizeof(nrf_answer)-1, 200 / portTICK_RATE_MS);
  std::string ans="";
  if (len>0)
  {
    nrf_answer[std::min(len,(int)(sizeof(nrf_answer)-1))]=0;
    KillReturnAndEndl(nrf_answer);
    ans=std::string(nrf_answer);
    ESP_LOGI(TAG, "%d bytes vom NRF24 gelesen: %s", len, nrf_answer);
  }
  else if (len<0)
    ESP_LOGE(TAG, "Fehler beim Lesen vom NRF01!");
  else
    ESP_LOGE(TAG, "Timeout beim Lesen vom NRF01!");
  gpioSetup(GPIO_NUM_17, OUTPUT, HIGH);
  return ans;
}

void NRFLog(std::string aLog)
{
  gpio_set_level(GPIO_NUM_17, HIGH); // Write through einschalten
  uart_write_bytes(UART_NUM_1, aLog.c_str(), aLog.size());
}

int nrf_vprintf(const char *fmt, va_list args)
{
  std::string log=strprintf(fmt,args);
  NRFLog(log);
  return log.size();
}

void SetSensorErr(uint16_t & aSensorErr, SensorType aSensorType, bool aStatus)
{
  if (aStatus)
    aSensorErr |= (1 << (uint16_t)aSensorType);
  else
    aSensorErr &= ~(1 << (uint16_t)aSensorType);
}

bool GetSensorErr(const uint16_t aSensorErr, SensorType aSensorType)
{
  return (aSensorErr & (1 << (uint16_t)aSensorType)) > 0;
}

void led_cmd_task(void * arg)
{
  // rote LED auf dem ESP32
  gpioSetup(GPIO_NUM_2, OUTPUT, HIGH);
  digitalLeds_initDriver();
  ESP_LOGI(TAG, "digitalLeds_initDriver ready");
  for (int i = 0; i < STRANDCNT; i++)
  {
    gpioSetup(STRANDS[i].gpioNum, OUTPUT, LOW);
  }
  ESP_LOGI(TAG, "gpioSetup ready");
  strand_t *MyStrand[] = {&STRANDS[0]};
  strand_t *pStrand = &STRANDS[0];
  int rc = digitalLeds_addStrands(MyStrand, STRANDCNT);
  bool toggle = false;
  if (rc)
  {
    ESP_LOGI(TAG, "digitalLeds_addStrands error code: %d. Halting", rc);
    while (true)
    {
      toggle = !toggle;
      gpio_set_level(GPIO_NUM_2, (uint32_t)toggle);
      vTaskDelay(100 / portTICK_RATE_MS);
    };
  }

  LEDData * NewLEDCmd=NULL;

  for(;;)
  {
    vTaskDelay(10 / portTICK_PERIOD_MS);
    if(xQueueReceive(led_cmd_queue, &NewLEDCmd, portMAX_DELAY))
    {
      // Bei meinen 5mm-LEDs sind die Farben rot-gruen vertauscht...
      //ESP_LOGI(TAG, "NewLEDCmd*:0x%X",(uint32_t)NewLEDCmd);
      if (NewLEDCmd!=NULL)
      {
        //ESP_LOGI(TAG, "Setup digital LEDs...");
        pixelColor_t Color = pixelFromRGB(NewLEDCmd->g, NewLEDCmd->r, NewLEDCmd->b);
        pStrand->pixels[NewLEDCmd->LEDNum]=Color;
        //ESP_LOGI(TAG, "Setting digital LEDs...");
        digitalLeds_drawPixels(MyStrand, STRANDCNT);
        //ESP_LOGI(TAG, "Setting done.");
        free(NewLEDCmd);
      }
    }
  }
  vTaskDelete(NULL);
}

void led_task_init(void)
{
  led_cmd_queue = xQueueCreate(10, sizeof(uint32_t));

  xTaskCreate(led_cmd_task, "led_cmd_task", 2048, NULL, 10, NULL);
}

// Farbei einer LED setzen
void SetLEDColor(uint8_t aLEDNum, uint8_t r, uint8_t g, uint8_t b)
{
  if (aLEDNum>1)
    return;
  LEDData * NewLEDCmd=(LEDData*)malloc(sizeof(LEDData));
  //SP_LOGI(TAG, "Alloc NewLEDCmd*:0x%X",(uint32_t)NewLEDCmd);
  NewLEDCmd->r=r;
  NewLEDCmd->g=g;
  NewLEDCmd->b=b;
  NewLEDCmd->LEDNum=aLEDNum;
  if (xQueueSend(led_cmd_queue,&NewLEDCmd,10/portTICK_PERIOD_MS)!=pdPASS)
    free(NewLEDCmd);
}


/**
 * @brief i2c master initialization
 */
esp_err_t i2c_master_init(void)
{
  // Erster I2C-Bus
  esp_err_t status=ESP_OK;
  int i2c_master_port = I2C_MASTER_NUM;
  i2c_config_t conf;
  conf.mode = I2C_MODE_MASTER;
  conf.sda_io_num = I2C_MASTER_SDA_IO;
  conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
  conf.scl_io_num = I2C_MASTER_SCL_IO;
  conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
  conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
  i2c_param_config(i2c_master_port, &conf);
  status = i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
  if (status!=ESP_OK)
    return status;

  // Zweiter I2C-Bus
  conf.mode = I2C_MODE_MASTER;
  conf.sda_io_num = 26;
  conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
  conf.scl_io_num = 27;
  conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
  conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
  i2c_param_config(1, &conf);
  status = i2c_driver_install(1, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
  return status;
}

void i2c_master_reset()
{
  i2c_reset_tx_fifo(I2C_MASTER_NUM);
  i2c_reset_rx_fifo(I2C_MASTER_NUM);
  periph_module_disable(PERIPH_I2C0_MODULE);
  periph_module_enable(PERIPH_I2C0_MODULE);
  i2c_driver_delete(I2C_MASTER_NUM);

  i2c_reset_tx_fifo(1);
  i2c_reset_rx_fifo(1);
  periph_module_disable(PERIPH_I2C0_MODULE);
  periph_module_enable(PERIPH_I2C0_MODULE);
  i2c_driver_delete(1);
  i2c_master_init();
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
    gpio_set_level(GPIO_NUM_2, 1);
    vTaskDelay(50 / portTICK_RATE_MS);
    SetLEDColor(1,0,0,0);
    gpio_set_level(GPIO_NUM_2, 0);
    vTaskDelay(50 / portTICK_RATE_MS);
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
