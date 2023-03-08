# Batteriebetriebener LoRa-Funksensor
Das ESP32-Modul (LILYGO TTGO LoRa32 V2.1 _ 1.6) hat ein integriertes LoRa-Modul und nimmt im Deep-Sleep 150 Mikroampere auf. 
Das Modul hat einen recht sparsamen integrierten Step-Down-Wandler und einen Ladechip für Li-Ion-Akkus. Angeschlossen wird daran ein 18650-Akku.

Alle 30-60 s wird die Temperatur und Luftfeuchtigkeit mit einem SHT45-Sensor gemessen und dann in einer CBOR-Datei über LoRa an den Raspberry versendet.

Zur Programmierung wurde das ESP-IDF 5.0 verwendet mit dem Espressif-Plugin für Visual Studio Code
