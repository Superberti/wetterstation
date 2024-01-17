# Hauptwetterstation
Die Hauptwetterstation. Verwendet wird ein ESP32-DevKit-V4-Board
mit 433 MHz-SX1278-LoRa-Modul.

Angeschlossene Sensorik:
BMP390 - Luftdruck
SHT40 - Temperatur und Luftfeuchte
Davis Anemometer (Windgeschwindigkeit und Winrichtung) am 12-Bit-ADC (ADS1015)
Luxmeter (bis 200000 lx) am ADS1015
Davis Niederschlagsmesser (Regenwippe)
Gewittersensor AS3935
Sonstiges:
Display: Nokia 5110-Display (Philips PCD8544)
Zur Programmierung wurde das ESP-IDF 5.1 verwendet mit dem Espressif-Plugin für Visual Studio Code
