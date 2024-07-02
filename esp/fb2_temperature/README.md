# Firebeetle 2 (ESP32-C6) Temperatur- Druck- und Luftfeuchtigkeitslogger (Akkubetrieb)

## Verwendete Sensoren:
- BMP390 (Luftdruck)
- SHT40 (Temperatur und Luftfeuchtigkeit)

## LoRa-Modul
SX1278 bei 433 MHz

## Hinweise zum Compilieren
- Verwendung vom ESP-IDF 5.2.2 mit VSCode

## Hinweise zum Flashen
- Wenn die MCU zu schnell in den Sleep-Modus geht, dann kann kein COM-Port zum Flashen angelegt werden. Beim Firebeetle reicht es NICHT aus, Reset+Boot zu drücken und dann nur Reset loszulassen (wie es auf den ESP32-S3-Lilygo-Board funktioniert). Hier muss die Stromversorgung getrennt werden (also USB abziehen), Boot-Knopf drücken und halten, USB-Stecker rein. Danach befindet der Firebeetle sich im flashbaren Bootmodus.