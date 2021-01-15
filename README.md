# Wetterstation
Diese Wetterstation hat vor allen Dingen den Zweck, verschiedene Technologien miteinander zu verknüpfen. Dabei steht der Spaß am Basteln, die Erweiterbarkeit und die Wartbarkeit im Vordergrund. 
Ansonsten kann man mit einer Wetterstation heutzutage niemanden mehr hinter'm Ofen hervorlocken, aber immerhin kann ich hierbei wesentlich bessere Sensoren verwenden, als gewöhnlich in bezahlbaren Wetterstationen verbaut sind.
# Prinzip
* Die eigentlichen Sensoren sind an einen ESP32 angeschlossen, der in C++ mit dem ESP-IDF programmiert wurde.
* Alle 5 Sekunden werden die Sensoren abgefragt und an einen MQTT-Server (Raspi-Zero) weitergeleitet.
* Das WLAN des ESP kann über Bluetooth eingerichtet werden.
* Der MQTT-Server läuft auf einem Raspberry Pi Zero, an das ein 264X176 Pixel ePaper-Display angeschlossen ist.
* Ein Python-Skript (auf dem Pi Zero) speichert neue Messdaten aus dem MQTT-Server in eine RRD-Datenbank (rrdtool).
* Ein weiteres Python-Skript erzeugt jede Minute eine neue Grafik aus der RRD-DB und den aktuellen Messwerten. Diese Daten werden zusammen in einem PNG-Bild verarbeitet und auf dem ePaper-Display angezeigt.
* Die unterschiedlichen Kurven der Messadaten (Temperatur, Luftfeuchtigkeit usw.) können per Knopfdruck (im Gehäuse des Raspi-Zero integriert) ausgewählt werden.
* Neue Sensoren (Windstärke, Regenmenge und Windrichtung) sind in Vorbereitung und können problemlos dem MQTT-Server hinzugefügt werden.
* Ein akkubetriebenes Anzeigemodul (STM32 mit LoRaWAN und transflektivem LCD-Display) ist in Planung. Hier steht eine stromsparende Technik im Vordergrund. Der Akku sollte schon ein Jahr lang durchhalten.
# Verwendete Sensoren 
| Sensor        | Aufgabe       | Anzahl|
| ------------- |---------------| -----:|
| SHT35 | Temperatur und Luftfeuchte | 1 |
| VEML7700 | LUX | 1 |
| BMP280 | Luftdruck und Temperatur | 1 |
