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
* Batteriebetriebene Sensoren (z.B. im Gewächshaus) werden mit LoRa-Funkmodulen angebunden und senden nur alle 30-60 s ihre Werte an den Raspberry (mit LoRa-Modul). Die Sensoren halten mit einem 18650-Akku mindestens zwei Jahre durch. Die neueste Generation der ESP32-S3-LoRa-Module halten sogar (theoretisch) über 10 Jahre damit durch.
* Ein akkubetriebenes Anzeigemodul (STM32 mit LoRa und transflektivem LCD-Display) ist in Planung. Hier steht eine stromsparende Technik im Vordergrund. Der Akku sollte schon ein Jahr lang durchhalten.
# Verwendete Sensoren 
| Sensor        | Aufgabe       | Anzahl|
| ------------- |---------------| -----:|
| SHT35 | Temperatur und Luftfeuchte | 1 |
| VEML7700 | LUX | 1 |
| BMP280 | Luftdruck und Temperatur | 1 |
# Installation
Auf dem Raspberry sind folgende Befehle auszuführen:
    sudo raspi-config nonint do_spi 0
    sudo apt-get install python3-dev
Für die Python3-Skripte sind folgende Pakete zu installieren:
    sudo pip3 install paho-mqtt RPi.GPIO spidev pyLoRa cbor2
Das LoRa-Modul wird an folgende Pins des Raspberrys angeschlossen:
|LoRa-Modul|Raspberry|
|----------|--------|
|RST|GPIO25|
|CS/NSS|CE1|
|DI/MOSI|MOSI|
|DO/MISO|MISO|
|DIO0|GPIO22|
|DIO1|GPIO23|
|DIO2|GPIO24|

Das ist die Pinbelegung des Adafruits-Funk-Bonnets, die man auch nicht ändern kann. Beim Einsatz von Ra02-Modulen können auch DIO1 und DIO2 weggelassen werden.
