#!/bin/bash
# rrd Datenbank für die Wetterstation anlegen
# Die Messwerte kommen ca. alle 10 s rein
# Alle 5 Minuten einen Mittelwert (60 s)  
# Jede Minute einen Minimumswert für 90 Tage
# Jede Minute einen Maximumswert für 90 Tage
# Jede Minute einen Wert, 90 Tage lang
# dann Jede Stunde für 18 Monate
# dann einen Tageswert für 10 Jahre

rrdtool create weather.rrd --step 60s \
DS:temps1:GAUGE:15m:-40:60 \
DS:temps2:GAUGE:15m:-40:60 \
DS:temps3:GAUGE:15m:-40:60 \
DS:temps4:GAUGE:15m:-40:60 \
DS:hums1:GAUGE:15m:0:100 \
DS:hums2:GAUGE:15m:0:100 \
DS:hums3:GAUGE:15m:0:100 \
DS:hums4:GAUGE:15m:0:100 \
DS:press1:GAUGE:15m:900:1100 \
DS:press2:GAUGE:15m:900:1100 \
DS:windspeed:GAUGE:15m:0:200 \
DS:winddir:GAUGE:15m:0:360 \
DS:rain:DERIVE:15m:0:200 \
DS:brightness:GAUGE:15m:0:200000 \
RRA:AVERAGE:0.5:1m:90d \
RRA:MIN:0.5:1m:90d \
RRA:MAX:0.5:1m:90d \
RRA:AVERAGE:0.5:1h:18M \
RRA:AVERAGE:0.5:1d:10y \
