#ifndef LORASTRUCTS_H
#define LORASTRUCTS_H

#include <stdint.h>

enum LoRaBoardTypes
{
  LilygoT3,
  HeltecESPLoRa,
  HeltecWirelessStick_V3,
  DevKitC_V4
};

// Kennung für einen gültigen Paketheader
#define PACKET_MAGIC 0x2008

enum LoraCommand
{
  // Bestätigungskommando (ohne Payload)
  CMD_ACK,
  // Kommando nicht bestätigen (ohne Payload)
  CMD_NACK,
  // Payload ist CBOR-encoded
  CMD_CBORDATA,
  // Payload ist ein Textlog
  CMD_LOG,
  // Payload wurde außerhalb der Reihe gesendet und
  // stellt ein singuläres Ereignis dar (z.B. ein Blitz)
  CMD_INT,
};

// Lora-Paketheader. Ein Einzelpaket darf incl. Payload nicht größer als 255 bytes werden (max. Lora-Paketgröße)
// Größe 12 Byte
struct LoraPacketHeader
{
  // Erkennungswert für Datenpaket
  const uint16_t Magic = PACKET_MAGIC;
  // Adresse bei mehreren Teilnehmern
  uint16_t Address;
  // Gesamtgröße einer Lora-Kommunikation. Diese kann mehrere Pakete beinhalten und größer sein,
  // als die max. Lora-Paketgröße. Nur totaler Payload ohne Header!
  uint16_t TotalTransmissionSize;
  // Aktuelle Paketnummer (von 0..NumPackets-1)
  uint8_t PacketNumber;
  // Anzahl Gesamtpakete der Sendung
  uint8_t NumPackets;
  // Aktuelle Payloadgröße. Darf incl. Headersize nicht 255 überschreiten!
  uint8_t PacketPayloadSize;
  // Lora-Kommando. Es können nicht nur CBOR-Daten übertragen werden, sondern auch Kommandos von und zu beiden
  // Seiten
  uint8_t Cmd;
  // Zur freien Verwendung
  uint32_t Tag; 

  // CRC16 über den reinen Payload (nicht Header)
  uint16_t PayloadCRC;
} __attribute__((packed));

// Temperatur in Grad Celsius
#define TEMP_TAG "TE"
// Luftfeuchtigkeit in Prozent
#define HUM_TAG "LF"
// Luftdruck in mbar
#define PRESS_TAG "LD"
// Beleuchtungsstärke in Lux
#define ILLU_TAG "BS"
// Lüfterumdrehungen in UpM
#define COOL_TAG "LU"
// Ort
#define POS_TAG "Ort"
// Wert
#define VAL_TAG "W"
// Batteriespannung
#define VOL_TAG "V"
// Paket-Zähler
#define PC_TAG "PC"
// Alle Daten
#define DATA_TAG "DATA"
// Watt (Solar)
#define POWER_TAG "P"
// Blitze pro Stunde
#define FLASH_TAG "BL"
// Windgeschwindigkeit in m/s
#define WINDSPEED_TAG "WG"
// Windrichtung in Grad
#define WINDDIR_TAG "WR"
// Tagesregenmenge in Liter pro Quadratmeter und Tag
#define RAIN_TAG "RM"
// Feuchtesensor spricht an (0/1)
#define HUM_DET_TAG "FD"
// Sensorfehlerzähler
#define SENS_ERR_TAG "SFZ"

// Orte
#define ORT_SCHUPPEN_SCHATTEN "SCHU_SCHA"
#define ORT_SCHUPPEN_SONNE "SCHU_SO"
#define ORT_SCHUPPEN_INNEN "SCHU_INN"
#define ORT_CARPORT "CARPORT"
#define ORT_GARTENHAUS "GARTEN"
#define ORT_GEWAECHSHAUS "gwhs"
#define ORT_ARBEITSZIMMER "ARBEIT"
#define ORT_WOHNZIMMER "WOHN"
#define ORT_KUECHE "KUECHE"
#define ORT_SCHLAFZIMMER "SCHLAF"
#define ORT_DACHBODEN "DACH"
#define ORT_HEIZUNG "HEIZUNG"
#define ORT_KELLER "KELLER"
#define ORT_BAD "BAD"
#define ORT_TERRASSE "TERRASSE"
#define ORT_CHILLECKE "CHILL"
#define ORT_SYMPATEC "SYMPATEC"

// Adressen der einzelnen Lora-Module
#define LORA_ADDR_RASPI 0				// Raspi-Zentrale
#define LORA_ADDR_GWHS 1				// Gewächshaus (Temperatur)
#define LORA_ADDR_CARPORT 2			// Carport-Wetterstation
#define LORA_ADDR_BIKEPORT 3		// Fahrradschuppen
#define LORA_ADDR_SYMPATEC 4    // Sympatec in Clausthal
#define LORA_ADDR_SYMPATEC_2 5  // Sympatec in Clausthal, Sensor 2


#endif