#ifndef LORASTRUCTS_H
#define LORASTRUCTS_H

#include <stdint.h>

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

#define TEMP_TAG "TE"
#define HUM_TAG "LF"
#define PRESS_TAG "LD"
#define ILLU_TAG "BS"
#define COOL_TAG "LU"
#define POS_TAG "Ort"
#define VAL_TAG "W"
// Paket-Zähler
#define PC_TAG "PC"

#define ORT_SCHUPPEN_SCHATTEN "A"
#define ORT_SCHUPPEN_SONNE "B"
#define ORT_SCHUPPEN_INNEN "C"
#define ORT_CARPORT "D"
#define ORT_GARTENHAUS "E"
#define ORT_GEWAECHSHAUS "F"


#endif