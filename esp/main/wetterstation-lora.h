#ifndef WETTERSTATION_LORA_H
#define WETTERSTATION_LORA_H

#include "../components/lora/include/lora.h"

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

// Nachricht über LoRa senden. Wird evtl in mehrere Pakete aufgeteilt
esp_err_t SendLoraMsg(SX1278_LoRa & aLoRa, uint8_t* aBuf, uint16_t aSize, uint32_t aTag);
void task_tx(void *p);
void error(const char *format, ...);
void ParseLoraPacket(uint8_t *buf, uint8_t len);
void InitLora();
int64_t GetTime_us();
void InitSSD1306();

#endif