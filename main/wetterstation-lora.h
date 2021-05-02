#ifndef WETTERSTATION_LORA_H
#define WETTERSTATION_LORA_H

extern "C"
{
#include "../components/lora/include/lora.h"
}

// Kennung für einen gültigen Paketheader
#define PACKET_MAGIC 0x2008

// Lora-Paketheader. Ein Einzelpaket darf incl. Payload nicht größer als 255 bytes werden (max. Lora-Paketgröße)
// Größe 10 Byte
struct LoraPacketHeader
{
  // Erkennungswert für Datenpaket
  const uint16_t Magic = PACKET_MAGIC;
  // Gesamtgröße einer Lora-Kommunikation. Diese kann mehrere Pakete beinhalten und größer sein,
  // als die max. Lora-Paketgröße 
  uint16_t TotalPacketSize;
  // Aktuelle Paketnummer (von 0..NumPackets-1)
  uint8_t PacketNumber;
  // Anzahl Gesamtpakete der Sendung
  uint8_t NumPackets;
  // Aktuelle Payloadgröße. Darf incl. Headersize nicht 255 überschreiten!
  uint8_t PacketPayloadSize;
  uint8_t reserved;

  // CRC16 über den reinen Payload (nicht Header)
  uint16_t PayloadCRC;
} __attribute__((packed));

// Nachricht über LoRa senden. Wird evtl in mehrere Pakete aufgeteilt
esp_err_t SendLoraMsg(uint8_t* aBuf, uint16_t aSize);

void task_tx(void *p);
#endif