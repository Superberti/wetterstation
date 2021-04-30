#ifndef CBOR_TOOLS_H
#define CBOR_TOOLS_H

extern "C"
{
#include "../components/cbor/cbor.h"
}

CborError example_dump_cbor_buffer(CborValue *it, int nestingLevel);
/// HexDump eines Speicherbereiches auf stdout ausgeben
/// \param Buffer Pointer auf Speicher
/// \param dwBytes Größe des Speicherbereiches
/// \param offset Offset zum Pointer
void HexDump(unsigned char *Buffer, const unsigned int dwBytes, const unsigned int offset = 0);
void indent(int nestingLevel);

// Kennung für einen gültigen Paketheader
#define PACKET_MAGIC 0x2008

// Lora-Paketheader. Ein Einzelpaket darf incl. Payload nicht größer als 255 bytes werden (max. Lora-Paketgröße)
// Größe 96 Bit=12 Byte
struct LoraPacketHeader
{
  uint16_t Magic = PACKET_MAGIC;
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

  // CRC32 über den reinen Payload (nicht Header)
  uint32_t PayloadCRC;
} __attribute__((packed));

#endif