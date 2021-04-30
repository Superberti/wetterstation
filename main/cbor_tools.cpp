#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include <string.h>
#include "esp_log.h"
#include "cJSON.h"
#include <string>
#include "esp_log.h"

#include "cbor_tools.h"

static const char *TAG = "JSON";

#define CBOR_CHECK(a, str, goto_tag, ret_value, ...)                        \
  do                                                                        \
  {                                                                         \
    if ((a) != CborNoError)                                                 \
    {                                                                       \
      ESP_LOGE(TAG, "%s(%d): " str, __FUNCTION__, __LINE__, ##__VA_ARGS__); \
      ret = ret_value;                                                      \
      goto goto_tag;                                                        \
    }                                                                       \
  } while (0)

void indent(int nestingLevel)
{
  while (nestingLevel--)
  {
    printf("  ");
  }
}

/**
 * Decode CBOR data manuallly
 */
CborError example_dump_cbor_buffer(CborValue *it, int nestingLevel)
{
  CborError ret = CborNoError;
  while (!cbor_value_at_end(it))
  {
    CborType type = cbor_value_get_type(it);

    indent(nestingLevel);
    switch (type)
    {
    case CborArrayType:
    {
      CborValue recursed;
      assert(cbor_value_is_container(it));
      puts("Array[");
      ret = cbor_value_enter_container(it, &recursed);
      CBOR_CHECK(ret, "enter container failed", err, ret);
      ret = example_dump_cbor_buffer(&recursed, nestingLevel + 1);
      CBOR_CHECK(ret, "recursive dump failed", err, ret);
      ret = cbor_value_leave_container(it, &recursed);
      CBOR_CHECK(ret, "leave container failed", err, ret);
      indent(nestingLevel);
      puts("]");
      continue;
    }
    case CborMapType:
    {
      CborValue recursed;
      assert(cbor_value_is_container(it));
      puts("Map{");
      ret = cbor_value_enter_container(it, &recursed);
      CBOR_CHECK(ret, "enter container failed", err, ret);
      ret = example_dump_cbor_buffer(&recursed, nestingLevel + 1);
      CBOR_CHECK(ret, "recursive dump failed", err, ret);
      ret = cbor_value_leave_container(it, &recursed);
      CBOR_CHECK(ret, "leave container failed", err, ret);
      indent(nestingLevel);
      puts("}");
      continue;
    }
    case CborIntegerType:
    {
      int64_t val;
      ret = cbor_value_get_int64(it, &val);
      CBOR_CHECK(ret, "parse int64 failed", err, ret);
      printf("%lld\n", (long long)val);
      break;
    }
    case CborByteStringType:
    {
      uint8_t *buf;
      size_t n;
      ret = cbor_value_dup_byte_string(it, &buf, &n, it);
      CBOR_CHECK(ret, "parse byte string failed", err, ret);
      HexDump(buf, n);
      puts("");
      free(buf);
      continue;
    }
    case CborTextStringType:
    {
      char *buf;
      size_t n;
      ret = cbor_value_dup_text_string(it, &buf, &n, it);
      CBOR_CHECK(ret, "parse text string failed", err, ret);
      puts(buf);
      free(buf);
      continue;
    }
    case CborTagType:
    {
      CborTag tag;
      ret = cbor_value_get_tag(it, &tag);
      CBOR_CHECK(ret, "parse tag failed", err, ret);
      printf("Tag(%lld)\n", (long long)tag);
      break;
    }
    case CborSimpleType:
    {
      uint8_t type;
      ret = cbor_value_get_simple_type(it, &type);
      CBOR_CHECK(ret, "parse simple type failed", err, ret);
      printf("simple(%u)\n", type);
      break;
    }
    case CborNullType:
      puts("null");
      break;
    case CborUndefinedType:
      puts("undefined");
      break;
    case CborBooleanType:
    {
      bool val;
      ret = cbor_value_get_boolean(it, &val);
      CBOR_CHECK(ret, "parse boolean type failed", err, ret);
      puts(val ? "true" : "false");
      break;
    }
    case CborHalfFloatType:
    {
      uint16_t val;
      ret = cbor_value_get_half_float(it, &val);
      CBOR_CHECK(ret, "parse half float type failed", err, ret);
      printf("__f16(%04x)\n", val);
      break;
    }
    case CborFloatType:
    {
      float val;
      ret = cbor_value_get_float(it, &val);
      CBOR_CHECK(ret, "parse float type failed", err, ret);
      printf("%g\n", val);
      break;
    }
    case CborDoubleType:
    {
      double val;
      ret = cbor_value_get_double(it, &val);
      CBOR_CHECK(ret, "parse double float type failed", err, ret);
      printf("%g\n", val);
      break;
    }
    case CborInvalidType:
    {
      ret = CborErrorUnknownType;
      CBOR_CHECK(ret, "unknown cbor type", err, ret);
      break;
    }
    }

    ret = cbor_value_advance_fixed(it);
    CBOR_CHECK(ret, "fix value failed", err, ret);
  }
  return CborNoError;
err:
  return ret;
}

void HexDump(unsigned char *Buffer, const unsigned int dwBytes, const unsigned int offset)
{
  std::string Dump, AsciiDump;
  char cell[10];
  const unsigned int BytesPerLine = 16;
  for (unsigned int i = 0; i < dwBytes; i++)
  {
    sprintf(cell, "%02X ", Buffer[i]);
    Dump += std::string(cell);
    AsciiDump += Buffer[i] >= 0x20 ? char(Buffer[i]) : '.';
    if ((i && !((i + 1) % BytesPerLine)) || (i == dwBytes - 1))
    {
      if (i == dwBytes - 1 && dwBytes % BytesPerLine)
      {
        // Letzte, unvollständige Zeile
        // Spaces einfügen
        Dump += std::string(BytesPerLine * 3 - Dump.size(), ' ');
        printf("%04Xh:%s | %s\n", i / BytesPerLine * BytesPerLine + offset, Dump.c_str(), AsciiDump.c_str());
      }
      else
      {
        // Zeile rausschreiben
        printf("%04Xh:%s | %s\n", (i - 1) / BytesPerLine * BytesPerLine + offset, Dump.c_str(), AsciiDump.c_str());
        Dump = "";
        AsciiDump = "";
      }
    }
  }
}
