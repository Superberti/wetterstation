#ifndef CBOR_TOOLS_H
#define CBOR_TOOLS_H

extern "C"
{
#include "../components/cbor/cbor.h"
}

CborError example_dump_cbor_buffer(CborValue *it, int nestingLevel);

void indent(int nestingLevel);


#endif