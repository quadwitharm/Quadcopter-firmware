#ifndef __B64_H__
#define __B64_H__

#include "clib.h"

void b64Encode(uint8_t *in, uint8_t *out, int len);
bool b64Decode(uint8_t *in, uint8_t *out, int len, int *outl);

int getB64EncodeLen(int len);
int getB64DecodeLen(int len);

#endif
