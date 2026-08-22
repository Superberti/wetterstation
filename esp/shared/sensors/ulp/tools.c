#include <stdint.h>
#include "ulp_riscv_utils.h"

void delay(uint32_t cycles)
{
  uint32_t mark = ULP_RISCV_GET_CCOUNT();
  uint32_t interval;
  do
  {
    interval = ULP_RISCV_GET_CCOUNT() - mark; // underflow is well defined
  } while (interval < cycles);
}

//int32_t MAX(int32_t a, int32_t b) { return ((a) > (b) ? a : b); }
//int32_t MIN(int32_t a, int32_t b) { return ((a) < (b) ? a : b); }