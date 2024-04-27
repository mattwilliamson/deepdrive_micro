#include "memory_usage.h"

uint32_t get_total_heap(void) {
  extern char __StackLimit, __bss_end__;

  return &__StackLimit - &__bss_end__;
}

uint32_t get_used_heap(void) {
  struct mallinfo m = mallinfo();
  return m.uordblks;
}

uint32_t get_free_heap(void) {
  return get_total_heap() - get_used_heap();
}