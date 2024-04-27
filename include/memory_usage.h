#ifndef MEMORY_USAGE_H
#define MEMORY_USAGE_H

#include <malloc.h>
#include <stdint.h>

/**
 * @brief Get the total heap size.
 * 
 * @return The total heap size in bytes.
 */
uint32_t get_total_heap(void);

/**
 * @brief Get the used heap size.
 * 
 * @return The used heap size in bytes.
 */
uint32_t get_used_heap(void);

/**
 * @brief Get the free heap size.
 * 
 * @return The free heap size in bytes.
 */
uint32_t get_free_heap(void);

#endif // MEMORY_USAGE_H