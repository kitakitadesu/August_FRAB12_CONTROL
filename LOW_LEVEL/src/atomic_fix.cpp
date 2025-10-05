// Atomic operations fix for micro-ROS on RP2040
// Provides missing __atomic_test_and_set implementation

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// RP2040 hardware spinlock base address
#define SIO_BASE 0xd0000000
#define SIO_SPINLOCK0 (SIO_BASE + 0x100)

// Use RP2040 hardware spinlock for atomic operations
// Spinlock 31 is typically reserved for SDK use, so we use spinlock 30
#define ATOMIC_SPINLOCK_NUM 30

bool __atomic_test_and_set(volatile void *ptr, int memorder) {
    volatile uint8_t *flag = (volatile uint8_t *)ptr;
    volatile uint32_t *spinlock = (volatile uint32_t *)(SIO_SPINLOCK0 + (ATOMIC_SPINLOCK_NUM * 4));
    
    // Acquire hardware spinlock
    while (!*spinlock) {
        __asm__ __volatile__("" ::: "memory");
    }
    
    // Read the current value
    uint8_t result = *flag;
    
    // Set to 1
    *flag = 1;
    
    // Release hardware spinlock (write any value)
    *spinlock = 1;
    
    return result != 0;
}

#ifdef __cplusplus
}
#endif
