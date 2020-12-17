#ifndef _CUSTOM_MEMORY_H_
#define _CUSTOM_MEMORY_H_
#include <iostream>

#define ALIGN_BYTES 64 // AVX, AVX2 (256 bits = 32 Bytes), SSE4.2 (128 bits = 16 Bytes)
/** \internal Like malloc, but the returned pointer is guaranteed to be 32-byte aligned.
* Fast, but wastes 32 additional bytes of memory. Does not throw any exception.
* (512 bits) two LSB addresses of 64 bytes-aligned : 00, 40, 80, C0
* (256 bits) two LSB addresses of 32 bytes-aligned : 00, 20, 40, 60, 80, A0, C0, E0
* (128 bits) two LSB addresses of 16 bytes-aligned : 00, 10, 20, 30, 40, 50, 60, 70, 80, 90, A0, B0, C0, D0, E0, F0
*/
// NOT USED!!!! in Linux, 'aligned_alloc' can be used.
inline void* custom_aligned_malloc(std::size_t size)
{
    void* original = std::malloc(size + ALIGN_BYTES); //size+ALIGN_BYTESžžÅ­ ÇÒŽçÇÏ°í,
    if (original == 0) return nullptr; // if allocation is failed, return nullptr;
    void* aligned = reinterpret_cast<void*>( (reinterpret_cast<std::size_t>(original) & ~(std::size_t(ALIGN_BYTES - 1))) + ALIGN_BYTES);
    *(reinterpret_cast<void**>(aligned) - 1) = original;
    return aligned;
};

/** \internal Frees memory allocated with handmade_aligned_malloc */
inline void custom_aligned_free(void *ptr)
{
    if (ptr) std::free(*(reinterpret_cast<void**>(ptr) - 1));
};

#endif