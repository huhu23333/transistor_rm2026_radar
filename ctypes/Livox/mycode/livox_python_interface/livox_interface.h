// livox_interface.h
#ifndef LIVOX_INTERFACE_H
#define LIVOX_INTERFACE_H

#include <stdint.h>
#include <stdbool.h>

#ifdef _WIN32
    #define EXPORT_API __declspec(dllexport)
#else
    #define EXPORT_API __attribute__((visibility("default")))
#endif

#define PYIF_PonitDataArrayLen 4096

typedef struct {
    int32_t x;
    int32_t y;
    int32_t z;
    uint8_t reflectivity;
    uint8_t tag;
} PYIF_PonitData;

// 导出函数
EXPORT_API int pyif_Init();
EXPORT_API void pyif_Uninit();
EXPORT_API void pyif_draw2dImageF(double* yaws, double* pitchs, double* values,
                                  uint64_t point_number, uint16_t imageSize,
                                  uint16_t values_number, double* result, uint8_t* mask);

// 导出变量
extern EXPORT_API PYIF_PonitData pyif_ponitDataArray[2][PYIF_PonitDataArrayLen];
extern EXPORT_API bool pyif_usedFlag_0;
extern EXPORT_API bool pyif_usedFlag_1;
extern EXPORT_API uint64_t pyif_writeCount;

#ifndef max
#define max(a, b) (((a) > (b)) ? (a) : (b))
#endif

#ifndef min
#define min(a, b) (((a) < (b)) ? (a) : (b))
#endif

#endif
