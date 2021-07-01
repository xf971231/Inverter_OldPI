#ifndef SENSOR_H
#define SENSOR_H

// #define DC_VOLTAGE_MUL -54.0f
// #define AC_VOLTAGE_MUL -54.0f

// Attention: The resistor of current hall sensor is two 300 ohm in parallel
// Not the same as the schematic of two 400 ohm in parallel
// #define IF_CURRENT_MUL -4.0f // IO, J101, J102
// #define IC_CURRENT_MUL -6.666667f // IL, J201, J201

#define DC_VOLTAGE_MUL (-0.016479504f)
#define AC_VOLTAGE_MUL (-0.016479504f)

// Attention: The resistor of current hall sensor is two 300 ohm in parallel
// Not the same as the schematic of two 400 ohm in parallel
#define IG_CURRENT_MUL (-0.001220704f) // IO, J101, J102
#define IC_CURRENT_MUL (-0.002034506f) // IL, J201, J201

#define CH_DC_BUS 0
#define CH_AC_VOLTAGE 1
#define CH_GRID_CURRENT 2
#define CH_CAP_CURRENT 3

// Real Value = Sampled Value + Offset
// Offset should be set manually
// First done by D.H. 2018.10.23
#define DC_MEASURE_OFFSET -1.82f
#define AC_MEASURE_OFFSET -2.75f
#define IG_MEASURE_OFFSET 0.07f // 0.0245f
#define IC_MEASURE_OFFSET 0.218f // 0.020f

extern float MeasureBuf[8];
extern float MeasureBuf_prev[8];

#endif
