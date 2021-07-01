/*
 * ltrt.h
 *
 *  Created on: 2018Äê3ÔÂ15ÈÕ
 *      Author: donghao
 */

#include "../setup.h"

#ifndef LTRT_H_
#define LTRT_H_

//#define DC_VOLTAGE_MAX 120.0f
//#define AC_VOLTAGE_MAX 60.0f
//#define IF_CURRENT_MAX 10.0f
//#define IC_CURRENT_MAX 12.0f
#if FAKE_GRID
    #define DC_VOLTAGE_MAX 360.0f
    #define AC_VOLTAGE_MAX 20.0f
    #define IF_CURRENT_MAX 12.0f
    #define IC_CURRENT_MAX 10.0f
#else
    #define DC_VOLTAGE_MAX 395.0f
    #define AC_VOLTAGE_MAX 380.0f
    #define IF_CURRENT_MAX 15.0f
    #define IC_CURRENT_MAX 15.0f
#endif

// Declaration of Long Time Running Test
#define LONG_TIME_RUNNING_TEST 0

#if LONG_TIME_RUNNING_TEST

#define DC_VOLTAGE_CHECK_MAX 375.0f
#define AC_VOLTAGE_CHECK_MAX 345.0f
#define IF_CURRENT_CHECK_MAX 16.0f
#define IC_CURRENT_CHECK_MAX 16.0f

void MeasureCheck(void) ;

#endif

#endif /* LTRT_H_ */
