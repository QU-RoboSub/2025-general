/* Copyright 2023 The MathWorks, Inc. */
#ifndef EXAMPLE_H
#define EXAMPLE_H
#if !( defined(MATLAB_MEX_FILE) || defined(RSIM_PARAMETER_LOADING) ||  defined(RSIM_WITH_SL_SOLVER))
#include "rtwtypes.h"
#ifdef __cplusplus
extern "C" {
    #endif
    void stepFunctionBR_Ping(int8_T * Distance,int size_vector_1);
    void setupFunctionBR_Ping(uint8_T  arduinoRxPin,int size_vector__1,uint8_T  arduinoTxPin,int size_vector__2,uint8_T  ledPin,int size_vector__3);
    #ifdef __cplusplus
}
#endif
#else
#define loop(void) (0)
#define setup(void) (0)
#endif
#endif