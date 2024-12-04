#include "rtw_capi.h"
#ifdef HOST_CAPI_BUILD
#include "MathematicalModel_capi_host.h"
#define sizeof(...) ((size_t)(0xFFFF))
#undef rt_offsetof
#define rt_offsetof(s,el) ((uint16_T)(0xFFFF))
#define TARGET_CONST
#define TARGET_STRING(s) (s)
#ifndef SS_UINT64
#define SS_UINT64 19
#endif
#ifndef SS_INT64
#define SS_INT64 20
#endif
#else
#include "builtin_typeid_types.h"
#include "MathematicalModel.h"
#include "MathematicalModel_capi.h"
#include "MathematicalModel_private.h"
#ifdef LIGHT_WEIGHT_CAPI
#define TARGET_CONST
#define TARGET_STRING(s)               ((NULL))
#else
#define TARGET_CONST                   const
#define TARGET_STRING(s)               (s)
#endif
#endif
static const rtwCAPI_Signals rtBlockSignals [ ] = { { 0 , 0 , TARGET_STRING ( "MathematicalModel/Gain" ) , TARGET_STRING ( "" ) , 0 , 0 , 0 , 0 , 0 } , { 1 , 0 , TARGET_STRING ( "MathematicalModel/Gain1" ) , TARGET_STRING ( "Thrust Force" ) , 0 , 0 , 0 , 0 , 1 } , { 2 , 0 , TARGET_STRING ( "MathematicalModel/m to mm" ) , TARGET_STRING ( "" ) , 0 , 0 , 0 , 0 , 2 } , { 3 , 0 , TARGET_STRING ( "MathematicalModel/1-D Lookup Table1" ) , TARGET_STRING ( "" ) , 0 , 0 , 0 , 0 , 0 } , { 4 , 0 , TARGET_STRING ( "MathematicalModel/1-D Lookup Table2" ) , TARGET_STRING ( "" ) , 0 , 0 , 0 , 0 , 0 } , { 5 , 0 , TARGET_STRING ( "MathematicalModel/Depth Limit" ) , TARGET_STRING ( "Simulation" ) , 0 , 0 , 0 , 0 , 2 } , { 6 , 0 , TARGET_STRING ( "MathematicalModel/Desired Depth (mm)" ) , TARGET_STRING ( "Reference" ) , 0 , 0 , 0 , 0 , 3 } , { 7 , 0 , TARGET_STRING ( "MathematicalModel/Sum" ) , TARGET_STRING ( "" ) , 0 , 0 , 0 , 0 , 2 } , { 8 , 0 , TARGET_STRING ( "MathematicalModel/Sum2" ) , TARGET_STRING ( "Resultant Force" ) , 0 , 0 , 0 , 0 , 2 } , { 9 , 0 , TARGET_STRING ( "MathematicalModel/Sum3" ) , TARGET_STRING ( "Noisy Readings" ) , 0 , 0 , 0 , 0 , 2 } , { 10 , 0 , TARGET_STRING ( "MathematicalModel/Sum4" ) , TARGET_STRING ( "Total Disturbance" ) , 0 , 0 , 0 , 0 , 2 } , { 11 , 0 , TARGET_STRING ( "MathematicalModel/Sensor Noise" ) , TARGET_STRING ( "" ) , 0 , 0 , 0 , 0 , 4 } , { 12 , 2 , TARGET_STRING ( "MathematicalModel/MATLAB System" ) , TARGET_STRING ( "Physical" ) , 0 , 1 , 0 , 0 , 2 } , { 13 , 2 , TARGET_STRING ( "MathematicalModel/MATLAB System" ) , TARGET_STRING ( "" ) , 1 , 2 , 0 , 0 , 2 } , { 14 , 0 , TARGET_STRING ( "MathematicalModel/Transfer Fcn (with initial outputs)/State Space" ) , TARGET_STRING ( "" ) , 0 , 0 , 0 , 0 , 2 } , { 15 , 0 , TARGET_STRING ( "MathematicalModel/Ultrasonic Sensor/State Space" ) , TARGET_STRING ( "" ) , 0 , 0 , 0 , 0 , 2 } , { 16 , 0 , TARGET_STRING ( "MathematicalModel/Depth Control/I Gain/Internal Parameters/Integral Gain" ) , TARGET_STRING ( "" ) , 0 , 0 , 0 , 0 , 1 } , { 17 , 0 , TARGET_STRING ( "MathematicalModel/Depth Control/N Gain/Internal Parameters/Filter Coefficient" ) , TARGET_STRING ( "" ) , 0 , 0 , 0 , 0 , 1 } , { 18 , 0 , TARGET_STRING ( "MathematicalModel/PID Controller2/D Gain/Internal Parameters/Derivative Gain" ) , TARGET_STRING ( "" ) , 0 , 0 , 0 , 0 , 3 } , { 19 , 0 , TARGET_STRING ( "MathematicalModel/PID Controller2/Filter/Cont. Filter/Filter" ) , TARGET_STRING ( "" ) , 0 , 0 , 0 , 0 , 2 } , { 20 , 0 , TARGET_STRING ( "MathematicalModel/PID Controller2/Filter/Cont. Filter/SumD" ) , TARGET_STRING ( "" ) , 0 , 0 , 0 , 0 , 2 } , { 21 , 0 , TARGET_STRING ( "MathematicalModel/PID Controller2/I Gain/Internal Parameters/Integral Gain" ) , TARGET_STRING ( "" ) , 0 , 0 , 0 , 0 , 3 } , { 22 , 0 , TARGET_STRING ( "MathematicalModel/PID Controller2/N Gain/Internal Parameters/Filter Coefficient" ) , TARGET_STRING ( "" ) , 0 , 0 , 0 , 0 , 2 } , { 23 , 0 , TARGET_STRING ( "MathematicalModel/Depth Control/Saturation/External/Saturation Dynamic/Switch2" ) , TARGET_STRING ( "" ) , 0 , 0 , 0 , 0 , 1 } , { 0 , 0 , ( NULL ) , ( NULL ) , 0 , 0 , 0 , 0 , 0 } } ; static const rtwCAPI_BlockParameters rtBlockParameters [ ] = { { 24 , TARGET_STRING ( "MathematicalModel/Depth Control" ) , TARGET_STRING ( "P" ) , 0 , 0 , 0 } , { 25 , TARGET_STRING ( "MathematicalModel/Depth Control" ) , TARGET_STRING ( "I" ) , 0 , 0 , 0 } , { 26 , TARGET_STRING ( "MathematicalModel/Depth Control" ) , TARGET_STRING ( "D" ) , 0 , 0 , 0 } , { 27 , TARGET_STRING ( "MathematicalModel/Depth Control" ) , TARGET_STRING ( "N" ) , 0 , 0 , 0 } , { 28 , TARGET_STRING ( "MathematicalModel/Depth Control" ) , TARGET_STRING ( "InitialConditionForIntegrator" ) , 0 , 0 , 0 } , { 29 , TARGET_STRING ( "MathematicalModel/Depth Control" ) , TARGET_STRING ( "InitialConditionForFilter" ) , 0 , 0 , 0 } , { 30 , TARGET_STRING ( "MathematicalModel/PID Controller2" ) , TARGET_STRING ( "I" ) , 0 , 0 , 0 } , { 31 , TARGET_STRING ( "MathematicalModel/PID Controller2" ) , TARGET_STRING ( "D" ) , 0 , 0 , 0 } , { 32 , TARGET_STRING ( "MathematicalModel/PID Controller2" ) , TARGET_STRING ( "N" ) , 0 , 0 , 0 } , { 33 , TARGET_STRING ( "MathematicalModel/PID Controller2" ) , TARGET_STRING ( "InitialConditionForIntegrator" ) , 0 , 0 , 0 } , { 34 , TARGET_STRING ( "MathematicalModel/PID Controller2" ) , TARGET_STRING ( "InitialConditionForFilter" ) , 0 , 0 , 0 } , { 35 , TARGET_STRING ( "MathematicalModel/Constant1" ) , TARGET_STRING ( "Value" ) , 0 , 0 , 0 } , { 36 , TARGET_STRING ( "MathematicalModel/Speed Multiplier" ) , TARGET_STRING ( "Value" ) , 0 , 0 , 0 } , { 37 , TARGET_STRING ( "MathematicalModel/Gain" ) , TARGET_STRING ( "Gain" ) , 0 , 0 , 0 } , { 38 , TARGET_STRING ( "MathematicalModel/Gain1" ) , TARGET_STRING ( "Gain" ) , 0 , 0 , 0 } , { 39 , TARGET_STRING ( "MathematicalModel/m to mm" ) , TARGET_STRING ( "Gain" ) , 0 , 0 , 0 } , { 40 , TARGET_STRING ( "MathematicalModel/1-D Lookup Table1" ) , TARGET_STRING ( "Table" ) , 0 , 1 , 0 } , { 41 , TARGET_STRING ( "MathematicalModel/1-D Lookup Table1" ) , TARGET_STRING ( "BreakpointsForDimension1" ) , 0 , 1 , 0 } , { 42 , TARGET_STRING ( "MathematicalModel/1-D Lookup Table2" ) , TARGET_STRING ( "Table" ) , 0 , 1 , 0 } , { 43 , TARGET_STRING ( "MathematicalModel/1-D Lookup Table2" ) , TARGET_STRING ( "BreakpointsForDimension1" ) , 0 , 1 , 0 } , { 44 , TARGET_STRING ( "MathematicalModel/T200 Power Curve" ) , TARGET_STRING ( "Table" ) , 0 , 2 , 0 } , { 45 , TARGET_STRING ( "MathematicalModel/T200 Power Curve" ) , TARGET_STRING ( "BreakpointsForDimension1" ) , 0 , 2 , 0 } , { 46 , TARGET_STRING ( "MathematicalModel/Thrust Control to PWM" ) , TARGET_STRING ( "Table" ) , 0 , 3 , 0 } , { 47 , TARGET_STRING ( "MathematicalModel/Thrust Control to PWM" ) , TARGET_STRING ( "BreakpointsForDimension1" ) , 0 , 3 , 0 } , { 48 , TARGET_STRING ( "MathematicalModel/Depth Limit" ) , TARGET_STRING ( "UpperLimit" ) , 0 , 0 , 0 } , { 49 , TARGET_STRING ( "MathematicalModel/Depth Limit" ) , TARGET_STRING ( "LowerLimit" ) , 0 , 0 , 0 } , { 50 , TARGET_STRING ( "MathematicalModel/Disturbance " ) , TARGET_STRING ( "Amplitude" ) , 0 , 0 , 0 } , { 51 , TARGET_STRING ( "MathematicalModel/Disturbance " ) , TARGET_STRING ( "Bias" ) , 0 , 0 , 0 } , { 52 , TARGET_STRING ( "MathematicalModel/Disturbance " ) , TARGET_STRING ( "Frequency" ) , 0 , 0 , 0 } , { 53 , TARGET_STRING ( "MathematicalModel/Disturbance " ) , TARGET_STRING ( "Phase" ) , 0 , 0 , 0 } , { 54 , TARGET_STRING ( "MathematicalModel/Desired Depth (mm)" ) , TARGET_STRING ( "Time" ) , 0 , 0 , 0 } , { 55 , TARGET_STRING ( "MathematicalModel/Desired Depth (mm)" ) , TARGET_STRING ( "Before" ) , 0 , 0 , 0 } , { 56 , TARGET_STRING ( "MathematicalModel/Desired Depth (mm)" ) , TARGET_STRING ( "After" ) , 0 , 0 , 0 } , { 57 , TARGET_STRING ( "MathematicalModel/Sensor Noise" ) , TARGET_STRING ( "Minimum" ) , 0 , 0 , 0 } , { 58 , TARGET_STRING ( "MathematicalModel/Sensor Noise" ) , TARGET_STRING ( "Maximum" ) , 0 , 0 , 0 } , { 59 , TARGET_STRING ( "MathematicalModel/Sensor Noise" ) , TARGET_STRING ( "Seed" ) , 0 , 0 , 0 } , { 60 , TARGET_STRING ( "MathematicalModel/MATLAB System" ) , TARGET_STRING ( "SampleTime" ) , 0 , 0 , 0 } , { 61 , TARGET_STRING ( "MathematicalModel/Transfer Fcn (with initial outputs)/State Space" ) , TARGET_STRING ( "A_pr" ) , 0 , 4 , 0 } , { 62 , TARGET_STRING ( "MathematicalModel/Transfer Fcn (with initial outputs)/State Space" ) , TARGET_STRING ( "A_ir" ) , 1 , 4 , 0 } , { 63 , TARGET_STRING ( "MathematicalModel/Transfer Fcn (with initial outputs)/State Space" ) , TARGET_STRING ( "A_jc" ) , 1 , 5 , 0 } , { 64 , TARGET_STRING ( "MathematicalModel/Transfer Fcn (with initial outputs)/State Space" ) , TARGET_STRING ( "B_pr" ) , 0 , 0 , 0 } , { 65 , TARGET_STRING ( "MathematicalModel/Transfer Fcn (with initial outputs)/State Space" ) , TARGET_STRING ( "B_ir" ) , 1 , 0 , 0 } , { 66 , TARGET_STRING ( "MathematicalModel/Transfer Fcn (with initial outputs)/State Space" ) , TARGET_STRING ( "B_jc" ) , 1 , 4 , 0 } , { 67 , TARGET_STRING ( "MathematicalModel/Transfer Fcn (with initial outputs)/State Space" ) , TARGET_STRING ( "C_pr" ) , 0 , 0 , 0 } , { 68 , TARGET_STRING ( "MathematicalModel/Transfer Fcn (with initial outputs)/State Space" ) , TARGET_STRING ( "C_ir" ) , 1 , 0 , 0 } , { 69 , TARGET_STRING ( "MathematicalModel/Transfer Fcn (with initial outputs)/State Space" ) , TARGET_STRING ( "C_jc" ) , 1 , 5 , 0 } , { 70 , TARGET_STRING ( "MathematicalModel/Transfer Fcn (with initial outputs)/State Space" ) , TARGET_STRING ( "InitialCondition" ) , 0 , 4 , 0 } , { 71 , TARGET_STRING ( "MathematicalModel/Ultrasonic Sensor/State Space" ) , TARGET_STRING ( "A_pr" ) , 0 , 0 , 0 } , { 72 , TARGET_STRING ( "MathematicalModel/Ultrasonic Sensor/State Space" ) , TARGET_STRING ( "A_ir" ) , 1 , 0 , 0 } , { 73 , TARGET_STRING ( "MathematicalModel/Ultrasonic Sensor/State Space" ) , TARGET_STRING ( "A_jc" ) , 1 , 4 , 0 } , { 74 , TARGET_STRING ( "MathematicalModel/Ultrasonic Sensor/State Space" ) , TARGET_STRING ( "B_pr" ) , 0 , 0 , 0 } , { 75 , TARGET_STRING ( "MathematicalModel/Ultrasonic Sensor/State Space" ) , TARGET_STRING ( "B_ir" ) , 1 , 0 , 0 } , { 76 , TARGET_STRING ( "MathematicalModel/Ultrasonic Sensor/State Space" ) , TARGET_STRING ( "B_jc" ) , 1 , 4 , 0 } , { 77 , TARGET_STRING ( "MathematicalModel/Ultrasonic Sensor/State Space" ) , TARGET_STRING ( "C_pr" ) , 0 , 0 , 0 } , { 78 , TARGET_STRING ( "MathematicalModel/Ultrasonic Sensor/State Space" ) , TARGET_STRING ( "C_ir" ) , 1 , 0 , 0 } , { 79 , TARGET_STRING ( "MathematicalModel/Ultrasonic Sensor/State Space" ) , TARGET_STRING ( "C_jc" ) , 1 , 4 , 0 } , { 80 , TARGET_STRING ( "MathematicalModel/Ultrasonic Sensor/State Space" ) , TARGET_STRING ( "InitialCondition" ) , 0 , 0 , 0 } , { 81 , TARGET_STRING ( "MathematicalModel/Depth Control/Filter/Disc. Forward Euler Filter/Filter" ) , TARGET_STRING ( "gainval" ) , 0 , 0 , 0 } , { 82 , TARGET_STRING ( "MathematicalModel/Depth Control/Integrator/Discrete/Integrator" ) , TARGET_STRING ( "gainval" ) , 0 , 0 , 0 } , { 0 , ( NULL ) , ( NULL ) , 0 , 0 , 0 } } ; static int_T rt_LoggedStateIdxList [ ] = { - 1 } ; static const rtwCAPI_Signals rtRootInputs [ ] = { { 0 , 0 , ( NULL ) , ( NULL ) , 0 , 0 , 0 , 0 , 0 } } ; static const rtwCAPI_Signals rtRootOutputs [ ] = { { 0 , 0 , ( NULL ) , ( NULL ) , 0 , 0 , 0 , 0 , 0 } } ; static const rtwCAPI_ModelParameters rtModelParameters [ ] = { { 83 , TARGET_STRING ( "V" ) , 0 , 0 , 0 } , { 84 , TARGET_STRING ( "g" ) , 0 , 0 , 0 } , { 85 , TARGET_STRING ( "m" ) , 0 , 0 , 0 } , { 86 , TARGET_STRING ( "rho" ) , 0 , 0 , 0 } , { 0 , ( NULL ) , 0 , 0 , 0 } } ;
#ifndef HOST_CAPI_BUILD
static void * rtDataAddrMap [ ] = { & rtB . j1h0no0d4b , & rtB . l5manf4ikg ,
& rtB . j4ai4z50gk , & rtB . oihzkslm5h , & rtB . kdvlwjj4tq , & rtB .
ks2yz4xb4p , & rtB . lt1rdvoctr , & rtB . koclthgufh , & rtB . lny4bmp0zj , &
rtB . od11d41rmb , & rtB . mqu2twqjlp , & rtB . kjm434vqfg , & rtB .
dzv1mxhlqy , & rtB . ajoc0paeou , & rtB . acg1d02c2x , & rtB . dttgtgicym , &
rtB . kta3pakdpy , & rtB . ot4biexhrc , & rtB . l3gxjc55ml , & rtB .
jvycygbmvu , & rtB . isocrfb3eq , & rtB . hiu3ufhlyn , & rtB . b0vho2tiwj , &
rtB . os02ukstpo , & rtP . DepthControl_P , & rtP . DepthControl_I , & rtP .
DepthControl_D , & rtP . DepthControl_N , & rtP .
DepthControl_InitialConditionForIntegrator , & rtP .
DepthControl_InitialConditionForFilter , & rtP . PIDController2_I , & rtP .
PIDController2_D , & rtP . PIDController2_N , & rtP .
PIDController2_InitialConditionForIntegrator , & rtP .
PIDController2_InitialConditionForFilter , & rtP . Constant1_Value , & rtP .
SpeedMultiplier_Value , & rtP . Gain_Gain , & rtP . Gain1_Gain , & rtP .
mtomm_Gain , & rtP . uDLookupTable1_tableData [ 0 ] , & rtP .
uDLookupTable1_bp01Data [ 0 ] , & rtP . uDLookupTable2_tableData [ 0 ] , &
rtP . uDLookupTable2_bp01Data [ 0 ] , & rtP . T200PowerCurve_tableData [ 0 ]
, & rtP . T200PowerCurve_bp01Data [ 0 ] , & rtP .
ThrustControltoPWM_tableData [ 0 ] , & rtP . ThrustControltoPWM_bp01Data [ 0
] , & rtP . DepthLimit_UpperSat , & rtP . DepthLimit_LowerSat , & rtP .
Disturbance_Amp , & rtP . Disturbance_Bias , & rtP . Disturbance_Freq , & rtP
. Disturbance_Phase , & rtP . DesiredDepthmm_Time , & rtP . DesiredDepthmm_Y0
, & rtP . DesiredDepthmm_YFinal , & rtP . SensorNoise_Minimum , & rtP .
SensorNoise_Maximum , & rtP . SensorNoise_Seed , & rtP .
MATLABSystem_SampleTime , & rtP . StateSpace_A_pr_jy4jjiufdj [ 0 ] , & rtP .
StateSpace_A_ir_d34gaajqxg [ 0 ] , & rtP . StateSpace_A_jc_ogjw4zhixw [ 0 ] ,
& rtP . StateSpace_B_pr_dn004bbuke , & rtP . StateSpace_B_ir_o1eztmk1co , &
rtP . StateSpace_B_jc_k02fgthtcj [ 0 ] , & rtP . StateSpace_C_pr_jgvflc2hx0 ,
& rtP . StateSpace_C_ir_n00n3nmwj5 , & rtP . StateSpace_C_jc_jwkolzzbma [ 0 ]
, & rtP . StateSpace_InitialCondition_cvrppagalz [ 0 ] , & rtP .
StateSpace_A_pr , & rtP . StateSpace_A_ir , & rtP . StateSpace_A_jc [ 0 ] , &
rtP . StateSpace_B_pr , & rtP . StateSpace_B_ir , & rtP . StateSpace_B_jc [ 0
] , & rtP . StateSpace_C_pr , & rtP . StateSpace_C_ir , & rtP .
StateSpace_C_jc [ 0 ] , & rtP . StateSpace_InitialCondition , & rtP .
Filter_gainval , & rtP . Integrator_gainval , & rtP . V , & rtP . g , & rtP .
m , & rtP . rho , } ; static int32_T * rtVarDimsAddrMap [ ] = { ( NULL ) } ;
#endif
static TARGET_CONST rtwCAPI_DataTypeMap rtDataTypeMap [ ] = { { "double" ,
"real_T" , 0 , 0 , sizeof ( real_T ) , ( uint8_T ) SS_DOUBLE , 0 , 0 , 0 } ,
{ "unsigned int" , "uint32_T" , 0 , 0 , sizeof ( uint32_T ) , ( uint8_T )
SS_UINT32 , 0 , 0 , 0 } , { "unsigned short" , "uint16_T" , 0 , 0 , sizeof ( uint16_T ) , ( uint8_T ) SS_UINT16 , 0 , 0 , 0 } } ;
#ifdef HOST_CAPI_BUILD
#undef sizeof
#endif
static TARGET_CONST rtwCAPI_ElementMap rtElementMap [ ] = { { ( NULL ) , 0 ,
0 , 0 , 0 } , } ; static const rtwCAPI_DimensionMap rtDimensionMap [ ] = { {
rtwCAPI_SCALAR , 0 , 2 , 0 } , { rtwCAPI_VECTOR , 2 , 2 , 0 } , {
rtwCAPI_VECTOR , 4 , 2 , 0 } , { rtwCAPI_VECTOR , 6 , 2 , 0 } , {
rtwCAPI_VECTOR , 8 , 2 , 0 } , { rtwCAPI_VECTOR , 10 , 2 , 0 } } ; static
const uint_T rtDimensionArray [ ] = { 1 , 1 , 1 , 801 , 201 , 1 , 1 , 201 , 2
, 1 , 3 , 1 } ; static const real_T rtcapiStoredFloats [ ] = { 2.0E-5 , 0.0 ,
1.0 , 0.5 } ; static const rtwCAPI_FixPtMap rtFixPtMap [ ] = { { ( NULL ) , ( NULL ) , rtwCAPI_FIX_RESERVED , 0 , 0 , ( boolean_T ) 0 } , } ; static const rtwCAPI_SampleTimeMap rtSampleTimeMap [ ] = { { ( NULL ) , ( NULL ) , 4 , 0 } , { ( const void * ) & rtcapiStoredFloats [ 0 ] , ( const void * ) & rtcapiStoredFloats [ 1 ] , ( int8_T ) 2 , ( uint8_T ) 0 } , { ( const void * ) & rtcapiStoredFloats [ 1 ] , ( const void * ) & rtcapiStoredFloats [ 1 ] , ( int8_T ) 0 , ( uint8_T ) 0 } , { ( const void * ) & rtcapiStoredFloats [ 1 ] , ( const void * ) & rtcapiStoredFloats [ 2 ] , ( int8_T ) 1 , ( uint8_T ) 0 } , { ( const void * ) & rtcapiStoredFloats [ 3 ] , ( const void * ) & rtcapiStoredFloats [ 1 ] , ( int8_T ) 3 , ( uint8_T ) 0 } } ; static rtwCAPI_ModelMappingStaticInfo mmiStatic = { { rtBlockSignals , 24 , rtRootInputs , 0 , rtRootOutputs , 0 } , { rtBlockParameters , 59 , rtModelParameters , 4 } , { ( NULL ) , 0 } , { rtDataTypeMap , rtDimensionMap , rtFixPtMap , rtElementMap , rtSampleTimeMap , rtDimensionArray } , "float" , { 109747031U , 2709809023U , 2004263737U , 379092649U } , ( NULL ) , 0 , ( boolean_T ) 0 , rt_LoggedStateIdxList } ; const rtwCAPI_ModelMappingStaticInfo * MathematicalModel_GetCAPIStaticMap ( void ) { return & mmiStatic ; }
#ifndef HOST_CAPI_BUILD
void MathematicalModel_InitializeDataMapInfo ( void ) { rtwCAPI_SetVersion ( ( *
rt_dataMapInfoPtr ) . mmi , 1 ) ; rtwCAPI_SetStaticMap ( ( *
rt_dataMapInfoPtr ) . mmi , & mmiStatic ) ; rtwCAPI_SetLoggingStaticMap ( ( *
rt_dataMapInfoPtr ) . mmi , ( NULL ) ) ; rtwCAPI_SetDataAddressMap ( ( *
rt_dataMapInfoPtr ) . mmi , rtDataAddrMap ) ; rtwCAPI_SetVarDimsAddressMap ( ( *
rt_dataMapInfoPtr ) . mmi , rtVarDimsAddrMap ) ;
rtwCAPI_SetInstanceLoggingInfo ( ( * rt_dataMapInfoPtr ) . mmi , ( NULL ) ) ;
rtwCAPI_SetChildMMIArray ( ( * rt_dataMapInfoPtr ) . mmi , ( NULL ) ) ;
rtwCAPI_SetChildMMIArrayLen ( ( * rt_dataMapInfoPtr ) . mmi , 0 ) ; }
#else
#ifdef __cplusplus
extern "C" {
#endif
void MathematicalModel_host_InitializeDataMapInfo ( MathematicalModel_host_DataMapInfo_T * dataMap , const char * path ) { rtwCAPI_SetVersion ( dataMap -> mmi , 1 ) ; rtwCAPI_SetStaticMap ( dataMap -> mmi , & mmiStatic ) ; rtwCAPI_SetDataAddressMap ( dataMap -> mmi , ( NULL ) ) ; rtwCAPI_SetVarDimsAddressMap ( dataMap -> mmi , ( NULL ) ) ; rtwCAPI_SetPath ( dataMap -> mmi , path ) ; rtwCAPI_SetFullPath ( dataMap -> mmi , ( NULL ) ) ; rtwCAPI_SetChildMMIArray ( dataMap -> mmi , ( NULL ) ) ; rtwCAPI_SetChildMMIArrayLen ( dataMap -> mmi , 0 ) ; }
#ifdef __cplusplus
}
#endif
#endif
