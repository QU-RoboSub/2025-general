#ifndef MathematicalModel_h_
#define MathematicalModel_h_
#ifndef MathematicalModel_COMMON_INCLUDES_
#define MathematicalModel_COMMON_INCLUDES_
#include <stdlib.h>
#include "sl_AsyncioQueue/AsyncioQueueCAPI.h"
#include "rtwtypes.h"
#include "sigstream_rtw.h"
#include "simtarget/slSimTgtSigstreamRTW.h"
#include "simtarget/slSimTgtSlioCoreRTW.h"
#include "simtarget/slSimTgtSlioClientsRTW.h"
#include "simtarget/slSimTgtSlioSdiRTW.h"
#include "simstruc.h"
#include "fixedpoint.h"
#include "raccel.h"
#include "slsv_diagnostic_codegen_c_api.h"
#include "rt_logging_simtarget.h"
#include "rt_nonfinite.h"
#include "math.h"
#include "dt_info.h"
#include "ext_work.h"
#include "Ping_v3.h"
#include "MW_PWM.h"
#endif
#include "MathematicalModel_types.h"
#include <stddef.h>
#include "rtw_modelmap_simtarget.h"
#include "rt_defines.h"
#include <string.h>
#include "rtGetInf.h"
#define MODEL_NAME MathematicalModel
#define NSAMPLE_TIMES (5) 
#define NINPUTS (0)       
#define NOUTPUTS (0)     
#define NBLOCKIO (24) 
#define NUM_ZC_EVENTS (0) 
#ifndef NCSTATES
#define NCSTATES (5)   
#elif NCSTATES != 5
#error Invalid specification of NCSTATES defined in compiler command
#endif
#ifndef rtmGetDataMapInfo
#define rtmGetDataMapInfo(rtm) (*rt_dataMapInfoPtr)
#endif
#ifndef rtmSetDataMapInfo
#define rtmSetDataMapInfo(rtm, val) (rt_dataMapInfoPtr = &val)
#endif
#ifndef IN_RACCEL_MAIN
#endif
typedef struct { real_T lt1rdvoctr ; real_T dttgtgicym ; real_T koclthgufh ;
real_T ot4biexhrc ; real_T os02ukstpo ; real_T mqu2twqjlp ; real_T l5manf4ikg
; real_T lny4bmp0zj ; real_T acg1d02c2x ; real_T j4ai4z50gk ; real_T
ks2yz4xb4p ; real_T kjm434vqfg ; real_T od11d41rmb ; real_T kta3pakdpy ;
real_T l3gxjc55ml ; real_T jvycygbmvu ; real_T isocrfb3eq ; real_T hiu3ufhlyn
; real_T b0vho2tiwj ; real_T oihzkslm5h ; real_T kdvlwjj4tq ; real_T
j1h0no0d4b ; uint32_T dzv1mxhlqy ; uint16_T ajoc0paeou ; } B ; typedef struct
{ luufrrieoo kyf2nclza1 ; hr3cntwh2j ojmumzzinc ; real_T gdqbf2crkk ; real_T
d2gjyneiwi ; real_T mqqpv5nckr ; struct { void * AQHandles ; } ayjhvvhiab ;
struct { void * LoggedData ; } gkjdrzz2x1 ; struct { void * LoggedData ; }
bindmu0m5v ; struct { void * LoggedData ; } ag142pt11j ; struct { void *
LoggedData ; } pjjo4vsiiw ; struct { void * LoggedData ; } k53jbvhb4r ;
struct { void * LoggedData ; } p00iq10u1v ; uint32_T lzqjq4aymt ; int_T
e3ooy4be24 ; int_T h20vrohnnu ; boolean_T cr445o1xte ; boolean_T if4q3i004h ;
} DW ; typedef struct { real_T nwz4bzja4j ; real_T ec0ecggc1k [ 2 ] ; real_T
as4jknb3rz ; real_T nxnvlbl4qq ; } X ; typedef struct { real_T nwz4bzja4j ;
real_T ec0ecggc1k [ 2 ] ; real_T as4jknb3rz ; real_T nxnvlbl4qq ; } XDot ;
typedef struct { boolean_T nwz4bzja4j ; boolean_T ec0ecggc1k [ 2 ] ;
boolean_T as4jknb3rz ; boolean_T nxnvlbl4qq ; } XDis ; typedef struct {
real_T nwz4bzja4j ; real_T ec0ecggc1k [ 2 ] ; real_T as4jknb3rz ; real_T
nxnvlbl4qq ; } CStateAbsTol ; typedef struct { real_T nwz4bzja4j ; real_T
ec0ecggc1k [ 2 ] ; real_T as4jknb3rz ; real_T nxnvlbl4qq ; } CXPtMin ;
typedef struct { real_T nwz4bzja4j ; real_T ec0ecggc1k [ 2 ] ; real_T
as4jknb3rz ; real_T nxnvlbl4qq ; } CXPtMax ; typedef struct { real_T
o5jmwaaczw ; real_T mqexkk12ke ; real_T e2qy0rfqde ; } ZCV ; typedef struct {
rtwCAPI_ModelMappingInfo mmi ; } DataMapInfo ; struct P_ { real_T V ; real_T
g ; real_T m ; real_T rho ; real_T DepthControl_D ; real_T PIDController2_D ;
real_T DepthControl_I ; real_T PIDController2_I ; real_T
DepthControl_InitialConditionForFilter ; real_T
PIDController2_InitialConditionForFilter ; real_T
DepthControl_InitialConditionForIntegrator ; real_T
PIDController2_InitialConditionForIntegrator ; real_T DepthControl_N ; real_T
PIDController2_N ; real_T DepthControl_P ; real_T MATLABSystem_SampleTime ;
real_T DesiredDepthmm_Time ; real_T DesiredDepthmm_Y0 ; real_T
DesiredDepthmm_YFinal ; real_T StateSpace_A_pr ; real_T StateSpace_B_pr ;
real_T StateSpace_C_pr ; real_T StateSpace_InitialCondition ; real_T
Integrator_gainval ; real_T Filter_gainval ; real_T
ThrustControltoPWM_tableData [ 201 ] ; real_T ThrustControltoPWM_bp01Data [
201 ] ; real_T T200PowerCurve_tableData [ 201 ] ; real_T
T200PowerCurve_bp01Data [ 201 ] ; real_T Disturbance_Amp ; real_T
Disturbance_Bias ; real_T Disturbance_Freq ; real_T Disturbance_Phase ;
real_T Gain1_Gain ; real_T StateSpace_A_pr_jy4jjiufdj [ 2 ] ; real_T
StateSpace_B_pr_dn004bbuke ; real_T StateSpace_C_pr_jgvflc2hx0 ; real_T
StateSpace_InitialCondition_cvrppagalz [ 2 ] ; real_T mtomm_Gain ; real_T
DepthLimit_UpperSat ; real_T DepthLimit_LowerSat ; real_T SensorNoise_Minimum
; real_T SensorNoise_Maximum ; real_T SensorNoise_Seed ; real_T
Constant1_Value ; real_T uDLookupTable1_tableData [ 801 ] ; real_T
uDLookupTable1_bp01Data [ 801 ] ; real_T uDLookupTable2_tableData [ 801 ] ;
real_T uDLookupTable2_bp01Data [ 801 ] ; real_T SpeedMultiplier_Value ;
real_T Gain_Gain ; uint32_T StateSpace_A_ir ; uint32_T StateSpace_A_jc [ 2 ]
; uint32_T StateSpace_B_ir ; uint32_T StateSpace_B_jc [ 2 ] ; uint32_T
StateSpace_C_ir ; uint32_T StateSpace_C_jc [ 2 ] ; uint32_T
StateSpace_A_ir_d34gaajqxg [ 2 ] ; uint32_T StateSpace_A_jc_ogjw4zhixw [ 3 ]
; uint32_T StateSpace_B_ir_o1eztmk1co ; uint32_T StateSpace_B_jc_k02fgthtcj [
2 ] ; uint32_T StateSpace_C_ir_n00n3nmwj5 ; uint32_T
StateSpace_C_jc_jwkolzzbma [ 3 ] ; } ; extern const char_T *
RT_MEMORY_ALLOCATION_ERROR ; extern B rtB ; extern X rtX ; extern DW rtDW ;
extern P rtP ; extern mxArray * mr_MathematicalModel_GetDWork ( ) ; extern
void mr_MathematicalModel_SetDWork ( const mxArray * ssDW ) ; extern mxArray
* mr_MathematicalModel_GetSimStateDisallowedBlocks ( ) ; extern const
rtwCAPI_ModelMappingStaticInfo * MathematicalModel_GetCAPIStaticMap ( void )
; extern SimStruct * const rtS ; extern DataMapInfo * rt_dataMapInfoPtr ;
extern rtwCAPI_ModelMappingInfo * rt_modelMapInfoPtr ; void MdlOutputs ( int_T
tid ) ; void MdlOutputsParameterSampleTime ( int_T tid ) ; void MdlUpdate ( int_T tid ) ; void MdlTerminate ( void ) ; void MdlInitializeSizes ( void ) ; void MdlInitializeSampleTimes ( void ) ; SimStruct * raccel_register_model ( ssExecutionInfo * executionInfo ) ;
#endif
