#ifndef RTW_HEADER_wheel_leg_h_
#define RTW_HEADER_wheel_leg_h_
#include <stddef.h>
#include <string.h>
#include "rtw_modelmap.h"
#ifndef wheel_leg_COMMON_INCLUDES_
#define wheel_leg_COMMON_INCLUDES_
#include <stdlib.h>
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
#include "rt_logging.h"
#include "dt_info.h"
#include "ext_work.h"
#include "nesl_rtw.h"
#include "wheel_leg_d9e82f16_1_gateway.h"
#endif
#include "wheel_leg_types.h"
#include "multiword_types.h"
#include "mwmathutil.h"
#include "rt_defines.h"
#include "rtGetInf.h"
#include "rt_nonfinite.h"
#define MODEL_NAME wheel_leg
#define NSAMPLE_TIMES (3) 
#define NINPUTS (0)       
#define NOUTPUTS (0)     
#define NBLOCKIO (69) 
#define NUM_ZC_EVENTS (1) 
#ifndef NCSTATES
#define NCSTATES (39)   
#elif NCSTATES != 39
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
typedef struct { real_T eh2cmnfpcd [ 32 ] ; real_T ne53mrvzfq [ 13 ] ; real_T
nwuie3h1zj ; real_T liiv3zehzs ; real_T hkrrrfopbq ; real_T oaaq01l5fu ;
real_T oqhhk34uis ; real_T fx0alrhetp ; real_T c4gioprvct ; real_T ewa15giale
; real_T cfmoived0b ; real_T jsqo11gz4n ; real_T fvygvwpi1d ; real_T
nkvqeuwnam ; real_T cufla4blkp ; real_T a5rxjeqmnm ; real_T lp2kjlaox2 ;
real_T d4fftorabg ; real_T o0acnzn2gn ; real_T lno5gwkvsm [ 6 ] ; real_T
m0bxxy51bt [ 2 ] ; real_T ct5ptiiqfm ; real_T egyva5trzl ; real_T hhykaoe4za
; real_T emby3igyk1 ; real_T lunhybn4j1 ; real_T bibxaqp42j ; real_T
ngl2rc54wx ; real_T mkoprhigc1 ; real_T p4cj0opfml ; real_T kxvii2moqu ;
real_T dz5u3bubv3 ; real_T b0mlwzzkzn ; real_T bpbscqkzum ; real_T
lno5gwkvsmk [ 6 ] ; real_T kourgu0gp0 [ 2 ] ; real_T luhg3pfwco ; real_T
fasi4utync ; real_T mj2xnu20kl ; real_T cp1jjx2a4p ; real_T furrbhkomt ;
real_T mrjzncw5q4 ; real_T imk0sd2jrm ; real_T jteoenmoec ; real_T jw4ohhvqrt
; real_T f1443sccd2 ; real_T jfiugehczg ; real_T hdeqxnl10m ; real_T
djd0cbchne ; real_T hl2vsnjvah ; real_T hfyfla4srx ; real_T nlnzoie4ft ;
real_T affemz5v0v ; real_T amehlztnnv ; real_T fviua014cu ; real_T kaed4psp05
; real_T chcwz4ujvj ; real_T nxxivmzdo0 ; real_T k3exkay1ay ; real_T
kma1dpwhhf ; real_T nik4ggyist [ 4 ] ; real_T ludhxwcsap [ 4 ] ; real_T
jl2dsktpay [ 4 ] ; real_T clrzjaih1v [ 4 ] ; real_T dmfybrfnod [ 4 ] ; real_T
l1345vcjcj [ 4 ] ; real_T bdqfv1d2cw ; real_T lbhtwulbcm ; real_T p1iy05u0ee
; } B ; typedef struct { real_T pvnlmwwllm [ 2 ] ; real_T mxlt45ij3o [ 2 ] ;
real_T f1r4g4whvp [ 2 ] ; real_T aokoeghayj [ 2 ] ; real_T kulfjc4a2j [ 2 ] ;
real_T leayjggnf1 [ 2 ] ; real_T ooldbvhge3 ; real_T cq3o2ec5fw ; real_T
h5eppyllsj ; void * eukrvu0tgn ; void * gahukgbj3h ; void * bq244s1auv ; void
* jsgic4uyjm ; void * cxtggfsfxm ; void * go2zdudlka ; void * jg2ci40rqj ;
void * bmoh4rr253 ; void * h2fxcrou4b ; void * gmcd5pdmpl ; struct { void *
LoggedData ; } pxbtt2xwqv ; struct { void * LoggedData ; } cgimjkdh23 ;
struct { void * LoggedData ; } hpnbv4vlgo ; struct { void * LoggedData ; }
ditp2lm0v5 ; struct { void * LoggedData ; } heiomuhezr ; struct { void *
LoggedData ; } euizrqhecv ; struct { void * LoggedData ; } ghm5gdrsfi ;
struct { void * LoggedData ; } mxi5rvja4b ; struct { void * LoggedData ; }
evkpec522i ; struct { void * LoggedData ; } iwbd2ehmq0 ; struct { void *
LoggedData ; } nrwpiptbpa ; struct { void * LoggedData ; } ae5ijrvxng ;
struct { void * LoggedData ; } ijaanpy3ox ; struct { void * LoggedData ; }
oj0cyu2zrs ; struct { void * LoggedData ; } nebmwciflv ; struct { void *
LoggedData ; } nxzkrnlszw ; struct { void * LoggedData ; } nssaobtpxs ; void
* loahdpiobx ; void * hpnunkuqpe ; void * ojbc53o3xt ; void * bzw2pamqyl ;
void * pifpfpnwqn ; int_T nzztmagq15 ; int_T d5bjwf4dqd ; int_T g24kz15cah ;
int_T kbppfzielz ; int_T acajdtsjkp ; int_T pkcg1jz5ka ; int_T ei5ynuzssw ;
int_T ekmth0nsne ; int_T b15ck32r2h ; int_T gy03yk4tjx ; int_T dgksrckq3m ;
int_T cojkg3rcgw ; int_T a04qfypmbt ; int_T hjkpjosmn0 ; int_T huukfn1whc ;
int_T bj5n1250ac ; int_T g1jau2oi4y ; int_T phsfelswbk ; int_T emdjtjjfhz ;
int_T i1mxirq5kb ; int_T dhbkidd3zo ; int_T iy1pcv3xo4 ; int_T eqwdrnbx1x ;
int_T pny00ejsqz ; int_T aqiduhop2a ; int_T fpt2stpuqn ; int_T ctsfqdbiky ;
int8_T kpjnyoicn4 ; boolean_T mif2ebuzod ; boolean_T o50jinjfvw ; boolean_T
kikuoijf40 ; } DW ; typedef struct { real_T mefcq5joqv [ 32 ] ; real_T
cbywjryyoi ; real_T b2hgpxvjzc ; real_T abxab3djbi ; real_T p5p3ogk4mi ;
real_T cwaruwgj0n ; real_T ecnkbzok1f ; real_T iko4pycqyt ; } X ; typedef
struct { real_T mefcq5joqv [ 32 ] ; real_T cbywjryyoi ; real_T b2hgpxvjzc ;
real_T abxab3djbi ; real_T p5p3ogk4mi ; real_T cwaruwgj0n ; real_T ecnkbzok1f
; real_T iko4pycqyt ; } XDot ; typedef struct { boolean_T mefcq5joqv [ 32 ] ;
boolean_T cbywjryyoi ; boolean_T b2hgpxvjzc ; boolean_T abxab3djbi ;
boolean_T p5p3ogk4mi ; boolean_T cwaruwgj0n ; boolean_T ecnkbzok1f ;
boolean_T iko4pycqyt ; } XDis ; typedef struct { real_T mefcq5joqv [ 32 ] ;
real_T cbywjryyoi ; real_T b2hgpxvjzc ; real_T abxab3djbi ; real_T p5p3ogk4mi
; real_T cwaruwgj0n ; real_T ecnkbzok1f ; real_T iko4pycqyt ; } CStateAbsTol
; typedef struct { real_T mefcq5joqv [ 32 ] ; real_T cbywjryyoi ; real_T
b2hgpxvjzc ; real_T abxab3djbi ; real_T p5p3ogk4mi ; real_T cwaruwgj0n ;
real_T ecnkbzok1f ; real_T iko4pycqyt ; } CXPtMin ; typedef struct { real_T
mefcq5joqv [ 32 ] ; real_T cbywjryyoi ; real_T b2hgpxvjzc ; real_T abxab3djbi
; real_T p5p3ogk4mi ; real_T cwaruwgj0n ; real_T ecnkbzok1f ; real_T
iko4pycqyt ; } CXPtMax ; typedef struct { real_T gpfksi43yf ; real_T
hrb1f4zrkp ; real_T mx3ej2d0lz ; real_T j2w11ulpbr ; real_T h0ndtenaza ;
real_T pqo3v35tj2 ; real_T p2lke5fdjt ; real_T kwayygqstb ; real_T npan3fd15s
; real_T gwz0n5o4x3 ; real_T gqi0j3vxe4 ; real_T c0javv13ug ; real_T
jfgmng40eu ; real_T obibk1x1hr ; real_T pvexx3vpoz ; real_T cii3tbqbnf ;
real_T l3zjpcmhxv ; real_T kkb3dwir2d ; real_T f32leexdmb ; real_T ladlc0iipq
; real_T m4wqxj1mr5 ; real_T e4xrxgq33o ; real_T pbjb22nahj ; real_T
hr4f4iah5g ; real_T ag3ne5iazt ; real_T mf3q5omeon ; real_T mcy5qbv5vq ;
real_T jbunrt0jv0 ; real_T ouocp21etn ; real_T d3fdpqxbhg ; real_T koyy2anae0
; } ZCV ; typedef struct { ZCSigState hcfhnl2xxl ; } PrevZCX ; typedef struct
{ rtwCAPI_ModelMappingInfo mmi ; } DataMapInfo ; struct P_ { real_T K [ 12 ]
; real_T PIDController3_D ; real_T PIDController2_D ; real_T PIDController2_I
; real_T PIDController3_I ; real_T PIDController3_InitialConditionForFilter ;
real_T PIDController2_InitialConditionForFilter ; real_T
PIDController3_InitialConditionForIntegrator ; real_T
PIDController2_InitialConditionForIntegrator ; real_T PIDController3_N ;
real_T PIDController2_N ; real_T PIDController3_P ; real_T PIDController2_P ;
real_T jump_control_jump_time ; real_T T1_Y0 ; real_T T2_Y0 ; real_T
Gain26_Gain ; real_T Gain25_Gain ; real_T Gain33_Gain ; real_T Gain24_Gain ;
real_T Gain40_Gain ; real_T Gain38_Gain ; real_T Gain22_Gain ; real_T
Gain36_Gain ; real_T Integrator_IC ; real_T Step_Y0 ; real_T Step_YFinal ;
real_T Step16_Y0 ; real_T Step16_YFinal ; real_T Step1_Y0 ; real_T
Step1_YFinal ; real_T Step13_Y0 ; real_T Step13_YFinal ; real_T Step6_Y0 ;
real_T Step6_YFinal ; real_T Step14_Y0 ; real_T Step14_YFinal ; real_T
Step7_Y0 ; real_T Step7_YFinal ; real_T Step12_Y0 ; real_T Step12_YFinal ;
real_T Step8_Y0 ; real_T Step8_YFinal ; real_T Step15_Y0 ; real_T
Step15_YFinal ; real_T Step9_Y0 ; real_T Step9_YFinal ; real_T Step11_Y0 ;
real_T Step11_YFinal ; real_T Saturation5_UpperSat ; real_T
Saturation5_LowerSat ; real_T Step4_Y0 ; real_T Step4_YFinal ; real_T
Step17_Y0 ; real_T Step17_YFinal ; real_T Step5_Y0 ; real_T Step5_YFinal ;
real_T Step10_Y0 ; real_T Step10_YFinal ; real_T Saturation1_UpperSat ;
real_T Saturation1_LowerSat ; real_T Saturation2_UpperSat ; real_T
Saturation2_LowerSat ; real_T Saturation3_UpperSat ; real_T
Saturation3_LowerSat ; real_T Saturation4_UpperSat ; real_T
Saturation4_LowerSat ; real_T Saturation6_UpperSat ; real_T
Saturation6_LowerSat ; real_T Saturation7_UpperSat ; real_T
Saturation7_LowerSat ; real_T Saturation8_UpperSat ; real_T
Saturation8_LowerSat ; real_T Step2_Time ; real_T Step2_Y0 ; real_T
Step2_YFinal ; real_T Step3_Time ; real_T Step3_Y0 ; real_T Step3_YFinal ;
real_T Integrator4_IC ; real_T Gain29_Gain ; real_T Integrator2_IC ; real_T
Gain41_Gain ; real_T Constant_Value ; real_T Constant1_Value ; real_T
Constant10_Value ; real_T Constant12_Value ; real_T Value ; real_T _Value ; }
; extern const char * RT_MEMORY_ALLOCATION_ERROR ; extern B rtB ; extern X
rtX ; extern DW rtDW ; extern PrevZCX rtPrevZCX ; extern P rtP ; extern const
rtwCAPI_ModelMappingStaticInfo * wheel_leg_GetCAPIStaticMap ( void ) ; extern
SimStruct * const rtS ; extern const int_T gblNumToFiles ; extern const int_T
gblNumFrFiles ; extern const int_T gblNumFrWksBlocks ; extern rtInportTUtable
* gblInportTUtables ; extern const char * gblInportFileName ; extern const
int_T gblNumRootInportBlks ; extern const int_T gblNumModelInputs ; extern
const int_T gblInportDataTypeIdx [ ] ; extern const int_T gblInportDims [ ] ;
extern const int_T gblInportComplex [ ] ; extern const int_T
gblInportInterpoFlag [ ] ; extern const int_T gblInportContinuous [ ] ;
extern const int_T gblParameterTuningTid ; extern DataMapInfo *
rt_dataMapInfoPtr ; extern rtwCAPI_ModelMappingInfo * rt_modelMapInfoPtr ;
void MdlOutputs ( int_T tid ) ; void MdlOutputsParameterSampleTime ( int_T
tid ) ; void MdlUpdate ( int_T tid ) ; void MdlTerminate ( void ) ; void
MdlInitializeSizes ( void ) ; void MdlInitializeSampleTimes ( void ) ;
SimStruct * raccel_register_model ( ssExecutionInfo * executionInfo ) ;
#endif
