#include "rtw_capi.h"
#ifdef HOST_CAPI_BUILD
#include "wheel_leg_capi_host.h"
#define sizeof(s) ((size_t)(0xFFFF))
#undef rt_offsetof
#define rt_offsetof(s,el) ((uint16_T)(0xFFFF))
#define TARGET_CONST
#define TARGET_STRING(s) (s)    
#else
#include "builtin_typeid_types.h"
#include "wheel_leg.h"
#include "wheel_leg_capi.h"
#include "wheel_leg_private.h"
#ifdef LIGHT_WEIGHT_CAPI
#define TARGET_CONST                  
#define TARGET_STRING(s)               (NULL)                    
#else
#define TARGET_CONST                   const
#define TARGET_STRING(s)               (s)
#endif
#endif
static const rtwCAPI_Signals rtBlockSignals [ ] = { { 0 , 0 , TARGET_STRING (
"wheel_leg/Gain22" ) , TARGET_STRING ( "" ) , 0 , 0 , 0 , 0 , 0 } , { 1 , 0 ,
TARGET_STRING ( "wheel_leg/Gain24" ) , TARGET_STRING ( "" ) , 0 , 0 , 0 , 0 ,
0 } , { 2 , 0 , TARGET_STRING ( "wheel_leg/Gain25" ) , TARGET_STRING ( "" ) ,
0 , 0 , 0 , 0 , 0 } , { 3 , 0 , TARGET_STRING ( "wheel_leg/Gain26" ) ,
TARGET_STRING ( "" ) , 0 , 0 , 0 , 0 , 0 } , { 4 , 0 , TARGET_STRING (
"wheel_leg/Gain29" ) , TARGET_STRING ( "x'" ) , 0 , 0 , 0 , 0 , 0 } , { 5 , 0
, TARGET_STRING ( "wheel_leg/Gain33" ) , TARGET_STRING ( "" ) , 0 , 0 , 0 , 0
, 0 } , { 6 , 0 , TARGET_STRING ( "wheel_leg/Gain36" ) , TARGET_STRING ( "" )
, 0 , 0 , 0 , 0 , 0 } , { 7 , 0 , TARGET_STRING ( "wheel_leg/Gain38" ) ,
TARGET_STRING ( "" ) , 0 , 0 , 0 , 0 , 0 } , { 8 , 0 , TARGET_STRING (
"wheel_leg/Gain40" ) , TARGET_STRING ( "" ) , 0 , 0 , 0 , 0 , 0 } , { 9 , 0 ,
TARGET_STRING ( "wheel_leg/Gain41" ) , TARGET_STRING ( "x'" ) , 0 , 0 , 0 , 0
, 0 } , { 10 , 0 , TARGET_STRING ( "wheel_leg/Integrator" ) , TARGET_STRING (
"" ) , 0 , 0 , 0 , 0 , 0 } , { 11 , 0 , TARGET_STRING (
"wheel_leg/Integrator2" ) , TARGET_STRING ( "x" ) , 0 , 0 , 0 , 0 , 0 } , {
12 , 0 , TARGET_STRING ( "wheel_leg/Integrator4" ) , TARGET_STRING ( "x" ) ,
0 , 0 , 0 , 0 , 0 } , { 13 , 0 , TARGET_STRING ( "wheel_leg/Saturation1" ) ,
TARGET_STRING ( "" ) , 0 , 0 , 0 , 0 , 1 } , { 14 , 0 , TARGET_STRING (
"wheel_leg/Saturation2" ) , TARGET_STRING ( "" ) , 0 , 0 , 0 , 0 , 1 } , { 15
, 0 , TARGET_STRING ( "wheel_leg/Saturation3" ) , TARGET_STRING ( "" ) , 0 ,
0 , 0 , 0 , 0 } , { 16 , 0 , TARGET_STRING ( "wheel_leg/Saturation4" ) ,
TARGET_STRING ( "" ) , 0 , 0 , 0 , 0 , 0 } , { 17 , 0 , TARGET_STRING (
"wheel_leg/Saturation5" ) , TARGET_STRING ( "" ) , 0 , 0 , 0 , 0 , 0 } , { 18
, 0 , TARGET_STRING ( "wheel_leg/Saturation7" ) , TARGET_STRING ( "" ) , 0 ,
0 , 0 , 0 , 0 } , { 19 , 0 , TARGET_STRING ( "wheel_leg/Saturation8" ) ,
TARGET_STRING ( "" ) , 0 , 0 , 0 , 0 , 0 } , { 20 , 0 , TARGET_STRING (
"wheel_leg/Sum13" ) , TARGET_STRING ( "" ) , 0 , 0 , 0 , 0 , 0 } , { 21 , 0 ,
TARGET_STRING ( "wheel_leg/Sum14" ) , TARGET_STRING ( "" ) , 0 , 0 , 0 , 0 ,
0 } , { 22 , 0 , TARGET_STRING ( "wheel_leg/Sum15" ) , TARGET_STRING ( "" ) ,
0 , 0 , 0 , 0 , 0 } , { 23 , 0 , TARGET_STRING ( "wheel_leg/Sum16" ) ,
TARGET_STRING ( "" ) , 0 , 0 , 0 , 0 , 0 } , { 24 , 0 , TARGET_STRING (
"wheel_leg/Sum17" ) , TARGET_STRING ( "" ) , 0 , 0 , 0 , 0 , 0 } , { 25 , 0 ,
TARGET_STRING ( "wheel_leg/Sum18" ) , TARGET_STRING ( "" ) , 0 , 0 , 0 , 0 ,
0 } , { 26 , 0 , TARGET_STRING ( "wheel_leg/Sum20" ) , TARGET_STRING ( "" ) ,
0 , 0 , 0 , 0 , 0 } , { 27 , 0 , TARGET_STRING ( "wheel_leg/Sum22" ) ,
TARGET_STRING ( "speed_set" ) , 0 , 0 , 0 , 0 , 1 } , { 28 , 0 ,
TARGET_STRING ( "wheel_leg/Sum23" ) , TARGET_STRING ( "" ) , 0 , 0 , 0 , 0 ,
0 } , { 29 , 0 , TARGET_STRING ( "wheel_leg/Sum24" ) , TARGET_STRING ( "" ) ,
0 , 0 , 0 , 0 , 0 } , { 30 , 0 , TARGET_STRING ( "wheel_leg/Sum25" ) ,
TARGET_STRING ( "" ) , 0 , 0 , 0 , 0 , 0 } , { 31 , 0 , TARGET_STRING (
"wheel_leg/Sum26" ) , TARGET_STRING ( "" ) , 0 , 0 , 0 , 0 , 0 } , { 32 , 0 ,
TARGET_STRING ( "wheel_leg/Sum27" ) , TARGET_STRING ( "" ) , 0 , 0 , 0 , 0 ,
0 } , { 33 , 0 , TARGET_STRING ( "wheel_leg/Sum29" ) , TARGET_STRING ( "" ) ,
0 , 0 , 0 , 0 , 0 } , { 34 , 0 , TARGET_STRING (
"wheel_leg/Subsystem1/Gain23" ) , TARGET_STRING ( "" ) , 0 , 0 , 1 , 0 , 0 }
, { 35 , 1 , TARGET_STRING ( "wheel_leg/Subsystem4/MATLAB Function10" ) ,
TARGET_STRING ( "" ) , 0 , 0 , 0 , 0 , 0 } , { 36 , 0 , TARGET_STRING (
"wheel_leg/calc/Gain23" ) , TARGET_STRING ( "" ) , 0 , 0 , 1 , 0 , 0 } , { 37
, 6 , TARGET_STRING ( "wheel_leg/control/Subsystem" ) , TARGET_STRING ( "" )
, 0 , 0 , 0 , 0 , 2 } , { 38 , 6 , TARGET_STRING (
"wheel_leg/control/Subsystem" ) , TARGET_STRING ( "" ) , 1 , 0 , 0 , 0 , 2 }
, { 39 , 0 , TARGET_STRING ( "wheel_leg/jump_control/Sum1" ) , TARGET_STRING
( "" ) , 0 , 0 , 0 , 0 , 1 } , { 40 , 0 , TARGET_STRING (
"wheel_leg/jump_control/Sum2" ) , TARGET_STRING ( "" ) , 0 , 0 , 0 , 0 , 1 }
, { 41 , 0 , TARGET_STRING ( "wheel_leg/jump_control/Sum3" ) , TARGET_STRING
( "" ) , 0 , 0 , 0 , 0 , 0 } , { 42 , 0 , TARGET_STRING (
"wheel_leg/jump_control/Sum4" ) , TARGET_STRING ( "" ) , 0 , 0 , 0 , 0 , 0 }
, { 43 , 0 , TARGET_STRING ( "wheel_leg/jump_control/Sum6" ) , TARGET_STRING
( "" ) , 0 , 0 , 0 , 0 , 1 } , { 44 , 0 , TARGET_STRING (
"wheel_leg/jump_control/Sum8" ) , TARGET_STRING ( "" ) , 0 , 0 , 0 , 0 , 1 }
, { 45 , 6 , TARGET_STRING ( "wheel_leg/control/Subsystem/MATLAB Function6" )
, TARGET_STRING ( "" ) , 0 , 0 , 0 , 0 , 2 } , { 46 , 6 , TARGET_STRING (
"wheel_leg/control/Subsystem/MATLAB Function6" ) , TARGET_STRING ( "" ) , 1 ,
0 , 0 , 0 , 2 } , { 47 , 0 , TARGET_STRING (
"wheel_leg/PID Controller2/D Gain/Internal Parameters/Derivative Gain" ) ,
TARGET_STRING ( "" ) , 0 , 0 , 0 , 0 , 0 } , { 48 , 0 , TARGET_STRING (
"wheel_leg/PID Controller2/Filter/Cont. Filter/Filter" ) , TARGET_STRING ( ""
) , 0 , 0 , 0 , 0 , 0 } , { 49 , 0 , TARGET_STRING (
"wheel_leg/PID Controller2/Filter/Cont. Filter/SumD" ) , TARGET_STRING ( "" )
, 0 , 0 , 0 , 0 , 0 } , { 50 , 0 , TARGET_STRING (
"wheel_leg/PID Controller2/I Gain/Internal Parameters/Integral Gain" ) ,
TARGET_STRING ( "" ) , 0 , 0 , 0 , 0 , 0 } , { 51 , 0 , TARGET_STRING (
"wheel_leg/PID Controller2/Integrator/Continuous/Integrator" ) ,
TARGET_STRING ( "" ) , 0 , 0 , 0 , 0 , 0 } , { 52 , 0 , TARGET_STRING (
"wheel_leg/PID Controller2/N Gain/Internal Parameters/Filter Coefficient" ) ,
TARGET_STRING ( "" ) , 0 , 0 , 0 , 0 , 0 } , { 53 , 0 , TARGET_STRING (
"wheel_leg/PID Controller3/D Gain/Internal Parameters/Derivative Gain" ) ,
TARGET_STRING ( "" ) , 0 , 0 , 0 , 0 , 0 } , { 54 , 0 , TARGET_STRING (
"wheel_leg/PID Controller3/Filter/Cont. Filter/Filter" ) , TARGET_STRING ( ""
) , 0 , 0 , 0 , 0 , 0 } , { 55 , 0 , TARGET_STRING (
"wheel_leg/PID Controller3/Filter/Cont. Filter/SumD" ) , TARGET_STRING ( "" )
, 0 , 0 , 0 , 0 , 0 } , { 56 , 0 , TARGET_STRING (
"wheel_leg/PID Controller3/I Gain/Internal Parameters/Integral Gain" ) ,
TARGET_STRING ( "" ) , 0 , 0 , 0 , 0 , 0 } , { 57 , 0 , TARGET_STRING (
"wheel_leg/PID Controller3/Integrator/Continuous/Integrator" ) ,
TARGET_STRING ( "" ) , 0 , 0 , 0 , 0 , 0 } , { 58 , 0 , TARGET_STRING (
"wheel_leg/PID Controller3/N Gain/Internal Parameters/Filter Coefficient" ) ,
TARGET_STRING ( "" ) , 0 , 0 , 0 , 0 , 0 } , { 59 , 0 , TARGET_STRING (
 "wheel_leg/PID Controller3/Parallel P Gain/Internal Parameters/Proportional Gain"
) , TARGET_STRING ( "" ) , 0 , 0 , 0 , 0 , 0 } , { 60 , 0 , TARGET_STRING (
"wheel_leg/PID Controller3/Sum/Sum_PID/Sum" ) , TARGET_STRING ( "" ) , 0 , 0
, 0 , 0 , 0 } , { 61 , 0 , TARGET_STRING (
"wheel_leg/blance_leg_model/Solver Configuration1/EVAL_KEY/INPUT_1_1_1" ) ,
TARGET_STRING ( "" ) , 0 , 0 , 2 , 0 , 0 } , { 62 , 0 , TARGET_STRING (
"wheel_leg/blance_leg_model/Solver Configuration1/EVAL_KEY/INPUT_2_1_1" ) ,
TARGET_STRING ( "" ) , 0 , 0 , 2 , 0 , 0 } , { 63 , 0 , TARGET_STRING (
"wheel_leg/blance_leg_model/Solver Configuration1/EVAL_KEY/INPUT_3_1_1" ) ,
TARGET_STRING ( "" ) , 0 , 0 , 2 , 0 , 0 } , { 64 , 0 , TARGET_STRING (
"wheel_leg/blance_leg_model/Solver Configuration1/EVAL_KEY/INPUT_4_1_1" ) ,
TARGET_STRING ( "" ) , 0 , 0 , 2 , 0 , 0 } , { 65 , 0 , TARGET_STRING (
"wheel_leg/blance_leg_model/Solver Configuration1/EVAL_KEY/INPUT_5_1_1" ) ,
TARGET_STRING ( "" ) , 0 , 0 , 2 , 0 , 0 } , { 66 , 0 , TARGET_STRING (
"wheel_leg/blance_leg_model/Solver Configuration1/EVAL_KEY/INPUT_6_1_1" ) ,
TARGET_STRING ( "" ) , 0 , 0 , 2 , 0 , 0 } , { 67 , 0 , TARGET_STRING (
"wheel_leg/blance_leg_model/Solver Configuration1/EVAL_KEY/OUTPUT_1_0" ) ,
TARGET_STRING ( "" ) , 0 , 0 , 3 , 0 , 0 } , { 68 , 0 , TARGET_STRING (
"wheel_leg/blance_leg_model/Solver Configuration1/EVAL_KEY/STATE_1" ) ,
TARGET_STRING ( "" ) , 0 , 0 , 4 , 0 , 0 } , { 0 , 0 , ( NULL ) , ( NULL ) ,
0 , 0 , 0 , 0 , 0 } } ; static const rtwCAPI_BlockParameters
rtBlockParameters [ ] = { { 69 , TARGET_STRING ( "wheel_leg/PID Controller2"
) , TARGET_STRING ( "P" ) , 0 , 0 , 0 } , { 70 , TARGET_STRING (
"wheel_leg/PID Controller2" ) , TARGET_STRING ( "I" ) , 0 , 0 , 0 } , { 71 ,
TARGET_STRING ( "wheel_leg/PID Controller2" ) , TARGET_STRING ( "D" ) , 0 , 0
, 0 } , { 72 , TARGET_STRING ( "wheel_leg/PID Controller2" ) , TARGET_STRING
( "N" ) , 0 , 0 , 0 } , { 73 , TARGET_STRING ( "wheel_leg/PID Controller2" )
, TARGET_STRING ( "InitialConditionForIntegrator" ) , 0 , 0 , 0 } , { 74 ,
TARGET_STRING ( "wheel_leg/PID Controller2" ) , TARGET_STRING (
"InitialConditionForFilter" ) , 0 , 0 , 0 } , { 75 , TARGET_STRING (
"wheel_leg/PID Controller3" ) , TARGET_STRING ( "P" ) , 0 , 0 , 0 } , { 76 ,
TARGET_STRING ( "wheel_leg/PID Controller3" ) , TARGET_STRING ( "I" ) , 0 , 0
, 0 } , { 77 , TARGET_STRING ( "wheel_leg/PID Controller3" ) , TARGET_STRING
( "D" ) , 0 , 0 , 0 } , { 78 , TARGET_STRING ( "wheel_leg/PID Controller3" )
, TARGET_STRING ( "N" ) , 0 , 0 , 0 } , { 79 , TARGET_STRING (
"wheel_leg/PID Controller3" ) , TARGET_STRING (
"InitialConditionForIntegrator" ) , 0 , 0 , 0 } , { 80 , TARGET_STRING (
"wheel_leg/PID Controller3" ) , TARGET_STRING ( "InitialConditionForFilter" )
, 0 , 0 , 0 } , { 81 , TARGET_STRING ( "wheel_leg/jump_control" ) ,
TARGET_STRING ( "jump_time" ) , 0 , 0 , 0 } , { 82 , TARGET_STRING (
"wheel_leg/Constant" ) , TARGET_STRING ( "Value" ) , 0 , 0 , 0 } , { 83 ,
TARGET_STRING ( "wheel_leg/Constant1" ) , TARGET_STRING ( "Value" ) , 0 , 0 ,
0 } , { 84 , TARGET_STRING ( "wheel_leg/Constant10" ) , TARGET_STRING (
"Value" ) , 0 , 0 , 0 } , { 85 , TARGET_STRING ( "wheel_leg/Constant12" ) ,
TARGET_STRING ( "Value" ) , 0 , 0 , 0 } , { 86 , TARGET_STRING (
"wheel_leg/除去轮子后机体重量" ) , TARGET_STRING ( "Value" ) , 0 , 0 , 0 } ,
{ 87 , TARGET_STRING ( "wheel_leg/除去轮子后机体重量，" ) , TARGET_STRING (
"Value" ) , 0 , 0 , 0 } , { 88 , TARGET_STRING ( "wheel_leg/Gain22" ) ,
TARGET_STRING ( "Gain" ) , 0 , 0 , 0 } , { 89 , TARGET_STRING (
"wheel_leg/Gain24" ) , TARGET_STRING ( "Gain" ) , 0 , 0 , 0 } , { 90 ,
TARGET_STRING ( "wheel_leg/Gain25" ) , TARGET_STRING ( "Gain" ) , 0 , 0 , 0 }
, { 91 , TARGET_STRING ( "wheel_leg/Gain26" ) , TARGET_STRING ( "Gain" ) , 0
, 0 , 0 } , { 92 , TARGET_STRING ( "wheel_leg/Gain29" ) , TARGET_STRING (
"Gain" ) , 0 , 0 , 0 } , { 93 , TARGET_STRING ( "wheel_leg/Gain33" ) ,
TARGET_STRING ( "Gain" ) , 0 , 0 , 0 } , { 94 , TARGET_STRING (
"wheel_leg/Gain36" ) , TARGET_STRING ( "Gain" ) , 0 , 0 , 0 } , { 95 ,
TARGET_STRING ( "wheel_leg/Gain38" ) , TARGET_STRING ( "Gain" ) , 0 , 0 , 0 }
, { 96 , TARGET_STRING ( "wheel_leg/Gain40" ) , TARGET_STRING ( "Gain" ) , 0
, 0 , 0 } , { 97 , TARGET_STRING ( "wheel_leg/Gain41" ) , TARGET_STRING (
"Gain" ) , 0 , 0 , 0 } , { 98 , TARGET_STRING ( "wheel_leg/Integrator" ) ,
TARGET_STRING ( "InitialCondition" ) , 0 , 0 , 0 } , { 99 , TARGET_STRING (
"wheel_leg/Integrator2" ) , TARGET_STRING ( "InitialCondition" ) , 0 , 0 , 0
} , { 100 , TARGET_STRING ( "wheel_leg/Integrator4" ) , TARGET_STRING (
"InitialCondition" ) , 0 , 0 , 0 } , { 101 , TARGET_STRING (
"wheel_leg/Saturation1" ) , TARGET_STRING ( "UpperLimit" ) , 0 , 0 , 0 } , {
102 , TARGET_STRING ( "wheel_leg/Saturation1" ) , TARGET_STRING (
"LowerLimit" ) , 0 , 0 , 0 } , { 103 , TARGET_STRING (
"wheel_leg/Saturation2" ) , TARGET_STRING ( "UpperLimit" ) , 0 , 0 , 0 } , {
104 , TARGET_STRING ( "wheel_leg/Saturation2" ) , TARGET_STRING (
"LowerLimit" ) , 0 , 0 , 0 } , { 105 , TARGET_STRING (
"wheel_leg/Saturation3" ) , TARGET_STRING ( "UpperLimit" ) , 0 , 0 , 0 } , {
106 , TARGET_STRING ( "wheel_leg/Saturation3" ) , TARGET_STRING (
"LowerLimit" ) , 0 , 0 , 0 } , { 107 , TARGET_STRING (
"wheel_leg/Saturation4" ) , TARGET_STRING ( "UpperLimit" ) , 0 , 0 , 0 } , {
108 , TARGET_STRING ( "wheel_leg/Saturation4" ) , TARGET_STRING (
"LowerLimit" ) , 0 , 0 , 0 } , { 109 , TARGET_STRING (
"wheel_leg/Saturation5" ) , TARGET_STRING ( "UpperLimit" ) , 0 , 0 , 0 } , {
110 , TARGET_STRING ( "wheel_leg/Saturation5" ) , TARGET_STRING (
"LowerLimit" ) , 0 , 0 , 0 } , { 111 , TARGET_STRING (
"wheel_leg/Saturation6" ) , TARGET_STRING ( "UpperLimit" ) , 0 , 0 , 0 } , {
112 , TARGET_STRING ( "wheel_leg/Saturation6" ) , TARGET_STRING (
"LowerLimit" ) , 0 , 0 , 0 } , { 113 , TARGET_STRING (
"wheel_leg/Saturation7" ) , TARGET_STRING ( "UpperLimit" ) , 0 , 0 , 0 } , {
114 , TARGET_STRING ( "wheel_leg/Saturation7" ) , TARGET_STRING (
"LowerLimit" ) , 0 , 0 , 0 } , { 115 , TARGET_STRING (
"wheel_leg/Saturation8" ) , TARGET_STRING ( "UpperLimit" ) , 0 , 0 , 0 } , {
116 , TARGET_STRING ( "wheel_leg/Saturation8" ) , TARGET_STRING (
"LowerLimit" ) , 0 , 0 , 0 } , { 117 , TARGET_STRING ( "wheel_leg/Step2" ) ,
TARGET_STRING ( "Time" ) , 0 , 0 , 0 } , { 118 , TARGET_STRING (
"wheel_leg/Step2" ) , TARGET_STRING ( "Before" ) , 0 , 0 , 0 } , { 119 ,
TARGET_STRING ( "wheel_leg/Step2" ) , TARGET_STRING ( "After" ) , 0 , 0 , 0 }
, { 120 , TARGET_STRING ( "wheel_leg/Step3" ) , TARGET_STRING ( "Time" ) , 0
, 0 , 0 } , { 121 , TARGET_STRING ( "wheel_leg/Step3" ) , TARGET_STRING (
"Before" ) , 0 , 0 , 0 } , { 122 , TARGET_STRING ( "wheel_leg/Step3" ) ,
TARGET_STRING ( "After" ) , 0 , 0 , 0 } , { 123 , TARGET_STRING (
"wheel_leg/jump_control/Step" ) , TARGET_STRING ( "Before" ) , 0 , 0 , 0 } ,
{ 124 , TARGET_STRING ( "wheel_leg/jump_control/Step" ) , TARGET_STRING (
"After" ) , 0 , 0 , 0 } , { 125 , TARGET_STRING (
"wheel_leg/jump_control/Step1" ) , TARGET_STRING ( "Before" ) , 0 , 0 , 0 } ,
{ 126 , TARGET_STRING ( "wheel_leg/jump_control/Step1" ) , TARGET_STRING (
"After" ) , 0 , 0 , 0 } , { 127 , TARGET_STRING (
"wheel_leg/jump_control/Step10" ) , TARGET_STRING ( "Before" ) , 0 , 0 , 0 }
, { 128 , TARGET_STRING ( "wheel_leg/jump_control/Step10" ) , TARGET_STRING (
"After" ) , 0 , 0 , 0 } , { 129 , TARGET_STRING (
"wheel_leg/jump_control/Step11" ) , TARGET_STRING ( "Before" ) , 0 , 0 , 0 }
, { 130 , TARGET_STRING ( "wheel_leg/jump_control/Step11" ) , TARGET_STRING (
"After" ) , 0 , 0 , 0 } , { 131 , TARGET_STRING (
"wheel_leg/jump_control/Step12" ) , TARGET_STRING ( "Before" ) , 0 , 0 , 0 }
, { 132 , TARGET_STRING ( "wheel_leg/jump_control/Step12" ) , TARGET_STRING (
"After" ) , 0 , 0 , 0 } , { 133 , TARGET_STRING (
"wheel_leg/jump_control/Step13" ) , TARGET_STRING ( "Before" ) , 0 , 0 , 0 }
, { 134 , TARGET_STRING ( "wheel_leg/jump_control/Step13" ) , TARGET_STRING (
"After" ) , 0 , 0 , 0 } , { 135 , TARGET_STRING (
"wheel_leg/jump_control/Step14" ) , TARGET_STRING ( "Before" ) , 0 , 0 , 0 }
, { 136 , TARGET_STRING ( "wheel_leg/jump_control/Step14" ) , TARGET_STRING (
"After" ) , 0 , 0 , 0 } , { 137 , TARGET_STRING (
"wheel_leg/jump_control/Step15" ) , TARGET_STRING ( "Before" ) , 0 , 0 , 0 }
, { 138 , TARGET_STRING ( "wheel_leg/jump_control/Step15" ) , TARGET_STRING (
"After" ) , 0 , 0 , 0 } , { 139 , TARGET_STRING (
"wheel_leg/jump_control/Step16" ) , TARGET_STRING ( "Before" ) , 0 , 0 , 0 }
, { 140 , TARGET_STRING ( "wheel_leg/jump_control/Step16" ) , TARGET_STRING (
"After" ) , 0 , 0 , 0 } , { 141 , TARGET_STRING (
"wheel_leg/jump_control/Step17" ) , TARGET_STRING ( "Before" ) , 0 , 0 , 0 }
, { 142 , TARGET_STRING ( "wheel_leg/jump_control/Step17" ) , TARGET_STRING (
"After" ) , 0 , 0 , 0 } , { 143 , TARGET_STRING (
"wheel_leg/jump_control/Step4" ) , TARGET_STRING ( "Before" ) , 0 , 0 , 0 } ,
{ 144 , TARGET_STRING ( "wheel_leg/jump_control/Step4" ) , TARGET_STRING (
"After" ) , 0 , 0 , 0 } , { 145 , TARGET_STRING (
"wheel_leg/jump_control/Step5" ) , TARGET_STRING ( "Before" ) , 0 , 0 , 0 } ,
{ 146 , TARGET_STRING ( "wheel_leg/jump_control/Step5" ) , TARGET_STRING (
"After" ) , 0 , 0 , 0 } , { 147 , TARGET_STRING (
"wheel_leg/jump_control/Step6" ) , TARGET_STRING ( "Before" ) , 0 , 0 , 0 } ,
{ 148 , TARGET_STRING ( "wheel_leg/jump_control/Step6" ) , TARGET_STRING (
"After" ) , 0 , 0 , 0 } , { 149 , TARGET_STRING (
"wheel_leg/jump_control/Step7" ) , TARGET_STRING ( "Before" ) , 0 , 0 , 0 } ,
{ 150 , TARGET_STRING ( "wheel_leg/jump_control/Step7" ) , TARGET_STRING (
"After" ) , 0 , 0 , 0 } , { 151 , TARGET_STRING (
"wheel_leg/jump_control/Step8" ) , TARGET_STRING ( "Before" ) , 0 , 0 , 0 } ,
{ 152 , TARGET_STRING ( "wheel_leg/jump_control/Step8" ) , TARGET_STRING (
"After" ) , 0 , 0 , 0 } , { 153 , TARGET_STRING (
"wheel_leg/jump_control/Step9" ) , TARGET_STRING ( "Before" ) , 0 , 0 , 0 } ,
{ 154 , TARGET_STRING ( "wheel_leg/jump_control/Step9" ) , TARGET_STRING (
"After" ) , 0 , 0 , 0 } , { 155 , TARGET_STRING (
"wheel_leg/control/Subsystem/T1" ) , TARGET_STRING ( "InitialOutput" ) , 0 ,
0 , 0 } , { 156 , TARGET_STRING ( "wheel_leg/control/Subsystem/T2" ) ,
TARGET_STRING ( "InitialOutput" ) , 0 , 0 , 0 } , { 0 , ( NULL ) , ( NULL ) ,
0 , 0 , 0 } } ; static const rtwCAPI_Signals rtRootInputs [ ] = { { 0 , 0 , (
NULL ) , ( NULL ) , 0 , 0 , 0 , 0 , 0 } } ; static const rtwCAPI_Signals
rtRootOutputs [ ] = { { 0 , 0 , ( NULL ) , ( NULL ) , 0 , 0 , 0 , 0 , 0 } } ;
static const rtwCAPI_ModelParameters rtModelParameters [ ] = { { 157 ,
TARGET_STRING ( "K" ) , 0 , 5 , 0 } , { 0 , ( NULL ) , 0 , 0 , 0 } } ;
#ifndef HOST_CAPI_BUILD
static void * rtDataAddrMap [ ] = { & rtB . a5rxjeqmnm , & rtB . ewa15giale ,
& rtB . oqhhk34uis , & rtB . nwuie3h1zj , & rtB . nxxivmzdo0 , & rtB .
fx0alrhetp , & rtB . d4fftorabg , & rtB . cufla4blkp , & rtB . cfmoived0b , &
rtB . kma1dpwhhf , & rtB . o0acnzn2gn , & rtB . k3exkay1ay , & rtB .
chcwz4ujvj , & rtB . hdeqxnl10m , & rtB . djd0cbchne , & rtB . hl2vsnjvah , &
rtB . hfyfla4srx , & rtB . bpbscqkzum , & rtB . affemz5v0v , & rtB .
amehlztnnv , & rtB . jsqo11gz4n , & rtB . liiv3zehzs , & rtB . hkrrrfopbq , &
rtB . nkvqeuwnam , & rtB . c4gioprvct , & rtB . b0mlwzzkzn , & rtB .
fvygvwpi1d , & rtB . fviua014cu , & rtB . cp1jjx2a4p , & rtB . lp2kjlaox2 , &
rtB . nlnzoie4ft , & rtB . kaed4psp05 , & rtB . oaaq01l5fu , & rtB .
emby3igyk1 , & rtB . kourgu0gp0 [ 0 ] , & rtB . p1iy05u0ee , & rtB .
m0bxxy51bt [ 0 ] , & rtB . bdqfv1d2cw , & rtB . lbhtwulbcm , & rtB .
ct5ptiiqfm , & rtB . egyva5trzl , & rtB . luhg3pfwco , & rtB . mj2xnu20kl , &
rtB . fasi4utync , & rtB . hhykaoe4za , & rtB . bdqfv1d2cw , & rtB .
lbhtwulbcm , & rtB . furrbhkomt , & rtB . mrjzncw5q4 , & rtB . imk0sd2jrm , &
rtB . jteoenmoec , & rtB . jw4ohhvqrt , & rtB . f1443sccd2 , & rtB .
ngl2rc54wx , & rtB . mkoprhigc1 , & rtB . p4cj0opfml , & rtB . jfiugehczg , &
rtB . bibxaqp42j , & rtB . kxvii2moqu , & rtB . lunhybn4j1 , & rtB .
dz5u3bubv3 , & rtB . jl2dsktpay [ 0 ] , & rtB . l1345vcjcj [ 0 ] , & rtB .
clrzjaih1v [ 0 ] , & rtB . ludhxwcsap [ 0 ] , & rtB . dmfybrfnod [ 0 ] , &
rtB . nik4ggyist [ 0 ] , & rtB . ne53mrvzfq [ 0 ] , & rtB . eh2cmnfpcd [ 0 ]
, & rtP . PIDController2_P , & rtP . PIDController2_I , & rtP .
PIDController2_D , & rtP . PIDController2_N , & rtP .
PIDController2_InitialConditionForIntegrator , & rtP .
PIDController2_InitialConditionForFilter , & rtP . PIDController3_P , & rtP .
PIDController3_I , & rtP . PIDController3_D , & rtP . PIDController3_N , &
rtP . PIDController3_InitialConditionForIntegrator , & rtP .
PIDController3_InitialConditionForFilter , & rtP . jump_control_jump_time , &
rtP . Constant_Value , & rtP . Constant1_Value , & rtP . Constant10_Value , &
rtP . Constant12_Value , & rtP . Value , & rtP . _Value , & rtP . Gain22_Gain
, & rtP . Gain24_Gain , & rtP . Gain25_Gain , & rtP . Gain26_Gain , & rtP .
Gain29_Gain , & rtP . Gain33_Gain , & rtP . Gain36_Gain , & rtP . Gain38_Gain
, & rtP . Gain40_Gain , & rtP . Gain41_Gain , & rtP . Integrator_IC , & rtP .
Integrator2_IC , & rtP . Integrator4_IC , & rtP . Saturation1_UpperSat , &
rtP . Saturation1_LowerSat , & rtP . Saturation2_UpperSat , & rtP .
Saturation2_LowerSat , & rtP . Saturation3_UpperSat , & rtP .
Saturation3_LowerSat , & rtP . Saturation4_UpperSat , & rtP .
Saturation4_LowerSat , & rtP . Saturation5_UpperSat , & rtP .
Saturation5_LowerSat , & rtP . Saturation6_UpperSat , & rtP .
Saturation6_LowerSat , & rtP . Saturation7_UpperSat , & rtP .
Saturation7_LowerSat , & rtP . Saturation8_UpperSat , & rtP .
Saturation8_LowerSat , & rtP . Step2_Time , & rtP . Step2_Y0 , & rtP .
Step2_YFinal , & rtP . Step3_Time , & rtP . Step3_Y0 , & rtP . Step3_YFinal ,
& rtP . Step_Y0 , & rtP . Step_YFinal , & rtP . Step1_Y0 , & rtP .
Step1_YFinal , & rtP . Step10_Y0 , & rtP . Step10_YFinal , & rtP . Step11_Y0
, & rtP . Step11_YFinal , & rtP . Step12_Y0 , & rtP . Step12_YFinal , & rtP .
Step13_Y0 , & rtP . Step13_YFinal , & rtP . Step14_Y0 , & rtP . Step14_YFinal
, & rtP . Step15_Y0 , & rtP . Step15_YFinal , & rtP . Step16_Y0 , & rtP .
Step16_YFinal , & rtP . Step17_Y0 , & rtP . Step17_YFinal , & rtP . Step4_Y0
, & rtP . Step4_YFinal , & rtP . Step5_Y0 , & rtP . Step5_YFinal , & rtP .
Step6_Y0 , & rtP . Step6_YFinal , & rtP . Step7_Y0 , & rtP . Step7_YFinal , &
rtP . Step8_Y0 , & rtP . Step8_YFinal , & rtP . Step9_Y0 , & rtP .
Step9_YFinal , & rtP . T1_Y0 , & rtP . T2_Y0 , & rtP . K [ 0 ] , } ; static
int32_T * rtVarDimsAddrMap [ ] = { ( NULL ) } ;
#endif
static TARGET_CONST rtwCAPI_DataTypeMap rtDataTypeMap [ ] = { { "double" ,
"real_T" , 0 , 0 , sizeof ( real_T ) , SS_DOUBLE , 0 , 0 , 0 } } ;
#ifdef HOST_CAPI_BUILD
#undef sizeof
#endif
static TARGET_CONST rtwCAPI_ElementMap rtElementMap [ ] = { { ( NULL ) , 0 ,
0 , 0 , 0 } , } ; static const rtwCAPI_DimensionMap rtDimensionMap [ ] = { {
rtwCAPI_SCALAR , 0 , 2 , 0 } , { rtwCAPI_VECTOR , 2 , 2 , 0 } , {
rtwCAPI_VECTOR , 4 , 2 , 0 } , { rtwCAPI_VECTOR , 6 , 2 , 0 } , {
rtwCAPI_VECTOR , 8 , 2 , 0 } , { rtwCAPI_MATRIX_COL_MAJOR , 10 , 2 , 0 } } ;
static const uint_T rtDimensionArray [ ] = { 1 , 1 , 2 , 1 , 4 , 1 , 13 , 1 ,
32 , 1 , 2 , 6 } ; static const real_T rtcapiStoredFloats [ ] = { 0.0 , 1.0 }
; static const rtwCAPI_FixPtMap rtFixPtMap [ ] = { { ( NULL ) , ( NULL ) ,
rtwCAPI_FIX_RESERVED , 0 , 0 , 0 } , } ; static const rtwCAPI_SampleTimeMap
rtSampleTimeMap [ ] = { { ( const void * ) & rtcapiStoredFloats [ 0 ] , (
const void * ) & rtcapiStoredFloats [ 0 ] , 0 , 0 } , { ( const void * ) &
rtcapiStoredFloats [ 0 ] , ( const void * ) & rtcapiStoredFloats [ 1 ] , 1 ,
0 } , { ( NULL ) , ( NULL ) , - 1 , 0 } } ; static
rtwCAPI_ModelMappingStaticInfo mmiStatic = { { rtBlockSignals , 69 ,
rtRootInputs , 0 , rtRootOutputs , 0 } , { rtBlockParameters , 88 ,
rtModelParameters , 1 } , { ( NULL ) , 0 } , { rtDataTypeMap , rtDimensionMap
, rtFixPtMap , rtElementMap , rtSampleTimeMap , rtDimensionArray } , "float"
, { 3625382941U , 3387521915U , 3494444731U , 3437323851U } , ( NULL ) , 0 ,
0 } ; const rtwCAPI_ModelMappingStaticInfo * wheel_leg_GetCAPIStaticMap (
void ) { return & mmiStatic ; }
#ifndef HOST_CAPI_BUILD
void wheel_leg_InitializeDataMapInfo ( void ) { rtwCAPI_SetVersion ( ( *
rt_dataMapInfoPtr ) . mmi , 1 ) ; rtwCAPI_SetStaticMap ( ( *
rt_dataMapInfoPtr ) . mmi , & mmiStatic ) ; rtwCAPI_SetLoggingStaticMap ( ( *
rt_dataMapInfoPtr ) . mmi , ( NULL ) ) ; rtwCAPI_SetDataAddressMap ( ( *
rt_dataMapInfoPtr ) . mmi , rtDataAddrMap ) ; rtwCAPI_SetVarDimsAddressMap (
( * rt_dataMapInfoPtr ) . mmi , rtVarDimsAddrMap ) ;
rtwCAPI_SetInstanceLoggingInfo ( ( * rt_dataMapInfoPtr ) . mmi , ( NULL ) ) ;
rtwCAPI_SetChildMMIArray ( ( * rt_dataMapInfoPtr ) . mmi , ( NULL ) ) ;
rtwCAPI_SetChildMMIArrayLen ( ( * rt_dataMapInfoPtr ) . mmi , 0 ) ; }
#else
#ifdef __cplusplus
extern "C" {
#endif
void wheel_leg_host_InitializeDataMapInfo ( wheel_leg_host_DataMapInfo_T *
dataMap , const char * path ) { rtwCAPI_SetVersion ( dataMap -> mmi , 1 ) ;
rtwCAPI_SetStaticMap ( dataMap -> mmi , & mmiStatic ) ;
rtwCAPI_SetDataAddressMap ( dataMap -> mmi , NULL ) ;
rtwCAPI_SetVarDimsAddressMap ( dataMap -> mmi , NULL ) ; rtwCAPI_SetPath (
dataMap -> mmi , path ) ; rtwCAPI_SetFullPath ( dataMap -> mmi , NULL ) ;
rtwCAPI_SetChildMMIArray ( dataMap -> mmi , ( NULL ) ) ;
rtwCAPI_SetChildMMIArrayLen ( dataMap -> mmi , 0 ) ; }
#ifdef __cplusplus
}
#endif
#endif
