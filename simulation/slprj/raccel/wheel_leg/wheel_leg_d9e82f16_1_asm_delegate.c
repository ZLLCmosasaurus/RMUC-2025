#include <math.h>
#include <string.h>
#include "pm_std.h"
#include "sm_std.h"
#include "ne_std.h"
#include "ne_dae.h"
#include "sm_ssci_run_time_errors.h"
#include "sm_RuntimeDerivedValuesBundle.h"
#include "sm_CTarget.h"
void wheel_leg_d9e82f16_1_setTargets ( const RuntimeDerivedValuesBundle *
rtdv , CTarget * targets ) { ( void ) rtdv ; ( void ) targets ; } void
wheel_leg_d9e82f16_1_resetAsmStateVector ( const void * mech , double * state
) { double xx [ 1 ] ; ( void ) mech ; xx [ 0 ] = 0.0 ; state [ 0 ] = xx [ 0 ]
; state [ 1 ] = xx [ 0 ] ; state [ 2 ] = xx [ 0 ] ; state [ 3 ] = xx [ 0 ] ;
state [ 4 ] = xx [ 0 ] ; state [ 5 ] = xx [ 0 ] ; state [ 6 ] = xx [ 0 ] ;
state [ 7 ] = xx [ 0 ] ; state [ 8 ] = xx [ 0 ] ; state [ 9 ] = xx [ 0 ] ;
state [ 10 ] = xx [ 0 ] ; state [ 11 ] = xx [ 0 ] ; state [ 12 ] = xx [ 0 ] ;
state [ 13 ] = xx [ 0 ] ; state [ 14 ] = xx [ 0 ] ; state [ 15 ] = xx [ 0 ] ;
state [ 16 ] = xx [ 0 ] ; state [ 17 ] = xx [ 0 ] ; state [ 18 ] = xx [ 0 ] ;
state [ 19 ] = xx [ 0 ] ; state [ 20 ] = xx [ 0 ] ; state [ 21 ] = xx [ 0 ] ;
state [ 22 ] = xx [ 0 ] ; state [ 23 ] = xx [ 0 ] ; state [ 24 ] = xx [ 0 ] ;
state [ 25 ] = xx [ 0 ] ; state [ 26 ] = xx [ 0 ] ; state [ 27 ] = xx [ 0 ] ;
state [ 28 ] = xx [ 0 ] ; state [ 29 ] = xx [ 0 ] ; state [ 30 ] = xx [ 0 ] ;
state [ 31 ] = xx [ 0 ] ; } void
wheel_leg_d9e82f16_1_initializeTrackedAngleState ( const void * mech , const
RuntimeDerivedValuesBundle * rtdv , const int * modeVector , const double *
motionData , double * state ) { const double * rtdvd = rtdv -> mDoubles .
mValues ; const int * rtdvi = rtdv -> mInts . mValues ; double xx [ 21 ] ; (
void ) mech ; ( void ) rtdvd ; ( void ) rtdvi ; ( void ) modeVector ; xx [ 0
] = 2.0 ; xx [ 1 ] = motionData [ 42 ] ; xx [ 2 ] = motionData [ 43 ] ; xx [
3 ] = motionData [ 44 ] ; xx [ 4 ] = motionData [ 45 ] ; xx [ 5 ] =
motionData [ 91 ] ; xx [ 6 ] = motionData [ 92 ] ; xx [ 7 ] = motionData [ 93
] ; xx [ 8 ] = motionData [ 94 ] ; pm_math_Quaternion_inverseCompose_ra ( xx
+ 1 , xx + 5 , xx + 9 ) ; xx [ 1 ] = motionData [ 211 ] ; xx [ 2 ] =
motionData [ 212 ] ; xx [ 3 ] = motionData [ 213 ] ;
pm_math_Quaternion_inverseXform_ra ( xx + 9 , xx + 1 , xx + 4 ) ; xx [ 1 ] =
motionData [ 77 ] ; xx [ 2 ] = motionData [ 78 ] ; xx [ 3 ] = motionData [ 79
] ; xx [ 4 ] = motionData [ 80 ] ; xx [ 13 ] = motionData [ 112 ] ; xx [ 14 ]
= motionData [ 113 ] ; xx [ 15 ] = motionData [ 114 ] ; xx [ 16 ] =
motionData [ 115 ] ; pm_math_Quaternion_inverseCompose_ra ( xx + 1 , xx + 13
, xx + 17 ) ; xx [ 1 ] = motionData [ 241 ] ; xx [ 2 ] = motionData [ 242 ] ;
xx [ 3 ] = motionData [ 243 ] ; pm_math_Quaternion_inverseXform_ra ( xx + 17
, xx + 1 , xx + 13 ) ; state [ 28 ] = sm_core_canonicalAngle ( xx [ 0 ] *
atan2 ( sqrt ( xx [ 10 ] * xx [ 10 ] + xx [ 11 ] * xx [ 11 ] + xx [ 12 ] * xx
[ 12 ] ) , fabs ( xx [ 9 ] ) ) * ( ( xx [ 9 ] * xx [ 12 ] ) < 0.0 ? - 1.0 : +
1.0 ) ) ; state [ 29 ] = motionData [ 201 ] - xx [ 6 ] ; state [ 30 ] =
sm_core_canonicalAngle ( xx [ 0 ] * atan2 ( sqrt ( xx [ 18 ] * xx [ 18 ] + xx
[ 19 ] * xx [ 19 ] + xx [ 20 ] * xx [ 20 ] ) , fabs ( xx [ 17 ] ) ) * ( ( xx
[ 17 ] * xx [ 20 ] ) < 0.0 ? - 1.0 : + 1.0 ) ) ; state [ 31 ] = motionData [
231 ] - xx [ 15 ] ; } void wheel_leg_d9e82f16_1_computeDiscreteState ( const
void * mech , const RuntimeDerivedValuesBundle * rtdv , double * state ) {
const double * rtdvd = rtdv -> mDoubles . mValues ; const int * rtdvi = rtdv
-> mInts . mValues ; ( void ) mech ; ( void ) rtdvd ; ( void ) rtdvi ; ( void
) state ; } void wheel_leg_d9e82f16_1_adjustPosition ( const void * mech ,
const double * dofDeltas , double * state ) { ( void ) mech ; state [ 0 ] =
state [ 0 ] + dofDeltas [ 0 ] ; state [ 1 ] = state [ 1 ] + dofDeltas [ 1 ] ;
state [ 2 ] = state [ 2 ] + dofDeltas [ 2 ] ; state [ 6 ] = state [ 6 ] +
dofDeltas [ 3 ] ; state [ 8 ] = state [ 8 ] + dofDeltas [ 4 ] ; state [ 10 ]
= state [ 10 ] + dofDeltas [ 5 ] ; state [ 12 ] = state [ 12 ] + dofDeltas [
6 ] ; state [ 14 ] = state [ 14 ] + dofDeltas [ 7 ] ; state [ 16 ] = state [
16 ] + dofDeltas [ 8 ] ; state [ 18 ] = state [ 18 ] + dofDeltas [ 9 ] ;
state [ 20 ] = state [ 20 ] + dofDeltas [ 10 ] ; state [ 22 ] = state [ 22 ]
+ dofDeltas [ 11 ] ; state [ 24 ] = state [ 24 ] + dofDeltas [ 12 ] ; state [
26 ] = state [ 26 ] + dofDeltas [ 13 ] ; } static void
perturbAsmJointPrimitiveState_0_0 ( double mag , double * state ) { state [ 0
] = state [ 0 ] + mag ; } static void perturbAsmJointPrimitiveState_0_0v (
double mag , double * state ) { state [ 0 ] = state [ 0 ] + mag ; state [ 3 ]
= state [ 3 ] - 0.875 * mag ; } static void perturbAsmJointPrimitiveState_0_1
( double mag , double * state ) { state [ 1 ] = state [ 1 ] + mag ; } static
void perturbAsmJointPrimitiveState_0_1v ( double mag , double * state ) {
state [ 1 ] = state [ 1 ] + mag ; state [ 4 ] = state [ 4 ] - 0.875 * mag ; }
static void perturbAsmJointPrimitiveState_0_2 ( double mag , double * state )
{ state [ 2 ] = state [ 2 ] + mag ; } static void
perturbAsmJointPrimitiveState_0_2v ( double mag , double * state ) { state [
2 ] = state [ 2 ] + mag ; state [ 5 ] = state [ 5 ] - 0.875 * mag ; } static
void perturbAsmJointPrimitiveState_1_0 ( double mag , double * state ) {
state [ 6 ] = state [ 6 ] + mag ; } static void
perturbAsmJointPrimitiveState_1_0v ( double mag , double * state ) { state [
6 ] = state [ 6 ] + mag ; state [ 7 ] = state [ 7 ] - 0.875 * mag ; } static
void perturbAsmJointPrimitiveState_2_0 ( double mag , double * state ) {
state [ 8 ] = state [ 8 ] + mag ; } static void
perturbAsmJointPrimitiveState_2_0v ( double mag , double * state ) { state [
8 ] = state [ 8 ] + mag ; state [ 9 ] = state [ 9 ] - 0.875 * mag ; } static
void perturbAsmJointPrimitiveState_3_0 ( double mag , double * state ) {
state [ 10 ] = state [ 10 ] + mag ; } static void
perturbAsmJointPrimitiveState_3_0v ( double mag , double * state ) { state [
10 ] = state [ 10 ] + mag ; state [ 11 ] = state [ 11 ] - 0.875 * mag ; }
static void perturbAsmJointPrimitiveState_4_0 ( double mag , double * state )
{ state [ 12 ] = state [ 12 ] + mag ; } static void
perturbAsmJointPrimitiveState_4_0v ( double mag , double * state ) { state [
12 ] = state [ 12 ] + mag ; state [ 13 ] = state [ 13 ] - 0.875 * mag ; }
static void perturbAsmJointPrimitiveState_5_0 ( double mag , double * state )
{ state [ 14 ] = state [ 14 ] + mag ; } static void
perturbAsmJointPrimitiveState_5_0v ( double mag , double * state ) { state [
14 ] = state [ 14 ] + mag ; state [ 15 ] = state [ 15 ] - 0.875 * mag ; }
static void perturbAsmJointPrimitiveState_6_0 ( double mag , double * state )
{ state [ 16 ] = state [ 16 ] + mag ; } static void
perturbAsmJointPrimitiveState_6_0v ( double mag , double * state ) { state [
16 ] = state [ 16 ] + mag ; state [ 17 ] = state [ 17 ] - 0.875 * mag ; }
static void perturbAsmJointPrimitiveState_7_0 ( double mag , double * state )
{ state [ 18 ] = state [ 18 ] + mag ; } static void
perturbAsmJointPrimitiveState_7_0v ( double mag , double * state ) { state [
18 ] = state [ 18 ] + mag ; state [ 19 ] = state [ 19 ] - 0.875 * mag ; }
static void perturbAsmJointPrimitiveState_8_0 ( double mag , double * state )
{ state [ 20 ] = state [ 20 ] + mag ; } static void
perturbAsmJointPrimitiveState_8_0v ( double mag , double * state ) { state [
20 ] = state [ 20 ] + mag ; state [ 21 ] = state [ 21 ] - 0.875 * mag ; }
static void perturbAsmJointPrimitiveState_9_0 ( double mag , double * state )
{ state [ 22 ] = state [ 22 ] + mag ; } static void
perturbAsmJointPrimitiveState_9_0v ( double mag , double * state ) { state [
22 ] = state [ 22 ] + mag ; state [ 23 ] = state [ 23 ] - 0.875 * mag ; }
static void perturbAsmJointPrimitiveState_10_0 ( double mag , double * state
) { state [ 24 ] = state [ 24 ] + mag ; } static void
perturbAsmJointPrimitiveState_10_0v ( double mag , double * state ) { state [
24 ] = state [ 24 ] + mag ; state [ 25 ] = state [ 25 ] - 0.875 * mag ; }
static void perturbAsmJointPrimitiveState_11_0 ( double mag , double * state
) { state [ 26 ] = state [ 26 ] + mag ; } static void
perturbAsmJointPrimitiveState_11_0v ( double mag , double * state ) { state [
26 ] = state [ 26 ] + mag ; state [ 27 ] = state [ 27 ] - 0.875 * mag ; }
void wheel_leg_d9e82f16_1_perturbAsmJointPrimitiveState ( const void * mech ,
size_t stageIdx , size_t primIdx , double mag , boolean_T doPerturbVelocity ,
double * state ) { ( void ) mech ; ( void ) stageIdx ; ( void ) primIdx ; (
void ) mag ; ( void ) doPerturbVelocity ; ( void ) state ; switch ( (
stageIdx * 6 + primIdx ) * 2 + ( doPerturbVelocity ? 1 : 0 ) ) { case 0 :
perturbAsmJointPrimitiveState_0_0 ( mag , state ) ; break ; case 1 :
perturbAsmJointPrimitiveState_0_0v ( mag , state ) ; break ; case 2 :
perturbAsmJointPrimitiveState_0_1 ( mag , state ) ; break ; case 3 :
perturbAsmJointPrimitiveState_0_1v ( mag , state ) ; break ; case 4 :
perturbAsmJointPrimitiveState_0_2 ( mag , state ) ; break ; case 5 :
perturbAsmJointPrimitiveState_0_2v ( mag , state ) ; break ; case 12 :
perturbAsmJointPrimitiveState_1_0 ( mag , state ) ; break ; case 13 :
perturbAsmJointPrimitiveState_1_0v ( mag , state ) ; break ; case 24 :
perturbAsmJointPrimitiveState_2_0 ( mag , state ) ; break ; case 25 :
perturbAsmJointPrimitiveState_2_0v ( mag , state ) ; break ; case 36 :
perturbAsmJointPrimitiveState_3_0 ( mag , state ) ; break ; case 37 :
perturbAsmJointPrimitiveState_3_0v ( mag , state ) ; break ; case 48 :
perturbAsmJointPrimitiveState_4_0 ( mag , state ) ; break ; case 49 :
perturbAsmJointPrimitiveState_4_0v ( mag , state ) ; break ; case 60 :
perturbAsmJointPrimitiveState_5_0 ( mag , state ) ; break ; case 61 :
perturbAsmJointPrimitiveState_5_0v ( mag , state ) ; break ; case 72 :
perturbAsmJointPrimitiveState_6_0 ( mag , state ) ; break ; case 73 :
perturbAsmJointPrimitiveState_6_0v ( mag , state ) ; break ; case 84 :
perturbAsmJointPrimitiveState_7_0 ( mag , state ) ; break ; case 85 :
perturbAsmJointPrimitiveState_7_0v ( mag , state ) ; break ; case 96 :
perturbAsmJointPrimitiveState_8_0 ( mag , state ) ; break ; case 97 :
perturbAsmJointPrimitiveState_8_0v ( mag , state ) ; break ; case 108 :
perturbAsmJointPrimitiveState_9_0 ( mag , state ) ; break ; case 109 :
perturbAsmJointPrimitiveState_9_0v ( mag , state ) ; break ; case 120 :
perturbAsmJointPrimitiveState_10_0 ( mag , state ) ; break ; case 121 :
perturbAsmJointPrimitiveState_10_0v ( mag , state ) ; break ; case 132 :
perturbAsmJointPrimitiveState_11_0 ( mag , state ) ; break ; case 133 :
perturbAsmJointPrimitiveState_11_0v ( mag , state ) ; break ; } } void
wheel_leg_d9e82f16_1_computePosDofBlendMatrix ( const void * mech , size_t
stageIdx , size_t primIdx , const double * state , int partialType , double *
matrix ) { ( void ) mech ; ( void ) stageIdx ; ( void ) primIdx ; ( void )
state ; ( void ) partialType ; ( void ) matrix ; switch ( ( stageIdx * 6 +
primIdx ) ) { } } void wheel_leg_d9e82f16_1_computeVelDofBlendMatrix ( const
void * mech , size_t stageIdx , size_t primIdx , const double * state , int
partialType , double * matrix ) { ( void ) mech ; ( void ) stageIdx ; ( void
) primIdx ; ( void ) state ; ( void ) partialType ; ( void ) matrix ; switch
( ( stageIdx * 6 + primIdx ) ) { } } void
wheel_leg_d9e82f16_1_projectPartiallyTargetedPos ( const void * mech , size_t
stageIdx , size_t primIdx , const double * origState , int partialType ,
double * state ) { ( void ) mech ; ( void ) stageIdx ; ( void ) primIdx ; (
void ) origState ; ( void ) partialType ; ( void ) state ; switch ( (
stageIdx * 6 + primIdx ) ) { } } void wheel_leg_d9e82f16_1_propagateMotion (
const void * mech , const RuntimeDerivedValuesBundle * rtdv , const double *
state , double * motionData ) { const double * rtdvd = rtdv -> mDoubles .
mValues ; const int * rtdvi = rtdv -> mInts . mValues ; double xx [ 214 ] ; (
void ) mech ; ( void ) rtdvd ; ( void ) rtdvi ; xx [ 0 ] = 0.7071067811865476
; xx [ 1 ] = - xx [ 0 ] ; xx [ 2 ] = 0.0 ; xx [ 3 ] = 0.02 + state [ 1 ] ; xx
[ 4 ] = 0.4 - state [ 0 ] ; xx [ 5 ] = 0.5 ; xx [ 6 ] = xx [ 5 ] * state [ 6
] ; xx [ 7 ] = xx [ 0 ] * cos ( xx [ 6 ] ) ; xx [ 8 ] = - xx [ 7 ] ; xx [ 9 ]
= xx [ 0 ] * sin ( xx [ 6 ] ) ; xx [ 6 ] = - xx [ 9 ] ; xx [ 10 ] = 2.0 ; xx
[ 11 ] = 0.03 ; xx [ 12 ] = xx [ 11 ] * xx [ 9 ] ; xx [ 13 ] = xx [ 12 ] * xx
[ 9 ] ; xx [ 14 ] = xx [ 11 ] * xx [ 7 ] ; xx [ 15 ] = xx [ 14 ] * xx [ 7 ] ;
xx [ 16 ] = xx [ 10 ] * ( xx [ 13 ] - xx [ 15 ] ) ; xx [ 17 ] = ( xx [ 12 ] *
xx [ 7 ] + xx [ 14 ] * xx [ 9 ] ) * xx [ 10 ] ; xx [ 12 ] = xx [ 11 ] - ( xx
[ 13 ] + xx [ 15 ] ) * xx [ 10 ] ; xx [ 13 ] = xx [ 5 ] * state [ 8 ] ; xx [
14 ] = xx [ 0 ] * cos ( xx [ 13 ] ) ; xx [ 15 ] = - xx [ 14 ] ; xx [ 18 ] =
xx [ 0 ] * sin ( xx [ 13 ] ) ; xx [ 13 ] = - xx [ 18 ] ; xx [ 19 ] = 0.195 ;
xx [ 20 ] = 0.075 ; xx [ 21 ] = xx [ 20 ] * xx [ 14 ] ; xx [ 22 ] = xx [ 21 ]
* xx [ 14 ] ; xx [ 23 ] = xx [ 20 ] * xx [ 18 ] ; xx [ 24 ] = xx [ 23 ] * xx
[ 18 ] ; xx [ 25 ] = xx [ 19 ] + ( xx [ 22 ] + xx [ 24 ] ) * xx [ 10 ] - xx [
20 ] ; xx [ 26 ] = - xx [ 25 ] ; xx [ 27 ] = 0.05400000000000001 ; xx [ 28 ]
= xx [ 27 ] + ( xx [ 23 ] * xx [ 14 ] + xx [ 21 ] * xx [ 18 ] ) * xx [ 10 ] ;
xx [ 21 ] = xx [ 11 ] - xx [ 10 ] * ( xx [ 22 ] - xx [ 24 ] ) ; xx [ 22 ] =
xx [ 5 ] * state [ 10 ] ; xx [ 23 ] = cos ( xx [ 22 ] ) ; xx [ 24 ] = sin (
xx [ 22 ] ) ; xx [ 22 ] = 0.125 ; xx [ 29 ] = xx [ 22 ] * xx [ 24 ] ; xx [ 30
] = xx [ 20 ] - ( xx [ 10 ] * xx [ 29 ] * xx [ 24 ] - xx [ 22 ] ) ; xx [ 31 ]
= xx [ 10 ] * xx [ 23 ] * xx [ 29 ] ; xx [ 29 ] = xx [ 5 ] * state [ 12 ] ;
xx [ 32 ] = cos ( xx [ 29 ] ) ; xx [ 33 ] = sin ( xx [ 29 ] ) ; xx [ 29 ] =
xx [ 22 ] * xx [ 33 ] ; xx [ 34 ] = xx [ 22 ] - ( xx [ 22 ] - xx [ 10 ] * xx
[ 29 ] * xx [ 33 ] ) ; xx [ 35 ] = xx [ 10 ] * xx [ 32 ] * xx [ 29 ] ; xx [
29 ] = xx [ 5 ] * state [ 14 ] ; xx [ 36 ] = cos ( xx [ 29 ] ) ; xx [ 37 ] =
sin ( xx [ 29 ] ) ; xx [ 29 ] = 0.04 ; xx [ 38 ] = xx [ 5 ] * state [ 16 ] ;
xx [ 39 ] = xx [ 0 ] * cos ( xx [ 38 ] ) ; xx [ 40 ] = - xx [ 39 ] ; xx [ 41
] = xx [ 0 ] * sin ( xx [ 38 ] ) ; xx [ 38 ] = - xx [ 41 ] ; xx [ 42 ] = xx [
20 ] * xx [ 39 ] ; xx [ 43 ] = xx [ 42 ] * xx [ 39 ] ; xx [ 44 ] = xx [ 20 ]
* xx [ 41 ] ; xx [ 45 ] = xx [ 44 ] * xx [ 41 ] ; xx [ 46 ] = ( xx [ 44 ] *
xx [ 39 ] + xx [ 42 ] * xx [ 41 ] ) * xx [ 10 ] - xx [ 27 ] ; xx [ 42 ] = xx
[ 11 ] - xx [ 10 ] * ( xx [ 43 ] - xx [ 45 ] ) ; xx [ 44 ] = xx [ 5 ] * state
[ 18 ] ; xx [ 47 ] = xx [ 0 ] * cos ( xx [ 44 ] ) ; xx [ 48 ] = xx [ 0 ] *
sin ( xx [ 44 ] ) ; xx [ 44 ] = xx [ 20 ] * xx [ 47 ] ; xx [ 49 ] = xx [ 44 ]
* xx [ 47 ] ; xx [ 50 ] = xx [ 20 ] * xx [ 48 ] ; xx [ 51 ] = xx [ 50 ] * xx
[ 48 ] ; xx [ 52 ] = xx [ 19 ] - ( ( xx [ 49 ] + xx [ 51 ] ) * xx [ 10 ] - xx
[ 20 ] ) ; xx [ 19 ] = xx [ 27 ] + ( xx [ 50 ] * xx [ 47 ] + xx [ 44 ] * xx [
48 ] ) * xx [ 10 ] ; xx [ 44 ] = xx [ 11 ] - xx [ 10 ] * ( xx [ 49 ] - xx [
51 ] ) ; xx [ 49 ] = xx [ 5 ] * state [ 20 ] ; xx [ 50 ] = cos ( xx [ 49 ] )
; xx [ 51 ] = sin ( xx [ 49 ] ) ; xx [ 49 ] = xx [ 22 ] * xx [ 51 ] ; xx [ 53
] = xx [ 20 ] - ( xx [ 10 ] * xx [ 49 ] * xx [ 51 ] - xx [ 22 ] ) ; xx [ 54 ]
= xx [ 10 ] * xx [ 50 ] * xx [ 49 ] ; xx [ 49 ] = xx [ 5 ] * state [ 22 ] ;
xx [ 55 ] = cos ( xx [ 49 ] ) ; xx [ 56 ] = sin ( xx [ 49 ] ) ; xx [ 49 ] =
xx [ 22 ] * xx [ 56 ] ; xx [ 57 ] = xx [ 22 ] - ( xx [ 22 ] - xx [ 10 ] * xx
[ 49 ] * xx [ 56 ] ) ; xx [ 58 ] = xx [ 10 ] * xx [ 55 ] * xx [ 49 ] ; xx [
49 ] = xx [ 5 ] * state [ 24 ] ; xx [ 59 ] = cos ( xx [ 49 ] ) ; xx [ 60 ] =
sin ( xx [ 49 ] ) ; xx [ 49 ] = 0.035 ; xx [ 61 ] = xx [ 5 ] * state [ 26 ] ;
xx [ 5 ] = xx [ 0 ] * cos ( xx [ 61 ] ) ; xx [ 62 ] = xx [ 0 ] * sin ( xx [
61 ] ) ; xx [ 61 ] = xx [ 20 ] * xx [ 5 ] ; xx [ 63 ] = xx [ 61 ] * xx [ 5 ]
; xx [ 64 ] = xx [ 20 ] * xx [ 62 ] ; xx [ 65 ] = xx [ 64 ] * xx [ 62 ] ; xx
[ 66 ] = ( xx [ 64 ] * xx [ 5 ] + xx [ 61 ] * xx [ 62 ] ) * xx [ 10 ] - xx [
27 ] ; xx [ 27 ] = xx [ 11 ] - xx [ 10 ] * ( xx [ 63 ] - xx [ 65 ] ) ; xx [
11 ] = xx [ 23 ] * xx [ 14 ] - xx [ 18 ] * xx [ 24 ] ; xx [ 61 ] = xx [ 23 ]
* xx [ 18 ] ; xx [ 64 ] = xx [ 14 ] * xx [ 24 ] ; xx [ 67 ] = xx [ 61 ] + xx
[ 64 ] ; xx [ 68 ] = xx [ 64 ] + xx [ 61 ] ; xx [ 69 ] = xx [ 13 ] ; xx [ 70
] = xx [ 15 ] ; xx [ 71 ] = xx [ 13 ] ; xx [ 61 ] = xx [ 18 ] * xx [ 31 ] ;
xx [ 64 ] = xx [ 18 ] * xx [ 30 ] ; xx [ 72 ] = xx [ 61 ] - xx [ 14 ] * xx [
30 ] ; xx [ 73 ] = xx [ 61 ] ; xx [ 74 ] = - xx [ 64 ] ; xx [ 75 ] = - xx [
72 ] ; pm_math_Vector3_cross_ra ( xx + 69 , xx + 73 , xx + 76 ) ; xx [ 69 ] =
xx [ 30 ] + ( xx [ 76 ] - xx [ 14 ] * xx [ 61 ] ) * xx [ 10 ] - xx [ 25 ] ;
xx [ 25 ] = ( xx [ 14 ] * xx [ 64 ] + xx [ 77 ] ) * xx [ 10 ] + xx [ 31 ] +
xx [ 28 ] ; xx [ 61 ] = xx [ 10 ] * ( xx [ 78 ] + xx [ 72 ] * xx [ 14 ] ) +
xx [ 21 ] ; xx [ 64 ] = xx [ 32 ] * xx [ 11 ] ; xx [ 70 ] = xx [ 33 ] * xx [
11 ] ; xx [ 71 ] = xx [ 67 ] ; xx [ 72 ] = xx [ 11 ] ; xx [ 73 ] = xx [ 68 ]
; xx [ 74 ] = xx [ 68 ] * xx [ 35 ] ; xx [ 75 ] = xx [ 68 ] * xx [ 34 ] ; xx
[ 76 ] = xx [ 34 ] * xx [ 11 ] - xx [ 67 ] * xx [ 35 ] ; xx [ 77 ] = - xx [
74 ] ; xx [ 78 ] = xx [ 75 ] ; xx [ 79 ] = - xx [ 76 ] ;
pm_math_Vector3_cross_ra ( xx + 71 , xx + 77 , xx + 80 ) ; xx [ 71 ] = xx [
35 ] * xx [ 24 ] ; xx [ 72 ] = xx [ 24 ] * xx [ 34 ] ; xx [ 73 ] = xx [ 48 ]
* xx [ 51 ] - xx [ 50 ] * xx [ 47 ] ; xx [ 77 ] = xx [ 50 ] * xx [ 48 ] ; xx
[ 78 ] = xx [ 47 ] * xx [ 51 ] ; xx [ 79 ] = xx [ 77 ] + xx [ 78 ] ; xx [ 83
] = - xx [ 79 ] ; xx [ 84 ] = xx [ 78 ] + xx [ 77 ] ; xx [ 77 ] = - xx [ 84 ]
; xx [ 85 ] = xx [ 48 ] ; xx [ 86 ] = xx [ 47 ] ; xx [ 87 ] = xx [ 48 ] ; xx
[ 78 ] = xx [ 48 ] * xx [ 54 ] ; xx [ 88 ] = xx [ 48 ] * xx [ 53 ] ; xx [ 89
] = xx [ 47 ] * xx [ 53 ] - xx [ 78 ] ; xx [ 90 ] = - xx [ 78 ] ; xx [ 91 ] =
xx [ 88 ] ; xx [ 92 ] = - xx [ 89 ] ; pm_math_Vector3_cross_ra ( xx + 85 , xx
+ 90 , xx + 93 ) ; xx [ 85 ] = xx [ 53 ] + ( xx [ 93 ] - xx [ 47 ] * xx [ 78
] ) * xx [ 10 ] + xx [ 52 ] ; xx [ 78 ] = ( xx [ 47 ] * xx [ 88 ] + xx [ 94 ]
) * xx [ 10 ] + xx [ 54 ] + xx [ 19 ] ; xx [ 86 ] = xx [ 10 ] * ( xx [ 95 ] -
xx [ 89 ] * xx [ 47 ] ) + xx [ 44 ] ; xx [ 87 ] = xx [ 55 ] * xx [ 73 ] ; xx
[ 88 ] = xx [ 56 ] * xx [ 73 ] ; xx [ 89 ] = xx [ 84 ] * xx [ 58 ] ; xx [ 90
] = xx [ 83 ] ; xx [ 91 ] = xx [ 73 ] ; xx [ 92 ] = xx [ 77 ] ; xx [ 93 ] =
xx [ 84 ] * xx [ 57 ] ; xx [ 94 ] = xx [ 79 ] * xx [ 58 ] + xx [ 57 ] * xx [
73 ] ; xx [ 95 ] = xx [ 89 ] ; xx [ 96 ] = - xx [ 93 ] ; xx [ 97 ] = - xx [
94 ] ; pm_math_Vector3_cross_ra ( xx + 90 , xx + 95 , xx + 98 ) ; xx [ 90 ] =
xx [ 58 ] * xx [ 51 ] ; xx [ 91 ] = xx [ 51 ] * xx [ 57 ] ; xx [ 92 ] = xx [
0 ] * xx [ 7 ] ; xx [ 95 ] = xx [ 92 ] + xx [ 92 ] ; xx [ 96 ] = xx [ 0 ] *
xx [ 9 ] ; xx [ 9 ] = xx [ 96 ] + xx [ 96 ] ; xx [ 97 ] = xx [ 92 ] - xx [ 92
] ; xx [ 92 ] = - ( xx [ 96 ] - xx [ 96 ] ) ; xx [ 96 ] = xx [ 0 ] * xx [ 0 ]
* xx [ 16 ] ; xx [ 101 ] = xx [ 0 ] * xx [ 0 ] * xx [ 12 ] ; xx [ 0 ] = xx [
10 ] * ( xx [ 96 ] - xx [ 101 ] ) - xx [ 16 ] + state [ 2 ] ; xx [ 102 ] = xx
[ 17 ] + xx [ 3 ] ; xx [ 103 ] = ( xx [ 96 ] + xx [ 101 ] ) * xx [ 10 ] - xx
[ 12 ] + xx [ 4 ] ; xx [ 104 ] = xx [ 95 ] ; xx [ 105 ] = xx [ 9 ] ; xx [ 106
] = xx [ 97 ] ; xx [ 107 ] = xx [ 92 ] ; xx [ 108 ] = xx [ 15 ] ; xx [ 109 ]
= xx [ 13 ] ; xx [ 110 ] = xx [ 15 ] ; xx [ 111 ] = xx [ 13 ] ;
pm_math_Quaternion_compose_ra ( xx + 104 , xx + 108 , xx + 112 ) ; xx [ 116 ]
= xx [ 26 ] ; xx [ 117 ] = xx [ 28 ] ; xx [ 118 ] = xx [ 21 ] ;
pm_math_Quaternion_xform_ra ( xx + 104 , xx + 116 , xx + 119 ) ; xx [ 96 ] =
xx [ 119 ] + xx [ 0 ] ; xx [ 101 ] = xx [ 120 ] + xx [ 102 ] ; xx [ 116 ] =
xx [ 121 ] + xx [ 103 ] ; xx [ 117 ] = xx [ 115 ] * xx [ 24 ] - xx [ 23 ] *
xx [ 112 ] ; xx [ 118 ] = xx [ 23 ] * xx [ 113 ] + xx [ 114 ] * xx [ 24 ] ;
xx [ 119 ] = - xx [ 118 ] ; xx [ 120 ] = xx [ 113 ] * xx [ 24 ] - xx [ 23 ] *
xx [ 114 ] ; xx [ 121 ] = xx [ 112 ] * xx [ 24 ] + xx [ 23 ] * xx [ 115 ] ;
xx [ 122 ] = - xx [ 121 ] ; xx [ 123 ] = xx [ 115 ] * xx [ 31 ] ; xx [ 124 ]
= xx [ 115 ] * xx [ 30 ] ; xx [ 125 ] = xx [ 114 ] * xx [ 30 ] - xx [ 113 ] *
xx [ 31 ] ; xx [ 126 ] = - xx [ 123 ] ; xx [ 127 ] = xx [ 124 ] ; xx [ 128 ]
= - xx [ 125 ] ; pm_math_Vector3_cross_ra ( xx + 113 , xx + 126 , xx + 129 )
; xx [ 126 ] = xx [ 30 ] + ( xx [ 129 ] - xx [ 112 ] * xx [ 123 ] ) * xx [ 10
] ; xx [ 123 ] = ( xx [ 112 ] * xx [ 124 ] + xx [ 130 ] ) * xx [ 10 ] + xx [
31 ] + xx [ 101 ] ; xx [ 124 ] = xx [ 10 ] * ( xx [ 131 ] - xx [ 125 ] * xx [
112 ] ) ; xx [ 127 ] = xx [ 119 ] ; xx [ 128 ] = xx [ 120 ] ; xx [ 129 ] = xx
[ 122 ] ; xx [ 125 ] = xx [ 29 ] * xx [ 120 ] ; xx [ 130 ] = xx [ 118 ] * xx
[ 29 ] + xx [ 121 ] * xx [ 22 ] ; xx [ 131 ] = xx [ 22 ] * xx [ 120 ] ; xx [
132 ] = - xx [ 125 ] ; xx [ 133 ] = - xx [ 130 ] ; xx [ 134 ] = - xx [ 131 ]
; pm_math_Vector3_cross_ra ( xx + 127 , xx + 132 , xx + 135 ) ; xx [ 138 ] =
xx [ 47 ] ; xx [ 139 ] = xx [ 48 ] ; xx [ 140 ] = xx [ 47 ] ; xx [ 141 ] = xx
[ 48 ] ; pm_math_Quaternion_compose_ra ( xx + 104 , xx + 138 , xx + 142 ) ;
xx [ 127 ] = xx [ 52 ] ; xx [ 128 ] = xx [ 19 ] ; xx [ 129 ] = xx [ 44 ] ;
pm_math_Quaternion_xform_ra ( xx + 104 , xx + 127 , xx + 132 ) ; xx [ 104 ] =
xx [ 132 ] + xx [ 0 ] ; xx [ 105 ] = xx [ 133 ] + xx [ 102 ] ; xx [ 106 ] =
xx [ 134 ] + xx [ 103 ] ; xx [ 107 ] = xx [ 145 ] * xx [ 51 ] - xx [ 50 ] *
xx [ 142 ] ; xx [ 127 ] = xx [ 50 ] * xx [ 143 ] + xx [ 144 ] * xx [ 51 ] ;
xx [ 128 ] = - xx [ 127 ] ; xx [ 129 ] = xx [ 143 ] * xx [ 51 ] - xx [ 50 ] *
xx [ 144 ] ; xx [ 132 ] = xx [ 142 ] * xx [ 51 ] + xx [ 50 ] * xx [ 145 ] ;
xx [ 133 ] = - xx [ 132 ] ; xx [ 134 ] = xx [ 145 ] * xx [ 54 ] ; xx [ 146 ]
= xx [ 145 ] * xx [ 53 ] ; xx [ 147 ] = xx [ 144 ] * xx [ 53 ] - xx [ 143 ] *
xx [ 54 ] ; xx [ 148 ] = - xx [ 134 ] ; xx [ 149 ] = xx [ 146 ] ; xx [ 150 ]
= - xx [ 147 ] ; pm_math_Vector3_cross_ra ( xx + 143 , xx + 148 , xx + 151 )
; xx [ 148 ] = xx [ 53 ] + ( xx [ 151 ] - xx [ 142 ] * xx [ 134 ] ) * xx [ 10
] ; xx [ 134 ] = ( xx [ 142 ] * xx [ 146 ] + xx [ 152 ] ) * xx [ 10 ] + xx [
54 ] + xx [ 105 ] ; xx [ 146 ] = xx [ 10 ] * ( xx [ 153 ] - xx [ 147 ] * xx [
142 ] ) ; xx [ 147 ] = xx [ 49 ] * xx [ 129 ] ; xx [ 149 ] = xx [ 128 ] ; xx
[ 150 ] = xx [ 129 ] ; xx [ 151 ] = xx [ 133 ] ; xx [ 152 ] = xx [ 127 ] * xx
[ 49 ] - xx [ 132 ] * xx [ 22 ] ; xx [ 153 ] = xx [ 22 ] * xx [ 129 ] ; xx [
154 ] = xx [ 147 ] ; xx [ 155 ] = xx [ 152 ] ; xx [ 156 ] = - xx [ 153 ] ;
pm_math_Vector3_cross_ra ( xx + 149 , xx + 154 , xx + 157 ) ; xx [ 160 ] = xx
[ 8 ] ; xx [ 161 ] = xx [ 6 ] ; xx [ 162 ] = xx [ 7 ] ; xx [ 163 ] = xx [ 6 ]
; xx [ 149 ] = state [ 3 ] ; xx [ 150 ] = state [ 4 ] ; xx [ 151 ] = state [
5 ] ; pm_math_Quaternion_inverseXform_ra ( xx + 160 , xx + 149 , xx + 154 ) ;
xx [ 149 ] = xx [ 155 ] + 0.03000000000000001 * state [ 7 ] ; xx [ 150 ] = xx
[ 14 ] * state [ 7 ] ; xx [ 151 ] = xx [ 18 ] * state [ 7 ] ; xx [ 155 ] = (
xx [ 14 ] * xx [ 150 ] + xx [ 18 ] * xx [ 151 ] ) * xx [ 10 ] ; xx [ 160 ] =
state [ 7 ] - xx [ 155 ] ; xx [ 161 ] = xx [ 10 ] * ( xx [ 18 ] * xx [ 150 ]
- xx [ 14 ] * xx [ 151 ] ) ; xx [ 14 ] = xx [ 155 ] + state [ 9 ] ; xx [ 162
] = xx [ 154 ] ; xx [ 163 ] = xx [ 149 ] - xx [ 21 ] * state [ 7 ] ; xx [ 164
] = xx [ 28 ] * state [ 7 ] + xx [ 156 ] ; pm_math_Quaternion_inverseXform_ra
( xx + 108 , xx + 162 , xx + 165 ) ; xx [ 18 ] = xx [ 166 ] + xx [ 20 ] *
state [ 9 ] ; xx [ 108 ] = xx [ 161 ] * xx [ 24 ] ; xx [ 109 ] = xx [ 24 ] *
xx [ 160 ] ; xx [ 110 ] = xx [ 160 ] + xx [ 10 ] * ( xx [ 23 ] * xx [ 108 ] -
xx [ 109 ] * xx [ 24 ] ) ; xx [ 111 ] = xx [ 161 ] - ( xx [ 23 ] * xx [ 109 ]
+ xx [ 108 ] * xx [ 24 ] ) * xx [ 10 ] ; xx [ 108 ] = xx [ 14 ] + state [ 11
] ; xx [ 109 ] = xx [ 165 ] - xx [ 14 ] * xx [ 31 ] ; xx [ 150 ] = xx [ 14 ]
* xx [ 30 ] + xx [ 18 ] ; xx [ 151 ] = xx [ 150 ] * xx [ 24 ] ; xx [ 155 ] =
xx [ 109 ] * xx [ 24 ] ; xx [ 162 ] = xx [ 109 ] + xx [ 10 ] * ( xx [ 23 ] *
xx [ 151 ] - xx [ 155 ] * xx [ 24 ] ) ; xx [ 109 ] = xx [ 150 ] - ( xx [ 23 ]
* xx [ 155 ] + xx [ 151 ] * xx [ 24 ] ) * xx [ 10 ] + xx [ 22 ] * state [ 11
] ; xx [ 150 ] = xx [ 167 ] - ( xx [ 161 ] * xx [ 30 ] - xx [ 31 ] * xx [ 160
] ) ; xx [ 151 ] = xx [ 33 ] * xx [ 111 ] ; xx [ 155 ] = xx [ 110 ] * xx [ 33
] ; xx [ 163 ] = xx [ 162 ] - xx [ 108 ] * xx [ 35 ] ; xx [ 164 ] = xx [ 108
] * xx [ 34 ] + xx [ 109 ] ; xx [ 166 ] = xx [ 164 ] * xx [ 33 ] ; xx [ 168 ]
= xx [ 33 ] * xx [ 163 ] ; xx [ 169 ] = xx [ 37 ] * xx [ 111 ] ; xx [ 170 ] =
xx [ 110 ] * xx [ 37 ] ; xx [ 171 ] = xx [ 162 ] - xx [ 29 ] * xx [ 111 ] ;
xx [ 172 ] = xx [ 108 ] * xx [ 22 ] + xx [ 110 ] * xx [ 29 ] + xx [ 109 ] ;
xx [ 173 ] = xx [ 172 ] * xx [ 37 ] ; xx [ 174 ] = xx [ 171 ] * xx [ 37 ] ;
xx [ 175 ] = xx [ 39 ] * state [ 7 ] ; xx [ 176 ] = xx [ 41 ] * state [ 7 ] ;
xx [ 177 ] = ( xx [ 39 ] * xx [ 175 ] + xx [ 41 ] * xx [ 176 ] ) * xx [ 10 ]
; xx [ 178 ] = xx [ 40 ] ; xx [ 179 ] = xx [ 38 ] ; xx [ 180 ] = xx [ 40 ] ;
xx [ 181 ] = xx [ 38 ] ; xx [ 182 ] = xx [ 154 ] ; xx [ 183 ] = xx [ 149 ] -
xx [ 42 ] * state [ 7 ] ; xx [ 184 ] = xx [ 46 ] * state [ 7 ] + xx [ 156 ] ;
pm_math_Quaternion_inverseXform_ra ( xx + 178 , xx + 182 , xx + 185 ) ; xx [
178 ] = xx [ 47 ] * state [ 7 ] ; xx [ 179 ] = xx [ 48 ] * state [ 7 ] ; xx [
180 ] = ( xx [ 47 ] * xx [ 178 ] + xx [ 48 ] * xx [ 179 ] ) * xx [ 10 ] ; xx
[ 181 ] = state [ 7 ] - xx [ 180 ] ; xx [ 182 ] = xx [ 10 ] * ( xx [ 48 ] *
xx [ 178 ] - xx [ 47 ] * xx [ 179 ] ) ; xx [ 178 ] = xx [ 180 ] + state [ 19
] ; xx [ 188 ] = xx [ 154 ] ; xx [ 189 ] = xx [ 149 ] - xx [ 44 ] * state [ 7
] ; xx [ 190 ] = xx [ 19 ] * state [ 7 ] + xx [ 156 ] ;
pm_math_Quaternion_inverseXform_ra ( xx + 138 , xx + 188 , xx + 191 ) ; xx [
138 ] = xx [ 192 ] + xx [ 20 ] * state [ 19 ] ; xx [ 139 ] = xx [ 182 ] * xx
[ 51 ] ; xx [ 140 ] = xx [ 51 ] * xx [ 181 ] ; xx [ 141 ] = xx [ 181 ] + xx [
10 ] * ( xx [ 50 ] * xx [ 139 ] - xx [ 140 ] * xx [ 51 ] ) ; xx [ 179 ] = xx
[ 182 ] - ( xx [ 50 ] * xx [ 140 ] + xx [ 139 ] * xx [ 51 ] ) * xx [ 10 ] ;
xx [ 139 ] = xx [ 178 ] + state [ 21 ] ; xx [ 140 ] = xx [ 191 ] - xx [ 178 ]
* xx [ 54 ] ; xx [ 180 ] = xx [ 178 ] * xx [ 53 ] + xx [ 138 ] ; xx [ 183 ] =
xx [ 180 ] * xx [ 51 ] ; xx [ 184 ] = xx [ 140 ] * xx [ 51 ] ; xx [ 188 ] =
xx [ 140 ] + xx [ 10 ] * ( xx [ 50 ] * xx [ 183 ] - xx [ 184 ] * xx [ 51 ] )
; xx [ 140 ] = xx [ 180 ] - ( xx [ 50 ] * xx [ 184 ] + xx [ 183 ] * xx [ 51 ]
) * xx [ 10 ] + xx [ 22 ] * state [ 21 ] ; xx [ 180 ] = xx [ 193 ] - ( xx [
182 ] * xx [ 53 ] - xx [ 54 ] * xx [ 181 ] ) ; xx [ 183 ] = xx [ 56 ] * xx [
179 ] ; xx [ 184 ] = xx [ 141 ] * xx [ 56 ] ; xx [ 189 ] = xx [ 188 ] - xx [
139 ] * xx [ 58 ] ; xx [ 190 ] = xx [ 139 ] * xx [ 57 ] + xx [ 140 ] ; xx [
192 ] = xx [ 190 ] * xx [ 56 ] ; xx [ 194 ] = xx [ 56 ] * xx [ 189 ] ; xx [
195 ] = xx [ 60 ] * xx [ 179 ] ; xx [ 196 ] = xx [ 141 ] * xx [ 60 ] ; xx [
197 ] = xx [ 49 ] * xx [ 179 ] + xx [ 188 ] ; xx [ 198 ] = xx [ 139 ] * xx [
22 ] - xx [ 141 ] * xx [ 49 ] + xx [ 140 ] ; xx [ 199 ] = xx [ 198 ] * xx [
60 ] ; xx [ 200 ] = xx [ 197 ] * xx [ 60 ] ; xx [ 201 ] = xx [ 5 ] * state [
7 ] ; xx [ 202 ] = xx [ 62 ] * state [ 7 ] ; xx [ 203 ] = ( xx [ 5 ] * xx [
201 ] + xx [ 62 ] * xx [ 202 ] ) * xx [ 10 ] ; xx [ 204 ] = xx [ 5 ] ; xx [
205 ] = xx [ 62 ] ; xx [ 206 ] = xx [ 5 ] ; xx [ 207 ] = xx [ 62 ] ; xx [ 208
] = xx [ 154 ] ; xx [ 209 ] = xx [ 149 ] - xx [ 27 ] * state [ 7 ] ; xx [ 210
] = xx [ 66 ] * state [ 7 ] + xx [ 156 ] ; pm_math_Quaternion_inverseXform_ra
( xx + 204 , xx + 208 , xx + 211 ) ; motionData [ 0 ] = xx [ 1 ] ; motionData
[ 1 ] = xx [ 2 ] ; motionData [ 2 ] = xx [ 1 ] ; motionData [ 3 ] = xx [ 2 ]
; motionData [ 4 ] = state [ 2 ] ; motionData [ 5 ] = xx [ 3 ] ; motionData [
6 ] = xx [ 4 ] ; motionData [ 7 ] = xx [ 8 ] ; motionData [ 8 ] = xx [ 6 ] ;
motionData [ 9 ] = xx [ 7 ] ; motionData [ 10 ] = xx [ 6 ] ; motionData [ 11
] = - xx [ 16 ] ; motionData [ 12 ] = xx [ 17 ] ; motionData [ 13 ] = - xx [
12 ] ; motionData [ 14 ] = xx [ 15 ] ; motionData [ 15 ] = xx [ 13 ] ;
motionData [ 16 ] = xx [ 15 ] ; motionData [ 17 ] = xx [ 13 ] ; motionData [
18 ] = xx [ 26 ] ; motionData [ 19 ] = xx [ 28 ] ; motionData [ 20 ] = xx [
21 ] ; motionData [ 21 ] = - xx [ 23 ] ; motionData [ 22 ] = xx [ 2 ] ;
motionData [ 23 ] = xx [ 2 ] ; motionData [ 24 ] = - xx [ 24 ] ; motionData [
25 ] = xx [ 30 ] ; motionData [ 26 ] = xx [ 31 ] ; motionData [ 27 ] = xx [ 2
] ; motionData [ 28 ] = - xx [ 32 ] ; motionData [ 29 ] = xx [ 2 ] ;
motionData [ 30 ] = xx [ 2 ] ; motionData [ 31 ] = xx [ 33 ] ; motionData [
32 ] = xx [ 34 ] ; motionData [ 33 ] = xx [ 35 ] ; motionData [ 34 ] = xx [ 2
] ; motionData [ 35 ] = - xx [ 36 ] ; motionData [ 36 ] = xx [ 2 ] ;
motionData [ 37 ] = xx [ 2 ] ; motionData [ 38 ] = - xx [ 37 ] ; motionData [
39 ] = xx [ 22 ] ; motionData [ 40 ] = xx [ 2 ] ; motionData [ 41 ] = - xx [
29 ] ; motionData [ 42 ] = xx [ 40 ] ; motionData [ 43 ] = xx [ 38 ] ;
motionData [ 44 ] = xx [ 40 ] ; motionData [ 45 ] = xx [ 38 ] ; motionData [
46 ] = - ( ( xx [ 43 ] + xx [ 45 ] ) * xx [ 10 ] + 0.12 ) ; motionData [ 47 ]
= xx [ 46 ] ; motionData [ 48 ] = xx [ 42 ] ; motionData [ 49 ] = xx [ 47 ] ;
motionData [ 50 ] = xx [ 48 ] ; motionData [ 51 ] = xx [ 47 ] ; motionData [
52 ] = xx [ 48 ] ; motionData [ 53 ] = xx [ 52 ] ; motionData [ 54 ] = xx [
19 ] ; motionData [ 55 ] = xx [ 44 ] ; motionData [ 56 ] = - xx [ 50 ] ;
motionData [ 57 ] = xx [ 2 ] ; motionData [ 58 ] = xx [ 2 ] ; motionData [ 59
] = - xx [ 51 ] ; motionData [ 60 ] = xx [ 53 ] ; motionData [ 61 ] = xx [ 54
] ; motionData [ 62 ] = xx [ 2 ] ; motionData [ 63 ] = - xx [ 55 ] ;
motionData [ 64 ] = xx [ 2 ] ; motionData [ 65 ] = xx [ 2 ] ; motionData [ 66
] = xx [ 56 ] ; motionData [ 67 ] = xx [ 57 ] ; motionData [ 68 ] = xx [ 58 ]
; motionData [ 69 ] = xx [ 2 ] ; motionData [ 70 ] = - xx [ 59 ] ; motionData
[ 71 ] = xx [ 2 ] ; motionData [ 72 ] = xx [ 2 ] ; motionData [ 73 ] = - xx [
60 ] ; motionData [ 74 ] = xx [ 22 ] ; motionData [ 75 ] = xx [ 2 ] ;
motionData [ 76 ] = xx [ 49 ] ; motionData [ 77 ] = xx [ 5 ] ; motionData [
78 ] = xx [ 62 ] ; motionData [ 79 ] = xx [ 5 ] ; motionData [ 80 ] = xx [ 62
] ; motionData [ 81 ] = 0.27 - ( xx [ 63 ] + xx [ 65 ] ) * xx [ 10 ] ;
motionData [ 82 ] = xx [ 66 ] ; motionData [ 83 ] = xx [ 27 ] ; motionData [
84 ] = xx [ 11 ] ; motionData [ 85 ] = xx [ 67 ] ; motionData [ 86 ] = xx [
11 ] ; motionData [ 87 ] = xx [ 68 ] ; motionData [ 88 ] = xx [ 69 ] ;
motionData [ 89 ] = xx [ 25 ] ; motionData [ 90 ] = xx [ 61 ] ; motionData [
91 ] = - ( xx [ 68 ] * xx [ 33 ] + xx [ 64 ] ) ; motionData [ 92 ] = xx [ 70
] - xx [ 67 ] * xx [ 32 ] ; motionData [ 93 ] = - ( xx [ 67 ] * xx [ 33 ] +
xx [ 64 ] ) ; motionData [ 94 ] = xx [ 70 ] - xx [ 68 ] * xx [ 32 ] ;
motionData [ 95 ] = xx [ 34 ] + ( xx [ 80 ] - xx [ 74 ] * xx [ 11 ] ) * xx [
10 ] + xx [ 69 ] ; motionData [ 96 ] = xx [ 35 ] + xx [ 10 ] * ( xx [ 81 ] +
xx [ 75 ] * xx [ 11 ] ) + xx [ 25 ] ; motionData [ 97 ] = xx [ 10 ] * ( xx [
82 ] - xx [ 76 ] * xx [ 11 ] ) + xx [ 61 ] ; motionData [ 98 ] = xx [ 23 ] *
xx [ 32 ] + xx [ 24 ] * xx [ 33 ] ; motionData [ 99 ] = xx [ 2 ] ; motionData
[ 100 ] = xx [ 2 ] ; motionData [ 101 ] = xx [ 32 ] * xx [ 24 ] - xx [ 23 ] *
xx [ 33 ] ; motionData [ 102 ] = xx [ 34 ] - ( xx [ 23 ] * xx [ 71 ] + xx [
72 ] * xx [ 24 ] ) * xx [ 10 ] + xx [ 30 ] ; motionData [ 103 ] = xx [ 35 ] +
xx [ 10 ] * ( xx [ 23 ] * xx [ 72 ] - xx [ 71 ] * xx [ 24 ] ) + xx [ 31 ] ;
motionData [ 104 ] = xx [ 2 ] ; motionData [ 105 ] = xx [ 73 ] ; motionData [
106 ] = xx [ 83 ] ; motionData [ 107 ] = xx [ 73 ] ; motionData [ 108 ] = xx
[ 77 ] ; motionData [ 109 ] = xx [ 85 ] ; motionData [ 110 ] = xx [ 78 ] ;
motionData [ 111 ] = xx [ 86 ] ; motionData [ 112 ] = xx [ 84 ] * xx [ 56 ] -
xx [ 87 ] ; motionData [ 113 ] = xx [ 79 ] * xx [ 55 ] + xx [ 88 ] ;
motionData [ 114 ] = xx [ 79 ] * xx [ 56 ] - xx [ 87 ] ; motionData [ 115 ] =
xx [ 88 ] + xx [ 84 ] * xx [ 55 ] ; motionData [ 116 ] = xx [ 57 ] + ( xx [
89 ] * xx [ 73 ] + xx [ 98 ] ) * xx [ 10 ] + xx [ 85 ] ; motionData [ 117 ] =
xx [ 58 ] + xx [ 10 ] * ( xx [ 99 ] - xx [ 93 ] * xx [ 73 ] ) + xx [ 78 ] ;
motionData [ 118 ] = xx [ 10 ] * ( xx [ 100 ] - xx [ 94 ] * xx [ 73 ] ) + xx
[ 86 ] ; motionData [ 119 ] = xx [ 50 ] * xx [ 55 ] + xx [ 51 ] * xx [ 56 ] ;
motionData [ 120 ] = xx [ 2 ] ; motionData [ 121 ] = xx [ 2 ] ; motionData [
122 ] = xx [ 55 ] * xx [ 51 ] - xx [ 50 ] * xx [ 56 ] ; motionData [ 123 ] =
xx [ 57 ] - ( xx [ 50 ] * xx [ 90 ] + xx [ 91 ] * xx [ 51 ] ) * xx [ 10 ] +
xx [ 53 ] ; motionData [ 124 ] = xx [ 58 ] + xx [ 10 ] * ( xx [ 50 ] * xx [
91 ] - xx [ 90 ] * xx [ 51 ] ) + xx [ 54 ] ; motionData [ 125 ] = xx [ 2 ] ;
motionData [ 126 ] = xx [ 95 ] ; motionData [ 127 ] = xx [ 9 ] ; motionData [
128 ] = xx [ 97 ] ; motionData [ 129 ] = xx [ 92 ] ; motionData [ 130 ] = xx
[ 0 ] ; motionData [ 131 ] = xx [ 102 ] ; motionData [ 132 ] = xx [ 103 ] ;
motionData [ 133 ] = xx [ 112 ] ; motionData [ 134 ] = xx [ 113 ] ;
motionData [ 135 ] = xx [ 114 ] ; motionData [ 136 ] = xx [ 115 ] ;
motionData [ 137 ] = xx [ 96 ] ; motionData [ 138 ] = xx [ 101 ] ; motionData
[ 139 ] = xx [ 116 ] ; motionData [ 140 ] = xx [ 117 ] ; motionData [ 141 ] =
xx [ 119 ] ; motionData [ 142 ] = xx [ 120 ] ; motionData [ 143 ] = xx [ 122
] ; motionData [ 144 ] = xx [ 126 ] + xx [ 96 ] ; motionData [ 145 ] = xx [
123 ] ; motionData [ 146 ] = xx [ 124 ] + xx [ 116 ] ; motionData [ 147 ] = -
( xx [ 36 ] * xx [ 117 ] + xx [ 121 ] * xx [ 37 ] ) ; motionData [ 148 ] = xx
[ 118 ] * xx [ 36 ] - xx [ 37 ] * xx [ 120 ] ; motionData [ 149 ] = - ( xx [
36 ] * xx [ 120 ] + xx [ 118 ] * xx [ 37 ] ) ; motionData [ 150 ] = xx [ 121
] * xx [ 36 ] - xx [ 37 ] * xx [ 117 ] ; motionData [ 151 ] = ( xx [ 135 ] -
xx [ 125 ] * xx [ 117 ] ) * xx [ 10 ] + xx [ 126 ] + xx [ 96 ] + xx [ 22 ] ;
motionData [ 152 ] = ( xx [ 136 ] - xx [ 117 ] * xx [ 130 ] ) * xx [ 10 ] +
xx [ 123 ] ; motionData [ 153 ] = xx [ 10 ] * ( xx [ 137 ] - xx [ 131 ] * xx
[ 117 ] ) + xx [ 124 ] + xx [ 116 ] - xx [ 29 ] ; motionData [ 154 ] = xx [
142 ] ; motionData [ 155 ] = xx [ 143 ] ; motionData [ 156 ] = xx [ 144 ] ;
motionData [ 157 ] = xx [ 145 ] ; motionData [ 158 ] = xx [ 104 ] ;
motionData [ 159 ] = xx [ 105 ] ; motionData [ 160 ] = xx [ 106 ] ;
motionData [ 161 ] = xx [ 107 ] ; motionData [ 162 ] = xx [ 128 ] ;
motionData [ 163 ] = xx [ 129 ] ; motionData [ 164 ] = xx [ 133 ] ;
motionData [ 165 ] = xx [ 148 ] + xx [ 104 ] ; motionData [ 166 ] = xx [ 134
] ; motionData [ 167 ] = xx [ 146 ] + xx [ 106 ] ; motionData [ 168 ] = - (
xx [ 59 ] * xx [ 107 ] + xx [ 132 ] * xx [ 60 ] ) ; motionData [ 169 ] = xx [
127 ] * xx [ 59 ] - xx [ 60 ] * xx [ 129 ] ; motionData [ 170 ] = - ( xx [ 59
] * xx [ 129 ] + xx [ 127 ] * xx [ 60 ] ) ; motionData [ 171 ] = xx [ 132 ] *
xx [ 59 ] - xx [ 60 ] * xx [ 107 ] ; motionData [ 172 ] = ( xx [ 147 ] * xx [
107 ] + xx [ 157 ] ) * xx [ 10 ] + xx [ 148 ] + xx [ 104 ] + xx [ 22 ] ;
motionData [ 173 ] = ( xx [ 152 ] * xx [ 107 ] + xx [ 158 ] ) * xx [ 10 ] +
xx [ 134 ] ; motionData [ 174 ] = xx [ 10 ] * ( xx [ 159 ] - xx [ 153 ] * xx
[ 107 ] ) + xx [ 146 ] + xx [ 106 ] + xx [ 49 ] ; motionData [ 175 ] = xx [ 2
] ; motionData [ 176 ] = xx [ 2 ] ; motionData [ 177 ] = xx [ 2 ] ;
motionData [ 178 ] = state [ 3 ] ; motionData [ 179 ] = state [ 4 ] ;
motionData [ 180 ] = state [ 5 ] ; motionData [ 181 ] = state [ 7 ] ;
motionData [ 182 ] = xx [ 2 ] ; motionData [ 183 ] = xx [ 2 ] ; motionData [
184 ] = xx [ 154 ] ; motionData [ 185 ] = xx [ 149 ] ; motionData [ 186 ] =
xx [ 156 ] ; motionData [ 187 ] = xx [ 160 ] ; motionData [ 188 ] = xx [ 161
] ; motionData [ 189 ] = xx [ 14 ] ; motionData [ 190 ] = xx [ 165 ] ;
motionData [ 191 ] = xx [ 18 ] ; motionData [ 192 ] = xx [ 167 ] ; motionData
[ 193 ] = xx [ 110 ] ; motionData [ 194 ] = xx [ 111 ] ; motionData [ 195 ] =
xx [ 108 ] ; motionData [ 196 ] = xx [ 162 ] ; motionData [ 197 ] = xx [ 109
] ; motionData [ 198 ] = xx [ 150 ] ; motionData [ 199 ] = xx [ 110 ] - ( xx
[ 32 ] * xx [ 151 ] + xx [ 155 ] * xx [ 33 ] ) * xx [ 10 ] ; motionData [ 200
] = xx [ 111 ] + xx [ 10 ] * ( xx [ 32 ] * xx [ 155 ] - xx [ 151 ] * xx [ 33
] ) ; motionData [ 201 ] = xx [ 108 ] - state [ 13 ] ; motionData [ 202 ] =
xx [ 163 ] - ( xx [ 32 ] * xx [ 166 ] + xx [ 168 ] * xx [ 33 ] ) * xx [ 10 ]
; motionData [ 203 ] = xx [ 164 ] + xx [ 10 ] * ( xx [ 32 ] * xx [ 168 ] - xx
[ 166 ] * xx [ 33 ] ) + xx [ 22 ] * state [ 13 ] ; motionData [ 204 ] = xx [
110 ] * xx [ 35 ] - xx [ 34 ] * xx [ 111 ] + xx [ 150 ] ; motionData [ 205 ]
= xx [ 110 ] + xx [ 10 ] * ( xx [ 36 ] * xx [ 169 ] - xx [ 170 ] * xx [ 37 ]
) ; motionData [ 206 ] = xx [ 111 ] - ( xx [ 36 ] * xx [ 170 ] + xx [ 169 ] *
xx [ 37 ] ) * xx [ 10 ] ; motionData [ 207 ] = xx [ 108 ] + state [ 15 ] ;
motionData [ 208 ] = xx [ 171 ] + xx [ 10 ] * ( xx [ 36 ] * xx [ 173 ] - xx [
174 ] * xx [ 37 ] ) ; motionData [ 209 ] = xx [ 172 ] - ( xx [ 36 ] * xx [
174 ] + xx [ 173 ] * xx [ 37 ] ) * xx [ 10 ] ; motionData [ 210 ] = xx [ 150
] - xx [ 22 ] * xx [ 111 ] ; motionData [ 211 ] = state [ 7 ] - xx [ 177 ] ;
motionData [ 212 ] = xx [ 10 ] * ( xx [ 41 ] * xx [ 175 ] - xx [ 39 ] * xx [
176 ] ) ; motionData [ 213 ] = xx [ 177 ] + state [ 17 ] ; motionData [ 214 ]
= xx [ 185 ] ; motionData [ 215 ] = xx [ 186 ] + xx [ 20 ] * state [ 17 ] ;
motionData [ 216 ] = xx [ 187 ] ; motionData [ 217 ] = xx [ 181 ] ;
motionData [ 218 ] = xx [ 182 ] ; motionData [ 219 ] = xx [ 178 ] ;
motionData [ 220 ] = xx [ 191 ] ; motionData [ 221 ] = xx [ 138 ] ;
motionData [ 222 ] = xx [ 193 ] ; motionData [ 223 ] = xx [ 141 ] ;
motionData [ 224 ] = xx [ 179 ] ; motionData [ 225 ] = xx [ 139 ] ;
motionData [ 226 ] = xx [ 188 ] ; motionData [ 227 ] = xx [ 140 ] ;
motionData [ 228 ] = xx [ 180 ] ; motionData [ 229 ] = xx [ 141 ] - ( xx [ 55
] * xx [ 183 ] + xx [ 184 ] * xx [ 56 ] ) * xx [ 10 ] ; motionData [ 230 ] =
xx [ 179 ] + xx [ 10 ] * ( xx [ 55 ] * xx [ 184 ] - xx [ 183 ] * xx [ 56 ] )
; motionData [ 231 ] = xx [ 139 ] - state [ 23 ] ; motionData [ 232 ] = xx [
189 ] - ( xx [ 55 ] * xx [ 192 ] + xx [ 194 ] * xx [ 56 ] ) * xx [ 10 ] ;
motionData [ 233 ] = xx [ 190 ] + xx [ 10 ] * ( xx [ 55 ] * xx [ 194 ] - xx [
192 ] * xx [ 56 ] ) + xx [ 22 ] * state [ 23 ] ; motionData [ 234 ] = xx [
141 ] * xx [ 58 ] - xx [ 57 ] * xx [ 179 ] + xx [ 180 ] ; motionData [ 235 ]
= xx [ 141 ] + xx [ 10 ] * ( xx [ 59 ] * xx [ 195 ] - xx [ 196 ] * xx [ 60 ]
) ; motionData [ 236 ] = xx [ 179 ] - ( xx [ 59 ] * xx [ 196 ] + xx [ 195 ] *
xx [ 60 ] ) * xx [ 10 ] ; motionData [ 237 ] = xx [ 139 ] + state [ 25 ] ;
motionData [ 238 ] = xx [ 197 ] + xx [ 10 ] * ( xx [ 59 ] * xx [ 199 ] - xx [
200 ] * xx [ 60 ] ) ; motionData [ 239 ] = xx [ 198 ] - ( xx [ 59 ] * xx [
200 ] + xx [ 199 ] * xx [ 60 ] ) * xx [ 10 ] ; motionData [ 240 ] = xx [ 180
] - xx [ 22 ] * xx [ 179 ] ; motionData [ 241 ] = state [ 7 ] - xx [ 203 ] ;
motionData [ 242 ] = xx [ 10 ] * ( xx [ 62 ] * xx [ 201 ] - xx [ 5 ] * xx [
202 ] ) ; motionData [ 243 ] = xx [ 203 ] + state [ 27 ] ; motionData [ 244 ]
= xx [ 211 ] ; motionData [ 245 ] = xx [ 212 ] + xx [ 20 ] * state [ 27 ] ;
motionData [ 246 ] = xx [ 213 ] ; } static size_t computeAssemblyError_0 (
const RuntimeDerivedValuesBundle * rtdv , const int * modeVector , const
double * motionData , double * error ) { const double * rtdvd = rtdv ->
mDoubles . mValues ; const int * rtdvi = rtdv -> mInts . mValues ; double xx
[ 12 ] ; ( void ) rtdvd ; ( void ) rtdvi ; ( void ) modeVector ; xx [ 0 ] =
motionData [ 42 ] ; xx [ 1 ] = motionData [ 43 ] ; xx [ 2 ] = motionData [ 44
] ; xx [ 3 ] = motionData [ 45 ] ; xx [ 4 ] = motionData [ 91 ] ; xx [ 5 ] =
motionData [ 92 ] ; xx [ 6 ] = motionData [ 93 ] ; xx [ 7 ] = motionData [ 94
] ; pm_math_Quaternion_inverseCompose_ra ( xx + 0 , xx + 4 , xx + 8 ) ; xx [
0 ] = 0.125 ; xx [ 1 ] = xx [ 0 ] * motionData [ 93 ] ; xx [ 2 ] = xx [ 0 ] *
motionData [ 94 ] ; xx [ 0 ] = 2.0 ; xx [ 3 ] = 0.075 ; xx [ 4 ] = xx [ 3 ] *
motionData [ 44 ] ; xx [ 5 ] = xx [ 3 ] * motionData [ 45 ] ; error [ 0 ] =
xx [ 9 ] ; error [ 1 ] = xx [ 10 ] ; error [ 2 ] = motionData [ 95 ] + ( xx [
1 ] * motionData [ 93 ] + xx [ 2 ] * motionData [ 94 ] ) * xx [ 0 ] - (
motionData [ 46 ] - ( xx [ 4 ] * motionData [ 44 ] + xx [ 5 ] * motionData [
45 ] ) * xx [ 0 ] ) - 0.2 ; error [ 3 ] = motionData [ 96 ] - ( xx [ 2 ] *
motionData [ 91 ] + xx [ 1 ] * motionData [ 92 ] ) * xx [ 0 ] - ( ( xx [ 5 ]
* motionData [ 42 ] + xx [ 4 ] * motionData [ 43 ] ) * xx [ 0 ] + motionData
[ 47 ] ) ; error [ 4 ] = xx [ 0 ] * ( xx [ 1 ] * motionData [ 91 ] - xx [ 2 ]
* motionData [ 92 ] ) + motionData [ 97 ] - ( xx [ 0 ] * ( xx [ 5 ] *
motionData [ 43 ] - xx [ 4 ] * motionData [ 42 ] ) + motionData [ 48 ] ) ;
return 5 ; } static size_t computeAssemblyError_1 ( const
RuntimeDerivedValuesBundle * rtdv , const int * modeVector , const double *
motionData , double * error ) { const double * rtdvd = rtdv -> mDoubles .
mValues ; const int * rtdvi = rtdv -> mInts . mValues ; double xx [ 12 ] ; (
void ) rtdvd ; ( void ) rtdvi ; ( void ) modeVector ; xx [ 0 ] = motionData [
77 ] ; xx [ 1 ] = motionData [ 78 ] ; xx [ 2 ] = motionData [ 79 ] ; xx [ 3 ]
= motionData [ 80 ] ; xx [ 4 ] = motionData [ 112 ] ; xx [ 5 ] = motionData [
113 ] ; xx [ 6 ] = motionData [ 114 ] ; xx [ 7 ] = motionData [ 115 ] ;
pm_math_Quaternion_inverseCompose_ra ( xx + 0 , xx + 4 , xx + 8 ) ; xx [ 0 ]
= 0.125 ; xx [ 1 ] = xx [ 0 ] * motionData [ 114 ] ; xx [ 2 ] = xx [ 0 ] *
motionData [ 115 ] ; xx [ 0 ] = 2.0 ; xx [ 3 ] = 0.075 ; xx [ 4 ] = xx [ 3 ]
* motionData [ 79 ] ; xx [ 5 ] = xx [ 3 ] * motionData [ 80 ] ; error [ 0 ] =
xx [ 9 ] ; error [ 1 ] = xx [ 10 ] ; error [ 2 ] = motionData [ 116 ] + ( xx
[ 1 ] * motionData [ 114 ] + xx [ 2 ] * motionData [ 115 ] ) * xx [ 0 ] - (
motionData [ 81 ] - ( xx [ 4 ] * motionData [ 79 ] + xx [ 5 ] * motionData [
80 ] ) * xx [ 0 ] ) - 0.2 ; error [ 3 ] = motionData [ 117 ] - ( xx [ 2 ] *
motionData [ 112 ] + xx [ 1 ] * motionData [ 113 ] ) * xx [ 0 ] - ( ( xx [ 5
] * motionData [ 77 ] + xx [ 4 ] * motionData [ 78 ] ) * xx [ 0 ] +
motionData [ 82 ] ) ; error [ 4 ] = xx [ 0 ] * ( xx [ 1 ] * motionData [ 112
] - xx [ 2 ] * motionData [ 113 ] ) + motionData [ 118 ] - ( xx [ 0 ] * ( xx
[ 5 ] * motionData [ 78 ] - xx [ 4 ] * motionData [ 77 ] ) + motionData [ 83
] ) ; return 5 ; } size_t wheel_leg_d9e82f16_1_computeAssemblyError ( const
void * mech , const RuntimeDerivedValuesBundle * rtdv , size_t constraintIdx
, const int * modeVector , const double * motionData , double * error ) { (
void ) mech ; ( void ) rtdv ; ( void ) modeVector ; ( void ) motionData ; (
void ) error ; switch ( constraintIdx ) { case 0 : return
computeAssemblyError_0 ( rtdv , modeVector , motionData , error ) ; case 1 :
return computeAssemblyError_1 ( rtdv , modeVector , motionData , error ) ; }
return 0 ; } static size_t computeAssemblyJacobian_0 ( const
RuntimeDerivedValuesBundle * rtdv , const double * state , const int *
modeVector , const double * motionData , double * J ) { const double * rtdvd
= rtdv -> mDoubles . mValues ; const int * rtdvi = rtdv -> mInts . mValues ;
double xx [ 70 ] ; ( void ) rtdvd ; ( void ) rtdvi ; ( void ) modeVector ; xx
[ 0 ] = 0.0 ; xx [ 1 ] = motionData [ 42 ] ; xx [ 2 ] = motionData [ 43 ] ;
xx [ 3 ] = motionData [ 44 ] ; xx [ 4 ] = motionData [ 45 ] ; xx [ 5 ] =
motionData [ 91 ] ; xx [ 6 ] = motionData [ 92 ] ; xx [ 7 ] = motionData [ 93
] ; xx [ 8 ] = motionData [ 94 ] ; pm_math_Quaternion_inverseCompose_ra ( xx
+ 1 , xx + 5 , xx + 9 ) ; xx [ 1 ] = 2.0 ; xx [ 2 ] = motionData [ 98 ] *
motionData [ 99 ] + motionData [ 100 ] * motionData [ 101 ] ; xx [ 3 ] = 1.0
; xx [ 4 ] = xx [ 3 ] - ( motionData [ 99 ] * motionData [ 99 ] + motionData
[ 100 ] * motionData [ 100 ] ) * xx [ 1 ] ; xx [ 13 ] = xx [ 1 ] * (
motionData [ 99 ] * motionData [ 101 ] - motionData [ 98 ] * motionData [ 100
] ) ; xx [ 14 ] = xx [ 2 ] * xx [ 1 ] ; xx [ 15 ] = xx [ 4 ] ;
pm_math_Quaternion_compDeriv_ra ( xx + 9 , xx + 13 , xx + 16 ) ; xx [ 13 ] =
motionData [ 28 ] * motionData [ 29 ] + motionData [ 30 ] * motionData [ 31 ]
; xx [ 14 ] = xx [ 3 ] - ( motionData [ 29 ] * motionData [ 29 ] + motionData
[ 30 ] * motionData [ 30 ] ) * xx [ 1 ] ; xx [ 20 ] = xx [ 1 ] * ( motionData
[ 29 ] * motionData [ 31 ] - motionData [ 28 ] * motionData [ 30 ] ) ; xx [
21 ] = xx [ 13 ] * xx [ 1 ] ; xx [ 22 ] = xx [ 14 ] ;
pm_math_Quaternion_compDeriv_ra ( xx + 9 , xx + 20 , xx + 23 ) ; xx [ 15 ] =
0.0 ; xx [ 20 ] = xx [ 15 ] ; xx [ 21 ] = xx [ 15 ] ; xx [ 22 ] = - xx [ 3 ]
; pm_math_Quaternion_compDeriv_ra ( xx + 9 , xx + 20 , xx + 27 ) ; xx [ 20 ]
= ( motionData [ 42 ] * motionData [ 44 ] + motionData [ 43 ] * motionData [
45 ] ) * xx [ 1 ] ; xx [ 21 ] = xx [ 1 ] * ( motionData [ 44 ] * motionData [
45 ] - motionData [ 42 ] * motionData [ 43 ] ) ; xx [ 22 ] = xx [ 3 ] - (
motionData [ 43 ] * motionData [ 43 ] + motionData [ 44 ] * motionData [ 44 ]
) * xx [ 1 ] ; pm_math_Quaternion_inverseXform_ra ( xx + 5 , xx + 20 , xx +
31 ) ; xx [ 5 ] = - xx [ 31 ] ; xx [ 6 ] = - xx [ 32 ] ; xx [ 7 ] = - xx [ 33
] ; pm_math_Quaternion_compDeriv_ra ( xx + 9 , xx + 5 , xx + 31 ) ; xx [ 3 ]
= 0.7071067811865476 ; xx [ 5 ] = 0.5 ; xx [ 6 ] = xx [ 5 ] * state [ 8 ] ;
xx [ 7 ] = xx [ 3 ] * cos ( xx [ 6 ] ) ; xx [ 8 ] = - xx [ 7 ] ; xx [ 9 ] =
xx [ 3 ] * sin ( xx [ 6 ] ) ; xx [ 6 ] = - xx [ 9 ] ; xx [ 19 ] = xx [ 8 ] ;
xx [ 20 ] = xx [ 6 ] ; xx [ 21 ] = xx [ 8 ] ; xx [ 22 ] = xx [ 6 ] ; xx [ 34
] = motionData [ 98 ] ; xx [ 35 ] = motionData [ 99 ] ; xx [ 36 ] =
motionData [ 100 ] ; xx [ 37 ] = motionData [ 101 ] ;
pm_math_Quaternion_compose_ra ( xx + 19 , xx + 34 , xx + 38 ) ; xx [ 10 ] =
0.25 ; xx [ 11 ] = xx [ 2 ] * xx [ 10 ] ; xx [ 2 ] = 0.125 ; xx [ 12 ] = xx [
2 ] * xx [ 4 ] ; xx [ 4 ] = xx [ 40 ] * xx [ 11 ] + xx [ 41 ] * xx [ 12 ] ;
xx [ 15 ] = xx [ 39 ] * xx [ 11 ] ; xx [ 16 ] = xx [ 39 ] * xx [ 12 ] ; xx [
19 ] = xx [ 4 ] ; xx [ 20 ] = - xx [ 15 ] ; xx [ 21 ] = - xx [ 16 ] ;
pm_math_Vector3_cross_ra ( xx + 39 , xx + 19 , xx + 34 ) ; xx [ 19 ] = xx [ 6
] ; xx [ 20 ] = xx [ 8 ] ; xx [ 21 ] = xx [ 6 ] ; xx [ 6 ] = xx [ 9 ] *
motionData [ 102 ] ; xx [ 8 ] = xx [ 9 ] * motionData [ 103 ] ; xx [ 22 ] =
xx [ 6 ] + xx [ 7 ] * motionData [ 103 ] ; xx [ 39 ] = xx [ 6 ] ; xx [ 40 ] =
xx [ 8 ] ; xx [ 41 ] = - xx [ 22 ] ; pm_math_Vector3_cross_ra ( xx + 19 , xx
+ 39 , xx + 42 ) ; xx [ 19 ] = 0.075 ; xx [ 20 ] = xx [ 19 ] * xx [ 9 ] ; xx
[ 21 ] = xx [ 20 ] * xx [ 7 ] ; xx [ 23 ] = xx [ 5 ] * state [ 10 ] ; xx [ 26
] = sin ( xx [ 23 ] ) ; xx [ 27 ] = cos ( xx [ 23 ] ) ; xx [ 45 ] = xx [ 26 ]
* motionData [ 17 ] - xx [ 27 ] * motionData [ 14 ] ; xx [ 46 ] = - ( xx [ 27
] * motionData [ 15 ] + xx [ 26 ] * motionData [ 16 ] ) ; xx [ 47 ] = xx [ 26
] * motionData [ 15 ] - xx [ 27 ] * motionData [ 16 ] ; xx [ 48 ] = - ( xx [
26 ] * motionData [ 14 ] + xx [ 27 ] * motionData [ 17 ] ) ; xx [ 49 ] =
motionData [ 28 ] ; xx [ 50 ] = motionData [ 29 ] ; xx [ 51 ] = motionData [
30 ] ; xx [ 52 ] = motionData [ 31 ] ; pm_math_Quaternion_compose_ra ( xx +
45 , xx + 49 , xx + 53 ) ; xx [ 23 ] = xx [ 13 ] * xx [ 10 ] ; xx [ 10 ] = xx
[ 2 ] * xx [ 14 ] ; xx [ 13 ] = xx [ 55 ] * xx [ 23 ] + xx [ 56 ] * xx [ 10 ]
; xx [ 14 ] = xx [ 54 ] * xx [ 23 ] ; xx [ 30 ] = xx [ 54 ] * xx [ 10 ] ; xx
[ 39 ] = xx [ 13 ] ; xx [ 40 ] = - xx [ 14 ] ; xx [ 41 ] = - xx [ 30 ] ;
pm_math_Vector3_cross_ra ( xx + 54 , xx + 39 , xx + 45 ) ; xx [ 31 ] = xx [
26 ] * motionData [ 33 ] ; xx [ 37 ] = xx [ 26 ] * motionData [ 32 ] ; xx [
39 ] = xx [ 2 ] * xx [ 26 ] ; xx [ 40 ] = xx [ 1 ] * ( xx [ 31 ] * xx [ 26 ]
- xx [ 27 ] * xx [ 37 ] ) - motionData [ 33 ] - xx [ 1 ] * xx [ 27 ] * xx [
39 ] ; xx [ 48 ] = motionData [ 15 ] ; xx [ 49 ] = motionData [ 16 ] ; xx [
50 ] = motionData [ 17 ] ; xx [ 41 ] = motionData [ 32 ] - ( ( xx [ 27 ] * xx
[ 31 ] + xx [ 37 ] * xx [ 26 ] ) * xx [ 1 ] + xx [ 1 ] * xx [ 39 ] * xx [ 26
] ) + xx [ 2 ] ; xx [ 26 ] = xx [ 41 ] * motionData [ 17 ] ; xx [ 27 ] = xx [
40 ] * motionData [ 17 ] ; xx [ 31 ] = xx [ 41 ] * motionData [ 15 ] - xx [
40 ] * motionData [ 16 ] ; xx [ 54 ] = - xx [ 26 ] ; xx [ 55 ] = xx [ 27 ] ;
xx [ 56 ] = xx [ 31 ] ; pm_math_Vector3_cross_ra ( xx + 48 , xx + 54 , xx +
57 ) ; xx [ 37 ] = xx [ 5 ] * state [ 12 ] ; xx [ 39 ] = cos ( xx [ 37 ] ) ;
xx [ 48 ] = sin ( xx [ 37 ] ) ; xx [ 37 ] = xx [ 39 ] * motionData [ 84 ] +
xx [ 48 ] * motionData [ 87 ] ; xx [ 49 ] = xx [ 48 ] * motionData [ 84 ] -
xx [ 39 ] * motionData [ 87 ] ; xx [ 50 ] = xx [ 2 ] * xx [ 49 ] ; xx [ 51 ]
= xx [ 39 ] * motionData [ 86 ] + xx [ 48 ] * motionData [ 85 ] ; xx [ 52 ] =
xx [ 48 ] * motionData [ 86 ] - xx [ 39 ] * motionData [ 85 ] ; xx [ 54 ] =
xx [ 2 ] * xx [ 52 ] ; xx [ 55 ] = xx [ 2 ] * xx [ 48 ] ; xx [ 56 ] = xx [ 1
] * xx [ 39 ] * xx [ 55 ] ; xx [ 60 ] = motionData [ 85 ] ; xx [ 61 ] =
motionData [ 86 ] ; xx [ 62 ] = motionData [ 87 ] ; xx [ 39 ] = xx [ 2 ] - xx
[ 1 ] * xx [ 55 ] * xx [ 48 ] ; xx [ 48 ] = xx [ 39 ] * motionData [ 87 ] ;
xx [ 55 ] = xx [ 56 ] * motionData [ 87 ] ; xx [ 63 ] = xx [ 39 ] *
motionData [ 85 ] - xx [ 56 ] * motionData [ 86 ] ; xx [ 64 ] = - xx [ 48 ] ;
xx [ 65 ] = xx [ 55 ] ; xx [ 66 ] = xx [ 63 ] ; pm_math_Vector3_cross_ra ( xx
+ 60 , xx + 64 , xx + 67 ) ; xx [ 60 ] = xx [ 5 ] * state [ 16 ] ; xx [ 5 ] =
xx [ 3 ] * sin ( xx [ 60 ] ) ; xx [ 61 ] = xx [ 19 ] * xx [ 5 ] ; xx [ 62 ] =
xx [ 61 ] * xx [ 3 ] * cos ( xx [ 60 ] ) ; xx [ 3 ] = xx [ 1 ] * ( xx [ 62 ]
- xx [ 62 ] ) ; xx [ 60 ] = xx [ 20 ] * xx [ 9 ] ; xx [ 9 ] = xx [ 61 ] * xx
[ 5 ] ; xx [ 5 ] = ( xx [ 9 ] + xx [ 9 ] ) * xx [ 1 ] ; xx [ 9 ] = ( xx [ 62
] + xx [ 62 ] ) * xx [ 1 ] ; J [ 4 ] = xx [ 17 ] ; J [ 5 ] = xx [ 24 ] ; J [
6 ] = xx [ 28 ] ; J [ 8 ] = xx [ 32 ] ; J [ 18 ] = xx [ 18 ] ; J [ 19 ] = xx
[ 25 ] ; J [ 20 ] = xx [ 29 ] ; J [ 22 ] = xx [ 33 ] ; J [ 32 ] = xx [ 1 ] *
( xx [ 34 ] + xx [ 4 ] * xx [ 38 ] ) + xx [ 1 ] * ( xx [ 42 ] - xx [ 7 ] * xx
[ 6 ] ) - motionData [ 103 ] + xx [ 1 ] * ( xx [ 21 ] - xx [ 21 ] ) ; J [ 33
] = xx [ 1 ] * ( xx [ 45 ] + xx [ 13 ] * xx [ 53 ] ) + xx [ 40 ] + xx [ 1 ] *
( xx [ 57 ] - xx [ 26 ] * motionData [ 14 ] ) ; J [ 34 ] = xx [ 1 ] * ( xx [
37 ] * xx [ 50 ] - xx [ 51 ] * xx [ 54 ] ) + xx [ 56 ] + xx [ 1 ] * ( xx [ 67
] - xx [ 48 ] * motionData [ 84 ] ) ; J [ 36 ] = - ( xx [ 3 ] + xx [ 3 ] ) ;
J [ 46 ] = ( xx [ 35 ] - xx [ 38 ] * xx [ 15 ] ) * xx [ 1 ] - xx [ 12 ] +
motionData [ 102 ] + xx [ 1 ] * ( xx [ 43 ] - xx [ 7 ] * xx [ 8 ] ) - ( xx [
60 ] + xx [ 60 ] ) * xx [ 1 ] + xx [ 19 ] ; J [ 47 ] = ( xx [ 46 ] - xx [ 53
] * xx [ 14 ] ) * xx [ 1 ] - xx [ 10 ] + xx [ 41 ] + ( xx [ 27 ] * motionData
[ 14 ] + xx [ 58 ] ) * xx [ 1 ] ; J [ 48 ] = xx [ 39 ] + ( xx [ 55 ] *
motionData [ 84 ] + xx [ 68 ] ) * xx [ 1 ] - ( xx [ 50 ] * xx [ 49 ] + xx [
54 ] * xx [ 52 ] ) * xx [ 1 ] + xx [ 2 ] ; J [ 50 ] = xx [ 5 ] + xx [ 5 ] -
0.15 ; J [ 60 ] = ( xx [ 36 ] - xx [ 38 ] * xx [ 16 ] ) * xx [ 1 ] + xx [ 11
] + ( xx [ 22 ] * xx [ 7 ] + xx [ 44 ] ) * xx [ 1 ] + ( xx [ 21 ] + xx [ 21 ]
) * xx [ 1 ] ; J [ 61 ] = ( xx [ 47 ] - xx [ 53 ] * xx [ 30 ] ) * xx [ 1 ] +
xx [ 23 ] + ( xx [ 31 ] * motionData [ 14 ] + xx [ 59 ] ) * xx [ 1 ] ; J [ 62
] = ( xx [ 63 ] * motionData [ 84 ] + xx [ 69 ] ) * xx [ 1 ] - ( xx [ 37 ] *
xx [ 54 ] + xx [ 51 ] * xx [ 50 ] ) * xx [ 1 ] ; J [ 64 ] = - ( xx [ 9 ] + xx
[ 9 ] ) ; return 5 ; } static size_t computeAssemblyJacobian_1 ( const
RuntimeDerivedValuesBundle * rtdv , const double * state , const int *
modeVector , const double * motionData , double * J ) { const double * rtdvd
= rtdv -> mDoubles . mValues ; const int * rtdvi = rtdv -> mInts . mValues ;
double xx [ 70 ] ; ( void ) rtdvd ; ( void ) rtdvi ; ( void ) modeVector ; xx
[ 0 ] = 0.0 ; xx [ 1 ] = motionData [ 77 ] ; xx [ 2 ] = motionData [ 78 ] ;
xx [ 3 ] = motionData [ 79 ] ; xx [ 4 ] = motionData [ 80 ] ; xx [ 5 ] =
motionData [ 112 ] ; xx [ 6 ] = motionData [ 113 ] ; xx [ 7 ] = motionData [
114 ] ; xx [ 8 ] = motionData [ 115 ] ; pm_math_Quaternion_inverseCompose_ra
( xx + 1 , xx + 5 , xx + 9 ) ; xx [ 1 ] = 2.0 ; xx [ 2 ] = motionData [ 119 ]
* motionData [ 120 ] + motionData [ 121 ] * motionData [ 122 ] ; xx [ 3 ] =
1.0 ; xx [ 4 ] = xx [ 3 ] - ( motionData [ 120 ] * motionData [ 120 ] +
motionData [ 121 ] * motionData [ 121 ] ) * xx [ 1 ] ; xx [ 13 ] = xx [ 1 ] *
( motionData [ 120 ] * motionData [ 122 ] - motionData [ 119 ] * motionData [
121 ] ) ; xx [ 14 ] = xx [ 2 ] * xx [ 1 ] ; xx [ 15 ] = xx [ 4 ] ;
pm_math_Quaternion_compDeriv_ra ( xx + 9 , xx + 13 , xx + 16 ) ; xx [ 13 ] =
motionData [ 63 ] * motionData [ 64 ] + motionData [ 65 ] * motionData [ 66 ]
; xx [ 14 ] = xx [ 3 ] - ( motionData [ 64 ] * motionData [ 64 ] + motionData
[ 65 ] * motionData [ 65 ] ) * xx [ 1 ] ; xx [ 20 ] = xx [ 1 ] * ( motionData
[ 64 ] * motionData [ 66 ] - motionData [ 63 ] * motionData [ 65 ] ) ; xx [
21 ] = xx [ 13 ] * xx [ 1 ] ; xx [ 22 ] = xx [ 14 ] ;
pm_math_Quaternion_compDeriv_ra ( xx + 9 , xx + 20 , xx + 23 ) ; xx [ 15 ] =
0.0 ; xx [ 20 ] = xx [ 15 ] ; xx [ 21 ] = xx [ 15 ] ; xx [ 22 ] = - xx [ 3 ]
; pm_math_Quaternion_compDeriv_ra ( xx + 9 , xx + 20 , xx + 27 ) ; xx [ 20 ]
= ( motionData [ 77 ] * motionData [ 79 ] + motionData [ 78 ] * motionData [
80 ] ) * xx [ 1 ] ; xx [ 21 ] = xx [ 1 ] * ( motionData [ 79 ] * motionData [
80 ] - motionData [ 77 ] * motionData [ 78 ] ) ; xx [ 22 ] = xx [ 3 ] - (
motionData [ 78 ] * motionData [ 78 ] + motionData [ 79 ] * motionData [ 79 ]
) * xx [ 1 ] ; pm_math_Quaternion_inverseXform_ra ( xx + 5 , xx + 20 , xx +
31 ) ; xx [ 5 ] = - xx [ 31 ] ; xx [ 6 ] = - xx [ 32 ] ; xx [ 7 ] = - xx [ 33
] ; pm_math_Quaternion_compDeriv_ra ( xx + 9 , xx + 5 , xx + 31 ) ; xx [ 3 ]
= 0.7071067811865476 ; xx [ 5 ] = 0.5 ; xx [ 6 ] = xx [ 5 ] * state [ 18 ] ;
xx [ 7 ] = xx [ 3 ] * cos ( xx [ 6 ] ) ; xx [ 8 ] = xx [ 3 ] * sin ( xx [ 6 ]
) ; xx [ 9 ] = xx [ 7 ] ; xx [ 10 ] = xx [ 8 ] ; xx [ 11 ] = xx [ 7 ] ; xx [
12 ] = xx [ 8 ] ; xx [ 19 ] = motionData [ 119 ] ; xx [ 20 ] = motionData [
120 ] ; xx [ 21 ] = motionData [ 121 ] ; xx [ 22 ] = motionData [ 122 ] ;
pm_math_Quaternion_compose_ra ( xx + 9 , xx + 19 , xx + 34 ) ; xx [ 6 ] =
0.25 ; xx [ 9 ] = xx [ 2 ] * xx [ 6 ] ; xx [ 2 ] = 0.125 ; xx [ 10 ] = xx [ 2
] * xx [ 4 ] ; xx [ 4 ] = xx [ 36 ] * xx [ 9 ] + xx [ 37 ] * xx [ 10 ] ; xx [
11 ] = xx [ 35 ] * xx [ 9 ] ; xx [ 12 ] = xx [ 35 ] * xx [ 10 ] ; xx [ 19 ] =
xx [ 4 ] ; xx [ 20 ] = - xx [ 11 ] ; xx [ 21 ] = - xx [ 12 ] ;
pm_math_Vector3_cross_ra ( xx + 35 , xx + 19 , xx + 38 ) ; xx [ 19 ] = xx [ 8
] ; xx [ 20 ] = xx [ 7 ] ; xx [ 21 ] = xx [ 8 ] ; xx [ 15 ] = xx [ 8 ] *
motionData [ 123 ] ; xx [ 16 ] = xx [ 8 ] * motionData [ 124 ] ; xx [ 22 ] =
xx [ 15 ] + xx [ 7 ] * motionData [ 124 ] ; xx [ 35 ] = - xx [ 15 ] ; xx [ 36
] = - xx [ 16 ] ; xx [ 37 ] = xx [ 22 ] ; pm_math_Vector3_cross_ra ( xx + 19
, xx + 35 , xx + 41 ) ; xx [ 19 ] = 0.075 ; xx [ 20 ] = xx [ 19 ] * xx [ 8 ]
; xx [ 21 ] = xx [ 20 ] * xx [ 7 ] ; xx [ 23 ] = xx [ 5 ] * state [ 20 ] ; xx
[ 26 ] = sin ( xx [ 23 ] ) ; xx [ 27 ] = cos ( xx [ 23 ] ) ; xx [ 44 ] = xx [
26 ] * motionData [ 52 ] - xx [ 27 ] * motionData [ 49 ] ; xx [ 45 ] = - ( xx
[ 27 ] * motionData [ 50 ] + xx [ 26 ] * motionData [ 51 ] ) ; xx [ 46 ] = xx
[ 26 ] * motionData [ 50 ] - xx [ 27 ] * motionData [ 51 ] ; xx [ 47 ] = - (
xx [ 26 ] * motionData [ 49 ] + xx [ 27 ] * motionData [ 52 ] ) ; xx [ 48 ] =
motionData [ 63 ] ; xx [ 49 ] = motionData [ 64 ] ; xx [ 50 ] = motionData [
65 ] ; xx [ 51 ] = motionData [ 66 ] ; pm_math_Quaternion_compose_ra ( xx +
44 , xx + 48 , xx + 52 ) ; xx [ 23 ] = xx [ 13 ] * xx [ 6 ] ; xx [ 6 ] = xx [
2 ] * xx [ 14 ] ; xx [ 13 ] = xx [ 54 ] * xx [ 23 ] + xx [ 55 ] * xx [ 6 ] ;
xx [ 14 ] = xx [ 53 ] * xx [ 23 ] ; xx [ 30 ] = xx [ 53 ] * xx [ 6 ] ; xx [
35 ] = xx [ 13 ] ; xx [ 36 ] = - xx [ 14 ] ; xx [ 37 ] = - xx [ 30 ] ;
pm_math_Vector3_cross_ra ( xx + 53 , xx + 35 , xx + 44 ) ; xx [ 31 ] = xx [
26 ] * motionData [ 68 ] ; xx [ 35 ] = xx [ 26 ] * motionData [ 67 ] ; xx [
36 ] = xx [ 2 ] * xx [ 26 ] ; xx [ 37 ] = xx [ 1 ] * ( xx [ 31 ] * xx [ 26 ]
- xx [ 27 ] * xx [ 35 ] ) - motionData [ 68 ] - xx [ 1 ] * xx [ 27 ] * xx [
36 ] ; xx [ 47 ] = motionData [ 50 ] ; xx [ 48 ] = motionData [ 51 ] ; xx [
49 ] = motionData [ 52 ] ; xx [ 50 ] = motionData [ 67 ] - ( ( xx [ 27 ] * xx
[ 31 ] + xx [ 35 ] * xx [ 26 ] ) * xx [ 1 ] + xx [ 1 ] * xx [ 36 ] * xx [ 26
] ) + xx [ 2 ] ; xx [ 26 ] = xx [ 50 ] * motionData [ 52 ] ; xx [ 27 ] = xx [
37 ] * motionData [ 52 ] ; xx [ 31 ] = xx [ 50 ] * motionData [ 50 ] - xx [
37 ] * motionData [ 51 ] ; xx [ 53 ] = - xx [ 26 ] ; xx [ 54 ] = xx [ 27 ] ;
xx [ 55 ] = xx [ 31 ] ; pm_math_Vector3_cross_ra ( xx + 47 , xx + 53 , xx +
56 ) ; xx [ 35 ] = xx [ 5 ] * state [ 22 ] ; xx [ 36 ] = cos ( xx [ 35 ] ) ;
xx [ 47 ] = sin ( xx [ 35 ] ) ; xx [ 35 ] = xx [ 36 ] * motionData [ 105 ] +
xx [ 47 ] * motionData [ 108 ] ; xx [ 48 ] = xx [ 47 ] * motionData [ 105 ] -
xx [ 36 ] * motionData [ 108 ] ; xx [ 49 ] = xx [ 2 ] * xx [ 48 ] ; xx [ 51 ]
= xx [ 36 ] * motionData [ 107 ] + xx [ 47 ] * motionData [ 106 ] ; xx [ 53 ]
= xx [ 47 ] * motionData [ 107 ] - xx [ 36 ] * motionData [ 106 ] ; xx [ 54 ]
= xx [ 2 ] * xx [ 53 ] ; xx [ 55 ] = xx [ 2 ] * xx [ 47 ] ; xx [ 59 ] = xx [
1 ] * xx [ 36 ] * xx [ 55 ] ; xx [ 60 ] = motionData [ 106 ] ; xx [ 61 ] =
motionData [ 107 ] ; xx [ 62 ] = motionData [ 108 ] ; xx [ 36 ] = xx [ 2 ] -
xx [ 1 ] * xx [ 55 ] * xx [ 47 ] ; xx [ 47 ] = xx [ 36 ] * motionData [ 108 ]
; xx [ 55 ] = xx [ 59 ] * motionData [ 108 ] ; xx [ 63 ] = xx [ 36 ] *
motionData [ 106 ] - xx [ 59 ] * motionData [ 107 ] ; xx [ 64 ] = - xx [ 47 ]
; xx [ 65 ] = xx [ 55 ] ; xx [ 66 ] = xx [ 63 ] ; pm_math_Vector3_cross_ra (
xx + 60 , xx + 64 , xx + 67 ) ; xx [ 60 ] = xx [ 5 ] * state [ 26 ] ; xx [ 5
] = xx [ 3 ] * sin ( xx [ 60 ] ) ; xx [ 61 ] = xx [ 19 ] * xx [ 5 ] ; xx [ 62
] = xx [ 61 ] * xx [ 3 ] * cos ( xx [ 60 ] ) ; xx [ 3 ] = xx [ 1 ] * ( xx [
62 ] - xx [ 62 ] ) ; xx [ 60 ] = xx [ 20 ] * xx [ 8 ] ; xx [ 8 ] = xx [ 61 ]
* xx [ 5 ] ; xx [ 5 ] = ( xx [ 8 ] + xx [ 8 ] ) * xx [ 1 ] ; xx [ 8 ] = ( xx
[ 62 ] + xx [ 62 ] ) * xx [ 1 ] ; J [ 9 ] = xx [ 17 ] ; J [ 10 ] = xx [ 24 ]
; J [ 11 ] = xx [ 28 ] ; J [ 13 ] = xx [ 32 ] ; J [ 23 ] = xx [ 18 ] ; J [ 24
] = xx [ 25 ] ; J [ 25 ] = xx [ 29 ] ; J [ 27 ] = xx [ 33 ] ; J [ 37 ] = xx [
1 ] * ( xx [ 38 ] + xx [ 4 ] * xx [ 34 ] ) + xx [ 1 ] * ( xx [ 41 ] - xx [ 7
] * xx [ 15 ] ) - motionData [ 124 ] + xx [ 1 ] * ( xx [ 21 ] - xx [ 21 ] ) ;
J [ 38 ] = xx [ 1 ] * ( xx [ 44 ] + xx [ 13 ] * xx [ 52 ] ) + xx [ 37 ] + xx
[ 1 ] * ( xx [ 56 ] - xx [ 26 ] * motionData [ 49 ] ) ; J [ 39 ] = xx [ 1 ] *
( xx [ 35 ] * xx [ 49 ] - xx [ 51 ] * xx [ 54 ] ) + xx [ 59 ] + xx [ 1 ] * (
xx [ 67 ] - xx [ 47 ] * motionData [ 105 ] ) ; J [ 41 ] = - ( xx [ 3 ] + xx [
3 ] ) ; J [ 51 ] = ( xx [ 39 ] - xx [ 34 ] * xx [ 11 ] ) * xx [ 1 ] - xx [ 10
] + motionData [ 123 ] + xx [ 1 ] * ( xx [ 42 ] - xx [ 7 ] * xx [ 16 ] ) - (
xx [ 60 ] + xx [ 60 ] ) * xx [ 1 ] + xx [ 19 ] ; J [ 52 ] = ( xx [ 45 ] - xx
[ 52 ] * xx [ 14 ] ) * xx [ 1 ] - xx [ 6 ] + xx [ 50 ] + ( xx [ 27 ] *
motionData [ 49 ] + xx [ 57 ] ) * xx [ 1 ] ; J [ 53 ] = xx [ 36 ] + ( xx [ 55
] * motionData [ 105 ] + xx [ 68 ] ) * xx [ 1 ] - ( xx [ 49 ] * xx [ 48 ] +
xx [ 54 ] * xx [ 53 ] ) * xx [ 1 ] + xx [ 2 ] ; J [ 55 ] = xx [ 5 ] + xx [ 5
] - 0.15 ; J [ 65 ] = ( xx [ 40 ] - xx [ 34 ] * xx [ 12 ] ) * xx [ 1 ] + xx [
9 ] + ( xx [ 22 ] * xx [ 7 ] + xx [ 43 ] ) * xx [ 1 ] + ( xx [ 21 ] + xx [ 21
] ) * xx [ 1 ] ; J [ 66 ] = ( xx [ 46 ] - xx [ 52 ] * xx [ 30 ] ) * xx [ 1 ]
+ xx [ 23 ] + ( xx [ 31 ] * motionData [ 49 ] + xx [ 58 ] ) * xx [ 1 ] ; J [
67 ] = ( xx [ 63 ] * motionData [ 105 ] + xx [ 69 ] ) * xx [ 1 ] - ( xx [ 35
] * xx [ 54 ] + xx [ 51 ] * xx [ 49 ] ) * xx [ 1 ] ; J [ 69 ] = - ( xx [ 8 ]
+ xx [ 8 ] ) ; return 5 ; } size_t
wheel_leg_d9e82f16_1_computeAssemblyJacobian ( const void * mech , const
RuntimeDerivedValuesBundle * rtdv , size_t constraintIdx , boolean_T
forVelocitySatisfaction , const double * state , const int * modeVector ,
const double * motionData , double * J ) { ( void ) mech ; ( void ) rtdv ; (
void ) state ; ( void ) modeVector ; ( void ) forVelocitySatisfaction ; (
void ) motionData ; ( void ) J ; switch ( constraintIdx ) { case 0 : return
computeAssemblyJacobian_0 ( rtdv , state , modeVector , motionData , J ) ;
case 1 : return computeAssemblyJacobian_1 ( rtdv , state , modeVector ,
motionData , J ) ; } return 0 ; } size_t
wheel_leg_d9e82f16_1_computeFullAssemblyJacobian ( const void * mech , const
RuntimeDerivedValuesBundle * rtdv , const double * state , const int *
modeVector , const double * motionData , double * J ) { const double * rtdvd
= rtdv -> mDoubles . mValues ; const int * rtdvi = rtdv -> mInts . mValues ;
double xx [ 131 ] ; ( void ) mech ; ( void ) rtdvd ; ( void ) rtdvi ; ( void
) modeVector ; xx [ 0 ] = 0.0 ; xx [ 1 ] = motionData [ 42 ] ; xx [ 2 ] =
motionData [ 43 ] ; xx [ 3 ] = motionData [ 44 ] ; xx [ 4 ] = motionData [ 45
] ; xx [ 5 ] = motionData [ 91 ] ; xx [ 6 ] = motionData [ 92 ] ; xx [ 7 ] =
motionData [ 93 ] ; xx [ 8 ] = motionData [ 94 ] ;
pm_math_Quaternion_inverseCompose_ra ( xx + 1 , xx + 5 , xx + 9 ) ; xx [ 1 ]
= 2.0 ; xx [ 2 ] = motionData [ 98 ] * motionData [ 99 ] + motionData [ 100 ]
* motionData [ 101 ] ; xx [ 3 ] = 1.0 ; xx [ 4 ] = xx [ 3 ] - ( motionData [
99 ] * motionData [ 99 ] + motionData [ 100 ] * motionData [ 100 ] ) * xx [ 1
] ; xx [ 13 ] = xx [ 1 ] * ( motionData [ 99 ] * motionData [ 101 ] -
motionData [ 98 ] * motionData [ 100 ] ) ; xx [ 14 ] = xx [ 2 ] * xx [ 1 ] ;
xx [ 15 ] = xx [ 4 ] ; pm_math_Quaternion_compDeriv_ra ( xx + 9 , xx + 13 ,
xx + 16 ) ; xx [ 13 ] = motionData [ 28 ] * motionData [ 29 ] + motionData [
30 ] * motionData [ 31 ] ; xx [ 14 ] = xx [ 3 ] - ( motionData [ 29 ] *
motionData [ 29 ] + motionData [ 30 ] * motionData [ 30 ] ) * xx [ 1 ] ; xx [
20 ] = xx [ 1 ] * ( motionData [ 29 ] * motionData [ 31 ] - motionData [ 28 ]
* motionData [ 30 ] ) ; xx [ 21 ] = xx [ 13 ] * xx [ 1 ] ; xx [ 22 ] = xx [
14 ] ; pm_math_Quaternion_compDeriv_ra ( xx + 9 , xx + 20 , xx + 23 ) ; xx [
15 ] = 0.0 ; xx [ 20 ] = xx [ 15 ] ; xx [ 21 ] = xx [ 15 ] ; xx [ 22 ] = - xx
[ 3 ] ; pm_math_Quaternion_compDeriv_ra ( xx + 9 , xx + 20 , xx + 27 ) ; xx [
31 ] = ( motionData [ 42 ] * motionData [ 44 ] + motionData [ 43 ] *
motionData [ 45 ] ) * xx [ 1 ] ; xx [ 32 ] = xx [ 1 ] * ( motionData [ 44 ] *
motionData [ 45 ] - motionData [ 42 ] * motionData [ 43 ] ) ; xx [ 33 ] = xx
[ 3 ] - ( motionData [ 43 ] * motionData [ 43 ] + motionData [ 44 ] *
motionData [ 44 ] ) * xx [ 1 ] ; pm_math_Quaternion_inverseXform_ra ( xx + 5
, xx + 31 , xx + 34 ) ; xx [ 5 ] = - xx [ 34 ] ; xx [ 6 ] = - xx [ 35 ] ; xx
[ 7 ] = - xx [ 36 ] ; pm_math_Quaternion_compDeriv_ra ( xx + 9 , xx + 5 , xx
+ 31 ) ; xx [ 5 ] = 0.7071067811865476 ; xx [ 6 ] = 0.5 ; xx [ 7 ] = xx [ 6 ]
* state [ 8 ] ; xx [ 8 ] = xx [ 5 ] * cos ( xx [ 7 ] ) ; xx [ 9 ] = - xx [ 8
] ; xx [ 10 ] = xx [ 5 ] * sin ( xx [ 7 ] ) ; xx [ 7 ] = - xx [ 10 ] ; xx [
34 ] = xx [ 9 ] ; xx [ 35 ] = xx [ 7 ] ; xx [ 36 ] = xx [ 9 ] ; xx [ 37 ] =
xx [ 7 ] ; xx [ 38 ] = motionData [ 98 ] ; xx [ 39 ] = motionData [ 99 ] ; xx
[ 40 ] = motionData [ 100 ] ; xx [ 41 ] = motionData [ 101 ] ;
pm_math_Quaternion_compose_ra ( xx + 34 , xx + 38 , xx + 42 ) ; xx [ 11 ] =
0.25 ; xx [ 12 ] = xx [ 2 ] * xx [ 11 ] ; xx [ 2 ] = 0.125 ; xx [ 15 ] = xx [
2 ] * xx [ 4 ] ; xx [ 4 ] = xx [ 44 ] * xx [ 12 ] + xx [ 45 ] * xx [ 15 ] ;
xx [ 16 ] = xx [ 43 ] * xx [ 12 ] ; xx [ 19 ] = xx [ 43 ] * xx [ 15 ] ; xx [
34 ] = xx [ 4 ] ; xx [ 35 ] = - xx [ 16 ] ; xx [ 36 ] = - xx [ 19 ] ;
pm_math_Vector3_cross_ra ( xx + 43 , xx + 34 , xx + 37 ) ; xx [ 34 ] = xx [ 7
] ; xx [ 35 ] = xx [ 9 ] ; xx [ 36 ] = xx [ 7 ] ; xx [ 7 ] = xx [ 10 ] *
motionData [ 102 ] ; xx [ 9 ] = xx [ 10 ] * motionData [ 103 ] ; xx [ 23 ] =
xx [ 7 ] + xx [ 8 ] * motionData [ 103 ] ; xx [ 43 ] = xx [ 7 ] ; xx [ 44 ] =
xx [ 9 ] ; xx [ 45 ] = - xx [ 23 ] ; pm_math_Vector3_cross_ra ( xx + 34 , xx
+ 43 , xx + 46 ) ; xx [ 26 ] = 0.075 ; xx [ 27 ] = xx [ 26 ] * xx [ 10 ] ; xx
[ 30 ] = xx [ 27 ] * xx [ 8 ] ; xx [ 31 ] = xx [ 6 ] * state [ 10 ] ; xx [ 34
] = sin ( xx [ 31 ] ) ; xx [ 35 ] = cos ( xx [ 31 ] ) ; xx [ 49 ] = xx [ 34 ]
* motionData [ 17 ] - xx [ 35 ] * motionData [ 14 ] ; xx [ 50 ] = - ( xx [ 35
] * motionData [ 15 ] + xx [ 34 ] * motionData [ 16 ] ) ; xx [ 51 ] = xx [ 34
] * motionData [ 15 ] - xx [ 35 ] * motionData [ 16 ] ; xx [ 52 ] = - ( xx [
34 ] * motionData [ 14 ] + xx [ 35 ] * motionData [ 17 ] ) ; xx [ 53 ] =
motionData [ 28 ] ; xx [ 54 ] = motionData [ 29 ] ; xx [ 55 ] = motionData [
30 ] ; xx [ 56 ] = motionData [ 31 ] ; pm_math_Quaternion_compose_ra ( xx +
49 , xx + 53 , xx + 57 ) ; xx [ 31 ] = xx [ 13 ] * xx [ 11 ] ; xx [ 13 ] = xx
[ 2 ] * xx [ 14 ] ; xx [ 14 ] = xx [ 59 ] * xx [ 31 ] + xx [ 60 ] * xx [ 13 ]
; xx [ 36 ] = xx [ 58 ] * xx [ 31 ] ; xx [ 40 ] = xx [ 58 ] * xx [ 13 ] ; xx
[ 43 ] = xx [ 14 ] ; xx [ 44 ] = - xx [ 36 ] ; xx [ 45 ] = - xx [ 40 ] ;
pm_math_Vector3_cross_ra ( xx + 58 , xx + 43 , xx + 49 ) ; xx [ 41 ] = xx [
34 ] * motionData [ 33 ] ; xx [ 43 ] = xx [ 34 ] * motionData [ 32 ] ; xx [
44 ] = xx [ 2 ] * xx [ 34 ] ; xx [ 45 ] = xx [ 1 ] * ( xx [ 41 ] * xx [ 34 ]
- xx [ 35 ] * xx [ 43 ] ) - motionData [ 33 ] - xx [ 1 ] * xx [ 35 ] * xx [
44 ] ; xx [ 52 ] = motionData [ 15 ] ; xx [ 53 ] = motionData [ 16 ] ; xx [
54 ] = motionData [ 17 ] ; xx [ 55 ] = motionData [ 32 ] - ( ( xx [ 35 ] * xx
[ 41 ] + xx [ 43 ] * xx [ 34 ] ) * xx [ 1 ] + xx [ 1 ] * xx [ 44 ] * xx [ 34
] ) + xx [ 2 ] ; xx [ 34 ] = xx [ 55 ] * motionData [ 17 ] ; xx [ 35 ] = xx [
45 ] * motionData [ 17 ] ; xx [ 41 ] = xx [ 55 ] * motionData [ 15 ] - xx [
45 ] * motionData [ 16 ] ; xx [ 58 ] = - xx [ 34 ] ; xx [ 59 ] = xx [ 35 ] ;
xx [ 60 ] = xx [ 41 ] ; pm_math_Vector3_cross_ra ( xx + 52 , xx + 58 , xx +
61 ) ; xx [ 43 ] = xx [ 6 ] * state [ 12 ] ; xx [ 44 ] = cos ( xx [ 43 ] ) ;
xx [ 52 ] = sin ( xx [ 43 ] ) ; xx [ 43 ] = xx [ 44 ] * motionData [ 84 ] +
xx [ 52 ] * motionData [ 87 ] ; xx [ 53 ] = xx [ 52 ] * motionData [ 84 ] -
xx [ 44 ] * motionData [ 87 ] ; xx [ 54 ] = xx [ 2 ] * xx [ 53 ] ; xx [ 56 ]
= xx [ 44 ] * motionData [ 86 ] + xx [ 52 ] * motionData [ 85 ] ; xx [ 58 ] =
xx [ 52 ] * motionData [ 86 ] - xx [ 44 ] * motionData [ 85 ] ; xx [ 59 ] =
xx [ 2 ] * xx [ 58 ] ; xx [ 60 ] = xx [ 2 ] * xx [ 52 ] ; xx [ 64 ] = xx [ 1
] * xx [ 44 ] * xx [ 60 ] ; xx [ 65 ] = motionData [ 85 ] ; xx [ 66 ] =
motionData [ 86 ] ; xx [ 67 ] = motionData [ 87 ] ; xx [ 44 ] = xx [ 2 ] - xx
[ 1 ] * xx [ 60 ] * xx [ 52 ] ; xx [ 52 ] = xx [ 44 ] * motionData [ 87 ] ;
xx [ 60 ] = xx [ 64 ] * motionData [ 87 ] ; xx [ 68 ] = xx [ 44 ] *
motionData [ 85 ] - xx [ 64 ] * motionData [ 86 ] ; xx [ 69 ] = - xx [ 52 ] ;
xx [ 70 ] = xx [ 60 ] ; xx [ 71 ] = xx [ 68 ] ; pm_math_Vector3_cross_ra ( xx
+ 65 , xx + 69 , xx + 72 ) ; xx [ 65 ] = xx [ 6 ] * state [ 16 ] ; xx [ 66 ]
= xx [ 5 ] * sin ( xx [ 65 ] ) ; xx [ 67 ] = xx [ 26 ] * xx [ 66 ] ; xx [ 69
] = xx [ 67 ] * xx [ 5 ] * cos ( xx [ 65 ] ) ; xx [ 65 ] = xx [ 1 ] * ( xx [
69 ] - xx [ 69 ] ) ; xx [ 70 ] = xx [ 27 ] * xx [ 10 ] ; xx [ 10 ] = xx [ 67
] * xx [ 66 ] ; xx [ 27 ] = ( xx [ 10 ] + xx [ 10 ] ) * xx [ 1 ] ; xx [ 10 ]
= 0.15 ; xx [ 66 ] = ( xx [ 69 ] + xx [ 69 ] ) * xx [ 1 ] ; xx [ 75 ] =
motionData [ 77 ] ; xx [ 76 ] = motionData [ 78 ] ; xx [ 77 ] = motionData [
79 ] ; xx [ 78 ] = motionData [ 80 ] ; xx [ 79 ] = motionData [ 112 ] ; xx [
80 ] = motionData [ 113 ] ; xx [ 81 ] = motionData [ 114 ] ; xx [ 82 ] =
motionData [ 115 ] ; pm_math_Quaternion_inverseCompose_ra ( xx + 75 , xx + 79
, xx + 83 ) ; xx [ 67 ] = motionData [ 119 ] * motionData [ 120 ] +
motionData [ 121 ] * motionData [ 122 ] ; xx [ 69 ] = xx [ 3 ] - ( motionData
[ 120 ] * motionData [ 120 ] + motionData [ 121 ] * motionData [ 121 ] ) * xx
[ 1 ] ; xx [ 75 ] = xx [ 1 ] * ( motionData [ 120 ] * motionData [ 122 ] -
motionData [ 119 ] * motionData [ 121 ] ) ; xx [ 76 ] = xx [ 67 ] * xx [ 1 ]
; xx [ 77 ] = xx [ 69 ] ; pm_math_Quaternion_compDeriv_ra ( xx + 83 , xx + 75
, xx + 87 ) ; xx [ 71 ] = motionData [ 63 ] * motionData [ 64 ] + motionData
[ 65 ] * motionData [ 66 ] ; xx [ 75 ] = xx [ 3 ] - ( motionData [ 64 ] *
motionData [ 64 ] + motionData [ 65 ] * motionData [ 65 ] ) * xx [ 1 ] ; xx [
76 ] = xx [ 1 ] * ( motionData [ 64 ] * motionData [ 66 ] - motionData [ 63 ]
* motionData [ 65 ] ) ; xx [ 77 ] = xx [ 71 ] * xx [ 1 ] ; xx [ 78 ] = xx [
75 ] ; pm_math_Quaternion_compDeriv_ra ( xx + 83 , xx + 76 , xx + 91 ) ;
pm_math_Quaternion_compDeriv_ra ( xx + 83 , xx + 20 , xx + 95 ) ; xx [ 20 ] =
( motionData [ 77 ] * motionData [ 79 ] + motionData [ 78 ] * motionData [ 80
] ) * xx [ 1 ] ; xx [ 21 ] = xx [ 1 ] * ( motionData [ 79 ] * motionData [ 80
] - motionData [ 77 ] * motionData [ 78 ] ) ; xx [ 22 ] = xx [ 3 ] - (
motionData [ 78 ] * motionData [ 78 ] + motionData [ 79 ] * motionData [ 79 ]
) * xx [ 1 ] ; pm_math_Quaternion_inverseXform_ra ( xx + 79 , xx + 20 , xx +
76 ) ; xx [ 20 ] = - xx [ 76 ] ; xx [ 21 ] = - xx [ 77 ] ; xx [ 22 ] = - xx [
78 ] ; pm_math_Quaternion_compDeriv_ra ( xx + 83 , xx + 20 , xx + 76 ) ; xx [
3 ] = xx [ 6 ] * state [ 18 ] ; xx [ 20 ] = xx [ 5 ] * cos ( xx [ 3 ] ) ; xx
[ 21 ] = xx [ 5 ] * sin ( xx [ 3 ] ) ; xx [ 79 ] = xx [ 20 ] ; xx [ 80 ] = xx
[ 21 ] ; xx [ 81 ] = xx [ 20 ] ; xx [ 82 ] = xx [ 21 ] ; xx [ 83 ] =
motionData [ 119 ] ; xx [ 84 ] = motionData [ 120 ] ; xx [ 85 ] = motionData
[ 121 ] ; xx [ 86 ] = motionData [ 122 ] ; pm_math_Quaternion_compose_ra ( xx
+ 79 , xx + 83 , xx + 98 ) ; xx [ 3 ] = xx [ 67 ] * xx [ 11 ] ; xx [ 22 ] =
xx [ 2 ] * xx [ 69 ] ; xx [ 67 ] = xx [ 100 ] * xx [ 3 ] + xx [ 101 ] * xx [
22 ] ; xx [ 69 ] = xx [ 99 ] * xx [ 3 ] ; xx [ 76 ] = xx [ 99 ] * xx [ 22 ] ;
xx [ 79 ] = xx [ 67 ] ; xx [ 80 ] = - xx [ 69 ] ; xx [ 81 ] = - xx [ 76 ] ;
pm_math_Vector3_cross_ra ( xx + 99 , xx + 79 , xx + 82 ) ; xx [ 79 ] = xx [
21 ] ; xx [ 80 ] = xx [ 20 ] ; xx [ 81 ] = xx [ 21 ] ; xx [ 85 ] = xx [ 21 ]
* motionData [ 123 ] ; xx [ 86 ] = xx [ 21 ] * motionData [ 124 ] ; xx [ 87 ]
= xx [ 85 ] + xx [ 20 ] * motionData [ 124 ] ; xx [ 99 ] = - xx [ 85 ] ; xx [
100 ] = - xx [ 86 ] ; xx [ 101 ] = xx [ 87 ] ; pm_math_Vector3_cross_ra ( xx
+ 79 , xx + 99 , xx + 102 ) ; xx [ 79 ] = xx [ 26 ] * xx [ 21 ] ; xx [ 80 ] =
xx [ 79 ] * xx [ 20 ] ; xx [ 81 ] = xx [ 6 ] * state [ 20 ] ; xx [ 90 ] = sin
( xx [ 81 ] ) ; xx [ 91 ] = cos ( xx [ 81 ] ) ; xx [ 105 ] = xx [ 90 ] *
motionData [ 52 ] - xx [ 91 ] * motionData [ 49 ] ; xx [ 106 ] = - ( xx [ 91
] * motionData [ 50 ] + xx [ 90 ] * motionData [ 51 ] ) ; xx [ 107 ] = xx [
90 ] * motionData [ 50 ] - xx [ 91 ] * motionData [ 51 ] ; xx [ 108 ] = - (
xx [ 90 ] * motionData [ 49 ] + xx [ 91 ] * motionData [ 52 ] ) ; xx [ 109 ]
= motionData [ 63 ] ; xx [ 110 ] = motionData [ 64 ] ; xx [ 111 ] =
motionData [ 65 ] ; xx [ 112 ] = motionData [ 66 ] ;
pm_math_Quaternion_compose_ra ( xx + 105 , xx + 109 , xx + 113 ) ; xx [ 81 ]
= xx [ 71 ] * xx [ 11 ] ; xx [ 11 ] = xx [ 2 ] * xx [ 75 ] ; xx [ 71 ] = xx [
115 ] * xx [ 81 ] + xx [ 116 ] * xx [ 11 ] ; xx [ 75 ] = xx [ 114 ] * xx [ 81
] ; xx [ 94 ] = xx [ 114 ] * xx [ 11 ] ; xx [ 99 ] = xx [ 71 ] ; xx [ 100 ] =
- xx [ 75 ] ; xx [ 101 ] = - xx [ 94 ] ; pm_math_Vector3_cross_ra ( xx + 114
, xx + 99 , xx + 105 ) ; xx [ 95 ] = xx [ 90 ] * motionData [ 68 ] ; xx [ 99
] = xx [ 90 ] * motionData [ 67 ] ; xx [ 100 ] = xx [ 2 ] * xx [ 90 ] ; xx [
101 ] = xx [ 1 ] * ( xx [ 95 ] * xx [ 90 ] - xx [ 91 ] * xx [ 99 ] ) -
motionData [ 68 ] - xx [ 1 ] * xx [ 91 ] * xx [ 100 ] ; xx [ 108 ] =
motionData [ 50 ] ; xx [ 109 ] = motionData [ 51 ] ; xx [ 110 ] = motionData
[ 52 ] ; xx [ 111 ] = motionData [ 67 ] - ( ( xx [ 91 ] * xx [ 95 ] + xx [ 99
] * xx [ 90 ] ) * xx [ 1 ] + xx [ 1 ] * xx [ 100 ] * xx [ 90 ] ) + xx [ 2 ] ;
xx [ 90 ] = xx [ 111 ] * motionData [ 52 ] ; xx [ 91 ] = xx [ 101 ] *
motionData [ 52 ] ; xx [ 95 ] = xx [ 111 ] * motionData [ 50 ] - xx [ 101 ] *
motionData [ 51 ] ; xx [ 114 ] = - xx [ 90 ] ; xx [ 115 ] = xx [ 91 ] ; xx [
116 ] = xx [ 95 ] ; pm_math_Vector3_cross_ra ( xx + 108 , xx + 114 , xx + 117
) ; xx [ 99 ] = xx [ 6 ] * state [ 22 ] ; xx [ 100 ] = cos ( xx [ 99 ] ) ; xx
[ 108 ] = sin ( xx [ 99 ] ) ; xx [ 99 ] = xx [ 100 ] * motionData [ 105 ] +
xx [ 108 ] * motionData [ 108 ] ; xx [ 109 ] = xx [ 108 ] * motionData [ 105
] - xx [ 100 ] * motionData [ 108 ] ; xx [ 110 ] = xx [ 2 ] * xx [ 109 ] ; xx
[ 112 ] = xx [ 100 ] * motionData [ 107 ] + xx [ 108 ] * motionData [ 106 ] ;
xx [ 114 ] = xx [ 108 ] * motionData [ 107 ] - xx [ 100 ] * motionData [ 106
] ; xx [ 115 ] = xx [ 2 ] * xx [ 114 ] ; xx [ 116 ] = xx [ 2 ] * xx [ 108 ] ;
xx [ 120 ] = xx [ 1 ] * xx [ 100 ] * xx [ 116 ] ; xx [ 121 ] = motionData [
106 ] ; xx [ 122 ] = motionData [ 107 ] ; xx [ 123 ] = motionData [ 108 ] ;
xx [ 100 ] = xx [ 2 ] - xx [ 1 ] * xx [ 116 ] * xx [ 108 ] ; xx [ 108 ] = xx
[ 100 ] * motionData [ 108 ] ; xx [ 116 ] = xx [ 120 ] * motionData [ 108 ] ;
xx [ 124 ] = xx [ 100 ] * motionData [ 106 ] - xx [ 120 ] * motionData [ 107
] ; xx [ 125 ] = - xx [ 108 ] ; xx [ 126 ] = xx [ 116 ] ; xx [ 127 ] = xx [
124 ] ; pm_math_Vector3_cross_ra ( xx + 121 , xx + 125 , xx + 128 ) ; xx [
121 ] = xx [ 6 ] * state [ 26 ] ; xx [ 6 ] = xx [ 5 ] * sin ( xx [ 121 ] ) ;
xx [ 122 ] = xx [ 26 ] * xx [ 6 ] ; xx [ 123 ] = xx [ 122 ] * xx [ 5 ] * cos
( xx [ 121 ] ) ; xx [ 5 ] = xx [ 1 ] * ( xx [ 123 ] - xx [ 123 ] ) ; xx [ 121
] = xx [ 79 ] * xx [ 21 ] ; xx [ 21 ] = xx [ 122 ] * xx [ 6 ] ; xx [ 6 ] = (
xx [ 21 ] + xx [ 21 ] ) * xx [ 1 ] ; xx [ 21 ] = ( xx [ 123 ] + xx [ 123 ] )
* xx [ 1 ] ; J [ 4 ] = xx [ 17 ] ; J [ 5 ] = xx [ 24 ] ; J [ 6 ] = xx [ 28 ]
; J [ 8 ] = xx [ 32 ] ; J [ 18 ] = xx [ 18 ] ; J [ 19 ] = xx [ 25 ] ; J [ 20
] = xx [ 29 ] ; J [ 22 ] = xx [ 33 ] ; J [ 32 ] = xx [ 1 ] * ( xx [ 37 ] + xx
[ 4 ] * xx [ 42 ] ) + xx [ 1 ] * ( xx [ 46 ] - xx [ 8 ] * xx [ 7 ] ) -
motionData [ 103 ] + xx [ 1 ] * ( xx [ 30 ] - xx [ 30 ] ) ; J [ 33 ] = xx [ 1
] * ( xx [ 49 ] + xx [ 14 ] * xx [ 57 ] ) + xx [ 45 ] + xx [ 1 ] * ( xx [ 61
] - xx [ 34 ] * motionData [ 14 ] ) ; J [ 34 ] = xx [ 1 ] * ( xx [ 43 ] * xx
[ 54 ] - xx [ 56 ] * xx [ 59 ] ) + xx [ 64 ] + xx [ 1 ] * ( xx [ 72 ] - xx [
52 ] * motionData [ 84 ] ) ; J [ 36 ] = - ( xx [ 65 ] + xx [ 65 ] ) ; J [ 46
] = ( xx [ 38 ] - xx [ 42 ] * xx [ 16 ] ) * xx [ 1 ] - xx [ 15 ] + motionData
[ 102 ] + xx [ 1 ] * ( xx [ 47 ] - xx [ 8 ] * xx [ 9 ] ) - ( xx [ 70 ] + xx [
70 ] ) * xx [ 1 ] + xx [ 26 ] ; J [ 47 ] = ( xx [ 50 ] - xx [ 57 ] * xx [ 36
] ) * xx [ 1 ] - xx [ 13 ] + xx [ 55 ] + ( xx [ 35 ] * motionData [ 14 ] + xx
[ 62 ] ) * xx [ 1 ] ; J [ 48 ] = xx [ 44 ] + ( xx [ 60 ] * motionData [ 84 ]
+ xx [ 73 ] ) * xx [ 1 ] - ( xx [ 54 ] * xx [ 53 ] + xx [ 59 ] * xx [ 58 ] )
* xx [ 1 ] + xx [ 2 ] ; J [ 50 ] = xx [ 27 ] + xx [ 27 ] - xx [ 10 ] ; J [ 60
] = ( xx [ 39 ] - xx [ 42 ] * xx [ 19 ] ) * xx [ 1 ] + xx [ 12 ] + ( xx [ 23
] * xx [ 8 ] + xx [ 48 ] ) * xx [ 1 ] + ( xx [ 30 ] + xx [ 30 ] ) * xx [ 1 ]
; J [ 61 ] = ( xx [ 51 ] - xx [ 57 ] * xx [ 40 ] ) * xx [ 1 ] + xx [ 31 ] + (
xx [ 41 ] * motionData [ 14 ] + xx [ 63 ] ) * xx [ 1 ] ; J [ 62 ] = ( xx [ 68
] * motionData [ 84 ] + xx [ 74 ] ) * xx [ 1 ] - ( xx [ 43 ] * xx [ 59 ] + xx
[ 56 ] * xx [ 54 ] ) * xx [ 1 ] ; J [ 64 ] = - ( xx [ 66 ] + xx [ 66 ] ) ; J
[ 79 ] = xx [ 88 ] ; J [ 80 ] = xx [ 92 ] ; J [ 81 ] = xx [ 96 ] ; J [ 83 ] =
xx [ 77 ] ; J [ 93 ] = xx [ 89 ] ; J [ 94 ] = xx [ 93 ] ; J [ 95 ] = xx [ 97
] ; J [ 97 ] = xx [ 78 ] ; J [ 107 ] = xx [ 1 ] * ( xx [ 82 ] + xx [ 67 ] *
xx [ 98 ] ) + xx [ 1 ] * ( xx [ 102 ] - xx [ 20 ] * xx [ 85 ] ) - motionData
[ 124 ] + xx [ 1 ] * ( xx [ 80 ] - xx [ 80 ] ) ; J [ 108 ] = xx [ 1 ] * ( xx
[ 105 ] + xx [ 71 ] * xx [ 113 ] ) + xx [ 101 ] + xx [ 1 ] * ( xx [ 117 ] -
xx [ 90 ] * motionData [ 49 ] ) ; J [ 109 ] = xx [ 1 ] * ( xx [ 99 ] * xx [
110 ] - xx [ 112 ] * xx [ 115 ] ) + xx [ 120 ] + xx [ 1 ] * ( xx [ 128 ] - xx
[ 108 ] * motionData [ 105 ] ) ; J [ 111 ] = - ( xx [ 5 ] + xx [ 5 ] ) ; J [
121 ] = ( xx [ 83 ] - xx [ 98 ] * xx [ 69 ] ) * xx [ 1 ] - xx [ 22 ] +
motionData [ 123 ] + xx [ 1 ] * ( xx [ 103 ] - xx [ 20 ] * xx [ 86 ] ) - ( xx
[ 121 ] + xx [ 121 ] ) * xx [ 1 ] + xx [ 26 ] ; J [ 122 ] = ( xx [ 106 ] - xx
[ 113 ] * xx [ 75 ] ) * xx [ 1 ] - xx [ 11 ] + xx [ 111 ] + ( xx [ 91 ] *
motionData [ 49 ] + xx [ 118 ] ) * xx [ 1 ] ; J [ 123 ] = xx [ 100 ] + ( xx [
116 ] * motionData [ 105 ] + xx [ 129 ] ) * xx [ 1 ] - ( xx [ 110 ] * xx [
109 ] + xx [ 115 ] * xx [ 114 ] ) * xx [ 1 ] + xx [ 2 ] ; J [ 125 ] = xx [ 6
] + xx [ 6 ] - xx [ 10 ] ; J [ 135 ] = ( xx [ 84 ] - xx [ 98 ] * xx [ 76 ] )
* xx [ 1 ] + xx [ 3 ] + ( xx [ 87 ] * xx [ 20 ] + xx [ 104 ] ) * xx [ 1 ] + (
xx [ 80 ] + xx [ 80 ] ) * xx [ 1 ] ; J [ 136 ] = ( xx [ 107 ] - xx [ 113 ] *
xx [ 94 ] ) * xx [ 1 ] + xx [ 81 ] + ( xx [ 95 ] * motionData [ 49 ] + xx [
119 ] ) * xx [ 1 ] ; J [ 137 ] = ( xx [ 124 ] * motionData [ 105 ] + xx [ 130
] ) * xx [ 1 ] - ( xx [ 99 ] * xx [ 115 ] + xx [ 112 ] * xx [ 110 ] ) * xx [
1 ] ; J [ 139 ] = - ( xx [ 21 ] + xx [ 21 ] ) ; return 10 ; } static int
isInKinematicSingularity_0 ( const RuntimeDerivedValuesBundle * rtdv , const
int * modeVector , const double * motionData ) { const double * rtdvd = rtdv
-> mDoubles . mValues ; const int * rtdvi = rtdv -> mInts . mValues ; ( void
) rtdvd ; ( void ) rtdvi ; ( void ) modeVector ; ( void ) motionData ; return
0 ; } static int isInKinematicSingularity_1 ( const
RuntimeDerivedValuesBundle * rtdv , const int * modeVector , const double *
motionData ) { const double * rtdvd = rtdv -> mDoubles . mValues ; const int
* rtdvi = rtdv -> mInts . mValues ; ( void ) rtdvd ; ( void ) rtdvi ; ( void
) modeVector ; ( void ) motionData ; return 0 ; } int
wheel_leg_d9e82f16_1_isInKinematicSingularity ( const void * mech , const
RuntimeDerivedValuesBundle * rtdv , size_t constraintIdx , const int *
modeVector , const double * motionData ) { ( void ) mech ; ( void ) rtdv ; (
void ) modeVector ; ( void ) motionData ; switch ( constraintIdx ) { case 0 :
return isInKinematicSingularity_0 ( rtdv , modeVector , motionData ) ; case 1
: return isInKinematicSingularity_1 ( rtdv , modeVector , motionData ) ; }
return 0 ; } void wheel_leg_d9e82f16_1_convertStateVector ( const void *
asmMech , const RuntimeDerivedValuesBundle * rtdv , const void * simMech ,
const double * asmState , const int * asmModeVector , const int *
simModeVector , double * simState ) { const double * rtdvd = rtdv -> mDoubles
. mValues ; const int * rtdvi = rtdv -> mInts . mValues ; ( void ) asmMech ;
( void ) rtdvd ; ( void ) rtdvi ; ( void ) simMech ; ( void ) asmModeVector ;
( void ) simModeVector ; simState [ 0 ] = asmState [ 0 ] ; simState [ 1 ] =
asmState [ 1 ] ; simState [ 2 ] = asmState [ 2 ] ; simState [ 3 ] = asmState
[ 3 ] ; simState [ 4 ] = asmState [ 4 ] ; simState [ 5 ] = asmState [ 5 ] ;
simState [ 6 ] = asmState [ 6 ] ; simState [ 7 ] = asmState [ 7 ] ; simState
[ 8 ] = asmState [ 8 ] ; simState [ 9 ] = asmState [ 9 ] ; simState [ 10 ] =
asmState [ 10 ] ; simState [ 11 ] = asmState [ 11 ] ; simState [ 12 ] =
asmState [ 14 ] ; simState [ 13 ] = asmState [ 15 ] ; simState [ 14 ] =
asmState [ 16 ] ; simState [ 15 ] = asmState [ 17 ] ; simState [ 16 ] =
asmState [ 28 ] ; simState [ 17 ] = asmState [ 29 ] ; simState [ 18 ] =
asmState [ 18 ] ; simState [ 19 ] = asmState [ 19 ] ; simState [ 20 ] =
asmState [ 20 ] ; simState [ 21 ] = asmState [ 21 ] ; simState [ 22 ] =
asmState [ 24 ] ; simState [ 23 ] = asmState [ 25 ] ; simState [ 24 ] =
asmState [ 26 ] ; simState [ 25 ] = asmState [ 27 ] ; simState [ 26 ] =
asmState [ 30 ] ; simState [ 27 ] = asmState [ 31 ] ; simState [ 28 ] =
asmState [ 12 ] ; simState [ 29 ] = asmState [ 13 ] ; simState [ 30 ] =
asmState [ 22 ] ; simState [ 31 ] = asmState [ 23 ] ; }
