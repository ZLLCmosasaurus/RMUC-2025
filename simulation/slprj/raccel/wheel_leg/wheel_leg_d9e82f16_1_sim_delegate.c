#include <math.h>
#include <string.h>
#include "pm_std.h"
#include "sm_std.h"
#include "ne_std.h"
#include "ne_dae.h"
#include "sm_ssci_run_time_errors.h"
#include "sm_RuntimeDerivedValuesBundle.h"
void wheel_leg_d9e82f16_1_resetSimStateVector ( const void * mech , double *
state ) { double xx [ 1 ] ; ( void ) mech ; xx [ 0 ] = 0.0 ; state [ 0 ] = xx
[ 0 ] ; state [ 1 ] = xx [ 0 ] ; state [ 2 ] = xx [ 0 ] ; state [ 3 ] = xx [
0 ] ; state [ 4 ] = xx [ 0 ] ; state [ 5 ] = xx [ 0 ] ; state [ 6 ] = xx [ 0
] ; state [ 7 ] = xx [ 0 ] ; state [ 8 ] = xx [ 0 ] ; state [ 9 ] = xx [ 0 ]
; state [ 10 ] = xx [ 0 ] ; state [ 11 ] = xx [ 0 ] ; state [ 12 ] = xx [ 0 ]
; state [ 13 ] = xx [ 0 ] ; state [ 14 ] = xx [ 0 ] ; state [ 15 ] = xx [ 0 ]
; state [ 16 ] = xx [ 0 ] ; state [ 17 ] = xx [ 0 ] ; state [ 18 ] = xx [ 0 ]
; state [ 19 ] = xx [ 0 ] ; state [ 20 ] = xx [ 0 ] ; state [ 21 ] = xx [ 0 ]
; state [ 22 ] = xx [ 0 ] ; state [ 23 ] = xx [ 0 ] ; state [ 24 ] = xx [ 0 ]
; state [ 25 ] = xx [ 0 ] ; state [ 26 ] = xx [ 0 ] ; state [ 27 ] = xx [ 0 ]
; state [ 28 ] = xx [ 0 ] ; state [ 29 ] = xx [ 0 ] ; state [ 30 ] = xx [ 0 ]
; state [ 31 ] = xx [ 0 ] ; } static void perturbSimJointPrimitiveState_0_0 (
double mag , double * state ) { state [ 0 ] = state [ 0 ] + mag ; } static
void perturbSimJointPrimitiveState_0_0v ( double mag , double * state ) {
state [ 0 ] = state [ 0 ] + mag ; state [ 3 ] = state [ 3 ] - 0.875 * mag ; }
static void perturbSimJointPrimitiveState_0_1 ( double mag , double * state )
{ state [ 1 ] = state [ 1 ] + mag ; } static void
perturbSimJointPrimitiveState_0_1v ( double mag , double * state ) { state [
1 ] = state [ 1 ] + mag ; state [ 4 ] = state [ 4 ] - 0.875 * mag ; } static
void perturbSimJointPrimitiveState_0_2 ( double mag , double * state ) {
state [ 2 ] = state [ 2 ] + mag ; } static void
perturbSimJointPrimitiveState_0_2v ( double mag , double * state ) { state [
2 ] = state [ 2 ] + mag ; state [ 5 ] = state [ 5 ] - 0.875 * mag ; } static
void perturbSimJointPrimitiveState_1_0 ( double mag , double * state ) {
state [ 6 ] = state [ 6 ] + mag ; } static void
perturbSimJointPrimitiveState_1_0v ( double mag , double * state ) { state [
6 ] = state [ 6 ] + mag ; state [ 7 ] = state [ 7 ] - 0.875 * mag ; } static
void perturbSimJointPrimitiveState_2_0 ( double mag , double * state ) {
state [ 8 ] = state [ 8 ] + mag ; } static void
perturbSimJointPrimitiveState_2_0v ( double mag , double * state ) { state [
8 ] = state [ 8 ] + mag ; state [ 9 ] = state [ 9 ] - 0.875 * mag ; } static
void perturbSimJointPrimitiveState_3_0 ( double mag , double * state ) {
state [ 10 ] = state [ 10 ] + mag ; } static void
perturbSimJointPrimitiveState_3_0v ( double mag , double * state ) { state [
10 ] = state [ 10 ] + mag ; state [ 11 ] = state [ 11 ] - 0.875 * mag ; }
static void perturbSimJointPrimitiveState_4_0 ( double mag , double * state )
{ state [ 12 ] = state [ 12 ] + mag ; } static void
perturbSimJointPrimitiveState_4_0v ( double mag , double * state ) { state [
12 ] = state [ 12 ] + mag ; state [ 13 ] = state [ 13 ] - 0.875 * mag ; }
static void perturbSimJointPrimitiveState_5_0 ( double mag , double * state )
{ state [ 14 ] = state [ 14 ] + mag ; } static void
perturbSimJointPrimitiveState_5_0v ( double mag , double * state ) { state [
14 ] = state [ 14 ] + mag ; state [ 15 ] = state [ 15 ] - 0.875 * mag ; }
static void perturbSimJointPrimitiveState_6_0 ( double mag , double * state )
{ state [ 16 ] = state [ 16 ] + mag ; } static void
perturbSimJointPrimitiveState_6_0v ( double mag , double * state ) { state [
16 ] = state [ 16 ] + mag ; state [ 17 ] = state [ 17 ] - 0.875 * mag ; }
static void perturbSimJointPrimitiveState_7_0 ( double mag , double * state )
{ state [ 18 ] = state [ 18 ] + mag ; } static void
perturbSimJointPrimitiveState_7_0v ( double mag , double * state ) { state [
18 ] = state [ 18 ] + mag ; state [ 19 ] = state [ 19 ] - 0.875 * mag ; }
static void perturbSimJointPrimitiveState_8_0 ( double mag , double * state )
{ state [ 20 ] = state [ 20 ] + mag ; } static void
perturbSimJointPrimitiveState_8_0v ( double mag , double * state ) { state [
20 ] = state [ 20 ] + mag ; state [ 21 ] = state [ 21 ] - 0.875 * mag ; }
static void perturbSimJointPrimitiveState_9_0 ( double mag , double * state )
{ state [ 22 ] = state [ 22 ] + mag ; } static void
perturbSimJointPrimitiveState_9_0v ( double mag , double * state ) { state [
22 ] = state [ 22 ] + mag ; state [ 23 ] = state [ 23 ] - 0.875 * mag ; }
static void perturbSimJointPrimitiveState_10_0 ( double mag , double * state
) { state [ 24 ] = state [ 24 ] + mag ; } static void
perturbSimJointPrimitiveState_10_0v ( double mag , double * state ) { state [
24 ] = state [ 24 ] + mag ; state [ 25 ] = state [ 25 ] - 0.875 * mag ; }
static void perturbSimJointPrimitiveState_11_0 ( double mag , double * state
) { state [ 26 ] = state [ 26 ] + mag ; } static void
perturbSimJointPrimitiveState_11_0v ( double mag , double * state ) { state [
26 ] = state [ 26 ] + mag ; state [ 27 ] = state [ 27 ] - 0.875 * mag ; }
void wheel_leg_d9e82f16_1_perturbSimJointPrimitiveState ( const void * mech ,
size_t stageIdx , size_t primIdx , double mag , boolean_T doPerturbVelocity ,
double * state ) { ( void ) mech ; ( void ) stageIdx ; ( void ) primIdx ; (
void ) mag ; ( void ) doPerturbVelocity ; ( void ) state ; switch ( (
stageIdx * 6 + primIdx ) * 2 + ( doPerturbVelocity ? 1 : 0 ) ) { case 0 :
perturbSimJointPrimitiveState_0_0 ( mag , state ) ; break ; case 1 :
perturbSimJointPrimitiveState_0_0v ( mag , state ) ; break ; case 2 :
perturbSimJointPrimitiveState_0_1 ( mag , state ) ; break ; case 3 :
perturbSimJointPrimitiveState_0_1v ( mag , state ) ; break ; case 4 :
perturbSimJointPrimitiveState_0_2 ( mag , state ) ; break ; case 5 :
perturbSimJointPrimitiveState_0_2v ( mag , state ) ; break ; case 12 :
perturbSimJointPrimitiveState_1_0 ( mag , state ) ; break ; case 13 :
perturbSimJointPrimitiveState_1_0v ( mag , state ) ; break ; case 24 :
perturbSimJointPrimitiveState_2_0 ( mag , state ) ; break ; case 25 :
perturbSimJointPrimitiveState_2_0v ( mag , state ) ; break ; case 36 :
perturbSimJointPrimitiveState_3_0 ( mag , state ) ; break ; case 37 :
perturbSimJointPrimitiveState_3_0v ( mag , state ) ; break ; case 48 :
perturbSimJointPrimitiveState_4_0 ( mag , state ) ; break ; case 49 :
perturbSimJointPrimitiveState_4_0v ( mag , state ) ; break ; case 60 :
perturbSimJointPrimitiveState_5_0 ( mag , state ) ; break ; case 61 :
perturbSimJointPrimitiveState_5_0v ( mag , state ) ; break ; case 72 :
perturbSimJointPrimitiveState_6_0 ( mag , state ) ; break ; case 73 :
perturbSimJointPrimitiveState_6_0v ( mag , state ) ; break ; case 84 :
perturbSimJointPrimitiveState_7_0 ( mag , state ) ; break ; case 85 :
perturbSimJointPrimitiveState_7_0v ( mag , state ) ; break ; case 96 :
perturbSimJointPrimitiveState_8_0 ( mag , state ) ; break ; case 97 :
perturbSimJointPrimitiveState_8_0v ( mag , state ) ; break ; case 108 :
perturbSimJointPrimitiveState_9_0 ( mag , state ) ; break ; case 109 :
perturbSimJointPrimitiveState_9_0v ( mag , state ) ; break ; case 120 :
perturbSimJointPrimitiveState_10_0 ( mag , state ) ; break ; case 121 :
perturbSimJointPrimitiveState_10_0v ( mag , state ) ; break ; case 132 :
perturbSimJointPrimitiveState_11_0 ( mag , state ) ; break ; case 133 :
perturbSimJointPrimitiveState_11_0v ( mag , state ) ; break ; } } void
wheel_leg_d9e82f16_1_perturbFlexibleBodyState ( const void * mech , size_t
stageIdx , double mag , boolean_T doPerturbVelocity , double * state ) { (
void ) mech ; ( void ) stageIdx ; ( void ) mag ; ( void ) doPerturbVelocity ;
( void ) state ; switch ( stageIdx * 2 + ( doPerturbVelocity ? 1 : 0 ) ) { }
} void wheel_leg_d9e82f16_1_constructStateVector ( const void * mech , const
double * solverState , const double * u , const double * uDot , double *
discreteState , double * fullState ) { ( void ) mech ; ( void ) u ; ( void )
uDot ; ( void ) discreteState ; fullState [ 0 ] = solverState [ 0 ] ;
fullState [ 1 ] = solverState [ 1 ] ; fullState [ 2 ] = solverState [ 2 ] ;
fullState [ 3 ] = solverState [ 3 ] ; fullState [ 4 ] = solverState [ 4 ] ;
fullState [ 5 ] = solverState [ 5 ] ; fullState [ 6 ] = solverState [ 6 ] ;
fullState [ 7 ] = solverState [ 7 ] ; fullState [ 8 ] = solverState [ 8 ] ;
fullState [ 9 ] = solverState [ 9 ] ; fullState [ 10 ] = solverState [ 10 ] ;
fullState [ 11 ] = solverState [ 11 ] ; fullState [ 12 ] = solverState [ 12 ]
; fullState [ 13 ] = solverState [ 13 ] ; fullState [ 14 ] = solverState [ 14
] ; fullState [ 15 ] = solverState [ 15 ] ; fullState [ 16 ] = solverState [
16 ] ; fullState [ 17 ] = solverState [ 17 ] ; fullState [ 18 ] = solverState
[ 18 ] ; fullState [ 19 ] = solverState [ 19 ] ; fullState [ 20 ] =
solverState [ 20 ] ; fullState [ 21 ] = solverState [ 21 ] ; fullState [ 22 ]
= solverState [ 22 ] ; fullState [ 23 ] = solverState [ 23 ] ; fullState [ 24
] = solverState [ 24 ] ; fullState [ 25 ] = solverState [ 25 ] ; fullState [
26 ] = solverState [ 26 ] ; fullState [ 27 ] = solverState [ 27 ] ; fullState
[ 28 ] = solverState [ 28 ] ; fullState [ 29 ] = solverState [ 29 ] ;
fullState [ 30 ] = solverState [ 30 ] ; fullState [ 31 ] = solverState [ 31 ]
; } void wheel_leg_d9e82f16_1_extractSolverStateVector ( const void * mech ,
const double * fullState , double * solverState ) { ( void ) mech ;
solverState [ 0 ] = fullState [ 0 ] ; solverState [ 1 ] = fullState [ 1 ] ;
solverState [ 2 ] = fullState [ 2 ] ; solverState [ 3 ] = fullState [ 3 ] ;
solverState [ 4 ] = fullState [ 4 ] ; solverState [ 5 ] = fullState [ 5 ] ;
solverState [ 6 ] = fullState [ 6 ] ; solverState [ 7 ] = fullState [ 7 ] ;
solverState [ 8 ] = fullState [ 8 ] ; solverState [ 9 ] = fullState [ 9 ] ;
solverState [ 10 ] = fullState [ 10 ] ; solverState [ 11 ] = fullState [ 11 ]
; solverState [ 12 ] = fullState [ 12 ] ; solverState [ 13 ] = fullState [ 13
] ; solverState [ 14 ] = fullState [ 14 ] ; solverState [ 15 ] = fullState [
15 ] ; solverState [ 16 ] = fullState [ 16 ] ; solverState [ 17 ] = fullState
[ 17 ] ; solverState [ 18 ] = fullState [ 18 ] ; solverState [ 19 ] =
fullState [ 19 ] ; solverState [ 20 ] = fullState [ 20 ] ; solverState [ 21 ]
= fullState [ 21 ] ; solverState [ 22 ] = fullState [ 22 ] ; solverState [ 23
] = fullState [ 23 ] ; solverState [ 24 ] = fullState [ 24 ] ; solverState [
25 ] = fullState [ 25 ] ; solverState [ 26 ] = fullState [ 26 ] ; solverState
[ 27 ] = fullState [ 27 ] ; solverState [ 28 ] = fullState [ 28 ] ;
solverState [ 29 ] = fullState [ 29 ] ; solverState [ 30 ] = fullState [ 30 ]
; solverState [ 31 ] = fullState [ 31 ] ; } boolean_T
wheel_leg_d9e82f16_1_isPositionViolation ( const void * mech , const
RuntimeDerivedValuesBundle * rtdv , const int * eqnEnableFlags , const double
* state , const int * modeVector ) { const double * rtdvd = rtdv -> mDoubles
. mValues ; const int * rtdvi = rtdv -> mInts . mValues ; int ii [ 1 ] ;
double xx [ 62 ] ; ( void ) mech ; ( void ) rtdvd ; ( void ) rtdvi ; ( void )
eqnEnableFlags ; ( void ) modeVector ; xx [ 0 ] = 0.2 ; xx [ 1 ] = 2.0 ; xx [
2 ] = 0.125 ; xx [ 3 ] = 0.5 ; xx [ 4 ] = xx [ 3 ] * state [ 10 ] ; xx [ 5 ]
= sin ( xx [ 4 ] ) ; xx [ 6 ] = xx [ 2 ] * xx [ 5 ] ; xx [ 7 ] = xx [ 0 ] -
xx [ 1 ] * xx [ 6 ] * xx [ 5 ] ; xx [ 8 ] = 0.7071067811865476 ; xx [ 9 ] =
xx [ 3 ] * state [ 8 ] ; xx [ 10 ] = xx [ 8 ] * sin ( xx [ 9 ] ) ; xx [ 11 ]
= xx [ 7 ] * xx [ 10 ] ; xx [ 12 ] = xx [ 8 ] * cos ( xx [ 9 ] ) ; xx [ 9 ] =
- xx [ 10 ] ; xx [ 13 ] = xx [ 9 ] ; xx [ 14 ] = - xx [ 12 ] ; xx [ 15 ] = xx
[ 9 ] ; xx [ 9 ] = cos ( xx [ 4 ] ) ; xx [ 4 ] = xx [ 1 ] * xx [ 9 ] * xx [ 6
] ; xx [ 6 ] = xx [ 10 ] * xx [ 4 ] ; xx [ 16 ] = xx [ 6 ] - xx [ 7 ] * xx [
12 ] ; xx [ 17 ] = xx [ 6 ] ; xx [ 18 ] = - xx [ 11 ] ; xx [ 19 ] = - xx [ 16
] ; pm_math_Vector3_cross_ra ( xx + 13 , xx + 17 , xx + 20 ) ; xx [ 6 ] =
0.075 ; xx [ 7 ] = xx [ 6 ] * xx [ 10 ] ; xx [ 13 ] = xx [ 6 ] * xx [ 12 ] ;
xx [ 14 ] = xx [ 12 ] * xx [ 5 ] ; xx [ 15 ] = xx [ 9 ] * xx [ 10 ] ; xx [ 17
] = ( xx [ 14 ] + xx [ 15 ] ) * xx [ 2 ] ; xx [ 18 ] = xx [ 9 ] * xx [ 12 ] -
xx [ 10 ] * xx [ 5 ] ; xx [ 5 ] = xx [ 15 ] + xx [ 14 ] ; xx [ 9 ] = xx [ 2 ]
* xx [ 18 ] ; xx [ 14 ] = xx [ 3 ] * state [ 16 ] ; xx [ 15 ] = sin ( xx [ 14
] ) ; xx [ 19 ] = xx [ 2 ] * xx [ 15 ] ; xx [ 23 ] = xx [ 0 ] - xx [ 1 ] * xx
[ 19 ] * xx [ 15 ] ; xx [ 24 ] = xx [ 3 ] * state [ 14 ] ; xx [ 25 ] = xx [ 8
] * sin ( xx [ 24 ] ) ; xx [ 26 ] = xx [ 23 ] * xx [ 25 ] ; xx [ 27 ] = xx [
8 ] * cos ( xx [ 24 ] ) ; xx [ 24 ] = - xx [ 25 ] ; xx [ 28 ] = xx [ 24 ] ;
xx [ 29 ] = - xx [ 27 ] ; xx [ 30 ] = xx [ 24 ] ; xx [ 24 ] = cos ( xx [ 14 ]
) ; xx [ 14 ] = xx [ 1 ] * xx [ 24 ] * xx [ 19 ] ; xx [ 19 ] = xx [ 25 ] * xx
[ 14 ] ; xx [ 31 ] = xx [ 19 ] - xx [ 23 ] * xx [ 27 ] ; xx [ 32 ] = xx [ 19
] ; xx [ 33 ] = - xx [ 26 ] ; xx [ 34 ] = - xx [ 31 ] ;
pm_math_Vector3_cross_ra ( xx + 28 , xx + 32 , xx + 35 ) ; xx [ 19 ] = xx [ 6
] * xx [ 25 ] ; xx [ 23 ] = xx [ 6 ] * xx [ 27 ] ; xx [ 28 ] = xx [ 27 ] * xx
[ 15 ] ; xx [ 29 ] = xx [ 24 ] * xx [ 25 ] ; xx [ 30 ] = ( xx [ 28 ] + xx [
29 ] ) * xx [ 2 ] ; xx [ 32 ] = xx [ 24 ] * xx [ 27 ] - xx [ 25 ] * xx [ 15 ]
; xx [ 15 ] = xx [ 29 ] + xx [ 28 ] ; xx [ 24 ] = xx [ 2 ] * xx [ 32 ] ; xx [
28 ] = 0.108 ; xx [ 20 ] = xx [ 3 ] * state [ 20 ] ; xx [ 29 ] = sin ( xx [
20 ] ) ; xx [ 33 ] = xx [ 2 ] * xx [ 29 ] ; xx [ 34 ] = xx [ 0 ] - xx [ 1 ] *
xx [ 33 ] * xx [ 29 ] ; xx [ 35 ] = xx [ 3 ] * state [ 18 ] ; xx [ 38 ] = xx
[ 8 ] * sin ( xx [ 35 ] ) ; xx [ 39 ] = xx [ 34 ] * xx [ 38 ] ; xx [ 40 ] =
xx [ 8 ] * cos ( xx [ 35 ] ) ; xx [ 41 ] = xx [ 38 ] ; xx [ 42 ] = xx [ 40 ]
; xx [ 43 ] = xx [ 38 ] ; xx [ 35 ] = cos ( xx [ 20 ] ) ; xx [ 20 ] = xx [ 1
] * xx [ 35 ] * xx [ 33 ] ; xx [ 33 ] = xx [ 38 ] * xx [ 20 ] ; xx [ 44 ] =
xx [ 34 ] * xx [ 40 ] - xx [ 33 ] ; xx [ 45 ] = - xx [ 33 ] ; xx [ 46 ] = xx
[ 39 ] ; xx [ 47 ] = - xx [ 44 ] ; pm_math_Vector3_cross_ra ( xx + 41 , xx +
45 , xx + 48 ) ; xx [ 33 ] = xx [ 6 ] * xx [ 38 ] ; xx [ 34 ] = xx [ 6 ] * xx
[ 40 ] ; xx [ 41 ] = xx [ 40 ] * xx [ 29 ] ; xx [ 42 ] = xx [ 35 ] * xx [ 38
] ; xx [ 43 ] = ( xx [ 41 ] + xx [ 42 ] ) * xx [ 2 ] ; xx [ 45 ] = xx [ 38 ]
* xx [ 29 ] - xx [ 35 ] * xx [ 40 ] ; xx [ 29 ] = xx [ 42 ] + xx [ 41 ] ; xx
[ 35 ] = xx [ 2 ] * xx [ 45 ] ; xx [ 41 ] = xx [ 3 ] * state [ 26 ] ; xx [ 42
] = sin ( xx [ 41 ] ) ; xx [ 46 ] = xx [ 2 ] * xx [ 42 ] ; xx [ 47 ] = xx [ 0
] - xx [ 1 ] * xx [ 46 ] * xx [ 42 ] ; xx [ 0 ] = xx [ 3 ] * state [ 24 ] ;
xx [ 3 ] = xx [ 8 ] * sin ( xx [ 0 ] ) ; xx [ 51 ] = xx [ 47 ] * xx [ 3 ] ;
xx [ 52 ] = xx [ 8 ] * cos ( xx [ 0 ] ) ; xx [ 53 ] = xx [ 3 ] ; xx [ 54 ] =
xx [ 52 ] ; xx [ 55 ] = xx [ 3 ] ; xx [ 0 ] = cos ( xx [ 41 ] ) ; xx [ 8 ] =
xx [ 1 ] * xx [ 0 ] * xx [ 46 ] ; xx [ 41 ] = xx [ 3 ] * xx [ 8 ] ; xx [ 46 ]
= xx [ 47 ] * xx [ 52 ] - xx [ 41 ] ; xx [ 56 ] = - xx [ 41 ] ; xx [ 57 ] =
xx [ 51 ] ; xx [ 58 ] = - xx [ 46 ] ; pm_math_Vector3_cross_ra ( xx + 53 , xx
+ 56 , xx + 59 ) ; xx [ 41 ] = xx [ 6 ] * xx [ 3 ] ; xx [ 47 ] = xx [ 6 ] *
xx [ 52 ] ; xx [ 6 ] = xx [ 52 ] * xx [ 42 ] ; xx [ 53 ] = xx [ 0 ] * xx [ 3
] ; xx [ 54 ] = ( xx [ 6 ] + xx [ 53 ] ) * xx [ 2 ] ; xx [ 55 ] = xx [ 3 ] *
xx [ 42 ] - xx [ 0 ] * xx [ 52 ] ; xx [ 0 ] = xx [ 53 ] + xx [ 6 ] ; xx [ 6 ]
= xx [ 2 ] * xx [ 55 ] ; xx [ 56 ] = fabs ( ( xx [ 11 ] * xx [ 12 ] + xx [ 21
] ) * xx [ 1 ] + xx [ 4 ] + ( xx [ 7 ] * xx [ 12 ] + xx [ 13 ] * xx [ 10 ] )
* xx [ 1 ] + ( xx [ 17 ] * xx [ 18 ] + xx [ 5 ] * xx [ 9 ] ) * xx [ 1 ] - ( (
xx [ 26 ] * xx [ 27 ] + xx [ 36 ] ) * xx [ 1 ] + xx [ 14 ] + ( xx [ 19 ] * xx
[ 27 ] + xx [ 23 ] * xx [ 25 ] ) * xx [ 1 ] + ( xx [ 30 ] * xx [ 32 ] + xx [
15 ] * xx [ 24 ] ) * xx [ 1 ] ) + xx [ 28 ] ) ; xx [ 57 ] = fabs ( xx [ 1 ] *
( xx [ 5 ] * xx [ 17 ] - xx [ 9 ] * xx [ 18 ] ) + xx [ 1 ] * ( xx [ 22 ] + xx
[ 16 ] * xx [ 12 ] ) - xx [ 1 ] * ( xx [ 13 ] * xx [ 12 ] - xx [ 7 ] * xx [
10 ] ) - ( xx [ 1 ] * ( xx [ 15 ] * xx [ 30 ] - xx [ 24 ] * xx [ 32 ] ) + xx
[ 1 ] * ( xx [ 37 ] + xx [ 31 ] * xx [ 27 ] ) - xx [ 1 ] * ( xx [ 23 ] * xx [
27 ] - xx [ 19 ] * xx [ 25 ] ) ) ) ; xx [ 58 ] = fabs ( ( xx [ 39 ] * xx [ 40
] + xx [ 49 ] ) * xx [ 1 ] + xx [ 20 ] + ( xx [ 33 ] * xx [ 40 ] + xx [ 34 ]
* xx [ 38 ] ) * xx [ 1 ] - ( xx [ 43 ] * xx [ 45 ] + xx [ 29 ] * xx [ 35 ] )
* xx [ 1 ] - ( ( xx [ 51 ] * xx [ 52 ] + xx [ 60 ] ) * xx [ 1 ] + xx [ 8 ] +
( xx [ 41 ] * xx [ 52 ] + xx [ 47 ] * xx [ 3 ] ) * xx [ 1 ] - ( xx [ 54 ] *
xx [ 55 ] + xx [ 0 ] * xx [ 6 ] ) * xx [ 1 ] ) + xx [ 28 ] ) ; xx [ 59 ] =
fabs ( xx [ 1 ] * ( xx [ 29 ] * xx [ 43 ] - xx [ 35 ] * xx [ 45 ] ) + xx [ 1
] * ( xx [ 50 ] - xx [ 44 ] * xx [ 40 ] ) - xx [ 1 ] * ( xx [ 34 ] * xx [ 40
] - xx [ 33 ] * xx [ 38 ] ) - ( xx [ 1 ] * ( xx [ 0 ] * xx [ 54 ] - xx [ 6 ]
* xx [ 55 ] ) + xx [ 1 ] * ( xx [ 61 ] - xx [ 46 ] * xx [ 52 ] ) - xx [ 1 ] *
( xx [ 47 ] * xx [ 52 ] - xx [ 41 ] * xx [ 3 ] ) ) ) ; ii [ 0 ] = 56 ; { int
ll ; for ( ll = 57 ; ll < 60 ; ++ ll ) if ( xx [ ll ] > xx [ ii [ 0 ] ] ) ii
[ 0 ] = ll ; } ii [ 0 ] -= 56 ; xx [ 0 ] = xx [ 56 + ( ii [ 0 ] ) ] ; return
xx [ 0 ] > 1.0e-9 ; } boolean_T wheel_leg_d9e82f16_1_isVelocityViolation (
const void * mech , const RuntimeDerivedValuesBundle * rtdv , const int *
eqnEnableFlags , const double * state , const int * modeVector ) { const
double * rtdvd = rtdv -> mDoubles . mValues ; const int * rtdvi = rtdv ->
mInts . mValues ; int ii [ 1 ] ; double xx [ 72 ] ; ( void ) mech ; ( void )
rtdvd ; ( void ) rtdvi ; ( void ) eqnEnableFlags ; ( void ) modeVector ; xx [
0 ] = 0.125 ; xx [ 1 ] = ( state [ 9 ] + state [ 11 ] ) * xx [ 0 ] ; xx [ 2 ]
= 0.7071067811865476 ; xx [ 3 ] = 0.5 ; xx [ 4 ] = xx [ 3 ] * state [ 8 ] ;
xx [ 5 ] = xx [ 2 ] * cos ( xx [ 4 ] ) ; xx [ 6 ] = xx [ 3 ] * state [ 10 ] ;
xx [ 7 ] = sin ( xx [ 6 ] ) ; xx [ 8 ] = xx [ 5 ] * xx [ 7 ] ; xx [ 9 ] = cos
( xx [ 6 ] ) ; xx [ 6 ] = xx [ 2 ] * sin ( xx [ 4 ] ) ; xx [ 4 ] = xx [ 9 ] *
xx [ 6 ] ; xx [ 10 ] = xx [ 8 ] + xx [ 4 ] ; xx [ 11 ] = xx [ 10 ] * xx [ 1 ]
; xx [ 12 ] = xx [ 4 ] + xx [ 8 ] ; xx [ 4 ] = xx [ 12 ] * xx [ 1 ] ; xx [ 8
] = 2.0 ; xx [ 13 ] = 0.075 ; xx [ 14 ] = xx [ 13 ] * xx [ 6 ] ; xx [ 15 ] =
xx [ 14 ] * xx [ 6 ] ; xx [ 16 ] = xx [ 0 ] * xx [ 7 ] ; xx [ 17 ] = xx [ 8 ]
* xx [ 16 ] * xx [ 7 ] ; xx [ 18 ] = 0.2 ; xx [ 19 ] = ( xx [ 0 ] - xx [ 17 ]
) * state [ 11 ] + ( xx [ 18 ] - xx [ 17 ] ) * state [ 9 ] ; xx [ 17 ] = - xx
[ 6 ] ; xx [ 20 ] = xx [ 17 ] ; xx [ 21 ] = - xx [ 5 ] ; xx [ 22 ] = xx [ 17
] ; xx [ 17 ] = xx [ 19 ] * xx [ 6 ] ; xx [ 23 ] = xx [ 9 ] * xx [ 16 ] ; xx
[ 16 ] = xx [ 8 ] * xx [ 23 ] * state [ 9 ] + xx [ 8 ] * xx [ 23 ] * state [
11 ] ; xx [ 23 ] = xx [ 6 ] * xx [ 16 ] ; xx [ 24 ] = xx [ 17 ] + xx [ 5 ] *
xx [ 16 ] ; xx [ 25 ] = xx [ 17 ] ; xx [ 26 ] = xx [ 23 ] ; xx [ 27 ] = - xx
[ 24 ] ; pm_math_Vector3_cross_ra ( xx + 20 , xx + 25 , xx + 28 ) ; xx [ 16 ]
= ( state [ 15 ] + state [ 17 ] ) * xx [ 0 ] ; xx [ 17 ] = xx [ 3 ] * state [
14 ] ; xx [ 20 ] = xx [ 2 ] * cos ( xx [ 17 ] ) ; xx [ 21 ] = xx [ 3 ] *
state [ 16 ] ; xx [ 22 ] = sin ( xx [ 21 ] ) ; xx [ 25 ] = xx [ 20 ] * xx [
22 ] ; xx [ 26 ] = cos ( xx [ 21 ] ) ; xx [ 21 ] = xx [ 2 ] * sin ( xx [ 17 ]
) ; xx [ 17 ] = xx [ 26 ] * xx [ 21 ] ; xx [ 27 ] = xx [ 25 ] + xx [ 17 ] ;
xx [ 31 ] = xx [ 27 ] * xx [ 16 ] ; xx [ 32 ] = xx [ 17 ] + xx [ 25 ] ; xx [
17 ] = xx [ 32 ] * xx [ 16 ] ; xx [ 25 ] = xx [ 13 ] * xx [ 21 ] ; xx [ 33 ]
= xx [ 25 ] * xx [ 21 ] ; xx [ 34 ] = xx [ 0 ] * xx [ 22 ] ; xx [ 35 ] = xx [
8 ] * xx [ 34 ] * xx [ 22 ] ; xx [ 36 ] = ( xx [ 0 ] - xx [ 35 ] ) * state [
17 ] + ( xx [ 18 ] - xx [ 35 ] ) * state [ 15 ] ; xx [ 35 ] = - xx [ 21 ] ;
xx [ 37 ] = xx [ 35 ] ; xx [ 38 ] = - xx [ 20 ] ; xx [ 39 ] = xx [ 35 ] ; xx
[ 35 ] = xx [ 36 ] * xx [ 21 ] ; xx [ 40 ] = xx [ 26 ] * xx [ 34 ] ; xx [ 34
] = xx [ 8 ] * xx [ 40 ] * state [ 15 ] + xx [ 8 ] * xx [ 40 ] * state [ 17 ]
; xx [ 40 ] = xx [ 21 ] * xx [ 34 ] ; xx [ 41 ] = xx [ 35 ] + xx [ 20 ] * xx
[ 34 ] ; xx [ 42 ] = xx [ 35 ] ; xx [ 43 ] = xx [ 40 ] ; xx [ 44 ] = - xx [
41 ] ; pm_math_Vector3_cross_ra ( xx + 37 , xx + 42 , xx + 45 ) ; xx [ 34 ] =
xx [ 14 ] * xx [ 5 ] ; xx [ 14 ] = xx [ 9 ] * xx [ 5 ] - xx [ 6 ] * xx [ 7 ]
; xx [ 6 ] = xx [ 25 ] * xx [ 20 ] ; xx [ 7 ] = xx [ 26 ] * xx [ 20 ] - xx [
21 ] * xx [ 22 ] ; xx [ 9 ] = ( state [ 19 ] + state [ 21 ] ) * xx [ 0 ] ; xx
[ 21 ] = xx [ 3 ] * state [ 18 ] ; xx [ 22 ] = xx [ 2 ] * cos ( xx [ 21 ] ) ;
xx [ 25 ] = xx [ 3 ] * state [ 20 ] ; xx [ 26 ] = sin ( xx [ 25 ] ) ; xx [ 28
] = xx [ 22 ] * xx [ 26 ] ; xx [ 35 ] = cos ( xx [ 25 ] ) ; xx [ 25 ] = xx [
2 ] * sin ( xx [ 21 ] ) ; xx [ 21 ] = xx [ 35 ] * xx [ 25 ] ; xx [ 37 ] = xx
[ 28 ] + xx [ 21 ] ; xx [ 38 ] = xx [ 37 ] * xx [ 9 ] ; xx [ 39 ] = xx [ 21 ]
+ xx [ 28 ] ; xx [ 21 ] = xx [ 39 ] * xx [ 9 ] ; xx [ 28 ] = xx [ 13 ] * xx [
25 ] ; xx [ 42 ] = xx [ 28 ] * xx [ 25 ] ; xx [ 43 ] = xx [ 0 ] * xx [ 26 ] ;
xx [ 44 ] = xx [ 8 ] * xx [ 43 ] * xx [ 26 ] ; xx [ 45 ] = ( xx [ 0 ] - xx [
44 ] ) * state [ 21 ] + ( xx [ 18 ] - xx [ 44 ] ) * state [ 19 ] ; xx [ 48 ]
= xx [ 25 ] ; xx [ 49 ] = xx [ 22 ] ; xx [ 50 ] = xx [ 25 ] ; xx [ 44 ] = xx
[ 45 ] * xx [ 25 ] ; xx [ 51 ] = xx [ 35 ] * xx [ 43 ] ; xx [ 43 ] = xx [ 8 ]
* xx [ 51 ] * state [ 19 ] + xx [ 8 ] * xx [ 51 ] * state [ 21 ] ; xx [ 51 ]
= xx [ 25 ] * xx [ 43 ] ; xx [ 52 ] = xx [ 44 ] + xx [ 22 ] * xx [ 43 ] ; xx
[ 53 ] = - xx [ 44 ] ; xx [ 54 ] = - xx [ 51 ] ; xx [ 55 ] = xx [ 52 ] ;
pm_math_Vector3_cross_ra ( xx + 48 , xx + 53 , xx + 56 ) ; xx [ 43 ] = (
state [ 25 ] + state [ 27 ] ) * xx [ 0 ] ; xx [ 44 ] = xx [ 3 ] * state [ 24
] ; xx [ 48 ] = xx [ 2 ] * cos ( xx [ 44 ] ) ; xx [ 49 ] = xx [ 3 ] * state [
26 ] ; xx [ 3 ] = sin ( xx [ 49 ] ) ; xx [ 50 ] = xx [ 48 ] * xx [ 3 ] ; xx [
53 ] = cos ( xx [ 49 ] ) ; xx [ 49 ] = xx [ 2 ] * sin ( xx [ 44 ] ) ; xx [ 2
] = xx [ 53 ] * xx [ 49 ] ; xx [ 44 ] = xx [ 50 ] + xx [ 2 ] ; xx [ 54 ] = xx
[ 44 ] * xx [ 43 ] ; xx [ 55 ] = xx [ 2 ] + xx [ 50 ] ; xx [ 2 ] = xx [ 55 ]
* xx [ 43 ] ; xx [ 50 ] = xx [ 13 ] * xx [ 49 ] ; xx [ 59 ] = xx [ 50 ] * xx
[ 49 ] ; xx [ 60 ] = xx [ 0 ] * xx [ 3 ] ; xx [ 61 ] = xx [ 8 ] * xx [ 60 ] *
xx [ 3 ] ; xx [ 62 ] = ( xx [ 0 ] - xx [ 61 ] ) * state [ 27 ] + ( xx [ 18 ]
- xx [ 61 ] ) * state [ 25 ] ; xx [ 63 ] = xx [ 49 ] ; xx [ 64 ] = xx [ 48 ]
; xx [ 65 ] = xx [ 49 ] ; xx [ 0 ] = xx [ 62 ] * xx [ 49 ] ; xx [ 18 ] = xx [
53 ] * xx [ 60 ] ; xx [ 60 ] = xx [ 8 ] * xx [ 18 ] * state [ 25 ] + xx [ 8 ]
* xx [ 18 ] * state [ 27 ] ; xx [ 18 ] = xx [ 49 ] * xx [ 60 ] ; xx [ 61 ] =
xx [ 0 ] + xx [ 48 ] * xx [ 60 ] ; xx [ 66 ] = - xx [ 0 ] ; xx [ 67 ] = - xx
[ 18 ] ; xx [ 68 ] = xx [ 61 ] ; pm_math_Vector3_cross_ra ( xx + 63 , xx + 66
, xx + 69 ) ; xx [ 0 ] = xx [ 28 ] * xx [ 22 ] ; xx [ 28 ] = xx [ 25 ] * xx [
26 ] - xx [ 35 ] * xx [ 22 ] ; xx [ 25 ] = xx [ 50 ] * xx [ 48 ] ; xx [ 26 ]
= xx [ 49 ] * xx [ 3 ] - xx [ 53 ] * xx [ 48 ] ; xx [ 63 ] = fabs ( xx [ 1 ]
- ( xx [ 10 ] * xx [ 11 ] + xx [ 12 ] * xx [ 4 ] ) * xx [ 8 ] + ( xx [ 13 ] -
( xx [ 15 ] + xx [ 15 ] ) * xx [ 8 ] ) * state [ 9 ] + xx [ 19 ] + ( xx [ 29
] - xx [ 5 ] * xx [ 23 ] ) * xx [ 8 ] - ( xx [ 16 ] - ( xx [ 27 ] * xx [ 31 ]
+ xx [ 32 ] * xx [ 17 ] ) * xx [ 8 ] + ( xx [ 13 ] - ( xx [ 33 ] + xx [ 33 ]
) * xx [ 8 ] ) * state [ 15 ] + xx [ 36 ] + ( xx [ 46 ] - xx [ 20 ] * xx [ 40
] ) * xx [ 8 ] ) ) ; xx [ 64 ] = fabs ( xx [ 8 ] * ( xx [ 34 ] + xx [ 34 ] )
* state [ 9 ] + ( xx [ 5 ] * xx [ 24 ] + xx [ 30 ] ) * xx [ 8 ] + ( xx [ 4 ]
* xx [ 14 ] + xx [ 11 ] * xx [ 14 ] ) * xx [ 8 ] - ( xx [ 8 ] * ( xx [ 6 ] +
xx [ 6 ] ) * state [ 15 ] + ( xx [ 20 ] * xx [ 41 ] + xx [ 47 ] ) * xx [ 8 ]
+ ( xx [ 17 ] * xx [ 7 ] + xx [ 31 ] * xx [ 7 ] ) * xx [ 8 ] ) ) ; xx [ 65 ]
= fabs ( xx [ 9 ] - ( xx [ 37 ] * xx [ 38 ] + xx [ 39 ] * xx [ 21 ] ) * xx [
8 ] + ( xx [ 13 ] - ( xx [ 42 ] + xx [ 42 ] ) * xx [ 8 ] ) * state [ 19 ] +
xx [ 45 ] + ( xx [ 57 ] - xx [ 22 ] * xx [ 51 ] ) * xx [ 8 ] - ( xx [ 43 ] -
( xx [ 44 ] * xx [ 54 ] + xx [ 55 ] * xx [ 2 ] ) * xx [ 8 ] + ( xx [ 13 ] - (
xx [ 59 ] + xx [ 59 ] ) * xx [ 8 ] ) * state [ 25 ] + xx [ 62 ] + ( xx [ 70 ]
- xx [ 48 ] * xx [ 18 ] ) * xx [ 8 ] ) ) ; xx [ 66 ] = fabs ( xx [ 8 ] * ( xx
[ 0 ] + xx [ 0 ] ) * state [ 19 ] + ( xx [ 22 ] * xx [ 52 ] + xx [ 58 ] ) *
xx [ 8 ] - ( xx [ 21 ] * xx [ 28 ] + xx [ 38 ] * xx [ 28 ] ) * xx [ 8 ] - (
xx [ 8 ] * ( xx [ 25 ] + xx [ 25 ] ) * state [ 25 ] + ( xx [ 48 ] * xx [ 61 ]
+ xx [ 71 ] ) * xx [ 8 ] - ( xx [ 2 ] * xx [ 26 ] + xx [ 54 ] * xx [ 26 ] ) *
xx [ 8 ] ) ) ; ii [ 0 ] = 63 ; { int ll ; for ( ll = 64 ; ll < 67 ; ++ ll )
if ( xx [ ll ] > xx [ ii [ 0 ] ] ) ii [ 0 ] = ll ; } ii [ 0 ] -= 63 ; xx [ 0
] = xx [ 63 + ( ii [ 0 ] ) ] ; return xx [ 0 ] > 1.0e-9 ; } PmfMessageId
wheel_leg_d9e82f16_1_projectStateSim ( const void * mech , const
RuntimeDerivedValuesBundle * rtdv , const int * eqnEnableFlags , const int *
modeVector , double * state , void * neDiagMgr0 ) { const double * rtdvd =
rtdv -> mDoubles . mValues ; const int * rtdvi = rtdv -> mInts . mValues ;
NeuDiagnosticManager * neDiagMgr = ( NeuDiagnosticManager * ) neDiagMgr0 ;
int ii [ 4 ] ; double xx [ 275 ] ; ( void ) mech ; ( void ) rtdvd ; ( void )
rtdvi ; ( void ) eqnEnableFlags ; ( void ) modeVector ; ( void ) neDiagMgr ;
xx [ 0 ] = 0.0 ; xx [ 1 ] = 0.2 ; xx [ 2 ] = 2.0 ; xx [ 3 ] = 0.125 ; xx [ 4
] = 0.5 ; xx [ 5 ] = xx [ 4 ] * state [ 10 ] ; xx [ 6 ] = sin ( xx [ 5 ] ) ;
xx [ 7 ] = xx [ 3 ] * xx [ 6 ] ; xx [ 8 ] = xx [ 2 ] * xx [ 7 ] * xx [ 6 ] ;
xx [ 9 ] = xx [ 1 ] - xx [ 8 ] ; xx [ 10 ] = 0.7071067811865476 ; xx [ 11 ] =
xx [ 4 ] * state [ 8 ] ; xx [ 12 ] = xx [ 10 ] * sin ( xx [ 11 ] ) ; xx [ 13
] = - xx [ 12 ] ; xx [ 14 ] = xx [ 10 ] * cos ( xx [ 11 ] ) ; xx [ 15 ] = xx
[ 13 ] ; xx [ 16 ] = - xx [ 14 ] ; xx [ 17 ] = xx [ 13 ] ; xx [ 11 ] = xx [ 9
] * xx [ 12 ] ; xx [ 13 ] = cos ( xx [ 5 ] ) ; xx [ 5 ] = xx [ 2 ] * xx [ 13
] * xx [ 7 ] ; xx [ 7 ] = xx [ 12 ] * xx [ 5 ] ; xx [ 18 ] = xx [ 14 ] * xx [
5 ] ; xx [ 19 ] = xx [ 11 ] + xx [ 18 ] ; xx [ 20 ] = xx [ 11 ] ; xx [ 21 ] =
xx [ 7 ] ; xx [ 22 ] = - xx [ 19 ] ; pm_math_Vector3_cross_ra ( xx + 15 , xx
+ 20 , xx + 23 ) ; xx [ 20 ] = xx [ 14 ] * xx [ 7 ] ; xx [ 21 ] = 0.075 ; xx
[ 22 ] = xx [ 21 ] * xx [ 12 ] ; xx [ 26 ] = xx [ 22 ] * xx [ 12 ] ; xx [ 27
] = xx [ 14 ] * xx [ 6 ] ; xx [ 28 ] = xx [ 13 ] * xx [ 12 ] ; xx [ 29 ] = xx
[ 27 ] + xx [ 28 ] ; xx [ 30 ] = xx [ 29 ] * xx [ 3 ] ; xx [ 31 ] = xx [ 28 ]
+ xx [ 27 ] ; xx [ 27 ] = xx [ 31 ] * xx [ 3 ] ; xx [ 28 ] = ( xx [ 29 ] * xx
[ 30 ] + xx [ 31 ] * xx [ 27 ] ) * xx [ 2 ] ; xx [ 29 ] = xx [ 3 ] - xx [ 8 ]
; xx [ 8 ] = xx [ 12 ] * xx [ 29 ] ; xx [ 32 ] = xx [ 8 ] + xx [ 18 ] ; xx [
33 ] = xx [ 8 ] ; xx [ 34 ] = xx [ 7 ] ; xx [ 35 ] = - xx [ 32 ] ;
pm_math_Vector3_cross_ra ( xx + 15 , xx + 33 , xx + 36 ) ; xx [ 8 ] = xx [ 4
] * state [ 16 ] ; xx [ 18 ] = sin ( xx [ 8 ] ) ; xx [ 33 ] = xx [ 3 ] * xx [
18 ] ; xx [ 34 ] = xx [ 2 ] * xx [ 33 ] * xx [ 18 ] ; xx [ 35 ] = xx [ 1 ] -
xx [ 34 ] ; xx [ 39 ] = xx [ 4 ] * state [ 14 ] ; xx [ 40 ] = xx [ 10 ] * sin
( xx [ 39 ] ) ; xx [ 41 ] = - xx [ 40 ] ; xx [ 42 ] = xx [ 10 ] * cos ( xx [
39 ] ) ; xx [ 43 ] = xx [ 41 ] ; xx [ 44 ] = - xx [ 42 ] ; xx [ 45 ] = xx [
41 ] ; xx [ 39 ] = xx [ 35 ] * xx [ 40 ] ; xx [ 41 ] = cos ( xx [ 8 ] ) ; xx
[ 8 ] = xx [ 2 ] * xx [ 41 ] * xx [ 33 ] ; xx [ 33 ] = xx [ 40 ] * xx [ 8 ] ;
xx [ 46 ] = xx [ 42 ] * xx [ 8 ] ; xx [ 47 ] = xx [ 39 ] + xx [ 46 ] ; xx [
48 ] = xx [ 39 ] ; xx [ 49 ] = xx [ 33 ] ; xx [ 50 ] = - xx [ 47 ] ;
pm_math_Vector3_cross_ra ( xx + 43 , xx + 48 , xx + 51 ) ; xx [ 48 ] = xx [
42 ] * xx [ 33 ] ; xx [ 49 ] = xx [ 21 ] * xx [ 40 ] ; xx [ 50 ] = xx [ 49 ]
* xx [ 40 ] ; xx [ 54 ] = xx [ 42 ] * xx [ 18 ] ; xx [ 55 ] = xx [ 41 ] * xx
[ 40 ] ; xx [ 56 ] = xx [ 54 ] + xx [ 55 ] ; xx [ 57 ] = xx [ 56 ] * xx [ 3 ]
; xx [ 58 ] = xx [ 55 ] + xx [ 54 ] ; xx [ 54 ] = xx [ 58 ] * xx [ 3 ] ; xx [
55 ] = ( xx [ 56 ] * xx [ 57 ] + xx [ 58 ] * xx [ 54 ] ) * xx [ 2 ] ; xx [ 56
] = xx [ 3 ] - xx [ 34 ] ; xx [ 34 ] = xx [ 40 ] * xx [ 56 ] ; xx [ 59 ] = xx
[ 34 ] + xx [ 46 ] ; xx [ 60 ] = xx [ 34 ] ; xx [ 61 ] = xx [ 33 ] ; xx [ 62
] = - xx [ 59 ] ; pm_math_Vector3_cross_ra ( xx + 43 , xx + 60 , xx + 63 ) ;
xx [ 23 ] = xx [ 22 ] * xx [ 14 ] ; xx [ 22 ] = xx [ 13 ] * xx [ 14 ] - xx [
12 ] * xx [ 6 ] ; xx [ 6 ] = xx [ 30 ] * xx [ 22 ] ; xx [ 13 ] = ( xx [ 27 ]
* xx [ 22 ] + xx [ 6 ] ) * xx [ 2 ] ; xx [ 27 ] = xx [ 41 ] * xx [ 42 ] - xx
[ 40 ] * xx [ 18 ] ; xx [ 18 ] = xx [ 57 ] * xx [ 27 ] ; xx [ 34 ] = ( xx [
54 ] * xx [ 27 ] + xx [ 18 ] ) * xx [ 2 ] ; xx [ 36 ] = xx [ 49 ] * xx [ 42 ]
; xx [ 41 ] = xx [ 4 ] * state [ 20 ] ; xx [ 46 ] = sin ( xx [ 41 ] ) ; xx [
49 ] = xx [ 3 ] * xx [ 46 ] ; xx [ 51 ] = xx [ 2 ] * xx [ 49 ] * xx [ 46 ] ;
xx [ 54 ] = xx [ 1 ] - xx [ 51 ] ; xx [ 60 ] = xx [ 4 ] * state [ 18 ] ; xx [
61 ] = xx [ 10 ] * sin ( xx [ 60 ] ) ; xx [ 62 ] = xx [ 10 ] * cos ( xx [ 60
] ) ; xx [ 66 ] = xx [ 61 ] ; xx [ 67 ] = xx [ 62 ] ; xx [ 68 ] = xx [ 61 ] ;
xx [ 60 ] = xx [ 54 ] * xx [ 61 ] ; xx [ 63 ] = cos ( xx [ 41 ] ) ; xx [ 41 ]
= xx [ 2 ] * xx [ 63 ] * xx [ 49 ] ; xx [ 49 ] = xx [ 61 ] * xx [ 41 ] ; xx [
69 ] = - xx [ 49 ] ; xx [ 70 ] = xx [ 62 ] * xx [ 41 ] ; xx [ 71 ] = xx [ 60
] + xx [ 70 ] ; xx [ 72 ] = - xx [ 60 ] ; xx [ 73 ] = xx [ 69 ] ; xx [ 74 ] =
xx [ 71 ] ; pm_math_Vector3_cross_ra ( xx + 66 , xx + 72 , xx + 75 ) ; xx [
72 ] = xx [ 62 ] * xx [ 49 ] ; xx [ 73 ] = xx [ 21 ] * xx [ 61 ] ; xx [ 74 ]
= xx [ 73 ] * xx [ 61 ] ; xx [ 78 ] = xx [ 62 ] * xx [ 46 ] ; xx [ 79 ] = xx
[ 63 ] * xx [ 61 ] ; xx [ 80 ] = xx [ 78 ] + xx [ 79 ] ; xx [ 81 ] = xx [ 80
] * xx [ 3 ] ; xx [ 82 ] = xx [ 79 ] + xx [ 78 ] ; xx [ 78 ] = xx [ 82 ] * xx
[ 3 ] ; xx [ 79 ] = ( xx [ 80 ] * xx [ 81 ] + xx [ 82 ] * xx [ 78 ] ) * xx [
2 ] ; xx [ 80 ] = xx [ 3 ] - xx [ 51 ] ; xx [ 51 ] = xx [ 61 ] * xx [ 80 ] ;
xx [ 83 ] = xx [ 51 ] + xx [ 70 ] ; xx [ 84 ] = - xx [ 51 ] ; xx [ 85 ] = xx
[ 69 ] ; xx [ 86 ] = xx [ 83 ] ; pm_math_Vector3_cross_ra ( xx + 66 , xx + 84
, xx + 87 ) ; xx [ 51 ] = xx [ 4 ] * state [ 26 ] ; xx [ 70 ] = sin ( xx [ 51
] ) ; xx [ 84 ] = xx [ 3 ] * xx [ 70 ] ; xx [ 85 ] = xx [ 2 ] * xx [ 84 ] *
xx [ 70 ] ; xx [ 86 ] = xx [ 1 ] - xx [ 85 ] ; xx [ 90 ] = xx [ 4 ] * state [
24 ] ; xx [ 91 ] = xx [ 10 ] * sin ( xx [ 90 ] ) ; xx [ 92 ] = xx [ 10 ] *
cos ( xx [ 90 ] ) ; xx [ 93 ] = xx [ 91 ] ; xx [ 94 ] = xx [ 92 ] ; xx [ 95 ]
= xx [ 91 ] ; xx [ 90 ] = xx [ 86 ] * xx [ 91 ] ; xx [ 96 ] = cos ( xx [ 51 ]
) ; xx [ 51 ] = xx [ 2 ] * xx [ 96 ] * xx [ 84 ] ; xx [ 84 ] = xx [ 91 ] * xx
[ 51 ] ; xx [ 97 ] = - xx [ 84 ] ; xx [ 98 ] = xx [ 92 ] * xx [ 51 ] ; xx [
99 ] = xx [ 90 ] + xx [ 98 ] ; xx [ 100 ] = - xx [ 90 ] ; xx [ 101 ] = xx [
97 ] ; xx [ 102 ] = xx [ 99 ] ; pm_math_Vector3_cross_ra ( xx + 93 , xx + 100
, xx + 103 ) ; xx [ 100 ] = xx [ 92 ] * xx [ 84 ] ; xx [ 101 ] = xx [ 21 ] *
xx [ 91 ] ; xx [ 102 ] = xx [ 101 ] * xx [ 91 ] ; xx [ 106 ] = xx [ 92 ] * xx
[ 70 ] ; xx [ 107 ] = xx [ 96 ] * xx [ 91 ] ; xx [ 108 ] = xx [ 106 ] + xx [
107 ] ; xx [ 109 ] = xx [ 108 ] * xx [ 3 ] ; xx [ 110 ] = xx [ 107 ] + xx [
106 ] ; xx [ 106 ] = xx [ 110 ] * xx [ 3 ] ; xx [ 107 ] = ( xx [ 108 ] * xx [
109 ] + xx [ 110 ] * xx [ 106 ] ) * xx [ 2 ] ; xx [ 108 ] = xx [ 3 ] - xx [
85 ] ; xx [ 85 ] = xx [ 91 ] * xx [ 108 ] ; xx [ 111 ] = xx [ 85 ] + xx [ 98
] ; xx [ 112 ] = - xx [ 85 ] ; xx [ 113 ] = xx [ 97 ] ; xx [ 114 ] = xx [ 111
] ; pm_math_Vector3_cross_ra ( xx + 93 , xx + 112 , xx + 115 ) ; xx [ 75 ] =
xx [ 73 ] * xx [ 62 ] ; xx [ 73 ] = xx [ 61 ] * xx [ 46 ] - xx [ 63 ] * xx [
62 ] ; xx [ 46 ] = xx [ 81 ] * xx [ 73 ] ; xx [ 63 ] = ( xx [ 78 ] * xx [ 73
] + xx [ 46 ] ) * xx [ 2 ] ; xx [ 78 ] = xx [ 91 ] * xx [ 70 ] - xx [ 96 ] *
xx [ 92 ] ; xx [ 70 ] = xx [ 109 ] * xx [ 78 ] ; xx [ 85 ] = ( xx [ 106 ] *
xx [ 78 ] + xx [ 70 ] ) * xx [ 2 ] ; xx [ 87 ] = xx [ 101 ] * xx [ 92 ] ; xx
[ 118 ] = xx [ 0 ] ; xx [ 119 ] = xx [ 0 ] ; xx [ 120 ] = xx [ 0 ] ; xx [ 121
] = xx [ 0 ] ; xx [ 122 ] = xx [ 9 ] + ( xx [ 24 ] - xx [ 20 ] ) * xx [ 2 ] -
( xx [ 26 ] + xx [ 26 ] ) * xx [ 2 ] - xx [ 28 ] + xx [ 1 ] ; xx [ 123 ] = xx
[ 29 ] + xx [ 2 ] * ( xx [ 37 ] - xx [ 20 ] ) - xx [ 28 ] + xx [ 3 ] ; xx [
124 ] = xx [ 0 ] ; xx [ 125 ] = - ( xx [ 35 ] + ( xx [ 52 ] - xx [ 48 ] ) *
xx [ 2 ] - ( xx [ 50 ] + xx [ 50 ] ) * xx [ 2 ] - xx [ 55 ] + xx [ 1 ] ) ; xx
[ 126 ] = - ( xx [ 56 ] + xx [ 2 ] * ( xx [ 64 ] - xx [ 48 ] ) - xx [ 55 ] +
xx [ 3 ] ) ; xx [ 127 ] = xx [ 0 ] ; xx [ 128 ] = xx [ 0 ] ; xx [ 129 ] = xx
[ 0 ] ; xx [ 130 ] = xx [ 0 ] ; xx [ 131 ] = xx [ 0 ] ; xx [ 132 ] = xx [ 0 ]
; xx [ 133 ] = xx [ 0 ] ; xx [ 134 ] = xx [ 0 ] ; xx [ 135 ] = xx [ 0 ] ; xx
[ 136 ] = ( xx [ 14 ] * xx [ 19 ] + xx [ 25 ] ) * xx [ 2 ] + ( xx [ 23 ] + xx
[ 23 ] ) * xx [ 2 ] + xx [ 13 ] ; xx [ 137 ] = ( xx [ 32 ] * xx [ 14 ] + xx [
38 ] ) * xx [ 2 ] + xx [ 13 ] ; xx [ 138 ] = xx [ 0 ] ; xx [ 139 ] = - ( xx [
34 ] + ( xx [ 42 ] * xx [ 47 ] + xx [ 53 ] ) * xx [ 2 ] + ( xx [ 36 ] + xx [
36 ] ) * xx [ 2 ] ) ; xx [ 140 ] = - ( xx [ 34 ] + ( xx [ 59 ] * xx [ 42 ] +
xx [ 65 ] ) * xx [ 2 ] ) ; xx [ 141 ] = xx [ 0 ] ; xx [ 142 ] = xx [ 0 ] ; xx
[ 143 ] = xx [ 0 ] ; xx [ 144 ] = xx [ 0 ] ; xx [ 145 ] = xx [ 0 ] ; xx [ 146
] = xx [ 0 ] ; xx [ 147 ] = xx [ 0 ] ; xx [ 148 ] = xx [ 0 ] ; xx [ 149 ] =
xx [ 0 ] ; xx [ 150 ] = xx [ 0 ] ; xx [ 151 ] = xx [ 0 ] ; xx [ 152 ] = xx [
0 ] ; xx [ 153 ] = xx [ 0 ] ; xx [ 154 ] = xx [ 0 ] ; xx [ 155 ] = xx [ 54 ]
+ ( xx [ 76 ] - xx [ 72 ] ) * xx [ 2 ] - ( xx [ 74 ] + xx [ 74 ] ) * xx [ 2 ]
- xx [ 79 ] + xx [ 1 ] ; xx [ 156 ] = xx [ 80 ] + xx [ 2 ] * ( xx [ 88 ] - xx
[ 72 ] ) - xx [ 79 ] + xx [ 3 ] ; xx [ 157 ] = xx [ 0 ] ; xx [ 158 ] = - ( xx
[ 86 ] + ( xx [ 104 ] - xx [ 100 ] ) * xx [ 2 ] - ( xx [ 102 ] + xx [ 102 ] )
* xx [ 2 ] - xx [ 107 ] + xx [ 1 ] ) ; xx [ 159 ] = - ( xx [ 108 ] + xx [ 2 ]
* ( xx [ 116 ] - xx [ 100 ] ) - xx [ 107 ] + xx [ 3 ] ) ; xx [ 160 ] = xx [ 0
] ; xx [ 161 ] = xx [ 0 ] ; xx [ 162 ] = xx [ 0 ] ; xx [ 163 ] = xx [ 0 ] ;
xx [ 164 ] = xx [ 0 ] ; xx [ 165 ] = xx [ 0 ] ; xx [ 166 ] = xx [ 0 ] ; xx [
167 ] = xx [ 0 ] ; xx [ 168 ] = xx [ 0 ] ; xx [ 169 ] = ( xx [ 62 ] * xx [ 71
] + xx [ 77 ] ) * xx [ 2 ] + ( xx [ 75 ] + xx [ 75 ] ) * xx [ 2 ] - xx [ 63 ]
; xx [ 170 ] = ( xx [ 83 ] * xx [ 62 ] + xx [ 89 ] ) * xx [ 2 ] - xx [ 63 ] ;
xx [ 171 ] = xx [ 0 ] ; xx [ 172 ] = xx [ 85 ] - ( ( xx [ 92 ] * xx [ 99 ] +
xx [ 105 ] ) * xx [ 2 ] + ( xx [ 87 ] + xx [ 87 ] ) * xx [ 2 ] ) ; xx [ 173 ]
= xx [ 85 ] - ( xx [ 111 ] * xx [ 92 ] + xx [ 117 ] ) * xx [ 2 ] ; xx [ 13 ]
= xx [ 7 ] - xx [ 9 ] * xx [ 14 ] ; xx [ 63 ] = xx [ 7 ] ; xx [ 64 ] = - xx [
11 ] ; xx [ 65 ] = - xx [ 13 ] ; pm_math_Vector3_cross_ra ( xx + 15 , xx + 63
, xx + 98 ) ; xx [ 7 ] = xx [ 21 ] * xx [ 14 ] ; xx [ 9 ] = xx [ 3 ] * xx [
22 ] ; xx [ 15 ] = xx [ 33 ] - xx [ 35 ] * xx [ 42 ] ; xx [ 63 ] = xx [ 33 ]
; xx [ 64 ] = - xx [ 39 ] ; xx [ 65 ] = - xx [ 15 ] ;
pm_math_Vector3_cross_ra ( xx + 43 , xx + 63 , xx + 32 ) ; xx [ 16 ] = xx [
21 ] * xx [ 42 ] ; xx [ 17 ] = xx [ 3 ] * xx [ 27 ] ; xx [ 19 ] = 0.108 ; xx
[ 20 ] = xx [ 54 ] * xx [ 62 ] - xx [ 49 ] ; xx [ 43 ] = xx [ 69 ] ; xx [ 44
] = xx [ 60 ] ; xx [ 45 ] = - xx [ 20 ] ; pm_math_Vector3_cross_ra ( xx + 66
, xx + 43 , xx + 47 ) ; xx [ 24 ] = xx [ 21 ] * xx [ 62 ] ; xx [ 25 ] = xx [
3 ] * xx [ 73 ] ; xx [ 28 ] = xx [ 86 ] * xx [ 92 ] - xx [ 84 ] ; xx [ 43 ] =
xx [ 97 ] ; xx [ 44 ] = xx [ 90 ] ; xx [ 45 ] = - xx [ 28 ] ;
pm_math_Vector3_cross_ra ( xx + 93 , xx + 43 , xx + 52 ) ; xx [ 29 ] = xx [
21 ] * xx [ 92 ] ; xx [ 32 ] = xx [ 3 ] * xx [ 78 ] ; xx [ 63 ] = - ( ( xx [
11 ] * xx [ 14 ] + xx [ 99 ] ) * xx [ 2 ] + xx [ 5 ] + ( xx [ 23 ] + xx [ 7 ]
* xx [ 12 ] ) * xx [ 2 ] + ( xx [ 6 ] + xx [ 31 ] * xx [ 9 ] ) * xx [ 2 ] - (
( xx [ 39 ] * xx [ 42 ] + xx [ 33 ] ) * xx [ 2 ] + xx [ 8 ] + ( xx [ 36 ] +
xx [ 16 ] * xx [ 40 ] ) * xx [ 2 ] + ( xx [ 18 ] + xx [ 58 ] * xx [ 17 ] ) *
xx [ 2 ] ) + xx [ 19 ] ) ; xx [ 64 ] = xx [ 2 ] * ( xx [ 58 ] * xx [ 57 ] -
xx [ 17 ] * xx [ 27 ] ) + xx [ 2 ] * ( xx [ 34 ] + xx [ 15 ] * xx [ 42 ] ) -
xx [ 2 ] * ( xx [ 16 ] * xx [ 42 ] - xx [ 50 ] ) - ( xx [ 2 ] * ( xx [ 31 ] *
xx [ 30 ] - xx [ 9 ] * xx [ 22 ] ) + xx [ 2 ] * ( xx [ 100 ] + xx [ 13 ] * xx
[ 14 ] ) - xx [ 2 ] * ( xx [ 7 ] * xx [ 14 ] - xx [ 26 ] ) ) ; xx [ 65 ] = -
( ( xx [ 60 ] * xx [ 62 ] + xx [ 48 ] ) * xx [ 2 ] + xx [ 41 ] + ( xx [ 75 ]
+ xx [ 24 ] * xx [ 61 ] ) * xx [ 2 ] - ( xx [ 46 ] + xx [ 82 ] * xx [ 25 ] )
* xx [ 2 ] - ( ( xx [ 90 ] * xx [ 92 ] + xx [ 53 ] ) * xx [ 2 ] + xx [ 51 ] +
( xx [ 87 ] + xx [ 29 ] * xx [ 91 ] ) * xx [ 2 ] - ( xx [ 70 ] + xx [ 110 ] *
xx [ 32 ] ) * xx [ 2 ] ) + xx [ 19 ] ) ; xx [ 66 ] = xx [ 2 ] * ( xx [ 110 ]
* xx [ 109 ] - xx [ 32 ] * xx [ 78 ] ) + xx [ 2 ] * ( xx [ 54 ] - xx [ 28 ] *
xx [ 92 ] ) - xx [ 2 ] * ( xx [ 29 ] * xx [ 92 ] - xx [ 102 ] ) - ( xx [ 2 ]
* ( xx [ 82 ] * xx [ 81 ] - xx [ 25 ] * xx [ 73 ] ) + xx [ 2 ] * ( xx [ 49 ]
- xx [ 20 ] * xx [ 62 ] ) - xx [ 2 ] * ( xx [ 24 ] * xx [ 62 ] - xx [ 74 ] )
) ; xx [ 5 ] = 1.0e-8 ; memcpy ( xx + 174 , xx + 118 , 56 * sizeof ( double )
) ; factorAndSolveWide ( 4 , 14 , xx + 174 , xx + 6 , xx + 11 , ii + 0 , xx +
63 , xx [ 5 ] , xx + 22 ) ; xx [ 6 ] = state [ 10 ] + xx [ 27 ] ; xx [ 7 ] =
xx [ 6 ] * xx [ 4 ] ; xx [ 8 ] = sin ( xx [ 7 ] ) ; xx [ 9 ] = xx [ 3 ] * xx
[ 8 ] ; xx [ 11 ] = xx [ 2 ] * xx [ 9 ] * xx [ 8 ] ; xx [ 12 ] = xx [ 1 ] -
xx [ 11 ] ; xx [ 13 ] = state [ 8 ] + xx [ 26 ] ; xx [ 14 ] = xx [ 13 ] * xx
[ 4 ] ; xx [ 15 ] = xx [ 10 ] * sin ( xx [ 14 ] ) ; xx [ 16 ] = - xx [ 15 ] ;
xx [ 17 ] = xx [ 10 ] * cos ( xx [ 14 ] ) ; xx [ 36 ] = xx [ 16 ] ; xx [ 37 ]
= - xx [ 17 ] ; xx [ 38 ] = xx [ 16 ] ; xx [ 14 ] = xx [ 12 ] * xx [ 15 ] ;
xx [ 16 ] = cos ( xx [ 7 ] ) ; xx [ 7 ] = xx [ 2 ] * xx [ 16 ] * xx [ 9 ] ;
xx [ 9 ] = xx [ 15 ] * xx [ 7 ] ; xx [ 18 ] = xx [ 17 ] * xx [ 7 ] ; xx [ 20
] = xx [ 14 ] + xx [ 18 ] ; xx [ 39 ] = xx [ 14 ] ; xx [ 40 ] = xx [ 9 ] ; xx
[ 41 ] = - xx [ 20 ] ; pm_math_Vector3_cross_ra ( xx + 36 , xx + 39 , xx + 42
) ; xx [ 39 ] = xx [ 17 ] * xx [ 9 ] ; xx [ 40 ] = xx [ 21 ] * xx [ 15 ] ; xx
[ 41 ] = xx [ 40 ] * xx [ 15 ] ; xx [ 45 ] = xx [ 17 ] * xx [ 8 ] ; xx [ 46 ]
= xx [ 16 ] * xx [ 15 ] ; xx [ 47 ] = xx [ 45 ] + xx [ 46 ] ; xx [ 48 ] = xx
[ 47 ] * xx [ 3 ] ; xx [ 49 ] = xx [ 46 ] + xx [ 45 ] ; xx [ 45 ] = xx [ 49 ]
* xx [ 3 ] ; xx [ 46 ] = ( xx [ 47 ] * xx [ 48 ] + xx [ 49 ] * xx [ 45 ] ) *
xx [ 2 ] ; xx [ 47 ] = xx [ 3 ] - xx [ 11 ] ; xx [ 11 ] = xx [ 15 ] * xx [ 47
] ; xx [ 50 ] = xx [ 11 ] + xx [ 18 ] ; xx [ 51 ] = xx [ 11 ] ; xx [ 52 ] =
xx [ 9 ] ; xx [ 53 ] = - xx [ 50 ] ; pm_math_Vector3_cross_ra ( xx + 36 , xx
+ 51 , xx + 54 ) ; xx [ 11 ] = state [ 16 ] + xx [ 30 ] ; xx [ 18 ] = xx [ 11
] * xx [ 4 ] ; xx [ 51 ] = sin ( xx [ 18 ] ) ; xx [ 52 ] = xx [ 3 ] * xx [ 51
] ; xx [ 53 ] = xx [ 2 ] * xx [ 52 ] * xx [ 51 ] ; xx [ 57 ] = xx [ 1 ] - xx
[ 53 ] ; xx [ 58 ] = state [ 14 ] + xx [ 29 ] ; xx [ 59 ] = xx [ 58 ] * xx [
4 ] ; xx [ 60 ] = xx [ 10 ] * sin ( xx [ 59 ] ) ; xx [ 61 ] = - xx [ 60 ] ;
xx [ 62 ] = xx [ 10 ] * cos ( xx [ 59 ] ) ; xx [ 63 ] = xx [ 61 ] ; xx [ 64 ]
= - xx [ 62 ] ; xx [ 65 ] = xx [ 61 ] ; xx [ 59 ] = xx [ 57 ] * xx [ 60 ] ;
xx [ 61 ] = cos ( xx [ 18 ] ) ; xx [ 18 ] = xx [ 2 ] * xx [ 61 ] * xx [ 52 ]
; xx [ 52 ] = xx [ 60 ] * xx [ 18 ] ; xx [ 66 ] = xx [ 62 ] * xx [ 18 ] ; xx
[ 67 ] = xx [ 59 ] + xx [ 66 ] ; xx [ 68 ] = xx [ 59 ] ; xx [ 69 ] = xx [ 52
] ; xx [ 70 ] = - xx [ 67 ] ; pm_math_Vector3_cross_ra ( xx + 63 , xx + 68 ,
xx + 71 ) ; xx [ 68 ] = xx [ 62 ] * xx [ 52 ] ; xx [ 69 ] = xx [ 21 ] * xx [
60 ] ; xx [ 70 ] = xx [ 69 ] * xx [ 60 ] ; xx [ 74 ] = xx [ 62 ] * xx [ 51 ]
; xx [ 75 ] = xx [ 61 ] * xx [ 60 ] ; xx [ 76 ] = xx [ 74 ] + xx [ 75 ] ; xx
[ 77 ] = xx [ 76 ] * xx [ 3 ] ; xx [ 78 ] = xx [ 75 ] + xx [ 74 ] ; xx [ 74 ]
= xx [ 78 ] * xx [ 3 ] ; xx [ 75 ] = ( xx [ 76 ] * xx [ 77 ] + xx [ 78 ] * xx
[ 74 ] ) * xx [ 2 ] ; xx [ 76 ] = xx [ 3 ] - xx [ 53 ] ; xx [ 53 ] = xx [ 60
] * xx [ 76 ] ; xx [ 79 ] = xx [ 53 ] + xx [ 66 ] ; xx [ 80 ] = xx [ 53 ] ;
xx [ 81 ] = xx [ 52 ] ; xx [ 82 ] = - xx [ 79 ] ; pm_math_Vector3_cross_ra (
xx + 63 , xx + 80 , xx + 83 ) ; xx [ 42 ] = xx [ 40 ] * xx [ 17 ] ; xx [ 40 ]
= xx [ 16 ] * xx [ 17 ] - xx [ 15 ] * xx [ 8 ] ; xx [ 8 ] = xx [ 48 ] * xx [
40 ] ; xx [ 16 ] = ( xx [ 45 ] * xx [ 40 ] + xx [ 8 ] ) * xx [ 2 ] ; xx [ 45
] = xx [ 61 ] * xx [ 62 ] - xx [ 60 ] * xx [ 51 ] ; xx [ 51 ] = xx [ 77 ] *
xx [ 45 ] ; xx [ 53 ] = ( xx [ 74 ] * xx [ 45 ] + xx [ 51 ] ) * xx [ 2 ] ; xx
[ 54 ] = xx [ 69 ] * xx [ 62 ] ; xx [ 61 ] = state [ 20 ] + xx [ 32 ] ; xx [
66 ] = xx [ 61 ] * xx [ 4 ] ; xx [ 69 ] = sin ( xx [ 66 ] ) ; xx [ 71 ] = xx
[ 3 ] * xx [ 69 ] ; xx [ 74 ] = xx [ 2 ] * xx [ 71 ] * xx [ 69 ] ; xx [ 80 ]
= xx [ 1 ] - xx [ 74 ] ; xx [ 81 ] = state [ 18 ] + xx [ 31 ] ; xx [ 82 ] =
xx [ 81 ] * xx [ 4 ] ; xx [ 83 ] = xx [ 10 ] * sin ( xx [ 82 ] ) ; xx [ 86 ]
= xx [ 10 ] * cos ( xx [ 82 ] ) ; xx [ 87 ] = xx [ 83 ] ; xx [ 88 ] = xx [ 86
] ; xx [ 89 ] = xx [ 83 ] ; xx [ 82 ] = xx [ 80 ] * xx [ 83 ] ; xx [ 90 ] =
cos ( xx [ 66 ] ) ; xx [ 66 ] = xx [ 2 ] * xx [ 90 ] * xx [ 71 ] ; xx [ 71 ]
= xx [ 83 ] * xx [ 66 ] ; xx [ 91 ] = - xx [ 71 ] ; xx [ 92 ] = xx [ 86 ] *
xx [ 66 ] ; xx [ 93 ] = xx [ 82 ] + xx [ 92 ] ; xx [ 94 ] = - xx [ 82 ] ; xx
[ 95 ] = xx [ 91 ] ; xx [ 96 ] = xx [ 93 ] ; pm_math_Vector3_cross_ra ( xx +
87 , xx + 94 , xx + 97 ) ; xx [ 94 ] = xx [ 86 ] * xx [ 71 ] ; xx [ 95 ] = xx
[ 21 ] * xx [ 83 ] ; xx [ 96 ] = xx [ 95 ] * xx [ 83 ] ; xx [ 100 ] = xx [ 86
] * xx [ 69 ] ; xx [ 101 ] = xx [ 90 ] * xx [ 83 ] ; xx [ 102 ] = xx [ 100 ]
+ xx [ 101 ] ; xx [ 103 ] = xx [ 102 ] * xx [ 3 ] ; xx [ 104 ] = xx [ 101 ] +
xx [ 100 ] ; xx [ 100 ] = xx [ 104 ] * xx [ 3 ] ; xx [ 101 ] = ( xx [ 102 ] *
xx [ 103 ] + xx [ 104 ] * xx [ 100 ] ) * xx [ 2 ] ; xx [ 102 ] = xx [ 3 ] -
xx [ 74 ] ; xx [ 74 ] = xx [ 83 ] * xx [ 102 ] ; xx [ 105 ] = xx [ 74 ] + xx
[ 92 ] ; xx [ 106 ] = - xx [ 74 ] ; xx [ 107 ] = xx [ 91 ] ; xx [ 108 ] = xx
[ 105 ] ; pm_math_Vector3_cross_ra ( xx + 87 , xx + 106 , xx + 109 ) ; xx [
74 ] = state [ 26 ] + xx [ 35 ] ; xx [ 92 ] = xx [ 74 ] * xx [ 4 ] ; xx [ 106
] = sin ( xx [ 92 ] ) ; xx [ 107 ] = xx [ 3 ] * xx [ 106 ] ; xx [ 108 ] = xx
[ 2 ] * xx [ 107 ] * xx [ 106 ] ; xx [ 112 ] = xx [ 1 ] - xx [ 108 ] ; xx [
113 ] = state [ 24 ] + xx [ 34 ] ; xx [ 114 ] = xx [ 113 ] * xx [ 4 ] ; xx [
115 ] = xx [ 10 ] * sin ( xx [ 114 ] ) ; xx [ 116 ] = xx [ 10 ] * cos ( xx [
114 ] ) ; xx [ 117 ] = xx [ 115 ] ; xx [ 118 ] = xx [ 116 ] ; xx [ 119 ] = xx
[ 115 ] ; xx [ 114 ] = xx [ 112 ] * xx [ 115 ] ; xx [ 120 ] = cos ( xx [ 92 ]
) ; xx [ 92 ] = xx [ 2 ] * xx [ 120 ] * xx [ 107 ] ; xx [ 107 ] = xx [ 115 ]
* xx [ 92 ] ; xx [ 121 ] = - xx [ 107 ] ; xx [ 122 ] = xx [ 116 ] * xx [ 92 ]
; xx [ 123 ] = xx [ 114 ] + xx [ 122 ] ; xx [ 124 ] = - xx [ 114 ] ; xx [ 125
] = xx [ 121 ] ; xx [ 126 ] = xx [ 123 ] ; pm_math_Vector3_cross_ra ( xx +
117 , xx + 124 , xx + 127 ) ; xx [ 124 ] = xx [ 116 ] * xx [ 107 ] ; xx [ 125
] = xx [ 21 ] * xx [ 115 ] ; xx [ 126 ] = xx [ 125 ] * xx [ 115 ] ; xx [ 130
] = xx [ 116 ] * xx [ 106 ] ; xx [ 131 ] = xx [ 120 ] * xx [ 115 ] ; xx [ 132
] = xx [ 130 ] + xx [ 131 ] ; xx [ 133 ] = xx [ 132 ] * xx [ 3 ] ; xx [ 134 ]
= xx [ 131 ] + xx [ 130 ] ; xx [ 130 ] = xx [ 134 ] * xx [ 3 ] ; xx [ 131 ] =
( xx [ 132 ] * xx [ 133 ] + xx [ 134 ] * xx [ 130 ] ) * xx [ 2 ] ; xx [ 132 ]
= xx [ 3 ] - xx [ 108 ] ; xx [ 108 ] = xx [ 115 ] * xx [ 132 ] ; xx [ 135 ] =
xx [ 108 ] + xx [ 122 ] ; xx [ 136 ] = - xx [ 108 ] ; xx [ 137 ] = xx [ 121 ]
; xx [ 138 ] = xx [ 135 ] ; pm_math_Vector3_cross_ra ( xx + 117 , xx + 136 ,
xx + 139 ) ; xx [ 97 ] = xx [ 95 ] * xx [ 86 ] ; xx [ 95 ] = xx [ 83 ] * xx [
69 ] - xx [ 90 ] * xx [ 86 ] ; xx [ 69 ] = xx [ 103 ] * xx [ 95 ] ; xx [ 90 ]
= ( xx [ 100 ] * xx [ 95 ] + xx [ 69 ] ) * xx [ 2 ] ; xx [ 100 ] = xx [ 115 ]
* xx [ 106 ] - xx [ 120 ] * xx [ 116 ] ; xx [ 106 ] = xx [ 133 ] * xx [ 100 ]
; xx [ 108 ] = ( xx [ 130 ] * xx [ 100 ] + xx [ 106 ] ) * xx [ 2 ] ; xx [ 109
] = xx [ 125 ] * xx [ 116 ] ; xx [ 142 ] = xx [ 0 ] ; xx [ 143 ] = xx [ 0 ] ;
xx [ 144 ] = xx [ 0 ] ; xx [ 145 ] = xx [ 0 ] ; xx [ 146 ] = xx [ 12 ] + ( xx
[ 43 ] - xx [ 39 ] ) * xx [ 2 ] - ( xx [ 41 ] + xx [ 41 ] ) * xx [ 2 ] - xx [
46 ] + xx [ 1 ] ; xx [ 147 ] = xx [ 47 ] + xx [ 2 ] * ( xx [ 55 ] - xx [ 39 ]
) - xx [ 46 ] + xx [ 3 ] ; xx [ 148 ] = xx [ 0 ] ; xx [ 149 ] = - ( xx [ 57 ]
+ ( xx [ 72 ] - xx [ 68 ] ) * xx [ 2 ] - ( xx [ 70 ] + xx [ 70 ] ) * xx [ 2 ]
- xx [ 75 ] + xx [ 1 ] ) ; xx [ 150 ] = - ( xx [ 76 ] + xx [ 2 ] * ( xx [ 84
] - xx [ 68 ] ) - xx [ 75 ] + xx [ 3 ] ) ; xx [ 151 ] = xx [ 0 ] ; xx [ 152 ]
= xx [ 0 ] ; xx [ 153 ] = xx [ 0 ] ; xx [ 154 ] = xx [ 0 ] ; xx [ 155 ] = xx
[ 0 ] ; xx [ 156 ] = xx [ 0 ] ; xx [ 157 ] = xx [ 0 ] ; xx [ 158 ] = xx [ 0 ]
; xx [ 159 ] = xx [ 0 ] ; xx [ 160 ] = ( xx [ 17 ] * xx [ 20 ] + xx [ 44 ] )
* xx [ 2 ] + ( xx [ 42 ] + xx [ 42 ] ) * xx [ 2 ] + xx [ 16 ] ; xx [ 161 ] =
( xx [ 50 ] * xx [ 17 ] + xx [ 56 ] ) * xx [ 2 ] + xx [ 16 ] ; xx [ 162 ] =
xx [ 0 ] ; xx [ 163 ] = - ( xx [ 53 ] + ( xx [ 62 ] * xx [ 67 ] + xx [ 73 ] )
* xx [ 2 ] + ( xx [ 54 ] + xx [ 54 ] ) * xx [ 2 ] ) ; xx [ 164 ] = - ( xx [
53 ] + ( xx [ 79 ] * xx [ 62 ] + xx [ 85 ] ) * xx [ 2 ] ) ; xx [ 165 ] = xx [
0 ] ; xx [ 166 ] = xx [ 0 ] ; xx [ 167 ] = xx [ 0 ] ; xx [ 168 ] = xx [ 0 ] ;
xx [ 169 ] = xx [ 0 ] ; xx [ 170 ] = xx [ 0 ] ; xx [ 171 ] = xx [ 0 ] ; xx [
172 ] = xx [ 0 ] ; xx [ 173 ] = xx [ 0 ] ; xx [ 174 ] = xx [ 0 ] ; xx [ 175 ]
= xx [ 0 ] ; xx [ 176 ] = xx [ 0 ] ; xx [ 177 ] = xx [ 0 ] ; xx [ 178 ] = xx
[ 0 ] ; xx [ 179 ] = xx [ 80 ] + ( xx [ 98 ] - xx [ 94 ] ) * xx [ 2 ] - ( xx
[ 96 ] + xx [ 96 ] ) * xx [ 2 ] - xx [ 101 ] + xx [ 1 ] ; xx [ 180 ] = xx [
102 ] + xx [ 2 ] * ( xx [ 110 ] - xx [ 94 ] ) - xx [ 101 ] + xx [ 3 ] ; xx [
181 ] = xx [ 0 ] ; xx [ 182 ] = - ( xx [ 112 ] + ( xx [ 128 ] - xx [ 124 ] )
* xx [ 2 ] - ( xx [ 126 ] + xx [ 126 ] ) * xx [ 2 ] - xx [ 131 ] + xx [ 1 ] )
; xx [ 183 ] = - ( xx [ 132 ] + xx [ 2 ] * ( xx [ 140 ] - xx [ 124 ] ) - xx [
131 ] + xx [ 3 ] ) ; xx [ 184 ] = xx [ 0 ] ; xx [ 185 ] = xx [ 0 ] ; xx [ 186
] = xx [ 0 ] ; xx [ 187 ] = xx [ 0 ] ; xx [ 188 ] = xx [ 0 ] ; xx [ 189 ] =
xx [ 0 ] ; xx [ 190 ] = xx [ 0 ] ; xx [ 191 ] = xx [ 0 ] ; xx [ 192 ] = xx [
0 ] ; xx [ 193 ] = ( xx [ 86 ] * xx [ 93 ] + xx [ 99 ] ) * xx [ 2 ] + ( xx [
97 ] + xx [ 97 ] ) * xx [ 2 ] - xx [ 90 ] ; xx [ 194 ] = ( xx [ 105 ] * xx [
86 ] + xx [ 111 ] ) * xx [ 2 ] - xx [ 90 ] ; xx [ 195 ] = xx [ 0 ] ; xx [ 196
] = xx [ 108 ] - ( ( xx [ 116 ] * xx [ 123 ] + xx [ 129 ] ) * xx [ 2 ] + ( xx
[ 109 ] + xx [ 109 ] ) * xx [ 2 ] ) ; xx [ 197 ] = xx [ 108 ] - ( xx [ 135 ]
* xx [ 116 ] + xx [ 141 ] ) * xx [ 2 ] ; xx [ 16 ] = xx [ 9 ] - xx [ 12 ] *
xx [ 17 ] ; xx [ 122 ] = xx [ 9 ] ; xx [ 123 ] = - xx [ 14 ] ; xx [ 124 ] = -
xx [ 16 ] ; pm_math_Vector3_cross_ra ( xx + 36 , xx + 122 , xx + 127 ) ; xx [
9 ] = xx [ 21 ] * xx [ 17 ] ; xx [ 12 ] = xx [ 3 ] * xx [ 40 ] ; xx [ 20 ] =
xx [ 52 ] - xx [ 57 ] * xx [ 62 ] ; xx [ 36 ] = xx [ 52 ] ; xx [ 37 ] = - xx
[ 59 ] ; xx [ 38 ] = - xx [ 20 ] ; pm_math_Vector3_cross_ra ( xx + 63 , xx +
36 , xx + 55 ) ; xx [ 36 ] = xx [ 21 ] * xx [ 62 ] ; xx [ 37 ] = xx [ 3 ] *
xx [ 45 ] ; xx [ 38 ] = xx [ 80 ] * xx [ 86 ] - xx [ 71 ] ; xx [ 63 ] = xx [
91 ] ; xx [ 64 ] = xx [ 82 ] ; xx [ 65 ] = - xx [ 38 ] ;
pm_math_Vector3_cross_ra ( xx + 87 , xx + 63 , xx + 71 ) ; xx [ 39 ] = xx [
21 ] * xx [ 86 ] ; xx [ 43 ] = xx [ 3 ] * xx [ 95 ] ; xx [ 44 ] = xx [ 112 ]
* xx [ 116 ] - xx [ 107 ] ; xx [ 63 ] = xx [ 121 ] ; xx [ 64 ] = xx [ 114 ] ;
xx [ 65 ] = - xx [ 44 ] ; pm_math_Vector3_cross_ra ( xx + 117 , xx + 63 , xx
+ 87 ) ; xx [ 46 ] = xx [ 21 ] * xx [ 116 ] ; xx [ 47 ] = xx [ 3 ] * xx [ 100
] ; xx [ 117 ] = - ( ( xx [ 14 ] * xx [ 17 ] + xx [ 128 ] ) * xx [ 2 ] + xx [
7 ] + ( xx [ 42 ] + xx [ 9 ] * xx [ 15 ] ) * xx [ 2 ] + ( xx [ 8 ] + xx [ 49
] * xx [ 12 ] ) * xx [ 2 ] - ( ( xx [ 59 ] * xx [ 62 ] + xx [ 56 ] ) * xx [ 2
] + xx [ 18 ] + ( xx [ 54 ] + xx [ 36 ] * xx [ 60 ] ) * xx [ 2 ] + ( xx [ 51
] + xx [ 78 ] * xx [ 37 ] ) * xx [ 2 ] ) + xx [ 19 ] ) ; xx [ 118 ] = xx [ 2
] * ( xx [ 78 ] * xx [ 77 ] - xx [ 37 ] * xx [ 45 ] ) + xx [ 2 ] * ( xx [ 57
] + xx [ 20 ] * xx [ 62 ] ) - xx [ 2 ] * ( xx [ 36 ] * xx [ 62 ] - xx [ 70 ]
) - ( xx [ 2 ] * ( xx [ 49 ] * xx [ 48 ] - xx [ 12 ] * xx [ 40 ] ) + xx [ 2 ]
* ( xx [ 129 ] + xx [ 16 ] * xx [ 17 ] ) - xx [ 2 ] * ( xx [ 9 ] * xx [ 17 ]
- xx [ 41 ] ) ) ; xx [ 119 ] = - ( ( xx [ 82 ] * xx [ 86 ] + xx [ 72 ] ) * xx
[ 2 ] + xx [ 66 ] + ( xx [ 97 ] + xx [ 39 ] * xx [ 83 ] ) * xx [ 2 ] - ( xx [
69 ] + xx [ 104 ] * xx [ 43 ] ) * xx [ 2 ] - ( ( xx [ 114 ] * xx [ 116 ] + xx
[ 88 ] ) * xx [ 2 ] + xx [ 92 ] + ( xx [ 109 ] + xx [ 46 ] * xx [ 115 ] ) *
xx [ 2 ] - ( xx [ 106 ] + xx [ 134 ] * xx [ 47 ] ) * xx [ 2 ] ) + xx [ 19 ] )
; xx [ 120 ] = xx [ 2 ] * ( xx [ 134 ] * xx [ 133 ] - xx [ 47 ] * xx [ 100 ]
) + xx [ 2 ] * ( xx [ 89 ] - xx [ 44 ] * xx [ 116 ] ) - xx [ 2 ] * ( xx [ 46
] * xx [ 116 ] - xx [ 126 ] ) - ( xx [ 2 ] * ( xx [ 104 ] * xx [ 103 ] - xx [
43 ] * xx [ 95 ] ) + xx [ 2 ] * ( xx [ 73 ] - xx [ 38 ] * xx [ 86 ] ) - xx [
2 ] * ( xx [ 39 ] * xx [ 86 ] - xx [ 96 ] ) ) ; memcpy ( xx + 198 , xx + 142
, 56 * sizeof ( double ) ) ; factorAndSolveWide ( 4 , 14 , xx + 198 , xx + 14
, xx + 50 , ii + 0 , xx + 117 , xx [ 5 ] , xx + 36 ) ; xx [ 7 ] = xx [ 13 ] +
xx [ 40 ] ; xx [ 8 ] = xx [ 6 ] + xx [ 41 ] ; xx [ 6 ] = xx [ 58 ] + xx [ 43
] ; xx [ 9 ] = xx [ 11 ] + xx [ 44 ] ; xx [ 11 ] = xx [ 81 ] + xx [ 45 ] ; xx
[ 12 ] = xx [ 61 ] + xx [ 46 ] ; xx [ 13 ] = xx [ 113 ] + xx [ 48 ] ; xx [ 14
] = xx [ 74 ] + xx [ 49 ] ; xx [ 48 ] = state [ 0 ] + xx [ 22 ] + xx [ 36 ] ;
xx [ 49 ] = state [ 1 ] + xx [ 23 ] + xx [ 37 ] ; xx [ 50 ] = state [ 2 ] +
xx [ 24 ] + xx [ 38 ] ; xx [ 51 ] = state [ 3 ] ; xx [ 52 ] = state [ 4 ] ;
xx [ 53 ] = state [ 5 ] ; xx [ 54 ] = state [ 6 ] + xx [ 25 ] + xx [ 39 ] ;
xx [ 55 ] = state [ 7 ] ; xx [ 56 ] = xx [ 7 ] ; xx [ 57 ] = state [ 9 ] ; xx
[ 58 ] = xx [ 8 ] ; xx [ 59 ] = state [ 11 ] ; xx [ 60 ] = state [ 12 ] + xx
[ 28 ] + xx [ 42 ] ; xx [ 61 ] = state [ 13 ] ; xx [ 62 ] = xx [ 6 ] ; xx [
63 ] = state [ 15 ] ; xx [ 64 ] = xx [ 9 ] ; xx [ 65 ] = state [ 17 ] ; xx [
66 ] = xx [ 11 ] ; xx [ 67 ] = state [ 19 ] ; xx [ 68 ] = xx [ 12 ] ; xx [ 69
] = state [ 21 ] ; xx [ 70 ] = state [ 22 ] + xx [ 33 ] + xx [ 47 ] ; xx [ 71
] = state [ 23 ] ; xx [ 72 ] = xx [ 13 ] ; xx [ 73 ] = state [ 25 ] ; xx [ 74
] = xx [ 14 ] ; xx [ 75 ] = state [ 27 ] ; xx [ 76 ] = state [ 28 ] ; xx [ 77
] = state [ 29 ] ; xx [ 78 ] = state [ 30 ] ; xx [ 79 ] = state [ 31 ] ; xx [
15 ] = xx [ 8 ] * xx [ 4 ] ; xx [ 8 ] = sin ( xx [ 15 ] ) ; xx [ 16 ] = xx [
3 ] * xx [ 8 ] ; xx [ 17 ] = xx [ 1 ] - xx [ 2 ] * xx [ 16 ] * xx [ 8 ] ; xx
[ 18 ] = xx [ 7 ] * xx [ 4 ] ; xx [ 7 ] = xx [ 10 ] * sin ( xx [ 18 ] ) ; xx
[ 20 ] = xx [ 17 ] * xx [ 7 ] ; xx [ 22 ] = xx [ 10 ] * cos ( xx [ 18 ] ) ;
xx [ 18 ] = - xx [ 7 ] ; xx [ 23 ] = xx [ 18 ] ; xx [ 24 ] = - xx [ 22 ] ; xx
[ 25 ] = xx [ 18 ] ; xx [ 18 ] = cos ( xx [ 15 ] ) ; xx [ 15 ] = xx [ 2 ] *
xx [ 18 ] * xx [ 16 ] ; xx [ 16 ] = xx [ 7 ] * xx [ 15 ] ; xx [ 26 ] = xx [
16 ] - xx [ 17 ] * xx [ 22 ] ; xx [ 27 ] = xx [ 16 ] ; xx [ 28 ] = - xx [ 20
] ; xx [ 29 ] = - xx [ 26 ] ; pm_math_Vector3_cross_ra ( xx + 23 , xx + 27 ,
xx + 30 ) ; xx [ 16 ] = xx [ 21 ] * xx [ 7 ] ; xx [ 17 ] = xx [ 21 ] * xx [
22 ] ; xx [ 23 ] = xx [ 22 ] * xx [ 8 ] ; xx [ 24 ] = xx [ 18 ] * xx [ 7 ] ;
xx [ 25 ] = ( xx [ 23 ] + xx [ 24 ] ) * xx [ 3 ] ; xx [ 27 ] = xx [ 18 ] * xx
[ 22 ] - xx [ 7 ] * xx [ 8 ] ; xx [ 8 ] = xx [ 24 ] + xx [ 23 ] ; xx [ 18 ] =
xx [ 3 ] * xx [ 27 ] ; xx [ 23 ] = xx [ 9 ] * xx [ 4 ] ; xx [ 9 ] = sin ( xx
[ 23 ] ) ; xx [ 24 ] = xx [ 3 ] * xx [ 9 ] ; xx [ 28 ] = xx [ 1 ] - xx [ 2 ]
* xx [ 24 ] * xx [ 9 ] ; xx [ 29 ] = xx [ 6 ] * xx [ 4 ] ; xx [ 6 ] = xx [ 10
] * sin ( xx [ 29 ] ) ; xx [ 33 ] = xx [ 28 ] * xx [ 6 ] ; xx [ 34 ] = xx [
10 ] * cos ( xx [ 29 ] ) ; xx [ 29 ] = - xx [ 6 ] ; xx [ 35 ] = xx [ 29 ] ;
xx [ 36 ] = - xx [ 34 ] ; xx [ 37 ] = xx [ 29 ] ; xx [ 29 ] = cos ( xx [ 23 ]
) ; xx [ 23 ] = xx [ 2 ] * xx [ 29 ] * xx [ 24 ] ; xx [ 24 ] = xx [ 6 ] * xx
[ 23 ] ; xx [ 38 ] = xx [ 24 ] - xx [ 28 ] * xx [ 34 ] ; xx [ 39 ] = xx [ 24
] ; xx [ 40 ] = - xx [ 33 ] ; xx [ 41 ] = - xx [ 38 ] ;
pm_math_Vector3_cross_ra ( xx + 35 , xx + 39 , xx + 42 ) ; xx [ 24 ] = xx [
21 ] * xx [ 6 ] ; xx [ 28 ] = xx [ 21 ] * xx [ 34 ] ; xx [ 35 ] = xx [ 34 ] *
xx [ 9 ] ; xx [ 36 ] = xx [ 29 ] * xx [ 6 ] ; xx [ 37 ] = ( xx [ 35 ] + xx [
36 ] ) * xx [ 3 ] ; xx [ 39 ] = xx [ 29 ] * xx [ 34 ] - xx [ 6 ] * xx [ 9 ] ;
xx [ 9 ] = xx [ 36 ] + xx [ 35 ] ; xx [ 29 ] = xx [ 3 ] * xx [ 39 ] ; xx [ 30
] = xx [ 12 ] * xx [ 4 ] ; xx [ 12 ] = sin ( xx [ 30 ] ) ; xx [ 35 ] = xx [ 3
] * xx [ 12 ] ; xx [ 36 ] = xx [ 1 ] - xx [ 2 ] * xx [ 35 ] * xx [ 12 ] ; xx
[ 40 ] = xx [ 11 ] * xx [ 4 ] ; xx [ 11 ] = xx [ 10 ] * sin ( xx [ 40 ] ) ;
xx [ 41 ] = xx [ 36 ] * xx [ 11 ] ; xx [ 42 ] = xx [ 10 ] * cos ( xx [ 40 ] )
; xx [ 45 ] = xx [ 11 ] ; xx [ 46 ] = xx [ 42 ] ; xx [ 47 ] = xx [ 11 ] ; xx
[ 40 ] = cos ( xx [ 30 ] ) ; xx [ 30 ] = xx [ 2 ] * xx [ 40 ] * xx [ 35 ] ;
xx [ 35 ] = xx [ 11 ] * xx [ 30 ] ; xx [ 80 ] = xx [ 36 ] * xx [ 42 ] - xx [
35 ] ; xx [ 81 ] = - xx [ 35 ] ; xx [ 82 ] = xx [ 41 ] ; xx [ 83 ] = - xx [
80 ] ; pm_math_Vector3_cross_ra ( xx + 45 , xx + 81 , xx + 84 ) ; xx [ 35 ] =
xx [ 21 ] * xx [ 11 ] ; xx [ 36 ] = xx [ 21 ] * xx [ 42 ] ; xx [ 45 ] = xx [
42 ] * xx [ 12 ] ; xx [ 46 ] = xx [ 40 ] * xx [ 11 ] ; xx [ 47 ] = ( xx [ 45
] + xx [ 46 ] ) * xx [ 3 ] ; xx [ 81 ] = xx [ 11 ] * xx [ 12 ] - xx [ 40 ] *
xx [ 42 ] ; xx [ 12 ] = xx [ 46 ] + xx [ 45 ] ; xx [ 40 ] = xx [ 3 ] * xx [
81 ] ; xx [ 45 ] = xx [ 14 ] * xx [ 4 ] ; xx [ 14 ] = sin ( xx [ 45 ] ) ; xx
[ 46 ] = xx [ 3 ] * xx [ 14 ] ; xx [ 82 ] = xx [ 1 ] - xx [ 2 ] * xx [ 46 ] *
xx [ 14 ] ; xx [ 83 ] = xx [ 13 ] * xx [ 4 ] ; xx [ 13 ] = xx [ 10 ] * sin (
xx [ 83 ] ) ; xx [ 87 ] = xx [ 82 ] * xx [ 13 ] ; xx [ 88 ] = xx [ 10 ] * cos
( xx [ 83 ] ) ; xx [ 89 ] = xx [ 13 ] ; xx [ 90 ] = xx [ 88 ] ; xx [ 91 ] =
xx [ 13 ] ; xx [ 83 ] = cos ( xx [ 45 ] ) ; xx [ 45 ] = xx [ 2 ] * xx [ 83 ]
* xx [ 46 ] ; xx [ 46 ] = xx [ 13 ] * xx [ 45 ] ; xx [ 92 ] = xx [ 82 ] * xx
[ 88 ] - xx [ 46 ] ; xx [ 93 ] = - xx [ 46 ] ; xx [ 94 ] = xx [ 87 ] ; xx [
95 ] = - xx [ 92 ] ; pm_math_Vector3_cross_ra ( xx + 89 , xx + 93 , xx + 96 )
; xx [ 46 ] = xx [ 21 ] * xx [ 13 ] ; xx [ 82 ] = xx [ 21 ] * xx [ 88 ] ; xx
[ 89 ] = xx [ 88 ] * xx [ 14 ] ; xx [ 90 ] = xx [ 83 ] * xx [ 13 ] ; xx [ 91
] = ( xx [ 89 ] + xx [ 90 ] ) * xx [ 3 ] ; xx [ 93 ] = xx [ 13 ] * xx [ 14 ]
- xx [ 83 ] * xx [ 88 ] ; xx [ 14 ] = xx [ 90 ] + xx [ 89 ] ; xx [ 83 ] = xx
[ 3 ] * xx [ 93 ] ; xx [ 99 ] = fabs ( ( xx [ 20 ] * xx [ 22 ] + xx [ 31 ] )
* xx [ 2 ] + xx [ 15 ] + ( xx [ 16 ] * xx [ 22 ] + xx [ 17 ] * xx [ 7 ] ) *
xx [ 2 ] + ( xx [ 25 ] * xx [ 27 ] + xx [ 8 ] * xx [ 18 ] ) * xx [ 2 ] - ( (
xx [ 33 ] * xx [ 34 ] + xx [ 43 ] ) * xx [ 2 ] + xx [ 23 ] + ( xx [ 24 ] * xx
[ 34 ] + xx [ 28 ] * xx [ 6 ] ) * xx [ 2 ] + ( xx [ 37 ] * xx [ 39 ] + xx [ 9
] * xx [ 29 ] ) * xx [ 2 ] ) + xx [ 19 ] ) ; xx [ 100 ] = fabs ( xx [ 2 ] * (
xx [ 8 ] * xx [ 25 ] - xx [ 18 ] * xx [ 27 ] ) + xx [ 2 ] * ( xx [ 32 ] + xx
[ 26 ] * xx [ 22 ] ) - xx [ 2 ] * ( xx [ 17 ] * xx [ 22 ] - xx [ 16 ] * xx [
7 ] ) - ( xx [ 2 ] * ( xx [ 9 ] * xx [ 37 ] - xx [ 29 ] * xx [ 39 ] ) + xx [
2 ] * ( xx [ 44 ] + xx [ 38 ] * xx [ 34 ] ) - xx [ 2 ] * ( xx [ 28 ] * xx [
34 ] - xx [ 24 ] * xx [ 6 ] ) ) ) ; xx [ 101 ] = fabs ( ( xx [ 41 ] * xx [ 42
] + xx [ 85 ] ) * xx [ 2 ] + xx [ 30 ] + ( xx [ 35 ] * xx [ 42 ] + xx [ 36 ]
* xx [ 11 ] ) * xx [ 2 ] - ( xx [ 47 ] * xx [ 81 ] + xx [ 12 ] * xx [ 40 ] )
* xx [ 2 ] - ( ( xx [ 87 ] * xx [ 88 ] + xx [ 97 ] ) * xx [ 2 ] + xx [ 45 ] +
( xx [ 46 ] * xx [ 88 ] + xx [ 82 ] * xx [ 13 ] ) * xx [ 2 ] - ( xx [ 91 ] *
xx [ 93 ] + xx [ 14 ] * xx [ 83 ] ) * xx [ 2 ] ) + xx [ 19 ] ) ; xx [ 102 ] =
fabs ( xx [ 2 ] * ( xx [ 12 ] * xx [ 47 ] - xx [ 40 ] * xx [ 81 ] ) + xx [ 2
] * ( xx [ 86 ] - xx [ 80 ] * xx [ 42 ] ) - xx [ 2 ] * ( xx [ 36 ] * xx [ 42
] - xx [ 35 ] * xx [ 11 ] ) - ( xx [ 2 ] * ( xx [ 14 ] * xx [ 91 ] - xx [ 83
] * xx [ 93 ] ) + xx [ 2 ] * ( xx [ 98 ] - xx [ 92 ] * xx [ 88 ] ) - xx [ 2 ]
* ( xx [ 82 ] * xx [ 88 ] - xx [ 46 ] * xx [ 13 ] ) ) ) ; ii [ 0 ] = 99 ; {
int ll ; for ( ll = 100 ; ll < 103 ; ++ ll ) if ( xx [ ll ] > xx [ ii [ 0 ] ]
) ii [ 0 ] = ll ; } ii [ 0 ] -= 99 ; xx [ 6 ] = xx [ 99 + ( ii [ 0 ] ) ] ; xx
[ 7 ] = 1.0e-9 ; xx [ 8 ] = xx [ 6 ] - xx [ 7 ] ; if ( xx [ 8 ] < 0.0 ) ii [
1 ] = - 1 ; else if ( xx [ 8 ] > 0.0 ) ii [ 1 ] = + 1 ; else ii [ 1 ] = 0 ;
ii [ 2 ] = ii [ 1 ] ; if ( 0 > ii [ 2 ] ) ii [ 2 ] = 0 ; if ( ii [ 2 ] != 0 )
{ switch ( ii [ 0 ] ) { case 0 : case 1 : { return sm_ssci_recordRunTimeError
( "sm:compiler:messages:simulationErrors:ConstraintViolation" ,
 "'wheel_leg/blance_leg_model/Revolute Joint4' kinematic constraints cannot be maintained. Check solver type and consistency tolerance in the Simscape Solver Configuration block. Check Simulink solver type and tolerances in Model Configuration Parameters. A kinematic singularity might be the source of this problem."
, neDiagMgr ) ; } case 2 : case 3 : { return sm_ssci_recordRunTimeError (
"sm:compiler:messages:simulationErrors:ConstraintViolation" ,
 "'wheel_leg/blance_leg_model/Revolute Joint7' kinematic constraints cannot be maintained. Check solver type and consistency tolerance in the Simscape Solver Configuration block. Check Simulink solver type and tolerances in Model Configuration Parameters. A kinematic singularity might be the source of this problem."
, neDiagMgr ) ; } } } xx [ 6 ] = xx [ 4 ] * xx [ 58 ] ; xx [ 8 ] = sin ( xx [
6 ] ) ; xx [ 9 ] = xx [ 3 ] * xx [ 8 ] ; xx [ 11 ] = xx [ 2 ] * xx [ 9 ] * xx
[ 8 ] ; xx [ 12 ] = xx [ 1 ] - xx [ 11 ] ; xx [ 13 ] = xx [ 4 ] * xx [ 56 ] ;
xx [ 14 ] = xx [ 10 ] * sin ( xx [ 13 ] ) ; xx [ 15 ] = - xx [ 14 ] ; xx [ 16
] = xx [ 10 ] * cos ( xx [ 13 ] ) ; xx [ 17 ] = xx [ 15 ] ; xx [ 18 ] = - xx
[ 16 ] ; xx [ 19 ] = xx [ 15 ] ; xx [ 13 ] = xx [ 12 ] * xx [ 14 ] ; xx [ 15
] = cos ( xx [ 6 ] ) ; xx [ 6 ] = xx [ 15 ] * xx [ 9 ] ; xx [ 9 ] = xx [ 2 ]
* xx [ 6 ] ; xx [ 20 ] = xx [ 14 ] * xx [ 9 ] ; xx [ 22 ] = xx [ 16 ] * xx [
9 ] ; xx [ 23 ] = xx [ 13 ] + xx [ 22 ] ; xx [ 24 ] = xx [ 13 ] ; xx [ 25 ] =
xx [ 20 ] ; xx [ 26 ] = - xx [ 23 ] ; pm_math_Vector3_cross_ra ( xx + 17 , xx
+ 24 , xx + 27 ) ; xx [ 13 ] = xx [ 16 ] * xx [ 20 ] ; xx [ 24 ] = xx [ 21 ]
* xx [ 14 ] ; xx [ 25 ] = xx [ 24 ] * xx [ 14 ] ; xx [ 26 ] = ( xx [ 25 ] +
xx [ 25 ] ) * xx [ 2 ] ; xx [ 25 ] = xx [ 16 ] * xx [ 8 ] ; xx [ 30 ] = xx [
15 ] * xx [ 14 ] ; xx [ 31 ] = xx [ 25 ] + xx [ 30 ] ; xx [ 32 ] = xx [ 31 ]
* xx [ 3 ] ; xx [ 33 ] = xx [ 30 ] + xx [ 25 ] ; xx [ 25 ] = xx [ 33 ] * xx [
3 ] ; xx [ 30 ] = ( xx [ 31 ] * xx [ 32 ] + xx [ 33 ] * xx [ 25 ] ) * xx [ 2
] ; xx [ 34 ] = xx [ 3 ] - xx [ 11 ] ; xx [ 11 ] = xx [ 14 ] * xx [ 34 ] ; xx
[ 35 ] = xx [ 11 ] + xx [ 22 ] ; xx [ 36 ] = xx [ 11 ] ; xx [ 37 ] = xx [ 20
] ; xx [ 38 ] = - xx [ 35 ] ; pm_math_Vector3_cross_ra ( xx + 17 , xx + 36 ,
xx + 39 ) ; xx [ 11 ] = xx [ 4 ] * xx [ 64 ] ; xx [ 20 ] = sin ( xx [ 11 ] )
; xx [ 22 ] = xx [ 3 ] * xx [ 20 ] ; xx [ 36 ] = xx [ 2 ] * xx [ 22 ] * xx [
20 ] ; xx [ 37 ] = xx [ 1 ] - xx [ 36 ] ; xx [ 38 ] = xx [ 4 ] * xx [ 62 ] ;
xx [ 42 ] = xx [ 10 ] * sin ( xx [ 38 ] ) ; xx [ 43 ] = - xx [ 42 ] ; xx [ 44
] = xx [ 10 ] * cos ( xx [ 38 ] ) ; xx [ 45 ] = xx [ 43 ] ; xx [ 46 ] = - xx
[ 44 ] ; xx [ 47 ] = xx [ 43 ] ; xx [ 38 ] = xx [ 37 ] * xx [ 42 ] ; xx [ 43
] = cos ( xx [ 11 ] ) ; xx [ 11 ] = xx [ 43 ] * xx [ 22 ] ; xx [ 22 ] = xx [
2 ] * xx [ 11 ] ; xx [ 80 ] = xx [ 42 ] * xx [ 22 ] ; xx [ 81 ] = xx [ 44 ] *
xx [ 22 ] ; xx [ 82 ] = xx [ 38 ] + xx [ 81 ] ; xx [ 83 ] = xx [ 38 ] ; xx [
84 ] = xx [ 80 ] ; xx [ 85 ] = - xx [ 82 ] ; pm_math_Vector3_cross_ra ( xx +
45 , xx + 83 , xx + 86 ) ; xx [ 38 ] = xx [ 44 ] * xx [ 80 ] ; xx [ 83 ] = xx
[ 21 ] * xx [ 42 ] ; xx [ 84 ] = xx [ 83 ] * xx [ 42 ] ; xx [ 85 ] = ( xx [
84 ] + xx [ 84 ] ) * xx [ 2 ] ; xx [ 84 ] = xx [ 44 ] * xx [ 20 ] ; xx [ 89 ]
= xx [ 43 ] * xx [ 42 ] ; xx [ 90 ] = xx [ 84 ] + xx [ 89 ] ; xx [ 91 ] = xx
[ 90 ] * xx [ 3 ] ; xx [ 92 ] = xx [ 89 ] + xx [ 84 ] ; xx [ 84 ] = xx [ 92 ]
* xx [ 3 ] ; xx [ 89 ] = ( xx [ 90 ] * xx [ 91 ] + xx [ 92 ] * xx [ 84 ] ) *
xx [ 2 ] ; xx [ 93 ] = xx [ 3 ] - xx [ 36 ] ; xx [ 36 ] = xx [ 42 ] * xx [ 93
] ; xx [ 94 ] = xx [ 36 ] + xx [ 81 ] ; xx [ 95 ] = xx [ 36 ] ; xx [ 96 ] =
xx [ 80 ] ; xx [ 97 ] = - xx [ 94 ] ; pm_math_Vector3_cross_ra ( xx + 45 , xx
+ 95 , xx + 98 ) ; xx [ 27 ] = xx [ 24 ] * xx [ 16 ] ; xx [ 24 ] = xx [ 27 ]
+ xx [ 27 ] ; xx [ 27 ] = xx [ 24 ] * xx [ 2 ] ; xx [ 36 ] = xx [ 15 ] * xx [
16 ] - xx [ 14 ] * xx [ 8 ] ; xx [ 8 ] = ( xx [ 25 ] * xx [ 36 ] + xx [ 32 ]
* xx [ 36 ] ) * xx [ 2 ] ; xx [ 15 ] = xx [ 43 ] * xx [ 44 ] - xx [ 42 ] * xx
[ 20 ] ; xx [ 20 ] = ( xx [ 84 ] * xx [ 15 ] + xx [ 91 ] * xx [ 15 ] ) * xx [
2 ] ; xx [ 25 ] = xx [ 83 ] * xx [ 44 ] ; xx [ 32 ] = xx [ 25 ] + xx [ 25 ] ;
xx [ 25 ] = xx [ 32 ] * xx [ 2 ] ; xx [ 39 ] = xx [ 4 ] * xx [ 68 ] ; xx [ 43
] = sin ( xx [ 39 ] ) ; xx [ 80 ] = xx [ 3 ] * xx [ 43 ] ; xx [ 81 ] = xx [ 2
] * xx [ 80 ] * xx [ 43 ] ; xx [ 83 ] = xx [ 1 ] - xx [ 81 ] ; xx [ 84 ] = xx
[ 4 ] * xx [ 66 ] ; xx [ 86 ] = xx [ 10 ] * sin ( xx [ 84 ] ) ; xx [ 91 ] =
xx [ 10 ] * cos ( xx [ 84 ] ) ; xx [ 95 ] = xx [ 86 ] ; xx [ 96 ] = xx [ 91 ]
; xx [ 97 ] = xx [ 86 ] ; xx [ 84 ] = xx [ 83 ] * xx [ 86 ] ; xx [ 98 ] = cos
( xx [ 39 ] ) ; xx [ 39 ] = xx [ 98 ] * xx [ 80 ] ; xx [ 80 ] = xx [ 2 ] * xx
[ 39 ] ; xx [ 101 ] = xx [ 86 ] * xx [ 80 ] ; xx [ 102 ] = - xx [ 101 ] ; xx
[ 103 ] = xx [ 91 ] * xx [ 80 ] ; xx [ 104 ] = xx [ 84 ] + xx [ 103 ] ; xx [
105 ] = - xx [ 84 ] ; xx [ 106 ] = xx [ 102 ] ; xx [ 107 ] = xx [ 104 ] ;
pm_math_Vector3_cross_ra ( xx + 95 , xx + 105 , xx + 108 ) ; xx [ 84 ] = xx [
91 ] * xx [ 101 ] ; xx [ 101 ] = xx [ 21 ] * xx [ 86 ] ; xx [ 105 ] = xx [
101 ] * xx [ 86 ] ; xx [ 106 ] = ( xx [ 105 ] + xx [ 105 ] ) * xx [ 2 ] ; xx
[ 105 ] = xx [ 91 ] * xx [ 43 ] ; xx [ 107 ] = xx [ 98 ] * xx [ 86 ] ; xx [
111 ] = xx [ 105 ] + xx [ 107 ] ; xx [ 112 ] = xx [ 111 ] * xx [ 3 ] ; xx [
113 ] = xx [ 107 ] + xx [ 105 ] ; xx [ 105 ] = xx [ 113 ] * xx [ 3 ] ; xx [
107 ] = ( xx [ 111 ] * xx [ 112 ] + xx [ 113 ] * xx [ 105 ] ) * xx [ 2 ] ; xx
[ 114 ] = xx [ 3 ] - xx [ 81 ] ; xx [ 81 ] = xx [ 86 ] * xx [ 114 ] ; xx [
115 ] = xx [ 81 ] + xx [ 103 ] ; xx [ 116 ] = - xx [ 81 ] ; xx [ 117 ] = xx [
102 ] ; xx [ 118 ] = xx [ 115 ] ; pm_math_Vector3_cross_ra ( xx + 95 , xx +
116 , xx + 119 ) ; xx [ 81 ] = xx [ 4 ] * xx [ 74 ] ; xx [ 102 ] = sin ( xx [
81 ] ) ; xx [ 103 ] = xx [ 3 ] * xx [ 102 ] ; xx [ 116 ] = xx [ 2 ] * xx [
103 ] * xx [ 102 ] ; xx [ 117 ] = xx [ 1 ] - xx [ 116 ] ; xx [ 118 ] = xx [ 4
] * xx [ 72 ] ; xx [ 122 ] = xx [ 10 ] * sin ( xx [ 118 ] ) ; xx [ 123 ] = xx
[ 10 ] * cos ( xx [ 118 ] ) ; xx [ 124 ] = xx [ 122 ] ; xx [ 125 ] = xx [ 123
] ; xx [ 126 ] = xx [ 122 ] ; xx [ 118 ] = xx [ 117 ] * xx [ 122 ] ; xx [ 127
] = cos ( xx [ 81 ] ) ; xx [ 81 ] = xx [ 127 ] * xx [ 103 ] ; xx [ 103 ] = xx
[ 2 ] * xx [ 81 ] ; xx [ 128 ] = xx [ 122 ] * xx [ 103 ] ; xx [ 129 ] = - xx
[ 128 ] ; xx [ 130 ] = xx [ 123 ] * xx [ 103 ] ; xx [ 131 ] = xx [ 118 ] + xx
[ 130 ] ; xx [ 132 ] = - xx [ 118 ] ; xx [ 133 ] = xx [ 129 ] ; xx [ 134 ] =
xx [ 131 ] ; pm_math_Vector3_cross_ra ( xx + 124 , xx + 132 , xx + 135 ) ; xx
[ 118 ] = xx [ 123 ] * xx [ 128 ] ; xx [ 128 ] = xx [ 21 ] * xx [ 122 ] ; xx
[ 132 ] = xx [ 128 ] * xx [ 122 ] ; xx [ 133 ] = ( xx [ 132 ] + xx [ 132 ] )
* xx [ 2 ] ; xx [ 132 ] = xx [ 123 ] * xx [ 102 ] ; xx [ 134 ] = xx [ 127 ] *
xx [ 122 ] ; xx [ 138 ] = xx [ 132 ] + xx [ 134 ] ; xx [ 139 ] = xx [ 138 ] *
xx [ 3 ] ; xx [ 140 ] = xx [ 134 ] + xx [ 132 ] ; xx [ 132 ] = xx [ 140 ] *
xx [ 3 ] ; xx [ 134 ] = ( xx [ 138 ] * xx [ 139 ] + xx [ 140 ] * xx [ 132 ] )
* xx [ 2 ] ; xx [ 141 ] = xx [ 3 ] - xx [ 116 ] ; xx [ 116 ] = xx [ 122 ] *
xx [ 141 ] ; xx [ 142 ] = xx [ 116 ] + xx [ 130 ] ; xx [ 143 ] = - xx [ 116 ]
; xx [ 144 ] = xx [ 129 ] ; xx [ 145 ] = xx [ 142 ] ;
pm_math_Vector3_cross_ra ( xx + 124 , xx + 143 , xx + 146 ) ; xx [ 108 ] = xx
[ 101 ] * xx [ 91 ] ; xx [ 101 ] = xx [ 108 ] + xx [ 108 ] ; xx [ 108 ] = xx
[ 101 ] * xx [ 2 ] ; xx [ 116 ] = xx [ 86 ] * xx [ 43 ] - xx [ 98 ] * xx [ 91
] ; xx [ 43 ] = ( xx [ 105 ] * xx [ 116 ] + xx [ 112 ] * xx [ 116 ] ) * xx [
2 ] ; xx [ 98 ] = xx [ 122 ] * xx [ 102 ] - xx [ 127 ] * xx [ 123 ] ; xx [
102 ] = ( xx [ 132 ] * xx [ 98 ] + xx [ 139 ] * xx [ 98 ] ) * xx [ 2 ] ; xx [
105 ] = xx [ 128 ] * xx [ 123 ] ; xx [ 112 ] = xx [ 105 ] + xx [ 105 ] ; xx [
105 ] = xx [ 112 ] * xx [ 2 ] ; xx [ 149 ] = xx [ 0 ] ; xx [ 150 ] = xx [ 0 ]
; xx [ 151 ] = xx [ 0 ] ; xx [ 152 ] = xx [ 0 ] ; xx [ 153 ] = xx [ 12 ] + (
xx [ 28 ] - xx [ 13 ] ) * xx [ 2 ] - xx [ 26 ] - xx [ 30 ] + xx [ 1 ] ; xx [
154 ] = xx [ 34 ] + xx [ 2 ] * ( xx [ 40 ] - xx [ 13 ] ) - xx [ 30 ] + xx [ 3
] ; xx [ 155 ] = xx [ 0 ] ; xx [ 156 ] = - ( xx [ 37 ] + ( xx [ 87 ] - xx [
38 ] ) * xx [ 2 ] - xx [ 85 ] - xx [ 89 ] + xx [ 1 ] ) ; xx [ 157 ] = - ( xx
[ 93 ] + xx [ 2 ] * ( xx [ 99 ] - xx [ 38 ] ) - xx [ 89 ] + xx [ 3 ] ) ; xx [
158 ] = xx [ 0 ] ; xx [ 159 ] = xx [ 0 ] ; xx [ 160 ] = xx [ 0 ] ; xx [ 161 ]
= xx [ 0 ] ; xx [ 162 ] = xx [ 0 ] ; xx [ 163 ] = xx [ 0 ] ; xx [ 164 ] = xx
[ 0 ] ; xx [ 165 ] = xx [ 0 ] ; xx [ 166 ] = xx [ 0 ] ; xx [ 167 ] = ( xx [
16 ] * xx [ 23 ] + xx [ 29 ] ) * xx [ 2 ] + xx [ 27 ] + xx [ 8 ] ; xx [ 168 ]
= ( xx [ 35 ] * xx [ 16 ] + xx [ 41 ] ) * xx [ 2 ] + xx [ 8 ] ; xx [ 169 ] =
xx [ 0 ] ; xx [ 170 ] = - ( xx [ 20 ] + ( xx [ 44 ] * xx [ 82 ] + xx [ 88 ] )
* xx [ 2 ] + xx [ 25 ] ) ; xx [ 171 ] = - ( xx [ 20 ] + ( xx [ 94 ] * xx [ 44
] + xx [ 100 ] ) * xx [ 2 ] ) ; xx [ 172 ] = xx [ 0 ] ; xx [ 173 ] = xx [ 0 ]
; xx [ 174 ] = xx [ 0 ] ; xx [ 175 ] = xx [ 0 ] ; xx [ 176 ] = xx [ 0 ] ; xx
[ 177 ] = xx [ 0 ] ; xx [ 178 ] = xx [ 0 ] ; xx [ 179 ] = xx [ 0 ] ; xx [ 180
] = xx [ 0 ] ; xx [ 181 ] = xx [ 0 ] ; xx [ 182 ] = xx [ 0 ] ; xx [ 183 ] =
xx [ 0 ] ; xx [ 184 ] = xx [ 0 ] ; xx [ 185 ] = xx [ 0 ] ; xx [ 186 ] = xx [
83 ] + ( xx [ 109 ] - xx [ 84 ] ) * xx [ 2 ] - xx [ 106 ] - xx [ 107 ] + xx [
1 ] ; xx [ 187 ] = xx [ 114 ] + xx [ 2 ] * ( xx [ 120 ] - xx [ 84 ] ) - xx [
107 ] + xx [ 3 ] ; xx [ 188 ] = xx [ 0 ] ; xx [ 189 ] = - ( xx [ 117 ] + ( xx
[ 136 ] - xx [ 118 ] ) * xx [ 2 ] - xx [ 133 ] - xx [ 134 ] + xx [ 1 ] ) ; xx
[ 190 ] = - ( xx [ 141 ] + xx [ 2 ] * ( xx [ 147 ] - xx [ 118 ] ) - xx [ 134
] + xx [ 3 ] ) ; xx [ 191 ] = xx [ 0 ] ; xx [ 192 ] = xx [ 0 ] ; xx [ 193 ] =
xx [ 0 ] ; xx [ 194 ] = xx [ 0 ] ; xx [ 195 ] = xx [ 0 ] ; xx [ 196 ] = xx [
0 ] ; xx [ 197 ] = xx [ 0 ] ; xx [ 198 ] = xx [ 0 ] ; xx [ 199 ] = xx [ 0 ] ;
xx [ 200 ] = ( xx [ 91 ] * xx [ 104 ] + xx [ 110 ] ) * xx [ 2 ] + xx [ 108 ]
- xx [ 43 ] ; xx [ 201 ] = ( xx [ 115 ] * xx [ 91 ] + xx [ 121 ] ) * xx [ 2 ]
- xx [ 43 ] ; xx [ 202 ] = xx [ 0 ] ; xx [ 203 ] = xx [ 102 ] - ( ( xx [ 123
] * xx [ 131 ] + xx [ 137 ] ) * xx [ 2 ] + xx [ 105 ] ) ; xx [ 204 ] = xx [
102 ] - ( xx [ 142 ] * xx [ 123 ] + xx [ 148 ] ) * xx [ 2 ] ; xx [ 0 ] = ( xx
[ 63 ] + xx [ 65 ] ) * xx [ 3 ] ; xx [ 1 ] = xx [ 90 ] * xx [ 0 ] ; xx [ 8 ]
= xx [ 92 ] * xx [ 0 ] ; xx [ 13 ] = xx [ 21 ] - xx [ 85 ] ; xx [ 20 ] = xx [
65 ] * xx [ 93 ] + xx [ 37 ] * xx [ 63 ] ; xx [ 23 ] = xx [ 20 ] * xx [ 42 ]
; xx [ 28 ] = xx [ 63 ] * xx [ 22 ] + xx [ 2 ] * xx [ 65 ] * xx [ 11 ] ; xx [
11 ] = xx [ 42 ] * xx [ 28 ] ; xx [ 29 ] = xx [ 23 ] + xx [ 44 ] * xx [ 28 ]
; xx [ 87 ] = xx [ 23 ] ; xx [ 88 ] = xx [ 11 ] ; xx [ 89 ] = - xx [ 29 ] ;
pm_math_Vector3_cross_ra ( xx + 45 , xx + 87 , xx + 118 ) ; xx [ 23 ] = ( xx
[ 57 ] + xx [ 59 ] ) * xx [ 3 ] ; xx [ 28 ] = xx [ 31 ] * xx [ 23 ] ; xx [ 30
] = xx [ 33 ] * xx [ 23 ] ; xx [ 35 ] = xx [ 21 ] - xx [ 26 ] ; xx [ 26 ] =
xx [ 59 ] * xx [ 34 ] + xx [ 12 ] * xx [ 57 ] ; xx [ 38 ] = xx [ 26 ] * xx [
14 ] ; xx [ 40 ] = xx [ 57 ] * xx [ 9 ] + xx [ 2 ] * xx [ 59 ] * xx [ 6 ] ;
xx [ 6 ] = xx [ 14 ] * xx [ 40 ] ; xx [ 41 ] = xx [ 38 ] + xx [ 16 ] * xx [
40 ] ; xx [ 87 ] = xx [ 38 ] ; xx [ 88 ] = xx [ 6 ] ; xx [ 89 ] = - xx [ 41 ]
; pm_math_Vector3_cross_ra ( xx + 17 , xx + 87 , xx + 127 ) ; xx [ 38 ] = (
xx [ 73 ] + xx [ 75 ] ) * xx [ 3 ] ; xx [ 40 ] = xx [ 138 ] * xx [ 38 ] ; xx
[ 43 ] = xx [ 140 ] * xx [ 38 ] ; xx [ 82 ] = xx [ 21 ] - xx [ 133 ] ; xx [
84 ] = xx [ 75 ] * xx [ 141 ] + xx [ 117 ] * xx [ 73 ] ; xx [ 85 ] = xx [ 84
] * xx [ 122 ] ; xx [ 87 ] = xx [ 73 ] * xx [ 103 ] + xx [ 2 ] * xx [ 75 ] *
xx [ 81 ] ; xx [ 81 ] = xx [ 122 ] * xx [ 87 ] ; xx [ 88 ] = xx [ 85 ] + xx [
123 ] * xx [ 87 ] ; xx [ 130 ] = - xx [ 85 ] ; xx [ 131 ] = - xx [ 81 ] ; xx
[ 132 ] = xx [ 88 ] ; pm_math_Vector3_cross_ra ( xx + 124 , xx + 130 , xx +
133 ) ; xx [ 85 ] = ( xx [ 67 ] + xx [ 69 ] ) * xx [ 3 ] ; xx [ 87 ] = xx [
111 ] * xx [ 85 ] ; xx [ 89 ] = xx [ 113 ] * xx [ 85 ] ; xx [ 94 ] = xx [ 21
] - xx [ 106 ] ; xx [ 21 ] = xx [ 69 ] * xx [ 114 ] + xx [ 83 ] * xx [ 67 ] ;
xx [ 99 ] = xx [ 21 ] * xx [ 86 ] ; xx [ 100 ] = xx [ 67 ] * xx [ 80 ] + xx [
2 ] * xx [ 69 ] * xx [ 39 ] ; xx [ 39 ] = xx [ 86 ] * xx [ 100 ] ; xx [ 102 ]
= xx [ 99 ] + xx [ 91 ] * xx [ 100 ] ; xx [ 130 ] = - xx [ 99 ] ; xx [ 131 ]
= - xx [ 39 ] ; xx [ 132 ] = xx [ 102 ] ; pm_math_Vector3_cross_ra ( xx + 95
, xx + 130 , xx + 142 ) ; xx [ 130 ] = xx [ 0 ] - ( xx [ 90 ] * xx [ 1 ] + xx
[ 92 ] * xx [ 8 ] ) * xx [ 2 ] + xx [ 63 ] * xx [ 13 ] + xx [ 20 ] + ( xx [
119 ] - xx [ 44 ] * xx [ 11 ] ) * xx [ 2 ] - ( xx [ 23 ] - ( xx [ 31 ] * xx [
28 ] + xx [ 33 ] * xx [ 30 ] ) * xx [ 2 ] + xx [ 57 ] * xx [ 35 ] + xx [ 26 ]
+ ( xx [ 128 ] - xx [ 16 ] * xx [ 6 ] ) * xx [ 2 ] ) ; xx [ 131 ] = xx [ 2 ]
* xx [ 32 ] * xx [ 63 ] + ( xx [ 44 ] * xx [ 29 ] + xx [ 120 ] ) * xx [ 2 ] +
( xx [ 8 ] * xx [ 15 ] + xx [ 1 ] * xx [ 15 ] ) * xx [ 2 ] - ( xx [ 2 ] * xx
[ 24 ] * xx [ 57 ] + ( xx [ 16 ] * xx [ 41 ] + xx [ 129 ] ) * xx [ 2 ] + ( xx
[ 30 ] * xx [ 36 ] + xx [ 28 ] * xx [ 36 ] ) * xx [ 2 ] ) ; xx [ 132 ] = xx [
38 ] - ( xx [ 138 ] * xx [ 40 ] + xx [ 140 ] * xx [ 43 ] ) * xx [ 2 ] + xx [
73 ] * xx [ 82 ] + xx [ 84 ] + ( xx [ 134 ] - xx [ 123 ] * xx [ 81 ] ) * xx [
2 ] - ( xx [ 85 ] - ( xx [ 111 ] * xx [ 87 ] + xx [ 113 ] * xx [ 89 ] ) * xx
[ 2 ] + xx [ 67 ] * xx [ 94 ] + xx [ 21 ] + ( xx [ 143 ] - xx [ 91 ] * xx [
39 ] ) * xx [ 2 ] ) ; xx [ 133 ] = xx [ 2 ] * xx [ 112 ] * xx [ 73 ] + ( xx [
123 ] * xx [ 88 ] + xx [ 135 ] ) * xx [ 2 ] - ( xx [ 43 ] * xx [ 98 ] + xx [
40 ] * xx [ 98 ] ) * xx [ 2 ] - ( xx [ 2 ] * xx [ 101 ] * xx [ 67 ] + ( xx [
91 ] * xx [ 102 ] + xx [ 144 ] ) * xx [ 2 ] - ( xx [ 89 ] * xx [ 116 ] + xx [
87 ] * xx [ 116 ] ) * xx [ 2 ] ) ; memcpy ( xx + 219 , xx + 149 , 56 * sizeof
( double ) ) ; factorAndSolveWide ( 4 , 14 , xx + 219 , xx + 38 , xx + 99 ,
ii + 0 , xx + 130 , xx [ 5 ] , xx + 205 ) ; xx [ 0 ] = xx [ 57 ] + xx [ 209 ]
; xx [ 1 ] = xx [ 59 ] + xx [ 210 ] ; xx [ 5 ] = xx [ 63 ] + xx [ 212 ] ; xx
[ 6 ] = xx [ 65 ] + xx [ 213 ] ; xx [ 8 ] = xx [ 67 ] + xx [ 214 ] ; xx [ 11
] = xx [ 69 ] + xx [ 215 ] ; xx [ 20 ] = xx [ 73 ] + xx [ 217 ] ; xx [ 21 ] =
xx [ 75 ] + xx [ 218 ] ; xx [ 142 ] = xx [ 48 ] ; xx [ 143 ] = xx [ 49 ] ; xx
[ 144 ] = xx [ 50 ] ; xx [ 145 ] = xx [ 51 ] + xx [ 205 ] ; xx [ 146 ] = xx [
52 ] + xx [ 206 ] ; xx [ 147 ] = xx [ 53 ] + xx [ 207 ] ; xx [ 148 ] = xx [
54 ] ; xx [ 149 ] = xx [ 55 ] + xx [ 208 ] ; xx [ 150 ] = xx [ 56 ] ; xx [
151 ] = xx [ 0 ] ; xx [ 152 ] = xx [ 58 ] ; xx [ 153 ] = xx [ 1 ] ; xx [ 154
] = xx [ 60 ] ; xx [ 155 ] = xx [ 61 ] + xx [ 211 ] ; xx [ 156 ] = xx [ 62 ]
; xx [ 157 ] = xx [ 5 ] ; xx [ 158 ] = xx [ 64 ] ; xx [ 159 ] = xx [ 6 ] ; xx
[ 160 ] = xx [ 66 ] ; xx [ 161 ] = xx [ 8 ] ; xx [ 162 ] = xx [ 68 ] ; xx [
163 ] = xx [ 11 ] ; xx [ 164 ] = xx [ 70 ] ; xx [ 165 ] = xx [ 71 ] + xx [
216 ] ; xx [ 166 ] = xx [ 72 ] ; xx [ 167 ] = xx [ 20 ] ; xx [ 168 ] = xx [
74 ] ; xx [ 169 ] = xx [ 21 ] ; xx [ 170 ] = xx [ 76 ] ; xx [ 171 ] = xx [ 77
] ; xx [ 172 ] = xx [ 78 ] ; xx [ 173 ] = xx [ 79 ] ; xx [ 23 ] = ( xx [ 0 ]
+ xx [ 1 ] ) * xx [ 3 ] ; xx [ 24 ] = xx [ 31 ] * xx [ 23 ] ; xx [ 26 ] = xx
[ 33 ] * xx [ 23 ] ; xx [ 28 ] = xx [ 1 ] * xx [ 34 ] + xx [ 0 ] * xx [ 12 ]
; xx [ 12 ] = xx [ 28 ] * xx [ 14 ] ; xx [ 29 ] = xx [ 0 ] * xx [ 9 ] + xx [
1 ] * xx [ 9 ] ; xx [ 1 ] = xx [ 14 ] * xx [ 29 ] ; xx [ 9 ] = xx [ 12 ] + xx
[ 16 ] * xx [ 29 ] ; xx [ 38 ] = xx [ 12 ] ; xx [ 39 ] = xx [ 1 ] ; xx [ 40 ]
= - xx [ 9 ] ; pm_math_Vector3_cross_ra ( xx + 17 , xx + 38 , xx + 48 ) ; xx
[ 12 ] = ( xx [ 5 ] + xx [ 6 ] ) * xx [ 3 ] ; xx [ 14 ] = xx [ 90 ] * xx [ 12
] ; xx [ 17 ] = xx [ 92 ] * xx [ 12 ] ; xx [ 18 ] = xx [ 6 ] * xx [ 93 ] + xx
[ 5 ] * xx [ 37 ] ; xx [ 19 ] = xx [ 18 ] * xx [ 42 ] ; xx [ 29 ] = xx [ 5 ]
* xx [ 22 ] + xx [ 6 ] * xx [ 22 ] ; xx [ 6 ] = xx [ 42 ] * xx [ 29 ] ; xx [
22 ] = xx [ 19 ] + xx [ 44 ] * xx [ 29 ] ; xx [ 37 ] = xx [ 19 ] ; xx [ 38 ]
= xx [ 6 ] ; xx [ 39 ] = - xx [ 22 ] ; pm_math_Vector3_cross_ra ( xx + 45 ,
xx + 37 , xx + 40 ) ; xx [ 19 ] = ( xx [ 8 ] + xx [ 11 ] ) * xx [ 3 ] ; xx [
29 ] = xx [ 111 ] * xx [ 19 ] ; xx [ 30 ] = xx [ 113 ] * xx [ 19 ] ; xx [ 32
] = xx [ 11 ] * xx [ 114 ] + xx [ 8 ] * xx [ 83 ] ; xx [ 34 ] = xx [ 32 ] *
xx [ 86 ] ; xx [ 37 ] = xx [ 8 ] * xx [ 80 ] + xx [ 11 ] * xx [ 80 ] ; xx [
11 ] = xx [ 86 ] * xx [ 37 ] ; xx [ 38 ] = xx [ 34 ] + xx [ 91 ] * xx [ 37 ]
; xx [ 45 ] = - xx [ 34 ] ; xx [ 46 ] = - xx [ 11 ] ; xx [ 47 ] = xx [ 38 ] ;
pm_math_Vector3_cross_ra ( xx + 95 , xx + 45 , xx + 51 ) ; xx [ 34 ] = ( xx [
20 ] + xx [ 21 ] ) * xx [ 3 ] ; xx [ 3 ] = xx [ 138 ] * xx [ 34 ] ; xx [ 37 ]
= xx [ 140 ] * xx [ 34 ] ; xx [ 39 ] = xx [ 21 ] * xx [ 141 ] + xx [ 20 ] *
xx [ 117 ] ; xx [ 40 ] = xx [ 39 ] * xx [ 122 ] ; xx [ 43 ] = xx [ 20 ] * xx
[ 103 ] + xx [ 21 ] * xx [ 103 ] ; xx [ 21 ] = xx [ 122 ] * xx [ 43 ] ; xx [
45 ] = xx [ 40 ] + xx [ 123 ] * xx [ 43 ] ; xx [ 46 ] = - xx [ 40 ] ; xx [ 47
] = - xx [ 21 ] ; xx [ 48 ] = xx [ 45 ] ; pm_math_Vector3_cross_ra ( xx + 124
, xx + 46 , xx + 54 ) ; xx [ 57 ] = fabs ( xx [ 23 ] - ( xx [ 31 ] * xx [ 24
] + xx [ 33 ] * xx [ 26 ] ) * xx [ 2 ] + xx [ 0 ] * xx [ 35 ] + xx [ 28 ] + (
xx [ 49 ] - xx [ 16 ] * xx [ 1 ] ) * xx [ 2 ] - ( xx [ 12 ] - ( xx [ 90 ] *
xx [ 14 ] + xx [ 92 ] * xx [ 17 ] ) * xx [ 2 ] + xx [ 5 ] * xx [ 13 ] + xx [
18 ] + ( xx [ 41 ] - xx [ 44 ] * xx [ 6 ] ) * xx [ 2 ] ) ) ; xx [ 58 ] = fabs
( xx [ 0 ] * xx [ 27 ] + ( xx [ 16 ] * xx [ 9 ] + xx [ 50 ] ) * xx [ 2 ] + (
xx [ 26 ] * xx [ 36 ] + xx [ 24 ] * xx [ 36 ] ) * xx [ 2 ] - ( xx [ 5 ] * xx
[ 25 ] + ( xx [ 44 ] * xx [ 22 ] + xx [ 42 ] ) * xx [ 2 ] + ( xx [ 17 ] * xx
[ 15 ] + xx [ 14 ] * xx [ 15 ] ) * xx [ 2 ] ) ) ; xx [ 59 ] = fabs ( xx [ 19
] - ( xx [ 111 ] * xx [ 29 ] + xx [ 113 ] * xx [ 30 ] ) * xx [ 2 ] + xx [ 8 ]
* xx [ 94 ] + xx [ 32 ] + ( xx [ 52 ] - xx [ 91 ] * xx [ 11 ] ) * xx [ 2 ] -
( xx [ 34 ] - ( xx [ 138 ] * xx [ 3 ] + xx [ 140 ] * xx [ 37 ] ) * xx [ 2 ] +
xx [ 20 ] * xx [ 82 ] + xx [ 39 ] + ( xx [ 55 ] - xx [ 123 ] * xx [ 21 ] ) *
xx [ 2 ] ) ) ; xx [ 60 ] = fabs ( xx [ 8 ] * xx [ 108 ] + ( xx [ 91 ] * xx [
38 ] + xx [ 53 ] ) * xx [ 2 ] - ( xx [ 30 ] * xx [ 116 ] + xx [ 29 ] * xx [
116 ] ) * xx [ 2 ] - ( xx [ 20 ] * xx [ 105 ] + ( xx [ 123 ] * xx [ 45 ] + xx
[ 56 ] ) * xx [ 2 ] - ( xx [ 37 ] * xx [ 98 ] + xx [ 3 ] * xx [ 98 ] ) * xx [
2 ] ) ) ; ii [ 0 ] = 57 ; { int ll ; for ( ll = 58 ; ll < 61 ; ++ ll ) if (
xx [ ll ] > xx [ ii [ 0 ] ] ) ii [ 0 ] = ll ; } ii [ 0 ] -= 57 ; xx [ 0 ] =
xx [ 57 + ( ii [ 0 ] ) ] ; xx [ 1 ] = xx [ 0 ] - xx [ 7 ] ; if ( xx [ 1 ] <
0.0 ) ii [ 1 ] = - 1 ; else if ( xx [ 1 ] > 0.0 ) ii [ 1 ] = + 1 ; else ii [
1 ] = 0 ; ii [ 2 ] = ii [ 1 ] ; if ( 0 > ii [ 2 ] ) ii [ 2 ] = 0 ; if ( ii [
2 ] != 0 ) { switch ( ii [ 0 ] ) { case 0 : case 1 : { return
sm_ssci_recordRunTimeError (
"sm:compiler:messages:simulationErrors:ConstraintViolation" ,
 "'wheel_leg/blance_leg_model/Revolute Joint4' kinematic constraints cannot be maintained. Check solver type and consistency tolerance in the Simscape Solver Configuration block. Check Simulink solver type and tolerances in Model Configuration Parameters. A kinematic singularity might be the source of this problem."
, neDiagMgr ) ; } case 2 : case 3 : { return sm_ssci_recordRunTimeError (
"sm:compiler:messages:simulationErrors:ConstraintViolation" ,
 "'wheel_leg/blance_leg_model/Revolute Joint7' kinematic constraints cannot be maintained. Check solver type and consistency tolerance in the Simscape Solver Configuration block. Check Simulink solver type and tolerances in Model Configuration Parameters. A kinematic singularity might be the source of this problem."
, neDiagMgr ) ; } } } xx [ 0 ] = xx [ 4 ] * xx [ 158 ] ; xx [ 1 ] = cos ( xx
[ 0 ] ) ; xx [ 3 ] = xx [ 4 ] * xx [ 156 ] ; xx [ 5 ] = xx [ 10 ] * cos ( xx
[ 3 ] ) ; xx [ 6 ] = xx [ 10 ] * sin ( xx [ 3 ] ) ; xx [ 3 ] = sin ( xx [ 0 ]
) ; xx [ 0 ] = xx [ 1 ] * xx [ 5 ] - xx [ 6 ] * xx [ 3 ] ; xx [ 7 ] = xx [ 1
] * xx [ 6 ] ; xx [ 8 ] = xx [ 5 ] * xx [ 3 ] ; xx [ 11 ] = xx [ 0 ] ; xx [
12 ] = xx [ 7 ] + xx [ 8 ] ; xx [ 13 ] = xx [ 0 ] ; xx [ 14 ] = xx [ 8 ] + xx
[ 7 ] ; xx [ 0 ] = xx [ 4 ] * xx [ 152 ] ; xx [ 7 ] = cos ( xx [ 0 ] ) ; xx [
8 ] = xx [ 4 ] * xx [ 150 ] ; xx [ 9 ] = xx [ 10 ] * cos ( xx [ 8 ] ) ; xx [
15 ] = xx [ 10 ] * sin ( xx [ 8 ] ) ; xx [ 8 ] = sin ( xx [ 0 ] ) ; xx [ 0 ]
= xx [ 7 ] * xx [ 9 ] - xx [ 15 ] * xx [ 8 ] ; xx [ 16 ] = xx [ 7 ] * xx [ 15
] ; xx [ 7 ] = xx [ 9 ] * xx [ 8 ] ; xx [ 17 ] = xx [ 0 ] ; xx [ 18 ] = xx [
16 ] + xx [ 7 ] ; xx [ 19 ] = xx [ 0 ] ; xx [ 20 ] = xx [ 7 ] + xx [ 16 ] ;
pm_math_Quaternion_inverseCompose_ra ( xx + 11 , xx + 17 , xx + 21 ) ; xx [ 0
] = xx [ 149 ] * xx [ 5 ] ; xx [ 7 ] = xx [ 149 ] * xx [ 6 ] ; xx [ 8 ] = (
xx [ 5 ] * xx [ 0 ] + xx [ 6 ] * xx [ 7 ] ) * xx [ 2 ] ; xx [ 11 ] = xx [ 149
] - xx [ 8 ] ; xx [ 12 ] = xx [ 2 ] * ( xx [ 6 ] * xx [ 0 ] - xx [ 5 ] * xx [
7 ] ) ; xx [ 0 ] = xx [ 12 ] * xx [ 3 ] ; xx [ 5 ] = xx [ 3 ] * xx [ 11 ] ;
xx [ 16 ] = xx [ 11 ] + xx [ 2 ] * ( xx [ 1 ] * xx [ 0 ] - xx [ 5 ] * xx [ 3
] ) ; xx [ 17 ] = xx [ 12 ] - ( xx [ 1 ] * xx [ 5 ] + xx [ 0 ] * xx [ 3 ] ) *
xx [ 2 ] ; xx [ 18 ] = xx [ 8 ] + xx [ 157 ] + xx [ 159 ] ;
pm_math_Quaternion_inverseXform_ra ( xx + 21 , xx + 16 , xx + 5 ) ; xx [ 0 ]
= xx [ 4 ] * xx [ 166 ] ; xx [ 1 ] = xx [ 10 ] * sin ( xx [ 0 ] ) ; xx [ 3 ]
= xx [ 4 ] * xx [ 168 ] ; xx [ 5 ] = sin ( xx [ 3 ] ) ; xx [ 6 ] = cos ( xx [
3 ] ) ; xx [ 3 ] = xx [ 10 ] * cos ( xx [ 0 ] ) ; xx [ 0 ] = xx [ 1 ] * xx [
5 ] - xx [ 6 ] * xx [ 3 ] ; xx [ 8 ] = xx [ 6 ] * xx [ 1 ] ; xx [ 11 ] = xx [
3 ] * xx [ 5 ] ; xx [ 16 ] = xx [ 0 ] ; xx [ 17 ] = - ( xx [ 8 ] + xx [ 11 ]
) ; xx [ 18 ] = xx [ 0 ] ; xx [ 19 ] = - ( xx [ 11 ] + xx [ 8 ] ) ; xx [ 0 ]
= xx [ 4 ] * xx [ 160 ] ; xx [ 8 ] = xx [ 10 ] * sin ( xx [ 0 ] ) ; xx [ 11 ]
= xx [ 4 ] * xx [ 162 ] ; xx [ 4 ] = sin ( xx [ 11 ] ) ; xx [ 12 ] = cos ( xx
[ 11 ] ) ; xx [ 11 ] = xx [ 10 ] * cos ( xx [ 0 ] ) ; xx [ 0 ] = xx [ 8 ] *
xx [ 4 ] - xx [ 12 ] * xx [ 11 ] ; xx [ 10 ] = xx [ 12 ] * xx [ 8 ] ; xx [ 12
] = xx [ 11 ] * xx [ 4 ] ; xx [ 25 ] = xx [ 0 ] ; xx [ 26 ] = - ( xx [ 10 ] +
xx [ 12 ] ) ; xx [ 27 ] = xx [ 0 ] ; xx [ 28 ] = - ( xx [ 12 ] + xx [ 10 ] )
; pm_math_Quaternion_inverseCompose_ra ( xx + 16 , xx + 25 , xx + 29 ) ; xx [
0 ] = xx [ 149 ] * xx [ 3 ] ; xx [ 4 ] = xx [ 149 ] * xx [ 1 ] ; xx [ 10 ] =
( xx [ 3 ] * xx [ 0 ] + xx [ 1 ] * xx [ 4 ] ) * xx [ 2 ] ; xx [ 12 ] = xx [
149 ] - xx [ 10 ] ; xx [ 13 ] = xx [ 2 ] * ( xx [ 1 ] * xx [ 0 ] - xx [ 3 ] *
xx [ 4 ] ) ; xx [ 0 ] = xx [ 13 ] * xx [ 5 ] ; xx [ 1 ] = xx [ 5 ] * xx [ 12
] ; xx [ 16 ] = xx [ 12 ] + xx [ 2 ] * ( xx [ 6 ] * xx [ 0 ] - xx [ 1 ] * xx
[ 5 ] ) ; xx [ 17 ] = xx [ 13 ] - ( xx [ 6 ] * xx [ 1 ] + xx [ 0 ] * xx [ 5 ]
) * xx [ 2 ] ; xx [ 18 ] = xx [ 10 ] + xx [ 167 ] + xx [ 169 ] ;
pm_math_Quaternion_inverseXform_ra ( xx + 29 , xx + 16 , xx + 3 ) ; state [ 0
] = xx [ 142 ] ; state [ 1 ] = xx [ 143 ] ; state [ 2 ] = xx [ 144 ] ; state
[ 3 ] = xx [ 145 ] ; state [ 4 ] = xx [ 146 ] ; state [ 5 ] = xx [ 147 ] ;
state [ 6 ] = xx [ 148 ] ; state [ 7 ] = xx [ 149 ] ; state [ 8 ] = xx [ 150
] ; state [ 9 ] = xx [ 151 ] ; state [ 10 ] = xx [ 152 ] ; state [ 11 ] = xx
[ 153 ] ; state [ 12 ] = xx [ 154 ] ; state [ 13 ] = xx [ 155 ] ; state [ 14
] = xx [ 156 ] ; state [ 15 ] = xx [ 157 ] ; state [ 16 ] = xx [ 158 ] ;
state [ 17 ] = xx [ 159 ] ; state [ 18 ] = xx [ 160 ] ; state [ 19 ] = xx [
161 ] ; state [ 20 ] = xx [ 162 ] ; state [ 21 ] = xx [ 163 ] ; state [ 22 ]
= xx [ 164 ] ; state [ 23 ] = xx [ 165 ] ; state [ 24 ] = xx [ 166 ] ; state
[ 25 ] = xx [ 167 ] ; state [ 26 ] = xx [ 168 ] ; state [ 27 ] = xx [ 169 ] ;
state [ 28 ] = xx [ 170 ] + sm_core_canonicalAngle ( xx [ 2 ] * atan2 ( sqrt
( xx [ 22 ] * xx [ 22 ] + xx [ 23 ] * xx [ 23 ] + xx [ 24 ] * xx [ 24 ] ) ,
fabs ( xx [ 21 ] ) ) * ( ( xx [ 21 ] * xx [ 24 ] ) < 0.0 ? - 1.0 : + 1.0 ) -
xx [ 170 ] ) ; state [ 29 ] = ( xx [ 9 ] * xx [ 149 ] * xx [ 9 ] + xx [ 15 ]
* xx [ 149 ] * xx [ 15 ] ) * xx [ 2 ] + xx [ 151 ] + xx [ 153 ] - xx [ 7 ] ;
state [ 30 ] = xx [ 172 ] + sm_core_canonicalAngle ( xx [ 2 ] * atan2 ( sqrt
( xx [ 30 ] * xx [ 30 ] + xx [ 31 ] * xx [ 31 ] + xx [ 32 ] * xx [ 32 ] ) ,
fabs ( xx [ 29 ] ) ) * ( ( xx [ 29 ] * xx [ 32 ] ) < 0.0 ? - 1.0 : + 1.0 ) -
xx [ 172 ] ) ; state [ 31 ] = ( xx [ 11 ] * xx [ 149 ] * xx [ 11 ] + xx [ 8 ]
* xx [ 149 ] * xx [ 8 ] ) * xx [ 2 ] + xx [ 161 ] + xx [ 163 ] - xx [ 5 ] ;
return NULL ; } void wheel_leg_d9e82f16_1_computeConstraintError ( const void
* mech , const RuntimeDerivedValuesBundle * rtdv , const double * state ,
const int * modeVector , double * error ) { const double * rtdvd = rtdv ->
mDoubles . mValues ; const int * rtdvi = rtdv -> mInts . mValues ; double xx
[ 62 ] ; ( void ) mech ; ( void ) rtdvd ; ( void ) rtdvi ; ( void )
modeVector ; xx [ 0 ] = 0.2 ; xx [ 1 ] = 2.0 ; xx [ 2 ] = 0.125 ; xx [ 3 ] =
0.5 ; xx [ 4 ] = xx [ 3 ] * state [ 10 ] ; xx [ 5 ] = sin ( xx [ 4 ] ) ; xx [
6 ] = xx [ 2 ] * xx [ 5 ] ; xx [ 7 ] = xx [ 0 ] - xx [ 1 ] * xx [ 6 ] * xx [
5 ] ; xx [ 8 ] = 0.7071067811865476 ; xx [ 9 ] = xx [ 3 ] * state [ 8 ] ; xx
[ 10 ] = xx [ 8 ] * sin ( xx [ 9 ] ) ; xx [ 11 ] = xx [ 7 ] * xx [ 10 ] ; xx
[ 12 ] = xx [ 8 ] * cos ( xx [ 9 ] ) ; xx [ 9 ] = - xx [ 10 ] ; xx [ 13 ] =
xx [ 9 ] ; xx [ 14 ] = - xx [ 12 ] ; xx [ 15 ] = xx [ 9 ] ; xx [ 9 ] = cos (
xx [ 4 ] ) ; xx [ 4 ] = xx [ 1 ] * xx [ 9 ] * xx [ 6 ] ; xx [ 6 ] = xx [ 10 ]
* xx [ 4 ] ; xx [ 16 ] = xx [ 6 ] - xx [ 7 ] * xx [ 12 ] ; xx [ 17 ] = xx [ 6
] ; xx [ 18 ] = - xx [ 11 ] ; xx [ 19 ] = - xx [ 16 ] ;
pm_math_Vector3_cross_ra ( xx + 13 , xx + 17 , xx + 20 ) ; xx [ 6 ] = 0.075 ;
xx [ 7 ] = xx [ 6 ] * xx [ 10 ] ; xx [ 13 ] = xx [ 6 ] * xx [ 12 ] ; xx [ 14
] = xx [ 12 ] * xx [ 5 ] ; xx [ 15 ] = xx [ 9 ] * xx [ 10 ] ; xx [ 17 ] = (
xx [ 14 ] + xx [ 15 ] ) * xx [ 2 ] ; xx [ 18 ] = xx [ 9 ] * xx [ 12 ] - xx [
10 ] * xx [ 5 ] ; xx [ 5 ] = xx [ 15 ] + xx [ 14 ] ; xx [ 9 ] = xx [ 2 ] * xx
[ 18 ] ; xx [ 14 ] = xx [ 3 ] * state [ 16 ] ; xx [ 15 ] = sin ( xx [ 14 ] )
; xx [ 19 ] = xx [ 2 ] * xx [ 15 ] ; xx [ 23 ] = xx [ 0 ] - xx [ 1 ] * xx [
19 ] * xx [ 15 ] ; xx [ 24 ] = xx [ 3 ] * state [ 14 ] ; xx [ 25 ] = xx [ 8 ]
* sin ( xx [ 24 ] ) ; xx [ 26 ] = xx [ 23 ] * xx [ 25 ] ; xx [ 27 ] = xx [ 8
] * cos ( xx [ 24 ] ) ; xx [ 24 ] = - xx [ 25 ] ; xx [ 28 ] = xx [ 24 ] ; xx
[ 29 ] = - xx [ 27 ] ; xx [ 30 ] = xx [ 24 ] ; xx [ 24 ] = cos ( xx [ 14 ] )
; xx [ 14 ] = xx [ 1 ] * xx [ 24 ] * xx [ 19 ] ; xx [ 19 ] = xx [ 25 ] * xx [
14 ] ; xx [ 31 ] = xx [ 19 ] - xx [ 23 ] * xx [ 27 ] ; xx [ 32 ] = xx [ 19 ]
; xx [ 33 ] = - xx [ 26 ] ; xx [ 34 ] = - xx [ 31 ] ;
pm_math_Vector3_cross_ra ( xx + 28 , xx + 32 , xx + 35 ) ; xx [ 19 ] = xx [ 6
] * xx [ 25 ] ; xx [ 23 ] = xx [ 6 ] * xx [ 27 ] ; xx [ 28 ] = xx [ 27 ] * xx
[ 15 ] ; xx [ 29 ] = xx [ 24 ] * xx [ 25 ] ; xx [ 30 ] = ( xx [ 28 ] + xx [
29 ] ) * xx [ 2 ] ; xx [ 32 ] = xx [ 24 ] * xx [ 27 ] - xx [ 25 ] * xx [ 15 ]
; xx [ 15 ] = xx [ 29 ] + xx [ 28 ] ; xx [ 24 ] = xx [ 2 ] * xx [ 32 ] ; xx [
28 ] = 0.108 ; xx [ 20 ] = xx [ 3 ] * state [ 20 ] ; xx [ 29 ] = sin ( xx [
20 ] ) ; xx [ 33 ] = xx [ 2 ] * xx [ 29 ] ; xx [ 34 ] = xx [ 0 ] - xx [ 1 ] *
xx [ 33 ] * xx [ 29 ] ; xx [ 35 ] = xx [ 3 ] * state [ 18 ] ; xx [ 38 ] = xx
[ 8 ] * sin ( xx [ 35 ] ) ; xx [ 39 ] = xx [ 34 ] * xx [ 38 ] ; xx [ 40 ] =
xx [ 8 ] * cos ( xx [ 35 ] ) ; xx [ 41 ] = xx [ 38 ] ; xx [ 42 ] = xx [ 40 ]
; xx [ 43 ] = xx [ 38 ] ; xx [ 35 ] = cos ( xx [ 20 ] ) ; xx [ 20 ] = xx [ 1
] * xx [ 35 ] * xx [ 33 ] ; xx [ 33 ] = xx [ 38 ] * xx [ 20 ] ; xx [ 44 ] =
xx [ 34 ] * xx [ 40 ] - xx [ 33 ] ; xx [ 45 ] = - xx [ 33 ] ; xx [ 46 ] = xx
[ 39 ] ; xx [ 47 ] = - xx [ 44 ] ; pm_math_Vector3_cross_ra ( xx + 41 , xx +
45 , xx + 48 ) ; xx [ 33 ] = xx [ 6 ] * xx [ 38 ] ; xx [ 34 ] = xx [ 6 ] * xx
[ 40 ] ; xx [ 41 ] = xx [ 40 ] * xx [ 29 ] ; xx [ 42 ] = xx [ 35 ] * xx [ 38
] ; xx [ 43 ] = ( xx [ 41 ] + xx [ 42 ] ) * xx [ 2 ] ; xx [ 45 ] = xx [ 38 ]
* xx [ 29 ] - xx [ 35 ] * xx [ 40 ] ; xx [ 29 ] = xx [ 42 ] + xx [ 41 ] ; xx
[ 35 ] = xx [ 2 ] * xx [ 45 ] ; xx [ 41 ] = xx [ 3 ] * state [ 26 ] ; xx [ 42
] = sin ( xx [ 41 ] ) ; xx [ 46 ] = xx [ 2 ] * xx [ 42 ] ; xx [ 47 ] = xx [ 0
] - xx [ 1 ] * xx [ 46 ] * xx [ 42 ] ; xx [ 0 ] = xx [ 3 ] * state [ 24 ] ;
xx [ 3 ] = xx [ 8 ] * sin ( xx [ 0 ] ) ; xx [ 51 ] = xx [ 47 ] * xx [ 3 ] ;
xx [ 52 ] = xx [ 8 ] * cos ( xx [ 0 ] ) ; xx [ 53 ] = xx [ 3 ] ; xx [ 54 ] =
xx [ 52 ] ; xx [ 55 ] = xx [ 3 ] ; xx [ 0 ] = cos ( xx [ 41 ] ) ; xx [ 8 ] =
xx [ 1 ] * xx [ 0 ] * xx [ 46 ] ; xx [ 41 ] = xx [ 3 ] * xx [ 8 ] ; xx [ 46 ]
= xx [ 47 ] * xx [ 52 ] - xx [ 41 ] ; xx [ 56 ] = - xx [ 41 ] ; xx [ 57 ] =
xx [ 51 ] ; xx [ 58 ] = - xx [ 46 ] ; pm_math_Vector3_cross_ra ( xx + 53 , xx
+ 56 , xx + 59 ) ; xx [ 41 ] = xx [ 6 ] * xx [ 3 ] ; xx [ 47 ] = xx [ 6 ] *
xx [ 52 ] ; xx [ 6 ] = xx [ 52 ] * xx [ 42 ] ; xx [ 53 ] = xx [ 0 ] * xx [ 3
] ; xx [ 54 ] = ( xx [ 6 ] + xx [ 53 ] ) * xx [ 2 ] ; xx [ 55 ] = xx [ 3 ] *
xx [ 42 ] - xx [ 0 ] * xx [ 52 ] ; xx [ 0 ] = xx [ 53 ] + xx [ 6 ] ; xx [ 6 ]
= xx [ 2 ] * xx [ 55 ] ; error [ 0 ] = ( xx [ 11 ] * xx [ 12 ] + xx [ 21 ] )
* xx [ 1 ] + xx [ 4 ] + ( xx [ 7 ] * xx [ 12 ] + xx [ 13 ] * xx [ 10 ] ) * xx
[ 1 ] + ( xx [ 17 ] * xx [ 18 ] + xx [ 5 ] * xx [ 9 ] ) * xx [ 1 ] - ( ( xx [
26 ] * xx [ 27 ] + xx [ 36 ] ) * xx [ 1 ] + xx [ 14 ] + ( xx [ 19 ] * xx [ 27
] + xx [ 23 ] * xx [ 25 ] ) * xx [ 1 ] + ( xx [ 30 ] * xx [ 32 ] + xx [ 15 ]
* xx [ 24 ] ) * xx [ 1 ] ) + xx [ 28 ] ; error [ 1 ] = xx [ 1 ] * ( xx [ 5 ]
* xx [ 17 ] - xx [ 9 ] * xx [ 18 ] ) + xx [ 1 ] * ( xx [ 22 ] + xx [ 16 ] *
xx [ 12 ] ) - xx [ 1 ] * ( xx [ 13 ] * xx [ 12 ] - xx [ 7 ] * xx [ 10 ] ) - (
xx [ 1 ] * ( xx [ 15 ] * xx [ 30 ] - xx [ 24 ] * xx [ 32 ] ) + xx [ 1 ] * (
xx [ 37 ] + xx [ 31 ] * xx [ 27 ] ) - xx [ 1 ] * ( xx [ 23 ] * xx [ 27 ] - xx
[ 19 ] * xx [ 25 ] ) ) ; error [ 2 ] = ( xx [ 39 ] * xx [ 40 ] + xx [ 49 ] )
* xx [ 1 ] + xx [ 20 ] + ( xx [ 33 ] * xx [ 40 ] + xx [ 34 ] * xx [ 38 ] ) *
xx [ 1 ] - ( xx [ 43 ] * xx [ 45 ] + xx [ 29 ] * xx [ 35 ] ) * xx [ 1 ] - ( (
xx [ 51 ] * xx [ 52 ] + xx [ 60 ] ) * xx [ 1 ] + xx [ 8 ] + ( xx [ 41 ] * xx
[ 52 ] + xx [ 47 ] * xx [ 3 ] ) * xx [ 1 ] - ( xx [ 54 ] * xx [ 55 ] + xx [ 0
] * xx [ 6 ] ) * xx [ 1 ] ) + xx [ 28 ] ; error [ 3 ] = xx [ 1 ] * ( xx [ 29
] * xx [ 43 ] - xx [ 35 ] * xx [ 45 ] ) + xx [ 1 ] * ( xx [ 50 ] - xx [ 44 ]
* xx [ 40 ] ) - xx [ 1 ] * ( xx [ 34 ] * xx [ 40 ] - xx [ 33 ] * xx [ 38 ] )
- ( xx [ 1 ] * ( xx [ 0 ] * xx [ 54 ] - xx [ 6 ] * xx [ 55 ] ) + xx [ 1 ] * (
xx [ 61 ] - xx [ 46 ] * xx [ 52 ] ) - xx [ 1 ] * ( xx [ 47 ] * xx [ 52 ] - xx
[ 41 ] * xx [ 3 ] ) ) ; } void wheel_leg_d9e82f16_1_resetModeVector ( const
void * mech , int * modeVector ) { ( void ) mech ; ( void ) modeVector ; }
boolean_T wheel_leg_d9e82f16_1_hasJointDisToNormModeChange ( const void *
mech , const int * prevModeVector , const int * modeVector ) { ( void ) mech
; ( void ) prevModeVector ; ( void ) modeVector ; return 0 ; } PmfMessageId
wheel_leg_d9e82f16_1_performJointDisToNormModeChange ( const void * mech ,
const RuntimeDerivedValuesBundle * rtdv , const int * eqnEnableFlags , const
int * prevModeVector , const int * modeVector , const double * input , double
* state , void * neDiagMgr0 ) { const double * rtdvd = rtdv -> mDoubles .
mValues ; const int * rtdvi = rtdv -> mInts . mValues ; NeuDiagnosticManager
* neDiagMgr = ( NeuDiagnosticManager * ) neDiagMgr0 ; ( void ) mech ; ( void
) rtdvd ; ( void ) rtdvi ; ( void ) eqnEnableFlags ; ( void ) prevModeVector
; ( void ) modeVector ; ( void ) input ; ( void ) state ; ( void ) neDiagMgr
; return NULL ; } void wheel_leg_d9e82f16_1_onModeChangedCutJoints ( const
void * mech , const int * prevModeVector , const int * modeVector , double *
state ) { ( void ) mech ; ( void ) prevModeVector ; ( void ) modeVector ; (
void ) state ; }
