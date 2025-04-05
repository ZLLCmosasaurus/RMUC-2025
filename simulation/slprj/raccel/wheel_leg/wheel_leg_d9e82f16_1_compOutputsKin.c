#include <math.h>
#include <string.h>
#include "pm_std.h"
#include "sm_std.h"
#include "ne_std.h"
#include "ne_dae.h"
#include "sm_ssci_run_time_errors.h"
#include "sm_RuntimeDerivedValuesBundle.h"
#include "wheel_leg_d9e82f16_1_geometries.h"
PmfMessageId wheel_leg_d9e82f16_1_compOutputsKin ( const
RuntimeDerivedValuesBundle * rtdv , const double * state , const int *
modeVector , const double * input , const double * inputDot , const double *
inputDdot , const double * discreteState , double * output ,
NeuDiagnosticManager * neDiagMgr ) { const double * rtdvd = rtdv -> mDoubles
. mValues ; const int * rtdvi = rtdv -> mInts . mValues ; double xx [ 17 ] ;
( void ) rtdvd ; ( void ) rtdvi ; ( void ) modeVector ; ( void ) input ; (
void ) inputDot ; ( void ) inputDdot ; ( void ) discreteState ; ( void )
neDiagMgr ; xx [ 0 ] = 0.0 ; xx [ 1 ] = 0.7071067811865476 ; xx [ 2 ] = 0.5 *
state [ 6 ] ; xx [ 3 ] = xx [ 1 ] * cos ( xx [ 2 ] ) ; xx [ 4 ] = xx [ 1 ] *
xx [ 3 ] ; xx [ 5 ] = xx [ 1 ] * sin ( xx [ 2 ] ) ; xx [ 2 ] = xx [ 1 ] * xx
[ 5 ] ; xx [ 6 ] = xx [ 4 ] + xx [ 4 ] ; xx [ 7 ] = xx [ 2 ] + xx [ 2 ] ; xx
[ 8 ] = xx [ 4 ] - xx [ 4 ] ; xx [ 9 ] = - ( xx [ 2 ] - xx [ 2 ] ) ; xx [ 1 ]
= - xx [ 5 ] ; xx [ 10 ] = - xx [ 3 ] ; xx [ 11 ] = xx [ 1 ] ; xx [ 12 ] = xx
[ 3 ] ; xx [ 13 ] = xx [ 1 ] ; xx [ 1 ] = state [ 3 ] ; xx [ 2 ] = state [ 4
] ; xx [ 3 ] = state [ 5 ] ; pm_math_Quaternion_inverseXform_ra ( xx + 10 ,
xx + 1 , xx + 14 ) ; xx [ 1 ] = xx [ 14 ] ; xx [ 2 ] = xx [ 15 ] +
0.03000000000000001 * state [ 7 ] ; xx [ 3 ] = xx [ 16 ] ;
pm_math_Quaternion_xform_ra ( xx + 6 , xx + 1 , xx + 10 ) ; output [ 0 ] =
state [ 13 ] ; output [ 1 ] = state [ 23 ] ; output [ 2 ] = state [ 8 ] ;
output [ 3 ] = state [ 9 ] ; output [ 7 ] = state [ 14 ] ; output [ 8 ] =
state [ 15 ] ; output [ 12 ] = state [ 6 ] ; output [ 13 ] = state [ 7 ] ;
output [ 14 ] = state [ 18 ] ; output [ 15 ] = state [ 19 ] ; output [ 19 ] =
state [ 24 ] ; output [ 20 ] = state [ 25 ] ; output [ 24 ] = xx [ 11 ] ;
return NULL ; }
