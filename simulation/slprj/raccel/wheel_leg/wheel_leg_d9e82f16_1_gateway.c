#ifdef MATLAB_MEX_FILE
#include "tmwtypes.h"
#else
#include "rtwtypes.h"
#endif
#include "nesl_rtw.h"
#include "wheel_leg_d9e82f16_1.h"
#include "wheel_leg_d9e82f16_1_gateway.h"
void wheel_leg_d9e82f16_1_gateway ( void ) { NeModelParameters modelparams =
{ ( NeSolverType ) 0 , 0.001 , 1 , 0 , 0.001 , 0 , 0 , 0 , 0 , (
SscLoggingSetting ) 0 , 625702392 , } ; NeSolverParameters solverparams = { 0
, 0 , 1 , 0 , 0 , 0.001 , 1e-06 , 1e-09 , 0 , 0 , 100 , 0 , 1 , 0 , 1e-09 , 0
, ( NeLocalSolverChoice ) 0 , 0.001 , 0 , 3 , 2 , 0 , 2 , (
NeLinearAlgebraChoice ) 1 , ( NeEquationFormulationChoice ) 0 , 1024 , 1 ,
0.001 , ( NePartitionStorageMethod ) 0 , 1024 , ( NePartitionMethod ) 1 , } ;
const NeOutputParameters * outputparameters = NULL ; NeDae * dae ; size_t
numOutputs = 0 ; int * rtpDaes = NULL ; { static const NeOutputParameters
outputparameters_init [ ] = { { 0 , 0 , } , { 0 , 1 , } , } ;
outputparameters = outputparameters_init ; numOutputs = sizeof (
outputparameters_init ) / sizeof ( outputparameters_init [ 0 ] ) ; }
wheel_leg_d9e82f16_1_dae ( & dae , & modelparams , & solverparams ) ;
nesl_register_simulator_group (
"wheel_leg/blance_leg_model/Solver Configuration1_1" , 1 , & dae , &
solverparams , & modelparams , numOutputs , outputparameters , 0 , rtpDaes )
; }
