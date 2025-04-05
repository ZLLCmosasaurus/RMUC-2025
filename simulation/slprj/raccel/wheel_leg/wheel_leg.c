#include "rt_logging_mmi.h"
#include "wheel_leg_capi.h"
#include <math.h>
#include "wheel_leg.h"
#include "wheel_leg_private.h"
#include "wheel_leg_dt.h"
#include "simstruc.h"
#include "fixedpoint.h"
#include "slsa_sim_engine.h"
extern void * CreateDiagnosticAsVoidPtr_wrapper ( const char * id , int nargs
, ... ) ; RTWExtModeInfo * gblRTWExtModeInfo = NULL ; void
raccelForceExtModeShutdown ( boolean_T extModeStartPktReceived ) { if ( !
extModeStartPktReceived ) { boolean_T stopRequested = false ;
rtExtModeWaitForStartPkt ( gblRTWExtModeInfo , 2 , & stopRequested ) ; }
rtExtModeShutdown ( 2 ) ; }
#include "slsv_diagnostic_codegen_c_api.h"
const int_T gblNumToFiles = 0 ; const int_T gblNumFrFiles = 0 ; const int_T
gblNumFrWksBlocks = 0 ;
#ifdef RSIM_WITH_SOLVER_MULTITASKING
boolean_T gbl_raccel_isMultitasking = 1 ;
#else
boolean_T gbl_raccel_isMultitasking = 0 ;
#endif
boolean_T gbl_raccel_tid01eq = 0 ; int_T gbl_raccel_NumST = 3 ; const char_T
* gbl_raccel_Version = "9.4 (R2020b) 29-Jul-2020" ; void
raccel_setup_MMIStateLog ( SimStruct * S ) {
#ifdef UseMMIDataLogging
rt_FillStateSigInfoFromMMI ( ssGetRTWLogInfo ( S ) , & ssGetErrorStatus ( S )
) ;
#else
UNUSED_PARAMETER ( S ) ;
#endif
} static DataMapInfo rt_dataMapInfo ; DataMapInfo * rt_dataMapInfoPtr = &
rt_dataMapInfo ; rtwCAPI_ModelMappingInfo * rt_modelMapInfoPtr = & (
rt_dataMapInfo . mmi ) ; const int_T gblNumRootInportBlks = 0 ; const int_T
gblNumModelInputs = 0 ; extern const char * gblInportFileName ; extern
rtInportTUtable * gblInportTUtables ; const int_T gblInportDataTypeIdx [ ] =
{ - 1 } ; const int_T gblInportDims [ ] = { - 1 } ; const int_T
gblInportComplex [ ] = { - 1 } ; const int_T gblInportInterpoFlag [ ] = { - 1
} ; const int_T gblInportContinuous [ ] = { - 1 } ; int_T enableFcnCallFlag [
] = { 1 , 1 , 1 } ; const char * raccelLoadInputsAndAperiodicHitTimes (
SimStruct * S , const char * inportFileName , int * matFileFormat ) { return
rt_RAccelReadInportsMatFile ( S , inportFileName , matFileFormat ) ; } B rtB
; X rtX ; DW rtDW ; PrevZCX rtPrevZCX ; static SimStruct model_S ; SimStruct
* const rtS = & model_S ; void MdlInitialize ( void ) { char * tmp_e ; int_T
tmp_p ; boolean_T tmp ; tmp = false ; if ( tmp ) { tmp_p = strcmp (
"VariableStepAuto" , ssGetSolverName ( rtS ) ) ; if ( tmp_p != 0 ) { tmp_e =
solver_mismatch_message ( "VariableStepAuto" , ssGetSolverName ( rtS ) ) ;
ssSetErrorStatus ( rtS , tmp_e ) ; } } rtX . cbywjryyoi = rtP . Integrator_IC
; rtX . b2hgpxvjzc = rtP . PIDController3_InitialConditionForIntegrator ; rtX
. abxab3djbi = rtP . PIDController3_InitialConditionForFilter ; rtX .
p5p3ogk4mi = rtP . PIDController2_InitialConditionForFilter ; rtX .
cwaruwgj0n = rtP . PIDController2_InitialConditionForIntegrator ; rtX .
ecnkbzok1f = rtP . Integrator4_IC ; rtX . iko4pycqyt = rtP . Integrator2_IC ;
} void MdlStart ( void ) { CXPtMax * _rtXPerturbMax ; CXPtMin *
_rtXPerturbMin ; NeModelParameters modelParameters ; NeModelParameters
modelParameters_e ; NeModelParameters modelParameters_p ; NeslSimulationData
* simulationData ; NeslSimulator * tmp ; NeuDiagnosticManager *
diagnosticManager ; NeuDiagnosticTree * diagnosticTree ; char * msg ; real_T
tmp_m [ 24 ] ; real_T time ; real_T tmp_e ; int32_T tmp_i ; int_T tmp_g [ 7 ]
; boolean_T tmp_p ; { bool externalInputIsInDatasetFormat = false ; void *
pISigstreamManager = rt_GetISigstreamManager ( rtS ) ;
rtwISigstreamManagerGetInputIsInDatasetFormat ( pISigstreamManager , &
externalInputIsInDatasetFormat ) ; if ( externalInputIsInDatasetFormat ) { }
} _rtXPerturbMax = ( ( CXPtMax * ) ssGetJacobianPerturbationBoundsMaxVec (
rtS ) ) ; _rtXPerturbMin = ( ( CXPtMin * )
ssGetJacobianPerturbationBoundsMinVec ( rtS ) ) ; tmp = nesl_lease_simulator
( "wheel_leg/blance_leg_model/Solver Configuration1_1" , 0 , 0 ) ; rtDW .
eukrvu0tgn = ( void * ) tmp ; tmp_p = pointer_is_null ( rtDW . eukrvu0tgn ) ;
if ( tmp_p ) { wheel_leg_d9e82f16_1_gateway ( ) ; tmp = nesl_lease_simulator
( "wheel_leg/blance_leg_model/Solver Configuration1_1" , 0 , 0 ) ; rtDW .
eukrvu0tgn = ( void * ) tmp ; } simulationData = nesl_create_simulation_data
( ) ; rtDW . gahukgbj3h = ( void * ) simulationData ; diagnosticManager =
rtw_create_diagnostics ( ) ; rtDW . bq244s1auv = ( void * ) diagnosticManager
; modelParameters . mSolverType = NE_SOLVER_TYPE_DAE ; modelParameters .
mSolverTolerance = 0.001 ; modelParameters . mVariableStepSolver = true ;
modelParameters . mIsUsingODEN = false ; modelParameters . mFixedStepSize =
0.001 ; modelParameters . mStartTime = 0.0 ; modelParameters .
mLoadInitialState = false ; modelParameters . mUseSimState = false ;
modelParameters . mLinTrimCompile = false ; modelParameters . mLoggingMode =
SSC_LOGGING_NONE ; modelParameters . mRTWModifiedTimeStamp = 6.25702392E+8 ;
tmp_e = 0.001 ; modelParameters . mSolverTolerance = tmp_e ; tmp_e = 0.0 ;
modelParameters . mFixedStepSize = tmp_e ; tmp_p = true ; modelParameters .
mVariableStepSolver = tmp_p ; tmp_p = false ; modelParameters . mIsUsingODEN
= tmp_p ; tmp_p = ssGetGlobalInitialStatesAvailable ( rtS ) ; modelParameters
. mLoadInitialState = ( tmp_p && ssIsFirstInitCond ( rtS ) ) ;
diagnosticManager = ( NeuDiagnosticManager * ) rtDW . bq244s1auv ;
diagnosticTree = neu_diagnostic_manager_get_initial_tree ( diagnosticManager
) ; tmp_i = nesl_initialize_simulator ( ( NeslSimulator * ) rtDW . eukrvu0tgn
, & modelParameters , diagnosticManager ) ; if ( tmp_i != 0 ) { tmp_p =
error_buffer_is_empty ( ssGetErrorStatus ( rtS ) ) ; if ( tmp_p ) { msg =
rtw_diagnostics_msg ( diagnosticTree ) ; ssSetErrorStatus ( rtS , msg ) ; } }
simulationData = ( NeslSimulationData * ) rtDW . gahukgbj3h ; time = ssGetT (
rtS ) ; simulationData -> mData -> mTime . mN = 1 ; simulationData -> mData
-> mTime . mX = & time ; simulationData -> mData -> mContStates . mN = 32 ;
simulationData -> mData -> mContStates . mX = & rtX . mefcq5joqv [ 0 ] ;
simulationData -> mData -> mDiscStates . mN = 0 ; simulationData -> mData ->
mDiscStates . mX = & rtDW . ooldbvhge3 ; simulationData -> mData ->
mModeVector . mN = 0 ; simulationData -> mData -> mModeVector . mX = & rtDW .
nzztmagq15 ; tmp_p = ( ssIsMajorTimeStep ( rtS ) && ssGetRTWSolverInfo ( rtS
) -> foundContZcEvents ) ; simulationData -> mData -> mFoundZcEvents = tmp_p
; simulationData -> mData -> mIsMajorTimeStep = ssIsMajorTimeStep ( rtS ) ;
tmp_p = ( ssGetMdlInfoPtr ( rtS ) -> mdlFlags . solverAssertCheck == 1U ) ;
simulationData -> mData -> mIsSolverAssertCheck = tmp_p ; tmp_p =
ssIsSolverCheckingCIC ( rtS ) ; simulationData -> mData ->
mIsSolverCheckingCIC = tmp_p ; tmp_p = ssIsSolverComputingJacobian ( rtS ) ;
simulationData -> mData -> mIsComputingJacobian = tmp_p ; simulationData ->
mData -> mIsEvaluatingF0 = ( ssGetEvaluatingF0ForJacobian ( rtS ) != 0 ) ;
tmp_p = ssIsSolverRequestingReset ( rtS ) ; simulationData -> mData ->
mIsSolverRequestingReset = tmp_p ; tmp_g [ 0 ] = 0 ; tmp_m [ 0 ] = rtB .
nik4ggyist [ 0 ] ; tmp_m [ 1 ] = rtB . nik4ggyist [ 1 ] ; tmp_m [ 2 ] = rtB .
nik4ggyist [ 2 ] ; tmp_m [ 3 ] = rtB . nik4ggyist [ 3 ] ; tmp_g [ 1 ] = 4 ;
tmp_m [ 4 ] = rtB . ludhxwcsap [ 0 ] ; tmp_m [ 5 ] = rtB . ludhxwcsap [ 1 ] ;
tmp_m [ 6 ] = rtB . ludhxwcsap [ 2 ] ; tmp_m [ 7 ] = rtB . ludhxwcsap [ 3 ] ;
tmp_g [ 2 ] = 8 ; tmp_m [ 8 ] = rtB . jl2dsktpay [ 0 ] ; tmp_m [ 9 ] = rtB .
jl2dsktpay [ 1 ] ; tmp_m [ 10 ] = rtB . jl2dsktpay [ 2 ] ; tmp_m [ 11 ] = rtB
. jl2dsktpay [ 3 ] ; tmp_g [ 3 ] = 12 ; tmp_m [ 12 ] = rtB . clrzjaih1v [ 0 ]
; tmp_m [ 13 ] = rtB . clrzjaih1v [ 1 ] ; tmp_m [ 14 ] = rtB . clrzjaih1v [ 2
] ; tmp_m [ 15 ] = rtB . clrzjaih1v [ 3 ] ; tmp_g [ 4 ] = 16 ; tmp_m [ 16 ] =
rtB . dmfybrfnod [ 0 ] ; tmp_m [ 17 ] = rtB . dmfybrfnod [ 1 ] ; tmp_m [ 18 ]
= rtB . dmfybrfnod [ 2 ] ; tmp_m [ 19 ] = rtB . dmfybrfnod [ 3 ] ; tmp_g [ 5
] = 20 ; tmp_m [ 20 ] = rtB . l1345vcjcj [ 0 ] ; tmp_m [ 21 ] = rtB .
l1345vcjcj [ 1 ] ; tmp_m [ 22 ] = rtB . l1345vcjcj [ 2 ] ; tmp_m [ 23 ] = rtB
. l1345vcjcj [ 3 ] ; tmp_g [ 6 ] = 24 ; simulationData -> mData ->
mInputValues . mN = 24 ; simulationData -> mData -> mInputValues . mX = &
tmp_m [ 0 ] ; simulationData -> mData -> mInputOffsets . mN = 7 ;
simulationData -> mData -> mInputOffsets . mX = & tmp_g [ 0 ] ;
simulationData -> mData -> mNumjacDxLo . mN = 32 ; simulationData -> mData ->
mNumjacDxLo . mX = & _rtXPerturbMin -> mefcq5joqv [ 0 ] ; simulationData ->
mData -> mNumjacDxHi . mN = 32 ; simulationData -> mData -> mNumjacDxHi . mX
= & _rtXPerturbMax -> mefcq5joqv [ 0 ] ; diagnosticManager = (
NeuDiagnosticManager * ) rtDW . bq244s1auv ; diagnosticTree =
neu_diagnostic_manager_get_initial_tree ( diagnosticManager ) ; tmp_i =
ne_simulator_method ( ( NeslSimulator * ) rtDW . eukrvu0tgn ,
NESL_SIM_NUMJAC_DX_BOUNDS , simulationData , diagnosticManager ) ; if ( tmp_i
!= 0 ) { tmp_p = error_buffer_is_empty ( ssGetErrorStatus ( rtS ) ) ; if (
tmp_p ) { msg = rtw_diagnostics_msg ( diagnosticTree ) ; ssSetErrorStatus (
rtS , msg ) ; } } tmp = nesl_lease_simulator (
"wheel_leg/blance_leg_model/Solver Configuration1_1" , 1 , 0 ) ; rtDW .
go2zdudlka = ( void * ) tmp ; tmp_p = pointer_is_null ( rtDW . go2zdudlka ) ;
if ( tmp_p ) { wheel_leg_d9e82f16_1_gateway ( ) ; tmp = nesl_lease_simulator
( "wheel_leg/blance_leg_model/Solver Configuration1_1" , 1 , 0 ) ; rtDW .
go2zdudlka = ( void * ) tmp ; } simulationData = nesl_create_simulation_data
( ) ; rtDW . jg2ci40rqj = ( void * ) simulationData ; diagnosticManager =
rtw_create_diagnostics ( ) ; rtDW . bmoh4rr253 = ( void * ) diagnosticManager
; modelParameters_p . mSolverType = NE_SOLVER_TYPE_DAE ; modelParameters_p .
mSolverTolerance = 0.001 ; modelParameters_p . mVariableStepSolver = true ;
modelParameters_p . mIsUsingODEN = false ; modelParameters_p . mFixedStepSize
= 0.001 ; modelParameters_p . mStartTime = 0.0 ; modelParameters_p .
mLoadInitialState = false ; modelParameters_p . mUseSimState = false ;
modelParameters_p . mLinTrimCompile = false ; modelParameters_p .
mLoggingMode = SSC_LOGGING_NONE ; modelParameters_p . mRTWModifiedTimeStamp =
6.25702392E+8 ; tmp_e = 0.001 ; modelParameters_p . mSolverTolerance = tmp_e
; tmp_e = 0.0 ; modelParameters_p . mFixedStepSize = tmp_e ; tmp_p = true ;
modelParameters_p . mVariableStepSolver = tmp_p ; tmp_p = false ;
modelParameters_p . mIsUsingODEN = tmp_p ; tmp_p =
ssGetGlobalInitialStatesAvailable ( rtS ) ; modelParameters_p .
mLoadInitialState = ( tmp_p && ssIsFirstInitCond ( rtS ) ) ;
diagnosticManager = ( NeuDiagnosticManager * ) rtDW . bmoh4rr253 ;
diagnosticTree = neu_diagnostic_manager_get_initial_tree ( diagnosticManager
) ; tmp_i = nesl_initialize_simulator ( ( NeslSimulator * ) rtDW . go2zdudlka
, & modelParameters_p , diagnosticManager ) ; if ( tmp_i != 0 ) { tmp_p =
error_buffer_is_empty ( ssGetErrorStatus ( rtS ) ) ; if ( tmp_p ) { msg =
rtw_diagnostics_msg ( diagnosticTree ) ; ssSetErrorStatus ( rtS , msg ) ; } }
tmp = nesl_lease_simulator (
"wheel_leg/blance_leg_model/Solver Configuration1_1" , 1 , 1 ) ; rtDW .
loahdpiobx = ( void * ) tmp ; tmp_p = pointer_is_null ( rtDW . loahdpiobx ) ;
if ( tmp_p ) { wheel_leg_d9e82f16_1_gateway ( ) ; tmp = nesl_lease_simulator
( "wheel_leg/blance_leg_model/Solver Configuration1_1" , 1 , 1 ) ; rtDW .
loahdpiobx = ( void * ) tmp ; } simulationData = nesl_create_simulation_data
( ) ; rtDW . hpnunkuqpe = ( void * ) simulationData ; diagnosticManager =
rtw_create_diagnostics ( ) ; rtDW . ojbc53o3xt = ( void * ) diagnosticManager
; modelParameters_e . mSolverType = NE_SOLVER_TYPE_DAE ; modelParameters_e .
mSolverTolerance = 0.001 ; modelParameters_e . mVariableStepSolver = true ;
modelParameters_e . mIsUsingODEN = false ; modelParameters_e . mFixedStepSize
= 0.001 ; modelParameters_e . mStartTime = 0.0 ; modelParameters_e .
mLoadInitialState = false ; modelParameters_e . mUseSimState = false ;
modelParameters_e . mLinTrimCompile = false ; modelParameters_e .
mLoggingMode = SSC_LOGGING_NONE ; modelParameters_e . mRTWModifiedTimeStamp =
6.25702392E+8 ; tmp_e = 0.001 ; modelParameters_e . mSolverTolerance = tmp_e
; tmp_e = 0.0 ; modelParameters_e . mFixedStepSize = tmp_e ; tmp_p = true ;
modelParameters_e . mVariableStepSolver = tmp_p ; tmp_p = false ;
modelParameters_e . mIsUsingODEN = tmp_p ; tmp_p =
ssGetGlobalInitialStatesAvailable ( rtS ) ; modelParameters_e .
mLoadInitialState = ( tmp_p && ssIsFirstInitCond ( rtS ) ) ;
diagnosticManager = ( NeuDiagnosticManager * ) rtDW . ojbc53o3xt ;
diagnosticTree = neu_diagnostic_manager_get_initial_tree ( diagnosticManager
) ; tmp_i = nesl_initialize_simulator ( ( NeslSimulator * ) rtDW . loahdpiobx
, & modelParameters_e , diagnosticManager ) ; if ( tmp_i != 0 ) { tmp_p =
error_buffer_is_empty ( ssGetErrorStatus ( rtS ) ) ; if ( tmp_p ) { msg =
rtw_diagnostics_msg ( diagnosticTree ) ; ssSetErrorStatus ( rtS , msg ) ; } }
rtB . bdqfv1d2cw = rtP . T1_Y0 ; rtB . lbhtwulbcm = rtP . T2_Y0 ;
MdlInitialize ( ) ; } void MdlOutputs ( int_T tid ) { NeslSimulationData *
simulationData ; NeuDiagnosticManager * diagnosticManager ; NeuDiagnosticTree
* diagnosticTree ; char * msg ; real_T tmp_g [ 56 ] ; real_T tmp_i [ 56 ] ;
real_T tmp_p [ 24 ] ; real_T ogp5aqtkms [ 13 ] ; real_T A0 ; real_T B0 ;
real_T XB ; real_T XD ; real_T YB ; real_T YC ; real_T YD ; real_T ab_a ;
real_T af_a ; real_T ag_a ; real_T bb_a ; real_T bf_a ; real_T bg_a ; real_T
bh_a ; real_T cc_a ; real_T ce_a ; real_T cg_a ; real_T ch_a ; real_T dc_a ;
real_T dg_a ; real_T ec_a ; real_T ef_a ; real_T fc_a ; real_T ff_a ; real_T
gc_a ; real_T gd_a ; real_T gg_a ; real_T gi_a ; real_T hc_a ; real_T hd_a ;
real_T hg_a ; real_T ic_a ; real_T ig_a ; real_T jb_a ; real_T jg_a ; real_T
kb_a ; real_T kd_a ; real_T ke_a ; real_T kfo1vde04o ; real_T kg_a ; real_T
kh_a ; real_T lBD ; real_T l_a ; real_T lc_a ; real_T ld_a ; real_T le_a ;
real_T lg_a ; real_T lh_a ; real_T mc_a ; real_T me_a ; real_T mg_a ; real_T
nb_a ; real_T nc_a ; real_T ne_a ; real_T nf_a ; real_T o_a ; real_T ob_a ;
real_T oc_a ; real_T oe_a ; real_T of_a ; real_T oh_a ; real_T p_a ; real_T
pb_a ; real_T pc_a ; real_T pe_a ; real_T pg_a ; real_T ph_a ; real_T q_a ;
real_T qb_a ; real_T qg_a ; real_T r_a ; real_T rb_a ; real_T rf_a ; real_T
rg_a ; real_T s_a ; real_T sc_a ; real_T se_a ; real_T sf_a ; real_T sg_a ;
real_T tc_a ; real_T td_a ; real_T te_a ; real_T tf_a ; real_T tg_a ; real_T
time ; real_T time_e ; real_T time_p ; real_T ub_a ; real_T ue_a ; real_T
uf_a ; real_T v_a ; real_T vb_a ; real_T ve_a ; real_T vf_a ; real_T w_a ;
real_T wb_a ; real_T wc_a ; real_T we_a ; real_T wg_a ; real_T xb_a ; real_T
xc_a ; real_T xg_a ; real_T xh_a ; real_T yb_a ; real_T yf_a ; int32_T i ;
int32_T i_p ; int_T tmp_j [ 8 ] ; int_T tmp_m [ 8 ] ; int_T tmp_e [ 7 ] ;
boolean_T tmp ; simulationData = ( NeslSimulationData * ) rtDW . gahukgbj3h ;
time = ssGetT ( rtS ) ; simulationData -> mData -> mTime . mN = 1 ;
simulationData -> mData -> mTime . mX = & time ; simulationData -> mData ->
mContStates . mN = 32 ; simulationData -> mData -> mContStates . mX = & rtX .
mefcq5joqv [ 0 ] ; simulationData -> mData -> mDiscStates . mN = 0 ;
simulationData -> mData -> mDiscStates . mX = & rtDW . ooldbvhge3 ;
simulationData -> mData -> mModeVector . mN = 0 ; simulationData -> mData ->
mModeVector . mX = & rtDW . nzztmagq15 ; tmp = ( ssIsMajorTimeStep ( rtS ) &&
ssGetRTWSolverInfo ( rtS ) -> foundContZcEvents ) ; simulationData -> mData
-> mFoundZcEvents = tmp ; simulationData -> mData -> mIsMajorTimeStep =
ssIsMajorTimeStep ( rtS ) ; tmp = ( ssGetMdlInfoPtr ( rtS ) -> mdlFlags .
solverAssertCheck == 1U ) ; simulationData -> mData -> mIsSolverAssertCheck =
tmp ; tmp = ssIsSolverCheckingCIC ( rtS ) ; simulationData -> mData ->
mIsSolverCheckingCIC = tmp ; tmp = ssIsSolverComputingJacobian ( rtS ) ;
simulationData -> mData -> mIsComputingJacobian = tmp ; simulationData ->
mData -> mIsEvaluatingF0 = ( ssGetEvaluatingF0ForJacobian ( rtS ) != 0 ) ;
tmp = ssIsSolverRequestingReset ( rtS ) ; simulationData -> mData ->
mIsSolverRequestingReset = tmp ; tmp_e [ 0 ] = 0 ; tmp_p [ 0 ] = rtB .
nik4ggyist [ 0 ] ; tmp_p [ 1 ] = rtB . nik4ggyist [ 1 ] ; tmp_p [ 2 ] = rtB .
nik4ggyist [ 2 ] ; tmp_p [ 3 ] = rtB . nik4ggyist [ 3 ] ; tmp_e [ 1 ] = 4 ;
tmp_p [ 4 ] = rtB . ludhxwcsap [ 0 ] ; tmp_p [ 5 ] = rtB . ludhxwcsap [ 1 ] ;
tmp_p [ 6 ] = rtB . ludhxwcsap [ 2 ] ; tmp_p [ 7 ] = rtB . ludhxwcsap [ 3 ] ;
tmp_e [ 2 ] = 8 ; tmp_p [ 8 ] = rtB . jl2dsktpay [ 0 ] ; tmp_p [ 9 ] = rtB .
jl2dsktpay [ 1 ] ; tmp_p [ 10 ] = rtB . jl2dsktpay [ 2 ] ; tmp_p [ 11 ] = rtB
. jl2dsktpay [ 3 ] ; tmp_e [ 3 ] = 12 ; tmp_p [ 12 ] = rtB . clrzjaih1v [ 0 ]
; tmp_p [ 13 ] = rtB . clrzjaih1v [ 1 ] ; tmp_p [ 14 ] = rtB . clrzjaih1v [ 2
] ; tmp_p [ 15 ] = rtB . clrzjaih1v [ 3 ] ; tmp_e [ 4 ] = 16 ; tmp_p [ 16 ] =
rtB . dmfybrfnod [ 0 ] ; tmp_p [ 17 ] = rtB . dmfybrfnod [ 1 ] ; tmp_p [ 18 ]
= rtB . dmfybrfnod [ 2 ] ; tmp_p [ 19 ] = rtB . dmfybrfnod [ 3 ] ; tmp_e [ 5
] = 20 ; tmp_p [ 20 ] = rtB . l1345vcjcj [ 0 ] ; tmp_p [ 21 ] = rtB .
l1345vcjcj [ 1 ] ; tmp_p [ 22 ] = rtB . l1345vcjcj [ 2 ] ; tmp_p [ 23 ] = rtB
. l1345vcjcj [ 3 ] ; tmp_e [ 6 ] = 24 ; simulationData -> mData ->
mInputValues . mN = 24 ; simulationData -> mData -> mInputValues . mX = &
tmp_p [ 0 ] ; simulationData -> mData -> mInputOffsets . mN = 7 ;
simulationData -> mData -> mInputOffsets . mX = & tmp_e [ 0 ] ;
simulationData -> mData -> mOutputs . mN = 32 ; simulationData -> mData ->
mOutputs . mX = & rtB . eh2cmnfpcd [ 0 ] ; simulationData -> mData ->
mSampleHits . mN = 0 ; simulationData -> mData -> mSampleHits . mX = NULL ;
simulationData -> mData -> mIsFundamentalSampleHit = false ; simulationData
-> mData -> mTolerances . mN = 0 ; simulationData -> mData -> mTolerances .
mX = NULL ; simulationData -> mData -> mCstateHasChanged = false ;
diagnosticManager = ( NeuDiagnosticManager * ) rtDW . bq244s1auv ;
diagnosticTree = neu_diagnostic_manager_get_initial_tree ( diagnosticManager
) ; i = ne_simulator_method ( ( NeslSimulator * ) rtDW . eukrvu0tgn ,
NESL_SIM_OUTPUTS , simulationData , diagnosticManager ) ; if ( i != 0 ) { tmp
= error_buffer_is_empty ( ssGetErrorStatus ( rtS ) ) ; if ( tmp ) { msg =
rtw_diagnostics_msg ( diagnosticTree ) ; ssSetErrorStatus ( rtS , msg ) ; } }
if ( ssIsMajorTimeStep ( rtS ) && simulationData -> mData ->
mCstateHasChanged ) { ssSetBlockStateForSolverChangedAtMajorStep ( rtS ) ; }
simulationData = ( NeslSimulationData * ) rtDW . jg2ci40rqj ; time_p = ssGetT
( rtS ) ; simulationData -> mData -> mTime . mN = 1 ; simulationData -> mData
-> mTime . mX = & time_p ; simulationData -> mData -> mContStates . mN = 0 ;
simulationData -> mData -> mContStates . mX = NULL ; simulationData -> mData
-> mDiscStates . mN = 0 ; simulationData -> mData -> mDiscStates . mX = &
rtDW . cq3o2ec5fw ; simulationData -> mData -> mModeVector . mN = 0 ;
simulationData -> mData -> mModeVector . mX = & rtDW . d5bjwf4dqd ; tmp = (
ssIsMajorTimeStep ( rtS ) && ssGetRTWSolverInfo ( rtS ) -> foundContZcEvents
) ; simulationData -> mData -> mFoundZcEvents = tmp ; simulationData -> mData
-> mIsMajorTimeStep = ssIsMajorTimeStep ( rtS ) ; tmp = ( ssGetMdlInfoPtr (
rtS ) -> mdlFlags . solverAssertCheck == 1U ) ; simulationData -> mData ->
mIsSolverAssertCheck = tmp ; tmp = ssIsSolverCheckingCIC ( rtS ) ;
simulationData -> mData -> mIsSolverCheckingCIC = tmp ; simulationData ->
mData -> mIsComputingJacobian = false ; simulationData -> mData ->
mIsEvaluatingF0 = false ; tmp = ssIsSolverRequestingReset ( rtS ) ;
simulationData -> mData -> mIsSolverRequestingReset = tmp ; tmp_m [ 0 ] = 0 ;
tmp_i [ 0 ] = rtB . nik4ggyist [ 0 ] ; tmp_i [ 1 ] = rtB . nik4ggyist [ 1 ] ;
tmp_i [ 2 ] = rtB . nik4ggyist [ 2 ] ; tmp_i [ 3 ] = rtB . nik4ggyist [ 3 ] ;
tmp_m [ 1 ] = 4 ; tmp_i [ 4 ] = rtB . ludhxwcsap [ 0 ] ; tmp_i [ 5 ] = rtB .
ludhxwcsap [ 1 ] ; tmp_i [ 6 ] = rtB . ludhxwcsap [ 2 ] ; tmp_i [ 7 ] = rtB .
ludhxwcsap [ 3 ] ; tmp_m [ 2 ] = 8 ; tmp_i [ 8 ] = rtB . jl2dsktpay [ 0 ] ;
tmp_i [ 9 ] = rtB . jl2dsktpay [ 1 ] ; tmp_i [ 10 ] = rtB . jl2dsktpay [ 2 ]
; tmp_i [ 11 ] = rtB . jl2dsktpay [ 3 ] ; tmp_m [ 3 ] = 12 ; tmp_i [ 12 ] =
rtB . clrzjaih1v [ 0 ] ; tmp_i [ 13 ] = rtB . clrzjaih1v [ 1 ] ; tmp_i [ 14 ]
= rtB . clrzjaih1v [ 2 ] ; tmp_i [ 15 ] = rtB . clrzjaih1v [ 3 ] ; tmp_m [ 4
] = 16 ; tmp_i [ 16 ] = rtB . dmfybrfnod [ 0 ] ; tmp_i [ 17 ] = rtB .
dmfybrfnod [ 1 ] ; tmp_i [ 18 ] = rtB . dmfybrfnod [ 2 ] ; tmp_i [ 19 ] = rtB
. dmfybrfnod [ 3 ] ; tmp_m [ 5 ] = 20 ; tmp_i [ 20 ] = rtB . l1345vcjcj [ 0 ]
; tmp_i [ 21 ] = rtB . l1345vcjcj [ 1 ] ; tmp_i [ 22 ] = rtB . l1345vcjcj [ 2
] ; tmp_i [ 23 ] = rtB . l1345vcjcj [ 3 ] ; tmp_m [ 6 ] = 24 ; memcpy ( &
tmp_i [ 24 ] , & rtB . eh2cmnfpcd [ 0 ] , sizeof ( real_T ) << 5U ) ; tmp_m [
7 ] = 56 ; simulationData -> mData -> mInputValues . mN = 56 ; simulationData
-> mData -> mInputValues . mX = & tmp_i [ 0 ] ; simulationData -> mData ->
mInputOffsets . mN = 8 ; simulationData -> mData -> mInputOffsets . mX = &
tmp_m [ 0 ] ; simulationData -> mData -> mOutputs . mN = 13 ; simulationData
-> mData -> mOutputs . mX = & rtB . ne53mrvzfq [ 0 ] ; simulationData ->
mData -> mSampleHits . mN = 0 ; simulationData -> mData -> mSampleHits . mX =
NULL ; simulationData -> mData -> mIsFundamentalSampleHit = false ;
simulationData -> mData -> mTolerances . mN = 0 ; simulationData -> mData ->
mTolerances . mX = NULL ; simulationData -> mData -> mCstateHasChanged =
false ; diagnosticManager = ( NeuDiagnosticManager * ) rtDW . bmoh4rr253 ;
diagnosticTree = neu_diagnostic_manager_get_initial_tree ( diagnosticManager
) ; i = ne_simulator_method ( ( NeslSimulator * ) rtDW . go2zdudlka ,
NESL_SIM_OUTPUTS , simulationData , diagnosticManager ) ; if ( i != 0 ) { tmp
= error_buffer_is_empty ( ssGetErrorStatus ( rtS ) ) ; if ( tmp ) { msg =
rtw_diagnostics_msg ( diagnosticTree ) ; ssSetErrorStatus ( rtS , msg ) ; } }
if ( ssIsMajorTimeStep ( rtS ) && simulationData -> mData ->
mCstateHasChanged ) { ssSetBlockStateForSolverChangedAtMajorStep ( rtS ) ; }
rtB . nwuie3h1zj = rtP . Gain26_Gain * rtB . ne53mrvzfq [ 7 ] ; rtB .
liiv3zehzs = rtB . ne53mrvzfq [ 8 ] + rtP . Constant12_Value ; rtB .
hkrrrfopbq = rtB . ne53mrvzfq [ 10 ] + rtP . Constant12_Value ; kfo1vde04o =
0.15 * muDoubleScalarSin ( rtB . liiv3zehzs ) - 0.15 * muDoubleScalarSin (
rtB . hkrrrfopbq ) ; YC = 0.15 * muDoubleScalarSin ( rtB . liiv3zehzs ) -
0.15 * muDoubleScalarSin ( rtB . hkrrrfopbq ) ; A0 = ( 0.108 - 0.15 *
muDoubleScalarCos ( rtB . liiv3zehzs ) ) + 0.15 * muDoubleScalarCos ( rtB .
hkrrrfopbq ) ; YC = ( ( YC * YC + A0 * A0 ) + 0.0625 ) - 0.0625 ; A0 = (
0.108 - 0.15 * muDoubleScalarCos ( rtB . liiv3zehzs ) ) + 0.15 *
muDoubleScalarCos ( rtB . hkrrrfopbq ) ; YD = 0.15 * muDoubleScalarSin ( rtB
. liiv3zehzs ) - 0.15 * muDoubleScalarSin ( rtB . hkrrrfopbq ) ; YB = ( 0.108
- 0.15 * muDoubleScalarCos ( rtB . liiv3zehzs ) ) + 0.15 * muDoubleScalarCos
( rtB . hkrrrfopbq ) ; XD = 0.15 * muDoubleScalarSin ( rtB . liiv3zehzs ) -
0.15 * muDoubleScalarSin ( rtB . hkrrrfopbq ) ; XB = ( 0.108 - 0.15 *
muDoubleScalarCos ( rtB . liiv3zehzs ) ) + 0.15 * muDoubleScalarCos ( rtB .
hkrrrfopbq ) ; B0 = 0.15 * muDoubleScalarSin ( rtB . liiv3zehzs ) - 0.15 *
muDoubleScalarSin ( rtB . hkrrrfopbq ) ; lBD = 0.15 * muDoubleScalarSin ( rtB
. liiv3zehzs ) - 0.15 * muDoubleScalarSin ( rtB . hkrrrfopbq ) ; l_a = (
0.108 - 0.15 * muDoubleScalarCos ( rtB . liiv3zehzs ) ) + 0.15 *
muDoubleScalarCos ( rtB . hkrrrfopbq ) ; lBD = ( ( lBD * lBD + l_a * l_a ) +
0.0625 ) - 0.0625 ; l_a = ( 0.108 - 0.15 * muDoubleScalarCos ( rtB .
liiv3zehzs ) ) + 0.15 * muDoubleScalarCos ( rtB . hkrrrfopbq ) ; o_a = 0.15 *
muDoubleScalarSin ( rtB . liiv3zehzs ) - 0.15 * muDoubleScalarSin ( rtB .
hkrrrfopbq ) ; p_a = ( 0.108 - 0.15 * muDoubleScalarCos ( rtB . liiv3zehzs )
) + 0.15 * muDoubleScalarCos ( rtB . hkrrrfopbq ) ; q_a = 0.15 *
muDoubleScalarSin ( rtB . liiv3zehzs ) - 0.15 * muDoubleScalarSin ( rtB .
hkrrrfopbq ) ; r_a = 0.15 * muDoubleScalarSin ( rtB . liiv3zehzs ) - 0.15 *
muDoubleScalarSin ( rtB . hkrrrfopbq ) ; s_a = ( 0.108 - 0.15 *
muDoubleScalarCos ( rtB . liiv3zehzs ) ) + 0.15 * muDoubleScalarCos ( rtB .
hkrrrfopbq ) ; r_a = ( ( r_a * r_a + s_a * s_a ) + 0.0625 ) - 0.0625 ; s_a =
( 0.108 - 0.15 * muDoubleScalarCos ( rtB . liiv3zehzs ) ) + 0.15 *
muDoubleScalarCos ( rtB . hkrrrfopbq ) ; v_a = 0.15 * muDoubleScalarSin ( rtB
. liiv3zehzs ) - 0.15 * muDoubleScalarSin ( rtB . hkrrrfopbq ) ; w_a = (
0.108 - 0.15 * muDoubleScalarCos ( rtB . liiv3zehzs ) ) + 0.15 *
muDoubleScalarCos ( rtB . hkrrrfopbq ) ; v_a = ( ( ( v_a * v_a + w_a * w_a )
+ 0.0625 ) - 0.0625 ) + ( ( 0.108 - 0.15 * muDoubleScalarCos ( rtB .
liiv3zehzs ) ) + 0.15 * muDoubleScalarCos ( rtB . hkrrrfopbq ) ) * 0.5 ; w_a
= 0.15 * muDoubleScalarSin ( rtB . liiv3zehzs ) - 0.15 * muDoubleScalarSin (
rtB . hkrrrfopbq ) ; ab_a = 0.15 * muDoubleScalarSin ( rtB . liiv3zehzs ) -
0.15 * muDoubleScalarSin ( rtB . hkrrrfopbq ) ; bb_a = ( 0.108 - 0.15 *
muDoubleScalarCos ( rtB . liiv3zehzs ) ) + 0.15 * muDoubleScalarCos ( rtB .
hkrrrfopbq ) ; ab_a = ( ( ab_a * ab_a + bb_a * bb_a ) + 0.0625 ) - 0.0625 ;
bb_a = ( 0.108 - 0.15 * muDoubleScalarCos ( rtB . liiv3zehzs ) ) + 0.15 *
muDoubleScalarCos ( rtB . hkrrrfopbq ) ; w_a = muDoubleScalarSqrt ( ( w_a *
w_a * 0.25 - ab_a * ab_a ) + bb_a * bb_a * 0.25 ) - ( 0.15 *
muDoubleScalarSin ( rtB . liiv3zehzs ) - 0.15 * muDoubleScalarSin ( rtB .
hkrrrfopbq ) ) * 0.5 ; ab_a = 0.15 * muDoubleScalarSin ( rtB . liiv3zehzs ) -
0.15 * muDoubleScalarSin ( rtB . hkrrrfopbq ) ; bb_a = ( 0.108 - 0.15 *
muDoubleScalarCos ( rtB . liiv3zehzs ) ) + 0.15 * muDoubleScalarCos ( rtB .
hkrrrfopbq ) ; ab_a = ( ( ( ab_a * ab_a + bb_a * bb_a ) + 0.0625 ) - 0.0625 )
+ ( ( 0.108 - 0.15 * muDoubleScalarCos ( rtB . liiv3zehzs ) ) + 0.15 *
muDoubleScalarCos ( rtB . hkrrrfopbq ) ) * 0.5 ; bb_a = 0.15 *
muDoubleScalarSin ( rtB . liiv3zehzs ) - 0.15 * muDoubleScalarSin ( rtB .
hkrrrfopbq ) ; jb_a = 0.15 * muDoubleScalarSin ( rtB . liiv3zehzs ) - 0.15 *
muDoubleScalarSin ( rtB . hkrrrfopbq ) ; kb_a = ( 0.108 - 0.15 *
muDoubleScalarCos ( rtB . liiv3zehzs ) ) + 0.15 * muDoubleScalarCos ( rtB .
hkrrrfopbq ) ; jb_a = ( ( jb_a * jb_a + kb_a * kb_a ) + 0.0625 ) - 0.0625 ;
kb_a = ( 0.108 - 0.15 * muDoubleScalarCos ( rtB . liiv3zehzs ) ) + 0.15 *
muDoubleScalarCos ( rtB . hkrrrfopbq ) ; nb_a = 0.15 * muDoubleScalarSin (
rtB . liiv3zehzs ) - 0.15 * muDoubleScalarSin ( rtB . hkrrrfopbq ) ; ob_a = (
0.108 - 0.15 * muDoubleScalarCos ( rtB . liiv3zehzs ) ) + 0.15 *
muDoubleScalarCos ( rtB . hkrrrfopbq ) ; pb_a = 0.15 * muDoubleScalarSin (
rtB . liiv3zehzs ) - 0.15 * muDoubleScalarSin ( rtB . hkrrrfopbq ) ; qb_a =
0.15 * muDoubleScalarSin ( rtB . liiv3zehzs ) - 0.15 * muDoubleScalarSin (
rtB . hkrrrfopbq ) ; rb_a = ( 0.108 - 0.15 * muDoubleScalarCos ( rtB .
liiv3zehzs ) ) + 0.15 * muDoubleScalarCos ( rtB . hkrrrfopbq ) ; qb_a = ( (
qb_a * qb_a + rb_a * rb_a ) + 0.0625 ) - 0.0625 ; rb_a = ( 0.108 - 0.15 *
muDoubleScalarCos ( rtB . liiv3zehzs ) ) + 0.15 * muDoubleScalarCos ( rtB .
hkrrrfopbq ) ; ub_a = 0.15 * muDoubleScalarSin ( rtB . liiv3zehzs ) - 0.15 *
muDoubleScalarSin ( rtB . hkrrrfopbq ) ; vb_a = ( 0.108 - 0.15 *
muDoubleScalarCos ( rtB . liiv3zehzs ) ) + 0.15 * muDoubleScalarCos ( rtB .
hkrrrfopbq ) ; wb_a = 0.15 * muDoubleScalarSin ( rtB . liiv3zehzs ) - 0.15 *
muDoubleScalarSin ( rtB . hkrrrfopbq ) ; xb_a = 0.15 * muDoubleScalarSin (
rtB . liiv3zehzs ) - 0.15 * muDoubleScalarSin ( rtB . hkrrrfopbq ) ; yb_a = (
0.108 - 0.15 * muDoubleScalarCos ( rtB . liiv3zehzs ) ) + 0.15 *
muDoubleScalarCos ( rtB . hkrrrfopbq ) ; xb_a = ( ( xb_a * xb_a + yb_a * yb_a
) + 0.0625 ) - 0.0625 ; yb_a = ( 0.108 - 0.15 * muDoubleScalarCos ( rtB .
liiv3zehzs ) ) + 0.15 * muDoubleScalarCos ( rtB . hkrrrfopbq ) ; cc_a = 0.15
* muDoubleScalarSin ( rtB . liiv3zehzs ) - 0.15 * muDoubleScalarSin ( rtB .
hkrrrfopbq ) ; dc_a = ( 0.108 - 0.15 * muDoubleScalarCos ( rtB . liiv3zehzs )
) + 0.15 * muDoubleScalarCos ( rtB . hkrrrfopbq ) ; ec_a = 0.15 *
muDoubleScalarSin ( rtB . liiv3zehzs ) - 0.15 * muDoubleScalarSin ( rtB .
hkrrrfopbq ) ; fc_a = ( 0.108 - 0.15 * muDoubleScalarCos ( rtB . liiv3zehzs )
) + 0.15 * muDoubleScalarCos ( rtB . hkrrrfopbq ) ; gc_a = 0.15 *
muDoubleScalarSin ( rtB . liiv3zehzs ) - 0.15 * muDoubleScalarSin ( rtB .
hkrrrfopbq ) ; hc_a = 0.15 * muDoubleScalarSin ( rtB . liiv3zehzs ) - 0.15 *
muDoubleScalarSin ( rtB . hkrrrfopbq ) ; ic_a = ( 0.108 - 0.15 *
muDoubleScalarCos ( rtB . liiv3zehzs ) ) + 0.15 * muDoubleScalarCos ( rtB .
hkrrrfopbq ) ; hc_a = ( ( hc_a * hc_a + ic_a * ic_a ) + 0.0625 ) - 0.0625 ;
ic_a = ( 0.108 - 0.15 * muDoubleScalarCos ( rtB . liiv3zehzs ) ) + 0.15 *
muDoubleScalarCos ( rtB . hkrrrfopbq ) ; lc_a = 0.15 * muDoubleScalarSin (
rtB . liiv3zehzs ) - 0.15 * muDoubleScalarSin ( rtB . hkrrrfopbq ) ; mc_a = (
0.108 - 0.15 * muDoubleScalarCos ( rtB . liiv3zehzs ) ) + 0.15 *
muDoubleScalarCos ( rtB . hkrrrfopbq ) ; nc_a = 0.15 * muDoubleScalarSin (
rtB . liiv3zehzs ) - 0.15 * muDoubleScalarSin ( rtB . hkrrrfopbq ) ; oc_a =
0.15 * muDoubleScalarSin ( rtB . liiv3zehzs ) - 0.15 * muDoubleScalarSin (
rtB . hkrrrfopbq ) ; pc_a = ( 0.108 - 0.15 * muDoubleScalarCos ( rtB .
liiv3zehzs ) ) + 0.15 * muDoubleScalarCos ( rtB . hkrrrfopbq ) ; oc_a = ( (
oc_a * oc_a + pc_a * pc_a ) + 0.0625 ) - 0.0625 ; pc_a = ( 0.108 - 0.15 *
muDoubleScalarCos ( rtB . liiv3zehzs ) ) + 0.15 * muDoubleScalarCos ( rtB .
hkrrrfopbq ) ; sc_a = 0.15 * muDoubleScalarSin ( rtB . liiv3zehzs ) - 0.15 *
muDoubleScalarSin ( rtB . hkrrrfopbq ) ; tc_a = ( 0.108 - 0.15 *
muDoubleScalarCos ( rtB . liiv3zehzs ) ) + 0.15 * muDoubleScalarCos ( rtB .
hkrrrfopbq ) ; sc_a = ( ( ( sc_a * sc_a + tc_a * tc_a ) + 0.0625 ) - 0.0625 )
+ ( ( 0.108 - 0.15 * muDoubleScalarCos ( rtB . liiv3zehzs ) ) + 0.15 *
muDoubleScalarCos ( rtB . hkrrrfopbq ) ) * 0.5 ; tc_a = 0.15 *
muDoubleScalarSin ( rtB . liiv3zehzs ) - 0.15 * muDoubleScalarSin ( rtB .
hkrrrfopbq ) ; wc_a = 0.15 * muDoubleScalarSin ( rtB . liiv3zehzs ) - 0.15 *
muDoubleScalarSin ( rtB . hkrrrfopbq ) ; xc_a = ( 0.108 - 0.15 *
muDoubleScalarCos ( rtB . liiv3zehzs ) ) + 0.15 * muDoubleScalarCos ( rtB .
hkrrrfopbq ) ; wc_a = ( ( wc_a * wc_a + xc_a * xc_a ) + 0.0625 ) - 0.0625 ;
xc_a = ( 0.108 - 0.15 * muDoubleScalarCos ( rtB . liiv3zehzs ) ) + 0.15 *
muDoubleScalarCos ( rtB . hkrrrfopbq ) ; tc_a = muDoubleScalarSqrt ( ( tc_a *
tc_a * 0.25 - wc_a * wc_a ) + xc_a * xc_a * 0.25 ) - ( 0.15 *
muDoubleScalarSin ( rtB . liiv3zehzs ) - 0.15 * muDoubleScalarSin ( rtB .
hkrrrfopbq ) ) * 0.5 ; wc_a = 0.15 * muDoubleScalarSin ( rtB . liiv3zehzs ) -
0.15 * muDoubleScalarSin ( rtB . hkrrrfopbq ) ; xc_a = ( 0.108 - 0.15 *
muDoubleScalarCos ( rtB . liiv3zehzs ) ) + 0.15 * muDoubleScalarCos ( rtB .
hkrrrfopbq ) ; wc_a = ( ( ( wc_a * wc_a + xc_a * xc_a ) + 0.0625 ) - 0.0625 )
+ ( ( 0.108 - 0.15 * muDoubleScalarCos ( rtB . liiv3zehzs ) ) + 0.15 *
muDoubleScalarCos ( rtB . hkrrrfopbq ) ) * 0.5 ; xc_a = 0.15 *
muDoubleScalarSin ( rtB . liiv3zehzs ) - 0.15 * muDoubleScalarSin ( rtB .
hkrrrfopbq ) ; gd_a = 0.15 * muDoubleScalarSin ( rtB . liiv3zehzs ) - 0.15 *
muDoubleScalarSin ( rtB . hkrrrfopbq ) ; hd_a = ( 0.108 - 0.15 *
muDoubleScalarCos ( rtB . liiv3zehzs ) ) + 0.15 * muDoubleScalarCos ( rtB .
hkrrrfopbq ) ; gd_a = ( ( gd_a * gd_a + hd_a * hd_a ) + 0.0625 ) - 0.0625 ;
hd_a = ( 0.108 - 0.15 * muDoubleScalarCos ( rtB . liiv3zehzs ) ) + 0.15 *
muDoubleScalarCos ( rtB . hkrrrfopbq ) ; kd_a = 0.15 * muDoubleScalarSin (
rtB . liiv3zehzs ) - 0.15 * muDoubleScalarSin ( rtB . hkrrrfopbq ) ; ld_a = (
0.108 - 0.15 * muDoubleScalarCos ( rtB . liiv3zehzs ) ) + 0.15 *
muDoubleScalarCos ( rtB . hkrrrfopbq ) ; xc_a = ( muDoubleScalarCos (
muDoubleScalarAtan ( ( muDoubleScalarSqrt ( ( xc_a * xc_a * 0.25 - gd_a *
gd_a ) + hd_a * hd_a * 0.25 ) - ( 0.15 * muDoubleScalarSin ( rtB . liiv3zehzs
) - 0.15 * muDoubleScalarSin ( rtB . hkrrrfopbq ) ) * 0.5 ) / ( ( ( ( kd_a *
kd_a + ld_a * ld_a ) + 0.0625 ) - 0.0625 ) + ( ( 0.108 - 0.15 *
muDoubleScalarCos ( rtB . liiv3zehzs ) ) + 0.15 * muDoubleScalarCos ( rtB .
hkrrrfopbq ) ) * 0.5 ) ) * 2.0 ) * 0.25 - 0.054 ) + 0.15 * muDoubleScalarCos
( rtB . liiv3zehzs ) ; gd_a = 0.15 * muDoubleScalarSin ( rtB . liiv3zehzs ) -
0.15 * muDoubleScalarSin ( rtB . hkrrrfopbq ) ; hd_a = 0.15 *
muDoubleScalarSin ( rtB . liiv3zehzs ) - 0.15 * muDoubleScalarSin ( rtB .
hkrrrfopbq ) ; kd_a = ( 0.108 - 0.15 * muDoubleScalarCos ( rtB . liiv3zehzs )
) + 0.15 * muDoubleScalarCos ( rtB . hkrrrfopbq ) ; hd_a = ( ( hd_a * hd_a +
kd_a * kd_a ) + 0.0625 ) - 0.0625 ; kd_a = ( 0.108 - 0.15 * muDoubleScalarCos
( rtB . liiv3zehzs ) ) + 0.15 * muDoubleScalarCos ( rtB . hkrrrfopbq ) ; ld_a
= 0.15 * muDoubleScalarSin ( rtB . liiv3zehzs ) - 0.15 * muDoubleScalarSin (
rtB . hkrrrfopbq ) ; td_a = ( 0.108 - 0.15 * muDoubleScalarCos ( rtB .
liiv3zehzs ) ) + 0.15 * muDoubleScalarCos ( rtB . hkrrrfopbq ) ; gd_a =
muDoubleScalarSin ( muDoubleScalarAtan ( ( muDoubleScalarSqrt ( ( gd_a * gd_a
* 0.25 - hd_a * hd_a ) + kd_a * kd_a * 0.25 ) - ( 0.15 * muDoubleScalarSin (
rtB . liiv3zehzs ) - 0.15 * muDoubleScalarSin ( rtB . hkrrrfopbq ) ) * 0.5 )
/ ( ( ( ( ld_a * ld_a + td_a * td_a ) + 0.0625 ) - 0.0625 ) + ( ( 0.108 -
0.15 * muDoubleScalarCos ( rtB . liiv3zehzs ) ) + 0.15 * muDoubleScalarCos (
rtB . hkrrrfopbq ) ) * 0.5 ) ) * 2.0 ) * 0.25 + 0.15 * muDoubleScalarSin (
rtB . liiv3zehzs ) ; hd_a = 0.15 * muDoubleScalarSin ( rtB . liiv3zehzs ) -
0.15 * muDoubleScalarSin ( rtB . hkrrrfopbq ) ; kd_a = 0.15 *
muDoubleScalarSin ( rtB . liiv3zehzs ) - 0.15 * muDoubleScalarSin ( rtB .
hkrrrfopbq ) ; ld_a = ( 0.108 - 0.15 * muDoubleScalarCos ( rtB . liiv3zehzs )
) + 0.15 * muDoubleScalarCos ( rtB . hkrrrfopbq ) ; kd_a = ( ( kd_a * kd_a +
ld_a * ld_a ) + 0.0625 ) - 0.0625 ; ld_a = ( 0.108 - 0.15 * muDoubleScalarCos
( rtB . liiv3zehzs ) ) + 0.15 * muDoubleScalarCos ( rtB . hkrrrfopbq ) ; td_a
= 0.15 * muDoubleScalarSin ( rtB . liiv3zehzs ) - 0.15 * muDoubleScalarSin (
rtB . hkrrrfopbq ) ; ce_a = ( 0.108 - 0.15 * muDoubleScalarCos ( rtB .
liiv3zehzs ) ) + 0.15 * muDoubleScalarCos ( rtB . hkrrrfopbq ) ; hd_a = (
muDoubleScalarCos ( muDoubleScalarAtan ( ( muDoubleScalarSqrt ( ( hd_a * hd_a
* 0.25 - kd_a * kd_a ) + ld_a * ld_a * 0.25 ) - ( 0.15 * muDoubleScalarSin (
rtB . liiv3zehzs ) - 0.15 * muDoubleScalarSin ( rtB . hkrrrfopbq ) ) * 0.5 )
/ ( ( ( ( td_a * td_a + ce_a * ce_a ) + 0.0625 ) - 0.0625 ) + ( ( 0.108 -
0.15 * muDoubleScalarCos ( rtB . liiv3zehzs ) ) + 0.15 * muDoubleScalarCos (
rtB . hkrrrfopbq ) ) * 0.5 ) ) * 2.0 ) * 0.25 - 0.054 ) + 0.15 *
muDoubleScalarCos ( rtB . liiv3zehzs ) ; kd_a = 0.15 * muDoubleScalarSin (
rtB . liiv3zehzs ) - 0.15 * muDoubleScalarSin ( rtB . hkrrrfopbq ) ; ld_a =
0.15 * muDoubleScalarSin ( rtB . liiv3zehzs ) - 0.15 * muDoubleScalarSin (
rtB . hkrrrfopbq ) ; td_a = ( 0.108 - 0.15 * muDoubleScalarCos ( rtB .
liiv3zehzs ) ) + 0.15 * muDoubleScalarCos ( rtB . hkrrrfopbq ) ; ld_a = ( (
ld_a * ld_a + td_a * td_a ) + 0.0625 ) - 0.0625 ; td_a = ( 0.108 - 0.15 *
muDoubleScalarCos ( rtB . liiv3zehzs ) ) + 0.15 * muDoubleScalarCos ( rtB .
hkrrrfopbq ) ; ce_a = 0.15 * muDoubleScalarSin ( rtB . liiv3zehzs ) - 0.15 *
muDoubleScalarSin ( rtB . hkrrrfopbq ) ; ke_a = ( 0.108 - 0.15 *
muDoubleScalarCos ( rtB . liiv3zehzs ) ) + 0.15 * muDoubleScalarCos ( rtB .
hkrrrfopbq ) ; le_a = 0.15 * muDoubleScalarSin ( rtB . liiv3zehzs ) - 0.15 *
muDoubleScalarSin ( rtB . hkrrrfopbq ) ; me_a = ( 0.108 - 0.15 *
muDoubleScalarCos ( rtB . liiv3zehzs ) ) + 0.15 * muDoubleScalarCos ( rtB .
hkrrrfopbq ) ; ne_a = 0.15 * muDoubleScalarSin ( rtB . liiv3zehzs ) - 0.15 *
muDoubleScalarSin ( rtB . hkrrrfopbq ) ; oe_a = 0.15 * muDoubleScalarSin (
rtB . liiv3zehzs ) - 0.15 * muDoubleScalarSin ( rtB . hkrrrfopbq ) ; pe_a = (
0.108 - 0.15 * muDoubleScalarCos ( rtB . liiv3zehzs ) ) + 0.15 *
muDoubleScalarCos ( rtB . hkrrrfopbq ) ; oe_a = ( ( oe_a * oe_a + pe_a * pe_a
) + 0.0625 ) - 0.0625 ; pe_a = ( 0.108 - 0.15 * muDoubleScalarCos ( rtB .
liiv3zehzs ) ) + 0.15 * muDoubleScalarCos ( rtB . hkrrrfopbq ) ; se_a = 0.15
* muDoubleScalarSin ( rtB . liiv3zehzs ) - 0.15 * muDoubleScalarSin ( rtB .
hkrrrfopbq ) ; te_a = ( 0.108 - 0.15 * muDoubleScalarCos ( rtB . liiv3zehzs )
) + 0.15 * muDoubleScalarCos ( rtB . hkrrrfopbq ) ; ue_a = 0.15 *
muDoubleScalarSin ( rtB . liiv3zehzs ) - 0.15 * muDoubleScalarSin ( rtB .
hkrrrfopbq ) ; ve_a = 0.15 * muDoubleScalarSin ( rtB . liiv3zehzs ) - 0.15 *
muDoubleScalarSin ( rtB . hkrrrfopbq ) ; we_a = ( 0.108 - 0.15 *
muDoubleScalarCos ( rtB . liiv3zehzs ) ) + 0.15 * muDoubleScalarCos ( rtB .
hkrrrfopbq ) ; ve_a = ( ( ve_a * ve_a + we_a * we_a ) + 0.0625 ) - 0.0625 ;
we_a = ( 0.108 - 0.15 * muDoubleScalarCos ( rtB . liiv3zehzs ) ) + 0.15 *
muDoubleScalarCos ( rtB . hkrrrfopbq ) ; af_a = 0.15 * muDoubleScalarSin (
rtB . liiv3zehzs ) - 0.15 * muDoubleScalarSin ( rtB . hkrrrfopbq ) ; bf_a = (
0.108 - 0.15 * muDoubleScalarCos ( rtB . liiv3zehzs ) ) + 0.15 *
muDoubleScalarCos ( rtB . hkrrrfopbq ) ; af_a = ( ( ( af_a * af_a + bf_a *
bf_a ) + 0.0625 ) - 0.0625 ) + ( ( 0.108 - 0.15 * muDoubleScalarCos ( rtB .
liiv3zehzs ) ) + 0.15 * muDoubleScalarCos ( rtB . hkrrrfopbq ) ) * 0.5 ; bf_a
= 0.15 * muDoubleScalarSin ( rtB . liiv3zehzs ) - 0.15 * muDoubleScalarSin (
rtB . hkrrrfopbq ) ; ef_a = 0.15 * muDoubleScalarSin ( rtB . liiv3zehzs ) -
0.15 * muDoubleScalarSin ( rtB . hkrrrfopbq ) ; ff_a = ( 0.108 - 0.15 *
muDoubleScalarCos ( rtB . liiv3zehzs ) ) + 0.15 * muDoubleScalarCos ( rtB .
hkrrrfopbq ) ; ef_a = ( ( ef_a * ef_a + ff_a * ff_a ) + 0.0625 ) - 0.0625 ;
ff_a = ( 0.108 - 0.15 * muDoubleScalarCos ( rtB . liiv3zehzs ) ) + 0.15 *
muDoubleScalarCos ( rtB . hkrrrfopbq ) ; bf_a = muDoubleScalarSqrt ( ( bf_a *
bf_a * 0.25 - ef_a * ef_a ) + ff_a * ff_a * 0.25 ) - ( 0.15 *
muDoubleScalarSin ( rtB . liiv3zehzs ) - 0.15 * muDoubleScalarSin ( rtB .
hkrrrfopbq ) ) * 0.5 ; ef_a = 0.15 * muDoubleScalarSin ( rtB . liiv3zehzs ) -
0.15 * muDoubleScalarSin ( rtB . hkrrrfopbq ) ; ff_a = ( 0.108 - 0.15 *
muDoubleScalarCos ( rtB . liiv3zehzs ) ) + 0.15 * muDoubleScalarCos ( rtB .
hkrrrfopbq ) ; ef_a = ( ( ( ef_a * ef_a + ff_a * ff_a ) + 0.0625 ) - 0.0625 )
+ ( ( 0.108 - 0.15 * muDoubleScalarCos ( rtB . liiv3zehzs ) ) + 0.15 *
muDoubleScalarCos ( rtB . hkrrrfopbq ) ) * 0.5 ; ff_a = 0.15 *
muDoubleScalarSin ( rtB . liiv3zehzs ) - 0.15 * muDoubleScalarSin ( rtB .
hkrrrfopbq ) ; nf_a = 0.15 * muDoubleScalarSin ( rtB . liiv3zehzs ) - 0.15 *
muDoubleScalarSin ( rtB . hkrrrfopbq ) ; of_a = ( 0.108 - 0.15 *
muDoubleScalarCos ( rtB . liiv3zehzs ) ) + 0.15 * muDoubleScalarCos ( rtB .
hkrrrfopbq ) ; nf_a = ( ( nf_a * nf_a + of_a * of_a ) + 0.0625 ) - 0.0625 ;
of_a = ( 0.108 - 0.15 * muDoubleScalarCos ( rtB . liiv3zehzs ) ) + 0.15 *
muDoubleScalarCos ( rtB . hkrrrfopbq ) ; rf_a = 0.15 * muDoubleScalarSin (
rtB . liiv3zehzs ) - 0.15 * muDoubleScalarSin ( rtB . hkrrrfopbq ) ; sf_a = (
0.108 - 0.15 * muDoubleScalarCos ( rtB . liiv3zehzs ) ) + 0.15 *
muDoubleScalarCos ( rtB . hkrrrfopbq ) ; tf_a = 0.15 * muDoubleScalarSin (
rtB . liiv3zehzs ) - 0.15 * muDoubleScalarSin ( rtB . hkrrrfopbq ) ; uf_a =
0.15 * muDoubleScalarSin ( rtB . liiv3zehzs ) - 0.15 * muDoubleScalarSin (
rtB . hkrrrfopbq ) ; vf_a = ( 0.108 - 0.15 * muDoubleScalarCos ( rtB .
liiv3zehzs ) ) + 0.15 * muDoubleScalarCos ( rtB . hkrrrfopbq ) ; uf_a = ( (
uf_a * uf_a + vf_a * vf_a ) + 0.0625 ) - 0.0625 ; vf_a = ( 0.108 - 0.15 *
muDoubleScalarCos ( rtB . liiv3zehzs ) ) + 0.15 * muDoubleScalarCos ( rtB .
hkrrrfopbq ) ; yf_a = 0.15 * muDoubleScalarSin ( rtB . liiv3zehzs ) - 0.15 *
muDoubleScalarSin ( rtB . hkrrrfopbq ) ; ag_a = ( 0.108 - 0.15 *
muDoubleScalarCos ( rtB . liiv3zehzs ) ) + 0.15 * muDoubleScalarCos ( rtB .
hkrrrfopbq ) ; bg_a = 0.15 * muDoubleScalarSin ( rtB . liiv3zehzs ) - 0.15 *
muDoubleScalarSin ( rtB . hkrrrfopbq ) ; cg_a = 0.15 * muDoubleScalarSin (
rtB . liiv3zehzs ) - 0.15 * muDoubleScalarSin ( rtB . hkrrrfopbq ) ; dg_a = (
0.108 - 0.15 * muDoubleScalarCos ( rtB . liiv3zehzs ) ) + 0.15 *
muDoubleScalarCos ( rtB . hkrrrfopbq ) ; cg_a = ( ( cg_a * cg_a + dg_a * dg_a
) + 0.0625 ) - 0.0625 ; dg_a = ( 0.108 - 0.15 * muDoubleScalarCos ( rtB .
liiv3zehzs ) ) + 0.15 * muDoubleScalarCos ( rtB . hkrrrfopbq ) ; gg_a = 0.15
* muDoubleScalarSin ( rtB . liiv3zehzs ) - 0.15 * muDoubleScalarSin ( rtB .
hkrrrfopbq ) ; hg_a = ( 0.108 - 0.15 * muDoubleScalarCos ( rtB . liiv3zehzs )
) + 0.15 * muDoubleScalarCos ( rtB . hkrrrfopbq ) ; ig_a = 0.15 *
muDoubleScalarSin ( rtB . liiv3zehzs ) - 0.15 * muDoubleScalarSin ( rtB .
hkrrrfopbq ) ; jg_a = ( 0.108 - 0.15 * muDoubleScalarCos ( rtB . liiv3zehzs )
) + 0.15 * muDoubleScalarCos ( rtB . hkrrrfopbq ) ; kg_a = 0.15 *
muDoubleScalarSin ( rtB . liiv3zehzs ) - 0.15 * muDoubleScalarSin ( rtB .
hkrrrfopbq ) ; lg_a = 0.15 * muDoubleScalarSin ( rtB . liiv3zehzs ) - 0.15 *
muDoubleScalarSin ( rtB . hkrrrfopbq ) ; mg_a = ( 0.108 - 0.15 *
muDoubleScalarCos ( rtB . liiv3zehzs ) ) + 0.15 * muDoubleScalarCos ( rtB .
hkrrrfopbq ) ; lg_a = ( ( lg_a * lg_a + mg_a * mg_a ) + 0.0625 ) - 0.0625 ;
mg_a = ( 0.108 - 0.15 * muDoubleScalarCos ( rtB . liiv3zehzs ) ) + 0.15 *
muDoubleScalarCos ( rtB . hkrrrfopbq ) ; pg_a = 0.15 * muDoubleScalarSin (
rtB . liiv3zehzs ) - 0.15 * muDoubleScalarSin ( rtB . hkrrrfopbq ) ; qg_a = (
0.108 - 0.15 * muDoubleScalarCos ( rtB . liiv3zehzs ) ) + 0.15 *
muDoubleScalarCos ( rtB . hkrrrfopbq ) ; rg_a = 0.15 * muDoubleScalarSin (
rtB . liiv3zehzs ) - 0.15 * muDoubleScalarSin ( rtB . hkrrrfopbq ) ; sg_a =
0.15 * muDoubleScalarSin ( rtB . liiv3zehzs ) - 0.15 * muDoubleScalarSin (
rtB . hkrrrfopbq ) ; tg_a = ( 0.108 - 0.15 * muDoubleScalarCos ( rtB .
liiv3zehzs ) ) + 0.15 * muDoubleScalarCos ( rtB . hkrrrfopbq ) ; sg_a = ( (
sg_a * sg_a + tg_a * tg_a ) + 0.0625 ) - 0.0625 ; tg_a = ( 0.108 - 0.15 *
muDoubleScalarCos ( rtB . liiv3zehzs ) ) + 0.15 * muDoubleScalarCos ( rtB .
hkrrrfopbq ) ; wg_a = 0.15 * muDoubleScalarSin ( rtB . liiv3zehzs ) - 0.15 *
muDoubleScalarSin ( rtB . hkrrrfopbq ) ; xg_a = ( 0.108 - 0.15 *
muDoubleScalarCos ( rtB . liiv3zehzs ) ) + 0.15 * muDoubleScalarCos ( rtB .
hkrrrfopbq ) ; wg_a = ( ( ( wg_a * wg_a + xg_a * xg_a ) + 0.0625 ) - 0.0625 )
+ ( ( 0.108 - 0.15 * muDoubleScalarCos ( rtB . liiv3zehzs ) ) + 0.15 *
muDoubleScalarCos ( rtB . hkrrrfopbq ) ) * 0.5 ; xg_a = 0.15 *
muDoubleScalarSin ( rtB . liiv3zehzs ) - 0.15 * muDoubleScalarSin ( rtB .
hkrrrfopbq ) ; bh_a = 0.15 * muDoubleScalarSin ( rtB . liiv3zehzs ) - 0.15 *
muDoubleScalarSin ( rtB . hkrrrfopbq ) ; ch_a = ( 0.108 - 0.15 *
muDoubleScalarCos ( rtB . liiv3zehzs ) ) + 0.15 * muDoubleScalarCos ( rtB .
hkrrrfopbq ) ; bh_a = ( ( bh_a * bh_a + ch_a * ch_a ) + 0.0625 ) - 0.0625 ;
ch_a = ( 0.108 - 0.15 * muDoubleScalarCos ( rtB . liiv3zehzs ) ) + 0.15 *
muDoubleScalarCos ( rtB . hkrrrfopbq ) ; xg_a = muDoubleScalarSqrt ( ( xg_a *
xg_a * 0.25 - bh_a * bh_a ) + ch_a * ch_a * 0.25 ) - ( 0.15 *
muDoubleScalarSin ( rtB . liiv3zehzs ) - 0.15 * muDoubleScalarSin ( rtB .
hkrrrfopbq ) ) * 0.5 ; bh_a = 0.15 * muDoubleScalarSin ( rtB . liiv3zehzs ) -
0.15 * muDoubleScalarSin ( rtB . hkrrrfopbq ) ; ch_a = ( 0.108 - 0.15 *
muDoubleScalarCos ( rtB . liiv3zehzs ) ) + 0.15 * muDoubleScalarCos ( rtB .
hkrrrfopbq ) ; bh_a = ( ( ( bh_a * bh_a + ch_a * ch_a ) + 0.0625 ) - 0.0625 )
+ ( ( 0.108 - 0.15 * muDoubleScalarCos ( rtB . liiv3zehzs ) ) + 0.15 *
muDoubleScalarCos ( rtB . hkrrrfopbq ) ) * 0.5 ; ch_a = 0.15 *
muDoubleScalarSin ( rtB . liiv3zehzs ) - 0.15 * muDoubleScalarSin ( rtB .
hkrrrfopbq ) ; kh_a = 0.15 * muDoubleScalarSin ( rtB . liiv3zehzs ) - 0.15 *
muDoubleScalarSin ( rtB . hkrrrfopbq ) ; lh_a = ( 0.108 - 0.15 *
muDoubleScalarCos ( rtB . liiv3zehzs ) ) + 0.15 * muDoubleScalarCos ( rtB .
hkrrrfopbq ) ; kh_a = ( ( kh_a * kh_a + lh_a * lh_a ) + 0.0625 ) - 0.0625 ;
lh_a = ( 0.108 - 0.15 * muDoubleScalarCos ( rtB . liiv3zehzs ) ) + 0.15 *
muDoubleScalarCos ( rtB . hkrrrfopbq ) ; oh_a = 0.15 * muDoubleScalarSin (
rtB . liiv3zehzs ) - 0.15 * muDoubleScalarSin ( rtB . hkrrrfopbq ) ; ph_a = (
0.108 - 0.15 * muDoubleScalarCos ( rtB . liiv3zehzs ) ) + 0.15 *
muDoubleScalarCos ( rtB . hkrrrfopbq ) ; ch_a = ( muDoubleScalarCos (
muDoubleScalarAtan ( ( muDoubleScalarSqrt ( ( ch_a * ch_a * 0.25 - kh_a *
kh_a ) + lh_a * lh_a * 0.25 ) - ( 0.15 * muDoubleScalarSin ( rtB . liiv3zehzs
) - 0.15 * muDoubleScalarSin ( rtB . hkrrrfopbq ) ) * 0.5 ) / ( ( ( ( oh_a *
oh_a + ph_a * ph_a ) + 0.0625 ) - 0.0625 ) + ( ( 0.108 - 0.15 *
muDoubleScalarCos ( rtB . liiv3zehzs ) ) + 0.15 * muDoubleScalarCos ( rtB .
hkrrrfopbq ) ) * 0.5 ) ) * 2.0 ) * 0.25 - 0.054 ) + 0.15 * muDoubleScalarCos
( rtB . liiv3zehzs ) ; kh_a = 0.15 * muDoubleScalarSin ( rtB . liiv3zehzs ) -
0.15 * muDoubleScalarSin ( rtB . hkrrrfopbq ) ; lh_a = 0.15 *
muDoubleScalarSin ( rtB . liiv3zehzs ) - 0.15 * muDoubleScalarSin ( rtB .
hkrrrfopbq ) ; oh_a = ( 0.108 - 0.15 * muDoubleScalarCos ( rtB . liiv3zehzs )
) + 0.15 * muDoubleScalarCos ( rtB . hkrrrfopbq ) ; lh_a = ( ( lh_a * lh_a +
oh_a * oh_a ) + 0.0625 ) - 0.0625 ; oh_a = ( 0.108 - 0.15 * muDoubleScalarCos
( rtB . liiv3zehzs ) ) + 0.15 * muDoubleScalarCos ( rtB . hkrrrfopbq ) ; ph_a
= 0.15 * muDoubleScalarSin ( rtB . liiv3zehzs ) - 0.15 * muDoubleScalarSin (
rtB . hkrrrfopbq ) ; xh_a = ( 0.108 - 0.15 * muDoubleScalarCos ( rtB .
liiv3zehzs ) ) + 0.15 * muDoubleScalarCos ( rtB . hkrrrfopbq ) ; kh_a =
muDoubleScalarSin ( muDoubleScalarAtan ( ( muDoubleScalarSqrt ( ( kh_a * kh_a
* 0.25 - lh_a * lh_a ) + oh_a * oh_a * 0.25 ) - ( 0.15 * muDoubleScalarSin (
rtB . liiv3zehzs ) - 0.15 * muDoubleScalarSin ( rtB . hkrrrfopbq ) ) * 0.5 )
/ ( ( ( ( ph_a * ph_a + xh_a * xh_a ) + 0.0625 ) - 0.0625 ) + ( ( 0.108 -
0.15 * muDoubleScalarCos ( rtB . liiv3zehzs ) ) + 0.15 * muDoubleScalarCos (
rtB . hkrrrfopbq ) ) * 0.5 ) ) * 2.0 ) * 0.25 + 0.15 * muDoubleScalarSin (
rtB . liiv3zehzs ) ; lh_a = 0.15 * muDoubleScalarSin ( rtB . liiv3zehzs ) -
0.15 * muDoubleScalarSin ( rtB . hkrrrfopbq ) ; oh_a = 0.15 *
muDoubleScalarSin ( rtB . liiv3zehzs ) - 0.15 * muDoubleScalarSin ( rtB .
hkrrrfopbq ) ; ph_a = ( 0.108 - 0.15 * muDoubleScalarCos ( rtB . liiv3zehzs )
) + 0.15 * muDoubleScalarCos ( rtB . hkrrrfopbq ) ; oh_a = ( ( oh_a * oh_a +
ph_a * ph_a ) + 0.0625 ) - 0.0625 ; ph_a = ( 0.108 - 0.15 * muDoubleScalarCos
( rtB . liiv3zehzs ) ) + 0.15 * muDoubleScalarCos ( rtB . hkrrrfopbq ) ; xh_a
= 0.15 * muDoubleScalarSin ( rtB . liiv3zehzs ) - 0.15 * muDoubleScalarSin (
rtB . hkrrrfopbq ) ; gi_a = ( 0.108 - 0.15 * muDoubleScalarCos ( rtB .
liiv3zehzs ) ) + 0.15 * muDoubleScalarCos ( rtB . hkrrrfopbq ) ; lh_a = (
muDoubleScalarCos ( muDoubleScalarAtan ( ( muDoubleScalarSqrt ( ( lh_a * lh_a
* 0.25 - oh_a * oh_a ) + ph_a * ph_a * 0.25 ) - ( 0.15 * muDoubleScalarSin (
rtB . liiv3zehzs ) - 0.15 * muDoubleScalarSin ( rtB . hkrrrfopbq ) ) * 0.5 )
/ ( ( ( ( xh_a * xh_a + gi_a * gi_a ) + 0.0625 ) - 0.0625 ) + ( ( 0.108 -
0.15 * muDoubleScalarCos ( rtB . liiv3zehzs ) ) + 0.15 * muDoubleScalarCos (
rtB . hkrrrfopbq ) ) * 0.5 ) ) * 2.0 ) * 0.25 - 0.054 ) + 0.15 *
muDoubleScalarCos ( rtB . liiv3zehzs ) ; kfo1vde04o = ( ( ( ( ( ( ( 0.15 *
muDoubleScalarSin ( rtB . liiv3zehzs ) - 0.15 * muDoubleScalarSin ( rtB .
hkrrrfopbq ) ) * ( 0.075 * muDoubleScalarCos ( rtB . liiv3zehzs ) ) - ( ( (
0.108 - 0.15 * muDoubleScalarCos ( rtB . liiv3zehzs ) ) + 0.15 *
muDoubleScalarCos ( rtB . hkrrrfopbq ) ) * ( 0.3 * muDoubleScalarSin ( rtB .
liiv3zehzs ) ) + ( 0.15 * muDoubleScalarSin ( rtB . liiv3zehzs ) - 0.15 *
muDoubleScalarSin ( rtB . hkrrrfopbq ) ) * ( 0.3 * muDoubleScalarCos ( rtB .
liiv3zehzs ) ) ) * 2.0 * ( ( ( ec_a * ec_a + fc_a * fc_a ) + 0.0625 ) -
0.0625 ) ) + ( ( 0.108 - 0.15 * muDoubleScalarCos ( rtB . liiv3zehzs ) ) +
0.15 * muDoubleScalarCos ( rtB . hkrrrfopbq ) ) * ( 0.075 * muDoubleScalarSin
( rtB . liiv3zehzs ) ) ) / ( muDoubleScalarSqrt ( ( gc_a * gc_a * 0.25 - hc_a
* hc_a ) + ic_a * ic_a * 0.25 ) * 2.0 ) - 0.075 * muDoubleScalarCos ( rtB .
liiv3zehzs ) ) / ( ( ( ( lc_a * lc_a + mc_a * mc_a ) + 0.0625 ) - 0.0625 ) +
( ( 0.108 - 0.15 * muDoubleScalarCos ( rtB . liiv3zehzs ) ) + 0.15 *
muDoubleScalarCos ( rtB . hkrrrfopbq ) ) * 0.5 ) - ( muDoubleScalarSqrt ( (
nc_a * nc_a * 0.25 - oc_a * oc_a ) + pc_a * pc_a * 0.25 ) - ( 0.15 *
muDoubleScalarSin ( rtB . liiv3zehzs ) - 0.15 * muDoubleScalarSin ( rtB .
hkrrrfopbq ) ) * 0.5 ) * ( ( ( 0.15 * muDoubleScalarSin ( rtB . liiv3zehzs )
- 0.15 * muDoubleScalarSin ( rtB . hkrrrfopbq ) ) * ( 0.3 * muDoubleScalarCos
( rtB . liiv3zehzs ) ) + 0.075 * muDoubleScalarSin ( rtB . liiv3zehzs ) ) + (
( 0.108 - 0.15 * muDoubleScalarCos ( rtB . liiv3zehzs ) ) + 0.15 *
muDoubleScalarCos ( rtB . hkrrrfopbq ) ) * ( 0.3 * muDoubleScalarSin ( rtB .
liiv3zehzs ) ) ) / ( sc_a * sc_a ) ) * ( muDoubleScalarSin (
muDoubleScalarAtan ( ( muDoubleScalarSqrt ( ( wb_a * wb_a * 0.25 - xb_a *
xb_a ) + yb_a * yb_a * 0.25 ) - ( 0.15 * muDoubleScalarSin ( rtB . liiv3zehzs
) - 0.15 * muDoubleScalarSin ( rtB . hkrrrfopbq ) ) * 0.5 ) / ( ( ( ( cc_a *
cc_a + dc_a * dc_a ) + 0.0625 ) - 0.0625 ) + ( ( 0.108 - 0.15 *
muDoubleScalarCos ( rtB . liiv3zehzs ) ) + 0.15 * muDoubleScalarCos ( rtB .
hkrrrfopbq ) ) * 0.5 ) ) * 2.0 ) * 0.5 ) / ( tc_a * tc_a / ( wc_a * wc_a ) +
1.0 ) + 0.15 * muDoubleScalarSin ( rtB . liiv3zehzs ) ) * ( muDoubleScalarSin
( muDoubleScalarAtan ( ( muDoubleScalarSqrt ( ( pb_a * pb_a * 0.25 - qb_a *
qb_a ) + rb_a * rb_a * 0.25 ) - ( 0.15 * muDoubleScalarSin ( rtB . liiv3zehzs
) - 0.15 * muDoubleScalarSin ( rtB . hkrrrfopbq ) ) * 0.5 ) / ( ( ( ( ub_a *
ub_a + vb_a * vb_a ) + 0.0625 ) - 0.0625 ) + ( ( 0.108 - 0.15 *
muDoubleScalarCos ( rtB . liiv3zehzs ) ) + 0.15 * muDoubleScalarCos ( rtB .
hkrrrfopbq ) ) * 0.5 ) ) * 2.0 ) * 0.25 + 0.15 * muDoubleScalarSin ( rtB .
liiv3zehzs ) ) / ( xc_a * xc_a ) + ( ( ( ( ( ( 0.15 * muDoubleScalarSin ( rtB
. liiv3zehzs ) - 0.15 * muDoubleScalarSin ( rtB . hkrrrfopbq ) ) * ( 0.075 *
muDoubleScalarCos ( rtB . liiv3zehzs ) ) - ( ( ( 0.108 - 0.15 *
muDoubleScalarCos ( rtB . liiv3zehzs ) ) + 0.15 * muDoubleScalarCos ( rtB .
hkrrrfopbq ) ) * ( 0.3 * muDoubleScalarSin ( rtB . liiv3zehzs ) ) + ( 0.15 *
muDoubleScalarSin ( rtB . liiv3zehzs ) - 0.15 * muDoubleScalarSin ( rtB .
hkrrrfopbq ) ) * ( 0.3 * muDoubleScalarCos ( rtB . liiv3zehzs ) ) ) * 2.0 * (
( ( XD * XD + XB * XB ) + 0.0625 ) - 0.0625 ) ) + ( ( 0.108 - 0.15 *
muDoubleScalarCos ( rtB . liiv3zehzs ) ) + 0.15 * muDoubleScalarCos ( rtB .
hkrrrfopbq ) ) * ( 0.075 * muDoubleScalarSin ( rtB . liiv3zehzs ) ) ) / (
muDoubleScalarSqrt ( ( B0 * B0 * 0.25 - lBD * lBD ) + l_a * l_a * 0.25 ) *
2.0 ) - 0.075 * muDoubleScalarCos ( rtB . liiv3zehzs ) ) / ( ( ( ( o_a * o_a
+ p_a * p_a ) + 0.0625 ) - 0.0625 ) + ( ( 0.108 - 0.15 * muDoubleScalarCos (
rtB . liiv3zehzs ) ) + 0.15 * muDoubleScalarCos ( rtB . hkrrrfopbq ) ) * 0.5
) - ( muDoubleScalarSqrt ( ( q_a * q_a * 0.25 - r_a * r_a ) + s_a * s_a *
0.25 ) - ( 0.15 * muDoubleScalarSin ( rtB . liiv3zehzs ) - 0.15 *
muDoubleScalarSin ( rtB . hkrrrfopbq ) ) * 0.5 ) * ( ( ( 0.15 *
muDoubleScalarSin ( rtB . liiv3zehzs ) - 0.15 * muDoubleScalarSin ( rtB .
hkrrrfopbq ) ) * ( 0.3 * muDoubleScalarCos ( rtB . liiv3zehzs ) ) + 0.075 *
muDoubleScalarSin ( rtB . liiv3zehzs ) ) + ( ( 0.108 - 0.15 *
muDoubleScalarCos ( rtB . liiv3zehzs ) ) + 0.15 * muDoubleScalarCos ( rtB .
hkrrrfopbq ) ) * ( 0.3 * muDoubleScalarSin ( rtB . liiv3zehzs ) ) ) / ( v_a *
v_a ) ) * ( muDoubleScalarCos ( muDoubleScalarAtan ( ( muDoubleScalarSqrt ( (
kfo1vde04o * kfo1vde04o * 0.25 - YC * YC ) + A0 * A0 * 0.25 ) - ( 0.15 *
muDoubleScalarSin ( rtB . liiv3zehzs ) - 0.15 * muDoubleScalarSin ( rtB .
hkrrrfopbq ) ) * 0.5 ) / ( ( ( ( YD * YD + YB * YB ) + 0.0625 ) - 0.0625 ) +
( ( 0.108 - 0.15 * muDoubleScalarCos ( rtB . liiv3zehzs ) ) + 0.15 *
muDoubleScalarCos ( rtB . hkrrrfopbq ) ) * 0.5 ) ) * 2.0 ) * 0.5 ) / ( w_a *
w_a / ( ab_a * ab_a ) + 1.0 ) + 0.15 * muDoubleScalarCos ( rtB . liiv3zehzs )
) / ( ( muDoubleScalarCos ( muDoubleScalarAtan ( ( muDoubleScalarSqrt ( (
bb_a * bb_a * 0.25 - jb_a * jb_a ) + kb_a * kb_a * 0.25 ) - ( 0.15 *
muDoubleScalarSin ( rtB . liiv3zehzs ) - 0.15 * muDoubleScalarSin ( rtB .
hkrrrfopbq ) ) * 0.5 ) / ( ( ( ( nb_a * nb_a + ob_a * ob_a ) + 0.0625 ) -
0.0625 ) + ( ( 0.108 - 0.15 * muDoubleScalarCos ( rtB . liiv3zehzs ) ) + 0.15
* muDoubleScalarCos ( rtB . hkrrrfopbq ) ) * 0.5 ) ) * 2.0 ) * 0.25 - 0.054 )
+ 0.15 * muDoubleScalarCos ( rtB . liiv3zehzs ) ) ) * rtB . ne53mrvzfq [ 9 ]
/ ( gd_a * gd_a / ( hd_a * hd_a ) + 1.0 ) - ( ( ( ( ( ( 0.15 *
muDoubleScalarSin ( rtB . liiv3zehzs ) - 0.15 * muDoubleScalarSin ( rtB .
hkrrrfopbq ) ) * ( 0.075 * muDoubleScalarCos ( rtB . hkrrrfopbq ) ) - ( ( (
0.108 - 0.15 * muDoubleScalarCos ( rtB . liiv3zehzs ) ) + 0.15 *
muDoubleScalarCos ( rtB . hkrrrfopbq ) ) * ( 0.3 * muDoubleScalarSin ( rtB .
hkrrrfopbq ) ) + ( 0.15 * muDoubleScalarSin ( rtB . liiv3zehzs ) - 0.15 *
muDoubleScalarSin ( rtB . hkrrrfopbq ) ) * ( 0.3 * muDoubleScalarCos ( rtB .
hkrrrfopbq ) ) ) * 2.0 * ( ( ( le_a * le_a + me_a * me_a ) + 0.0625 ) -
0.0625 ) ) + ( ( 0.108 - 0.15 * muDoubleScalarCos ( rtB . liiv3zehzs ) ) +
0.15 * muDoubleScalarCos ( rtB . hkrrrfopbq ) ) * ( 0.075 * muDoubleScalarSin
( rtB . hkrrrfopbq ) ) ) / ( muDoubleScalarSqrt ( ( ne_a * ne_a * 0.25 - oe_a
* oe_a ) + pe_a * pe_a * 0.25 ) * 2.0 ) - 0.075 * muDoubleScalarCos ( rtB .
hkrrrfopbq ) ) / ( ( ( ( se_a * se_a + te_a * te_a ) + 0.0625 ) - 0.0625 ) +
( ( 0.108 - 0.15 * muDoubleScalarCos ( rtB . liiv3zehzs ) ) + 0.15 *
muDoubleScalarCos ( rtB . hkrrrfopbq ) ) * 0.5 ) - ( muDoubleScalarSqrt ( (
ue_a * ue_a * 0.25 - ve_a * ve_a ) + we_a * we_a * 0.25 ) - ( 0.15 *
muDoubleScalarSin ( rtB . liiv3zehzs ) - 0.15 * muDoubleScalarSin ( rtB .
hkrrrfopbq ) ) * 0.5 ) * ( ( ( 0.15 * muDoubleScalarSin ( rtB . liiv3zehzs )
- 0.15 * muDoubleScalarSin ( rtB . hkrrrfopbq ) ) * ( 0.3 * muDoubleScalarCos
( rtB . hkrrrfopbq ) ) + 0.075 * muDoubleScalarSin ( rtB . hkrrrfopbq ) ) + (
( 0.108 - 0.15 * muDoubleScalarCos ( rtB . liiv3zehzs ) ) + 0.15 *
muDoubleScalarCos ( rtB . hkrrrfopbq ) ) * ( 0.3 * muDoubleScalarSin ( rtB .
hkrrrfopbq ) ) ) / ( af_a * af_a ) ) * ( muDoubleScalarCos (
muDoubleScalarAtan ( ( muDoubleScalarSqrt ( ( kd_a * kd_a * 0.25 - ld_a *
ld_a ) + td_a * td_a * 0.25 ) - ( 0.15 * muDoubleScalarSin ( rtB . liiv3zehzs
) - 0.15 * muDoubleScalarSin ( rtB . hkrrrfopbq ) ) * 0.5 ) / ( ( ( ( ce_a *
ce_a + ke_a * ke_a ) + 0.0625 ) - 0.0625 ) + ( ( 0.108 - 0.15 *
muDoubleScalarCos ( rtB . liiv3zehzs ) ) + 0.15 * muDoubleScalarCos ( rtB .
hkrrrfopbq ) ) * 0.5 ) ) * 2.0 ) * 0.5 ) / ( ( ( muDoubleScalarCos (
muDoubleScalarAtan ( ( muDoubleScalarSqrt ( ( ff_a * ff_a * 0.25 - nf_a *
nf_a ) + of_a * of_a * 0.25 ) - ( 0.15 * muDoubleScalarSin ( rtB . liiv3zehzs
) - 0.15 * muDoubleScalarSin ( rtB . hkrrrfopbq ) ) * 0.5 ) / ( ( ( ( rf_a *
rf_a + sf_a * sf_a ) + 0.0625 ) - 0.0625 ) + ( ( 0.108 - 0.15 *
muDoubleScalarCos ( rtB . liiv3zehzs ) ) + 0.15 * muDoubleScalarCos ( rtB .
hkrrrfopbq ) ) * 0.5 ) ) * 2.0 ) * 0.25 - 0.054 ) + 0.15 * muDoubleScalarCos
( rtB . liiv3zehzs ) ) * ( bf_a * bf_a / ( ef_a * ef_a ) + 1.0 ) ) + ( ( ( (
( 0.15 * muDoubleScalarSin ( rtB . liiv3zehzs ) - 0.15 * muDoubleScalarSin (
rtB . hkrrrfopbq ) ) * ( 0.075 * muDoubleScalarCos ( rtB . hkrrrfopbq ) ) - (
( ( 0.108 - 0.15 * muDoubleScalarCos ( rtB . liiv3zehzs ) ) + 0.15 *
muDoubleScalarCos ( rtB . hkrrrfopbq ) ) * ( 0.3 * muDoubleScalarSin ( rtB .
hkrrrfopbq ) ) + ( 0.15 * muDoubleScalarSin ( rtB . liiv3zehzs ) - 0.15 *
muDoubleScalarSin ( rtB . hkrrrfopbq ) ) * ( 0.3 * muDoubleScalarCos ( rtB .
hkrrrfopbq ) ) ) * 2.0 * ( ( ( ig_a * ig_a + jg_a * jg_a ) + 0.0625 ) -
0.0625 ) ) + ( ( 0.108 - 0.15 * muDoubleScalarCos ( rtB . liiv3zehzs ) ) +
0.15 * muDoubleScalarCos ( rtB . hkrrrfopbq ) ) * ( 0.075 * muDoubleScalarSin
( rtB . hkrrrfopbq ) ) ) / ( muDoubleScalarSqrt ( ( kg_a * kg_a * 0.25 - lg_a
* lg_a ) + mg_a * mg_a * 0.25 ) * 2.0 ) - 0.075 * muDoubleScalarCos ( rtB .
hkrrrfopbq ) ) / ( ( ( ( pg_a * pg_a + qg_a * qg_a ) + 0.0625 ) - 0.0625 ) +
( ( 0.108 - 0.15 * muDoubleScalarCos ( rtB . liiv3zehzs ) ) + 0.15 *
muDoubleScalarCos ( rtB . hkrrrfopbq ) ) * 0.5 ) - ( muDoubleScalarSqrt ( (
rg_a * rg_a * 0.25 - sg_a * sg_a ) + tg_a * tg_a * 0.25 ) - ( 0.15 *
muDoubleScalarSin ( rtB . liiv3zehzs ) - 0.15 * muDoubleScalarSin ( rtB .
hkrrrfopbq ) ) * 0.5 ) * ( ( ( 0.15 * muDoubleScalarSin ( rtB . liiv3zehzs )
- 0.15 * muDoubleScalarSin ( rtB . hkrrrfopbq ) ) * ( 0.3 * muDoubleScalarCos
( rtB . hkrrrfopbq ) ) + 0.075 * muDoubleScalarSin ( rtB . hkrrrfopbq ) ) + (
( 0.108 - 0.15 * muDoubleScalarCos ( rtB . liiv3zehzs ) ) + 0.15 *
muDoubleScalarCos ( rtB . hkrrrfopbq ) ) * ( 0.3 * muDoubleScalarSin ( rtB .
hkrrrfopbq ) ) ) / ( wg_a * wg_a ) ) * ( ( muDoubleScalarSin (
muDoubleScalarAtan ( ( muDoubleScalarSqrt ( ( bg_a * bg_a * 0.25 - cg_a *
cg_a ) + dg_a * dg_a * 0.25 ) - ( 0.15 * muDoubleScalarSin ( rtB . liiv3zehzs
) - 0.15 * muDoubleScalarSin ( rtB . hkrrrfopbq ) ) * 0.5 ) / ( ( ( ( gg_a *
gg_a + hg_a * hg_a ) + 0.0625 ) - 0.0625 ) + ( ( 0.108 - 0.15 *
muDoubleScalarCos ( rtB . liiv3zehzs ) ) + 0.15 * muDoubleScalarCos ( rtB .
hkrrrfopbq ) ) * 0.5 ) ) * 2.0 ) * 0.25 + 0.15 * muDoubleScalarSin ( rtB .
liiv3zehzs ) ) * ( muDoubleScalarSin ( muDoubleScalarAtan ( (
muDoubleScalarSqrt ( ( tf_a * tf_a * 0.25 - uf_a * uf_a ) + vf_a * vf_a *
0.25 ) - ( 0.15 * muDoubleScalarSin ( rtB . liiv3zehzs ) - 0.15 *
muDoubleScalarSin ( rtB . hkrrrfopbq ) ) * 0.5 ) / ( ( ( ( yf_a * yf_a + ag_a
* ag_a ) + 0.0625 ) - 0.0625 ) + ( ( 0.108 - 0.15 * muDoubleScalarCos ( rtB .
liiv3zehzs ) ) + 0.15 * muDoubleScalarCos ( rtB . hkrrrfopbq ) ) * 0.5 ) ) *
2.0 ) * 0.5 ) ) / ( ( xg_a * xg_a / ( bh_a * bh_a ) + 1.0 ) * ( ch_a * ch_a )
) ) * rtB . ne53mrvzfq [ 11 ] / ( kh_a * kh_a / ( lh_a * lh_a ) + 1.0 ) ; rtB
. oaaq01l5fu = ( 0.0 - kfo1vde04o ) - rtB . ne53mrvzfq [ 7 ] ; rtB .
oqhhk34uis = rtP . Gain25_Gain * rtB . oaaq01l5fu ; rtB . fx0alrhetp = rtP .
Gain33_Gain * rtB . ne53mrvzfq [ 6 ] ; YD = 0.15 * muDoubleScalarSin ( rtB .
hkrrrfopbq ) ; YB = 0.15 * muDoubleScalarSin ( rtB . liiv3zehzs ) ; XD = 0.15
* muDoubleScalarCos ( rtB . hkrrrfopbq ) + 0.108 ; XB = 0.15 *
muDoubleScalarCos ( rtB . liiv3zehzs ) ; lBD = muDoubleScalarSqrt ( ( XD - XB
) * ( XD - XB ) + ( YD - YB ) * ( YD - YB ) ) ; A0 = ( XD - XB ) * 0.5 ; B0 =
( YD - YB ) * 0.5 ; lBD = ( lBD * lBD + 0.0625 ) - 0.0625 ; A0 =
muDoubleScalarAtan2 ( muDoubleScalarSqrt ( ( A0 * A0 + B0 * B0 ) - lBD * lBD
) + B0 , A0 + lBD ) * 2.0 ; kfo1vde04o = 0.15 * muDoubleScalarCos ( rtB .
liiv3zehzs ) + 0.25 * muDoubleScalarCos ( A0 ) ; YC = 0.15 *
muDoubleScalarSin ( rtB . liiv3zehzs ) + 0.25 * muDoubleScalarSin ( A0 ) ;
rtB . p1iy05u0ee = ( kfo1vde04o - 0.054 ) * ( kfo1vde04o - 0.054 ) + YC * YC
; rtB . p1iy05u0ee = muDoubleScalarSqrt ( rtB . p1iy05u0ee ) ; rtB .
c4gioprvct = ( rtP . Constant12_Value - muDoubleScalarAtan2 ( YC , kfo1vde04o
- 0.054 ) ) - rtB . ne53mrvzfq [ 6 ] ; rtB . ewa15giale = rtP . Gain24_Gain *
rtB . c4gioprvct ; rtB . cfmoived0b = rtP . Gain40_Gain * rtB . ne53mrvzfq [
7 ] ; rtB . jsqo11gz4n = rtP . Constant10_Value + rtB . ne53mrvzfq [ 2 ] ;
rtB . fvygvwpi1d = rtP . Constant10_Value + rtB . ne53mrvzfq [ 4 ] ;
kfo1vde04o = 0.15 * muDoubleScalarSin ( rtB . jsqo11gz4n ) - 0.15 *
muDoubleScalarSin ( rtB . fvygvwpi1d ) ; YC = 0.15 * muDoubleScalarSin ( rtB
. jsqo11gz4n ) - 0.15 * muDoubleScalarSin ( rtB . fvygvwpi1d ) ; A0 = ( 0.108
- 0.15 * muDoubleScalarCos ( rtB . jsqo11gz4n ) ) + 0.15 * muDoubleScalarCos
( rtB . fvygvwpi1d ) ; YC = ( ( YC * YC + A0 * A0 ) + 0.0625 ) - 0.0625 ; A0
= ( 0.108 - 0.15 * muDoubleScalarCos ( rtB . jsqo11gz4n ) ) + 0.15 *
muDoubleScalarCos ( rtB . fvygvwpi1d ) ; YD = 0.15 * muDoubleScalarSin ( rtB
. jsqo11gz4n ) - 0.15 * muDoubleScalarSin ( rtB . fvygvwpi1d ) ; YB = ( 0.108
- 0.15 * muDoubleScalarCos ( rtB . jsqo11gz4n ) ) + 0.15 * muDoubleScalarCos
( rtB . fvygvwpi1d ) ; XD = 0.15 * muDoubleScalarSin ( rtB . jsqo11gz4n ) -
0.15 * muDoubleScalarSin ( rtB . fvygvwpi1d ) ; XB = ( 0.108 - 0.15 *
muDoubleScalarCos ( rtB . jsqo11gz4n ) ) + 0.15 * muDoubleScalarCos ( rtB .
fvygvwpi1d ) ; B0 = 0.15 * muDoubleScalarSin ( rtB . jsqo11gz4n ) - 0.15 *
muDoubleScalarSin ( rtB . fvygvwpi1d ) ; lBD = 0.15 * muDoubleScalarSin ( rtB
. jsqo11gz4n ) - 0.15 * muDoubleScalarSin ( rtB . fvygvwpi1d ) ; l_a = (
0.108 - 0.15 * muDoubleScalarCos ( rtB . jsqo11gz4n ) ) + 0.15 *
muDoubleScalarCos ( rtB . fvygvwpi1d ) ; lBD = ( ( lBD * lBD + l_a * l_a ) +
0.0625 ) - 0.0625 ; l_a = ( 0.108 - 0.15 * muDoubleScalarCos ( rtB .
jsqo11gz4n ) ) + 0.15 * muDoubleScalarCos ( rtB . fvygvwpi1d ) ; o_a = 0.15 *
muDoubleScalarSin ( rtB . jsqo11gz4n ) - 0.15 * muDoubleScalarSin ( rtB .
fvygvwpi1d ) ; p_a = ( 0.108 - 0.15 * muDoubleScalarCos ( rtB . jsqo11gz4n )
) + 0.15 * muDoubleScalarCos ( rtB . fvygvwpi1d ) ; q_a = 0.15 *
muDoubleScalarSin ( rtB . jsqo11gz4n ) - 0.15 * muDoubleScalarSin ( rtB .
fvygvwpi1d ) ; r_a = 0.15 * muDoubleScalarSin ( rtB . jsqo11gz4n ) - 0.15 *
muDoubleScalarSin ( rtB . fvygvwpi1d ) ; s_a = ( 0.108 - 0.15 *
muDoubleScalarCos ( rtB . jsqo11gz4n ) ) + 0.15 * muDoubleScalarCos ( rtB .
fvygvwpi1d ) ; r_a = ( ( r_a * r_a + s_a * s_a ) + 0.0625 ) - 0.0625 ; s_a =
( 0.108 - 0.15 * muDoubleScalarCos ( rtB . jsqo11gz4n ) ) + 0.15 *
muDoubleScalarCos ( rtB . fvygvwpi1d ) ; v_a = 0.15 * muDoubleScalarSin ( rtB
. jsqo11gz4n ) - 0.15 * muDoubleScalarSin ( rtB . fvygvwpi1d ) ; w_a = (
0.108 - 0.15 * muDoubleScalarCos ( rtB . jsqo11gz4n ) ) + 0.15 *
muDoubleScalarCos ( rtB . fvygvwpi1d ) ; v_a = ( ( ( v_a * v_a + w_a * w_a )
+ 0.0625 ) - 0.0625 ) + ( ( 0.108 - 0.15 * muDoubleScalarCos ( rtB .
jsqo11gz4n ) ) + 0.15 * muDoubleScalarCos ( rtB . fvygvwpi1d ) ) * 0.5 ; w_a
= 0.15 * muDoubleScalarSin ( rtB . jsqo11gz4n ) - 0.15 * muDoubleScalarSin (
rtB . fvygvwpi1d ) ; ab_a = 0.15 * muDoubleScalarSin ( rtB . jsqo11gz4n ) -
0.15 * muDoubleScalarSin ( rtB . fvygvwpi1d ) ; bb_a = ( 0.108 - 0.15 *
muDoubleScalarCos ( rtB . jsqo11gz4n ) ) + 0.15 * muDoubleScalarCos ( rtB .
fvygvwpi1d ) ; ab_a = ( ( ab_a * ab_a + bb_a * bb_a ) + 0.0625 ) - 0.0625 ;
bb_a = ( 0.108 - 0.15 * muDoubleScalarCos ( rtB . jsqo11gz4n ) ) + 0.15 *
muDoubleScalarCos ( rtB . fvygvwpi1d ) ; w_a = muDoubleScalarSqrt ( ( w_a *
w_a * 0.25 - ab_a * ab_a ) + bb_a * bb_a * 0.25 ) - ( 0.15 *
muDoubleScalarSin ( rtB . jsqo11gz4n ) - 0.15 * muDoubleScalarSin ( rtB .
fvygvwpi1d ) ) * 0.5 ; ab_a = 0.15 * muDoubleScalarSin ( rtB . jsqo11gz4n ) -
0.15 * muDoubleScalarSin ( rtB . fvygvwpi1d ) ; bb_a = ( 0.108 - 0.15 *
muDoubleScalarCos ( rtB . jsqo11gz4n ) ) + 0.15 * muDoubleScalarCos ( rtB .
fvygvwpi1d ) ; ab_a = ( ( ( ab_a * ab_a + bb_a * bb_a ) + 0.0625 ) - 0.0625 )
+ ( ( 0.108 - 0.15 * muDoubleScalarCos ( rtB . jsqo11gz4n ) ) + 0.15 *
muDoubleScalarCos ( rtB . fvygvwpi1d ) ) * 0.5 ; bb_a = 0.15 *
muDoubleScalarSin ( rtB . jsqo11gz4n ) - 0.15 * muDoubleScalarSin ( rtB .
fvygvwpi1d ) ; jb_a = 0.15 * muDoubleScalarSin ( rtB . jsqo11gz4n ) - 0.15 *
muDoubleScalarSin ( rtB . fvygvwpi1d ) ; kb_a = ( 0.108 - 0.15 *
muDoubleScalarCos ( rtB . jsqo11gz4n ) ) + 0.15 * muDoubleScalarCos ( rtB .
fvygvwpi1d ) ; jb_a = ( ( jb_a * jb_a + kb_a * kb_a ) + 0.0625 ) - 0.0625 ;
kb_a = ( 0.108 - 0.15 * muDoubleScalarCos ( rtB . jsqo11gz4n ) ) + 0.15 *
muDoubleScalarCos ( rtB . fvygvwpi1d ) ; nb_a = 0.15 * muDoubleScalarSin (
rtB . jsqo11gz4n ) - 0.15 * muDoubleScalarSin ( rtB . fvygvwpi1d ) ; ob_a = (
0.108 - 0.15 * muDoubleScalarCos ( rtB . jsqo11gz4n ) ) + 0.15 *
muDoubleScalarCos ( rtB . fvygvwpi1d ) ; pb_a = 0.15 * muDoubleScalarSin (
rtB . jsqo11gz4n ) - 0.15 * muDoubleScalarSin ( rtB . fvygvwpi1d ) ; qb_a =
0.15 * muDoubleScalarSin ( rtB . jsqo11gz4n ) - 0.15 * muDoubleScalarSin (
rtB . fvygvwpi1d ) ; rb_a = ( 0.108 - 0.15 * muDoubleScalarCos ( rtB .
jsqo11gz4n ) ) + 0.15 * muDoubleScalarCos ( rtB . fvygvwpi1d ) ; qb_a = ( (
qb_a * qb_a + rb_a * rb_a ) + 0.0625 ) - 0.0625 ; rb_a = ( 0.108 - 0.15 *
muDoubleScalarCos ( rtB . jsqo11gz4n ) ) + 0.15 * muDoubleScalarCos ( rtB .
fvygvwpi1d ) ; ub_a = 0.15 * muDoubleScalarSin ( rtB . jsqo11gz4n ) - 0.15 *
muDoubleScalarSin ( rtB . fvygvwpi1d ) ; vb_a = ( 0.108 - 0.15 *
muDoubleScalarCos ( rtB . jsqo11gz4n ) ) + 0.15 * muDoubleScalarCos ( rtB .
fvygvwpi1d ) ; wb_a = 0.15 * muDoubleScalarSin ( rtB . jsqo11gz4n ) - 0.15 *
muDoubleScalarSin ( rtB . fvygvwpi1d ) ; xb_a = 0.15 * muDoubleScalarSin (
rtB . jsqo11gz4n ) - 0.15 * muDoubleScalarSin ( rtB . fvygvwpi1d ) ; yb_a = (
0.108 - 0.15 * muDoubleScalarCos ( rtB . jsqo11gz4n ) ) + 0.15 *
muDoubleScalarCos ( rtB . fvygvwpi1d ) ; xb_a = ( ( xb_a * xb_a + yb_a * yb_a
) + 0.0625 ) - 0.0625 ; yb_a = ( 0.108 - 0.15 * muDoubleScalarCos ( rtB .
jsqo11gz4n ) ) + 0.15 * muDoubleScalarCos ( rtB . fvygvwpi1d ) ; cc_a = 0.15
* muDoubleScalarSin ( rtB . jsqo11gz4n ) - 0.15 * muDoubleScalarSin ( rtB .
fvygvwpi1d ) ; dc_a = ( 0.108 - 0.15 * muDoubleScalarCos ( rtB . jsqo11gz4n )
) + 0.15 * muDoubleScalarCos ( rtB . fvygvwpi1d ) ; ec_a = 0.15 *
muDoubleScalarSin ( rtB . jsqo11gz4n ) - 0.15 * muDoubleScalarSin ( rtB .
fvygvwpi1d ) ; fc_a = ( 0.108 - 0.15 * muDoubleScalarCos ( rtB . jsqo11gz4n )
) + 0.15 * muDoubleScalarCos ( rtB . fvygvwpi1d ) ; gc_a = 0.15 *
muDoubleScalarSin ( rtB . jsqo11gz4n ) - 0.15 * muDoubleScalarSin ( rtB .
fvygvwpi1d ) ; hc_a = 0.15 * muDoubleScalarSin ( rtB . jsqo11gz4n ) - 0.15 *
muDoubleScalarSin ( rtB . fvygvwpi1d ) ; ic_a = ( 0.108 - 0.15 *
muDoubleScalarCos ( rtB . jsqo11gz4n ) ) + 0.15 * muDoubleScalarCos ( rtB .
fvygvwpi1d ) ; hc_a = ( ( hc_a * hc_a + ic_a * ic_a ) + 0.0625 ) - 0.0625 ;
ic_a = ( 0.108 - 0.15 * muDoubleScalarCos ( rtB . jsqo11gz4n ) ) + 0.15 *
muDoubleScalarCos ( rtB . fvygvwpi1d ) ; lc_a = 0.15 * muDoubleScalarSin (
rtB . jsqo11gz4n ) - 0.15 * muDoubleScalarSin ( rtB . fvygvwpi1d ) ; mc_a = (
0.108 - 0.15 * muDoubleScalarCos ( rtB . jsqo11gz4n ) ) + 0.15 *
muDoubleScalarCos ( rtB . fvygvwpi1d ) ; nc_a = 0.15 * muDoubleScalarSin (
rtB . jsqo11gz4n ) - 0.15 * muDoubleScalarSin ( rtB . fvygvwpi1d ) ; oc_a =
0.15 * muDoubleScalarSin ( rtB . jsqo11gz4n ) - 0.15 * muDoubleScalarSin (
rtB . fvygvwpi1d ) ; pc_a = ( 0.108 - 0.15 * muDoubleScalarCos ( rtB .
jsqo11gz4n ) ) + 0.15 * muDoubleScalarCos ( rtB . fvygvwpi1d ) ; oc_a = ( (
oc_a * oc_a + pc_a * pc_a ) + 0.0625 ) - 0.0625 ; pc_a = ( 0.108 - 0.15 *
muDoubleScalarCos ( rtB . jsqo11gz4n ) ) + 0.15 * muDoubleScalarCos ( rtB .
fvygvwpi1d ) ; sc_a = 0.15 * muDoubleScalarSin ( rtB . jsqo11gz4n ) - 0.15 *
muDoubleScalarSin ( rtB . fvygvwpi1d ) ; tc_a = ( 0.108 - 0.15 *
muDoubleScalarCos ( rtB . jsqo11gz4n ) ) + 0.15 * muDoubleScalarCos ( rtB .
fvygvwpi1d ) ; sc_a = ( ( ( sc_a * sc_a + tc_a * tc_a ) + 0.0625 ) - 0.0625 )
+ ( ( 0.108 - 0.15 * muDoubleScalarCos ( rtB . jsqo11gz4n ) ) + 0.15 *
muDoubleScalarCos ( rtB . fvygvwpi1d ) ) * 0.5 ; tc_a = 0.15 *
muDoubleScalarSin ( rtB . jsqo11gz4n ) - 0.15 * muDoubleScalarSin ( rtB .
fvygvwpi1d ) ; wc_a = 0.15 * muDoubleScalarSin ( rtB . jsqo11gz4n ) - 0.15 *
muDoubleScalarSin ( rtB . fvygvwpi1d ) ; xc_a = ( 0.108 - 0.15 *
muDoubleScalarCos ( rtB . jsqo11gz4n ) ) + 0.15 * muDoubleScalarCos ( rtB .
fvygvwpi1d ) ; wc_a = ( ( wc_a * wc_a + xc_a * xc_a ) + 0.0625 ) - 0.0625 ;
xc_a = ( 0.108 - 0.15 * muDoubleScalarCos ( rtB . jsqo11gz4n ) ) + 0.15 *
muDoubleScalarCos ( rtB . fvygvwpi1d ) ; tc_a = muDoubleScalarSqrt ( ( tc_a *
tc_a * 0.25 - wc_a * wc_a ) + xc_a * xc_a * 0.25 ) - ( 0.15 *
muDoubleScalarSin ( rtB . jsqo11gz4n ) - 0.15 * muDoubleScalarSin ( rtB .
fvygvwpi1d ) ) * 0.5 ; wc_a = 0.15 * muDoubleScalarSin ( rtB . jsqo11gz4n ) -
0.15 * muDoubleScalarSin ( rtB . fvygvwpi1d ) ; xc_a = ( 0.108 - 0.15 *
muDoubleScalarCos ( rtB . jsqo11gz4n ) ) + 0.15 * muDoubleScalarCos ( rtB .
fvygvwpi1d ) ; wc_a = ( ( ( wc_a * wc_a + xc_a * xc_a ) + 0.0625 ) - 0.0625 )
+ ( ( 0.108 - 0.15 * muDoubleScalarCos ( rtB . jsqo11gz4n ) ) + 0.15 *
muDoubleScalarCos ( rtB . fvygvwpi1d ) ) * 0.5 ; xc_a = 0.15 *
muDoubleScalarSin ( rtB . jsqo11gz4n ) - 0.15 * muDoubleScalarSin ( rtB .
fvygvwpi1d ) ; gd_a = 0.15 * muDoubleScalarSin ( rtB . jsqo11gz4n ) - 0.15 *
muDoubleScalarSin ( rtB . fvygvwpi1d ) ; hd_a = ( 0.108 - 0.15 *
muDoubleScalarCos ( rtB . jsqo11gz4n ) ) + 0.15 * muDoubleScalarCos ( rtB .
fvygvwpi1d ) ; gd_a = ( ( gd_a * gd_a + hd_a * hd_a ) + 0.0625 ) - 0.0625 ;
hd_a = ( 0.108 - 0.15 * muDoubleScalarCos ( rtB . jsqo11gz4n ) ) + 0.15 *
muDoubleScalarCos ( rtB . fvygvwpi1d ) ; kd_a = 0.15 * muDoubleScalarSin (
rtB . jsqo11gz4n ) - 0.15 * muDoubleScalarSin ( rtB . fvygvwpi1d ) ; ld_a = (
0.108 - 0.15 * muDoubleScalarCos ( rtB . jsqo11gz4n ) ) + 0.15 *
muDoubleScalarCos ( rtB . fvygvwpi1d ) ; xc_a = ( muDoubleScalarCos (
muDoubleScalarAtan ( ( muDoubleScalarSqrt ( ( xc_a * xc_a * 0.25 - gd_a *
gd_a ) + hd_a * hd_a * 0.25 ) - ( 0.15 * muDoubleScalarSin ( rtB . jsqo11gz4n
) - 0.15 * muDoubleScalarSin ( rtB . fvygvwpi1d ) ) * 0.5 ) / ( ( ( ( kd_a *
kd_a + ld_a * ld_a ) + 0.0625 ) - 0.0625 ) + ( ( 0.108 - 0.15 *
muDoubleScalarCos ( rtB . jsqo11gz4n ) ) + 0.15 * muDoubleScalarCos ( rtB .
fvygvwpi1d ) ) * 0.5 ) ) * 2.0 ) * 0.25 - 0.054 ) + 0.15 * muDoubleScalarCos
( rtB . jsqo11gz4n ) ; gd_a = 0.15 * muDoubleScalarSin ( rtB . jsqo11gz4n ) -
0.15 * muDoubleScalarSin ( rtB . fvygvwpi1d ) ; hd_a = 0.15 *
muDoubleScalarSin ( rtB . jsqo11gz4n ) - 0.15 * muDoubleScalarSin ( rtB .
fvygvwpi1d ) ; kd_a = ( 0.108 - 0.15 * muDoubleScalarCos ( rtB . jsqo11gz4n )
) + 0.15 * muDoubleScalarCos ( rtB . fvygvwpi1d ) ; hd_a = ( ( hd_a * hd_a +
kd_a * kd_a ) + 0.0625 ) - 0.0625 ; kd_a = ( 0.108 - 0.15 * muDoubleScalarCos
( rtB . jsqo11gz4n ) ) + 0.15 * muDoubleScalarCos ( rtB . fvygvwpi1d ) ; ld_a
= 0.15 * muDoubleScalarSin ( rtB . jsqo11gz4n ) - 0.15 * muDoubleScalarSin (
rtB . fvygvwpi1d ) ; td_a = ( 0.108 - 0.15 * muDoubleScalarCos ( rtB .
jsqo11gz4n ) ) + 0.15 * muDoubleScalarCos ( rtB . fvygvwpi1d ) ; gd_a =
muDoubleScalarSin ( muDoubleScalarAtan ( ( muDoubleScalarSqrt ( ( gd_a * gd_a
* 0.25 - hd_a * hd_a ) + kd_a * kd_a * 0.25 ) - ( 0.15 * muDoubleScalarSin (
rtB . jsqo11gz4n ) - 0.15 * muDoubleScalarSin ( rtB . fvygvwpi1d ) ) * 0.5 )
/ ( ( ( ( ld_a * ld_a + td_a * td_a ) + 0.0625 ) - 0.0625 ) + ( ( 0.108 -
0.15 * muDoubleScalarCos ( rtB . jsqo11gz4n ) ) + 0.15 * muDoubleScalarCos (
rtB . fvygvwpi1d ) ) * 0.5 ) ) * 2.0 ) * 0.25 + 0.15 * muDoubleScalarSin (
rtB . jsqo11gz4n ) ; hd_a = 0.15 * muDoubleScalarSin ( rtB . jsqo11gz4n ) -
0.15 * muDoubleScalarSin ( rtB . fvygvwpi1d ) ; kd_a = 0.15 *
muDoubleScalarSin ( rtB . jsqo11gz4n ) - 0.15 * muDoubleScalarSin ( rtB .
fvygvwpi1d ) ; ld_a = ( 0.108 - 0.15 * muDoubleScalarCos ( rtB . jsqo11gz4n )
) + 0.15 * muDoubleScalarCos ( rtB . fvygvwpi1d ) ; kd_a = ( ( kd_a * kd_a +
ld_a * ld_a ) + 0.0625 ) - 0.0625 ; ld_a = ( 0.108 - 0.15 * muDoubleScalarCos
( rtB . jsqo11gz4n ) ) + 0.15 * muDoubleScalarCos ( rtB . fvygvwpi1d ) ; td_a
= 0.15 * muDoubleScalarSin ( rtB . jsqo11gz4n ) - 0.15 * muDoubleScalarSin (
rtB . fvygvwpi1d ) ; ce_a = ( 0.108 - 0.15 * muDoubleScalarCos ( rtB .
jsqo11gz4n ) ) + 0.15 * muDoubleScalarCos ( rtB . fvygvwpi1d ) ; hd_a = (
muDoubleScalarCos ( muDoubleScalarAtan ( ( muDoubleScalarSqrt ( ( hd_a * hd_a
* 0.25 - kd_a * kd_a ) + ld_a * ld_a * 0.25 ) - ( 0.15 * muDoubleScalarSin (
rtB . jsqo11gz4n ) - 0.15 * muDoubleScalarSin ( rtB . fvygvwpi1d ) ) * 0.5 )
/ ( ( ( ( td_a * td_a + ce_a * ce_a ) + 0.0625 ) - 0.0625 ) + ( ( 0.108 -
0.15 * muDoubleScalarCos ( rtB . jsqo11gz4n ) ) + 0.15 * muDoubleScalarCos (
rtB . fvygvwpi1d ) ) * 0.5 ) ) * 2.0 ) * 0.25 - 0.054 ) + 0.15 *
muDoubleScalarCos ( rtB . jsqo11gz4n ) ; kd_a = 0.15 * muDoubleScalarSin (
rtB . jsqo11gz4n ) - 0.15 * muDoubleScalarSin ( rtB . fvygvwpi1d ) ; ld_a =
0.15 * muDoubleScalarSin ( rtB . jsqo11gz4n ) - 0.15 * muDoubleScalarSin (
rtB . fvygvwpi1d ) ; td_a = ( 0.108 - 0.15 * muDoubleScalarCos ( rtB .
jsqo11gz4n ) ) + 0.15 * muDoubleScalarCos ( rtB . fvygvwpi1d ) ; ld_a = ( (
ld_a * ld_a + td_a * td_a ) + 0.0625 ) - 0.0625 ; td_a = ( 0.108 - 0.15 *
muDoubleScalarCos ( rtB . jsqo11gz4n ) ) + 0.15 * muDoubleScalarCos ( rtB .
fvygvwpi1d ) ; ce_a = 0.15 * muDoubleScalarSin ( rtB . jsqo11gz4n ) - 0.15 *
muDoubleScalarSin ( rtB . fvygvwpi1d ) ; ke_a = ( 0.108 - 0.15 *
muDoubleScalarCos ( rtB . jsqo11gz4n ) ) + 0.15 * muDoubleScalarCos ( rtB .
fvygvwpi1d ) ; le_a = 0.15 * muDoubleScalarSin ( rtB . jsqo11gz4n ) - 0.15 *
muDoubleScalarSin ( rtB . fvygvwpi1d ) ; me_a = ( 0.108 - 0.15 *
muDoubleScalarCos ( rtB . jsqo11gz4n ) ) + 0.15 * muDoubleScalarCos ( rtB .
fvygvwpi1d ) ; ne_a = 0.15 * muDoubleScalarSin ( rtB . jsqo11gz4n ) - 0.15 *
muDoubleScalarSin ( rtB . fvygvwpi1d ) ; oe_a = 0.15 * muDoubleScalarSin (
rtB . jsqo11gz4n ) - 0.15 * muDoubleScalarSin ( rtB . fvygvwpi1d ) ; pe_a = (
0.108 - 0.15 * muDoubleScalarCos ( rtB . jsqo11gz4n ) ) + 0.15 *
muDoubleScalarCos ( rtB . fvygvwpi1d ) ; oe_a = ( ( oe_a * oe_a + pe_a * pe_a
) + 0.0625 ) - 0.0625 ; pe_a = ( 0.108 - 0.15 * muDoubleScalarCos ( rtB .
jsqo11gz4n ) ) + 0.15 * muDoubleScalarCos ( rtB . fvygvwpi1d ) ; se_a = 0.15
* muDoubleScalarSin ( rtB . jsqo11gz4n ) - 0.15 * muDoubleScalarSin ( rtB .
fvygvwpi1d ) ; te_a = ( 0.108 - 0.15 * muDoubleScalarCos ( rtB . jsqo11gz4n )
) + 0.15 * muDoubleScalarCos ( rtB . fvygvwpi1d ) ; ue_a = 0.15 *
muDoubleScalarSin ( rtB . jsqo11gz4n ) - 0.15 * muDoubleScalarSin ( rtB .
fvygvwpi1d ) ; ve_a = 0.15 * muDoubleScalarSin ( rtB . jsqo11gz4n ) - 0.15 *
muDoubleScalarSin ( rtB . fvygvwpi1d ) ; we_a = ( 0.108 - 0.15 *
muDoubleScalarCos ( rtB . jsqo11gz4n ) ) + 0.15 * muDoubleScalarCos ( rtB .
fvygvwpi1d ) ; ve_a = ( ( ve_a * ve_a + we_a * we_a ) + 0.0625 ) - 0.0625 ;
we_a = ( 0.108 - 0.15 * muDoubleScalarCos ( rtB . jsqo11gz4n ) ) + 0.15 *
muDoubleScalarCos ( rtB . fvygvwpi1d ) ; af_a = 0.15 * muDoubleScalarSin (
rtB . jsqo11gz4n ) - 0.15 * muDoubleScalarSin ( rtB . fvygvwpi1d ) ; bf_a = (
0.108 - 0.15 * muDoubleScalarCos ( rtB . jsqo11gz4n ) ) + 0.15 *
muDoubleScalarCos ( rtB . fvygvwpi1d ) ; af_a = ( ( ( af_a * af_a + bf_a *
bf_a ) + 0.0625 ) - 0.0625 ) + ( ( 0.108 - 0.15 * muDoubleScalarCos ( rtB .
jsqo11gz4n ) ) + 0.15 * muDoubleScalarCos ( rtB . fvygvwpi1d ) ) * 0.5 ; bf_a
= 0.15 * muDoubleScalarSin ( rtB . jsqo11gz4n ) - 0.15 * muDoubleScalarSin (
rtB . fvygvwpi1d ) ; ef_a = 0.15 * muDoubleScalarSin ( rtB . jsqo11gz4n ) -
0.15 * muDoubleScalarSin ( rtB . fvygvwpi1d ) ; ff_a = ( 0.108 - 0.15 *
muDoubleScalarCos ( rtB . jsqo11gz4n ) ) + 0.15 * muDoubleScalarCos ( rtB .
fvygvwpi1d ) ; ef_a = ( ( ef_a * ef_a + ff_a * ff_a ) + 0.0625 ) - 0.0625 ;
ff_a = ( 0.108 - 0.15 * muDoubleScalarCos ( rtB . jsqo11gz4n ) ) + 0.15 *
muDoubleScalarCos ( rtB . fvygvwpi1d ) ; bf_a = muDoubleScalarSqrt ( ( bf_a *
bf_a * 0.25 - ef_a * ef_a ) + ff_a * ff_a * 0.25 ) - ( 0.15 *
muDoubleScalarSin ( rtB . jsqo11gz4n ) - 0.15 * muDoubleScalarSin ( rtB .
fvygvwpi1d ) ) * 0.5 ; ef_a = 0.15 * muDoubleScalarSin ( rtB . jsqo11gz4n ) -
0.15 * muDoubleScalarSin ( rtB . fvygvwpi1d ) ; ff_a = ( 0.108 - 0.15 *
muDoubleScalarCos ( rtB . jsqo11gz4n ) ) + 0.15 * muDoubleScalarCos ( rtB .
fvygvwpi1d ) ; ef_a = ( ( ( ef_a * ef_a + ff_a * ff_a ) + 0.0625 ) - 0.0625 )
+ ( ( 0.108 - 0.15 * muDoubleScalarCos ( rtB . jsqo11gz4n ) ) + 0.15 *
muDoubleScalarCos ( rtB . fvygvwpi1d ) ) * 0.5 ; ff_a = 0.15 *
muDoubleScalarSin ( rtB . jsqo11gz4n ) - 0.15 * muDoubleScalarSin ( rtB .
fvygvwpi1d ) ; nf_a = 0.15 * muDoubleScalarSin ( rtB . jsqo11gz4n ) - 0.15 *
muDoubleScalarSin ( rtB . fvygvwpi1d ) ; of_a = ( 0.108 - 0.15 *
muDoubleScalarCos ( rtB . jsqo11gz4n ) ) + 0.15 * muDoubleScalarCos ( rtB .
fvygvwpi1d ) ; nf_a = ( ( nf_a * nf_a + of_a * of_a ) + 0.0625 ) - 0.0625 ;
of_a = ( 0.108 - 0.15 * muDoubleScalarCos ( rtB . jsqo11gz4n ) ) + 0.15 *
muDoubleScalarCos ( rtB . fvygvwpi1d ) ; rf_a = 0.15 * muDoubleScalarSin (
rtB . jsqo11gz4n ) - 0.15 * muDoubleScalarSin ( rtB . fvygvwpi1d ) ; sf_a = (
0.108 - 0.15 * muDoubleScalarCos ( rtB . jsqo11gz4n ) ) + 0.15 *
muDoubleScalarCos ( rtB . fvygvwpi1d ) ; tf_a = 0.15 * muDoubleScalarSin (
rtB . jsqo11gz4n ) - 0.15 * muDoubleScalarSin ( rtB . fvygvwpi1d ) ; uf_a =
0.15 * muDoubleScalarSin ( rtB . jsqo11gz4n ) - 0.15 * muDoubleScalarSin (
rtB . fvygvwpi1d ) ; vf_a = ( 0.108 - 0.15 * muDoubleScalarCos ( rtB .
jsqo11gz4n ) ) + 0.15 * muDoubleScalarCos ( rtB . fvygvwpi1d ) ; uf_a = ( (
uf_a * uf_a + vf_a * vf_a ) + 0.0625 ) - 0.0625 ; vf_a = ( 0.108 - 0.15 *
muDoubleScalarCos ( rtB . jsqo11gz4n ) ) + 0.15 * muDoubleScalarCos ( rtB .
fvygvwpi1d ) ; yf_a = 0.15 * muDoubleScalarSin ( rtB . jsqo11gz4n ) - 0.15 *
muDoubleScalarSin ( rtB . fvygvwpi1d ) ; ag_a = ( 0.108 - 0.15 *
muDoubleScalarCos ( rtB . jsqo11gz4n ) ) + 0.15 * muDoubleScalarCos ( rtB .
fvygvwpi1d ) ; bg_a = 0.15 * muDoubleScalarSin ( rtB . jsqo11gz4n ) - 0.15 *
muDoubleScalarSin ( rtB . fvygvwpi1d ) ; cg_a = 0.15 * muDoubleScalarSin (
rtB . jsqo11gz4n ) - 0.15 * muDoubleScalarSin ( rtB . fvygvwpi1d ) ; dg_a = (
0.108 - 0.15 * muDoubleScalarCos ( rtB . jsqo11gz4n ) ) + 0.15 *
muDoubleScalarCos ( rtB . fvygvwpi1d ) ; cg_a = ( ( cg_a * cg_a + dg_a * dg_a
) + 0.0625 ) - 0.0625 ; dg_a = ( 0.108 - 0.15 * muDoubleScalarCos ( rtB .
jsqo11gz4n ) ) + 0.15 * muDoubleScalarCos ( rtB . fvygvwpi1d ) ; gg_a = 0.15
* muDoubleScalarSin ( rtB . jsqo11gz4n ) - 0.15 * muDoubleScalarSin ( rtB .
fvygvwpi1d ) ; hg_a = ( 0.108 - 0.15 * muDoubleScalarCos ( rtB . jsqo11gz4n )
) + 0.15 * muDoubleScalarCos ( rtB . fvygvwpi1d ) ; ig_a = 0.15 *
muDoubleScalarSin ( rtB . jsqo11gz4n ) - 0.15 * muDoubleScalarSin ( rtB .
fvygvwpi1d ) ; jg_a = ( 0.108 - 0.15 * muDoubleScalarCos ( rtB . jsqo11gz4n )
) + 0.15 * muDoubleScalarCos ( rtB . fvygvwpi1d ) ; kg_a = 0.15 *
muDoubleScalarSin ( rtB . jsqo11gz4n ) - 0.15 * muDoubleScalarSin ( rtB .
fvygvwpi1d ) ; lg_a = 0.15 * muDoubleScalarSin ( rtB . jsqo11gz4n ) - 0.15 *
muDoubleScalarSin ( rtB . fvygvwpi1d ) ; mg_a = ( 0.108 - 0.15 *
muDoubleScalarCos ( rtB . jsqo11gz4n ) ) + 0.15 * muDoubleScalarCos ( rtB .
fvygvwpi1d ) ; lg_a = ( ( lg_a * lg_a + mg_a * mg_a ) + 0.0625 ) - 0.0625 ;
mg_a = ( 0.108 - 0.15 * muDoubleScalarCos ( rtB . jsqo11gz4n ) ) + 0.15 *
muDoubleScalarCos ( rtB . fvygvwpi1d ) ; pg_a = 0.15 * muDoubleScalarSin (
rtB . jsqo11gz4n ) - 0.15 * muDoubleScalarSin ( rtB . fvygvwpi1d ) ; qg_a = (
0.108 - 0.15 * muDoubleScalarCos ( rtB . jsqo11gz4n ) ) + 0.15 *
muDoubleScalarCos ( rtB . fvygvwpi1d ) ; rg_a = 0.15 * muDoubleScalarSin (
rtB . jsqo11gz4n ) - 0.15 * muDoubleScalarSin ( rtB . fvygvwpi1d ) ; sg_a =
0.15 * muDoubleScalarSin ( rtB . jsqo11gz4n ) - 0.15 * muDoubleScalarSin (
rtB . fvygvwpi1d ) ; tg_a = ( 0.108 - 0.15 * muDoubleScalarCos ( rtB .
jsqo11gz4n ) ) + 0.15 * muDoubleScalarCos ( rtB . fvygvwpi1d ) ; sg_a = ( (
sg_a * sg_a + tg_a * tg_a ) + 0.0625 ) - 0.0625 ; tg_a = ( 0.108 - 0.15 *
muDoubleScalarCos ( rtB . jsqo11gz4n ) ) + 0.15 * muDoubleScalarCos ( rtB .
fvygvwpi1d ) ; wg_a = 0.15 * muDoubleScalarSin ( rtB . jsqo11gz4n ) - 0.15 *
muDoubleScalarSin ( rtB . fvygvwpi1d ) ; xg_a = ( 0.108 - 0.15 *
muDoubleScalarCos ( rtB . jsqo11gz4n ) ) + 0.15 * muDoubleScalarCos ( rtB .
fvygvwpi1d ) ; wg_a = ( ( ( wg_a * wg_a + xg_a * xg_a ) + 0.0625 ) - 0.0625 )
+ ( ( 0.108 - 0.15 * muDoubleScalarCos ( rtB . jsqo11gz4n ) ) + 0.15 *
muDoubleScalarCos ( rtB . fvygvwpi1d ) ) * 0.5 ; xg_a = 0.15 *
muDoubleScalarSin ( rtB . jsqo11gz4n ) - 0.15 * muDoubleScalarSin ( rtB .
fvygvwpi1d ) ; bh_a = 0.15 * muDoubleScalarSin ( rtB . jsqo11gz4n ) - 0.15 *
muDoubleScalarSin ( rtB . fvygvwpi1d ) ; ch_a = ( 0.108 - 0.15 *
muDoubleScalarCos ( rtB . jsqo11gz4n ) ) + 0.15 * muDoubleScalarCos ( rtB .
fvygvwpi1d ) ; bh_a = ( ( bh_a * bh_a + ch_a * ch_a ) + 0.0625 ) - 0.0625 ;
ch_a = ( 0.108 - 0.15 * muDoubleScalarCos ( rtB . jsqo11gz4n ) ) + 0.15 *
muDoubleScalarCos ( rtB . fvygvwpi1d ) ; xg_a = muDoubleScalarSqrt ( ( xg_a *
xg_a * 0.25 - bh_a * bh_a ) + ch_a * ch_a * 0.25 ) - ( 0.15 *
muDoubleScalarSin ( rtB . jsqo11gz4n ) - 0.15 * muDoubleScalarSin ( rtB .
fvygvwpi1d ) ) * 0.5 ; bh_a = 0.15 * muDoubleScalarSin ( rtB . jsqo11gz4n ) -
0.15 * muDoubleScalarSin ( rtB . fvygvwpi1d ) ; ch_a = ( 0.108 - 0.15 *
muDoubleScalarCos ( rtB . jsqo11gz4n ) ) + 0.15 * muDoubleScalarCos ( rtB .
fvygvwpi1d ) ; bh_a = ( ( ( bh_a * bh_a + ch_a * ch_a ) + 0.0625 ) - 0.0625 )
+ ( ( 0.108 - 0.15 * muDoubleScalarCos ( rtB . jsqo11gz4n ) ) + 0.15 *
muDoubleScalarCos ( rtB . fvygvwpi1d ) ) * 0.5 ; ch_a = 0.15 *
muDoubleScalarSin ( rtB . jsqo11gz4n ) - 0.15 * muDoubleScalarSin ( rtB .
fvygvwpi1d ) ; kh_a = 0.15 * muDoubleScalarSin ( rtB . jsqo11gz4n ) - 0.15 *
muDoubleScalarSin ( rtB . fvygvwpi1d ) ; lh_a = ( 0.108 - 0.15 *
muDoubleScalarCos ( rtB . jsqo11gz4n ) ) + 0.15 * muDoubleScalarCos ( rtB .
fvygvwpi1d ) ; kh_a = ( ( kh_a * kh_a + lh_a * lh_a ) + 0.0625 ) - 0.0625 ;
lh_a = ( 0.108 - 0.15 * muDoubleScalarCos ( rtB . jsqo11gz4n ) ) + 0.15 *
muDoubleScalarCos ( rtB . fvygvwpi1d ) ; oh_a = 0.15 * muDoubleScalarSin (
rtB . jsqo11gz4n ) - 0.15 * muDoubleScalarSin ( rtB . fvygvwpi1d ) ; ph_a = (
0.108 - 0.15 * muDoubleScalarCos ( rtB . jsqo11gz4n ) ) + 0.15 *
muDoubleScalarCos ( rtB . fvygvwpi1d ) ; ch_a = ( muDoubleScalarCos (
muDoubleScalarAtan ( ( muDoubleScalarSqrt ( ( ch_a * ch_a * 0.25 - kh_a *
kh_a ) + lh_a * lh_a * 0.25 ) - ( 0.15 * muDoubleScalarSin ( rtB . jsqo11gz4n
) - 0.15 * muDoubleScalarSin ( rtB . fvygvwpi1d ) ) * 0.5 ) / ( ( ( ( oh_a *
oh_a + ph_a * ph_a ) + 0.0625 ) - 0.0625 ) + ( ( 0.108 - 0.15 *
muDoubleScalarCos ( rtB . jsqo11gz4n ) ) + 0.15 * muDoubleScalarCos ( rtB .
fvygvwpi1d ) ) * 0.5 ) ) * 2.0 ) * 0.25 - 0.054 ) + 0.15 * muDoubleScalarCos
( rtB . jsqo11gz4n ) ; kh_a = 0.15 * muDoubleScalarSin ( rtB . jsqo11gz4n ) -
0.15 * muDoubleScalarSin ( rtB . fvygvwpi1d ) ; lh_a = 0.15 *
muDoubleScalarSin ( rtB . jsqo11gz4n ) - 0.15 * muDoubleScalarSin ( rtB .
fvygvwpi1d ) ; oh_a = ( 0.108 - 0.15 * muDoubleScalarCos ( rtB . jsqo11gz4n )
) + 0.15 * muDoubleScalarCos ( rtB . fvygvwpi1d ) ; lh_a = ( ( lh_a * lh_a +
oh_a * oh_a ) + 0.0625 ) - 0.0625 ; oh_a = ( 0.108 - 0.15 * muDoubleScalarCos
( rtB . jsqo11gz4n ) ) + 0.15 * muDoubleScalarCos ( rtB . fvygvwpi1d ) ; ph_a
= 0.15 * muDoubleScalarSin ( rtB . jsqo11gz4n ) - 0.15 * muDoubleScalarSin (
rtB . fvygvwpi1d ) ; xh_a = ( 0.108 - 0.15 * muDoubleScalarCos ( rtB .
jsqo11gz4n ) ) + 0.15 * muDoubleScalarCos ( rtB . fvygvwpi1d ) ; kh_a =
muDoubleScalarSin ( muDoubleScalarAtan ( ( muDoubleScalarSqrt ( ( kh_a * kh_a
* 0.25 - lh_a * lh_a ) + oh_a * oh_a * 0.25 ) - ( 0.15 * muDoubleScalarSin (
rtB . jsqo11gz4n ) - 0.15 * muDoubleScalarSin ( rtB . fvygvwpi1d ) ) * 0.5 )
/ ( ( ( ( ph_a * ph_a + xh_a * xh_a ) + 0.0625 ) - 0.0625 ) + ( ( 0.108 -
0.15 * muDoubleScalarCos ( rtB . jsqo11gz4n ) ) + 0.15 * muDoubleScalarCos (
rtB . fvygvwpi1d ) ) * 0.5 ) ) * 2.0 ) * 0.25 + 0.15 * muDoubleScalarSin (
rtB . jsqo11gz4n ) ; lh_a = 0.15 * muDoubleScalarSin ( rtB . jsqo11gz4n ) -
0.15 * muDoubleScalarSin ( rtB . fvygvwpi1d ) ; oh_a = 0.15 *
muDoubleScalarSin ( rtB . jsqo11gz4n ) - 0.15 * muDoubleScalarSin ( rtB .
fvygvwpi1d ) ; ph_a = ( 0.108 - 0.15 * muDoubleScalarCos ( rtB . jsqo11gz4n )
) + 0.15 * muDoubleScalarCos ( rtB . fvygvwpi1d ) ; oh_a = ( ( oh_a * oh_a +
ph_a * ph_a ) + 0.0625 ) - 0.0625 ; ph_a = ( 0.108 - 0.15 * muDoubleScalarCos
( rtB . jsqo11gz4n ) ) + 0.15 * muDoubleScalarCos ( rtB . fvygvwpi1d ) ; xh_a
= 0.15 * muDoubleScalarSin ( rtB . jsqo11gz4n ) - 0.15 * muDoubleScalarSin (
rtB . fvygvwpi1d ) ; gi_a = ( 0.108 - 0.15 * muDoubleScalarCos ( rtB .
jsqo11gz4n ) ) + 0.15 * muDoubleScalarCos ( rtB . fvygvwpi1d ) ; lh_a = (
muDoubleScalarCos ( muDoubleScalarAtan ( ( muDoubleScalarSqrt ( ( lh_a * lh_a
* 0.25 - oh_a * oh_a ) + ph_a * ph_a * 0.25 ) - ( 0.15 * muDoubleScalarSin (
rtB . jsqo11gz4n ) - 0.15 * muDoubleScalarSin ( rtB . fvygvwpi1d ) ) * 0.5 )
/ ( ( ( ( xh_a * xh_a + gi_a * gi_a ) + 0.0625 ) - 0.0625 ) + ( ( 0.108 -
0.15 * muDoubleScalarCos ( rtB . jsqo11gz4n ) ) + 0.15 * muDoubleScalarCos (
rtB . fvygvwpi1d ) ) * 0.5 ) ) * 2.0 ) * 0.25 - 0.054 ) + 0.15 *
muDoubleScalarCos ( rtB . jsqo11gz4n ) ; kfo1vde04o = ( ( ( ( ( ( ( 0.15 *
muDoubleScalarSin ( rtB . jsqo11gz4n ) - 0.15 * muDoubleScalarSin ( rtB .
fvygvwpi1d ) ) * ( 0.075 * muDoubleScalarCos ( rtB . jsqo11gz4n ) ) - ( ( (
0.108 - 0.15 * muDoubleScalarCos ( rtB . jsqo11gz4n ) ) + 0.15 *
muDoubleScalarCos ( rtB . fvygvwpi1d ) ) * ( 0.3 * muDoubleScalarSin ( rtB .
jsqo11gz4n ) ) + ( 0.15 * muDoubleScalarSin ( rtB . jsqo11gz4n ) - 0.15 *
muDoubleScalarSin ( rtB . fvygvwpi1d ) ) * ( 0.3 * muDoubleScalarCos ( rtB .
jsqo11gz4n ) ) ) * 2.0 * ( ( ( ec_a * ec_a + fc_a * fc_a ) + 0.0625 ) -
0.0625 ) ) + ( ( 0.108 - 0.15 * muDoubleScalarCos ( rtB . jsqo11gz4n ) ) +
0.15 * muDoubleScalarCos ( rtB . fvygvwpi1d ) ) * ( 0.075 * muDoubleScalarSin
( rtB . jsqo11gz4n ) ) ) / ( muDoubleScalarSqrt ( ( gc_a * gc_a * 0.25 - hc_a
* hc_a ) + ic_a * ic_a * 0.25 ) * 2.0 ) - 0.075 * muDoubleScalarCos ( rtB .
jsqo11gz4n ) ) / ( ( ( ( lc_a * lc_a + mc_a * mc_a ) + 0.0625 ) - 0.0625 ) +
( ( 0.108 - 0.15 * muDoubleScalarCos ( rtB . jsqo11gz4n ) ) + 0.15 *
muDoubleScalarCos ( rtB . fvygvwpi1d ) ) * 0.5 ) - ( muDoubleScalarSqrt ( (
nc_a * nc_a * 0.25 - oc_a * oc_a ) + pc_a * pc_a * 0.25 ) - ( 0.15 *
muDoubleScalarSin ( rtB . jsqo11gz4n ) - 0.15 * muDoubleScalarSin ( rtB .
fvygvwpi1d ) ) * 0.5 ) * ( ( ( 0.15 * muDoubleScalarSin ( rtB . jsqo11gz4n )
- 0.15 * muDoubleScalarSin ( rtB . fvygvwpi1d ) ) * ( 0.3 * muDoubleScalarCos
( rtB . jsqo11gz4n ) ) + 0.075 * muDoubleScalarSin ( rtB . jsqo11gz4n ) ) + (
( 0.108 - 0.15 * muDoubleScalarCos ( rtB . jsqo11gz4n ) ) + 0.15 *
muDoubleScalarCos ( rtB . fvygvwpi1d ) ) * ( 0.3 * muDoubleScalarSin ( rtB .
jsqo11gz4n ) ) ) / ( sc_a * sc_a ) ) * ( muDoubleScalarSin (
muDoubleScalarAtan ( ( muDoubleScalarSqrt ( ( wb_a * wb_a * 0.25 - xb_a *
xb_a ) + yb_a * yb_a * 0.25 ) - ( 0.15 * muDoubleScalarSin ( rtB . jsqo11gz4n
) - 0.15 * muDoubleScalarSin ( rtB . fvygvwpi1d ) ) * 0.5 ) / ( ( ( ( cc_a *
cc_a + dc_a * dc_a ) + 0.0625 ) - 0.0625 ) + ( ( 0.108 - 0.15 *
muDoubleScalarCos ( rtB . jsqo11gz4n ) ) + 0.15 * muDoubleScalarCos ( rtB .
fvygvwpi1d ) ) * 0.5 ) ) * 2.0 ) * 0.5 ) / ( tc_a * tc_a / ( wc_a * wc_a ) +
1.0 ) + 0.15 * muDoubleScalarSin ( rtB . jsqo11gz4n ) ) * ( muDoubleScalarSin
( muDoubleScalarAtan ( ( muDoubleScalarSqrt ( ( pb_a * pb_a * 0.25 - qb_a *
qb_a ) + rb_a * rb_a * 0.25 ) - ( 0.15 * muDoubleScalarSin ( rtB . jsqo11gz4n
) - 0.15 * muDoubleScalarSin ( rtB . fvygvwpi1d ) ) * 0.5 ) / ( ( ( ( ub_a *
ub_a + vb_a * vb_a ) + 0.0625 ) - 0.0625 ) + ( ( 0.108 - 0.15 *
muDoubleScalarCos ( rtB . jsqo11gz4n ) ) + 0.15 * muDoubleScalarCos ( rtB .
fvygvwpi1d ) ) * 0.5 ) ) * 2.0 ) * 0.25 + 0.15 * muDoubleScalarSin ( rtB .
jsqo11gz4n ) ) / ( xc_a * xc_a ) + ( ( ( ( ( ( 0.15 * muDoubleScalarSin ( rtB
. jsqo11gz4n ) - 0.15 * muDoubleScalarSin ( rtB . fvygvwpi1d ) ) * ( 0.075 *
muDoubleScalarCos ( rtB . jsqo11gz4n ) ) - ( ( ( 0.108 - 0.15 *
muDoubleScalarCos ( rtB . jsqo11gz4n ) ) + 0.15 * muDoubleScalarCos ( rtB .
fvygvwpi1d ) ) * ( 0.3 * muDoubleScalarSin ( rtB . jsqo11gz4n ) ) + ( 0.15 *
muDoubleScalarSin ( rtB . jsqo11gz4n ) - 0.15 * muDoubleScalarSin ( rtB .
fvygvwpi1d ) ) * ( 0.3 * muDoubleScalarCos ( rtB . jsqo11gz4n ) ) ) * 2.0 * (
( ( XD * XD + XB * XB ) + 0.0625 ) - 0.0625 ) ) + ( ( 0.108 - 0.15 *
muDoubleScalarCos ( rtB . jsqo11gz4n ) ) + 0.15 * muDoubleScalarCos ( rtB .
fvygvwpi1d ) ) * ( 0.075 * muDoubleScalarSin ( rtB . jsqo11gz4n ) ) ) / (
muDoubleScalarSqrt ( ( B0 * B0 * 0.25 - lBD * lBD ) + l_a * l_a * 0.25 ) *
2.0 ) - 0.075 * muDoubleScalarCos ( rtB . jsqo11gz4n ) ) / ( ( ( ( o_a * o_a
+ p_a * p_a ) + 0.0625 ) - 0.0625 ) + ( ( 0.108 - 0.15 * muDoubleScalarCos (
rtB . jsqo11gz4n ) ) + 0.15 * muDoubleScalarCos ( rtB . fvygvwpi1d ) ) * 0.5
) - ( muDoubleScalarSqrt ( ( q_a * q_a * 0.25 - r_a * r_a ) + s_a * s_a *
0.25 ) - ( 0.15 * muDoubleScalarSin ( rtB . jsqo11gz4n ) - 0.15 *
muDoubleScalarSin ( rtB . fvygvwpi1d ) ) * 0.5 ) * ( ( ( 0.15 *
muDoubleScalarSin ( rtB . jsqo11gz4n ) - 0.15 * muDoubleScalarSin ( rtB .
fvygvwpi1d ) ) * ( 0.3 * muDoubleScalarCos ( rtB . jsqo11gz4n ) ) + 0.075 *
muDoubleScalarSin ( rtB . jsqo11gz4n ) ) + ( ( 0.108 - 0.15 *
muDoubleScalarCos ( rtB . jsqo11gz4n ) ) + 0.15 * muDoubleScalarCos ( rtB .
fvygvwpi1d ) ) * ( 0.3 * muDoubleScalarSin ( rtB . jsqo11gz4n ) ) ) / ( v_a *
v_a ) ) * ( muDoubleScalarCos ( muDoubleScalarAtan ( ( muDoubleScalarSqrt ( (
kfo1vde04o * kfo1vde04o * 0.25 - YC * YC ) + A0 * A0 * 0.25 ) - ( 0.15 *
muDoubleScalarSin ( rtB . jsqo11gz4n ) - 0.15 * muDoubleScalarSin ( rtB .
fvygvwpi1d ) ) * 0.5 ) / ( ( ( ( YD * YD + YB * YB ) + 0.0625 ) - 0.0625 ) +
( ( 0.108 - 0.15 * muDoubleScalarCos ( rtB . jsqo11gz4n ) ) + 0.15 *
muDoubleScalarCos ( rtB . fvygvwpi1d ) ) * 0.5 ) ) * 2.0 ) * 0.5 ) / ( w_a *
w_a / ( ab_a * ab_a ) + 1.0 ) + 0.15 * muDoubleScalarCos ( rtB . jsqo11gz4n )
) / ( ( muDoubleScalarCos ( muDoubleScalarAtan ( ( muDoubleScalarSqrt ( (
bb_a * bb_a * 0.25 - jb_a * jb_a ) + kb_a * kb_a * 0.25 ) - ( 0.15 *
muDoubleScalarSin ( rtB . jsqo11gz4n ) - 0.15 * muDoubleScalarSin ( rtB .
fvygvwpi1d ) ) * 0.5 ) / ( ( ( ( nb_a * nb_a + ob_a * ob_a ) + 0.0625 ) -
0.0625 ) + ( ( 0.108 - 0.15 * muDoubleScalarCos ( rtB . jsqo11gz4n ) ) + 0.15
* muDoubleScalarCos ( rtB . fvygvwpi1d ) ) * 0.5 ) ) * 2.0 ) * 0.25 - 0.054 )
+ 0.15 * muDoubleScalarCos ( rtB . jsqo11gz4n ) ) ) * rtB . ne53mrvzfq [ 3 ]
/ ( gd_a * gd_a / ( hd_a * hd_a ) + 1.0 ) - ( ( ( ( ( ( 0.15 *
muDoubleScalarSin ( rtB . jsqo11gz4n ) - 0.15 * muDoubleScalarSin ( rtB .
fvygvwpi1d ) ) * ( 0.075 * muDoubleScalarCos ( rtB . fvygvwpi1d ) ) - ( ( (
0.108 - 0.15 * muDoubleScalarCos ( rtB . jsqo11gz4n ) ) + 0.15 *
muDoubleScalarCos ( rtB . fvygvwpi1d ) ) * ( 0.3 * muDoubleScalarSin ( rtB .
fvygvwpi1d ) ) + ( 0.15 * muDoubleScalarSin ( rtB . jsqo11gz4n ) - 0.15 *
muDoubleScalarSin ( rtB . fvygvwpi1d ) ) * ( 0.3 * muDoubleScalarCos ( rtB .
fvygvwpi1d ) ) ) * 2.0 * ( ( ( le_a * le_a + me_a * me_a ) + 0.0625 ) -
0.0625 ) ) + ( ( 0.108 - 0.15 * muDoubleScalarCos ( rtB . jsqo11gz4n ) ) +
0.15 * muDoubleScalarCos ( rtB . fvygvwpi1d ) ) * ( 0.075 * muDoubleScalarSin
( rtB . fvygvwpi1d ) ) ) / ( muDoubleScalarSqrt ( ( ne_a * ne_a * 0.25 - oe_a
* oe_a ) + pe_a * pe_a * 0.25 ) * 2.0 ) - 0.075 * muDoubleScalarCos ( rtB .
fvygvwpi1d ) ) / ( ( ( ( se_a * se_a + te_a * te_a ) + 0.0625 ) - 0.0625 ) +
( ( 0.108 - 0.15 * muDoubleScalarCos ( rtB . jsqo11gz4n ) ) + 0.15 *
muDoubleScalarCos ( rtB . fvygvwpi1d ) ) * 0.5 ) - ( muDoubleScalarSqrt ( (
ue_a * ue_a * 0.25 - ve_a * ve_a ) + we_a * we_a * 0.25 ) - ( 0.15 *
muDoubleScalarSin ( rtB . jsqo11gz4n ) - 0.15 * muDoubleScalarSin ( rtB .
fvygvwpi1d ) ) * 0.5 ) * ( ( ( 0.15 * muDoubleScalarSin ( rtB . jsqo11gz4n )
- 0.15 * muDoubleScalarSin ( rtB . fvygvwpi1d ) ) * ( 0.3 * muDoubleScalarCos
( rtB . fvygvwpi1d ) ) + 0.075 * muDoubleScalarSin ( rtB . fvygvwpi1d ) ) + (
( 0.108 - 0.15 * muDoubleScalarCos ( rtB . jsqo11gz4n ) ) + 0.15 *
muDoubleScalarCos ( rtB . fvygvwpi1d ) ) * ( 0.3 * muDoubleScalarSin ( rtB .
fvygvwpi1d ) ) ) / ( af_a * af_a ) ) * ( muDoubleScalarCos (
muDoubleScalarAtan ( ( muDoubleScalarSqrt ( ( kd_a * kd_a * 0.25 - ld_a *
ld_a ) + td_a * td_a * 0.25 ) - ( 0.15 * muDoubleScalarSin ( rtB . jsqo11gz4n
) - 0.15 * muDoubleScalarSin ( rtB . fvygvwpi1d ) ) * 0.5 ) / ( ( ( ( ce_a *
ce_a + ke_a * ke_a ) + 0.0625 ) - 0.0625 ) + ( ( 0.108 - 0.15 *
muDoubleScalarCos ( rtB . jsqo11gz4n ) ) + 0.15 * muDoubleScalarCos ( rtB .
fvygvwpi1d ) ) * 0.5 ) ) * 2.0 ) * 0.5 ) / ( ( ( muDoubleScalarCos (
muDoubleScalarAtan ( ( muDoubleScalarSqrt ( ( ff_a * ff_a * 0.25 - nf_a *
nf_a ) + of_a * of_a * 0.25 ) - ( 0.15 * muDoubleScalarSin ( rtB . jsqo11gz4n
) - 0.15 * muDoubleScalarSin ( rtB . fvygvwpi1d ) ) * 0.5 ) / ( ( ( ( rf_a *
rf_a + sf_a * sf_a ) + 0.0625 ) - 0.0625 ) + ( ( 0.108 - 0.15 *
muDoubleScalarCos ( rtB . jsqo11gz4n ) ) + 0.15 * muDoubleScalarCos ( rtB .
fvygvwpi1d ) ) * 0.5 ) ) * 2.0 ) * 0.25 - 0.054 ) + 0.15 * muDoubleScalarCos
( rtB . jsqo11gz4n ) ) * ( bf_a * bf_a / ( ef_a * ef_a ) + 1.0 ) ) + ( ( ( (
( 0.15 * muDoubleScalarSin ( rtB . jsqo11gz4n ) - 0.15 * muDoubleScalarSin (
rtB . fvygvwpi1d ) ) * ( 0.075 * muDoubleScalarCos ( rtB . fvygvwpi1d ) ) - (
( ( 0.108 - 0.15 * muDoubleScalarCos ( rtB . jsqo11gz4n ) ) + 0.15 *
muDoubleScalarCos ( rtB . fvygvwpi1d ) ) * ( 0.3 * muDoubleScalarSin ( rtB .
fvygvwpi1d ) ) + ( 0.15 * muDoubleScalarSin ( rtB . jsqo11gz4n ) - 0.15 *
muDoubleScalarSin ( rtB . fvygvwpi1d ) ) * ( 0.3 * muDoubleScalarCos ( rtB .
fvygvwpi1d ) ) ) * 2.0 * ( ( ( ig_a * ig_a + jg_a * jg_a ) + 0.0625 ) -
0.0625 ) ) + ( ( 0.108 - 0.15 * muDoubleScalarCos ( rtB . jsqo11gz4n ) ) +
0.15 * muDoubleScalarCos ( rtB . fvygvwpi1d ) ) * ( 0.075 * muDoubleScalarSin
( rtB . fvygvwpi1d ) ) ) / ( muDoubleScalarSqrt ( ( kg_a * kg_a * 0.25 - lg_a
* lg_a ) + mg_a * mg_a * 0.25 ) * 2.0 ) - 0.075 * muDoubleScalarCos ( rtB .
fvygvwpi1d ) ) / ( ( ( ( pg_a * pg_a + qg_a * qg_a ) + 0.0625 ) - 0.0625 ) +
( ( 0.108 - 0.15 * muDoubleScalarCos ( rtB . jsqo11gz4n ) ) + 0.15 *
muDoubleScalarCos ( rtB . fvygvwpi1d ) ) * 0.5 ) - ( muDoubleScalarSqrt ( (
rg_a * rg_a * 0.25 - sg_a * sg_a ) + tg_a * tg_a * 0.25 ) - ( 0.15 *
muDoubleScalarSin ( rtB . jsqo11gz4n ) - 0.15 * muDoubleScalarSin ( rtB .
fvygvwpi1d ) ) * 0.5 ) * ( ( ( 0.15 * muDoubleScalarSin ( rtB . jsqo11gz4n )
- 0.15 * muDoubleScalarSin ( rtB . fvygvwpi1d ) ) * ( 0.3 * muDoubleScalarCos
( rtB . fvygvwpi1d ) ) + 0.075 * muDoubleScalarSin ( rtB . fvygvwpi1d ) ) + (
( 0.108 - 0.15 * muDoubleScalarCos ( rtB . jsqo11gz4n ) ) + 0.15 *
muDoubleScalarCos ( rtB . fvygvwpi1d ) ) * ( 0.3 * muDoubleScalarSin ( rtB .
fvygvwpi1d ) ) ) / ( wg_a * wg_a ) ) * ( ( muDoubleScalarSin (
muDoubleScalarAtan ( ( muDoubleScalarSqrt ( ( bg_a * bg_a * 0.25 - cg_a *
cg_a ) + dg_a * dg_a * 0.25 ) - ( 0.15 * muDoubleScalarSin ( rtB . jsqo11gz4n
) - 0.15 * muDoubleScalarSin ( rtB . fvygvwpi1d ) ) * 0.5 ) / ( ( ( ( gg_a *
gg_a + hg_a * hg_a ) + 0.0625 ) - 0.0625 ) + ( ( 0.108 - 0.15 *
muDoubleScalarCos ( rtB . jsqo11gz4n ) ) + 0.15 * muDoubleScalarCos ( rtB .
fvygvwpi1d ) ) * 0.5 ) ) * 2.0 ) * 0.25 + 0.15 * muDoubleScalarSin ( rtB .
jsqo11gz4n ) ) * ( muDoubleScalarSin ( muDoubleScalarAtan ( (
muDoubleScalarSqrt ( ( tf_a * tf_a * 0.25 - uf_a * uf_a ) + vf_a * vf_a *
0.25 ) - ( 0.15 * muDoubleScalarSin ( rtB . jsqo11gz4n ) - 0.15 *
muDoubleScalarSin ( rtB . fvygvwpi1d ) ) * 0.5 ) / ( ( ( ( yf_a * yf_a + ag_a
* ag_a ) + 0.0625 ) - 0.0625 ) + ( ( 0.108 - 0.15 * muDoubleScalarCos ( rtB .
jsqo11gz4n ) ) + 0.15 * muDoubleScalarCos ( rtB . fvygvwpi1d ) ) * 0.5 ) ) *
2.0 ) * 0.5 ) ) / ( ( xg_a * xg_a / ( bh_a * bh_a ) + 1.0 ) * ( ch_a * ch_a )
) ) * rtB . ne53mrvzfq [ 5 ] / ( kh_a * kh_a / ( lh_a * lh_a ) + 1.0 ) ; rtB
. nkvqeuwnam = ( 0.0 - kfo1vde04o ) - rtB . ne53mrvzfq [ 7 ] ; rtB .
cufla4blkp = rtP . Gain38_Gain * rtB . nkvqeuwnam ; rtB . a5rxjeqmnm = rtP .
Gain22_Gain * rtB . ne53mrvzfq [ 6 ] ; YD = 0.15 * muDoubleScalarSin ( rtB .
fvygvwpi1d ) ; YB = 0.15 * muDoubleScalarSin ( rtB . jsqo11gz4n ) ; XD = 0.15
* muDoubleScalarCos ( rtB . fvygvwpi1d ) + 0.108 ; XB = 0.15 *
muDoubleScalarCos ( rtB . jsqo11gz4n ) ; lBD = muDoubleScalarSqrt ( ( XD - XB
) * ( XD - XB ) + ( YD - YB ) * ( YD - YB ) ) ; A0 = ( XD - XB ) * 0.5 ; B0 =
( YD - YB ) * 0.5 ; lBD = ( lBD * lBD + 0.0625 ) - 0.0625 ; A0 =
muDoubleScalarAtan2 ( muDoubleScalarSqrt ( ( A0 * A0 + B0 * B0 ) - lBD * lBD
) + B0 , A0 + lBD ) * 2.0 ; kfo1vde04o = 0.15 * muDoubleScalarCos ( rtB .
jsqo11gz4n ) + 0.25 * muDoubleScalarCos ( A0 ) ; YC = 0.15 *
muDoubleScalarSin ( rtB . jsqo11gz4n ) + 0.25 * muDoubleScalarSin ( A0 ) ;
rtB . lp2kjlaox2 = ( rtP . Constant10_Value - muDoubleScalarAtan2 ( YC ,
kfo1vde04o - 0.054 ) ) - rtB . ne53mrvzfq [ 6 ] ; rtB . d4fftorabg = rtP .
Gain36_Gain * rtB . lp2kjlaox2 ; rtB . o0acnzn2gn = rtX . cbywjryyoi ; rtB .
lno5gwkvsm [ 0 ] = rtB . lp2kjlaox2 ; rtB . lno5gwkvsm [ 1 ] = rtB .
nkvqeuwnam ; rtB . lno5gwkvsm [ 2 ] = rtB . o0acnzn2gn ; rtB . lno5gwkvsm [ 3
] = rtB . ne53mrvzfq [ 12 ] ; rtB . lno5gwkvsm [ 4 ] = rtB . ne53mrvzfq [ 6 ]
; rtB . lno5gwkvsm [ 5 ] = rtB . ne53mrvzfq [ 7 ] ; for ( i = 0 ; i < 2 ; i
++ ) { rtB . m0bxxy51bt [ i ] = 0.0 ; for ( i_p = 0 ; i_p < 6 ; i_p ++ ) {
rtB . m0bxxy51bt [ i ] += rtP . K [ ( i_p << 1 ) + i ] * rtB . lno5gwkvsm [
i_p ] ; } } if ( ssIsSampleHit ( rtS , 1 , 0 ) ) { rtDW . kbppfzielz = (
ssGetTaskTime ( rtS , 1 ) >= rtP . jump_control_jump_time ) ; rtDW .
acajdtsjkp = ( ssGetTaskTime ( rtS , 1 ) >= rtP . jump_control_jump_time +
0.2 ) ; rtDW . pkcg1jz5ka = ( ssGetTaskTime ( rtS , 1 ) >= rtP .
jump_control_jump_time + 0.15 ) ; rtDW . ei5ynuzssw = ( ssGetTaskTime ( rtS ,
1 ) >= rtP . jump_control_jump_time + 0.4 ) ; if ( rtDW . kbppfzielz == 1 ) {
XB = rtP . Step_YFinal ; } else { XB = rtP . Step_Y0 ; } if ( rtDW .
acajdtsjkp == 1 ) { B0 = rtP . Step16_YFinal ; } else { B0 = rtP . Step16_Y0
; } if ( rtDW . pkcg1jz5ka == 1 ) { lBD = rtP . Step1_YFinal ; } else { lBD =
rtP . Step1_Y0 ; } if ( rtDW . ei5ynuzssw == 1 ) { l_a = rtP . Step13_YFinal
; } else { l_a = rtP . Step13_Y0 ; } rtB . ct5ptiiqfm = ( XB * B0 - lBD * l_a
) + rtB . bdqfv1d2cw ; rtDW . ekmth0nsne = ( ssGetTaskTime ( rtS , 1 ) >= rtP
. jump_control_jump_time ) ; rtDW . b15ck32r2h = ( ssGetTaskTime ( rtS , 1 )
>= rtP . jump_control_jump_time + 0.2 ) ; rtDW . gy03yk4tjx = ( ssGetTaskTime
( rtS , 1 ) >= rtP . jump_control_jump_time + 0.15 ) ; rtDW . dgksrckq3m = (
ssGetTaskTime ( rtS , 1 ) >= rtP . jump_control_jump_time + 0.4 ) ; if ( rtDW
. ekmth0nsne == 1 ) { XB = rtP . Step6_YFinal ; } else { XB = rtP . Step6_Y0
; } if ( rtDW . b15ck32r2h == 1 ) { B0 = rtP . Step14_YFinal ; } else { B0 =
rtP . Step14_Y0 ; } if ( rtDW . gy03yk4tjx == 1 ) { lBD = rtP . Step7_YFinal
; } else { lBD = rtP . Step7_Y0 ; } if ( rtDW . dgksrckq3m == 1 ) { l_a = rtP
. Step12_YFinal ; } else { l_a = rtP . Step12_Y0 ; } rtB . egyva5trzl = ( XB
* B0 - lBD * l_a ) + rtB . lbhtwulbcm ; rtDW . cojkg3rcgw = ( ssGetTaskTime (
rtS , 1 ) >= rtP . jump_control_jump_time ) ; rtDW . a04qfypmbt = (
ssGetTaskTime ( rtS , 1 ) >= rtP . jump_control_jump_time + 0.2 ) ; rtDW .
hjkpjosmn0 = ( ssGetTaskTime ( rtS , 1 ) >= rtP . jump_control_jump_time +
0.15 ) ; rtDW . huukfn1whc = ( ssGetTaskTime ( rtS , 1 ) >= rtP .
jump_control_jump_time + 0.4 ) ; if ( rtDW . cojkg3rcgw == 1 ) { XB = rtP .
Step8_YFinal ; } else { XB = rtP . Step8_Y0 ; } if ( rtDW . a04qfypmbt == 1 )
{ B0 = rtP . Step15_YFinal ; } else { B0 = rtP . Step15_Y0 ; } if ( rtDW .
hjkpjosmn0 == 1 ) { lBD = rtP . Step9_YFinal ; } else { lBD = rtP . Step9_Y0
; } if ( rtDW . huukfn1whc == 1 ) { l_a = rtP . Step11_YFinal ; } else { l_a
= rtP . Step11_Y0 ; } rtB . hhykaoe4za = XB * B0 - lBD * l_a ; } rtB .
emby3igyk1 = rtP . Constant_Value - rtB . p1iy05u0ee ; rtB . lunhybn4j1 = rtP
. PIDController3_P * rtB . emby3igyk1 ; rtB . bibxaqp42j = rtX . b2hgpxvjzc ;
rtB . ngl2rc54wx = rtP . PIDController3_D * rtB . emby3igyk1 ; rtB .
mkoprhigc1 = rtX . abxab3djbi ; rtB . p4cj0opfml = rtB . ngl2rc54wx - rtB .
mkoprhigc1 ; rtB . kxvii2moqu = rtP . PIDController3_N * rtB . p4cj0opfml ;
rtB . dz5u3bubv3 = ( rtB . lunhybn4j1 + rtB . bibxaqp42j ) + rtB . kxvii2moqu
; rtB . b0mlwzzkzn = rtB . dz5u3bubv3 + rtP . Value ; if ( ssIsMajorTimeStep
( rtS ) ) { rtDW . bj5n1250ac = rtB . b0mlwzzkzn >= rtP .
Saturation5_UpperSat ? 1 : rtB . b0mlwzzkzn > rtP . Saturation5_LowerSat ? 0
: - 1 ; } rtB . bpbscqkzum = rtDW . bj5n1250ac == 1 ? rtP .
Saturation5_UpperSat : rtDW . bj5n1250ac == - 1 ? rtP . Saturation5_LowerSat
: rtB . b0mlwzzkzn ; rtB . lno5gwkvsmk [ 0 ] = rtB . c4gioprvct ; rtB .
lno5gwkvsmk [ 1 ] = rtB . oaaq01l5fu ; rtB . lno5gwkvsmk [ 2 ] = rtB .
o0acnzn2gn ; rtB . lno5gwkvsmk [ 3 ] = rtB . ne53mrvzfq [ 12 ] ; rtB .
lno5gwkvsmk [ 4 ] = rtB . ne53mrvzfq [ 6 ] ; rtB . lno5gwkvsmk [ 5 ] = rtB .
ne53mrvzfq [ 7 ] ; for ( i = 0 ; i < 2 ; i ++ ) { rtB . kourgu0gp0 [ i ] =
0.0 ; for ( i_p = 0 ; i_p < 6 ; i_p ++ ) { rtB . kourgu0gp0 [ i ] += rtP . K
[ ( i_p << 1 ) + i ] * rtB . lno5gwkvsmk [ i_p ] ; } } YD = 0.15 *
muDoubleScalarSin ( rtB . hkrrrfopbq ) ; YB = 0.15 * muDoubleScalarSin ( rtB
. liiv3zehzs ) ; XD = 0.15 * muDoubleScalarCos ( rtB . hkrrrfopbq ) + 0.108 ;
XB = 0.15 * muDoubleScalarCos ( rtB . liiv3zehzs ) ; lBD = muDoubleScalarSqrt
( ( XD - XB ) * ( XD - XB ) + ( YD - YB ) * ( YD - YB ) ) ; A0 = ( XD - XB )
* 0.5 ; B0 = ( YD - YB ) * 0.5 ; lBD = ( lBD * lBD + 0.0625 ) - 0.0625 ; A0 =
muDoubleScalarAtan2 ( muDoubleScalarSqrt ( ( A0 * A0 + B0 * B0 ) - lBD * lBD
) + B0 , A0 + lBD ) * 2.0 ; YD = muDoubleScalarAtan2 ( ( YB - YD ) + 0.25 *
muDoubleScalarSin ( A0 ) , ( XB - XD ) + 0.25 * muDoubleScalarCos ( A0 ) ) ;
XD = 0.15 * muDoubleScalarCos ( rtB . liiv3zehzs ) + 0.25 * muDoubleScalarCos
( A0 ) ; XB = 0.15 * muDoubleScalarSin ( rtB . liiv3zehzs ) + 0.25 *
muDoubleScalarSin ( A0 ) ; YB = muDoubleScalarSqrt ( ( XD - 0.054 ) * ( XD -
0.054 ) + XB * XB ) ; XD = muDoubleScalarAtan2 ( XB , XD - 0.054 ) ; rtB .
luhg3pfwco = ( muDoubleScalarCos ( XD - A0 ) * 0.15 * muDoubleScalarSin ( YD
- rtB . hkrrrfopbq ) / YB * muDoubleScalarSin ( YD - A0 ) * rtB . kourgu0gp0
[ 1 ] + muDoubleScalarSin ( XD - A0 ) * 0.15 * muDoubleScalarSin ( YD - rtB .
hkrrrfopbq ) / muDoubleScalarSin ( YD - A0 ) * rtB . bpbscqkzum ) + rtB .
hhykaoe4za ; if ( ssIsSampleHit ( rtS , 1 , 0 ) ) { rtDW . g1jau2oi4y = (
ssGetTaskTime ( rtS , 1 ) >= rtP . jump_control_jump_time ) ; rtDW .
phsfelswbk = ( ssGetTaskTime ( rtS , 1 ) >= rtP . jump_control_jump_time +
0.2 ) ; rtDW . emdjtjjfhz = ( ssGetTaskTime ( rtS , 1 ) >= rtP .
jump_control_jump_time + 0.15 ) ; rtDW . i1mxirq5kb = ( ssGetTaskTime ( rtS ,
1 ) >= rtP . jump_control_jump_time + 0.4 ) ; if ( rtDW . g1jau2oi4y == 1 ) {
XB = rtP . Step4_YFinal ; } else { XB = rtP . Step4_Y0 ; } if ( rtDW .
phsfelswbk == 1 ) { B0 = rtP . Step17_YFinal ; } else { B0 = rtP . Step17_Y0
; } if ( rtDW . emdjtjjfhz == 1 ) { lBD = rtP . Step5_YFinal ; } else { lBD =
rtP . Step5_Y0 ; } if ( rtDW . i1mxirq5kb == 1 ) { l_a = rtP . Step10_YFinal
; } else { l_a = rtP . Step10_Y0 ; } rtB . fasi4utync = XB * B0 - lBD * l_a ;
} rtB . mj2xnu20kl = ( muDoubleScalarCos ( XD - YD ) * 0.15 *
muDoubleScalarSin ( rtB . liiv3zehzs - A0 ) / YB * muDoubleScalarSin ( YD -
A0 ) * rtB . kourgu0gp0 [ 1 ] + muDoubleScalarSin ( XD - YD ) * 0.15 *
muDoubleScalarSin ( rtB . liiv3zehzs - A0 ) / muDoubleScalarSin ( YD - A0 ) *
rtB . bpbscqkzum ) + rtB . fasi4utync ; rtB . cp1jjx2a4p = rtP .
Constant1_Value - muDoubleScalarSqrt ( ( kfo1vde04o - 0.054 ) * ( kfo1vde04o
- 0.054 ) + YC * YC ) ; rtB . furrbhkomt = rtP . PIDController2_D * rtB .
cp1jjx2a4p ; rtB . mrjzncw5q4 = rtX . p5p3ogk4mi ; rtB . imk0sd2jrm = rtB .
furrbhkomt - rtB . mrjzncw5q4 ; rtB . jteoenmoec = rtP . PIDController2_I *
rtB . cp1jjx2a4p ; rtB . jw4ohhvqrt = rtX . cwaruwgj0n ; rtB . f1443sccd2 =
rtP . PIDController2_N * rtB . imk0sd2jrm ; rtB . jfiugehczg = rtP .
PIDController3_I * rtB . emby3igyk1 ; if ( ssIsSampleHit ( rtS , 1 , 0 ) ) {
if ( rtB . ct5ptiiqfm > rtP . Saturation1_UpperSat ) { rtB . hdeqxnl10m = rtP
. Saturation1_UpperSat ; } else if ( rtB . ct5ptiiqfm < rtP .
Saturation1_LowerSat ) { rtB . hdeqxnl10m = rtP . Saturation1_LowerSat ; }
else { rtB . hdeqxnl10m = rtB . ct5ptiiqfm ; } if ( rtB . egyva5trzl > rtP .
Saturation2_UpperSat ) { rtB . djd0cbchne = rtP . Saturation2_UpperSat ; }
else if ( rtB . egyva5trzl < rtP . Saturation2_LowerSat ) { rtB . djd0cbchne
= rtP . Saturation2_LowerSat ; } else { rtB . djd0cbchne = rtB . egyva5trzl ;
} } if ( ssIsMajorTimeStep ( rtS ) ) { rtDW . dhbkidd3zo = rtB . luhg3pfwco
>= rtP . Saturation3_UpperSat ? 1 : rtB . luhg3pfwco > rtP .
Saturation3_LowerSat ? 0 : - 1 ; rtDW . iy1pcv3xo4 = rtB . mj2xnu20kl >= rtP
. Saturation4_UpperSat ? 1 : rtB . mj2xnu20kl > rtP . Saturation4_LowerSat ?
0 : - 1 ; } rtB . hl2vsnjvah = rtDW . dhbkidd3zo == 1 ? rtP .
Saturation3_UpperSat : rtDW . dhbkidd3zo == - 1 ? rtP . Saturation3_LowerSat
: rtB . luhg3pfwco ; rtB . hfyfla4srx = rtDW . iy1pcv3xo4 == 1 ? rtP .
Saturation4_UpperSat : rtDW . iy1pcv3xo4 == - 1 ? rtP . Saturation4_LowerSat
: rtB . mj2xnu20kl ; rtB . nlnzoie4ft = ( ( rtP . PIDController2_P * rtB .
cp1jjx2a4p + rtB . jw4ohhvqrt ) + rtB . f1443sccd2 ) + rtP . _Value ; if (
ssIsMajorTimeStep ( rtS ) ) { rtDW . eqwdrnbx1x = rtB . nlnzoie4ft >= rtP .
Saturation6_UpperSat ? 1 : rtB . nlnzoie4ft > rtP . Saturation6_LowerSat ? 0
: - 1 ; rtDW . pny00ejsqz = rtB . m0bxxy51bt [ 0 ] >= rtP .
Saturation7_UpperSat ? 1 : rtB . m0bxxy51bt [ 0 ] > rtP .
Saturation7_LowerSat ? 0 : - 1 ; rtDW . aqiduhop2a = rtB . kourgu0gp0 [ 0 ]
>= rtP . Saturation8_UpperSat ? 1 : rtB . kourgu0gp0 [ 0 ] > rtP .
Saturation8_LowerSat ? 0 : - 1 ; } rtB . affemz5v0v = rtDW . pny00ejsqz == 1
? rtP . Saturation7_UpperSat : rtDW . pny00ejsqz == - 1 ? rtP .
Saturation7_LowerSat : rtB . m0bxxy51bt [ 0 ] ; rtB . amehlztnnv = rtDW .
aqiduhop2a == 1 ? rtP . Saturation8_UpperSat : rtDW . aqiduhop2a == - 1 ? rtP
. Saturation8_LowerSat : rtB . kourgu0gp0 [ 0 ] ; if ( ssIsSampleHit ( rtS ,
1 , 0 ) ) { rtDW . fpt2stpuqn = ( ssGetTaskTime ( rtS , 1 ) >= rtP .
Step2_Time ) ; rtDW . ctsfqdbiky = ( ssGetTaskTime ( rtS , 1 ) >= rtP .
Step3_Time ) ; if ( rtDW . fpt2stpuqn == 1 ) { XB = rtP . Step2_YFinal ; }
else { XB = rtP . Step2_Y0 ; } if ( rtDW . ctsfqdbiky == 1 ) { B0 = rtP .
Step3_YFinal ; } else { B0 = rtP . Step3_Y0 ; } rtB . fviua014cu = XB - B0 ;
} rtB . kaed4psp05 = rtB . ne53mrvzfq [ 12 ] - rtB . fviua014cu ; rtB .
chcwz4ujvj = rtX . ecnkbzok1f ; rtB . nxxivmzdo0 = rtP . Gain29_Gain * rtB .
ne53mrvzfq [ 1 ] ; rtB . k3exkay1ay = rtX . iko4pycqyt ; rtB . kma1dpwhhf =
rtP . Gain41_Gain * rtB . ne53mrvzfq [ 0 ] ; rtB . nik4ggyist [ 0 ] = rtB .
affemz5v0v ; rtB . nik4ggyist [ 1 ] = 0.0 ; rtB . nik4ggyist [ 2 ] = 0.0 ;
rtB . nik4ggyist [ 3 ] = 0.0 ; rtB . ludhxwcsap [ 0 ] = rtB . amehlztnnv ;
rtB . ludhxwcsap [ 1 ] = 0.0 ; rtB . ludhxwcsap [ 2 ] = 0.0 ; rtB .
ludhxwcsap [ 3 ] = 0.0 ; rtB . jl2dsktpay [ 0 ] = rtB . hdeqxnl10m ; rtB .
jl2dsktpay [ 1 ] = 0.0 ; rtB . jl2dsktpay [ 2 ] = 0.0 ; if (
ssIsMajorTimeStep ( rtS ) ) { rtDW . f1r4g4whvp [ 0 ] = ! ( rtB . jl2dsktpay
[ 0 ] == rtDW . f1r4g4whvp [ 1 ] ) ; rtDW . f1r4g4whvp [ 1 ] = rtB .
jl2dsktpay [ 0 ] ; } rtB . jl2dsktpay [ 0 ] = rtDW . f1r4g4whvp [ 1 ] ; rtB .
jl2dsktpay [ 3 ] = rtDW . f1r4g4whvp [ 0 ] ; rtB . clrzjaih1v [ 0 ] = rtB .
djd0cbchne ; rtB . clrzjaih1v [ 1 ] = 0.0 ; rtB . clrzjaih1v [ 2 ] = 0.0 ; if
( ssIsMajorTimeStep ( rtS ) ) { rtDW . aokoeghayj [ 0 ] = ! ( rtB .
clrzjaih1v [ 0 ] == rtDW . aokoeghayj [ 1 ] ) ; rtDW . aokoeghayj [ 1 ] = rtB
. clrzjaih1v [ 0 ] ; } rtB . clrzjaih1v [ 0 ] = rtDW . aokoeghayj [ 1 ] ; rtB
. clrzjaih1v [ 3 ] = rtDW . aokoeghayj [ 0 ] ; rtB . dmfybrfnod [ 0 ] = rtB .
hfyfla4srx ; rtB . dmfybrfnod [ 1 ] = 0.0 ; rtB . dmfybrfnod [ 2 ] = 0.0 ;
rtB . dmfybrfnod [ 3 ] = 0.0 ; rtB . l1345vcjcj [ 0 ] = rtB . hl2vsnjvah ;
rtB . l1345vcjcj [ 1 ] = 0.0 ; rtB . l1345vcjcj [ 2 ] = 0.0 ; rtB .
l1345vcjcj [ 3 ] = 0.0 ; simulationData = ( NeslSimulationData * ) rtDW .
hpnunkuqpe ; time_e = ssGetT ( rtS ) ; simulationData -> mData -> mTime . mN
= 1 ; simulationData -> mData -> mTime . mX = & time_e ; simulationData ->
mData -> mContStates . mN = 0 ; simulationData -> mData -> mContStates . mX =
NULL ; simulationData -> mData -> mDiscStates . mN = 0 ; simulationData ->
mData -> mDiscStates . mX = & rtDW . h5eppyllsj ; simulationData -> mData ->
mModeVector . mN = 0 ; simulationData -> mData -> mModeVector . mX = & rtDW .
g24kz15cah ; tmp = ( ssIsMajorTimeStep ( rtS ) && ssGetRTWSolverInfo ( rtS )
-> foundContZcEvents ) ; simulationData -> mData -> mFoundZcEvents = tmp ;
simulationData -> mData -> mIsMajorTimeStep = ssIsMajorTimeStep ( rtS ) ; tmp
= ( ssGetMdlInfoPtr ( rtS ) -> mdlFlags . solverAssertCheck == 1U ) ;
simulationData -> mData -> mIsSolverAssertCheck = tmp ; tmp =
ssIsSolverCheckingCIC ( rtS ) ; simulationData -> mData ->
mIsSolverCheckingCIC = tmp ; simulationData -> mData -> mIsComputingJacobian
= false ; simulationData -> mData -> mIsEvaluatingF0 = false ; tmp =
ssIsSolverRequestingReset ( rtS ) ; simulationData -> mData ->
mIsSolverRequestingReset = tmp ; tmp_j [ 0 ] = 0 ; tmp_g [ 0 ] = rtB .
nik4ggyist [ 0 ] ; tmp_g [ 1 ] = rtB . nik4ggyist [ 1 ] ; tmp_g [ 2 ] = rtB .
nik4ggyist [ 2 ] ; tmp_g [ 3 ] = rtB . nik4ggyist [ 3 ] ; tmp_j [ 1 ] = 4 ;
tmp_g [ 4 ] = rtB . ludhxwcsap [ 0 ] ; tmp_g [ 5 ] = rtB . ludhxwcsap [ 1 ] ;
tmp_g [ 6 ] = rtB . ludhxwcsap [ 2 ] ; tmp_g [ 7 ] = rtB . ludhxwcsap [ 3 ] ;
tmp_j [ 2 ] = 8 ; tmp_g [ 8 ] = rtB . jl2dsktpay [ 0 ] ; tmp_g [ 9 ] = rtB .
jl2dsktpay [ 1 ] ; tmp_g [ 10 ] = rtB . jl2dsktpay [ 2 ] ; tmp_g [ 11 ] = rtB
. jl2dsktpay [ 3 ] ; tmp_j [ 3 ] = 12 ; tmp_g [ 12 ] = rtB . clrzjaih1v [ 0 ]
; tmp_g [ 13 ] = rtB . clrzjaih1v [ 1 ] ; tmp_g [ 14 ] = rtB . clrzjaih1v [ 2
] ; tmp_g [ 15 ] = rtB . clrzjaih1v [ 3 ] ; tmp_j [ 4 ] = 16 ; tmp_g [ 16 ] =
rtB . dmfybrfnod [ 0 ] ; tmp_g [ 17 ] = rtB . dmfybrfnod [ 1 ] ; tmp_g [ 18 ]
= rtB . dmfybrfnod [ 2 ] ; tmp_g [ 19 ] = rtB . dmfybrfnod [ 3 ] ; tmp_j [ 5
] = 20 ; tmp_g [ 20 ] = rtB . l1345vcjcj [ 0 ] ; tmp_g [ 21 ] = rtB .
l1345vcjcj [ 1 ] ; tmp_g [ 22 ] = rtB . l1345vcjcj [ 2 ] ; tmp_g [ 23 ] = rtB
. l1345vcjcj [ 3 ] ; tmp_j [ 6 ] = 24 ; memcpy ( & tmp_g [ 24 ] , & rtB .
eh2cmnfpcd [ 0 ] , sizeof ( real_T ) << 5U ) ; tmp_j [ 7 ] = 56 ;
simulationData -> mData -> mInputValues . mN = 56 ; simulationData -> mData
-> mInputValues . mX = & tmp_g [ 0 ] ; simulationData -> mData ->
mInputOffsets . mN = 8 ; simulationData -> mData -> mInputOffsets . mX = &
tmp_j [ 0 ] ; simulationData -> mData -> mOutputs . mN = 13 ; simulationData
-> mData -> mOutputs . mX = & ogp5aqtkms [ 0 ] ; simulationData -> mData ->
mSampleHits . mN = 0 ; simulationData -> mData -> mSampleHits . mX = NULL ;
simulationData -> mData -> mIsFundamentalSampleHit = false ; simulationData
-> mData -> mTolerances . mN = 0 ; simulationData -> mData -> mTolerances .
mX = NULL ; simulationData -> mData -> mCstateHasChanged = false ;
diagnosticManager = ( NeuDiagnosticManager * ) rtDW . ojbc53o3xt ;
diagnosticTree = neu_diagnostic_manager_get_initial_tree ( diagnosticManager
) ; i = ne_simulator_method ( ( NeslSimulator * ) rtDW . loahdpiobx ,
NESL_SIM_OUTPUTS , simulationData , diagnosticManager ) ; if ( i != 0 ) { tmp
= error_buffer_is_empty ( ssGetErrorStatus ( rtS ) ) ; if ( tmp ) { msg =
rtw_diagnostics_msg ( diagnosticTree ) ; ssSetErrorStatus ( rtS , msg ) ; } }
if ( ssIsMajorTimeStep ( rtS ) && simulationData -> mData ->
mCstateHasChanged ) { ssSetBlockStateForSolverChangedAtMajorStep ( rtS ) ; }
UNUSED_PARAMETER ( tid ) ; } void MdlOutputsTID2 ( int_T tid ) {
UNUSED_PARAMETER ( tid ) ; } void MdlUpdate ( int_T tid ) {
NeslSimulationData * simulationData ; NeuDiagnosticManager *
diagnosticManager ; NeuDiagnosticTree * diagnosticTree ; char * msg ; real_T
tmp_p [ 24 ] ; real_T time ; int32_T tmp_i ; int_T tmp_e [ 7 ] ; boolean_T
tmp ; simulationData = ( NeslSimulationData * ) rtDW . gahukgbj3h ; time =
ssGetT ( rtS ) ; simulationData -> mData -> mTime . mN = 1 ; simulationData
-> mData -> mTime . mX = & time ; simulationData -> mData -> mContStates . mN
= 32 ; simulationData -> mData -> mContStates . mX = & rtX . mefcq5joqv [ 0 ]
; simulationData -> mData -> mDiscStates . mN = 0 ; simulationData -> mData
-> mDiscStates . mX = & rtDW . ooldbvhge3 ; simulationData -> mData ->
mModeVector . mN = 0 ; simulationData -> mData -> mModeVector . mX = & rtDW .
nzztmagq15 ; tmp = ( ssIsMajorTimeStep ( rtS ) && ssGetRTWSolverInfo ( rtS )
-> foundContZcEvents ) ; simulationData -> mData -> mFoundZcEvents = tmp ;
simulationData -> mData -> mIsMajorTimeStep = ssIsMajorTimeStep ( rtS ) ; tmp
= ( ssGetMdlInfoPtr ( rtS ) -> mdlFlags . solverAssertCheck == 1U ) ;
simulationData -> mData -> mIsSolverAssertCheck = tmp ; tmp =
ssIsSolverCheckingCIC ( rtS ) ; simulationData -> mData ->
mIsSolverCheckingCIC = tmp ; tmp = ssIsSolverComputingJacobian ( rtS ) ;
simulationData -> mData -> mIsComputingJacobian = tmp ; simulationData ->
mData -> mIsEvaluatingF0 = ( ssGetEvaluatingF0ForJacobian ( rtS ) != 0 ) ;
tmp = ssIsSolverRequestingReset ( rtS ) ; simulationData -> mData ->
mIsSolverRequestingReset = tmp ; tmp_e [ 0 ] = 0 ; tmp_p [ 0 ] = rtB .
nik4ggyist [ 0 ] ; tmp_p [ 1 ] = rtB . nik4ggyist [ 1 ] ; tmp_p [ 2 ] = rtB .
nik4ggyist [ 2 ] ; tmp_p [ 3 ] = rtB . nik4ggyist [ 3 ] ; tmp_e [ 1 ] = 4 ;
tmp_p [ 4 ] = rtB . ludhxwcsap [ 0 ] ; tmp_p [ 5 ] = rtB . ludhxwcsap [ 1 ] ;
tmp_p [ 6 ] = rtB . ludhxwcsap [ 2 ] ; tmp_p [ 7 ] = rtB . ludhxwcsap [ 3 ] ;
tmp_e [ 2 ] = 8 ; tmp_p [ 8 ] = rtB . jl2dsktpay [ 0 ] ; tmp_p [ 9 ] = rtB .
jl2dsktpay [ 1 ] ; tmp_p [ 10 ] = rtB . jl2dsktpay [ 2 ] ; tmp_p [ 11 ] = rtB
. jl2dsktpay [ 3 ] ; tmp_e [ 3 ] = 12 ; tmp_p [ 12 ] = rtB . clrzjaih1v [ 0 ]
; tmp_p [ 13 ] = rtB . clrzjaih1v [ 1 ] ; tmp_p [ 14 ] = rtB . clrzjaih1v [ 2
] ; tmp_p [ 15 ] = rtB . clrzjaih1v [ 3 ] ; tmp_e [ 4 ] = 16 ; tmp_p [ 16 ] =
rtB . dmfybrfnod [ 0 ] ; tmp_p [ 17 ] = rtB . dmfybrfnod [ 1 ] ; tmp_p [ 18 ]
= rtB . dmfybrfnod [ 2 ] ; tmp_p [ 19 ] = rtB . dmfybrfnod [ 3 ] ; tmp_e [ 5
] = 20 ; tmp_p [ 20 ] = rtB . l1345vcjcj [ 0 ] ; tmp_p [ 21 ] = rtB .
l1345vcjcj [ 1 ] ; tmp_p [ 22 ] = rtB . l1345vcjcj [ 2 ] ; tmp_p [ 23 ] = rtB
. l1345vcjcj [ 3 ] ; tmp_e [ 6 ] = 24 ; simulationData -> mData ->
mInputValues . mN = 24 ; simulationData -> mData -> mInputValues . mX = &
tmp_p [ 0 ] ; simulationData -> mData -> mInputOffsets . mN = 7 ;
simulationData -> mData -> mInputOffsets . mX = & tmp_e [ 0 ] ;
diagnosticManager = ( NeuDiagnosticManager * ) rtDW . bq244s1auv ;
diagnosticTree = neu_diagnostic_manager_get_initial_tree ( diagnosticManager
) ; tmp_i = ne_simulator_method ( ( NeslSimulator * ) rtDW . eukrvu0tgn ,
NESL_SIM_UPDATE , simulationData , diagnosticManager ) ; if ( tmp_i != 0 ) {
tmp = error_buffer_is_empty ( ssGetErrorStatus ( rtS ) ) ; if ( tmp ) { msg =
rtw_diagnostics_msg ( diagnosticTree ) ; ssSetErrorStatus ( rtS , msg ) ; } }
UNUSED_PARAMETER ( tid ) ; } void MdlUpdateTID2 ( int_T tid ) {
UNUSED_PARAMETER ( tid ) ; } void MdlDerivatives ( void ) {
NeslSimulationData * simulationData ; NeuDiagnosticManager *
diagnosticManager ; NeuDiagnosticTree * diagnosticTree ; XDot * _rtXdot ;
char * msg ; real_T tmp_p [ 24 ] ; real_T time ; int32_T tmp_i ; int_T tmp_e
[ 7 ] ; boolean_T tmp ; _rtXdot = ( ( XDot * ) ssGetdX ( rtS ) ) ;
simulationData = ( NeslSimulationData * ) rtDW . gahukgbj3h ; time = ssGetT (
rtS ) ; simulationData -> mData -> mTime . mN = 1 ; simulationData -> mData
-> mTime . mX = & time ; simulationData -> mData -> mContStates . mN = 32 ;
simulationData -> mData -> mContStates . mX = & rtX . mefcq5joqv [ 0 ] ;
simulationData -> mData -> mDiscStates . mN = 0 ; simulationData -> mData ->
mDiscStates . mX = & rtDW . ooldbvhge3 ; simulationData -> mData ->
mModeVector . mN = 0 ; simulationData -> mData -> mModeVector . mX = & rtDW .
nzztmagq15 ; tmp = ( ssIsMajorTimeStep ( rtS ) && ssGetRTWSolverInfo ( rtS )
-> foundContZcEvents ) ; simulationData -> mData -> mFoundZcEvents = tmp ;
simulationData -> mData -> mIsMajorTimeStep = ssIsMajorTimeStep ( rtS ) ; tmp
= ( ssGetMdlInfoPtr ( rtS ) -> mdlFlags . solverAssertCheck == 1U ) ;
simulationData -> mData -> mIsSolverAssertCheck = tmp ; tmp =
ssIsSolverCheckingCIC ( rtS ) ; simulationData -> mData ->
mIsSolverCheckingCIC = tmp ; tmp = ssIsSolverComputingJacobian ( rtS ) ;
simulationData -> mData -> mIsComputingJacobian = tmp ; simulationData ->
mData -> mIsEvaluatingF0 = ( ssGetEvaluatingF0ForJacobian ( rtS ) != 0 ) ;
tmp = ssIsSolverRequestingReset ( rtS ) ; simulationData -> mData ->
mIsSolverRequestingReset = tmp ; tmp_e [ 0 ] = 0 ; tmp_p [ 0 ] = rtB .
nik4ggyist [ 0 ] ; tmp_p [ 1 ] = rtB . nik4ggyist [ 1 ] ; tmp_p [ 2 ] = rtB .
nik4ggyist [ 2 ] ; tmp_p [ 3 ] = rtB . nik4ggyist [ 3 ] ; tmp_e [ 1 ] = 4 ;
tmp_p [ 4 ] = rtB . ludhxwcsap [ 0 ] ; tmp_p [ 5 ] = rtB . ludhxwcsap [ 1 ] ;
tmp_p [ 6 ] = rtB . ludhxwcsap [ 2 ] ; tmp_p [ 7 ] = rtB . ludhxwcsap [ 3 ] ;
tmp_e [ 2 ] = 8 ; tmp_p [ 8 ] = rtB . jl2dsktpay [ 0 ] ; tmp_p [ 9 ] = rtB .
jl2dsktpay [ 1 ] ; tmp_p [ 10 ] = rtB . jl2dsktpay [ 2 ] ; tmp_p [ 11 ] = rtB
. jl2dsktpay [ 3 ] ; tmp_e [ 3 ] = 12 ; tmp_p [ 12 ] = rtB . clrzjaih1v [ 0 ]
; tmp_p [ 13 ] = rtB . clrzjaih1v [ 1 ] ; tmp_p [ 14 ] = rtB . clrzjaih1v [ 2
] ; tmp_p [ 15 ] = rtB . clrzjaih1v [ 3 ] ; tmp_e [ 4 ] = 16 ; tmp_p [ 16 ] =
rtB . dmfybrfnod [ 0 ] ; tmp_p [ 17 ] = rtB . dmfybrfnod [ 1 ] ; tmp_p [ 18 ]
= rtB . dmfybrfnod [ 2 ] ; tmp_p [ 19 ] = rtB . dmfybrfnod [ 3 ] ; tmp_e [ 5
] = 20 ; tmp_p [ 20 ] = rtB . l1345vcjcj [ 0 ] ; tmp_p [ 21 ] = rtB .
l1345vcjcj [ 1 ] ; tmp_p [ 22 ] = rtB . l1345vcjcj [ 2 ] ; tmp_p [ 23 ] = rtB
. l1345vcjcj [ 3 ] ; tmp_e [ 6 ] = 24 ; simulationData -> mData ->
mInputValues . mN = 24 ; simulationData -> mData -> mInputValues . mX = &
tmp_p [ 0 ] ; simulationData -> mData -> mInputOffsets . mN = 7 ;
simulationData -> mData -> mInputOffsets . mX = & tmp_e [ 0 ] ;
simulationData -> mData -> mDx . mN = 32 ; simulationData -> mData -> mDx .
mX = & _rtXdot -> mefcq5joqv [ 0 ] ; diagnosticManager = (
NeuDiagnosticManager * ) rtDW . bq244s1auv ; diagnosticTree =
neu_diagnostic_manager_get_initial_tree ( diagnosticManager ) ; tmp_i =
ne_simulator_method ( ( NeslSimulator * ) rtDW . eukrvu0tgn ,
NESL_SIM_DERIVATIVES , simulationData , diagnosticManager ) ; if ( tmp_i != 0
) { tmp = error_buffer_is_empty ( ssGetErrorStatus ( rtS ) ) ; if ( tmp ) {
msg = rtw_diagnostics_msg ( diagnosticTree ) ; ssSetErrorStatus ( rtS , msg )
; } } _rtXdot -> cbywjryyoi = rtB . kaed4psp05 ; _rtXdot -> b2hgpxvjzc = rtB
. jfiugehczg ; _rtXdot -> abxab3djbi = rtB . kxvii2moqu ; _rtXdot ->
p5p3ogk4mi = rtB . f1443sccd2 ; _rtXdot -> cwaruwgj0n = rtB . jteoenmoec ;
_rtXdot -> ecnkbzok1f = rtB . nxxivmzdo0 ; _rtXdot -> iko4pycqyt = rtB .
kma1dpwhhf ; } void MdlProjection ( void ) { NeslSimulationData *
simulationData ; NeuDiagnosticManager * diagnosticManager ; NeuDiagnosticTree
* diagnosticTree ; char * msg ; real_T tmp_p [ 24 ] ; real_T time ; int32_T
tmp_i ; int_T tmp_e [ 7 ] ; boolean_T tmp ; simulationData = (
NeslSimulationData * ) rtDW . gahukgbj3h ; time = ssGetT ( rtS ) ;
simulationData -> mData -> mTime . mN = 1 ; simulationData -> mData -> mTime
. mX = & time ; simulationData -> mData -> mContStates . mN = 32 ;
simulationData -> mData -> mContStates . mX = & rtX . mefcq5joqv [ 0 ] ;
simulationData -> mData -> mDiscStates . mN = 0 ; simulationData -> mData ->
mDiscStates . mX = & rtDW . ooldbvhge3 ; simulationData -> mData ->
mModeVector . mN = 0 ; simulationData -> mData -> mModeVector . mX = & rtDW .
nzztmagq15 ; tmp = ( ssIsMajorTimeStep ( rtS ) && ssGetRTWSolverInfo ( rtS )
-> foundContZcEvents ) ; simulationData -> mData -> mFoundZcEvents = tmp ;
simulationData -> mData -> mIsMajorTimeStep = ssIsMajorTimeStep ( rtS ) ; tmp
= ( ssGetMdlInfoPtr ( rtS ) -> mdlFlags . solverAssertCheck == 1U ) ;
simulationData -> mData -> mIsSolverAssertCheck = tmp ; tmp =
ssIsSolverCheckingCIC ( rtS ) ; simulationData -> mData ->
mIsSolverCheckingCIC = tmp ; tmp = ssIsSolverComputingJacobian ( rtS ) ;
simulationData -> mData -> mIsComputingJacobian = tmp ; simulationData ->
mData -> mIsEvaluatingF0 = ( ssGetEvaluatingF0ForJacobian ( rtS ) != 0 ) ;
tmp = ssIsSolverRequestingReset ( rtS ) ; simulationData -> mData ->
mIsSolverRequestingReset = tmp ; tmp_e [ 0 ] = 0 ; tmp_p [ 0 ] = rtB .
nik4ggyist [ 0 ] ; tmp_p [ 1 ] = rtB . nik4ggyist [ 1 ] ; tmp_p [ 2 ] = rtB .
nik4ggyist [ 2 ] ; tmp_p [ 3 ] = rtB . nik4ggyist [ 3 ] ; tmp_e [ 1 ] = 4 ;
tmp_p [ 4 ] = rtB . ludhxwcsap [ 0 ] ; tmp_p [ 5 ] = rtB . ludhxwcsap [ 1 ] ;
tmp_p [ 6 ] = rtB . ludhxwcsap [ 2 ] ; tmp_p [ 7 ] = rtB . ludhxwcsap [ 3 ] ;
tmp_e [ 2 ] = 8 ; tmp_p [ 8 ] = rtB . jl2dsktpay [ 0 ] ; tmp_p [ 9 ] = rtB .
jl2dsktpay [ 1 ] ; tmp_p [ 10 ] = rtB . jl2dsktpay [ 2 ] ; tmp_p [ 11 ] = rtB
. jl2dsktpay [ 3 ] ; tmp_e [ 3 ] = 12 ; tmp_p [ 12 ] = rtB . clrzjaih1v [ 0 ]
; tmp_p [ 13 ] = rtB . clrzjaih1v [ 1 ] ; tmp_p [ 14 ] = rtB . clrzjaih1v [ 2
] ; tmp_p [ 15 ] = rtB . clrzjaih1v [ 3 ] ; tmp_e [ 4 ] = 16 ; tmp_p [ 16 ] =
rtB . dmfybrfnod [ 0 ] ; tmp_p [ 17 ] = rtB . dmfybrfnod [ 1 ] ; tmp_p [ 18 ]
= rtB . dmfybrfnod [ 2 ] ; tmp_p [ 19 ] = rtB . dmfybrfnod [ 3 ] ; tmp_e [ 5
] = 20 ; tmp_p [ 20 ] = rtB . l1345vcjcj [ 0 ] ; tmp_p [ 21 ] = rtB .
l1345vcjcj [ 1 ] ; tmp_p [ 22 ] = rtB . l1345vcjcj [ 2 ] ; tmp_p [ 23 ] = rtB
. l1345vcjcj [ 3 ] ; tmp_e [ 6 ] = 24 ; simulationData -> mData ->
mInputValues . mN = 24 ; simulationData -> mData -> mInputValues . mX = &
tmp_p [ 0 ] ; simulationData -> mData -> mInputOffsets . mN = 7 ;
simulationData -> mData -> mInputOffsets . mX = & tmp_e [ 0 ] ;
diagnosticManager = ( NeuDiagnosticManager * ) rtDW . bq244s1auv ;
diagnosticTree = neu_diagnostic_manager_get_initial_tree ( diagnosticManager
) ; tmp_i = ne_simulator_method ( ( NeslSimulator * ) rtDW . eukrvu0tgn ,
NESL_SIM_PROJECTION , simulationData , diagnosticManager ) ; if ( tmp_i != 0
) { tmp = error_buffer_is_empty ( ssGetErrorStatus ( rtS ) ) ; if ( tmp ) {
msg = rtw_diagnostics_msg ( diagnosticTree ) ; ssSetErrorStatus ( rtS , msg )
; } } } void MdlZeroCrossings ( void ) { ZCV * _rtZCSV ; _rtZCSV = ( ( ZCV *
) ssGetSolverZcSignalVector ( rtS ) ) ; _rtZCSV -> gpfksi43yf = ssGetT ( rtS
) - rtP . jump_control_jump_time ; _rtZCSV -> hrb1f4zrkp = ssGetT ( rtS ) - (
rtP . jump_control_jump_time + 0.2 ) ; _rtZCSV -> mx3ej2d0lz = ssGetT ( rtS )
- ( rtP . jump_control_jump_time + 0.15 ) ; _rtZCSV -> j2w11ulpbr = ssGetT (
rtS ) - ( rtP . jump_control_jump_time + 0.4 ) ; _rtZCSV -> h0ndtenaza =
ssGetT ( rtS ) - rtP . jump_control_jump_time ; _rtZCSV -> pqo3v35tj2 =
ssGetT ( rtS ) - ( rtP . jump_control_jump_time + 0.2 ) ; _rtZCSV ->
p2lke5fdjt = ssGetT ( rtS ) - ( rtP . jump_control_jump_time + 0.15 ) ;
_rtZCSV -> kwayygqstb = ssGetT ( rtS ) - ( rtP . jump_control_jump_time + 0.4
) ; _rtZCSV -> npan3fd15s = ssGetT ( rtS ) - rtP . jump_control_jump_time ;
_rtZCSV -> gwz0n5o4x3 = ssGetT ( rtS ) - ( rtP . jump_control_jump_time + 0.2
) ; _rtZCSV -> gqi0j3vxe4 = ssGetT ( rtS ) - ( rtP . jump_control_jump_time +
0.15 ) ; _rtZCSV -> c0javv13ug = ssGetT ( rtS ) - ( rtP .
jump_control_jump_time + 0.4 ) ; _rtZCSV -> jfgmng40eu = rtB . b0mlwzzkzn -
rtP . Saturation5_UpperSat ; _rtZCSV -> obibk1x1hr = rtB . b0mlwzzkzn - rtP .
Saturation5_LowerSat ; _rtZCSV -> pvexx3vpoz = ssGetT ( rtS ) - rtP .
jump_control_jump_time ; _rtZCSV -> cii3tbqbnf = ssGetT ( rtS ) - ( rtP .
jump_control_jump_time + 0.2 ) ; _rtZCSV -> l3zjpcmhxv = ssGetT ( rtS ) - (
rtP . jump_control_jump_time + 0.15 ) ; _rtZCSV -> kkb3dwir2d = ssGetT ( rtS
) - ( rtP . jump_control_jump_time + 0.4 ) ; _rtZCSV -> f32leexdmb = rtB .
luhg3pfwco - rtP . Saturation3_UpperSat ; _rtZCSV -> ladlc0iipq = rtB .
luhg3pfwco - rtP . Saturation3_LowerSat ; _rtZCSV -> m4wqxj1mr5 = rtB .
mj2xnu20kl - rtP . Saturation4_UpperSat ; _rtZCSV -> e4xrxgq33o = rtB .
mj2xnu20kl - rtP . Saturation4_LowerSat ; _rtZCSV -> pbjb22nahj = rtB .
nlnzoie4ft - rtP . Saturation6_UpperSat ; _rtZCSV -> hr4f4iah5g = rtB .
nlnzoie4ft - rtP . Saturation6_LowerSat ; _rtZCSV -> ag3ne5iazt = rtB .
m0bxxy51bt [ 0 ] - rtP . Saturation7_UpperSat ; _rtZCSV -> mf3q5omeon = rtB .
m0bxxy51bt [ 0 ] - rtP . Saturation7_LowerSat ; _rtZCSV -> mcy5qbv5vq = rtB .
kourgu0gp0 [ 0 ] - rtP . Saturation8_UpperSat ; _rtZCSV -> jbunrt0jv0 = rtB .
kourgu0gp0 [ 0 ] - rtP . Saturation8_LowerSat ; _rtZCSV -> ouocp21etn =
ssGetT ( rtS ) - rtP . Step2_Time ; _rtZCSV -> d3fdpqxbhg = ssGetT ( rtS ) -
rtP . Step3_Time ; } void MdlTerminate ( void ) {
neu_destroy_diagnostic_manager ( ( NeuDiagnosticManager * ) rtDW . bq244s1auv
) ; nesl_destroy_simulation_data ( ( NeslSimulationData * ) rtDW . gahukgbj3h
) ; nesl_erase_simulator (
"wheel_leg/blance_leg_model/Solver Configuration1_1" ) ;
nesl_destroy_registry ( ) ; neu_destroy_diagnostic_manager ( (
NeuDiagnosticManager * ) rtDW . bmoh4rr253 ) ; nesl_destroy_simulation_data (
( NeslSimulationData * ) rtDW . jg2ci40rqj ) ; nesl_erase_simulator (
"wheel_leg/blance_leg_model/Solver Configuration1_1" ) ;
nesl_destroy_registry ( ) ; neu_destroy_diagnostic_manager ( (
NeuDiagnosticManager * ) rtDW . ojbc53o3xt ) ; nesl_destroy_simulation_data (
( NeslSimulationData * ) rtDW . hpnunkuqpe ) ; nesl_erase_simulator (
"wheel_leg/blance_leg_model/Solver Configuration1_1" ) ;
nesl_destroy_registry ( ) ; } void MdlInitializeSizes ( void ) {
ssSetNumContStates ( rtS , 39 ) ; ssSetNumPeriodicContStates ( rtS , 0 ) ;
ssSetNumY ( rtS , 0 ) ; ssSetNumU ( rtS , 0 ) ; ssSetDirectFeedThrough ( rtS
, 0 ) ; ssSetNumSampleTimes ( rtS , 2 ) ; ssSetNumBlocks ( rtS , 267 ) ;
ssSetNumBlockIO ( rtS , 69 ) ; ssSetNumBlockParams ( rtS , 100 ) ; } void
MdlInitializeSampleTimes ( void ) { ssSetSampleTime ( rtS , 0 , 0.0 ) ;
ssSetSampleTime ( rtS , 1 , 0.0 ) ; ssSetOffsetTime ( rtS , 0 , 0.0 ) ;
ssSetOffsetTime ( rtS , 1 , 1.0 ) ; } void raccel_set_checksum ( ) {
ssSetChecksumVal ( rtS , 0 , 3625382941U ) ; ssSetChecksumVal ( rtS , 1 ,
3387521915U ) ; ssSetChecksumVal ( rtS , 2 , 3494444731U ) ; ssSetChecksumVal
( rtS , 3 , 3437323851U ) ; }
#if defined(_MSC_VER)
#pragma optimize( "", off )
#endif
SimStruct * raccel_register_model ( ssExecutionInfo * executionInfo ) {
static struct _ssMdlInfo mdlInfo ; ( void ) memset ( ( char * ) rtS , 0 ,
sizeof ( SimStruct ) ) ; ( void ) memset ( ( char * ) & mdlInfo , 0 , sizeof
( struct _ssMdlInfo ) ) ; ssSetMdlInfoPtr ( rtS , & mdlInfo ) ;
ssSetExecutionInfo ( rtS , executionInfo ) ; { static time_T mdlPeriod [
NSAMPLE_TIMES ] ; static time_T mdlOffset [ NSAMPLE_TIMES ] ; static time_T
mdlTaskTimes [ NSAMPLE_TIMES ] ; static int_T mdlTsMap [ NSAMPLE_TIMES ] ;
static int_T mdlSampleHits [ NSAMPLE_TIMES ] ; static boolean_T
mdlTNextWasAdjustedPtr [ NSAMPLE_TIMES ] ; static int_T mdlPerTaskSampleHits
[ NSAMPLE_TIMES * NSAMPLE_TIMES ] ; static time_T mdlTimeOfNextSampleHit [
NSAMPLE_TIMES ] ; { int_T i ; for ( i = 0 ; i < NSAMPLE_TIMES ; i ++ ) {
mdlPeriod [ i ] = 0.0 ; mdlOffset [ i ] = 0.0 ; mdlTaskTimes [ i ] = 0.0 ;
mdlTsMap [ i ] = i ; mdlSampleHits [ i ] = 1 ; } } ssSetSampleTimePtr ( rtS ,
& mdlPeriod [ 0 ] ) ; ssSetOffsetTimePtr ( rtS , & mdlOffset [ 0 ] ) ;
ssSetSampleTimeTaskIDPtr ( rtS , & mdlTsMap [ 0 ] ) ; ssSetTPtr ( rtS , &
mdlTaskTimes [ 0 ] ) ; ssSetSampleHitPtr ( rtS , & mdlSampleHits [ 0 ] ) ;
ssSetTNextWasAdjustedPtr ( rtS , & mdlTNextWasAdjustedPtr [ 0 ] ) ;
ssSetPerTaskSampleHitsPtr ( rtS , & mdlPerTaskSampleHits [ 0 ] ) ;
ssSetTimeOfNextSampleHitPtr ( rtS , & mdlTimeOfNextSampleHit [ 0 ] ) ; }
ssSetSolverMode ( rtS , SOLVER_MODE_SINGLETASKING ) ; { ssSetBlockIO ( rtS ,
( ( void * ) & rtB ) ) ; ssSetWorkSizeInBytes ( rtS , sizeof ( rtB ) ,
"BlockIO" ) ; ( void ) memset ( ( ( void * ) & rtB ) , 0 , sizeof ( B ) ) ; }
{ real_T * x = ( real_T * ) & rtX ; ssSetContStates ( rtS , x ) ; ( void )
memset ( ( void * ) x , 0 , sizeof ( X ) ) ; } { void * dwork = ( void * ) &
rtDW ; ssSetRootDWork ( rtS , dwork ) ; ssSetWorkSizeInBytes ( rtS , sizeof (
rtDW ) , "DWork" ) ; ( void ) memset ( dwork , 0 , sizeof ( DW ) ) ; } {
static DataTypeTransInfo dtInfo ; ( void ) memset ( ( char_T * ) & dtInfo , 0
, sizeof ( dtInfo ) ) ; ssSetModelMappingInfo ( rtS , & dtInfo ) ; dtInfo .
numDataTypes = 14 ; dtInfo . dataTypeSizes = & rtDataTypeSizes [ 0 ] ; dtInfo
. dataTypeNames = & rtDataTypeNames [ 0 ] ; dtInfo . BTransTable = &
rtBTransTable ; dtInfo . PTransTable = & rtPTransTable ; dtInfo .
dataTypeInfoTable = rtDataTypeInfoTable ; } wheel_leg_InitializeDataMapInfo (
) ; ssSetIsRapidAcceleratorActive ( rtS , true ) ; ssSetRootSS ( rtS , rtS )
; ssSetVersion ( rtS , SIMSTRUCT_VERSION_LEVEL2 ) ; ssSetModelName ( rtS ,
"wheel_leg" ) ; ssSetPath ( rtS , "wheel_leg" ) ; ssSetTStart ( rtS , 0.0 ) ;
ssSetTFinal ( rtS , 4.0 ) ; { static RTWLogInfo rt_DataLoggingInfo ;
rt_DataLoggingInfo . loggingInterval = NULL ; ssSetRTWLogInfo ( rtS , &
rt_DataLoggingInfo ) ; } { { static int_T rt_LoggedStateWidths [ ] = { 1 , 1
, 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 ,
1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 2 , 2
, 2 , 2 , 2 , 2 } ; static int_T rt_LoggedStateNumDimensions [ ] = { 1 , 1 ,
1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1
, 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 ,
1 , 1 , 1 , 1 } ; static int_T rt_LoggedStateDimensions [ ] = { 1 , 1 , 1 , 1
, 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 ,
1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 2 , 2 , 2 , 2
, 2 , 2 } ; static boolean_T rt_LoggedStateIsVarDims [ ] = { 0 , 0 , 0 , 0 ,
0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0
, 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 ,
0 , 0 } ; static BuiltInDTypeId rt_LoggedStateDataTypeIds [ ] = { SS_DOUBLE ,
SS_DOUBLE , SS_DOUBLE , SS_DOUBLE , SS_DOUBLE , SS_DOUBLE , SS_DOUBLE ,
SS_DOUBLE , SS_DOUBLE , SS_DOUBLE , SS_DOUBLE , SS_DOUBLE , SS_DOUBLE ,
SS_DOUBLE , SS_DOUBLE , SS_DOUBLE , SS_DOUBLE , SS_DOUBLE , SS_DOUBLE ,
SS_DOUBLE , SS_DOUBLE , SS_DOUBLE , SS_DOUBLE , SS_DOUBLE , SS_DOUBLE ,
SS_DOUBLE , SS_DOUBLE , SS_DOUBLE , SS_DOUBLE , SS_DOUBLE , SS_DOUBLE ,
SS_DOUBLE , SS_DOUBLE , SS_DOUBLE , SS_DOUBLE , SS_DOUBLE , SS_DOUBLE ,
SS_DOUBLE , SS_DOUBLE , SS_DOUBLE , SS_DOUBLE , SS_DOUBLE , SS_DOUBLE ,
SS_DOUBLE , SS_DOUBLE } ; static int_T rt_LoggedStateComplexSignals [ ] = { 0
, 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 ,
0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0
, 0 , 0 , 0 , 0 , 0 } ; static RTWPreprocessingFcnPtr
rt_LoggingStatePreprocessingFcnPtrs [ ] = { ( NULL ) , ( NULL ) , ( NULL ) ,
( NULL ) , ( NULL ) , ( NULL ) , ( NULL ) , ( NULL ) , ( NULL ) , ( NULL ) ,
( NULL ) , ( NULL ) , ( NULL ) , ( NULL ) , ( NULL ) , ( NULL ) , ( NULL ) ,
( NULL ) , ( NULL ) , ( NULL ) , ( NULL ) , ( NULL ) , ( NULL ) , ( NULL ) ,
( NULL ) , ( NULL ) , ( NULL ) , ( NULL ) , ( NULL ) , ( NULL ) , ( NULL ) ,
( NULL ) , ( NULL ) , ( NULL ) , ( NULL ) , ( NULL ) , ( NULL ) , ( NULL ) ,
( NULL ) , ( NULL ) , ( NULL ) , ( NULL ) , ( NULL ) , ( NULL ) , ( NULL ) }
; static const char_T * rt_LoggedStateLabels [ ] = { "CSTATE" , "CSTATE" ,
"CSTATE" , "CSTATE" , "CSTATE" , "CSTATE" , "CSTATE" , "CSTATE" , "CSTATE" ,
"CSTATE" , "CSTATE" , "CSTATE" , "CSTATE" , "CSTATE" , "CSTATE" , "CSTATE" ,
"CSTATE" , "CSTATE" , "CSTATE" , "CSTATE" , "CSTATE" , "CSTATE" , "CSTATE" ,
"CSTATE" , "CSTATE" , "CSTATE" , "CSTATE" , "CSTATE" , "CSTATE" , "CSTATE" ,
"CSTATE" , "CSTATE" , "CSTATE" , "CSTATE" , "CSTATE" , "CSTATE" , "CSTATE" ,
"CSTATE" , "CSTATE" , "Discrete" , "Discrete" , "Discrete" , "Discrete" ,
"Discrete" , "Discrete" } ; static const char_T * rt_LoggedStateBlockNames [
] = { "wheel_leg/blance_leg_model/Cartesian Joint1" ,
"wheel_leg/blance_leg_model/Cartesian Joint1" ,
"wheel_leg/blance_leg_model/Cartesian Joint1" ,
"wheel_leg/blance_leg_model/Cartesian Joint1" ,
"wheel_leg/blance_leg_model/Cartesian Joint1" ,
"wheel_leg/blance_leg_model/Cartesian Joint1" ,
"wheel_leg/blance_leg_model/pitch" , "wheel_leg/blance_leg_model/pitch" ,
"wheel_leg/blance_leg_model/left joint 1" ,
"wheel_leg/blance_leg_model/left joint 1" ,
"wheel_leg/blance_leg_model/Revolute Joint3" ,
"wheel_leg/blance_leg_model/Revolute Joint3" ,
"wheel_leg/blance_leg_model/Revolute Joint5" ,
"wheel_leg/blance_leg_model/Revolute Joint5" ,
"wheel_leg/blance_leg_model/left joint 4 " ,
"wheel_leg/blance_leg_model/left joint 4 " ,
"wheel_leg/blance_leg_model/Revolute Joint2" ,
"wheel_leg/blance_leg_model/Revolute Joint2" ,
"wheel_leg/blance_leg_model/right joint1" ,
"wheel_leg/blance_leg_model/right joint1" ,
"wheel_leg/blance_leg_model/Revolute Joint6" ,
"wheel_leg/blance_leg_model/Revolute Joint6" ,
"wheel_leg/blance_leg_model/Revolute Joint8" ,
"wheel_leg/blance_leg_model/Revolute Joint8" ,
"wheel_leg/blance_leg_model/right joint 4" ,
"wheel_leg/blance_leg_model/right joint 4" ,
"wheel_leg/blance_leg_model/Revolute Joint1" ,
"wheel_leg/blance_leg_model/Revolute Joint1" ,
"wheel_leg/blance_leg_model/Revolute Joint4" ,
"wheel_leg/blance_leg_model/Revolute Joint4" ,
"wheel_leg/blance_leg_model/Revolute Joint7" ,
"wheel_leg/blance_leg_model/Revolute Joint7" , "wheel_leg/Integrator" ,
"wheel_leg/PID Controller3/Integrator/Continuous/Integrator" ,
"wheel_leg/PID Controller3/Filter/Cont. Filter/Filter" ,
"wheel_leg/PID Controller2/Filter/Cont. Filter/Filter" ,
"wheel_leg/PID Controller2/Integrator/Continuous/Integrator" ,
"wheel_leg/Integrator4" , "wheel_leg/Integrator2" ,
"wheel_leg/blance_leg_model/Solver\nConfiguration1/EVAL_KEY/INPUT_6_1_1" ,
"wheel_leg/blance_leg_model/Solver\nConfiguration1/EVAL_KEY/INPUT_4_1_1" ,
"wheel_leg/blance_leg_model/Solver\nConfiguration1/EVAL_KEY/INPUT_1_1_1" ,
"wheel_leg/blance_leg_model/Solver\nConfiguration1/EVAL_KEY/INPUT_3_1_1" ,
"wheel_leg/blance_leg_model/Solver\nConfiguration1/EVAL_KEY/INPUT_5_1_1" ,
"wheel_leg/blance_leg_model/Solver\nConfiguration1/EVAL_KEY/INPUT_2_1_1" } ;
static const char_T * rt_LoggedStateNames [ ] = {
"wheel_leg.blance_leg_model.Cartesian_Joint1.Px.p" ,
"wheel_leg.blance_leg_model.Cartesian_Joint1.Py.p" ,
"wheel_leg.blance_leg_model.Cartesian_Joint1.Pz.p" ,
"wheel_leg.blance_leg_model.Cartesian_Joint1.Px.v" ,
"wheel_leg.blance_leg_model.Cartesian_Joint1.Py.v" ,
"wheel_leg.blance_leg_model.Cartesian_Joint1.Pz.v" ,
"wheel_leg.blance_leg_model.pitch.Rz.q" ,
"wheel_leg.blance_leg_model.pitch.Rz.w" ,
"wheel_leg.blance_leg_model.left_joint_1.Rz.q" ,
"wheel_leg.blance_leg_model.left_joint_1.Rz.w" ,
"wheel_leg.blance_leg_model.Revolute_Joint3.Rz.q" ,
"wheel_leg.blance_leg_model.Revolute_Joint3.Rz.w" ,
"wheel_leg.blance_leg_model.Revolute_Joint5.Rz.q" ,
"wheel_leg.blance_leg_model.Revolute_Joint5.Rz.w" ,
"wheel_leg.blance_leg_model.left_joint_4.Rz.q" ,
"wheel_leg.blance_leg_model.left_joint_4.Rz.w" ,
"wheel_leg.blance_leg_model.Revolute_Joint2.Rz.q" ,
"wheel_leg.blance_leg_model.Revolute_Joint2.Rz.w" ,
"wheel_leg.blance_leg_model.right_joint1.Rz.q" ,
"wheel_leg.blance_leg_model.right_joint1.Rz.w" ,
"wheel_leg.blance_leg_model.Revolute_Joint6.Rz.q" ,
"wheel_leg.blance_leg_model.Revolute_Joint6.Rz.w" ,
"wheel_leg.blance_leg_model.Revolute_Joint8.Rz.q" ,
"wheel_leg.blance_leg_model.Revolute_Joint8.Rz.w" ,
"wheel_leg.blance_leg_model.right_joint_4.Rz.q" ,
"wheel_leg.blance_leg_model.right_joint_4.Rz.w" ,
"wheel_leg.blance_leg_model.Revolute_Joint1.Rz.q" ,
"wheel_leg.blance_leg_model.Revolute_Joint1.Rz.w" ,
"wheel_leg.blance_leg_model.Revolute_Joint4.Rz.q" ,
"wheel_leg.blance_leg_model.Revolute_Joint4.Rz.w" ,
"wheel_leg.blance_leg_model.Revolute_Joint7.Rz.q" ,
"wheel_leg.blance_leg_model.Revolute_Joint7.Rz.w" , "" , "" , "" , "" , "" ,
"" , "" , "Discrete" , "Discrete" , "Discrete" , "Discrete" , "Discrete" ,
"Discrete" } ; static boolean_T rt_LoggedStateCrossMdlRef [ ] = { 0 , 0 , 0 ,
0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0
, 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 ,
0 , 0 , 0 } ; static RTWLogDataTypeConvert rt_RTWLogDataTypeConvert [ ] = { {
0 , SS_DOUBLE , SS_DOUBLE , 0 , 0 , 0 , 1.0 , 0 , 0.0 } , { 0 , SS_DOUBLE ,
SS_DOUBLE , 0 , 0 , 0 , 1.0 , 0 , 0.0 } , { 0 , SS_DOUBLE , SS_DOUBLE , 0 , 0
, 0 , 1.0 , 0 , 0.0 } , { 0 , SS_DOUBLE , SS_DOUBLE , 0 , 0 , 0 , 1.0 , 0 ,
0.0 } , { 0 , SS_DOUBLE , SS_DOUBLE , 0 , 0 , 0 , 1.0 , 0 , 0.0 } , { 0 ,
SS_DOUBLE , SS_DOUBLE , 0 , 0 , 0 , 1.0 , 0 , 0.0 } , { 0 , SS_DOUBLE ,
SS_DOUBLE , 0 , 0 , 0 , 1.0 , 0 , 0.0 } , { 0 , SS_DOUBLE , SS_DOUBLE , 0 , 0
, 0 , 1.0 , 0 , 0.0 } , { 0 , SS_DOUBLE , SS_DOUBLE , 0 , 0 , 0 , 1.0 , 0 ,
0.0 } , { 0 , SS_DOUBLE , SS_DOUBLE , 0 , 0 , 0 , 1.0 , 0 , 0.0 } , { 0 ,
SS_DOUBLE , SS_DOUBLE , 0 , 0 , 0 , 1.0 , 0 , 0.0 } , { 0 , SS_DOUBLE ,
SS_DOUBLE , 0 , 0 , 0 , 1.0 , 0 , 0.0 } , { 0 , SS_DOUBLE , SS_DOUBLE , 0 , 0
, 0 , 1.0 , 0 , 0.0 } , { 0 , SS_DOUBLE , SS_DOUBLE , 0 , 0 , 0 , 1.0 , 0 ,
0.0 } , { 0 , SS_DOUBLE , SS_DOUBLE , 0 , 0 , 0 , 1.0 , 0 , 0.0 } , { 0 ,
SS_DOUBLE , SS_DOUBLE , 0 , 0 , 0 , 1.0 , 0 , 0.0 } , { 0 , SS_DOUBLE ,
SS_DOUBLE , 0 , 0 , 0 , 1.0 , 0 , 0.0 } , { 0 , SS_DOUBLE , SS_DOUBLE , 0 , 0
, 0 , 1.0 , 0 , 0.0 } , { 0 , SS_DOUBLE , SS_DOUBLE , 0 , 0 , 0 , 1.0 , 0 ,
0.0 } , { 0 , SS_DOUBLE , SS_DOUBLE , 0 , 0 , 0 , 1.0 , 0 , 0.0 } , { 0 ,
SS_DOUBLE , SS_DOUBLE , 0 , 0 , 0 , 1.0 , 0 , 0.0 } , { 0 , SS_DOUBLE ,
SS_DOUBLE , 0 , 0 , 0 , 1.0 , 0 , 0.0 } , { 0 , SS_DOUBLE , SS_DOUBLE , 0 , 0
, 0 , 1.0 , 0 , 0.0 } , { 0 , SS_DOUBLE , SS_DOUBLE , 0 , 0 , 0 , 1.0 , 0 ,
0.0 } , { 0 , SS_DOUBLE , SS_DOUBLE , 0 , 0 , 0 , 1.0 , 0 , 0.0 } , { 0 ,
SS_DOUBLE , SS_DOUBLE , 0 , 0 , 0 , 1.0 , 0 , 0.0 } , { 0 , SS_DOUBLE ,
SS_DOUBLE , 0 , 0 , 0 , 1.0 , 0 , 0.0 } , { 0 , SS_DOUBLE , SS_DOUBLE , 0 , 0
, 0 , 1.0 , 0 , 0.0 } , { 0 , SS_DOUBLE , SS_DOUBLE , 0 , 0 , 0 , 1.0 , 0 ,
0.0 } , { 0 , SS_DOUBLE , SS_DOUBLE , 0 , 0 , 0 , 1.0 , 0 , 0.0 } , { 0 ,
SS_DOUBLE , SS_DOUBLE , 0 , 0 , 0 , 1.0 , 0 , 0.0 } , { 0 , SS_DOUBLE ,
SS_DOUBLE , 0 , 0 , 0 , 1.0 , 0 , 0.0 } , { 0 , SS_DOUBLE , SS_DOUBLE , 0 , 0
, 0 , 1.0 , 0 , 0.0 } , { 0 , SS_DOUBLE , SS_DOUBLE , 0 , 0 , 0 , 1.0 , 0 ,
0.0 } , { 0 , SS_DOUBLE , SS_DOUBLE , 0 , 0 , 0 , 1.0 , 0 , 0.0 } , { 0 ,
SS_DOUBLE , SS_DOUBLE , 0 , 0 , 0 , 1.0 , 0 , 0.0 } , { 0 , SS_DOUBLE ,
SS_DOUBLE , 0 , 0 , 0 , 1.0 , 0 , 0.0 } , { 0 , SS_DOUBLE , SS_DOUBLE , 0 , 0
, 0 , 1.0 , 0 , 0.0 } , { 0 , SS_DOUBLE , SS_DOUBLE , 0 , 0 , 0 , 1.0 , 0 ,
0.0 } , { 0 , SS_DOUBLE , SS_DOUBLE , 0 , 0 , 0 , 1.0 , 0 , 0.0 } , { 0 ,
SS_DOUBLE , SS_DOUBLE , 0 , 0 , 0 , 1.0 , 0 , 0.0 } , { 0 , SS_DOUBLE ,
SS_DOUBLE , 0 , 0 , 0 , 1.0 , 0 , 0.0 } , { 0 , SS_DOUBLE , SS_DOUBLE , 0 , 0
, 0 , 1.0 , 0 , 0.0 } , { 0 , SS_DOUBLE , SS_DOUBLE , 0 , 0 , 0 , 1.0 , 0 ,
0.0 } , { 0 , SS_DOUBLE , SS_DOUBLE , 0 , 0 , 0 , 1.0 , 0 , 0.0 } } ; static
RTWLogSignalInfo rt_LoggedStateSignalInfo = { 45 , rt_LoggedStateWidths ,
rt_LoggedStateNumDimensions , rt_LoggedStateDimensions ,
rt_LoggedStateIsVarDims , ( NULL ) , ( NULL ) , rt_LoggedStateDataTypeIds ,
rt_LoggedStateComplexSignals , ( NULL ) , rt_LoggingStatePreprocessingFcnPtrs
, { rt_LoggedStateLabels } , ( NULL ) , ( NULL ) , ( NULL ) , {
rt_LoggedStateBlockNames } , { rt_LoggedStateNames } ,
rt_LoggedStateCrossMdlRef , rt_RTWLogDataTypeConvert } ; static void *
rt_LoggedStateSignalPtrs [ 45 ] ; rtliSetLogXSignalPtrs ( ssGetRTWLogInfo (
rtS ) , ( LogSignalPtrsType ) rt_LoggedStateSignalPtrs ) ;
rtliSetLogXSignalInfo ( ssGetRTWLogInfo ( rtS ) , & rt_LoggedStateSignalInfo
) ; rt_LoggedStateSignalPtrs [ 0 ] = ( void * ) & rtX . mefcq5joqv [ 0 ] ;
rt_LoggedStateSignalPtrs [ 1 ] = ( void * ) & rtX . mefcq5joqv [ 1 ] ;
rt_LoggedStateSignalPtrs [ 2 ] = ( void * ) & rtX . mefcq5joqv [ 2 ] ;
rt_LoggedStateSignalPtrs [ 3 ] = ( void * ) & rtX . mefcq5joqv [ 3 ] ;
rt_LoggedStateSignalPtrs [ 4 ] = ( void * ) & rtX . mefcq5joqv [ 4 ] ;
rt_LoggedStateSignalPtrs [ 5 ] = ( void * ) & rtX . mefcq5joqv [ 5 ] ;
rt_LoggedStateSignalPtrs [ 6 ] = ( void * ) & rtX . mefcq5joqv [ 6 ] ;
rt_LoggedStateSignalPtrs [ 7 ] = ( void * ) & rtX . mefcq5joqv [ 7 ] ;
rt_LoggedStateSignalPtrs [ 8 ] = ( void * ) & rtX . mefcq5joqv [ 8 ] ;
rt_LoggedStateSignalPtrs [ 9 ] = ( void * ) & rtX . mefcq5joqv [ 9 ] ;
rt_LoggedStateSignalPtrs [ 10 ] = ( void * ) & rtX . mefcq5joqv [ 10 ] ;
rt_LoggedStateSignalPtrs [ 11 ] = ( void * ) & rtX . mefcq5joqv [ 11 ] ;
rt_LoggedStateSignalPtrs [ 12 ] = ( void * ) & rtX . mefcq5joqv [ 12 ] ;
rt_LoggedStateSignalPtrs [ 13 ] = ( void * ) & rtX . mefcq5joqv [ 13 ] ;
rt_LoggedStateSignalPtrs [ 14 ] = ( void * ) & rtX . mefcq5joqv [ 14 ] ;
rt_LoggedStateSignalPtrs [ 15 ] = ( void * ) & rtX . mefcq5joqv [ 15 ] ;
rt_LoggedStateSignalPtrs [ 16 ] = ( void * ) & rtX . mefcq5joqv [ 16 ] ;
rt_LoggedStateSignalPtrs [ 17 ] = ( void * ) & rtX . mefcq5joqv [ 17 ] ;
rt_LoggedStateSignalPtrs [ 18 ] = ( void * ) & rtX . mefcq5joqv [ 18 ] ;
rt_LoggedStateSignalPtrs [ 19 ] = ( void * ) & rtX . mefcq5joqv [ 19 ] ;
rt_LoggedStateSignalPtrs [ 20 ] = ( void * ) & rtX . mefcq5joqv [ 20 ] ;
rt_LoggedStateSignalPtrs [ 21 ] = ( void * ) & rtX . mefcq5joqv [ 21 ] ;
rt_LoggedStateSignalPtrs [ 22 ] = ( void * ) & rtX . mefcq5joqv [ 22 ] ;
rt_LoggedStateSignalPtrs [ 23 ] = ( void * ) & rtX . mefcq5joqv [ 23 ] ;
rt_LoggedStateSignalPtrs [ 24 ] = ( void * ) & rtX . mefcq5joqv [ 24 ] ;
rt_LoggedStateSignalPtrs [ 25 ] = ( void * ) & rtX . mefcq5joqv [ 25 ] ;
rt_LoggedStateSignalPtrs [ 26 ] = ( void * ) & rtX . mefcq5joqv [ 26 ] ;
rt_LoggedStateSignalPtrs [ 27 ] = ( void * ) & rtX . mefcq5joqv [ 27 ] ;
rt_LoggedStateSignalPtrs [ 28 ] = ( void * ) & rtX . mefcq5joqv [ 28 ] ;
rt_LoggedStateSignalPtrs [ 29 ] = ( void * ) & rtX . mefcq5joqv [ 29 ] ;
rt_LoggedStateSignalPtrs [ 30 ] = ( void * ) & rtX . mefcq5joqv [ 30 ] ;
rt_LoggedStateSignalPtrs [ 31 ] = ( void * ) & rtX . mefcq5joqv [ 31 ] ;
rt_LoggedStateSignalPtrs [ 32 ] = ( void * ) & rtX . cbywjryyoi ;
rt_LoggedStateSignalPtrs [ 33 ] = ( void * ) & rtX . b2hgpxvjzc ;
rt_LoggedStateSignalPtrs [ 34 ] = ( void * ) & rtX . abxab3djbi ;
rt_LoggedStateSignalPtrs [ 35 ] = ( void * ) & rtX . p5p3ogk4mi ;
rt_LoggedStateSignalPtrs [ 36 ] = ( void * ) & rtX . cwaruwgj0n ;
rt_LoggedStateSignalPtrs [ 37 ] = ( void * ) & rtX . ecnkbzok1f ;
rt_LoggedStateSignalPtrs [ 38 ] = ( void * ) & rtX . iko4pycqyt ;
rt_LoggedStateSignalPtrs [ 39 ] = ( void * ) rtDW . pvnlmwwllm ;
rt_LoggedStateSignalPtrs [ 40 ] = ( void * ) rtDW . mxlt45ij3o ;
rt_LoggedStateSignalPtrs [ 41 ] = ( void * ) rtDW . f1r4g4whvp ;
rt_LoggedStateSignalPtrs [ 42 ] = ( void * ) rtDW . aokoeghayj ;
rt_LoggedStateSignalPtrs [ 43 ] = ( void * ) rtDW . kulfjc4a2j ;
rt_LoggedStateSignalPtrs [ 44 ] = ( void * ) rtDW . leayjggnf1 ; }
rtliSetLogT ( ssGetRTWLogInfo ( rtS ) , "tout" ) ; rtliSetLogX (
ssGetRTWLogInfo ( rtS ) , "" ) ; rtliSetLogXFinal ( ssGetRTWLogInfo ( rtS ) ,
"" ) ; rtliSetLogVarNameModifier ( ssGetRTWLogInfo ( rtS ) , "none" ) ;
rtliSetLogFormat ( ssGetRTWLogInfo ( rtS ) , 4 ) ; rtliSetLogMaxRows (
ssGetRTWLogInfo ( rtS ) , 10000 ) ; rtliSetLogDecimation ( ssGetRTWLogInfo (
rtS ) , 1 ) ; rtliSetLogY ( ssGetRTWLogInfo ( rtS ) , "" ) ;
rtliSetLogYSignalInfo ( ssGetRTWLogInfo ( rtS ) , ( NULL ) ) ;
rtliSetLogYSignalPtrs ( ssGetRTWLogInfo ( rtS ) , ( NULL ) ) ; } { static
struct _ssStatesInfo2 statesInfo2 ; ssSetStatesInfo2 ( rtS , & statesInfo2 )
; } { static ssPeriodicStatesInfo periodicStatesInfo ;
ssSetPeriodicStatesInfo ( rtS , & periodicStatesInfo ) ; } { static
ssJacobianPerturbationBounds jacobianPerturbationBounds ;
ssSetJacobianPerturbationBounds ( rtS , & jacobianPerturbationBounds ) ; } {
static ssSolverInfo slvrInfo ; static struct _ssSFcnModelMethods3 mdlMethods3
; static struct _ssSFcnModelMethods2 mdlMethods2 ; static boolean_T
contStatesDisabled [ 39 ] ; static real_T absTol [ 39 ] = { 0.001 , 0.001 ,
0.001 , 0.001 , 0.001 , 0.001 , 0.001 , 0.001 , 0.001 , 0.001 , 0.001 , 0.001
, 0.001 , 0.001 , 0.001 , 0.001 , 0.001 , 0.001 , 0.001 , 0.001 , 0.001 ,
0.001 , 0.001 , 0.001 , 0.001 , 0.001 , 0.001 , 0.001 , 0.001 , 0.001 , 0.001
, 0.001 , 0.001 , 0.001 , 0.001 , 0.001 , 0.001 , 0.001 , 0.001 } ; static
uint8_T absTolControl [ 39 ] = { 2U , 2U , 2U , 2U , 2U , 2U , 2U , 2U , 2U ,
2U , 2U , 2U , 2U , 2U , 2U , 2U , 2U , 2U , 2U , 2U , 2U , 2U , 2U , 2U , 2U
, 2U , 2U , 2U , 2U , 2U , 2U , 2U , 2U , 2U , 2U , 2U , 2U , 2U , 2U } ;
static real_T contStateJacPerturbBoundMinVec [ 39 ] ; static real_T
contStateJacPerturbBoundMaxVec [ 39 ] ; static uint8_T zcAttributes [ 31 ] =
{ ( ZC_EVENT_ALL_UP ) , ( ZC_EVENT_ALL_UP ) , ( ZC_EVENT_ALL_UP ) , (
ZC_EVENT_ALL_UP ) , ( ZC_EVENT_ALL_UP ) , ( ZC_EVENT_ALL_UP ) , (
ZC_EVENT_ALL_UP ) , ( ZC_EVENT_ALL_UP ) , ( ZC_EVENT_ALL_UP ) , (
ZC_EVENT_ALL_UP ) , ( ZC_EVENT_ALL_UP ) , ( ZC_EVENT_ALL_UP ) , (
ZC_EVENT_ALL ) , ( ZC_EVENT_ALL ) , ( ZC_EVENT_ALL_UP ) , ( ZC_EVENT_ALL_UP )
, ( ZC_EVENT_ALL_UP ) , ( ZC_EVENT_ALL_UP ) , ( ZC_EVENT_ALL ) , (
ZC_EVENT_ALL ) , ( ZC_EVENT_ALL ) , ( ZC_EVENT_ALL ) , ( ZC_EVENT_ALL ) , (
ZC_EVENT_ALL ) , ( ZC_EVENT_ALL ) , ( ZC_EVENT_ALL ) , ( ZC_EVENT_ALL ) , (
ZC_EVENT_ALL ) , ( ZC_EVENT_ALL_UP ) , ( ZC_EVENT_ALL_UP ) , ( 0xc0 |
ZC_EVENT_ALL_UP ) } ; static ssNonContDerivSigInfo nonContDerivSigInfo [ 5 ]
= { { 1 * sizeof ( real_T ) , ( char * ) ( & rtB . fviua014cu ) , ( NULL ) }
, { 1 * sizeof ( real_T ) , ( char * ) ( & rtB . djd0cbchne ) , ( NULL ) } ,
{ 1 * sizeof ( real_T ) , ( char * ) ( & rtB . hdeqxnl10m ) , ( NULL ) } , {
1 * sizeof ( real_T ) , ( char * ) ( & rtB . fasi4utync ) , ( NULL ) } , { 1
* sizeof ( real_T ) , ( char * ) ( & rtB . hhykaoe4za ) , ( NULL ) } } ; {
int i ; for ( i = 0 ; i < 39 ; ++ i ) { contStateJacPerturbBoundMinVec [ i ]
= 0 ; contStateJacPerturbBoundMaxVec [ i ] = rtGetInf ( ) ; } }
ssSetSolverRelTol ( rtS , 0.001 ) ; ssSetStepSize ( rtS , 0.0 ) ;
ssSetMinStepSize ( rtS , 0.0 ) ; ssSetMaxNumMinSteps ( rtS , - 1 ) ;
ssSetMinStepViolatedError ( rtS , 0 ) ; ssSetMaxStepSize ( rtS , 0.01 ) ;
ssSetSolverMaxOrder ( rtS , - 1 ) ; ssSetSolverRefineFactor ( rtS , 1 ) ;
ssSetOutputTimes ( rtS , ( NULL ) ) ; ssSetNumOutputTimes ( rtS , 0 ) ;
ssSetOutputTimesOnly ( rtS , 0 ) ; ssSetOutputTimesIndex ( rtS , 0 ) ;
ssSetZCCacheNeedsReset ( rtS , 1 ) ; ssSetDerivCacheNeedsReset ( rtS , 0 ) ;
ssSetNumNonContDerivSigInfos ( rtS , 5 ) ; ssSetNonContDerivSigInfos ( rtS ,
nonContDerivSigInfo ) ; ssSetSolverInfo ( rtS , & slvrInfo ) ;
ssSetSolverName ( rtS , "VariableStepAuto" ) ; ssSetVariableStepSolver ( rtS
, 1 ) ; ssSetSolverConsistencyChecking ( rtS , 0 ) ;
ssSetSolverAdaptiveZcDetection ( rtS , 0 ) ; ssSetSolverRobustResetMethod (
rtS , 0 ) ; _ssSetSolverUpdateJacobianAtReset ( rtS , true ) ;
ssSetAbsTolVector ( rtS , absTol ) ; ssSetAbsTolControlVector ( rtS ,
absTolControl ) ; ssSetSolverAbsTol_Obsolete ( rtS , absTol ) ;
ssSetSolverAbsTolControl_Obsolete ( rtS , absTolControl ) ;
ssSetJacobianPerturbationBoundsMinVec ( rtS , contStateJacPerturbBoundMinVec
) ; ssSetJacobianPerturbationBoundsMaxVec ( rtS ,
contStateJacPerturbBoundMaxVec ) ; ssSetSolverStateProjection ( rtS , 1 ) ; (
void ) memset ( ( void * ) & mdlMethods2 , 0 , sizeof ( mdlMethods2 ) ) ;
ssSetModelMethods2 ( rtS , & mdlMethods2 ) ; ( void ) memset ( ( void * ) &
mdlMethods3 , 0 , sizeof ( mdlMethods3 ) ) ; ssSetModelMethods3 ( rtS , &
mdlMethods3 ) ; ssSetModelProjection ( rtS , MdlProjection ) ;
ssSetSolverMassMatrixType ( rtS , ( ssMatrixType ) 0 ) ;
ssSetSolverMassMatrixNzMax ( rtS , 0 ) ; ssSetModelOutputs ( rtS , MdlOutputs
) ; ssSetModelLogData ( rtS , rt_UpdateTXYLogVars ) ;
ssSetModelLogDataIfInInterval ( rtS , rt_UpdateTXXFYLogVars ) ;
ssSetModelUpdate ( rtS , MdlUpdate ) ; ssSetModelDerivatives ( rtS ,
MdlDerivatives ) ; ssSetSolverZcSignalAttrib ( rtS , zcAttributes ) ;
ssSetSolverNumZcSignals ( rtS , 31 ) ; ssSetModelZeroCrossings ( rtS ,
MdlZeroCrossings ) ; ssSetSolverConsecutiveZCsStepRelTol ( rtS ,
2.8421709430404007E-13 ) ; ssSetSolverMaxConsecutiveZCs ( rtS , 1000 ) ;
ssSetSolverConsecutiveZCsError ( rtS , 2 ) ; ssSetSolverMaskedZcDiagnostic (
rtS , 1 ) ; ssSetSolverIgnoredZcDiagnostic ( rtS , 1 ) ;
ssSetSolverMaxConsecutiveMinStep ( rtS , 1 ) ;
ssSetSolverShapePreserveControl ( rtS , 2 ) ; ssSetTNextTid ( rtS , INT_MIN )
; ssSetTNext ( rtS , rtMinusInf ) ; ssSetSolverNeedsReset ( rtS ) ;
ssSetNumNonsampledZCs ( rtS , 30 ) ; ssSetContStateDisabled ( rtS ,
contStatesDisabled ) ; ssSetSolverMaxConsecutiveMinStep ( rtS , 1 ) ; } {
ZCSigState * zc = ( ZCSigState * ) & rtPrevZCX ; ssSetPrevZCSigState ( rtS ,
zc ) ; ssSetWorkSizeInBytes ( rtS , sizeof ( rtPrevZCX ) , "PrevZCSigState" )
; } { rtPrevZCX . hcfhnl2xxl = UNINITIALIZED_ZCSIG ; } ssSetChecksumVal ( rtS
, 0 , 3625382941U ) ; ssSetChecksumVal ( rtS , 1 , 3387521915U ) ;
ssSetChecksumVal ( rtS , 2 , 3494444731U ) ; ssSetChecksumVal ( rtS , 3 ,
3437323851U ) ; { static const sysRanDType rtAlwaysEnabled =
SUBSYS_RAN_BC_ENABLE ; static RTWExtModeInfo rt_ExtModeInfo ; static const
sysRanDType * systemRan [ 8 ] ; gblRTWExtModeInfo = & rt_ExtModeInfo ;
ssSetRTWExtModeInfo ( rtS , & rt_ExtModeInfo ) ;
rteiSetSubSystemActiveVectorAddresses ( & rt_ExtModeInfo , systemRan ) ;
systemRan [ 0 ] = & rtAlwaysEnabled ; systemRan [ 1 ] = & rtAlwaysEnabled ;
systemRan [ 2 ] = & rtAlwaysEnabled ; systemRan [ 3 ] = & rtAlwaysEnabled ;
systemRan [ 4 ] = & rtAlwaysEnabled ; systemRan [ 5 ] = & rtAlwaysEnabled ;
systemRan [ 6 ] = ( sysRanDType * ) & rtDW . kpjnyoicn4 ; systemRan [ 7 ] = (
sysRanDType * ) & rtDW . kpjnyoicn4 ; rteiSetModelMappingInfoPtr (
ssGetRTWExtModeInfo ( rtS ) , & ssGetModelMappingInfo ( rtS ) ) ;
rteiSetChecksumsPtr ( ssGetRTWExtModeInfo ( rtS ) , ssGetChecksums ( rtS ) )
; rteiSetTPtr ( ssGetRTWExtModeInfo ( rtS ) , ssGetTPtr ( rtS ) ) ; } return
rtS ; }
#if defined(_MSC_VER)
#pragma optimize( "", on )
#endif
const int_T gblParameterTuningTid = 2 ; void MdlOutputsParameterSampleTime (
int_T tid ) { MdlOutputsTID2 ( tid ) ; }
