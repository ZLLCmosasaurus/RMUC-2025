#include <math.h>
#include <string.h>
#include "pm_std.h"
#include "sm_std.h"
#include "ne_std.h"
#include "ne_dae.h"
#include "sm_ssci_run_time_errors.h"
#include "sm_RuntimeDerivedValuesBundle.h"
const sm_core_compiler_Brick * wheel_leg_d9e82f16_1_geometry_0 ( const
RuntimeDerivedValuesBundle * rtdv ) { static const sm_core_compiler_Brick
brick = { 1.0 , 1.0 , 0.2 } ; ( void ) rtdv ; return & brick ; }
sm_core_compiler_ConvexPolyhedron * wheel_leg_d9e82f16_1_geometry_1 ( const
RuntimeDerivedValuesBundle * rtdv ) { static const double hull_points [ 24 ]
= { 0.1 , 0.0 , 0.5 , 0.075 , 0.025 , 0.5 , 0.0 , 0.0 , 0.5 , 0.1 , 0.0 , -
0.5 , 0.025 , 0.025 , 0.5 , 0.075 , 0.025 , - 0.5 , 0.025 , 0.025 , - 0.5 ,
0.0 , 0.0 , - 0.5 } ; static const int vx_index [ 8 ] = { 0 , 3 , 8 , 14 , 19
, 23 , 28 , 32 } ; static const int vx_valency [ 8 ] = { 3 , 5 , 6 , 5 , 4 ,
5 , 4 , 4 } ; static const int vx_adjacency_graph [ 36 ] = { 1 , 2 , 3 , 2 ,
0 , 3 , 5 , 4 , 0 , 1 , 4 , 6 , 7 , 3 , 0 , 2 , 7 , 5 , 1 , 2 , 1 , 5 , 6 , 1
, 3 , 7 , 6 , 4 , 4 , 5 , 7 , 2 , 2 , 6 , 5 , 3 } ; static const int
seed_vx_ids [ 8 ] = { 0 , 4 , 2 , 0 , 3 , 7 , 7 , 3 } ; static
sm_core_compiler_ConvexPolyhedron cxh ; cxh . hullPoints = hull_points ; cxh
. vxIndex = vx_index ; cxh . vxValency = vx_valency ; cxh . numHullPoints = 8
; cxh . vxAdjacencyGraph = vx_adjacency_graph ; cxh . graphSize = 36 ; cxh .
seedVxIds = seed_vx_ids ; cxh . diagonalAABB = 1.005298463144155 ; ( void )
rtdv ; return & cxh ; } const sm_core_compiler_Cylinder *
wheel_leg_d9e82f16_1_geometry_2 ( const RuntimeDerivedValuesBundle * rtdv ) {
static const sm_core_compiler_Cylinder cylinder = { 0.08599999999999999 ,
0.04 } ; ( void ) rtdv ; return & cylinder ; }
sm_core_compiler_ConvexPolyhedron * wheel_leg_d9e82f16_1_geometry_3 ( const
RuntimeDerivedValuesBundle * rtdv ) { static const double hull_points [ 18 ]
= { 1.73 , 0.0 , 0.5 , 0.0 , 0.5 , 0.5 , 0.0 , 0.0 , 0.5 , 1.73 , 0.0 , - 0.5
, 0.0 , 0.5 , - 0.5 , 0.0 , 0.0 , - 0.5 } ; static const int vx_index [ 6 ] =
{ 0 , 3 , 7 , 12 , 17 , 21 } ; static const int vx_valency [ 6 ] = { 3 , 4 ,
5 , 5 , 4 , 3 } ; static const int vx_adjacency_graph [ 24 ] = { 1 , 2 , 3 ,
2 , 0 , 3 , 4 , 0 , 1 , 4 , 5 , 3 , 0 , 2 , 5 , 4 , 1 , 1 , 3 , 5 , 2 , 2 , 4
, 3 } ; static const int seed_vx_ids [ 8 ] = { 0 , 1 , 2 , 0 , 3 , 4 , 5 , 3
} ; static sm_core_compiler_ConvexPolyhedron cxh ; cxh . hullPoints =
hull_points ; cxh . vxIndex = vx_index ; cxh . vxValency = vx_valency ; cxh .
numHullPoints = 6 ; cxh . vxAdjacencyGraph = vx_adjacency_graph ; cxh .
graphSize = 24 ; cxh . seedVxIds = seed_vx_ids ; cxh . diagonalAABB =
2.059830090080248 ; ( void ) rtdv ; return & cxh ; }
sm_core_compiler_ConvexPolyhedron * wheel_leg_d9e82f16_1_geometry_4 ( const
RuntimeDerivedValuesBundle * rtdv ) { static const double hull_points [ 18 ]
= { 1.73 , 0.0 , 0.25 , 0.0 , 0.2 , 0.25 , 0.0 , 0.0 , 0.25 , 1.73 , 0.0 , -
0.25 , 0.0 , 0.2 , - 0.25 , 0.0 , 0.0 , - 0.25 } ; static const int vx_index
[ 6 ] = { 0 , 3 , 7 , 12 , 17 , 21 } ; static const int vx_valency [ 6 ] = {
3 , 4 , 5 , 5 , 4 , 3 } ; static const int vx_adjacency_graph [ 24 ] = { 1 ,
2 , 3 , 2 , 0 , 3 , 4 , 0 , 1 , 4 , 5 , 3 , 0 , 2 , 5 , 4 , 1 , 1 , 3 , 5 , 2
, 2 , 4 , 3 } ; static const int seed_vx_ids [ 8 ] = { 0 , 1 , 2 , 0 , 3 , 4
, 5 , 3 } ; static sm_core_compiler_ConvexPolyhedron cxh ; cxh . hullPoints =
hull_points ; cxh . vxIndex = vx_index ; cxh . vxValency = vx_valency ; cxh .
numHullPoints = 6 ; cxh . vxAdjacencyGraph = vx_adjacency_graph ; cxh .
graphSize = 24 ; cxh . seedVxIds = seed_vx_ids ; cxh . diagonalAABB =
1.811877479301512 ; ( void ) rtdv ; return & cxh ; } const
sm_core_compiler_Brick * wheel_leg_d9e82f16_1_geometry_5 ( const
RuntimeDerivedValuesBundle * rtdv ) { static const sm_core_compiler_Brick
brick = { 1.0 , 30.0 , 0.01 } ; ( void ) rtdv ; return & brick ; }
