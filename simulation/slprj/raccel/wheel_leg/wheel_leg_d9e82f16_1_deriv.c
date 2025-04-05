#include <math.h>
#include <string.h>
#include "pm_std.h"
#include "sm_std.h"
#include "ne_std.h"
#include "ne_dae.h"
#include "sm_ssci_run_time_errors.h"
#include "sm_RuntimeDerivedValuesBundle.h"
#include "wheel_leg_d9e82f16_1_geometries.h"
PmfMessageId wheel_leg_d9e82f16_1_compDerivs ( const
RuntimeDerivedValuesBundle * rtdv , const int * eqnEnableFlags , const double
* state , const int * modeVector , const double * input , const double *
inputDot , const double * inputDdot , const double * discreteState , double *
deriv , double * errorResult , NeuDiagnosticManager * neDiagMgr ) { const
double * rtdvd = rtdv -> mDoubles . mValues ; const int * rtdvi = rtdv ->
mInts . mValues ; int ii [ 4 ] ; double xx [ 793 ] ; ( void ) rtdvd ; ( void
) rtdvi ; ( void ) eqnEnableFlags ; ( void ) modeVector ; ( void ) inputDot ;
( void ) inputDdot ; ( void ) discreteState ; ( void ) neDiagMgr ; xx [ 0 ] =
0.7071067811865476 ; xx [ 1 ] = 0.5 ; xx [ 2 ] = xx [ 1 ] * state [ 6 ] ; xx
[ 3 ] = xx [ 0 ] * cos ( xx [ 2 ] ) ; xx [ 4 ] = xx [ 3 ] * xx [ 3 ] ; xx [ 5
] = xx [ 0 ] * sin ( xx [ 2 ] ) ; xx [ 2 ] = xx [ 5 ] * xx [ 5 ] ; xx [ 6 ] =
2.0 ; xx [ 7 ] = 1.0 ; xx [ 8 ] = ( xx [ 4 ] + xx [ 2 ] ) * xx [ 6 ] - xx [ 7
] ; xx [ 9 ] = xx [ 3 ] * xx [ 5 ] ; xx [ 10 ] = xx [ 8 ] ; xx [ 11 ] = - ( (
xx [ 9 ] + xx [ 9 ] ) * xx [ 6 ] ) ; xx [ 12 ] = xx [ 6 ] * ( xx [ 2 ] - xx [
4 ] ) ; xx [ 13 ] = xx [ 6 ] * ( xx [ 9 ] - xx [ 9 ] ) ; xx [ 14 ] = ( xx [ 4
] + xx [ 4 ] ) * xx [ 6 ] - xx [ 7 ] ; xx [ 15 ] = - ( ( xx [ 9 ] + xx [ 9 ]
) * xx [ 6 ] ) ; xx [ 16 ] = ( xx [ 2 ] + xx [ 4 ] ) * xx [ 6 ] ; xx [ 17 ] =
xx [ 6 ] * ( xx [ 9 ] - xx [ 9 ] ) ; xx [ 18 ] = xx [ 8 ] ; xx [ 2 ] = xx [ 1
] * state [ 24 ] ; xx [ 4 ] = xx [ 0 ] * cos ( xx [ 2 ] ) ; xx [ 8 ] = xx [ 4
] * xx [ 4 ] ; xx [ 9 ] = xx [ 0 ] * sin ( xx [ 2 ] ) ; xx [ 2 ] = xx [ 9 ] *
xx [ 9 ] ; xx [ 19 ] = ( xx [ 8 ] + xx [ 2 ] ) * xx [ 6 ] - xx [ 7 ] ; xx [
20 ] = xx [ 4 ] * xx [ 9 ] ; xx [ 21 ] = xx [ 6 ] * ( xx [ 20 ] - xx [ 20 ] )
; xx [ 22 ] = ( xx [ 2 ] + xx [ 8 ] ) * xx [ 6 ] ; xx [ 23 ] = xx [ 20 ] + xx
[ 20 ] ; xx [ 20 ] = xx [ 23 ] * xx [ 6 ] ; xx [ 24 ] = ( xx [ 8 ] + xx [ 8 ]
) * xx [ 6 ] - xx [ 7 ] ; xx [ 25 ] = xx [ 2 ] - xx [ 8 ] ; xx [ 2 ] = xx [ 6
] * xx [ 25 ] ; xx [ 26 ] = xx [ 19 ] ; xx [ 27 ] = xx [ 21 ] ; xx [ 28 ] =
xx [ 22 ] ; xx [ 29 ] = xx [ 20 ] ; xx [ 30 ] = xx [ 24 ] ; xx [ 31 ] = xx [
21 ] ; xx [ 32 ] = xx [ 2 ] ; xx [ 33 ] = xx [ 20 ] ; xx [ 34 ] = xx [ 19 ] ;
xx [ 8 ] = 0.135 ; xx [ 35 ] = 0.375 ; xx [ 36 ] = xx [ 1 ] * state [ 26 ] ;
xx [ 37 ] = cos ( xx [ 36 ] ) ; xx [ 38 ] = xx [ 37 ] * xx [ 37 ] ; xx [ 39 ]
= xx [ 6 ] * xx [ 38 ] - xx [ 7 ] ; xx [ 40 ] = xx [ 35 ] * xx [ 39 ] ; xx [
41 ] = sin ( xx [ 36 ] ) ; xx [ 36 ] = xx [ 37 ] * xx [ 41 ] ; xx [ 42 ] = xx
[ 6 ] * xx [ 36 ] ; xx [ 43 ] = 0.046875 ; xx [ 44 ] = 7.815625e-3 ; ii [ 0 ]
= factorSymmetricPosDef ( xx + 44 , 1 , xx + 45 ) ; if ( ii [ 0 ] != 0 ) {
return sm_ssci_recordRunTimeError (
"sm:compiler:messages:simulationErrors:DegenerateMass" ,
 "'wheel_leg/blance_leg_model/Revolute Joint1' has a degenerate mass distribution on its follower side."
, neDiagMgr ) ; } memcpy ( xx + 45 , xx + 44 , 1 * sizeof ( double ) ) ; xx [
46 ] = xx [ 43 ] / xx [ 45 ] ; xx [ 47 ] = xx [ 35 ] - xx [ 43 ] * xx [ 46 ]
; xx [ 48 ] = xx [ 42 ] * xx [ 47 ] ; xx [ 49 ] = xx [ 40 ] * xx [ 39 ] + xx
[ 42 ] * xx [ 48 ] ; xx [ 50 ] = xx [ 8 ] + xx [ 49 ] ; xx [ 51 ] =
1.95625e-3 ; xx [ 52 ] = xx [ 51 ] * xx [ 46 ] ; xx [ 53 ] = ( xx [ 38 ] + xx
[ 41 ] * xx [ 41 ] ) * xx [ 6 ] - xx [ 7 ] ; xx [ 38 ] = xx [ 52 ] * xx [ 42
] * xx [ 53 ] ; xx [ 54 ] = 0.2 ; xx [ 55 ] = 0.125 ; xx [ 56 ] = xx [ 55 ] *
xx [ 41 ] ; xx [ 57 ] = xx [ 6 ] * xx [ 56 ] * xx [ 41 ] ; xx [ 58 ] = xx [
54 ] - xx [ 57 ] ; xx [ 59 ] = xx [ 35 ] * xx [ 42 ] ; xx [ 60 ] = xx [ 47 ]
* xx [ 39 ] ; xx [ 47 ] = xx [ 59 ] * xx [ 39 ] - xx [ 42 ] * xx [ 60 ] ; xx
[ 61 ] = xx [ 37 ] * xx [ 56 ] ; xx [ 56 ] = xx [ 6 ] * xx [ 61 ] ; xx [ 62 ]
= xx [ 58 ] * xx [ 47 ] - xx [ 49 ] * xx [ 56 ] ; xx [ 49 ] = xx [ 38 ] + xx
[ 62 ] ; xx [ 63 ] = 0.075 ; xx [ 64 ] = xx [ 49 ] + xx [ 63 ] * xx [ 47 ] ;
xx [ 65 ] = 2.5425e-4 ; xx [ 66 ] = xx [ 51 ] / xx [ 45 ] ; xx [ 67 ] = xx [
52 ] * xx [ 39 ] * xx [ 53 ] ; xx [ 52 ] = xx [ 58 ] * xx [ 67 ] + xx [ 56 ]
* xx [ 38 ] ; xx [ 38 ] = xx [ 59 ] * xx [ 42 ] + xx [ 60 ] * xx [ 39 ] ; xx
[ 59 ] = xx [ 40 ] * xx [ 42 ] - xx [ 48 ] * xx [ 39 ] ; xx [ 40 ] = xx [ 38
] * xx [ 58 ] - xx [ 56 ] * xx [ 59 ] ; xx [ 48 ] = xx [ 65 ] + ( xx [ 51 ] -
xx [ 51 ] * xx [ 66 ] ) * xx [ 53 ] * xx [ 53 ] - xx [ 52 ] - xx [ 52 ] + xx
[ 58 ] * xx [ 40 ] - xx [ 62 ] * xx [ 56 ] ; xx [ 52 ] = xx [ 40 ] - xx [ 67
] ; xx [ 40 ] = xx [ 48 ] + xx [ 63 ] * xx [ 52 ] ; xx [ 60 ] = xx [ 8 ] + xx
[ 38 ] ; xx [ 38 ] = xx [ 52 ] + xx [ 60 ] * xx [ 63 ] ; xx [ 62 ] = xx [ 40
] + xx [ 38 ] * xx [ 63 ] ; ii [ 1 ] = factorSymmetricPosDef ( xx + 62 , 1 ,
xx + 67 ) ; if ( ii [ 1 ] != 0 ) { return sm_ssci_recordRunTimeError (
"sm:compiler:messages:simulationErrors:DegenerateMass" ,
 "'wheel_leg/blance_leg_model/right joint 4' has a degenerate mass distribution on its follower side."
, neDiagMgr ) ; } xx [ 67 ] = xx [ 64 ] / xx [ 62 ] ; xx [ 68 ] = xx [ 50 ] -
xx [ 64 ] * xx [ 67 ] ; xx [ 69 ] = xx [ 38 ] * xx [ 67 ] ; xx [ 70 ] = xx [
47 ] - xx [ 69 ] ; xx [ 71 ] = xx [ 59 ] - xx [ 69 ] ; xx [ 69 ] = xx [ 38 ]
/ xx [ 62 ] ; xx [ 72 ] = xx [ 60 ] - xx [ 38 ] * xx [ 69 ] ; xx [ 73 ] = xx
[ 35 ] * xx [ 53 ] * xx [ 53 ] ; xx [ 53 ] = xx [ 8 ] + xx [ 73 ] ; xx [ 74 ]
= xx [ 68 ] * xx [ 19 ] + xx [ 21 ] * xx [ 70 ] ; xx [ 75 ] = xx [ 20 ] * xx
[ 68 ] + xx [ 24 ] * xx [ 70 ] ; xx [ 76 ] = xx [ 2 ] * xx [ 68 ] + xx [ 20 ]
* xx [ 70 ] ; xx [ 77 ] = xx [ 19 ] * xx [ 71 ] + xx [ 21 ] * xx [ 72 ] ; xx
[ 78 ] = xx [ 20 ] * xx [ 71 ] + xx [ 72 ] * xx [ 24 ] ; xx [ 79 ] = xx [ 2 ]
* xx [ 71 ] + xx [ 20 ] * xx [ 72 ] ; xx [ 80 ] = xx [ 53 ] * xx [ 22 ] ; xx
[ 81 ] = xx [ 53 ] * xx [ 21 ] ; xx [ 82 ] = xx [ 53 ] * xx [ 19 ] ;
pm_math_Matrix3x3_compose_ra ( xx + 26 , xx + 74 , xx + 83 ) ; xx [ 68 ] = xx
[ 1 ] * state [ 18 ] ; xx [ 70 ] = xx [ 0 ] * cos ( xx [ 68 ] ) ; xx [ 71 ] =
xx [ 70 ] * xx [ 70 ] ; xx [ 72 ] = xx [ 0 ] * sin ( xx [ 68 ] ) ; xx [ 68 ]
= xx [ 72 ] * xx [ 72 ] ; xx [ 74 ] = ( xx [ 71 ] + xx [ 68 ] ) * xx [ 6 ] -
xx [ 7 ] ; xx [ 75 ] = xx [ 70 ] * xx [ 72 ] ; xx [ 76 ] = xx [ 6 ] * ( xx [
75 ] - xx [ 75 ] ) ; xx [ 77 ] = ( xx [ 68 ] + xx [ 71 ] ) * xx [ 6 ] ; xx [
78 ] = xx [ 75 ] + xx [ 75 ] ; xx [ 75 ] = xx [ 78 ] * xx [ 6 ] ; xx [ 79 ] =
( xx [ 71 ] + xx [ 71 ] ) * xx [ 6 ] - xx [ 7 ] ; xx [ 80 ] = xx [ 68 ] - xx
[ 71 ] ; xx [ 68 ] = xx [ 6 ] * xx [ 80 ] ; xx [ 92 ] = xx [ 74 ] ; xx [ 93 ]
= xx [ 76 ] ; xx [ 94 ] = xx [ 77 ] ; xx [ 95 ] = xx [ 75 ] ; xx [ 96 ] = xx
[ 79 ] ; xx [ 97 ] = xx [ 76 ] ; xx [ 98 ] = xx [ 68 ] ; xx [ 99 ] = xx [ 75
] ; xx [ 100 ] = xx [ 74 ] ; xx [ 71 ] = xx [ 1 ] * state [ 20 ] ; xx [ 81 ]
= cos ( xx [ 71 ] ) ; xx [ 82 ] = xx [ 81 ] * xx [ 81 ] ; xx [ 101 ] = xx [ 6
] * xx [ 82 ] - xx [ 7 ] ; xx [ 102 ] = 0.465 ; xx [ 103 ] = 1.18 ; xx [ 104
] = xx [ 1 ] * state [ 22 ] ; xx [ 105 ] = cos ( xx [ 104 ] ) ; xx [ 106 ] =
xx [ 105 ] * xx [ 105 ] ; xx [ 107 ] = xx [ 6 ] * xx [ 106 ] - xx [ 7 ] ; xx
[ 108 ] = xx [ 103 ] * xx [ 107 ] ; xx [ 109 ] = xx [ 108 ] * xx [ 107 ] ; xx
[ 110 ] = sin ( xx [ 104 ] ) ; xx [ 104 ] = xx [ 6 ] * xx [ 105 ] * xx [ 110
] ; xx [ 111 ] = xx [ 103 ] * xx [ 104 ] ; xx [ 112 ] = xx [ 111 ] * xx [ 104
] ; xx [ 113 ] = xx [ 109 ] + xx [ 112 ] ; xx [ 114 ] = xx [ 102 ] + xx [ 113
] ; xx [ 115 ] = xx [ 111 ] * xx [ 107 ] ; xx [ 111 ] = xx [ 108 ] * xx [ 104
] ; xx [ 108 ] = xx [ 115 ] - xx [ 111 ] ; xx [ 116 ] = xx [ 55 ] * xx [ 108
] ; xx [ 117 ] = xx [ 116 ] + xx [ 116 ] ; xx [ 118 ] = 2.42575e-3 ; xx [ 119
] = 4.363639999999999e-3 ; memcpy ( xx + 120 , xx + 119 , 1 * sizeof ( double
) ) ; ii [ 1 ] = factorSymmetricPosDef ( xx + 120 , 1 , xx + 121 ) ; if ( ii
[ 1 ] != 0 ) { return sm_ssci_recordRunTimeError (
"sm:compiler:messages:simulationErrors:DegenerateMass" ,
 "'wheel_leg/blance_leg_model/Revolute Joint8' has a degenerate mass distribution on its follower side."
, neDiagMgr ) ; } memcpy ( xx + 121 , xx + 120 , 1 * sizeof ( double ) ) ; xx
[ 122 ] = xx [ 119 ] / xx [ 121 ] ; xx [ 123 ] = ( xx [ 106 ] + xx [ 110 ] *
xx [ 110 ] ) * xx [ 6 ] - xx [ 7 ] ; xx [ 106 ] = xx [ 112 ] + xx [ 109 ] ;
xx [ 109 ] = xx [ 106 ] * xx [ 55 ] ; xx [ 112 ] = xx [ 55 ] * xx [ 109 ] ;
xx [ 124 ] = xx [ 118 ] + ( xx [ 119 ] - xx [ 119 ] * xx [ 122 ] ) * xx [ 123
] * xx [ 123 ] + xx [ 112 ] ; xx [ 125 ] = xx [ 124 ] + xx [ 112 ] ; xx [ 112
] = xx [ 102 ] + xx [ 106 ] ; xx [ 126 ] = xx [ 109 ] + xx [ 112 ] * xx [ 55
] ; xx [ 127 ] = xx [ 125 ] + xx [ 126 ] * xx [ 55 ] ; ii [ 2 ] =
factorSymmetricPosDef ( xx + 127 , 1 , xx + 128 ) ; if ( ii [ 2 ] != 0 ) {
return sm_ssci_recordRunTimeError (
"sm:compiler:messages:simulationErrors:DegenerateMass" ,
 "'wheel_leg/blance_leg_model/Revolute Joint6' has a degenerate mass distribution on its follower side."
, neDiagMgr ) ; } xx [ 128 ] = xx [ 117 ] / xx [ 127 ] ; xx [ 129 ] = xx [
114 ] - xx [ 117 ] * xx [ 128 ] ; xx [ 130 ] = sin ( xx [ 71 ] ) ; xx [ 71 ]
= xx [ 81 ] * xx [ 130 ] ; xx [ 131 ] = xx [ 6 ] * xx [ 71 ] ; xx [ 132 ] =
xx [ 126 ] * xx [ 128 ] ; xx [ 133 ] = xx [ 108 ] - xx [ 132 ] ; xx [ 134 ] =
xx [ 129 ] * xx [ 101 ] - xx [ 131 ] * xx [ 133 ] ; xx [ 135 ] = xx [ 111 ] -
xx [ 115 ] ; xx [ 111 ] = xx [ 135 ] - xx [ 132 ] ; xx [ 115 ] = xx [ 126 ] /
xx [ 127 ] ; xx [ 132 ] = xx [ 112 ] - xx [ 126 ] * xx [ 115 ] ; xx [ 136 ] =
xx [ 101 ] * xx [ 111 ] - xx [ 131 ] * xx [ 132 ] ; xx [ 137 ] = xx [ 101 ] *
xx [ 134 ] - xx [ 131 ] * xx [ 136 ] ; xx [ 138 ] = xx [ 8 ] + xx [ 137 ] ;
xx [ 139 ] = ( xx [ 82 ] + xx [ 130 ] * xx [ 130 ] ) * xx [ 6 ] - xx [ 7 ] ;
xx [ 82 ] = xx [ 116 ] - xx [ 125 ] * xx [ 128 ] ; xx [ 140 ] = xx [ 109 ] -
xx [ 125 ] * xx [ 115 ] ; xx [ 141 ] = xx [ 139 ] * ( xx [ 82 ] * xx [ 101 ]
- xx [ 131 ] * xx [ 140 ] ) ; xx [ 142 ] = xx [ 55 ] * xx [ 130 ] ; xx [ 143
] = xx [ 6 ] * xx [ 142 ] * xx [ 130 ] ; xx [ 144 ] = xx [ 54 ] - xx [ 143 ]
; xx [ 145 ] = xx [ 131 ] * xx [ 129 ] + xx [ 101 ] * xx [ 133 ] ; xx [ 129 ]
= xx [ 131 ] * xx [ 111 ] + xx [ 132 ] * xx [ 101 ] ; xx [ 111 ] = xx [ 145 ]
* xx [ 101 ] - xx [ 129 ] * xx [ 131 ] ; xx [ 132 ] = xx [ 81 ] * xx [ 142 ]
; xx [ 133 ] = xx [ 6 ] * xx [ 132 ] ; xx [ 142 ] = xx [ 144 ] * xx [ 111 ] -
xx [ 133 ] * xx [ 137 ] ; xx [ 137 ] = xx [ 141 ] + xx [ 142 ] ; xx [ 146 ] =
xx [ 137 ] + xx [ 63 ] * xx [ 111 ] ; xx [ 147 ] = xx [ 125 ] / xx [ 127 ] ;
xx [ 148 ] = ( xx [ 131 ] * xx [ 82 ] + xx [ 140 ] * xx [ 101 ] ) * xx [ 139
] ; xx [ 82 ] = xx [ 144 ] * xx [ 148 ] - xx [ 133 ] * xx [ 141 ] ; xx [ 140
] = xx [ 145 ] * xx [ 131 ] + xx [ 129 ] * xx [ 101 ] ; xx [ 129 ] = xx [ 131
] * xx [ 134 ] + xx [ 101 ] * xx [ 136 ] ; xx [ 134 ] = xx [ 140 ] * xx [ 144
] - xx [ 129 ] * xx [ 133 ] ; xx [ 136 ] = xx [ 65 ] + ( xx [ 124 ] - xx [
125 ] * xx [ 147 ] ) * xx [ 139 ] * xx [ 139 ] + xx [ 82 ] + xx [ 82 ] + xx [
134 ] * xx [ 144 ] - xx [ 142 ] * xx [ 133 ] ; xx [ 82 ] = xx [ 148 ] + xx [
134 ] ; xx [ 124 ] = xx [ 136 ] + xx [ 82 ] * xx [ 63 ] ; xx [ 134 ] = xx [ 8
] + xx [ 140 ] ; xx [ 140 ] = xx [ 82 ] + xx [ 134 ] * xx [ 63 ] ; xx [ 141 ]
= xx [ 124 ] + xx [ 140 ] * xx [ 63 ] ; ii [ 2 ] = factorSymmetricPosDef ( xx
+ 141 , 1 , xx + 142 ) ; if ( ii [ 2 ] != 0 ) { return
sm_ssci_recordRunTimeError (
"sm:compiler:messages:simulationErrors:DegenerateMass" ,
 "'wheel_leg/blance_leg_model/right joint1' has a degenerate mass distribution on its follower side."
, neDiagMgr ) ; } xx [ 142 ] = xx [ 146 ] / xx [ 141 ] ; xx [ 145 ] = xx [
138 ] - xx [ 146 ] * xx [ 142 ] ; xx [ 148 ] = xx [ 140 ] * xx [ 142 ] ; xx [
149 ] = xx [ 111 ] - xx [ 148 ] ; xx [ 150 ] = xx [ 129 ] - xx [ 148 ] ; xx [
148 ] = xx [ 140 ] / xx [ 141 ] ; xx [ 151 ] = xx [ 134 ] - xx [ 140 ] * xx [
148 ] ; xx [ 152 ] = xx [ 103 ] * xx [ 123 ] * xx [ 123 ] ; xx [ 123 ] = xx [
102 ] + xx [ 152 ] ; xx [ 153 ] = xx [ 123 ] * xx [ 139 ] * xx [ 139 ] ; xx [
154 ] = xx [ 8 ] + xx [ 153 ] ; xx [ 155 ] = xx [ 145 ] * xx [ 74 ] + xx [ 76
] * xx [ 149 ] ; xx [ 156 ] = xx [ 75 ] * xx [ 145 ] + xx [ 79 ] * xx [ 149 ]
; xx [ 157 ] = xx [ 68 ] * xx [ 145 ] + xx [ 75 ] * xx [ 149 ] ; xx [ 158 ] =
xx [ 150 ] * xx [ 74 ] + xx [ 76 ] * xx [ 151 ] ; xx [ 159 ] = xx [ 75 ] * xx
[ 150 ] + xx [ 151 ] * xx [ 79 ] ; xx [ 160 ] = xx [ 68 ] * xx [ 150 ] + xx [
75 ] * xx [ 151 ] ; xx [ 161 ] = xx [ 154 ] * xx [ 77 ] ; xx [ 162 ] = xx [
154 ] * xx [ 76 ] ; xx [ 163 ] = xx [ 154 ] * xx [ 74 ] ;
pm_math_Matrix3x3_compose_ra ( xx + 92 , xx + 155 , xx + 164 ) ; xx [ 145 ] =
xx [ 1 ] * state [ 14 ] ; xx [ 149 ] = xx [ 0 ] * cos ( xx [ 145 ] ) ; xx [
150 ] = xx [ 149 ] * xx [ 149 ] ; xx [ 151 ] = xx [ 0 ] * sin ( xx [ 145 ] )
; xx [ 145 ] = xx [ 151 ] * xx [ 151 ] ; xx [ 155 ] = ( xx [ 150 ] + xx [ 145
] ) * xx [ 6 ] - xx [ 7 ] ; xx [ 156 ] = xx [ 149 ] * xx [ 151 ] ; xx [ 157 ]
= xx [ 6 ] * ( xx [ 156 ] - xx [ 156 ] ) ; xx [ 158 ] = ( xx [ 145 ] + xx [
150 ] ) * xx [ 6 ] ; xx [ 159 ] = xx [ 156 ] + xx [ 156 ] ; xx [ 156 ] = xx [
159 ] * xx [ 6 ] ; xx [ 160 ] = ( xx [ 150 ] + xx [ 150 ] ) * xx [ 6 ] - xx [
7 ] ; xx [ 161 ] = xx [ 145 ] - xx [ 150 ] ; xx [ 145 ] = xx [ 6 ] * xx [ 161
] ; xx [ 173 ] = xx [ 155 ] ; xx [ 174 ] = xx [ 157 ] ; xx [ 175 ] = xx [ 158
] ; xx [ 176 ] = xx [ 156 ] ; xx [ 177 ] = xx [ 160 ] ; xx [ 178 ] = xx [ 157
] ; xx [ 179 ] = xx [ 145 ] ; xx [ 180 ] = xx [ 156 ] ; xx [ 181 ] = xx [ 155
] ; xx [ 150 ] = xx [ 1 ] * state [ 16 ] ; xx [ 162 ] = cos ( xx [ 150 ] ) ;
xx [ 163 ] = xx [ 162 ] * xx [ 162 ] ; xx [ 182 ] = xx [ 6 ] * xx [ 163 ] -
xx [ 7 ] ; xx [ 183 ] = xx [ 35 ] * xx [ 182 ] ; xx [ 184 ] = sin ( xx [ 150
] ) ; xx [ 150 ] = xx [ 162 ] * xx [ 184 ] ; xx [ 185 ] = xx [ 6 ] * xx [ 150
] ; if ( ii [ 0 ] != 0 ) { return sm_ssci_recordRunTimeError (
"sm:compiler:messages:simulationErrors:DegenerateMass" ,
 "'wheel_leg/blance_leg_model/Revolute Joint2' has a degenerate mass distribution on its follower side."
, neDiagMgr ) ; } xx [ 186 ] = xx [ 43 ] / xx [ 44 ] ; xx [ 187 ] = xx [ 35 ]
- xx [ 43 ] * xx [ 186 ] ; xx [ 188 ] = xx [ 185 ] * xx [ 187 ] ; xx [ 189 ]
= xx [ 183 ] * xx [ 182 ] + xx [ 185 ] * xx [ 188 ] ; xx [ 190 ] = xx [ 8 ] +
xx [ 189 ] ; xx [ 191 ] = xx [ 51 ] * xx [ 186 ] ; xx [ 192 ] = ( xx [ 163 ]
+ xx [ 184 ] * xx [ 184 ] ) * xx [ 6 ] - xx [ 7 ] ; xx [ 163 ] = xx [ 191 ] *
xx [ 185 ] * xx [ 192 ] ; xx [ 193 ] = xx [ 55 ] * xx [ 184 ] ; xx [ 194 ] =
xx [ 6 ] * xx [ 193 ] * xx [ 184 ] ; xx [ 195 ] = xx [ 54 ] - xx [ 194 ] ; xx
[ 196 ] = xx [ 35 ] * xx [ 185 ] ; xx [ 197 ] = xx [ 187 ] * xx [ 182 ] ; xx
[ 187 ] = xx [ 196 ] * xx [ 182 ] - xx [ 185 ] * xx [ 197 ] ; xx [ 198 ] = xx
[ 162 ] * xx [ 193 ] ; xx [ 193 ] = xx [ 6 ] * xx [ 198 ] ; xx [ 199 ] = xx [
195 ] * xx [ 187 ] - xx [ 189 ] * xx [ 193 ] ; xx [ 189 ] = xx [ 163 ] + xx [
199 ] ; xx [ 200 ] = xx [ 189 ] + xx [ 63 ] * xx [ 187 ] ; xx [ 201 ] = xx [
51 ] / xx [ 44 ] ; xx [ 202 ] = xx [ 191 ] * xx [ 182 ] * xx [ 192 ] ; xx [
191 ] = xx [ 195 ] * xx [ 202 ] + xx [ 193 ] * xx [ 163 ] ; xx [ 163 ] = xx [
196 ] * xx [ 185 ] + xx [ 197 ] * xx [ 182 ] ; xx [ 196 ] = xx [ 183 ] * xx [
185 ] - xx [ 188 ] * xx [ 182 ] ; xx [ 183 ] = xx [ 163 ] * xx [ 195 ] - xx [
193 ] * xx [ 196 ] ; xx [ 188 ] = xx [ 65 ] + ( xx [ 51 ] - xx [ 51 ] * xx [
201 ] ) * xx [ 192 ] * xx [ 192 ] - xx [ 191 ] - xx [ 191 ] + xx [ 195 ] * xx
[ 183 ] - xx [ 199 ] * xx [ 193 ] ; xx [ 191 ] = xx [ 183 ] - xx [ 202 ] ; xx
[ 183 ] = xx [ 188 ] + xx [ 63 ] * xx [ 191 ] ; xx [ 197 ] = xx [ 8 ] + xx [
163 ] ; xx [ 163 ] = xx [ 191 ] + xx [ 197 ] * xx [ 63 ] ; xx [ 199 ] = xx [
183 ] + xx [ 163 ] * xx [ 63 ] ; ii [ 0 ] = factorSymmetricPosDef ( xx + 199
, 1 , xx + 202 ) ; if ( ii [ 0 ] != 0 ) { return sm_ssci_recordRunTimeError (
"sm:compiler:messages:simulationErrors:DegenerateMass" ,
 "'wheel_leg/blance_leg_model/left joint 4 ' has a degenerate mass distribution on its follower side."
, neDiagMgr ) ; } xx [ 202 ] = xx [ 200 ] / xx [ 199 ] ; xx [ 203 ] = xx [
190 ] - xx [ 200 ] * xx [ 202 ] ; xx [ 204 ] = xx [ 163 ] * xx [ 202 ] ; xx [
205 ] = xx [ 187 ] - xx [ 204 ] ; xx [ 206 ] = xx [ 196 ] - xx [ 204 ] ; xx [
204 ] = xx [ 163 ] / xx [ 199 ] ; xx [ 207 ] = xx [ 197 ] - xx [ 163 ] * xx [
204 ] ; xx [ 208 ] = xx [ 35 ] * xx [ 192 ] * xx [ 192 ] ; xx [ 192 ] = xx [
8 ] + xx [ 208 ] ; xx [ 209 ] = xx [ 203 ] * xx [ 155 ] + xx [ 157 ] * xx [
205 ] ; xx [ 210 ] = xx [ 156 ] * xx [ 203 ] + xx [ 160 ] * xx [ 205 ] ; xx [
211 ] = xx [ 145 ] * xx [ 203 ] + xx [ 156 ] * xx [ 205 ] ; xx [ 212 ] = xx [
155 ] * xx [ 206 ] + xx [ 157 ] * xx [ 207 ] ; xx [ 213 ] = xx [ 156 ] * xx [
206 ] + xx [ 207 ] * xx [ 160 ] ; xx [ 214 ] = xx [ 145 ] * xx [ 206 ] + xx [
156 ] * xx [ 207 ] ; xx [ 215 ] = xx [ 192 ] * xx [ 158 ] ; xx [ 216 ] = xx [
192 ] * xx [ 157 ] ; xx [ 217 ] = xx [ 192 ] * xx [ 155 ] ;
pm_math_Matrix3x3_compose_ra ( xx + 173 , xx + 209 , xx + 218 ) ; xx [ 203 ]
= xx [ 1 ] * state [ 8 ] ; xx [ 205 ] = xx [ 0 ] * cos ( xx [ 203 ] ) ; xx [
206 ] = xx [ 205 ] * xx [ 205 ] ; xx [ 207 ] = xx [ 0 ] * sin ( xx [ 203 ] )
; xx [ 203 ] = xx [ 207 ] * xx [ 207 ] ; xx [ 209 ] = ( xx [ 206 ] + xx [ 203
] ) * xx [ 6 ] - xx [ 7 ] ; xx [ 210 ] = xx [ 205 ] * xx [ 207 ] ; xx [ 211 ]
= xx [ 6 ] * ( xx [ 210 ] - xx [ 210 ] ) ; xx [ 212 ] = ( xx [ 203 ] + xx [
206 ] ) * xx [ 6 ] ; xx [ 213 ] = xx [ 210 ] + xx [ 210 ] ; xx [ 210 ] = xx [
213 ] * xx [ 6 ] ; xx [ 214 ] = ( xx [ 206 ] + xx [ 206 ] ) * xx [ 6 ] - xx [
7 ] ; xx [ 215 ] = xx [ 203 ] - xx [ 206 ] ; xx [ 203 ] = xx [ 6 ] * xx [ 215
] ; xx [ 227 ] = xx [ 209 ] ; xx [ 228 ] = xx [ 211 ] ; xx [ 229 ] = xx [ 212
] ; xx [ 230 ] = xx [ 210 ] ; xx [ 231 ] = xx [ 214 ] ; xx [ 232 ] = xx [ 211
] ; xx [ 233 ] = xx [ 203 ] ; xx [ 234 ] = xx [ 210 ] ; xx [ 235 ] = xx [ 209
] ; xx [ 206 ] = xx [ 1 ] * state [ 10 ] ; xx [ 216 ] = cos ( xx [ 206 ] ) ;
xx [ 217 ] = xx [ 216 ] * xx [ 216 ] ; xx [ 236 ] = xx [ 6 ] * xx [ 217 ] -
xx [ 7 ] ; xx [ 237 ] = xx [ 1 ] * state [ 12 ] ; xx [ 238 ] = cos ( xx [ 237
] ) ; xx [ 239 ] = xx [ 238 ] * xx [ 238 ] ; xx [ 240 ] = xx [ 6 ] * xx [ 239
] - xx [ 7 ] ; xx [ 241 ] = xx [ 103 ] * xx [ 240 ] ; xx [ 242 ] = xx [ 241 ]
* xx [ 240 ] ; xx [ 243 ] = sin ( xx [ 237 ] ) ; xx [ 237 ] = xx [ 6 ] * xx [
238 ] * xx [ 243 ] ; xx [ 244 ] = xx [ 103 ] * xx [ 237 ] ; xx [ 245 ] = xx [
244 ] * xx [ 237 ] ; xx [ 246 ] = xx [ 242 ] + xx [ 245 ] ; xx [ 247 ] = xx [
102 ] + xx [ 246 ] ; xx [ 248 ] = xx [ 244 ] * xx [ 240 ] ; xx [ 244 ] = xx [
241 ] * xx [ 237 ] ; xx [ 241 ] = xx [ 248 ] - xx [ 244 ] ; xx [ 249 ] = xx [
55 ] * xx [ 241 ] ; xx [ 250 ] = xx [ 249 ] + xx [ 249 ] ; if ( ii [ 1 ] != 0
) { return sm_ssci_recordRunTimeError (
"sm:compiler:messages:simulationErrors:DegenerateMass" ,
 "'wheel_leg/blance_leg_model/Revolute Joint5' has a degenerate mass distribution on its follower side."
, neDiagMgr ) ; } xx [ 251 ] = xx [ 119 ] / xx [ 120 ] ; xx [ 252 ] = ( xx [
239 ] + xx [ 243 ] * xx [ 243 ] ) * xx [ 6 ] - xx [ 7 ] ; xx [ 239 ] = xx [
245 ] + xx [ 242 ] ; xx [ 242 ] = xx [ 239 ] * xx [ 55 ] ; xx [ 245 ] = xx [
55 ] * xx [ 242 ] ; xx [ 253 ] = xx [ 118 ] + ( xx [ 119 ] - xx [ 119 ] * xx
[ 251 ] ) * xx [ 252 ] * xx [ 252 ] + xx [ 245 ] ; xx [ 254 ] = xx [ 253 ] +
xx [ 245 ] ; xx [ 245 ] = xx [ 102 ] + xx [ 239 ] ; xx [ 255 ] = xx [ 242 ] +
xx [ 245 ] * xx [ 55 ] ; xx [ 256 ] = xx [ 254 ] + xx [ 255 ] * xx [ 55 ] ;
ii [ 0 ] = factorSymmetricPosDef ( xx + 256 , 1 , xx + 257 ) ; if ( ii [ 0 ]
!= 0 ) { return sm_ssci_recordRunTimeError (
"sm:compiler:messages:simulationErrors:DegenerateMass" ,
 "'wheel_leg/blance_leg_model/Revolute Joint3' has a degenerate mass distribution on its follower side."
, neDiagMgr ) ; } xx [ 257 ] = xx [ 250 ] / xx [ 256 ] ; xx [ 258 ] = xx [
247 ] - xx [ 250 ] * xx [ 257 ] ; xx [ 259 ] = sin ( xx [ 206 ] ) ; xx [ 206
] = xx [ 216 ] * xx [ 259 ] ; xx [ 260 ] = xx [ 6 ] * xx [ 206 ] ; xx [ 261 ]
= xx [ 255 ] * xx [ 257 ] ; xx [ 262 ] = xx [ 241 ] - xx [ 261 ] ; xx [ 263 ]
= xx [ 258 ] * xx [ 236 ] - xx [ 260 ] * xx [ 262 ] ; xx [ 264 ] = xx [ 244 ]
- xx [ 248 ] ; xx [ 244 ] = xx [ 264 ] - xx [ 261 ] ; xx [ 248 ] = xx [ 255 ]
/ xx [ 256 ] ; xx [ 261 ] = xx [ 245 ] - xx [ 255 ] * xx [ 248 ] ; xx [ 265 ]
= xx [ 236 ] * xx [ 244 ] - xx [ 260 ] * xx [ 261 ] ; xx [ 266 ] = xx [ 236 ]
* xx [ 263 ] - xx [ 260 ] * xx [ 265 ] ; xx [ 267 ] = xx [ 8 ] + xx [ 266 ] ;
xx [ 268 ] = ( xx [ 217 ] + xx [ 259 ] * xx [ 259 ] ) * xx [ 6 ] - xx [ 7 ] ;
xx [ 217 ] = xx [ 249 ] - xx [ 254 ] * xx [ 257 ] ; xx [ 269 ] = xx [ 242 ] -
xx [ 254 ] * xx [ 248 ] ; xx [ 270 ] = xx [ 268 ] * ( xx [ 217 ] * xx [ 236 ]
- xx [ 260 ] * xx [ 269 ] ) ; xx [ 271 ] = xx [ 55 ] * xx [ 259 ] ; xx [ 272
] = xx [ 6 ] * xx [ 271 ] * xx [ 259 ] ; xx [ 273 ] = xx [ 54 ] - xx [ 272 ]
; xx [ 274 ] = xx [ 260 ] * xx [ 258 ] + xx [ 236 ] * xx [ 262 ] ; xx [ 258 ]
= xx [ 260 ] * xx [ 244 ] + xx [ 261 ] * xx [ 236 ] ; xx [ 244 ] = xx [ 274 ]
* xx [ 236 ] - xx [ 258 ] * xx [ 260 ] ; xx [ 261 ] = xx [ 216 ] * xx [ 271 ]
; xx [ 262 ] = xx [ 6 ] * xx [ 261 ] ; xx [ 271 ] = xx [ 273 ] * xx [ 244 ] -
xx [ 262 ] * xx [ 266 ] ; xx [ 266 ] = xx [ 270 ] + xx [ 271 ] ; xx [ 275 ] =
xx [ 266 ] + xx [ 63 ] * xx [ 244 ] ; xx [ 276 ] = xx [ 254 ] / xx [ 256 ] ;
xx [ 277 ] = ( xx [ 260 ] * xx [ 217 ] + xx [ 269 ] * xx [ 236 ] ) * xx [ 268
] ; xx [ 217 ] = xx [ 273 ] * xx [ 277 ] - xx [ 262 ] * xx [ 270 ] ; xx [ 269
] = xx [ 274 ] * xx [ 260 ] + xx [ 258 ] * xx [ 236 ] ; xx [ 258 ] = xx [ 260
] * xx [ 263 ] + xx [ 236 ] * xx [ 265 ] ; xx [ 263 ] = xx [ 269 ] * xx [ 273
] - xx [ 258 ] * xx [ 262 ] ; xx [ 265 ] = xx [ 65 ] + ( xx [ 253 ] - xx [
254 ] * xx [ 276 ] ) * xx [ 268 ] * xx [ 268 ] + xx [ 217 ] + xx [ 217 ] + xx
[ 263 ] * xx [ 273 ] - xx [ 271 ] * xx [ 262 ] ; xx [ 217 ] = xx [ 277 ] + xx
[ 263 ] ; xx [ 253 ] = xx [ 265 ] + xx [ 217 ] * xx [ 63 ] ; xx [ 263 ] = xx
[ 8 ] + xx [ 269 ] ; xx [ 269 ] = xx [ 217 ] + xx [ 263 ] * xx [ 63 ] ; xx [
270 ] = xx [ 253 ] + xx [ 269 ] * xx [ 63 ] ; ii [ 0 ] =
factorSymmetricPosDef ( xx + 270 , 1 , xx + 271 ) ; if ( ii [ 0 ] != 0 ) {
return sm_ssci_recordRunTimeError (
"sm:compiler:messages:simulationErrors:DegenerateMass" ,
 "'wheel_leg/blance_leg_model/left joint 1' has a degenerate mass distribution on its follower side."
, neDiagMgr ) ; } xx [ 271 ] = xx [ 275 ] / xx [ 270 ] ; xx [ 274 ] = xx [
267 ] - xx [ 275 ] * xx [ 271 ] ; xx [ 277 ] = xx [ 269 ] * xx [ 271 ] ; xx [
278 ] = xx [ 244 ] - xx [ 277 ] ; xx [ 279 ] = xx [ 258 ] - xx [ 277 ] ; xx [
277 ] = xx [ 269 ] / xx [ 270 ] ; xx [ 280 ] = xx [ 263 ] - xx [ 269 ] * xx [
277 ] ; xx [ 281 ] = xx [ 103 ] * xx [ 252 ] * xx [ 252 ] ; xx [ 252 ] = xx [
102 ] + xx [ 281 ] ; xx [ 102 ] = xx [ 252 ] * xx [ 268 ] * xx [ 268 ] ; xx [
282 ] = xx [ 8 ] + xx [ 102 ] ; xx [ 283 ] = xx [ 274 ] * xx [ 209 ] + xx [
211 ] * xx [ 278 ] ; xx [ 284 ] = xx [ 210 ] * xx [ 274 ] + xx [ 214 ] * xx [
278 ] ; xx [ 285 ] = xx [ 203 ] * xx [ 274 ] + xx [ 210 ] * xx [ 278 ] ; xx [
286 ] = xx [ 279 ] * xx [ 209 ] + xx [ 211 ] * xx [ 280 ] ; xx [ 287 ] = xx [
210 ] * xx [ 279 ] + xx [ 280 ] * xx [ 214 ] ; xx [ 288 ] = xx [ 203 ] * xx [
279 ] + xx [ 210 ] * xx [ 280 ] ; xx [ 289 ] = xx [ 282 ] * xx [ 212 ] ; xx [
290 ] = xx [ 282 ] * xx [ 211 ] ; xx [ 291 ] = xx [ 282 ] * xx [ 209 ] ;
pm_math_Matrix3x3_compose_ra ( xx + 227 , xx + 283 , xx + 292 ) ; xx [ 8 ] =
xx [ 56 ] * xx [ 73 ] ; xx [ 274 ] = xx [ 58 ] * xx [ 73 ] ; xx [ 73 ] = xx [
49 ] - xx [ 40 ] * xx [ 67 ] ; xx [ 278 ] = xx [ 52 ] - xx [ 40 ] * xx [ 69 ]
; xx [ 283 ] = xx [ 22 ] * xx [ 8 ] ; xx [ 284 ] = xx [ 21 ] * xx [ 8 ] ; xx
[ 285 ] = xx [ 8 ] * xx [ 19 ] ; xx [ 286 ] = - ( xx [ 22 ] * xx [ 274 ] ) ;
xx [ 287 ] = - ( xx [ 274 ] * xx [ 21 ] ) ; xx [ 288 ] = - ( xx [ 274 ] * xx
[ 19 ] ) ; xx [ 289 ] = xx [ 73 ] * xx [ 19 ] + xx [ 21 ] * xx [ 278 ] ; xx [
290 ] = xx [ 20 ] * xx [ 73 ] + xx [ 24 ] * xx [ 278 ] ; xx [ 291 ] = xx [ 2
] * xx [ 73 ] + xx [ 20 ] * xx [ 278 ] ; pm_math_Matrix3x3_compose_ra ( xx +
26 , xx + 283 , xx + 301 ) ; xx [ 73 ] = 0.195 ; xx [ 278 ] = xx [ 63 ] * xx
[ 4 ] ; xx [ 279 ] = xx [ 278 ] * xx [ 4 ] ; xx [ 280 ] = xx [ 63 ] * xx [ 9
] ; xx [ 283 ] = xx [ 280 ] * xx [ 9 ] ; xx [ 284 ] = xx [ 280 ] * xx [ 4 ] ;
xx [ 280 ] = ( xx [ 284 ] + xx [ 278 ] * xx [ 9 ] ) * xx [ 6 ] ; xx [ 278 ] =
0.05400000000000001 ; xx [ 285 ] = xx [ 280 ] - xx [ 278 ] ; xx [ 286 ] =
0.03 ; xx [ 287 ] = xx [ 6 ] * ( xx [ 279 ] - xx [ 283 ] ) ; xx [ 288 ] = xx
[ 286 ] - xx [ 287 ] ; xx [ 289 ] = xx [ 73 ] - ( ( xx [ 279 ] + xx [ 283 ] )
* xx [ 6 ] - xx [ 63 ] ) ; xx [ 290 ] = xx [ 285 ] ; xx [ 291 ] = xx [ 288 ]
; pm_math_Matrix3x3_postCross_ra ( xx + 83 , xx + 289 , xx + 310 ) ; xx [ 279
] = 0.035 ; xx [ 319 ] = xx [ 279 ] * xx [ 109 ] ; xx [ 320 ] = xx [ 106 ] *
xx [ 279 ] ; xx [ 106 ] = xx [ 55 ] * xx [ 320 ] ; xx [ 321 ] = xx [ 319 ] +
xx [ 106 ] ; xx [ 322 ] = xx [ 279 ] * xx [ 108 ] ; xx [ 323 ] = xx [ 321 ] *
xx [ 128 ] - xx [ 322 ] ; xx [ 324 ] = xx [ 321 ] * xx [ 115 ] - xx [ 320 ] ;
xx [ 325 ] = xx [ 323 ] * xx [ 101 ] - xx [ 131 ] * xx [ 324 ] ; xx [ 326 ] =
xx [ 113 ] * xx [ 279 ] ; xx [ 113 ] = xx [ 279 ] * xx [ 116 ] ; xx [ 327 ] =
xx [ 279 ] * xx [ 135 ] ; xx [ 328 ] = xx [ 55 ] * xx [ 327 ] ; xx [ 329 ] =
xx [ 113 ] + xx [ 328 ] ; xx [ 330 ] = xx [ 326 ] - xx [ 329 ] * xx [ 128 ] ;
xx [ 331 ] = xx [ 327 ] - xx [ 329 ] * xx [ 115 ] ; xx [ 332 ] = xx [ 330 ] *
xx [ 101 ] - xx [ 131 ] * xx [ 331 ] ; xx [ 333 ] = xx [ 101 ] * xx [ 325 ] -
xx [ 131 ] * xx [ 332 ] ; xx [ 334 ] = xx [ 321 ] / xx [ 127 ] ; xx [ 335 ] =
xx [ 125 ] * xx [ 334 ] ; xx [ 336 ] = ( xx [ 335 ] - xx [ 319 ] ) * xx [ 139
] ; xx [ 319 ] = xx [ 329 ] / xx [ 127 ] ; xx [ 337 ] = xx [ 125 ] * xx [ 319
] ; xx [ 338 ] = xx [ 139 ] * ( xx [ 113 ] - xx [ 337 ] ) ; xx [ 113 ] = xx [
131 ] * xx [ 323 ] + xx [ 324 ] * xx [ 101 ] ; xx [ 323 ] = xx [ 131 ] * xx [
330 ] + xx [ 331 ] * xx [ 101 ] ; xx [ 324 ] = xx [ 113 ] * xx [ 101 ] - xx [
323 ] * xx [ 131 ] ; xx [ 330 ] = xx [ 144 ] * xx [ 324 ] - xx [ 133 ] * xx [
333 ] ; xx [ 331 ] = xx [ 336 ] * xx [ 101 ] - xx [ 131 ] * xx [ 338 ] + xx [
330 ] ; xx [ 339 ] = xx [ 331 ] + xx [ 63 ] * xx [ 324 ] ; xx [ 340 ] = xx [
333 ] - xx [ 339 ] * xx [ 142 ] ; xx [ 341 ] = xx [ 324 ] - xx [ 339 ] * xx [
148 ] ; xx [ 342 ] = xx [ 55 ] * xx [ 152 ] ; xx [ 152 ] = xx [ 342 ] * xx [
139 ] ; xx [ 343 ] = xx [ 131 ] * xx [ 152 ] ; xx [ 344 ] = xx [ 133 ] * xx [
153 ] ; xx [ 345 ] = xx [ 343 ] + xx [ 344 ] ; xx [ 346 ] = xx [ 131 ] * xx [
325 ] + xx [ 101 ] * xx [ 332 ] ; xx [ 325 ] = xx [ 113 ] * xx [ 131 ] + xx [
323 ] * xx [ 101 ] ; xx [ 113 ] = xx [ 325 ] * xx [ 144 ] - xx [ 346 ] * xx [
133 ] ; xx [ 323 ] = xx [ 131 ] * xx [ 336 ] + xx [ 338 ] * xx [ 101 ] + xx [
113 ] ; xx [ 332 ] = xx [ 323 ] + xx [ 325 ] * xx [ 63 ] ; xx [ 336 ] = xx [
346 ] - xx [ 332 ] * xx [ 142 ] ; xx [ 338 ] = xx [ 325 ] - xx [ 332 ] * xx [
148 ] ; xx [ 347 ] = xx [ 152 ] * xx [ 101 ] ; xx [ 152 ] = xx [ 144 ] * xx [
153 ] ; xx [ 153 ] = xx [ 347 ] + xx [ 152 ] ; xx [ 348 ] = xx [ 137 ] - xx [
124 ] * xx [ 142 ] ; xx [ 349 ] = xx [ 82 ] - xx [ 124 ] * xx [ 148 ] ; xx [
350 ] = xx [ 74 ] * xx [ 340 ] + xx [ 76 ] * xx [ 341 ] + xx [ 77 ] * xx [
345 ] ; xx [ 351 ] = xx [ 75 ] * xx [ 340 ] + xx [ 79 ] * xx [ 341 ] + xx [
76 ] * xx [ 345 ] ; xx [ 352 ] = xx [ 68 ] * xx [ 340 ] + xx [ 75 ] * xx [
341 ] + xx [ 74 ] * xx [ 345 ] ; xx [ 353 ] = xx [ 336 ] * xx [ 74 ] + xx [
76 ] * xx [ 338 ] - xx [ 153 ] * xx [ 77 ] ; xx [ 354 ] = xx [ 75 ] * xx [
336 ] + xx [ 338 ] * xx [ 79 ] - xx [ 153 ] * xx [ 76 ] ; xx [ 355 ] = xx [
68 ] * xx [ 336 ] + xx [ 75 ] * xx [ 338 ] - xx [ 153 ] * xx [ 74 ] ; xx [
356 ] = xx [ 348 ] * xx [ 74 ] + xx [ 76 ] * xx [ 349 ] ; xx [ 357 ] = xx [
75 ] * xx [ 348 ] + xx [ 349 ] * xx [ 79 ] ; xx [ 358 ] = xx [ 68 ] * xx [
348 ] + xx [ 75 ] * xx [ 349 ] ; pm_math_Matrix3x3_compose_ra ( xx + 92 , xx
+ 350 , xx + 359 ) ; xx [ 74 ] = xx [ 63 ] * xx [ 70 ] ; xx [ 76 ] = xx [ 74
] * xx [ 70 ] ; xx [ 77 ] = xx [ 63 ] * xx [ 72 ] ; xx [ 336 ] = xx [ 77 ] *
xx [ 72 ] ; xx [ 338 ] = xx [ 77 ] * xx [ 70 ] ; xx [ 77 ] = ( xx [ 338 ] +
xx [ 74 ] * xx [ 72 ] ) * xx [ 6 ] ; xx [ 74 ] = xx [ 278 ] + xx [ 77 ] ; xx
[ 340 ] = xx [ 6 ] * ( xx [ 76 ] - xx [ 336 ] ) ; xx [ 341 ] = xx [ 286 ] -
xx [ 340 ] ; xx [ 348 ] = xx [ 73 ] - ( ( xx [ 76 ] + xx [ 336 ] ) * xx [ 6 ]
- xx [ 63 ] ) ; xx [ 349 ] = xx [ 74 ] ; xx [ 350 ] = xx [ 341 ] ;
pm_math_Matrix3x3_postCross_ra ( xx + 164 , xx + 348 , xx + 368 ) ; xx [ 76 ]
= xx [ 193 ] * xx [ 208 ] ; xx [ 351 ] = xx [ 195 ] * xx [ 208 ] ; xx [ 208 ]
= xx [ 189 ] - xx [ 183 ] * xx [ 202 ] ; xx [ 352 ] = xx [ 191 ] - xx [ 183 ]
* xx [ 204 ] ; xx [ 377 ] = xx [ 158 ] * xx [ 76 ] ; xx [ 378 ] = xx [ 157 ]
* xx [ 76 ] ; xx [ 379 ] = xx [ 76 ] * xx [ 155 ] ; xx [ 380 ] = - ( xx [ 158
] * xx [ 351 ] ) ; xx [ 381 ] = - ( xx [ 351 ] * xx [ 157 ] ) ; xx [ 382 ] =
- ( xx [ 351 ] * xx [ 155 ] ) ; xx [ 383 ] = xx [ 208 ] * xx [ 155 ] + xx [
157 ] * xx [ 352 ] ; xx [ 384 ] = xx [ 156 ] * xx [ 208 ] + xx [ 160 ] * xx [
352 ] ; xx [ 385 ] = xx [ 145 ] * xx [ 208 ] + xx [ 156 ] * xx [ 352 ] ;
pm_math_Matrix3x3_compose_ra ( xx + 173 , xx + 377 , xx + 386 ) ; xx [ 208 ]
= xx [ 63 ] * xx [ 149 ] ; xx [ 352 ] = xx [ 208 ] * xx [ 149 ] ; xx [ 353 ]
= xx [ 63 ] * xx [ 151 ] ; xx [ 354 ] = xx [ 353 ] * xx [ 151 ] ; xx [ 355 ]
= xx [ 353 ] * xx [ 149 ] ; xx [ 353 ] = ( xx [ 355 ] + xx [ 208 ] * xx [ 151
] ) * xx [ 6 ] ; xx [ 208 ] = xx [ 353 ] - xx [ 278 ] ; xx [ 356 ] = xx [ 6 ]
* ( xx [ 352 ] - xx [ 354 ] ) ; xx [ 357 ] = xx [ 286 ] - xx [ 356 ] ; xx [
377 ] = - ( xx [ 73 ] + ( xx [ 352 ] + xx [ 354 ] ) * xx [ 6 ] - xx [ 63 ] )
; xx [ 378 ] = xx [ 208 ] ; xx [ 379 ] = xx [ 357 ] ;
pm_math_Matrix3x3_postCross_ra ( xx + 218 , xx + 377 , xx + 395 ) ; xx [ 352
] = 0.04 ; xx [ 358 ] = xx [ 352 ] * xx [ 241 ] ; xx [ 380 ] = xx [ 352 ] *
xx [ 242 ] ; xx [ 381 ] = xx [ 239 ] * xx [ 352 ] ; xx [ 239 ] = xx [ 55 ] *
xx [ 381 ] ; xx [ 382 ] = xx [ 380 ] + xx [ 239 ] ; xx [ 383 ] = xx [ 358 ] -
xx [ 382 ] * xx [ 257 ] ; xx [ 384 ] = xx [ 381 ] - xx [ 382 ] * xx [ 248 ] ;
xx [ 385 ] = xx [ 383 ] * xx [ 236 ] - xx [ 260 ] * xx [ 384 ] ; xx [ 404 ] =
xx [ 352 ] * xx [ 249 ] ; xx [ 405 ] = xx [ 352 ] * xx [ 264 ] ; xx [ 406 ] =
xx [ 55 ] * xx [ 405 ] ; xx [ 407 ] = xx [ 404 ] + xx [ 406 ] ; xx [ 408 ] =
xx [ 246 ] * xx [ 352 ] ; xx [ 246 ] = xx [ 407 ] * xx [ 257 ] - xx [ 408 ] ;
xx [ 409 ] = xx [ 407 ] * xx [ 248 ] - xx [ 405 ] ; xx [ 410 ] = xx [ 236 ] *
xx [ 246 ] - xx [ 260 ] * xx [ 409 ] ; xx [ 411 ] = xx [ 236 ] * xx [ 385 ] -
xx [ 260 ] * xx [ 410 ] ; xx [ 412 ] = xx [ 382 ] / xx [ 256 ] ; xx [ 413 ] =
xx [ 254 ] * xx [ 412 ] ; xx [ 414 ] = ( xx [ 380 ] - xx [ 413 ] ) * xx [ 268
] ; xx [ 380 ] = xx [ 407 ] / xx [ 256 ] ; xx [ 415 ] = xx [ 254 ] * xx [ 380
] ; xx [ 416 ] = xx [ 268 ] * ( xx [ 415 ] - xx [ 404 ] ) ; xx [ 404 ] = xx [
260 ] * xx [ 383 ] + xx [ 384 ] * xx [ 236 ] ; xx [ 383 ] = xx [ 260 ] * xx [
246 ] + xx [ 236 ] * xx [ 409 ] ; xx [ 246 ] = xx [ 404 ] * xx [ 236 ] - xx [
383 ] * xx [ 260 ] ; xx [ 384 ] = xx [ 273 ] * xx [ 246 ] - xx [ 262 ] * xx [
411 ] ; xx [ 409 ] = xx [ 414 ] * xx [ 236 ] - xx [ 260 ] * xx [ 416 ] + xx [
384 ] ; xx [ 417 ] = xx [ 409 ] + xx [ 63 ] * xx [ 246 ] ; xx [ 418 ] = xx [
411 ] - xx [ 417 ] * xx [ 271 ] ; xx [ 419 ] = xx [ 246 ] - xx [ 417 ] * xx [
277 ] ; xx [ 420 ] = xx [ 55 ] * xx [ 281 ] ; xx [ 281 ] = xx [ 420 ] * xx [
268 ] ; xx [ 421 ] = xx [ 260 ] * xx [ 281 ] ; xx [ 422 ] = xx [ 262 ] * xx [
102 ] ; xx [ 423 ] = xx [ 421 ] + xx [ 422 ] ; xx [ 424 ] = xx [ 260 ] * xx [
385 ] + xx [ 236 ] * xx [ 410 ] ; xx [ 385 ] = xx [ 404 ] * xx [ 260 ] + xx [
383 ] * xx [ 236 ] ; xx [ 383 ] = xx [ 385 ] * xx [ 273 ] - xx [ 424 ] * xx [
262 ] ; xx [ 404 ] = xx [ 260 ] * xx [ 414 ] + xx [ 416 ] * xx [ 236 ] + xx [
383 ] ; xx [ 410 ] = xx [ 404 ] + xx [ 385 ] * xx [ 63 ] ; xx [ 414 ] = xx [
424 ] - xx [ 410 ] * xx [ 271 ] ; xx [ 416 ] = xx [ 385 ] - xx [ 410 ] * xx [
277 ] ; xx [ 425 ] = xx [ 281 ] * xx [ 236 ] ; xx [ 281 ] = xx [ 273 ] * xx [
102 ] ; xx [ 102 ] = xx [ 425 ] + xx [ 281 ] ; xx [ 426 ] = xx [ 266 ] - xx [
253 ] * xx [ 271 ] ; xx [ 427 ] = xx [ 217 ] - xx [ 253 ] * xx [ 277 ] ; xx [
428 ] = xx [ 209 ] * xx [ 418 ] + xx [ 211 ] * xx [ 419 ] + xx [ 212 ] * xx [
423 ] ; xx [ 429 ] = xx [ 210 ] * xx [ 418 ] + xx [ 214 ] * xx [ 419 ] + xx [
211 ] * xx [ 423 ] ; xx [ 430 ] = xx [ 203 ] * xx [ 418 ] + xx [ 210 ] * xx [
419 ] + xx [ 209 ] * xx [ 423 ] ; xx [ 431 ] = xx [ 414 ] * xx [ 209 ] + xx [
211 ] * xx [ 416 ] - xx [ 102 ] * xx [ 212 ] ; xx [ 432 ] = xx [ 210 ] * xx [
414 ] + xx [ 416 ] * xx [ 214 ] - xx [ 102 ] * xx [ 211 ] ; xx [ 433 ] = xx [
203 ] * xx [ 414 ] + xx [ 210 ] * xx [ 416 ] - xx [ 102 ] * xx [ 209 ] ; xx [
434 ] = xx [ 426 ] * xx [ 209 ] + xx [ 211 ] * xx [ 427 ] ; xx [ 435 ] = xx [
210 ] * xx [ 426 ] + xx [ 427 ] * xx [ 214 ] ; xx [ 436 ] = xx [ 203 ] * xx [
426 ] + xx [ 210 ] * xx [ 427 ] ; pm_math_Matrix3x3_compose_ra ( xx + 227 ,
xx + 428 , xx + 437 ) ; xx [ 209 ] = xx [ 63 ] * xx [ 205 ] ; xx [ 211 ] = xx
[ 209 ] * xx [ 205 ] ; xx [ 212 ] = xx [ 63 ] * xx [ 207 ] ; xx [ 414 ] = xx
[ 212 ] * xx [ 207 ] ; xx [ 416 ] = xx [ 212 ] * xx [ 205 ] ; xx [ 212 ] = (
xx [ 416 ] + xx [ 209 ] * xx [ 207 ] ) * xx [ 6 ] ; xx [ 209 ] = xx [ 278 ] +
xx [ 212 ] ; xx [ 278 ] = xx [ 6 ] * ( xx [ 211 ] - xx [ 414 ] ) ; xx [ 418 ]
= xx [ 286 ] - xx [ 278 ] ; xx [ 426 ] = - ( xx [ 73 ] + ( xx [ 211 ] + xx [
414 ] ) * xx [ 6 ] - xx [ 63 ] ) ; xx [ 427 ] = xx [ 209 ] ; xx [ 428 ] = xx
[ 418 ] ; pm_math_Matrix3x3_postCross_ra ( xx + 292 , xx + 426 , xx + 446 ) ;
xx [ 73 ] = xx [ 301 ] - xx [ 310 ] + xx [ 359 ] - xx [ 368 ] + xx [ 386 ] -
xx [ 395 ] + xx [ 437 ] - xx [ 446 ] ; xx [ 211 ] = xx [ 84 ] + xx [ 165 ] +
xx [ 219 ] + xx [ 293 ] ; xx [ 84 ] = 0.03000000000000001 ; xx [ 165 ] = xx [
73 ] + xx [ 211 ] * xx [ 84 ] ; xx [ 219 ] = 8.156250000000017e-6 ; xx [ 293
] = 2.265625000000013e-5 ; xx [ 419 ] = xx [ 293 ] * xx [ 39 ] ; xx [ 429 ] =
1.97265625e-3 ; xx [ 430 ] = xx [ 429 ] * xx [ 42 ] ; xx [ 431 ] = xx [ 219 ]
+ xx [ 419 ] * xx [ 39 ] + xx [ 430 ] * xx [ 42 ] + xx [ 56 ] * xx [ 8 ] ; xx
[ 432 ] = xx [ 293 ] * xx [ 42 ] ; xx [ 433 ] = xx [ 429 ] * xx [ 39 ] ; xx [
434 ] = xx [ 432 ] * xx [ 39 ] - xx [ 433 ] * xx [ 42 ] - xx [ 274 ] * xx [
56 ] ; xx [ 435 ] = xx [ 419 ] * xx [ 42 ] - xx [ 430 ] * xx [ 39 ] - xx [ 58
] * xx [ 8 ] ; xx [ 419 ] = 2.6015625e-4 ; xx [ 430 ] = xx [ 419 ] + xx [ 432
] * xx [ 42 ] + xx [ 433 ] * xx [ 39 ] + xx [ 58 ] * xx [ 274 ] ; xx [ 432 ]
= xx [ 40 ] / xx [ 62 ] ; xx [ 433 ] = xx [ 48 ] - xx [ 40 ] * xx [ 432 ] ;
xx [ 455 ] = xx [ 431 ] * xx [ 19 ] + xx [ 434 ] * xx [ 21 ] ; xx [ 456 ] =
xx [ 431 ] * xx [ 20 ] + xx [ 434 ] * xx [ 24 ] ; xx [ 457 ] = xx [ 431 ] *
xx [ 2 ] + xx [ 434 ] * xx [ 20 ] ; xx [ 458 ] = xx [ 435 ] * xx [ 19 ] + xx
[ 430 ] * xx [ 21 ] ; xx [ 459 ] = xx [ 435 ] * xx [ 20 ] + xx [ 430 ] * xx [
24 ] ; xx [ 460 ] = xx [ 435 ] * xx [ 2 ] + xx [ 430 ] * xx [ 20 ] ; xx [ 461
] = xx [ 22 ] * xx [ 433 ] ; xx [ 462 ] = xx [ 21 ] * xx [ 433 ] ; xx [ 463 ]
= xx [ 433 ] * xx [ 19 ] ; pm_math_Matrix3x3_compose_ra ( xx + 26 , xx + 455
, xx + 464 ) ; pm_math_Matrix3x3_postCross_ra ( xx + 301 , xx + 289 , xx + 26
) ; pm_math_Matrix3x3_preCross_ra ( xx + 310 , xx + 289 , xx + 455 ) ; xx [
19 ] = 2.809375000000016e-5 ; xx [ 21 ] = 2.339153333333333e-3 ; xx [ 22 ] =
xx [ 21 ] * xx [ 107 ] ; xx [ 48 ] = xx [ 22 ] * xx [ 107 ] ; xx [ 301 ] = xx
[ 21 ] * xx [ 104 ] ; xx [ 310 ] = xx [ 301 ] * xx [ 104 ] ; xx [ 433 ] = xx
[ 19 ] + xx [ 48 ] + xx [ 310 ] + xx [ 279 ] * xx [ 320 ] ; xx [ 436 ] = xx [
433 ] - xx [ 321 ] * xx [ 334 ] ; xx [ 473 ] = xx [ 301 ] * xx [ 107 ] ; xx [
107 ] = xx [ 22 ] * xx [ 104 ] ; xx [ 22 ] = xx [ 473 ] - xx [ 107 ] - xx [
279 ] * xx [ 327 ] ; xx [ 104 ] = xx [ 329 ] * xx [ 334 ] ; xx [ 301 ] = xx [
22 ] + xx [ 104 ] ; xx [ 474 ] = xx [ 436 ] * xx [ 101 ] - xx [ 301 ] * xx [
131 ] ; xx [ 475 ] = xx [ 107 ] - xx [ 473 ] - xx [ 279 ] * xx [ 322 ] ; xx [
107 ] = xx [ 475 ] + xx [ 104 ] ; xx [ 104 ] = 2.44609375e-3 ; xx [ 473 ] =
xx [ 104 ] + xx [ 310 ] + xx [ 48 ] + xx [ 279 ] * xx [ 326 ] + xx [ 55 ] *
xx [ 342 ] ; xx [ 48 ] = xx [ 473 ] - xx [ 329 ] * xx [ 319 ] ; xx [ 310 ] =
xx [ 107 ] * xx [ 101 ] - xx [ 131 ] * xx [ 48 ] ; xx [ 476 ] = xx [ 133 ] *
xx [ 343 ] ; xx [ 477 ] = xx [ 219 ] + xx [ 101 ] * xx [ 474 ] - xx [ 131 ] *
xx [ 310 ] + xx [ 476 ] + xx [ 476 ] + xx [ 133 ] * xx [ 344 ] ; xx [ 476 ] =
xx [ 339 ] / xx [ 141 ] ; xx [ 478 ] = xx [ 131 ] * xx [ 436 ] + xx [ 301 ] *
xx [ 101 ] ; xx [ 301 ] = xx [ 107 ] * xx [ 131 ] + xx [ 48 ] * xx [ 101 ] ;
xx [ 48 ] = xx [ 144 ] * xx [ 343 ] ; xx [ 107 ] = xx [ 133 ] * xx [ 347 ] ;
xx [ 343 ] = xx [ 478 ] * xx [ 101 ] - xx [ 301 ] * xx [ 131 ] - xx [ 48 ] -
xx [ 107 ] - xx [ 152 ] * xx [ 133 ] ; xx [ 436 ] = xx [ 332 ] * xx [ 476 ] ;
xx [ 479 ] = xx [ 124 ] * xx [ 476 ] ; xx [ 480 ] = xx [ 131 ] * xx [ 474 ] +
xx [ 310 ] * xx [ 101 ] - xx [ 107 ] - xx [ 48 ] - xx [ 144 ] * xx [ 344 ] ;
xx [ 48 ] = xx [ 144 ] * xx [ 347 ] ; xx [ 107 ] = xx [ 419 ] + xx [ 478 ] *
xx [ 131 ] + xx [ 301 ] * xx [ 101 ] + xx [ 48 ] + xx [ 48 ] + xx [ 144 ] *
xx [ 152 ] ; xx [ 48 ] = xx [ 332 ] / xx [ 141 ] ; xx [ 152 ] = xx [ 124 ] *
xx [ 48 ] ; xx [ 301 ] = xx [ 335 ] - xx [ 106 ] ; xx [ 310 ] = xx [ 328 ] -
xx [ 337 ] ; xx [ 335 ] = xx [ 139 ] * ( xx [ 301 ] * xx [ 101 ] - xx [ 131 ]
* xx [ 310 ] ) + xx [ 330 ] ; xx [ 330 ] = ( xx [ 131 ] * xx [ 301 ] + xx [
310 ] * xx [ 101 ] ) * xx [ 139 ] + xx [ 113 ] ; xx [ 113 ] = xx [ 124 ] / xx
[ 141 ] ; xx [ 481 ] = xx [ 477 ] - xx [ 339 ] * xx [ 476 ] ; xx [ 482 ] = xx
[ 343 ] - xx [ 436 ] ; xx [ 483 ] = xx [ 331 ] - xx [ 479 ] ; xx [ 484 ] = xx
[ 480 ] - xx [ 436 ] ; xx [ 485 ] = xx [ 107 ] - xx [ 332 ] * xx [ 48 ] ; xx
[ 486 ] = xx [ 323 ] - xx [ 152 ] ; xx [ 487 ] = xx [ 335 ] - xx [ 479 ] ; xx
[ 488 ] = xx [ 330 ] - xx [ 152 ] ; xx [ 489 ] = xx [ 136 ] - xx [ 124 ] * xx
[ 113 ] ; pm_math_Matrix3x3_composeTranspose_ra ( xx + 481 , xx + 92 , xx +
490 ) ; pm_math_Matrix3x3_compose_ra ( xx + 92 , xx + 490 , xx + 481 ) ;
pm_math_Matrix3x3_postCross_ra ( xx + 359 , xx + 348 , xx + 92 ) ;
pm_math_Matrix3x3_preCross_ra ( xx + 368 , xx + 348 , xx + 490 ) ; xx [ 136 ]
= xx [ 293 ] * xx [ 182 ] ; xx [ 139 ] = xx [ 429 ] * xx [ 185 ] ; xx [ 152 ]
= xx [ 219 ] + xx [ 136 ] * xx [ 182 ] + xx [ 139 ] * xx [ 185 ] + xx [ 193 ]
* xx [ 76 ] ; xx [ 301 ] = xx [ 293 ] * xx [ 185 ] ; xx [ 310 ] = xx [ 429 ]
* xx [ 182 ] ; xx [ 323 ] = xx [ 301 ] * xx [ 182 ] - xx [ 310 ] * xx [ 185 ]
- xx [ 351 ] * xx [ 193 ] ; xx [ 331 ] = xx [ 136 ] * xx [ 185 ] - xx [ 139 ]
* xx [ 182 ] - xx [ 195 ] * xx [ 76 ] ; xx [ 136 ] = xx [ 419 ] + xx [ 301 ]
* xx [ 185 ] + xx [ 310 ] * xx [ 182 ] + xx [ 195 ] * xx [ 351 ] ; xx [ 139 ]
= xx [ 183 ] / xx [ 199 ] ; xx [ 301 ] = xx [ 188 ] - xx [ 183 ] * xx [ 139 ]
; xx [ 499 ] = xx [ 152 ] * xx [ 155 ] + xx [ 323 ] * xx [ 157 ] ; xx [ 500 ]
= xx [ 152 ] * xx [ 156 ] + xx [ 323 ] * xx [ 160 ] ; xx [ 501 ] = xx [ 152 ]
* xx [ 145 ] + xx [ 323 ] * xx [ 156 ] ; xx [ 502 ] = xx [ 331 ] * xx [ 155 ]
+ xx [ 136 ] * xx [ 157 ] ; xx [ 503 ] = xx [ 331 ] * xx [ 156 ] + xx [ 136 ]
* xx [ 160 ] ; xx [ 504 ] = xx [ 331 ] * xx [ 145 ] + xx [ 136 ] * xx [ 156 ]
; xx [ 505 ] = xx [ 158 ] * xx [ 301 ] ; xx [ 506 ] = xx [ 157 ] * xx [ 301 ]
; xx [ 507 ] = xx [ 301 ] * xx [ 155 ] ; pm_math_Matrix3x3_compose_ra ( xx +
173 , xx + 499 , xx + 508 ) ; pm_math_Matrix3x3_postCross_ra ( xx + 386 , xx
+ 377 , xx + 173 ) ; pm_math_Matrix3x3_preCross_ra ( xx + 395 , xx + 377 , xx
+ 499 ) ; xx [ 155 ] = xx [ 21 ] * xx [ 240 ] ; xx [ 157 ] = xx [ 155 ] * xx
[ 240 ] ; xx [ 158 ] = xx [ 21 ] * xx [ 237 ] ; xx [ 188 ] = xx [ 158 ] * xx
[ 237 ] ; xx [ 301 ] = xx [ 19 ] + xx [ 157 ] + xx [ 188 ] + xx [ 352 ] * xx
[ 381 ] ; xx [ 310 ] = xx [ 301 ] - xx [ 382 ] * xx [ 412 ] ; xx [ 337 ] = xx
[ 158 ] * xx [ 240 ] ; xx [ 158 ] = xx [ 155 ] * xx [ 237 ] ; xx [ 155 ] = xx
[ 337 ] - xx [ 158 ] - xx [ 352 ] * xx [ 405 ] ; xx [ 237 ] = xx [ 407 ] * xx
[ 412 ] ; xx [ 240 ] = xx [ 155 ] + xx [ 237 ] ; xx [ 344 ] = xx [ 310 ] * xx
[ 236 ] - xx [ 240 ] * xx [ 260 ] ; xx [ 347 ] = xx [ 158 ] - xx [ 337 ] - xx
[ 352 ] * xx [ 358 ] ; xx [ 158 ] = xx [ 347 ] + xx [ 237 ] ; xx [ 237 ] = xx
[ 104 ] + xx [ 188 ] + xx [ 157 ] + xx [ 352 ] * xx [ 408 ] + xx [ 55 ] * xx
[ 420 ] ; xx [ 157 ] = xx [ 237 ] - xx [ 407 ] * xx [ 380 ] ; xx [ 188 ] = xx
[ 158 ] * xx [ 236 ] - xx [ 260 ] * xx [ 157 ] ; xx [ 337 ] = xx [ 262 ] * xx
[ 421 ] ; xx [ 359 ] = xx [ 219 ] + xx [ 236 ] * xx [ 344 ] - xx [ 260 ] * xx
[ 188 ] + xx [ 337 ] + xx [ 337 ] + xx [ 262 ] * xx [ 422 ] ; xx [ 337 ] = xx
[ 417 ] / xx [ 270 ] ; xx [ 368 ] = xx [ 260 ] * xx [ 310 ] + xx [ 240 ] * xx
[ 236 ] ; xx [ 240 ] = xx [ 158 ] * xx [ 260 ] + xx [ 157 ] * xx [ 236 ] ; xx
[ 157 ] = xx [ 273 ] * xx [ 421 ] ; xx [ 158 ] = xx [ 262 ] * xx [ 425 ] ; xx
[ 310 ] = xx [ 368 ] * xx [ 236 ] - xx [ 240 ] * xx [ 260 ] - xx [ 157 ] - xx
[ 158 ] - xx [ 281 ] * xx [ 262 ] ; xx [ 386 ] = xx [ 410 ] * xx [ 337 ] ; xx
[ 395 ] = xx [ 253 ] * xx [ 337 ] ; xx [ 421 ] = xx [ 260 ] * xx [ 344 ] + xx
[ 188 ] * xx [ 236 ] - xx [ 158 ] - xx [ 157 ] - xx [ 273 ] * xx [ 422 ] ; xx
[ 157 ] = xx [ 273 ] * xx [ 425 ] ; xx [ 158 ] = xx [ 419 ] + xx [ 368 ] * xx
[ 260 ] + xx [ 240 ] * xx [ 236 ] + xx [ 157 ] + xx [ 157 ] + xx [ 273 ] * xx
[ 281 ] ; xx [ 157 ] = xx [ 410 ] / xx [ 270 ] ; xx [ 188 ] = xx [ 253 ] * xx
[ 157 ] ; xx [ 240 ] = xx [ 239 ] - xx [ 413 ] ; xx [ 281 ] = xx [ 415 ] - xx
[ 406 ] ; xx [ 344 ] = xx [ 268 ] * ( xx [ 240 ] * xx [ 236 ] - xx [ 260 ] *
xx [ 281 ] ) + xx [ 384 ] ; xx [ 368 ] = ( xx [ 260 ] * xx [ 240 ] + xx [ 236
] * xx [ 281 ] ) * xx [ 268 ] + xx [ 383 ] ; xx [ 240 ] = xx [ 253 ] / xx [
270 ] ; xx [ 517 ] = xx [ 359 ] - xx [ 417 ] * xx [ 337 ] ; xx [ 518 ] = xx [
310 ] - xx [ 386 ] ; xx [ 519 ] = xx [ 409 ] - xx [ 395 ] ; xx [ 520 ] = xx [
421 ] - xx [ 386 ] ; xx [ 521 ] = xx [ 158 ] - xx [ 410 ] * xx [ 157 ] ; xx [
522 ] = xx [ 404 ] - xx [ 188 ] ; xx [ 523 ] = xx [ 344 ] - xx [ 395 ] ; xx [
524 ] = xx [ 368 ] - xx [ 188 ] ; xx [ 525 ] = xx [ 265 ] - xx [ 253 ] * xx [
240 ] ; pm_math_Matrix3x3_composeTranspose_ra ( xx + 517 , xx + 227 , xx +
526 ) ; pm_math_Matrix3x3_compose_ra ( xx + 227 , xx + 526 , xx + 517 ) ;
pm_math_Matrix3x3_postCross_ra ( xx + 437 , xx + 426 , xx + 227 ) ;
pm_math_Matrix3x3_preCross_ra ( xx + 446 , xx + 426 , xx + 526 ) ; xx [ 188 ]
= xx [ 302 ] - xx [ 313 ] + xx [ 360 ] - xx [ 371 ] + xx [ 387 ] - xx [ 398 ]
+ xx [ 438 ] - xx [ 449 ] ; xx [ 265 ] = xx [ 464 ] - xx [ 26 ] - xx [ 26 ] -
xx [ 455 ] + xx [ 481 ] - xx [ 92 ] - xx [ 92 ] - xx [ 490 ] + xx [ 508 ] -
xx [ 173 ] - xx [ 173 ] - xx [ 499 ] + xx [ 517 ] - xx [ 227 ] - xx [ 227 ] -
xx [ 526 ] + xx [ 188 ] * xx [ 84 ] + 0.08961000000000001 ; xx [ 268 ] = 10.3
; xx [ 281 ] = xx [ 87 ] + xx [ 168 ] + xx [ 222 ] + xx [ 296 ] + xx [ 268 ]
; xx [ 87 ] = xx [ 188 ] + xx [ 281 ] * xx [ 84 ] ; xx [ 168 ] = xx [ 265 ] +
xx [ 87 ] * xx [ 84 ] ; ii [ 0 ] = factorSymmetricPosDef ( xx + 168 , 1 , xx
+ 222 ) ; if ( ii [ 0 ] != 0 ) { return sm_ssci_recordRunTimeError (
"sm:compiler:messages:simulationErrors:DegenerateMass" ,
 "'wheel_leg/blance_leg_model/pitch' has a degenerate mass distribution on its follower side."
, neDiagMgr ) ; } xx [ 222 ] = xx [ 165 ] / xx [ 168 ] ; xx [ 296 ] = xx [ 87
] * xx [ 222 ] ; xx [ 302 ] = xx [ 85 ] + xx [ 166 ] + xx [ 220 ] + xx [ 294
] ; xx [ 85 ] = xx [ 303 ] - xx [ 316 ] + xx [ 361 ] - xx [ 374 ] + xx [ 388
] - xx [ 401 ] + xx [ 439 ] - xx [ 452 ] ; xx [ 166 ] = xx [ 90 ] + xx [ 171
] + xx [ 225 ] + xx [ 299 ] ; xx [ 90 ] = xx [ 85 ] + xx [ 166 ] * xx [ 84 ]
; xx [ 171 ] = xx [ 90 ] * xx [ 222 ] ; xx [ 220 ] = xx [ 87 ] / xx [ 168 ] ;
xx [ 225 ] = xx [ 88 ] + xx [ 169 ] + xx [ 223 ] + xx [ 297 ] ; xx [ 88 ] =
xx [ 90 ] * xx [ 220 ] ; xx [ 169 ] = xx [ 91 ] + xx [ 172 ] + xx [ 226 ] +
xx [ 300 ] + xx [ 268 ] ; xx [ 91 ] = xx [ 90 ] / xx [ 168 ] ; xx [ 535 ] =
xx [ 83 ] + xx [ 164 ] + xx [ 218 ] + xx [ 292 ] - xx [ 165 ] * xx [ 222 ] +
xx [ 268 ] ; xx [ 536 ] = xx [ 211 ] - xx [ 296 ] ; xx [ 537 ] = xx [ 302 ] -
xx [ 171 ] ; xx [ 538 ] = xx [ 86 ] + xx [ 167 ] + xx [ 221 ] + xx [ 295 ] -
xx [ 296 ] ; xx [ 539 ] = xx [ 281 ] - xx [ 87 ] * xx [ 220 ] ; xx [ 540 ] =
xx [ 225 ] - xx [ 88 ] ; xx [ 541 ] = xx [ 89 ] + xx [ 170 ] + xx [ 224 ] +
xx [ 298 ] - xx [ 171 ] ; xx [ 542 ] = xx [ 166 ] - xx [ 88 ] ; xx [ 543 ] =
xx [ 169 ] - xx [ 90 ] * xx [ 91 ] ; pm_math_Matrix3x3_composeTranspose_ra (
xx + 535 , xx + 10 , xx + 544 ) ; pm_math_Matrix3x3_compose_ra ( xx + 10 , xx
+ 544 , xx + 535 ) ; xx [ 544 ] = xx [ 535 ] ; xx [ 545 ] = xx [ 536 ] ; xx [
546 ] = xx [ 537 ] ; xx [ 547 ] = xx [ 536 ] ; xx [ 548 ] = xx [ 539 ] ; xx [
549 ] = xx [ 540 ] ; xx [ 550 ] = xx [ 537 ] ; xx [ 551 ] = xx [ 540 ] ; xx [
552 ] = xx [ 543 ] ; ii [ 0 ] = factorSymmetricPosDef ( xx + 544 , 3 , xx +
170 ) ; if ( ii [ 0 ] != 0 ) { return sm_ssci_recordRunTimeError (
"sm:compiler:messages:simulationErrors:DegenerateMass" ,
 "'wheel_leg/blance_leg_model/Cartesian Joint1' has a degenerate mass distribution on its follower side."
, neDiagMgr ) ; } xx [ 83 ] = - xx [ 5 ] ; xx [ 294 ] = - xx [ 3 ] ; xx [ 295
] = xx [ 83 ] ; xx [ 296 ] = xx [ 3 ] ; xx [ 297 ] = xx [ 83 ] ; xx [ 436 ] =
xx [ 4 ] ; xx [ 437 ] = xx [ 9 ] ; xx [ 438 ] = xx [ 4 ] ; xx [ 439 ] = xx [
9 ] ; xx [ 83 ] = xx [ 4 ] * state [ 7 ] ; xx [ 86 ] = xx [ 9 ] * state [ 7 ]
; xx [ 88 ] = ( xx [ 4 ] * xx [ 83 ] + xx [ 9 ] * xx [ 86 ] ) * xx [ 6 ] ; xx
[ 89 ] = state [ 7 ] - xx [ 88 ] ; xx [ 164 ] = xx [ 6 ] * ( xx [ 9 ] * xx [
83 ] - xx [ 4 ] * xx [ 86 ] ) ; xx [ 83 ] = xx [ 88 ] + state [ 25 ] ; xx [
170 ] = xx [ 89 ] ; xx [ 171 ] = xx [ 164 ] ; xx [ 172 ] = xx [ 83 ] ; xx [
298 ] = - ( xx [ 83 ] * xx [ 56 ] ) ; xx [ 299 ] = xx [ 83 ] * xx [ 58 ] ; xx
[ 300 ] = - ( xx [ 58 ] * xx [ 164 ] - xx [ 56 ] * xx [ 89 ] ) ;
pm_math_Vector3_cross_ra ( xx + 170 , xx + 298 , xx + 386 ) ; xx [ 86 ] = xx
[ 387 ] * xx [ 41 ] ; xx [ 166 ] = xx [ 386 ] * xx [ 41 ] ; xx [ 167 ] = xx [
83 ] + state [ 27 ] ; xx [ 211 ] = xx [ 55 ] * state [ 27 ] ; xx [ 218 ] = xx
[ 35 ] * ( xx [ 386 ] + xx [ 6 ] * ( xx [ 37 ] * xx [ 86 ] - xx [ 166 ] * xx
[ 41 ] ) - ( xx [ 83 ] + xx [ 167 ] ) * xx [ 211 ] ) ; xx [ 221 ] = xx [ 35 ]
* ( xx [ 387 ] - ( xx [ 37 ] * xx [ 166 ] + xx [ 86 ] * xx [ 41 ] ) * xx [ 6
] ) ; xx [ 86 ] = xx [ 164 ] * xx [ 41 ] ; xx [ 166 ] = xx [ 41 ] * xx [ 89 ]
; xx [ 223 ] = xx [ 89 ] + xx [ 6 ] * ( xx [ 37 ] * xx [ 86 ] - xx [ 166 ] *
xx [ 41 ] ) ; xx [ 224 ] = xx [ 164 ] - ( xx [ 37 ] * xx [ 166 ] + xx [ 86 ]
* xx [ 41 ] ) * xx [ 6 ] ; xx [ 298 ] = xx [ 223 ] ; xx [ 299 ] = xx [ 224 ]
; xx [ 300 ] = xx [ 167 ] ; xx [ 553 ] = xx [ 223 ] * xx [ 293 ] ; xx [ 554 ]
= xx [ 429 ] * xx [ 224 ] ; xx [ 555 ] = xx [ 167 ] * xx [ 51 ] ;
pm_math_Vector3_cross_ra ( xx + 298 , xx + 553 , xx + 556 ) ; xx [ 86 ] = xx
[ 558 ] + xx [ 55 ] * xx [ 221 ] ; xx [ 166 ] = xx [ 55 ] - xx [ 57 ] ; xx [
298 ] = xx [ 9 ] ; xx [ 299 ] = xx [ 4 ] ; xx [ 300 ] = xx [ 9 ] ; xx [ 167 ]
= xx [ 9 ] * xx [ 166 ] ; xx [ 226 ] = xx [ 9 ] * xx [ 56 ] ; xx [ 268 ] = -
xx [ 226 ] ; xx [ 281 ] = xx [ 4 ] * xx [ 56 ] ; xx [ 292 ] = xx [ 167 ] + xx
[ 281 ] ; xx [ 553 ] = - xx [ 167 ] ; xx [ 554 ] = xx [ 268 ] ; xx [ 555 ] =
xx [ 292 ] ; pm_math_Vector3_cross_ra ( xx + 298 , xx + 553 , xx + 559 ) ; xx
[ 167 ] = xx [ 4 ] * xx [ 226 ] ; xx [ 226 ] = xx [ 4 ] * xx [ 41 ] ; xx [
303 ] = xx [ 37 ] * xx [ 9 ] ; xx [ 313 ] = xx [ 226 ] + xx [ 303 ] ; xx [
316 ] = xx [ 313 ] * xx [ 55 ] ; xx [ 360 ] = xx [ 303 ] + xx [ 226 ] ; xx [
226 ] = xx [ 360 ] * xx [ 55 ] ; xx [ 303 ] = ( xx [ 313 ] * xx [ 316 ] + xx
[ 360 ] * xx [ 226 ] ) * xx [ 6 ] ; xx [ 361 ] = xx [ 166 ] + xx [ 6 ] * ( xx
[ 560 ] - xx [ 167 ] ) - xx [ 303 ] + xx [ 55 ] ; xx [ 371 ] = - xx [ 207 ] ;
xx [ 374 ] = - xx [ 205 ] ; xx [ 553 ] = xx [ 371 ] ; xx [ 554 ] = xx [ 374 ]
; xx [ 555 ] = xx [ 371 ] ; xx [ 383 ] = xx [ 273 ] * xx [ 207 ] ; xx [ 384 ]
= xx [ 207 ] * xx [ 262 ] ; xx [ 395 ] = xx [ 205 ] * xx [ 262 ] ; xx [ 398 ]
= xx [ 383 ] + xx [ 395 ] ; xx [ 562 ] = xx [ 383 ] ; xx [ 563 ] = xx [ 384 ]
; xx [ 564 ] = - xx [ 398 ] ; pm_math_Vector3_cross_ra ( xx + 553 , xx + 562
, xx + 565 ) ; xx [ 383 ] = xx [ 205 ] * xx [ 384 ] ; xx [ 401 ] = xx [ 205 ]
* xx [ 259 ] ; xx [ 404 ] = xx [ 216 ] * xx [ 207 ] ; xx [ 409 ] = xx [ 401 ]
+ xx [ 404 ] ; xx [ 413 ] = xx [ 409 ] * xx [ 55 ] ; xx [ 415 ] = xx [ 404 ]
+ xx [ 401 ] ; xx [ 401 ] = xx [ 415 ] * xx [ 55 ] ; xx [ 404 ] = ( xx [ 409
] * xx [ 413 ] + xx [ 415 ] * xx [ 401 ] ) * xx [ 6 ] ; xx [ 422 ] = xx [ 273
] + ( xx [ 566 ] - xx [ 383 ] ) * xx [ 6 ] - ( xx [ 414 ] + xx [ 414 ] ) * xx
[ 6 ] - xx [ 404 ] + xx [ 54 ] ; xx [ 414 ] = xx [ 55 ] - xx [ 272 ] ; xx [
425 ] = xx [ 207 ] * xx [ 414 ] ; xx [ 446 ] = xx [ 425 ] + xx [ 395 ] ; xx [
562 ] = xx [ 425 ] ; xx [ 563 ] = xx [ 384 ] ; xx [ 564 ] = - xx [ 446 ] ;
pm_math_Vector3_cross_ra ( xx + 553 , xx + 562 , xx + 568 ) ; xx [ 384 ] = xx
[ 414 ] + xx [ 6 ] * ( xx [ 569 ] - xx [ 383 ] ) - xx [ 404 ] + xx [ 55 ] ;
xx [ 383 ] = xx [ 384 ] / xx [ 256 ] ; xx [ 395 ] = xx [ 255 ] * xx [ 383 ] ;
xx [ 404 ] = xx [ 250 ] * xx [ 383 ] ; xx [ 425 ] = xx [ 404 ] * xx [ 259 ] ;
xx [ 449 ] = xx [ 395 ] * xx [ 259 ] ; xx [ 452 ] = xx [ 395 ] + xx [ 6 ] * (
xx [ 216 ] * xx [ 425 ] - xx [ 449 ] * xx [ 259 ] ) ; xx [ 395 ] = xx [ 404 ]
- ( xx [ 216 ] * xx [ 449 ] + xx [ 425 ] * xx [ 259 ] ) * xx [ 6 ] ; xx [ 404
] = xx [ 254 ] * xx [ 383 ] + xx [ 452 ] * xx [ 273 ] - xx [ 262 ] * xx [ 395
] ; xx [ 425 ] = ( xx [ 422 ] - ( xx [ 404 ] + xx [ 452 ] * xx [ 63 ] ) ) /
xx [ 270 ] ; xx [ 562 ] = xx [ 337 ] ; xx [ 563 ] = xx [ 157 ] ; xx [ 564 ] =
xx [ 240 ] ; xx [ 157 ] = xx [ 55 ] - xx [ 194 ] ; xx [ 240 ] = - xx [ 151 ]
; xx [ 337 ] = - xx [ 149 ] ; xx [ 571 ] = xx [ 240 ] ; xx [ 572 ] = xx [ 337
] ; xx [ 573 ] = xx [ 240 ] ; xx [ 449 ] = xx [ 151 ] * xx [ 157 ] ; xx [ 474
] = xx [ 151 ] * xx [ 193 ] ; xx [ 478 ] = xx [ 149 ] * xx [ 193 ] ; xx [ 479
] = xx [ 449 ] + xx [ 478 ] ; xx [ 574 ] = xx [ 449 ] ; xx [ 575 ] = xx [ 474
] ; xx [ 576 ] = - xx [ 479 ] ; pm_math_Vector3_cross_ra ( xx + 571 , xx +
574 , xx + 577 ) ; xx [ 449 ] = xx [ 149 ] * xx [ 474 ] ; xx [ 574 ] = xx [
149 ] * xx [ 184 ] ; xx [ 575 ] = xx [ 162 ] * xx [ 151 ] ; xx [ 576 ] = xx [
574 ] + xx [ 575 ] ; xx [ 580 ] = xx [ 576 ] * xx [ 55 ] ; xx [ 581 ] = xx [
575 ] + xx [ 574 ] ; xx [ 574 ] = xx [ 581 ] * xx [ 55 ] ; xx [ 575 ] = ( xx
[ 576 ] * xx [ 580 ] + xx [ 581 ] * xx [ 574 ] ) * xx [ 6 ] ; xx [ 582 ] = xx
[ 157 ] + xx [ 6 ] * ( xx [ 578 ] - xx [ 449 ] ) - xx [ 575 ] + xx [ 55 ] ;
xx [ 583 ] = xx [ 582 ] / xx [ 44 ] ; xx [ 584 ] = xx [ 43 ] * xx [ 583 ] ;
xx [ 585 ] = xx [ 584 ] * xx [ 184 ] ; xx [ 586 ] = xx [ 6 ] * xx [ 585 ] *
xx [ 184 ] - xx [ 584 ] ; xx [ 584 ] = xx [ 6 ] * xx [ 162 ] * xx [ 585 ] ;
xx [ 585 ] = xx [ 195 ] * xx [ 586 ] - xx [ 193 ] * xx [ 584 ] - xx [ 51 ] *
xx [ 583 ] ; xx [ 587 ] = xx [ 195 ] * xx [ 151 ] ; xx [ 588 ] = xx [ 587 ] +
xx [ 478 ] ; xx [ 589 ] = xx [ 587 ] ; xx [ 590 ] = xx [ 474 ] ; xx [ 591 ] =
- xx [ 588 ] ; pm_math_Vector3_cross_ra ( xx + 571 , xx + 589 , xx + 592 ) ;
xx [ 474 ] = xx [ 195 ] + ( xx [ 593 ] - xx [ 449 ] ) * xx [ 6 ] - ( xx [ 354
] + xx [ 354 ] ) * xx [ 6 ] - xx [ 575 ] + xx [ 54 ] ; xx [ 354 ] = ( xx [
585 ] + xx [ 63 ] * xx [ 586 ] + xx [ 474 ] ) / xx [ 199 ] ; xx [ 449 ] = xx
[ 585 ] - xx [ 183 ] * xx [ 354 ] ; xx [ 478 ] = xx [ 584 ] - xx [ 200 ] * xx
[ 354 ] ; xx [ 575 ] = xx [ 586 ] - xx [ 163 ] * xx [ 354 ] ; xx [ 584 ] = xx
[ 151 ] * xx [ 575 ] ; xx [ 585 ] = xx [ 151 ] * xx [ 478 ] ; xx [ 586 ] = xx
[ 149 ] * xx [ 478 ] - xx [ 584 ] ; xx [ 589 ] = xx [ 584 ] ; xx [ 590 ] = -
xx [ 585 ] ; xx [ 591 ] = xx [ 586 ] ; pm_math_Vector3_cross_ra ( xx + 571 ,
xx + 589 , xx + 595 ) ; xx [ 587 ] = xx [ 478 ] + xx [ 6 ] * ( xx [ 595 ] -
xx [ 149 ] * xx [ 584 ] ) ; xx [ 478 ] = xx [ 575 ] + ( xx [ 149 ] * xx [ 585
] + xx [ 596 ] ) * xx [ 6 ] ; xx [ 575 ] = ( xx [ 597 ] - xx [ 149 ] * xx [
586 ] ) * xx [ 6 ] ; xx [ 584 ] = xx [ 587 ] ; xx [ 585 ] = xx [ 478 ] ; xx [
586 ] = xx [ 575 ] ; pm_math_Vector3_cross_ra ( xx + 377 , xx + 584 , xx +
589 ) ; xx [ 595 ] = xx [ 374 ] ; xx [ 596 ] = xx [ 371 ] ; xx [ 597 ] = xx [
374 ] ; xx [ 598 ] = xx [ 371 ] ; xx [ 371 ] = xx [ 407 ] * xx [ 383 ] ; xx [
374 ] = xx [ 371 ] * xx [ 259 ] ; xx [ 584 ] = xx [ 382 ] * xx [ 383 ] ; xx [
585 ] = xx [ 584 ] * xx [ 259 ] ; xx [ 599 ] = xx [ 6 ] * ( xx [ 216 ] * xx [
374 ] - xx [ 585 ] * xx [ 259 ] ) + xx [ 584 ] + xx [ 417 ] * xx [ 425 ] ; xx
[ 600 ] = ( xx [ 216 ] * xx [ 585 ] + xx [ 374 ] * xx [ 259 ] ) * xx [ 6 ] -
xx [ 371 ] + xx [ 410 ] * xx [ 425 ] ; xx [ 601 ] = xx [ 404 ] + xx [ 253 ] *
xx [ 425 ] ; pm_math_Quaternion_xform_ra ( xx + 595 , xx + 599 , xx + 584 ) ;
xx [ 371 ] = xx [ 395 ] + xx [ 275 ] * xx [ 425 ] ; xx [ 374 ] = xx [ 452 ] +
xx [ 269 ] * xx [ 425 ] ; xx [ 395 ] = xx [ 374 ] * xx [ 207 ] ; xx [ 404 ] =
xx [ 371 ] * xx [ 207 ] ; xx [ 452 ] = xx [ 371 ] * xx [ 205 ] - xx [ 395 ] ;
xx [ 599 ] = xx [ 395 ] ; xx [ 600 ] = - xx [ 404 ] ; xx [ 601 ] = xx [ 452 ]
; pm_math_Vector3_cross_ra ( xx + 553 , xx + 599 , xx + 602 ) ; xx [ 585 ] =
xx [ 371 ] + xx [ 6 ] * ( xx [ 602 ] - xx [ 395 ] * xx [ 205 ] ) ; xx [ 371 ]
= xx [ 374 ] + ( xx [ 404 ] * xx [ 205 ] + xx [ 603 ] ) * xx [ 6 ] ; xx [ 374
] = ( xx [ 604 ] - xx [ 205 ] * xx [ 452 ] ) * xx [ 6 ] ; xx [ 599 ] = xx [
585 ] ; xx [ 600 ] = xx [ 371 ] ; xx [ 601 ] = xx [ 374 ] ;
pm_math_Vector3_cross_ra ( xx + 426 , xx + 599 , xx + 602 ) ; xx [ 395 ] = xx
[ 478 ] + xx [ 371 ] ; xx [ 371 ] = ( ( xx [ 149 ] * xx [ 149 ] * xx [ 449 ]
+ xx [ 151 ] * xx [ 151 ] * xx [ 449 ] ) * xx [ 6 ] + xx [ 589 ] + xx [ 584 ]
+ xx [ 602 ] + xx [ 395 ] * xx [ 84 ] ) / xx [ 168 ] ; xx [ 589 ] = xx [ 222
] ; xx [ 590 ] = xx [ 220 ] ; xx [ 591 ] = xx [ 91 ] ; xx [ 599 ] = xx [ 587
] + xx [ 585 ] - xx [ 165 ] * xx [ 371 ] ; xx [ 600 ] = xx [ 395 ] - xx [ 87
] * xx [ 371 ] ; xx [ 601 ] = xx [ 575 ] + xx [ 374 ] - xx [ 90 ] * xx [ 371
] ; pm_math_Quaternion_xform_ra ( xx + 294 , xx + 599 , xx + 584 ) ; xx [ 599
] = - xx [ 584 ] ; xx [ 600 ] = - xx [ 585 ] ; xx [ 601 ] = - xx [ 586 ] ;
solveSymmetricPosDef ( xx + 544 , xx + 599 , 3 , 1 , xx + 584 , xx + 602 ) ;
pm_math_Quaternion_inverseXform_ra ( xx + 294 , xx + 584 , xx + 599 ) ; xx [
374 ] = xx [ 371 ] + pm_math_Vector3_dot_ra ( xx + 589 , xx + 599 ) ; xx [
371 ] = xx [ 374 ] * xx [ 205 ] ; xx [ 395 ] = xx [ 374 ] * xx [ 207 ] ; xx [
404 ] = ( xx [ 371 ] * xx [ 205 ] + xx [ 395 ] * xx [ 207 ] ) * xx [ 6 ] ; xx
[ 449 ] = xx [ 404 ] - xx [ 374 ] ; xx [ 452 ] = xx [ 6 ] * ( xx [ 395 ] * xx
[ 205 ] - xx [ 371 ] * xx [ 207 ] ) ; xx [ 584 ] = xx [ 449 ] ; xx [ 585 ] =
xx [ 452 ] ; xx [ 586 ] = - xx [ 404 ] ; xx [ 371 ] = xx [ 600 ] - xx [ 374 ]
* xx [ 84 ] ; xx [ 602 ] = xx [ 599 ] ; xx [ 603 ] = xx [ 371 ] + xx [ 374 ]
* xx [ 418 ] ; xx [ 604 ] = xx [ 601 ] - xx [ 374 ] * xx [ 209 ] ;
pm_math_Quaternion_inverseXform_ra ( xx + 595 , xx + 602 , xx + 605 ) ; xx [
395 ] = xx [ 425 ] - ( pm_math_Vector3_dot_ra ( xx + 562 , xx + 584 ) + xx [
271 ] * xx [ 605 ] + xx [ 277 ] * xx [ 606 ] ) ; xx [ 584 ] = xx [ 412 ] ; xx
[ 585 ] = - xx [ 380 ] ; xx [ 586 ] = xx [ 276 ] ; xx [ 276 ] = xx [ 452 ] *
xx [ 259 ] ; xx [ 380 ] = xx [ 259 ] * xx [ 449 ] ; xx [ 412 ] = xx [ 395 ] -
xx [ 404 ] ; xx [ 602 ] = xx [ 449 ] + xx [ 6 ] * ( xx [ 216 ] * xx [ 276 ] -
xx [ 380 ] * xx [ 259 ] ) ; xx [ 603 ] = xx [ 452 ] - ( xx [ 216 ] * xx [ 380
] + xx [ 276 ] * xx [ 259 ] ) * xx [ 6 ] ; xx [ 604 ] = xx [ 412 ] ; xx [ 276
] = xx [ 605 ] - xx [ 262 ] * xx [ 412 ] ; xx [ 380 ] = xx [ 606 ] + xx [ 63
] * xx [ 395 ] + xx [ 273 ] * xx [ 412 ] ; xx [ 404 ] = xx [ 380 ] * xx [ 259
] ; xx [ 412 ] = xx [ 276 ] * xx [ 259 ] ; xx [ 605 ] = xx [ 337 ] ; xx [ 606
] = xx [ 240 ] ; xx [ 607 ] = xx [ 337 ] ; xx [ 608 ] = xx [ 240 ] ; xx [ 609
] = xx [ 599 ] ; xx [ 610 ] = xx [ 371 ] + xx [ 374 ] * xx [ 357 ] ; xx [ 611
] = xx [ 601 ] - xx [ 374 ] * xx [ 208 ] ; pm_math_Quaternion_inverseXform_ra
( xx + 605 , xx + 609 , xx + 599 ) ; xx [ 240 ] = ( xx [ 374 ] * xx [ 149 ] *
xx [ 149 ] + xx [ 374 ] * xx [ 151 ] * xx [ 151 ] ) * xx [ 6 ] ; xx [ 337 ] =
xx [ 354 ] + xx [ 202 ] * xx [ 599 ] + xx [ 204 ] * xx [ 600 ] - xx [ 139 ] *
xx [ 240 ] ; xx [ 354 ] = xx [ 240 ] + xx [ 337 ] ; xx [ 240 ] = xx [ 600 ] -
xx [ 337 ] * xx [ 63 ] - xx [ 354 ] * xx [ 195 ] ; xx [ 371 ] = xx [ 216 ] *
xx [ 205 ] - xx [ 207 ] * xx [ 259 ] ; xx [ 374 ] = ( xx [ 401 ] * xx [ 371 ]
+ xx [ 413 ] * xx [ 371 ] ) * xx [ 6 ] ; xx [ 401 ] = ( xx [ 205 ] * xx [ 398
] + xx [ 567 ] ) * xx [ 6 ] + ( xx [ 416 ] + xx [ 416 ] ) * xx [ 6 ] + xx [
374 ] ; xx [ 398 ] = ( xx [ 446 ] * xx [ 205 ] + xx [ 570 ] ) * xx [ 6 ] + xx
[ 374 ] ; xx [ 374 ] = xx [ 398 ] / xx [ 256 ] ; xx [ 413 ] = xx [ 255 ] * xx
[ 374 ] ; xx [ 416 ] = xx [ 250 ] * xx [ 374 ] ; xx [ 425 ] = xx [ 416 ] * xx
[ 259 ] ; xx [ 446 ] = xx [ 413 ] * xx [ 259 ] ; xx [ 449 ] = xx [ 413 ] + xx
[ 6 ] * ( xx [ 216 ] * xx [ 425 ] - xx [ 446 ] * xx [ 259 ] ) ; xx [ 413 ] =
xx [ 416 ] - ( xx [ 216 ] * xx [ 446 ] + xx [ 425 ] * xx [ 259 ] ) * xx [ 6 ]
; xx [ 416 ] = xx [ 254 ] * xx [ 374 ] + xx [ 449 ] * xx [ 273 ] - xx [ 262 ]
* xx [ 413 ] ; xx [ 425 ] = ( xx [ 401 ] - ( xx [ 416 ] + xx [ 449 ] * xx [
63 ] ) ) / xx [ 270 ] ; xx [ 446 ] = xx [ 162 ] * xx [ 149 ] - xx [ 151 ] *
xx [ 184 ] ; xx [ 452 ] = ( xx [ 574 ] * xx [ 446 ] + xx [ 580 ] * xx [ 446 ]
) * xx [ 6 ] ; xx [ 478 ] = ( xx [ 479 ] * xx [ 149 ] + xx [ 579 ] ) * xx [ 6
] + xx [ 452 ] ; xx [ 479 ] = xx [ 478 ] / xx [ 44 ] ; xx [ 565 ] = xx [ 43 ]
* xx [ 479 ] ; xx [ 566 ] = xx [ 565 ] * xx [ 184 ] ; xx [ 567 ] = xx [ 6 ] *
xx [ 566 ] * xx [ 184 ] - xx [ 565 ] ; xx [ 565 ] = xx [ 6 ] * xx [ 162 ] *
xx [ 566 ] ; xx [ 566 ] = xx [ 195 ] * xx [ 567 ] - xx [ 193 ] * xx [ 565 ] -
xx [ 51 ] * xx [ 479 ] ; xx [ 568 ] = ( xx [ 149 ] * xx [ 588 ] + xx [ 594 ]
) * xx [ 6 ] + ( xx [ 355 ] + xx [ 355 ] ) * xx [ 6 ] + xx [ 452 ] ; xx [ 355
] = ( xx [ 566 ] + xx [ 63 ] * xx [ 567 ] + xx [ 568 ] ) / xx [ 199 ] ; xx [
452 ] = xx [ 566 ] - xx [ 183 ] * xx [ 355 ] ; xx [ 566 ] = xx [ 565 ] - xx [
200 ] * xx [ 355 ] ; xx [ 565 ] = xx [ 567 ] - xx [ 163 ] * xx [ 355 ] ; xx [
567 ] = xx [ 151 ] * xx [ 565 ] ; xx [ 569 ] = xx [ 151 ] * xx [ 566 ] ; xx [
570 ] = xx [ 149 ] * xx [ 566 ] - xx [ 567 ] ; xx [ 577 ] = xx [ 567 ] ; xx [
578 ] = - xx [ 569 ] ; xx [ 579 ] = xx [ 570 ] ; pm_math_Vector3_cross_ra (
xx + 571 , xx + 577 , xx + 592 ) ; xx [ 574 ] = xx [ 566 ] + xx [ 6 ] * ( xx
[ 592 ] - xx [ 149 ] * xx [ 567 ] ) ; xx [ 566 ] = xx [ 565 ] + ( xx [ 149 ]
* xx [ 569 ] + xx [ 593 ] ) * xx [ 6 ] ; xx [ 565 ] = ( xx [ 594 ] - xx [ 149
] * xx [ 570 ] ) * xx [ 6 ] ; xx [ 577 ] = xx [ 574 ] ; xx [ 578 ] = xx [ 566
] ; xx [ 579 ] = xx [ 565 ] ; pm_math_Vector3_cross_ra ( xx + 377 , xx + 577
, xx + 592 ) ; xx [ 567 ] = xx [ 407 ] * xx [ 374 ] ; xx [ 569 ] = xx [ 567 ]
* xx [ 259 ] ; xx [ 570 ] = xx [ 382 ] * xx [ 374 ] ; xx [ 575 ] = xx [ 570 ]
* xx [ 259 ] ; xx [ 577 ] = xx [ 6 ] * ( xx [ 216 ] * xx [ 569 ] - xx [ 575 ]
* xx [ 259 ] ) + xx [ 570 ] + xx [ 417 ] * xx [ 425 ] ; xx [ 578 ] = ( xx [
216 ] * xx [ 575 ] + xx [ 569 ] * xx [ 259 ] ) * xx [ 6 ] - xx [ 567 ] + xx [
410 ] * xx [ 425 ] ; xx [ 579 ] = xx [ 416 ] + xx [ 253 ] * xx [ 425 ] ;
pm_math_Quaternion_xform_ra ( xx + 595 , xx + 577 , xx + 609 ) ; xx [ 416 ] =
xx [ 413 ] + xx [ 275 ] * xx [ 425 ] ; xx [ 413 ] = xx [ 449 ] + xx [ 269 ] *
xx [ 425 ] ; xx [ 449 ] = xx [ 413 ] * xx [ 207 ] ; xx [ 567 ] = xx [ 416 ] *
xx [ 207 ] ; xx [ 569 ] = xx [ 416 ] * xx [ 205 ] - xx [ 449 ] ; xx [ 577 ] =
xx [ 449 ] ; xx [ 578 ] = - xx [ 567 ] ; xx [ 579 ] = xx [ 569 ] ;
pm_math_Vector3_cross_ra ( xx + 553 , xx + 577 , xx + 610 ) ; xx [ 570 ] = xx
[ 416 ] + xx [ 6 ] * ( xx [ 610 ] - xx [ 449 ] * xx [ 205 ] ) ; xx [ 416 ] =
xx [ 413 ] + ( xx [ 567 ] * xx [ 205 ] + xx [ 611 ] ) * xx [ 6 ] ; xx [ 413 ]
= ( xx [ 612 ] - xx [ 205 ] * xx [ 569 ] ) * xx [ 6 ] ; xx [ 577 ] = xx [ 570
] ; xx [ 578 ] = xx [ 416 ] ; xx [ 579 ] = xx [ 413 ] ;
pm_math_Vector3_cross_ra ( xx + 426 , xx + 577 , xx + 610 ) ; xx [ 449 ] = xx
[ 566 ] + xx [ 416 ] ; xx [ 416 ] = ( ( xx [ 149 ] * xx [ 149 ] * xx [ 452 ]
+ xx [ 151 ] * xx [ 151 ] * xx [ 452 ] ) * xx [ 6 ] + xx [ 592 ] + xx [ 609 ]
+ xx [ 610 ] + xx [ 449 ] * xx [ 84 ] ) / xx [ 168 ] ; xx [ 577 ] = xx [ 574
] + xx [ 570 ] - xx [ 165 ] * xx [ 416 ] ; xx [ 578 ] = xx [ 449 ] - xx [ 87
] * xx [ 416 ] ; xx [ 579 ] = xx [ 565 ] + xx [ 413 ] - xx [ 90 ] * xx [ 416
] ; pm_math_Quaternion_xform_ra ( xx + 294 , xx + 577 , xx + 565 ) ; xx [ 577
] = - xx [ 565 ] ; xx [ 578 ] = - xx [ 566 ] ; xx [ 579 ] = - xx [ 567 ] ;
solveSymmetricPosDef ( xx + 544 , xx + 577 , 3 , 1 , xx + 565 , xx + 592 ) ;
pm_math_Quaternion_inverseXform_ra ( xx + 294 , xx + 565 , xx + 577 ) ; xx [
413 ] = xx [ 416 ] + pm_math_Vector3_dot_ra ( xx + 589 , xx + 577 ) ; xx [
416 ] = xx [ 413 ] * xx [ 205 ] ; xx [ 449 ] = xx [ 413 ] * xx [ 207 ] ; xx [
452 ] = ( xx [ 416 ] * xx [ 205 ] + xx [ 449 ] * xx [ 207 ] ) * xx [ 6 ] ; xx
[ 565 ] = xx [ 452 ] - xx [ 413 ] ; xx [ 566 ] = xx [ 6 ] * ( xx [ 449 ] * xx
[ 205 ] - xx [ 416 ] * xx [ 207 ] ) ; xx [ 592 ] = xx [ 565 ] ; xx [ 593 ] =
xx [ 566 ] ; xx [ 594 ] = - xx [ 452 ] ; xx [ 416 ] = xx [ 578 ] - xx [ 413 ]
* xx [ 84 ] ; xx [ 609 ] = xx [ 577 ] ; xx [ 610 ] = xx [ 416 ] + xx [ 413 ]
* xx [ 418 ] ; xx [ 611 ] = xx [ 579 ] - xx [ 413 ] * xx [ 209 ] ;
pm_math_Quaternion_inverseXform_ra ( xx + 595 , xx + 609 , xx + 612 ) ; xx [
449 ] = xx [ 425 ] - ( pm_math_Vector3_dot_ra ( xx + 562 , xx + 592 ) + xx [
271 ] * xx [ 612 ] + xx [ 277 ] * xx [ 613 ] ) ; xx [ 425 ] = xx [ 566 ] * xx
[ 259 ] ; xx [ 567 ] = xx [ 259 ] * xx [ 565 ] ; xx [ 569 ] = xx [ 449 ] - xx
[ 452 ] ; xx [ 592 ] = xx [ 565 ] + xx [ 6 ] * ( xx [ 216 ] * xx [ 425 ] - xx
[ 567 ] * xx [ 259 ] ) ; xx [ 593 ] = xx [ 566 ] - ( xx [ 216 ] * xx [ 567 ]
+ xx [ 425 ] * xx [ 259 ] ) * xx [ 6 ] ; xx [ 594 ] = xx [ 569 ] ; xx [ 425 ]
= xx [ 612 ] - xx [ 262 ] * xx [ 569 ] ; xx [ 452 ] = xx [ 613 ] + xx [ 63 ]
* xx [ 449 ] + xx [ 273 ] * xx [ 569 ] ; xx [ 565 ] = xx [ 452 ] * xx [ 259 ]
; xx [ 566 ] = xx [ 425 ] * xx [ 259 ] ; xx [ 567 ] = xx [ 374 ] - (
pm_math_Vector3_dot_ra ( xx + 584 , xx + 592 ) + ( xx [ 425 ] + xx [ 6 ] * (
xx [ 216 ] * xx [ 565 ] - xx [ 566 ] * xx [ 259 ] ) ) * xx [ 257 ] + xx [ 248
] * ( xx [ 452 ] - ( xx [ 216 ] * xx [ 566 ] + xx [ 565 ] * xx [ 259 ] ) * xx
[ 6 ] ) ) ; xx [ 592 ] = xx [ 577 ] ; xx [ 593 ] = xx [ 416 ] + xx [ 413 ] *
xx [ 357 ] ; xx [ 594 ] = xx [ 579 ] - xx [ 413 ] * xx [ 208 ] ;
pm_math_Quaternion_inverseXform_ra ( xx + 605 , xx + 592 , xx + 577 ) ; xx [
374 ] = ( xx [ 413 ] * xx [ 149 ] * xx [ 149 ] + xx [ 413 ] * xx [ 151 ] * xx
[ 151 ] ) * xx [ 6 ] ; xx [ 413 ] = xx [ 355 ] + xx [ 202 ] * xx [ 577 ] + xx
[ 204 ] * xx [ 578 ] - xx [ 139 ] * xx [ 374 ] ; xx [ 355 ] = xx [ 374 ] + xx
[ 413 ] ; xx [ 374 ] = xx [ 578 ] - xx [ 413 ] * xx [ 63 ] - xx [ 355 ] * xx
[ 195 ] ; xx [ 416 ] = xx [ 479 ] + xx [ 186 ] * ( xx [ 374 ] - ( xx [ 162 ]
* xx [ 184 ] * ( xx [ 577 ] + xx [ 355 ] * xx [ 193 ] ) + xx [ 184 ] * xx [
374 ] * xx [ 184 ] ) * xx [ 6 ] ) - xx [ 355 ] * xx [ 201 ] ; xx [ 355 ] = xx
[ 422 ] * xx [ 449 ] + xx [ 384 ] * xx [ 567 ] + xx [ 413 ] * xx [ 474 ] + xx
[ 416 ] * xx [ 582 ] ; xx [ 374 ] = xx [ 361 ] / xx [ 45 ] ; xx [ 425 ] = xx
[ 43 ] * xx [ 374 ] ; xx [ 452 ] = xx [ 425 ] * xx [ 41 ] ; xx [ 479 ] = xx [
6 ] * xx [ 37 ] * xx [ 452 ] ; xx [ 565 ] = xx [ 6 ] * xx [ 452 ] * xx [ 41 ]
- xx [ 425 ] ; xx [ 425 ] = xx [ 58 ] * xx [ 565 ] - xx [ 56 ] * xx [ 479 ] -
xx [ 51 ] * xx [ 374 ] ; xx [ 452 ] = xx [ 58 ] * xx [ 9 ] ; xx [ 566 ] = xx
[ 452 ] + xx [ 281 ] ; xx [ 577 ] = - xx [ 452 ] ; xx [ 578 ] = xx [ 268 ] ;
xx [ 579 ] = xx [ 566 ] ; pm_math_Vector3_cross_ra ( xx + 298 , xx + 577 , xx
+ 592 ) ; xx [ 268 ] = xx [ 58 ] + ( xx [ 593 ] - xx [ 167 ] ) * xx [ 6 ] - (
xx [ 283 ] + xx [ 283 ] ) * xx [ 6 ] - xx [ 303 ] + xx [ 54 ] ; xx [ 167 ] =
( xx [ 425 ] + xx [ 63 ] * xx [ 565 ] + xx [ 268 ] ) / xx [ 62 ] ; xx [ 281 ]
= xx [ 479 ] - xx [ 64 ] * xx [ 167 ] ; xx [ 283 ] = xx [ 565 ] - xx [ 38 ] *
xx [ 167 ] ; xx [ 303 ] = xx [ 9 ] * xx [ 283 ] ; xx [ 452 ] = xx [ 9 ] * xx
[ 281 ] ; xx [ 479 ] = xx [ 303 ] - xx [ 4 ] * xx [ 281 ] ; xx [ 577 ] = - xx
[ 303 ] ; xx [ 578 ] = xx [ 452 ] ; xx [ 579 ] = xx [ 479 ] ;
pm_math_Vector3_cross_ra ( xx + 298 , xx + 577 , xx + 609 ) ; xx [ 565 ] = xx
[ 281 ] + xx [ 6 ] * ( xx [ 609 ] - xx [ 4 ] * xx [ 303 ] ) ; xx [ 281 ] = xx
[ 55 ] - xx [ 143 ] ; xx [ 577 ] = xx [ 72 ] ; xx [ 578 ] = xx [ 70 ] ; xx [
579 ] = xx [ 72 ] ; xx [ 303 ] = xx [ 72 ] * xx [ 281 ] ; xx [ 569 ] = xx [
72 ] * xx [ 133 ] ; xx [ 570 ] = - xx [ 569 ] ; xx [ 574 ] = xx [ 70 ] * xx [
133 ] ; xx [ 575 ] = xx [ 303 ] + xx [ 574 ] ; xx [ 612 ] = - xx [ 303 ] ; xx
[ 613 ] = xx [ 570 ] ; xx [ 614 ] = xx [ 575 ] ; pm_math_Vector3_cross_ra (
xx + 577 , xx + 612 , xx + 615 ) ; xx [ 303 ] = xx [ 70 ] * xx [ 569 ] ; xx [
569 ] = xx [ 70 ] * xx [ 130 ] ; xx [ 580 ] = xx [ 81 ] * xx [ 72 ] ; xx [
587 ] = xx [ 569 ] + xx [ 580 ] ; xx [ 588 ] = xx [ 587 ] * xx [ 55 ] ; xx [
600 ] = xx [ 580 ] + xx [ 569 ] ; xx [ 569 ] = xx [ 600 ] * xx [ 55 ] ; xx [
580 ] = ( xx [ 587 ] * xx [ 588 ] + xx [ 600 ] * xx [ 569 ] ) * xx [ 6 ] ; xx
[ 601 ] = xx [ 281 ] + xx [ 6 ] * ( xx [ 616 ] - xx [ 303 ] ) - xx [ 580 ] +
xx [ 55 ] ; xx [ 612 ] = xx [ 601 ] / xx [ 127 ] ; xx [ 613 ] = xx [ 117 ] *
xx [ 612 ] ; xx [ 614 ] = xx [ 126 ] * xx [ 612 ] ; xx [ 618 ] = xx [ 614 ] *
xx [ 130 ] ; xx [ 619 ] = xx [ 613 ] * xx [ 130 ] ; xx [ 620 ] = xx [ 613 ] -
( xx [ 81 ] * xx [ 618 ] + xx [ 619 ] * xx [ 130 ] ) * xx [ 6 ] ; xx [ 613 ]
= xx [ 144 ] * xx [ 72 ] ; xx [ 621 ] = xx [ 613 ] + xx [ 574 ] ; xx [ 622 ]
= - xx [ 613 ] ; xx [ 623 ] = xx [ 570 ] ; xx [ 624 ] = xx [ 621 ] ;
pm_math_Vector3_cross_ra ( xx + 577 , xx + 622 , xx + 625 ) ; xx [ 570 ] = xx
[ 144 ] + ( xx [ 626 ] - xx [ 303 ] ) * xx [ 6 ] - ( xx [ 336 ] + xx [ 336 ]
) * xx [ 6 ] - xx [ 580 ] + xx [ 54 ] ; xx [ 54 ] = xx [ 614 ] + xx [ 6 ] * (
xx [ 81 ] * xx [ 619 ] - xx [ 618 ] * xx [ 130 ] ) ; xx [ 303 ] = xx [ 125 ]
* xx [ 612 ] + xx [ 54 ] * xx [ 144 ] - xx [ 133 ] * xx [ 620 ] ; xx [ 336 ]
= ( xx [ 570 ] - ( xx [ 303 ] + xx [ 54 ] * xx [ 63 ] ) ) / xx [ 141 ] ; xx [
574 ] = xx [ 620 ] + xx [ 146 ] * xx [ 336 ] ; xx [ 580 ] = xx [ 54 ] + xx [
140 ] * xx [ 336 ] ; xx [ 54 ] = xx [ 580 ] * xx [ 72 ] ; xx [ 613 ] = xx [
574 ] * xx [ 72 ] ; xx [ 614 ] = xx [ 54 ] - xx [ 574 ] * xx [ 70 ] ; xx [
618 ] = - xx [ 54 ] ; xx [ 619 ] = xx [ 613 ] ; xx [ 620 ] = xx [ 614 ] ;
pm_math_Vector3_cross_ra ( xx + 577 , xx + 618 , xx + 622 ) ; xx [ 618 ] = xx
[ 574 ] + xx [ 6 ] * ( xx [ 622 ] - xx [ 54 ] * xx [ 70 ] ) ; xx [ 54 ] = xx
[ 425 ] - xx [ 40 ] * xx [ 167 ] ; xx [ 425 ] = xx [ 283 ] + ( xx [ 4 ] * xx
[ 452 ] + xx [ 610 ] ) * xx [ 6 ] ; xx [ 283 ] = ( xx [ 4 ] * xx [ 479 ] + xx
[ 611 ] ) * xx [ 6 ] ; xx [ 609 ] = xx [ 565 ] ; xx [ 610 ] = xx [ 425 ] ; xx
[ 611 ] = xx [ 283 ] ; pm_math_Vector3_cross_ra ( xx + 289 , xx + 609 , xx +
628 ) ; xx [ 629 ] = xx [ 70 ] ; xx [ 630 ] = xx [ 72 ] ; xx [ 631 ] = xx [
70 ] ; xx [ 632 ] = xx [ 72 ] ; xx [ 452 ] = xx [ 321 ] * xx [ 612 ] ; xx [
479 ] = xx [ 452 ] * xx [ 130 ] ; xx [ 574 ] = xx [ 329 ] * xx [ 612 ] ; xx [
609 ] = xx [ 574 ] * xx [ 130 ] ; xx [ 633 ] = xx [ 6 ] * ( xx [ 479 ] * xx [
130 ] - xx [ 81 ] * xx [ 609 ] ) - xx [ 452 ] + xx [ 339 ] * xx [ 336 ] ; xx
[ 634 ] = xx [ 574 ] - ( xx [ 81 ] * xx [ 479 ] + xx [ 609 ] * xx [ 130 ] ) *
xx [ 6 ] + xx [ 332 ] * xx [ 336 ] ; xx [ 635 ] = xx [ 303 ] + xx [ 124 ] *
xx [ 336 ] ; pm_math_Quaternion_xform_ra ( xx + 629 , xx + 633 , xx + 609 ) ;
xx [ 303 ] = xx [ 580 ] + ( xx [ 613 ] * xx [ 70 ] + xx [ 623 ] ) * xx [ 6 ]
; xx [ 452 ] = ( xx [ 70 ] * xx [ 614 ] + xx [ 624 ] ) * xx [ 6 ] ; xx [ 622
] = xx [ 618 ] ; xx [ 623 ] = xx [ 303 ] ; xx [ 624 ] = xx [ 452 ] ;
pm_math_Vector3_cross_ra ( xx + 348 , xx + 622 , xx + 633 ) ; xx [ 479 ] = xx
[ 425 ] + xx [ 303 ] ; xx [ 303 ] = ( ( xx [ 4 ] * xx [ 4 ] * xx [ 54 ] + xx
[ 9 ] * xx [ 9 ] * xx [ 54 ] ) * xx [ 6 ] + xx [ 628 ] + xx [ 609 ] + xx [
633 ] + xx [ 479 ] * xx [ 84 ] ) / xx [ 168 ] ; xx [ 609 ] = xx [ 565 ] + xx
[ 618 ] - xx [ 165 ] * xx [ 303 ] ; xx [ 610 ] = xx [ 479 ] - xx [ 87 ] * xx
[ 303 ] ; xx [ 611 ] = xx [ 283 ] + xx [ 452 ] - xx [ 90 ] * xx [ 303 ] ;
pm_math_Quaternion_xform_ra ( xx + 294 , xx + 609 , xx + 618 ) ; xx [ 609 ] =
- xx [ 618 ] ; xx [ 610 ] = - xx [ 619 ] ; xx [ 611 ] = - xx [ 620 ] ;
solveSymmetricPosDef ( xx + 544 , xx + 609 , 3 , 1 , xx + 618 , xx + 622 ) ;
pm_math_Quaternion_inverseXform_ra ( xx + 294 , xx + 618 , xx + 609 ) ; xx [
54 ] = xx [ 303 ] + pm_math_Vector3_dot_ra ( xx + 589 , xx + 609 ) ; xx [ 283
] = xx [ 610 ] - xx [ 54 ] * xx [ 84 ] ; xx [ 618 ] = xx [ 609 ] ; xx [ 619 ]
= xx [ 283 ] + xx [ 54 ] * xx [ 357 ] ; xx [ 620 ] = xx [ 611 ] - xx [ 54 ] *
xx [ 208 ] ; pm_math_Quaternion_inverseXform_ra ( xx + 605 , xx + 618 , xx +
622 ) ; xx [ 303 ] = ( xx [ 54 ] * xx [ 149 ] * xx [ 149 ] + xx [ 54 ] * xx [
151 ] * xx [ 151 ] ) * xx [ 6 ] ; xx [ 425 ] = xx [ 202 ] * xx [ 622 ] + xx [
204 ] * xx [ 623 ] - xx [ 139 ] * xx [ 303 ] ; xx [ 452 ] = xx [ 54 ] * xx [
205 ] ; xx [ 479 ] = xx [ 54 ] * xx [ 207 ] ; xx [ 565 ] = ( xx [ 452 ] * xx
[ 205 ] + xx [ 479 ] * xx [ 207 ] ) * xx [ 6 ] ; xx [ 574 ] = xx [ 565 ] - xx
[ 54 ] ; xx [ 580 ] = xx [ 6 ] * ( xx [ 479 ] * xx [ 205 ] - xx [ 452 ] * xx
[ 207 ] ) ; xx [ 618 ] = xx [ 574 ] ; xx [ 619 ] = xx [ 580 ] ; xx [ 620 ] =
- xx [ 565 ] ; xx [ 633 ] = xx [ 609 ] ; xx [ 634 ] = xx [ 283 ] + xx [ 54 ]
* xx [ 418 ] ; xx [ 635 ] = xx [ 611 ] - xx [ 54 ] * xx [ 209 ] ;
pm_math_Quaternion_inverseXform_ra ( xx + 595 , xx + 633 , xx + 636 ) ; xx [
452 ] = pm_math_Vector3_dot_ra ( xx + 562 , xx + 618 ) + xx [ 271 ] * xx [
636 ] + xx [ 277 ] * xx [ 637 ] ; xx [ 479 ] = xx [ 580 ] * xx [ 259 ] ; xx [
610 ] = xx [ 259 ] * xx [ 574 ] ; xx [ 613 ] = xx [ 565 ] + xx [ 452 ] ; xx [
618 ] = xx [ 574 ] + xx [ 6 ] * ( xx [ 216 ] * xx [ 479 ] - xx [ 610 ] * xx [
259 ] ) ; xx [ 619 ] = xx [ 580 ] - ( xx [ 216 ] * xx [ 610 ] + xx [ 479 ] *
xx [ 259 ] ) * xx [ 6 ] ; xx [ 620 ] = - xx [ 613 ] ; xx [ 479 ] = xx [ 636 ]
+ xx [ 613 ] * xx [ 262 ] ; xx [ 565 ] = xx [ 637 ] - xx [ 452 ] * xx [ 63 ]
- xx [ 613 ] * xx [ 273 ] ; xx [ 574 ] = xx [ 259 ] * xx [ 565 ] ; xx [ 580 ]
= xx [ 259 ] * xx [ 479 ] ; xx [ 610 ] = pm_math_Vector3_dot_ra ( xx + 584 ,
xx + 618 ) + ( xx [ 479 ] + xx [ 6 ] * ( xx [ 216 ] * xx [ 574 ] - xx [ 580 ]
* xx [ 259 ] ) ) * xx [ 257 ] + xx [ 248 ] * ( xx [ 565 ] - ( xx [ 216 ] * xx
[ 580 ] + xx [ 574 ] * xx [ 259 ] ) * xx [ 6 ] ) ; xx [ 479 ] = xx [ 303 ] +
xx [ 425 ] ; xx [ 303 ] = xx [ 623 ] - xx [ 63 ] * xx [ 425 ] - xx [ 479 ] *
xx [ 195 ] ; xx [ 565 ] = xx [ 186 ] * ( xx [ 303 ] - ( xx [ 162 ] * xx [ 184
] * ( xx [ 622 ] + xx [ 479 ] * xx [ 193 ] ) + xx [ 184 ] * xx [ 303 ] * xx [
184 ] ) * xx [ 6 ] ) - xx [ 479 ] * xx [ 201 ] ; xx [ 303 ] = xx [ 474 ] * xx
[ 425 ] - ( xx [ 452 ] * xx [ 422 ] + xx [ 610 ] * xx [ 384 ] ) + xx [ 582 ]
* xx [ 565 ] ; xx [ 479 ] = xx [ 9 ] * xx [ 41 ] - xx [ 37 ] * xx [ 4 ] ; xx
[ 559 ] = ( xx [ 226 ] * xx [ 479 ] + xx [ 316 ] * xx [ 479 ] ) * xx [ 6 ] ;
xx [ 226 ] = ( xx [ 292 ] * xx [ 4 ] + xx [ 561 ] ) * xx [ 6 ] - xx [ 559 ] ;
xx [ 292 ] = xx [ 226 ] / xx [ 45 ] ; xx [ 316 ] = xx [ 43 ] * xx [ 292 ] ;
xx [ 560 ] = xx [ 316 ] * xx [ 41 ] ; xx [ 561 ] = xx [ 6 ] * xx [ 37 ] * xx
[ 560 ] ; xx [ 574 ] = xx [ 6 ] * xx [ 560 ] * xx [ 41 ] - xx [ 316 ] ; xx [
316 ] = xx [ 58 ] * xx [ 574 ] - xx [ 56 ] * xx [ 561 ] - xx [ 51 ] * xx [
292 ] ; xx [ 560 ] = ( xx [ 4 ] * xx [ 566 ] + xx [ 594 ] ) * xx [ 6 ] + ( xx
[ 284 ] + xx [ 284 ] ) * xx [ 6 ] - xx [ 559 ] ; xx [ 284 ] = ( xx [ 316 ] +
xx [ 63 ] * xx [ 574 ] + xx [ 560 ] ) / xx [ 62 ] ; xx [ 559 ] = xx [ 561 ] -
xx [ 64 ] * xx [ 284 ] ; xx [ 561 ] = xx [ 574 ] - xx [ 38 ] * xx [ 284 ] ;
xx [ 566 ] = xx [ 9 ] * xx [ 561 ] ; xx [ 574 ] = xx [ 9 ] * xx [ 559 ] ; xx
[ 580 ] = xx [ 566 ] - xx [ 4 ] * xx [ 559 ] ; xx [ 592 ] = - xx [ 566 ] ; xx
[ 593 ] = xx [ 574 ] ; xx [ 594 ] = xx [ 580 ] ; pm_math_Vector3_cross_ra (
xx + 298 , xx + 592 , xx + 618 ) ; xx [ 592 ] = xx [ 559 ] + xx [ 6 ] * ( xx
[ 618 ] - xx [ 4 ] * xx [ 566 ] ) ; xx [ 559 ] = xx [ 72 ] * xx [ 130 ] - xx
[ 81 ] * xx [ 70 ] ; xx [ 566 ] = ( xx [ 569 ] * xx [ 559 ] + xx [ 588 ] * xx
[ 559 ] ) * xx [ 6 ] ; xx [ 569 ] = ( xx [ 575 ] * xx [ 70 ] + xx [ 617 ] ) *
xx [ 6 ] - xx [ 566 ] ; xx [ 575 ] = xx [ 569 ] / xx [ 127 ] ; xx [ 588 ] =
xx [ 117 ] * xx [ 575 ] ; xx [ 593 ] = xx [ 126 ] * xx [ 575 ] ; xx [ 594 ] =
xx [ 593 ] * xx [ 130 ] ; xx [ 613 ] = xx [ 588 ] * xx [ 130 ] ; xx [ 614 ] =
xx [ 588 ] - ( xx [ 81 ] * xx [ 594 ] + xx [ 613 ] * xx [ 130 ] ) * xx [ 6 ]
; xx [ 588 ] = ( xx [ 70 ] * xx [ 621 ] + xx [ 627 ] ) * xx [ 6 ] + ( xx [
338 ] + xx [ 338 ] ) * xx [ 6 ] - xx [ 566 ] ; xx [ 338 ] = xx [ 593 ] + xx [
6 ] * ( xx [ 81 ] * xx [ 613 ] - xx [ 594 ] * xx [ 130 ] ) ; xx [ 566 ] = xx
[ 125 ] * xx [ 575 ] + xx [ 338 ] * xx [ 144 ] - xx [ 133 ] * xx [ 614 ] ; xx
[ 593 ] = ( xx [ 588 ] - ( xx [ 566 ] + xx [ 338 ] * xx [ 63 ] ) ) / xx [ 141
] ; xx [ 594 ] = xx [ 614 ] + xx [ 146 ] * xx [ 593 ] ; xx [ 613 ] = xx [ 338
] + xx [ 140 ] * xx [ 593 ] ; xx [ 338 ] = xx [ 613 ] * xx [ 72 ] ; xx [ 614
] = xx [ 594 ] * xx [ 72 ] ; xx [ 615 ] = xx [ 338 ] - xx [ 594 ] * xx [ 70 ]
; xx [ 621 ] = - xx [ 338 ] ; xx [ 622 ] = xx [ 614 ] ; xx [ 623 ] = xx [ 615
] ; pm_math_Vector3_cross_ra ( xx + 577 , xx + 621 , xx + 624 ) ; xx [ 616 ]
= xx [ 594 ] + xx [ 6 ] * ( xx [ 624 ] - xx [ 338 ] * xx [ 70 ] ) ; xx [ 338
] = xx [ 316 ] - xx [ 40 ] * xx [ 284 ] ; xx [ 316 ] = xx [ 561 ] + ( xx [ 4
] * xx [ 574 ] + xx [ 619 ] ) * xx [ 6 ] ; xx [ 561 ] = ( xx [ 4 ] * xx [ 580
] + xx [ 620 ] ) * xx [ 6 ] ; xx [ 617 ] = xx [ 592 ] ; xx [ 618 ] = xx [ 316
] ; xx [ 619 ] = xx [ 561 ] ; pm_math_Vector3_cross_ra ( xx + 289 , xx + 617
, xx + 620 ) ; xx [ 574 ] = xx [ 321 ] * xx [ 575 ] ; xx [ 580 ] = xx [ 574 ]
* xx [ 130 ] ; xx [ 594 ] = xx [ 329 ] * xx [ 575 ] ; xx [ 617 ] = xx [ 594 ]
* xx [ 130 ] ; xx [ 621 ] = xx [ 6 ] * ( xx [ 580 ] * xx [ 130 ] - xx [ 81 ]
* xx [ 617 ] ) - xx [ 574 ] + xx [ 339 ] * xx [ 593 ] ; xx [ 622 ] = xx [ 594
] - ( xx [ 81 ] * xx [ 580 ] + xx [ 617 ] * xx [ 130 ] ) * xx [ 6 ] + xx [
332 ] * xx [ 593 ] ; xx [ 623 ] = xx [ 566 ] + xx [ 124 ] * xx [ 593 ] ;
pm_math_Quaternion_xform_ra ( xx + 629 , xx + 621 , xx + 617 ) ; xx [ 566 ] =
xx [ 613 ] + ( xx [ 614 ] * xx [ 70 ] + xx [ 625 ] ) * xx [ 6 ] ; xx [ 574 ]
= ( xx [ 70 ] * xx [ 615 ] + xx [ 626 ] ) * xx [ 6 ] ; xx [ 613 ] = xx [ 616
] ; xx [ 614 ] = xx [ 566 ] ; xx [ 615 ] = xx [ 574 ] ;
pm_math_Vector3_cross_ra ( xx + 348 , xx + 613 , xx + 621 ) ; xx [ 580 ] = xx
[ 316 ] + xx [ 566 ] ; xx [ 316 ] = ( ( xx [ 4 ] * xx [ 4 ] * xx [ 338 ] + xx
[ 9 ] * xx [ 9 ] * xx [ 338 ] ) * xx [ 6 ] + xx [ 620 ] + xx [ 617 ] + xx [
621 ] + xx [ 580 ] * xx [ 84 ] ) / xx [ 168 ] ; xx [ 613 ] = xx [ 592 ] + xx
[ 616 ] - xx [ 165 ] * xx [ 316 ] ; xx [ 614 ] = xx [ 580 ] - xx [ 87 ] * xx
[ 316 ] ; xx [ 615 ] = xx [ 561 ] + xx [ 574 ] - xx [ 90 ] * xx [ 316 ] ;
pm_math_Quaternion_xform_ra ( xx + 294 , xx + 613 , xx + 616 ) ; xx [ 613 ] =
- xx [ 616 ] ; xx [ 614 ] = - xx [ 617 ] ; xx [ 615 ] = - xx [ 618 ] ;
solveSymmetricPosDef ( xx + 544 , xx + 613 , 3 , 1 , xx + 616 , xx + 619 ) ;
pm_math_Quaternion_inverseXform_ra ( xx + 294 , xx + 616 , xx + 613 ) ; xx [
338 ] = xx [ 316 ] + pm_math_Vector3_dot_ra ( xx + 589 , xx + 613 ) ; xx [
316 ] = xx [ 614 ] - xx [ 338 ] * xx [ 84 ] ; xx [ 616 ] = xx [ 613 ] ; xx [
617 ] = xx [ 316 ] + xx [ 338 ] * xx [ 357 ] ; xx [ 618 ] = xx [ 615 ] - xx [
338 ] * xx [ 208 ] ; pm_math_Quaternion_inverseXform_ra ( xx + 605 , xx + 616
, xx + 619 ) ; xx [ 561 ] = ( xx [ 338 ] * xx [ 149 ] * xx [ 149 ] + xx [ 338
] * xx [ 151 ] * xx [ 151 ] ) * xx [ 6 ] ; xx [ 566 ] = xx [ 202 ] * xx [ 619
] + xx [ 204 ] * xx [ 620 ] - xx [ 139 ] * xx [ 561 ] ; xx [ 574 ] = xx [ 338
] * xx [ 205 ] ; xx [ 580 ] = xx [ 338 ] * xx [ 207 ] ; xx [ 592 ] = ( xx [
574 ] * xx [ 205 ] + xx [ 580 ] * xx [ 207 ] ) * xx [ 6 ] ; xx [ 594 ] = xx [
592 ] - xx [ 338 ] ; xx [ 614 ] = xx [ 6 ] * ( xx [ 580 ] * xx [ 205 ] - xx [
574 ] * xx [ 207 ] ) ; xx [ 616 ] = xx [ 594 ] ; xx [ 617 ] = xx [ 614 ] ; xx
[ 618 ] = - xx [ 592 ] ; xx [ 621 ] = xx [ 613 ] ; xx [ 622 ] = xx [ 316 ] +
xx [ 338 ] * xx [ 418 ] ; xx [ 623 ] = xx [ 615 ] - xx [ 338 ] * xx [ 209 ] ;
pm_math_Quaternion_inverseXform_ra ( xx + 595 , xx + 621 , xx + 624 ) ; xx [
574 ] = pm_math_Vector3_dot_ra ( xx + 562 , xx + 616 ) + xx [ 271 ] * xx [
624 ] + xx [ 277 ] * xx [ 625 ] ; xx [ 580 ] = xx [ 614 ] * xx [ 259 ] ; xx [
616 ] = xx [ 259 ] * xx [ 594 ] ; xx [ 617 ] = xx [ 592 ] + xx [ 574 ] ; xx [
621 ] = xx [ 594 ] + xx [ 6 ] * ( xx [ 216 ] * xx [ 580 ] - xx [ 616 ] * xx [
259 ] ) ; xx [ 622 ] = xx [ 614 ] - ( xx [ 216 ] * xx [ 616 ] + xx [ 580 ] *
xx [ 259 ] ) * xx [ 6 ] ; xx [ 623 ] = - xx [ 617 ] ; xx [ 580 ] = xx [ 624 ]
+ xx [ 617 ] * xx [ 262 ] ; xx [ 592 ] = xx [ 625 ] - xx [ 574 ] * xx [ 63 ]
- xx [ 617 ] * xx [ 273 ] ; xx [ 594 ] = xx [ 259 ] * xx [ 592 ] ; xx [ 614 ]
= xx [ 259 ] * xx [ 580 ] ; xx [ 616 ] = pm_math_Vector3_dot_ra ( xx + 584 ,
xx + 621 ) + ( xx [ 580 ] + xx [ 6 ] * ( xx [ 216 ] * xx [ 594 ] - xx [ 614 ]
* xx [ 259 ] ) ) * xx [ 257 ] + xx [ 248 ] * ( xx [ 592 ] - ( xx [ 216 ] * xx
[ 614 ] + xx [ 594 ] * xx [ 259 ] ) * xx [ 6 ] ) ; xx [ 580 ] = xx [ 561 ] +
xx [ 566 ] ; xx [ 561 ] = xx [ 620 ] - xx [ 63 ] * xx [ 566 ] - xx [ 580 ] *
xx [ 195 ] ; xx [ 592 ] = xx [ 186 ] * ( xx [ 561 ] - ( xx [ 162 ] * xx [ 184
] * ( xx [ 619 ] + xx [ 580 ] * xx [ 193 ] ) + xx [ 184 ] * xx [ 561 ] * xx [
184 ] ) * xx [ 6 ] ) - xx [ 580 ] * xx [ 201 ] ; xx [ 561 ] = xx [ 474 ] * xx
[ 566 ] - ( xx [ 574 ] * xx [ 422 ] + xx [ 616 ] * xx [ 384 ] ) + xx [ 582 ]
* xx [ 592 ] ; xx [ 580 ] = xx [ 568 ] * xx [ 425 ] - ( xx [ 452 ] * xx [ 401
] + xx [ 610 ] * xx [ 398 ] ) + xx [ 478 ] * xx [ 565 ] ; xx [ 425 ] = xx [
568 ] * xx [ 566 ] - ( xx [ 574 ] * xx [ 401 ] + xx [ 616 ] * xx [ 398 ] ) +
xx [ 478 ] * xx [ 592 ] ; xx [ 616 ] = xx [ 476 ] ; xx [ 617 ] = xx [ 48 ] ;
xx [ 618 ] = xx [ 113 ] ; xx [ 48 ] = xx [ 54 ] * xx [ 70 ] ; xx [ 113 ] = xx
[ 54 ] * xx [ 72 ] ; xx [ 452 ] = ( xx [ 48 ] * xx [ 70 ] + xx [ 113 ] * xx [
72 ] ) * xx [ 6 ] ; xx [ 476 ] = xx [ 452 ] - xx [ 54 ] ; xx [ 565 ] = xx [ 6
] * ( xx [ 113 ] * xx [ 70 ] - xx [ 48 ] * xx [ 72 ] ) ; xx [ 619 ] = xx [
476 ] ; xx [ 620 ] = xx [ 565 ] ; xx [ 621 ] = - xx [ 452 ] ; xx [ 622 ] = xx
[ 609 ] ; xx [ 623 ] = xx [ 283 ] + xx [ 54 ] * xx [ 341 ] ; xx [ 624 ] = xx
[ 611 ] - xx [ 54 ] * xx [ 74 ] ; pm_math_Quaternion_inverseXform_ra ( xx +
629 , xx + 622 , xx + 625 ) ; xx [ 48 ] = xx [ 336 ] - (
pm_math_Vector3_dot_ra ( xx + 616 , xx + 619 ) + xx [ 142 ] * xx [ 625 ] + xx
[ 148 ] * xx [ 626 ] ) ; xx [ 619 ] = - xx [ 334 ] ; xx [ 620 ] = xx [ 319 ]
; xx [ 621 ] = xx [ 147 ] ; xx [ 113 ] = xx [ 565 ] * xx [ 130 ] ; xx [ 147 ]
= xx [ 130 ] * xx [ 476 ] ; xx [ 319 ] = xx [ 48 ] - xx [ 452 ] ; xx [ 622 ]
= xx [ 476 ] + xx [ 6 ] * ( xx [ 81 ] * xx [ 113 ] - xx [ 147 ] * xx [ 130 ]
) ; xx [ 623 ] = xx [ 565 ] - ( xx [ 81 ] * xx [ 147 ] + xx [ 113 ] * xx [
130 ] ) * xx [ 6 ] ; xx [ 624 ] = xx [ 319 ] ; xx [ 113 ] = xx [ 625 ] - xx [
133 ] * xx [ 319 ] ; xx [ 147 ] = xx [ 626 ] + xx [ 63 ] * xx [ 48 ] + xx [
144 ] * xx [ 319 ] ; xx [ 319 ] = xx [ 147 ] * xx [ 130 ] ; xx [ 334 ] = xx [
113 ] * xx [ 130 ] ; xx [ 625 ] = xx [ 609 ] ; xx [ 626 ] = xx [ 283 ] + xx [
54 ] * xx [ 288 ] ; xx [ 627 ] = xx [ 611 ] - xx [ 54 ] * xx [ 285 ] ;
pm_math_Quaternion_inverseXform_ra ( xx + 436 , xx + 625 , xx + 609 ) ; xx [
283 ] = ( xx [ 54 ] * xx [ 4 ] * xx [ 4 ] + xx [ 54 ] * xx [ 9 ] * xx [ 9 ] )
* xx [ 6 ] ; xx [ 54 ] = xx [ 167 ] + xx [ 67 ] * xx [ 609 ] + xx [ 69 ] * xx
[ 610 ] - xx [ 432 ] * xx [ 283 ] ; xx [ 167 ] = xx [ 283 ] + xx [ 54 ] ; xx
[ 283 ] = xx [ 610 ] - xx [ 54 ] * xx [ 63 ] - xx [ 167 ] * xx [ 58 ] ; xx [
336 ] = xx [ 338 ] * xx [ 70 ] ; xx [ 452 ] = xx [ 338 ] * xx [ 72 ] ; xx [
476 ] = ( xx [ 336 ] * xx [ 70 ] + xx [ 452 ] * xx [ 72 ] ) * xx [ 6 ] ; xx [
565 ] = xx [ 476 ] - xx [ 338 ] ; xx [ 566 ] = xx [ 6 ] * ( xx [ 452 ] * xx [
70 ] - xx [ 336 ] * xx [ 72 ] ) ; xx [ 625 ] = xx [ 565 ] ; xx [ 626 ] = xx [
566 ] ; xx [ 627 ] = - xx [ 476 ] ; xx [ 633 ] = xx [ 613 ] ; xx [ 634 ] = xx
[ 316 ] + xx [ 338 ] * xx [ 341 ] ; xx [ 635 ] = xx [ 615 ] - xx [ 338 ] * xx
[ 74 ] ; pm_math_Quaternion_inverseXform_ra ( xx + 629 , xx + 633 , xx + 636
) ; xx [ 336 ] = xx [ 593 ] - ( pm_math_Vector3_dot_ra ( xx + 616 , xx + 625
) + xx [ 142 ] * xx [ 636 ] + xx [ 148 ] * xx [ 637 ] ) ; xx [ 452 ] = xx [
566 ] * xx [ 130 ] ; xx [ 574 ] = xx [ 130 ] * xx [ 565 ] ; xx [ 592 ] = xx [
336 ] - xx [ 476 ] ; xx [ 625 ] = xx [ 565 ] + xx [ 6 ] * ( xx [ 81 ] * xx [
452 ] - xx [ 574 ] * xx [ 130 ] ) ; xx [ 626 ] = xx [ 566 ] - ( xx [ 81 ] *
xx [ 574 ] + xx [ 452 ] * xx [ 130 ] ) * xx [ 6 ] ; xx [ 627 ] = xx [ 592 ] ;
xx [ 452 ] = xx [ 636 ] - xx [ 133 ] * xx [ 592 ] ; xx [ 476 ] = xx [ 637 ] +
xx [ 63 ] * xx [ 336 ] + xx [ 144 ] * xx [ 592 ] ; xx [ 565 ] = xx [ 476 ] *
xx [ 130 ] ; xx [ 566 ] = xx [ 452 ] * xx [ 130 ] ; xx [ 574 ] = xx [ 575 ] -
( pm_math_Vector3_dot_ra ( xx + 619 , xx + 625 ) + ( xx [ 452 ] + xx [ 6 ] *
( xx [ 81 ] * xx [ 565 ] - xx [ 566 ] * xx [ 130 ] ) ) * xx [ 128 ] + xx [
115 ] * ( xx [ 476 ] - ( xx [ 81 ] * xx [ 566 ] + xx [ 565 ] * xx [ 130 ] ) *
xx [ 6 ] ) ) ; xx [ 592 ] = xx [ 613 ] ; xx [ 593 ] = xx [ 316 ] + xx [ 338 ]
* xx [ 288 ] ; xx [ 594 ] = xx [ 615 ] - xx [ 338 ] * xx [ 285 ] ;
pm_math_Quaternion_inverseXform_ra ( xx + 436 , xx + 592 , xx + 613 ) ; xx [
316 ] = ( xx [ 338 ] * xx [ 4 ] * xx [ 4 ] + xx [ 338 ] * xx [ 9 ] * xx [ 9 ]
) * xx [ 6 ] ; xx [ 338 ] = xx [ 284 ] + xx [ 67 ] * xx [ 613 ] + xx [ 69 ] *
xx [ 614 ] - xx [ 432 ] * xx [ 316 ] ; xx [ 284 ] = xx [ 316 ] + xx [ 338 ] ;
xx [ 316 ] = xx [ 614 ] - xx [ 338 ] * xx [ 63 ] - xx [ 284 ] * xx [ 58 ] ;
xx [ 452 ] = xx [ 292 ] + xx [ 46 ] * ( xx [ 316 ] - ( xx [ 37 ] * xx [ 41 ]
* ( xx [ 613 ] + xx [ 284 ] * xx [ 56 ] ) + xx [ 41 ] * xx [ 316 ] * xx [ 41
] ) * xx [ 6 ] ) - xx [ 284 ] * xx [ 66 ] ; xx [ 284 ] = xx [ 570 ] * xx [
336 ] + xx [ 601 ] * xx [ 574 ] + xx [ 338 ] * xx [ 268 ] + xx [ 452 ] * xx [
361 ] ; xx [ 633 ] = xx [ 422 ] * xx [ 395 ] + xx [ 384 ] * ( xx [ 383 ] - (
pm_math_Vector3_dot_ra ( xx + 584 , xx + 602 ) + ( xx [ 276 ] + xx [ 6 ] * (
xx [ 216 ] * xx [ 404 ] - xx [ 412 ] * xx [ 259 ] ) ) * xx [ 257 ] + xx [ 248
] * ( xx [ 380 ] - ( xx [ 216 ] * xx [ 412 ] + xx [ 404 ] * xx [ 259 ] ) * xx
[ 6 ] ) ) ) + xx [ 337 ] * xx [ 474 ] + ( xx [ 583 ] + xx [ 186 ] * ( xx [
240 ] - ( xx [ 162 ] * xx [ 184 ] * ( xx [ 599 ] + xx [ 354 ] * xx [ 193 ] )
+ xx [ 184 ] * xx [ 240 ] * xx [ 184 ] ) * xx [ 6 ] ) - xx [ 354 ] * xx [ 201
] ) * xx [ 582 ] ; xx [ 634 ] = xx [ 355 ] ; xx [ 635 ] = xx [ 303 ] ; xx [
636 ] = xx [ 561 ] ; xx [ 637 ] = xx [ 355 ] ; xx [ 638 ] = xx [ 401 ] * xx [
449 ] + xx [ 567 ] * xx [ 398 ] + xx [ 413 ] * xx [ 568 ] + xx [ 416 ] * xx [
478 ] ; xx [ 639 ] = xx [ 580 ] ; xx [ 640 ] = xx [ 425 ] ; xx [ 641 ] = xx [
303 ] ; xx [ 642 ] = xx [ 580 ] ; xx [ 643 ] = xx [ 570 ] * xx [ 48 ] + xx [
601 ] * ( xx [ 612 ] - ( pm_math_Vector3_dot_ra ( xx + 619 , xx + 622 ) + (
xx [ 113 ] + xx [ 6 ] * ( xx [ 81 ] * xx [ 319 ] - xx [ 334 ] * xx [ 130 ] )
) * xx [ 128 ] + xx [ 115 ] * ( xx [ 147 ] - ( xx [ 81 ] * xx [ 334 ] + xx [
319 ] * xx [ 130 ] ) * xx [ 6 ] ) ) ) + xx [ 54 ] * xx [ 268 ] + ( xx [ 374 ]
+ xx [ 46 ] * ( xx [ 283 ] - ( xx [ 37 ] * xx [ 41 ] * ( xx [ 609 ] + xx [
167 ] * xx [ 56 ] ) + xx [ 41 ] * xx [ 283 ] * xx [ 41 ] ) * xx [ 6 ] ) - xx
[ 167 ] * xx [ 66 ] ) * xx [ 361 ] ; xx [ 644 ] = xx [ 284 ] ; xx [ 645 ] =
xx [ 561 ] ; xx [ 646 ] = xx [ 425 ] ; xx [ 647 ] = xx [ 284 ] ; xx [ 648 ] =
xx [ 588 ] * xx [ 336 ] + xx [ 574 ] * xx [ 569 ] + xx [ 338 ] * xx [ 560 ] +
xx [ 452 ] * xx [ 226 ] ; xx [ 48 ] = xx [ 205 ] * state [ 7 ] ; xx [ 54 ] =
xx [ 207 ] * state [ 7 ] ; xx [ 113 ] = ( xx [ 205 ] * xx [ 48 ] + xx [ 207 ]
* xx [ 54 ] ) * xx [ 6 ] ; xx [ 147 ] = state [ 7 ] - xx [ 113 ] ; xx [ 167 ]
= xx [ 6 ] * ( xx [ 207 ] * xx [ 48 ] - xx [ 205 ] * xx [ 54 ] ) ; xx [ 48 ]
= xx [ 167 ] * xx [ 259 ] ; xx [ 54 ] = xx [ 259 ] * xx [ 147 ] ; xx [ 240 ]
= xx [ 147 ] + xx [ 6 ] * ( xx [ 216 ] * xx [ 48 ] - xx [ 54 ] * xx [ 259 ] )
; xx [ 276 ] = xx [ 167 ] - ( xx [ 216 ] * xx [ 54 ] + xx [ 48 ] * xx [ 259 ]
) * xx [ 6 ] ; xx [ 48 ] = xx [ 113 ] + state [ 9 ] ; xx [ 54 ] = xx [ 48 ] +
state [ 11 ] ; xx [ 336 ] = xx [ 240 ] ; xx [ 337 ] = xx [ 276 ] ; xx [ 338 ]
= xx [ 54 ] ; xx [ 565 ] = xx [ 240 ] * xx [ 19 ] ; xx [ 566 ] = xx [ 104 ] *
xx [ 276 ] ; xx [ 567 ] = xx [ 54 ] * xx [ 118 ] ; pm_math_Vector3_cross_ra (
xx + 336 , xx + 565 , xx + 592 ) ; xx [ 283 ] = xx [ 243 ] * xx [ 276 ] ; xx
[ 284 ] = xx [ 240 ] * xx [ 243 ] ; xx [ 292 ] = xx [ 240 ] + xx [ 6 ] * ( xx
[ 238 ] * xx [ 283 ] - xx [ 284 ] * xx [ 243 ] ) ; xx [ 303 ] = xx [ 276 ] -
( xx [ 238 ] * xx [ 284 ] + xx [ 283 ] * xx [ 243 ] ) * xx [ 6 ] ; xx [ 283 ]
= xx [ 54 ] + state [ 13 ] ; xx [ 565 ] = xx [ 292 ] ; xx [ 566 ] = xx [ 303
] ; xx [ 567 ] = xx [ 283 ] ; xx [ 602 ] = xx [ 292 ] * xx [ 21 ] ; xx [ 603
] = xx [ 21 ] * xx [ 303 ] ; xx [ 604 ] = xx [ 283 ] * xx [ 119 ] ;
pm_math_Vector3_cross_ra ( xx + 565 , xx + 602 , xx + 609 ) ; xx [ 284 ] =
0.0 ; xx [ 622 ] = xx [ 7 ] ; xx [ 623 ] = xx [ 284 ] ; xx [ 624 ] = xx [ 284
] ; xx [ 625 ] = xx [ 284 ] ; xx [ 626 ] = xx [ 284 ] ; xx [ 627 ] = xx [ 284
] ; xx [ 628 ] = xx [ 284 ] ; xx [ 316 ] = xx [ 0 ] * xx [ 3 ] ; xx [ 319 ] =
xx [ 0 ] * xx [ 5 ] ; xx [ 612 ] = xx [ 316 ] + xx [ 316 ] ; xx [ 613 ] = xx
[ 319 ] + xx [ 319 ] ; xx [ 614 ] = xx [ 316 ] - xx [ 316 ] ; xx [ 615 ] = -
( xx [ 319 ] - xx [ 319 ] ) ; pm_math_Quaternion_compose_ra ( xx + 612 , xx +
595 , xx + 649 ) ; xx [ 316 ] = xx [ 652 ] * xx [ 259 ] - xx [ 216 ] * xx [
649 ] ; xx [ 319 ] = xx [ 649 ] * xx [ 259 ] + xx [ 216 ] * xx [ 652 ] ; xx [
334 ] = xx [ 216 ] * xx [ 650 ] + xx [ 651 ] * xx [ 259 ] ; xx [ 354 ] = xx [
650 ] * xx [ 259 ] - xx [ 216 ] * xx [ 651 ] ; xx [ 565 ] = - xx [ 334 ] ; xx
[ 566 ] = xx [ 354 ] ; xx [ 567 ] = - xx [ 319 ] ; xx [ 355 ] = xx [ 352 ] *
xx [ 354 ] ; xx [ 374 ] = xx [ 334 ] * xx [ 352 ] + xx [ 319 ] * xx [ 55 ] ;
xx [ 380 ] = xx [ 55 ] * xx [ 354 ] ; xx [ 602 ] = - xx [ 355 ] ; xx [ 603 ]
= - xx [ 374 ] ; xx [ 604 ] = - xx [ 380 ] ; pm_math_Vector3_cross_ra ( xx +
565 , xx + 602 , xx + 653 ) ; xx [ 383 ] = xx [ 652 ] * xx [ 262 ] ; xx [ 395
] = xx [ 273 ] * xx [ 652 ] ; xx [ 404 ] = xx [ 273 ] * xx [ 651 ] - xx [ 650
] * xx [ 262 ] ; xx [ 565 ] = - xx [ 383 ] ; xx [ 566 ] = xx [ 395 ] ; xx [
567 ] = - xx [ 404 ] ; pm_math_Vector3_cross_ra ( xx + 650 , xx + 565 , xx +
602 ) ; pm_math_Quaternion_xform_ra ( xx + 612 , xx + 426 , xx + 565 ) ; xx [
412 ] = xx [ 286 ] * xx [ 5 ] ; xx [ 413 ] = xx [ 412 ] * xx [ 5 ] ; xx [ 416
] = xx [ 286 ] * xx [ 3 ] ; xx [ 425 ] = xx [ 416 ] * xx [ 3 ] ; xx [ 449 ] =
xx [ 6 ] * ( xx [ 413 ] - xx [ 425 ] ) ; xx [ 452 ] = xx [ 0 ] * xx [ 0 ] *
xx [ 449 ] ; xx [ 476 ] = xx [ 286 ] - ( xx [ 413 ] + xx [ 425 ] ) * xx [ 6 ]
; xx [ 286 ] = xx [ 0 ] * xx [ 0 ] * xx [ 476 ] ; xx [ 0 ] = xx [ 6 ] * ( xx
[ 452 ] - xx [ 286 ] ) - xx [ 449 ] + state [ 2 ] ; xx [ 413 ] = ( xx [ 412 ]
* xx [ 3 ] + xx [ 416 ] * xx [ 5 ] ) * xx [ 6 ] ; xx [ 3 ] = xx [ 413 ] +
0.02 + state [ 1 ] ; xx [ 5 ] = ( xx [ 452 ] + xx [ 286 ] ) * xx [ 6 ] - ( xx
[ 476 ] + state [ 0 ] ) + 0.4 ; xx [ 656 ] = - ( xx [ 238 ] * xx [ 316 ] + xx
[ 319 ] * xx [ 243 ] ) ; xx [ 657 ] = xx [ 334 ] * xx [ 238 ] - xx [ 243 ] *
xx [ 354 ] ; xx [ 658 ] = - ( xx [ 238 ] * xx [ 354 ] + xx [ 334 ] * xx [ 243
] ) ; xx [ 659 ] = xx [ 319 ] * xx [ 238 ] - xx [ 243 ] * xx [ 316 ] ; xx [
660 ] = xx [ 55 ] + ( xx [ 653 ] - xx [ 355 ] * xx [ 316 ] ) * xx [ 6 ] + xx
[ 273 ] + ( xx [ 602 ] - xx [ 649 ] * xx [ 383 ] ) * xx [ 6 ] + xx [ 565 ] +
xx [ 0 ] ; xx [ 661 ] = ( xx [ 654 ] - xx [ 316 ] * xx [ 374 ] ) * xx [ 6 ] +
( xx [ 649 ] * xx [ 395 ] + xx [ 603 ] ) * xx [ 6 ] + xx [ 262 ] + xx [ 566 ]
+ xx [ 3 ] ; xx [ 662 ] = xx [ 6 ] * ( xx [ 655 ] - xx [ 380 ] * xx [ 316 ] )
- xx [ 352 ] + xx [ 6 ] * ( xx [ 604 ] - xx [ 404 ] * xx [ 649 ] ) + xx [ 567
] + xx [ 5 ] ; xx [ 286 ] =
sm_core_compiler_computeProximityInfoBrickCylinder (
wheel_leg_d9e82f16_1_geometry_5 ( rtdv ) , wheel_leg_d9e82f16_1_geometry_2 (
rtdv ) , ( pm_math_Transform3 * ) ( xx + 622 ) , ( pm_math_Transform3 * ) (
xx + 656 ) , ( pm_math_Vector3 * ) ( xx + 565 ) , ( pm_math_Vector3 * ) ( xx
+ 602 ) , ( pm_math_Vector3 * ) ( xx + 649 ) , ( pm_math_Vector3 * ) ( xx +
652 ) ) ; xx [ 663 ] = state [ 3 ] ; xx [ 664 ] = state [ 4 ] ; xx [ 665 ] =
state [ 5 ] ; pm_math_Quaternion_inverseXform_ra ( xx + 294 , xx + 663 , xx +
666 ) ; xx [ 316 ] = xx [ 84 ] * state [ 7 ] ; xx [ 319 ] = xx [ 667 ] + xx [
316 ] ; xx [ 334 ] = xx [ 418 ] * state [ 7 ] ; xx [ 354 ] = xx [ 209 ] *
state [ 7 ] ; xx [ 663 ] = xx [ 666 ] ; xx [ 664 ] = xx [ 319 ] - xx [ 334 ]
; xx [ 665 ] = xx [ 354 ] + xx [ 668 ] ; pm_math_Quaternion_inverseXform_ra (
xx + 595 , xx + 663 , xx + 669 ) ; xx [ 355 ] = xx [ 48 ] * xx [ 262 ] ; xx [
374 ] = xx [ 669 ] - xx [ 355 ] ; xx [ 380 ] = xx [ 48 ] * xx [ 273 ] ; xx [
383 ] = xx [ 63 ] * state [ 9 ] ; xx [ 395 ] = xx [ 380 ] + xx [ 670 ] + xx [
383 ] ; xx [ 404 ] = xx [ 395 ] * xx [ 259 ] ; xx [ 412 ] = xx [ 374 ] * xx [
259 ] ; xx [ 416 ] = xx [ 352 ] * xx [ 276 ] ; xx [ 425 ] = xx [ 374 ] + xx [
6 ] * ( xx [ 216 ] * xx [ 404 ] - xx [ 412 ] * xx [ 259 ] ) - xx [ 416 ] ; xx
[ 374 ] = xx [ 54 ] * xx [ 55 ] + xx [ 240 ] * xx [ 352 ] ; xx [ 452 ] = xx [
55 ] * state [ 11 ] ; xx [ 561 ] = xx [ 374 ] + xx [ 395 ] - ( xx [ 216 ] *
xx [ 412 ] + xx [ 404 ] * xx [ 259 ] ) * xx [ 6 ] + xx [ 452 ] ; xx [ 395 ] =
xx [ 561 ] * xx [ 243 ] ; xx [ 404 ] = xx [ 425 ] * xx [ 243 ] ; xx [ 412 ] =
xx [ 273 ] * xx [ 167 ] - xx [ 262 ] * xx [ 147 ] ; xx [ 574 ] = xx [ 55 ] *
xx [ 276 ] ; xx [ 672 ] = xx [ 292 ] ; xx [ 673 ] = xx [ 303 ] ; xx [ 674 ] =
xx [ 283 ] ; xx [ 675 ] = xx [ 425 ] + xx [ 6 ] * ( xx [ 238 ] * xx [ 395 ] -
xx [ 404 ] * xx [ 243 ] ) ; xx [ 676 ] = xx [ 561 ] - ( xx [ 238 ] * xx [ 404
] + xx [ 395 ] * xx [ 243 ] ) * xx [ 6 ] ; xx [ 677 ] = xx [ 671 ] - xx [ 412
] - xx [ 574 ] ; xx [ 283 ] = 1.0e6 ; xx [ 395 ] = 1000.0 ; xx [ 404 ] =
1.0e-4 ; xx [ 425 ] = 0.3 ; xx [ 561 ] = 0.2119573811760597 ; xx [ 575 ] =
9.126024771145405e-4 ; sm_core_compiler_computeContactWrenches ( xx [ 286 ] ,
xx + 565 , xx + 602 , xx + 649 , xx + 652 , xx + 622 , xx + 622 , xx + 622 ,
xx + 656 , NULL , xx + 672 , 0 , 1 , xx [ 283 ] , xx [ 395 ] , xx [ 404 ] ,
xx [ 425 ] , xx [ 561 ] , xx [ 575 ] , NULL , xx + 678 ) ; xx [ 286 ] = - xx
[ 1 ] ; xx [ 565 ] = 5.0e-3 ; xx [ 649 ] = xx [ 1 ] ; xx [ 650 ] = xx [ 1 ] ;
xx [ 651 ] = xx [ 286 ] ; xx [ 652 ] = xx [ 286 ] ; xx [ 653 ] = xx [ 284 ] ;
xx [ 654 ] = 1.8 ; xx [ 655 ] = xx [ 565 ] ; xx [ 566 ] =
sm_core_compiler_computeProximityInfoCxpolyCylinder (
wheel_leg_d9e82f16_1_geometry_1 ( rtdv ) , wheel_leg_d9e82f16_1_geometry_2 (
rtdv ) , ( pm_math_Transform3 * ) ( xx + 649 ) , ( pm_math_Transform3 * ) (
xx + 656 ) , ( pm_math_Vector3 * ) ( xx + 602 ) , ( pm_math_Vector3 * ) ( xx
+ 663 ) , ( pm_math_Vector3 * ) ( xx + 669 ) , ( pm_math_Vector3 * ) ( xx +
684 ) ) ; sm_core_compiler_computeContactWrenches ( xx [ 566 ] , xx + 602 ,
xx + 663 , xx + 669 , xx + 684 , xx + 649 , xx + 622 , xx + 649 , xx + 656 ,
NULL , xx + 672 , 0 , 1 , xx [ 283 ] , xx [ 395 ] , xx [ 404 ] , xx [ 425 ] ,
xx [ 561 ] , xx [ 575 ] , NULL , xx + 687 ) ; xx [ 649 ] = xx [ 1 ] ; xx [
650 ] = xx [ 1 ] ; xx [ 651 ] = xx [ 286 ] ; xx [ 652 ] = xx [ 286 ] ; xx [
653 ] = xx [ 284 ] ; xx [ 654 ] = 2.2 ; xx [ 655 ] = xx [ 565 ] ; xx [ 566 ]
= sm_core_compiler_computeProximityInfoCxpolyCylinder (
wheel_leg_d9e82f16_1_geometry_1 ( rtdv ) , wheel_leg_d9e82f16_1_geometry_2 (
rtdv ) , ( pm_math_Transform3 * ) ( xx + 649 ) , ( pm_math_Transform3 * ) (
xx + 656 ) , ( pm_math_Vector3 * ) ( xx + 602 ) , ( pm_math_Vector3 * ) ( xx
+ 663 ) , ( pm_math_Vector3 * ) ( xx + 669 ) , ( pm_math_Vector3 * ) ( xx +
684 ) ) ; sm_core_compiler_computeContactWrenches ( xx [ 566 ] , xx + 602 ,
xx + 663 , xx + 669 , xx + 684 , xx + 649 , xx + 622 , xx + 649 , xx + 656 ,
NULL , xx + 672 , 0 , 1 , xx [ 283 ] , xx [ 395 ] , xx [ 404 ] , xx [ 425 ] ,
xx [ 561 ] , xx [ 575 ] , NULL , xx + 693 ) ; xx [ 649 ] = xx [ 1 ] ; xx [
650 ] = xx [ 1 ] ; xx [ 651 ] = xx [ 286 ] ; xx [ 652 ] = xx [ 286 ] ; xx [
653 ] = xx [ 284 ] ; xx [ 654 ] = 2.6 ; xx [ 655 ] = xx [ 565 ] ; xx [ 566 ]
= sm_core_compiler_computeProximityInfoCxpolyCylinder (
wheel_leg_d9e82f16_1_geometry_1 ( rtdv ) , wheel_leg_d9e82f16_1_geometry_2 (
rtdv ) , ( pm_math_Transform3 * ) ( xx + 649 ) , ( pm_math_Transform3 * ) (
xx + 656 ) , ( pm_math_Vector3 * ) ( xx + 602 ) , ( pm_math_Vector3 * ) ( xx
+ 663 ) , ( pm_math_Vector3 * ) ( xx + 669 ) , ( pm_math_Vector3 * ) ( xx +
684 ) ) ; sm_core_compiler_computeContactWrenches ( xx [ 566 ] , xx + 602 ,
xx + 663 , xx + 669 , xx + 684 , xx + 649 , xx + 622 , xx + 649 , xx + 656 ,
NULL , xx + 672 , 0 , 1 , xx [ 283 ] , xx [ 395 ] , xx [ 404 ] , xx [ 425 ] ,
xx [ 561 ] , xx [ 575 ] , NULL , xx + 699 ) ; xx [ 566 ] = 3.0 ; xx [ 649 ] =
xx [ 1 ] ; xx [ 650 ] = xx [ 1 ] ; xx [ 651 ] = xx [ 286 ] ; xx [ 652 ] = xx
[ 286 ] ; xx [ 653 ] = xx [ 284 ] ; xx [ 654 ] = xx [ 566 ] ; xx [ 655 ] = xx
[ 565 ] ; xx [ 565 ] = sm_core_compiler_computeProximityInfoCxpolyCylinder (
wheel_leg_d9e82f16_1_geometry_1 ( rtdv ) , wheel_leg_d9e82f16_1_geometry_2 (
rtdv ) , ( pm_math_Transform3 * ) ( xx + 649 ) , ( pm_math_Transform3 * ) (
xx + 656 ) , ( pm_math_Vector3 * ) ( xx + 602 ) , ( pm_math_Vector3 * ) ( xx
+ 663 ) , ( pm_math_Vector3 * ) ( xx + 669 ) , ( pm_math_Vector3 * ) ( xx +
684 ) ) ; sm_core_compiler_computeContactWrenches ( xx [ 565 ] , xx + 602 ,
xx + 663 , xx + 669 , xx + 684 , xx + 649 , xx + 622 , xx + 649 , xx + 656 ,
NULL , xx + 672 , 0 , 1 , xx [ 283 ] , xx [ 395 ] , xx [ 404 ] , xx [ 425 ] ,
xx [ 561 ] , xx [ 575 ] , NULL , xx + 705 ) ; xx [ 649 ] = xx [ 1 ] ; xx [
650 ] = xx [ 1 ] ; xx [ 651 ] = xx [ 286 ] ; xx [ 652 ] = xx [ 286 ] ; xx [
653 ] = xx [ 284 ] ; xx [ 654 ] = 9.0 ; xx [ 655 ] = xx [ 284 ] ; xx [ 565 ]
= sm_core_compiler_computeProximityInfoCxpolyCylinder (
wheel_leg_d9e82f16_1_geometry_3 ( rtdv ) , wheel_leg_d9e82f16_1_geometry_2 (
rtdv ) , ( pm_math_Transform3 * ) ( xx + 649 ) , ( pm_math_Transform3 * ) (
xx + 656 ) , ( pm_math_Vector3 * ) ( xx + 602 ) , ( pm_math_Vector3 * ) ( xx
+ 663 ) , ( pm_math_Vector3 * ) ( xx + 669 ) , ( pm_math_Vector3 * ) ( xx +
684 ) ) ; sm_core_compiler_computeContactWrenches ( xx [ 565 ] , xx + 602 ,
xx + 663 , xx + 669 , xx + 684 , xx + 649 , xx + 622 , xx + 649 , xx + 656 ,
NULL , xx + 672 , 0 , 1 , xx [ 283 ] , xx [ 395 ] , xx [ 404 ] , xx [ 425 ] ,
xx [ 561 ] , xx [ 575 ] , NULL , xx + 711 ) ; xx [ 717 ] = - xx [ 7 ] ; xx [
718 ] = xx [ 284 ] ; xx [ 719 ] = xx [ 284 ] ; xx [ 720 ] = xx [ 284 ] ; xx [
721 ] = xx [ 284 ] ; xx [ 722 ] = 7.2 ; xx [ 723 ] = 0.1 ; xx [ 565 ] =
sm_core_compiler_computeProximityInfoBrickCylinder (
wheel_leg_d9e82f16_1_geometry_0 ( rtdv ) , wheel_leg_d9e82f16_1_geometry_2 (
rtdv ) , ( pm_math_Transform3 * ) ( xx + 717 ) , ( pm_math_Transform3 * ) (
xx + 656 ) , ( pm_math_Vector3 * ) ( xx + 602 ) , ( pm_math_Vector3 * ) ( xx
+ 663 ) , ( pm_math_Vector3 * ) ( xx + 669 ) , ( pm_math_Vector3 * ) ( xx +
684 ) ) ; sm_core_compiler_computeContactWrenches ( xx [ 565 ] , xx + 602 ,
xx + 663 , xx + 669 , xx + 684 , xx + 717 , xx + 622 , xx + 717 , xx + 656 ,
NULL , xx + 672 , 0 , 1 , xx [ 283 ] , xx [ 395 ] , xx [ 404 ] , xx [ 425 ] ,
xx [ 561 ] , xx [ 575 ] , NULL , xx + 724 ) ; xx [ 730 ] = xx [ 1 ] ; xx [
731 ] = xx [ 1 ] ; xx [ 732 ] = xx [ 286 ] ; xx [ 733 ] = xx [ 286 ] ; xx [
734 ] = 0.25 ; xx [ 735 ] = xx [ 566 ] ; xx [ 736 ] = xx [ 284 ] ; xx [ 1 ] =
sm_core_compiler_computeProximityInfoCxpolyCylinder (
wheel_leg_d9e82f16_1_geometry_4 ( rtdv ) , wheel_leg_d9e82f16_1_geometry_2 (
rtdv ) , ( pm_math_Transform3 * ) ( xx + 730 ) , ( pm_math_Transform3 * ) (
xx + 656 ) , ( pm_math_Vector3 * ) ( xx + 602 ) , ( pm_math_Vector3 * ) ( xx
+ 663 ) , ( pm_math_Vector3 * ) ( xx + 669 ) , ( pm_math_Vector3 * ) ( xx +
684 ) ) ; sm_core_compiler_computeContactWrenches ( xx [ 1 ] , xx + 602 , xx
+ 663 , xx + 669 , xx + 684 , xx + 730 , xx + 622 , xx + 730 , xx + 656 ,
NULL , xx + 672 , 0 , 1 , xx [ 283 ] , xx [ 395 ] , xx [ 404 ] , xx [ 425 ] ,
xx [ 561 ] , xx [ 575 ] , NULL , xx + 737 ) ; xx [ 1 ] = xx [ 611 ] - xx [
680 ] - xx [ 689 ] - xx [ 695 ] - xx [ 701 ] - xx [ 707 ] - xx [ 713 ] - xx [
726 ] - xx [ 739 ] ; xx [ 286 ] = ( input [ 0 ] - xx [ 1 ] ) / xx [ 120 ] ;
xx [ 602 ] = - xx [ 416 ] ; xx [ 603 ] = xx [ 374 ] ; xx [ 604 ] = - xx [ 574
] ; pm_math_Vector3_cross_ra ( xx + 336 , xx + 602 , xx + 656 ) ; xx [ 120 ]
= xx [ 656 ] * xx [ 243 ] ; xx [ 336 ] = xx [ 657 ] * xx [ 243 ] ; xx [ 337 ]
= xx [ 103 ] * ( xx [ 657 ] - ( xx [ 238 ] * xx [ 120 ] + xx [ 336 ] * xx [
243 ] ) * xx [ 6 ] ) - ( xx [ 682 ] + xx [ 691 ] + xx [ 697 ] + xx [ 703 ] +
xx [ 709 ] + xx [ 715 ] + xx [ 728 ] + xx [ 741 ] ) ; xx [ 338 ] = ( xx [ 656
] + xx [ 6 ] * ( xx [ 238 ] * xx [ 336 ] - xx [ 120 ] * xx [ 243 ] ) ) * xx [
103 ] - ( xx [ 681 ] + xx [ 690 ] + xx [ 696 ] + xx [ 702 ] + xx [ 708 ] + xx
[ 714 ] + xx [ 727 ] + xx [ 740 ] ) ; xx [ 120 ] = xx [ 243 ] * xx [ 338 ] ;
xx [ 336 ] = xx [ 243 ] * xx [ 337 ] ; xx [ 374 ] = xx [ 337 ] + xx [ 6 ] * (
xx [ 238 ] * xx [ 120 ] - xx [ 336 ] * xx [ 243 ] ) ; xx [ 602 ] = xx [ 147 ]
; xx [ 603 ] = xx [ 167 ] ; xx [ 604 ] = xx [ 48 ] ; xx [ 659 ] = - xx [ 355
] ; xx [ 660 ] = xx [ 380 ] ; xx [ 661 ] = - xx [ 412 ] ;
pm_math_Vector3_cross_ra ( xx + 602 , xx + 659 , xx + 662 ) ; xx [ 337 ] = xx
[ 663 ] * xx [ 259 ] ; xx [ 355 ] = xx [ 662 ] * xx [ 259 ] ; xx [ 380 ] = xx
[ 662 ] + xx [ 6 ] * ( xx [ 216 ] * xx [ 337 ] - xx [ 355 ] * xx [ 259 ] ) -
( xx [ 48 ] + xx [ 54 ] ) * xx [ 452 ] ; xx [ 54 ] = xx [ 663 ] - ( xx [ 216
] * xx [ 355 ] + xx [ 337 ] * xx [ 259 ] ) * xx [ 6 ] ; xx [ 337 ] = xx [ 276
] * state [ 11 ] ; xx [ 276 ] = xx [ 240 ] * state [ 11 ] ; xx [ 355 ] = xx [
594 ] + xx [ 1 ] + xx [ 119 ] * xx [ 286 ] + xx [ 374 ] * xx [ 55 ] + xx [
249 ] * xx [ 380 ] + xx [ 242 ] * xx [ 54 ] + xx [ 239 ] * xx [ 337 ] + xx [
276 ] * xx [ 406 ] ; xx [ 1 ] = xx [ 374 ] + xx [ 380 ] * xx [ 264 ] + xx [
245 ] * xx [ 54 ] + xx [ 381 ] * xx [ 337 ] + xx [ 276 ] * xx [ 405 ] ; xx [
239 ] = xx [ 355 ] + xx [ 1 ] * xx [ 55 ] ; xx [ 242 ] = xx [ 239 ] / xx [
256 ] ; xx [ 659 ] = xx [ 219 ] * xx [ 89 ] ; xx [ 660 ] = xx [ 419 ] * xx [
164 ] ; xx [ 661 ] = xx [ 83 ] * xx [ 65 ] ; pm_math_Vector3_cross_ra ( xx +
170 , xx + 659 , xx + 669 ) ; xx [ 170 ] = xx [ 224 ] * state [ 27 ] ; xx [
171 ] = xx [ 556 ] + xx [ 293 ] * xx [ 170 ] ; xx [ 172 ] = xx [ 223 ] *
state [ 27 ] ; xx [ 224 ] = xx [ 557 ] - xx [ 429 ] * xx [ 172 ] ; xx [ 245 ]
= xx [ 41 ] * xx [ 224 ] ; xx [ 249 ] = xx [ 171 ] * xx [ 41 ] ; xx [ 264 ] =
( ( xx [ 223 ] + xx [ 223 ] ) * xx [ 211 ] + xx [ 388 ] ) * xx [ 35 ] ; xx [
211 ] = xx [ 164 ] * state [ 25 ] ; xx [ 164 ] = xx [ 89 ] * state [ 25 ] ;
xx [ 223 ] = xx [ 63 ] * state [ 25 ] ; xx [ 386 ] = xx [ 285 ] * state [ 7 ]
* state [ 7 ] ; xx [ 387 ] = xx [ 9 ] * xx [ 386 ] ; xx [ 388 ] = xx [ 288 ]
* state [ 7 ] * state [ 7 ] ; xx [ 406 ] = xx [ 387 ] - xx [ 4 ] * xx [ 388 ]
; xx [ 412 ] = xx [ 9 ] * xx [ 388 ] ; xx [ 659 ] = xx [ 406 ] ; xx [ 660 ] =
xx [ 412 ] ; xx [ 661 ] = - xx [ 387 ] ; pm_math_Vector3_cross_ra ( xx + 298
, xx + 659 , xx + 672 ) ; xx [ 416 ] = ( xx [ 89 ] + xx [ 89 ] ) * xx [ 223 ]
+ ( xx [ 4 ] * xx [ 387 ] + xx [ 674 ] ) * xx [ 6 ] - xx [ 388 ] ; xx [ 89 ]
= xx [ 669 ] + xx [ 171 ] - ( xx [ 37 ] * xx [ 245 ] + xx [ 249 ] * xx [ 41 ]
) * xx [ 6 ] + xx [ 264 ] * xx [ 56 ] + xx [ 431 ] * xx [ 211 ] - xx [ 434 ]
* xx [ 164 ] + xx [ 416 ] * xx [ 8 ] ; xx [ 171 ] = xx [ 670 ] + xx [ 224 ] +
xx [ 6 ] * ( xx [ 37 ] * xx [ 249 ] - xx [ 245 ] * xx [ 41 ] ) - xx [ 58 ] *
xx [ 264 ] + xx [ 435 ] * xx [ 211 ] - xx [ 430 ] * xx [ 164 ] - xx [ 416 ] *
xx [ 274 ] ; xx [ 224 ] = xx [ 86 ] / xx [ 45 ] ; xx [ 245 ] = xx [ 221 ] -
xx [ 43 ] * xx [ 224 ] ; xx [ 249 ] = xx [ 218 ] * xx [ 41 ] ; xx [ 387 ] =
xx [ 37 ] * xx [ 249 ] ; xx [ 388 ] = xx [ 41 ] * xx [ 245 ] ; xx [ 430 ] =
xx [ 245 ] + xx [ 6 ] * ( xx [ 387 ] - xx [ 388 ] * xx [ 41 ] ) ; xx [ 245 ]
= xx [ 249 ] * xx [ 41 ] ; xx [ 249 ] = xx [ 218 ] - ( xx [ 37 ] * xx [ 388 ]
+ xx [ 245 ] ) * xx [ 6 ] ; xx [ 388 ] = xx [ 6 ] * ( xx [ 672 ] - xx [ 4 ] *
xx [ 406 ] ) - ( xx [ 88 ] + xx [ 83 ] ) * xx [ 223 ] ; xx [ 83 ] = xx [ 6 ]
* ( xx [ 673 ] - xx [ 4 ] * xx [ 412 ] ) - xx [ 386 ] ; xx [ 88 ] = xx [ 49 ]
* xx [ 388 ] + xx [ 52 ] * xx [ 83 ] ; xx [ 49 ] = xx [ 671 ] + xx [ 558 ] -
xx [ 51 ] * xx [ 224 ] + xx [ 58 ] * xx [ 430 ] - xx [ 56 ] * xx [ 249 ] + xx
[ 88 ] ; xx [ 52 ] = 1.850049007113989 ; xx [ 223 ] = state [ 24 ] + xx [ 52
] ; if ( xx [ 284 ] < xx [ 223 ] ) xx [ 223 ] = xx [ 284 ] ; xx [ 386 ] =
1.74532925199433e-3 ; xx [ 406 ] = - ( xx [ 223 ] / xx [ 386 ] ) ; if ( xx [
7 ] < xx [ 406 ] ) xx [ 406 ] = xx [ 7 ] ; xx [ 412 ] = 5729.577951308233 ;
xx [ 431 ] = xx [ 412 ] * state [ 25 ] ; xx [ 434 ] = 5.729577951308233e6 ;
xx [ 435 ] = xx [ 406 ] * xx [ 406 ] * ( xx [ 566 ] - xx [ 6 ] * xx [ 406 ] )
* ( ( - xx [ 223 ] == xx [ 284 ] ? xx [ 284 ] : - xx [ 431 ] ) - xx [ 434 ] *
xx [ 223 ] ) ; if ( xx [ 284 ] > xx [ 435 ] ) xx [ 435 ] = xx [ 284 ] ; xx [
223 ] = 5.729577951308233e5 ; xx [ 406 ] = state [ 24 ] ; if ( xx [ 284 ] >
xx [ 406 ] ) xx [ 406 ] = xx [ 284 ] ; xx [ 556 ] = xx [ 406 ] / xx [ 386 ] ;
if ( xx [ 7 ] < xx [ 556 ] ) xx [ 556 ] = xx [ 7 ] ; xx [ 557 ] = ( xx [ 223
] * xx [ 406 ] + ( xx [ 406 ] == xx [ 284 ] ? xx [ 284 ] : xx [ 431 ] ) ) *
xx [ 556 ] * xx [ 556 ] * ( xx [ 566 ] - xx [ 6 ] * xx [ 556 ] ) ; if ( xx [
284 ] > xx [ 557 ] ) xx [ 557 ] = xx [ 284 ] ; xx [ 406 ] = input [ 5 ] + xx
[ 435 ] - xx [ 557 ] ; xx [ 431 ] = xx [ 388 ] * xx [ 59 ] + xx [ 60 ] * xx [
83 ] ; xx [ 59 ] = xx [ 430 ] + xx [ 431 ] ; xx [ 60 ] = ( xx [ 406 ] - ( xx
[ 49 ] + xx [ 59 ] * xx [ 63 ] ) ) / xx [ 62 ] ; xx [ 659 ] = xx [ 89 ] ; xx
[ 660 ] = xx [ 171 ] ; xx [ 661 ] = xx [ 49 ] + xx [ 40 ] * xx [ 60 ] ;
pm_math_Quaternion_xform_ra ( xx + 436 , xx + 659 , xx + 672 ) ; xx [ 49 ] =
xx [ 50 ] * xx [ 388 ] + xx [ 83 ] * xx [ 47 ] ; xx [ 47 ] = xx [ 264 ] + xx
[ 274 ] * xx [ 164 ] + xx [ 8 ] * xx [ 211 ] + xx [ 53 ] * xx [ 416 ] ; xx [
659 ] = xx [ 249 ] + xx [ 49 ] + xx [ 64 ] * xx [ 60 ] ; xx [ 660 ] = xx [ 59
] + xx [ 38 ] * xx [ 60 ] ; xx [ 661 ] = xx [ 47 ] ;
pm_math_Quaternion_xform_ra ( xx + 436 , xx + 659 , xx + 673 ) ;
pm_math_Vector3_cross_ra ( xx + 289 , xx + 673 , xx + 659 ) ; xx [ 8 ] = xx [
70 ] * state [ 7 ] ; xx [ 50 ] = xx [ 72 ] * state [ 7 ] ; xx [ 53 ] = ( xx [
70 ] * xx [ 8 ] + xx [ 72 ] * xx [ 50 ] ) * xx [ 6 ] ; xx [ 59 ] = state [ 7
] - xx [ 53 ] ; xx [ 249 ] = xx [ 6 ] * ( xx [ 72 ] * xx [ 8 ] - xx [ 70 ] *
xx [ 50 ] ) ; xx [ 8 ] = xx [ 53 ] + state [ 19 ] ; xx [ 684 ] = xx [ 59 ] ;
xx [ 685 ] = xx [ 249 ] ; xx [ 686 ] = xx [ 8 ] ; xx [ 743 ] = xx [ 219 ] *
xx [ 59 ] ; xx [ 744 ] = xx [ 419 ] * xx [ 249 ] ; xx [ 745 ] = xx [ 8 ] * xx
[ 65 ] ; pm_math_Vector3_cross_ra ( xx + 684 , xx + 743 , xx + 746 ) ; xx [
50 ] = xx [ 249 ] * xx [ 130 ] ; xx [ 264 ] = xx [ 130 ] * xx [ 59 ] ; xx [
274 ] = xx [ 59 ] + xx [ 6 ] * ( xx [ 81 ] * xx [ 50 ] - xx [ 264 ] * xx [
130 ] ) ; xx [ 416 ] = xx [ 249 ] - ( xx [ 81 ] * xx [ 264 ] + xx [ 50 ] * xx
[ 130 ] ) * xx [ 6 ] ; xx [ 50 ] = xx [ 8 ] + state [ 21 ] ; xx [ 743 ] = xx
[ 274 ] ; xx [ 744 ] = xx [ 416 ] ; xx [ 745 ] = xx [ 50 ] ; xx [ 749 ] = xx
[ 274 ] * xx [ 19 ] ; xx [ 750 ] = xx [ 104 ] * xx [ 416 ] ; xx [ 751 ] = xx
[ 50 ] * xx [ 118 ] ; pm_math_Vector3_cross_ra ( xx + 743 , xx + 749 , xx +
752 ) ; xx [ 19 ] = xx [ 110 ] * xx [ 416 ] ; xx [ 104 ] = xx [ 274 ] * xx [
110 ] ; xx [ 118 ] = xx [ 274 ] + xx [ 6 ] * ( xx [ 105 ] * xx [ 19 ] - xx [
104 ] * xx [ 110 ] ) ; xx [ 264 ] = xx [ 416 ] - ( xx [ 105 ] * xx [ 104 ] +
xx [ 19 ] * xx [ 110 ] ) * xx [ 6 ] ; xx [ 19 ] = xx [ 50 ] + state [ 23 ] ;
xx [ 749 ] = xx [ 118 ] ; xx [ 750 ] = xx [ 264 ] ; xx [ 751 ] = xx [ 19 ] ;
xx [ 755 ] = xx [ 118 ] * xx [ 21 ] ; xx [ 756 ] = xx [ 21 ] * xx [ 264 ] ;
xx [ 757 ] = xx [ 19 ] * xx [ 119 ] ; pm_math_Vector3_cross_ra ( xx + 749 ,
xx + 755 , xx + 758 ) ; pm_math_Quaternion_compose_ra ( xx + 612 , xx + 629 ,
xx + 761 ) ; xx [ 104 ] = xx [ 764 ] * xx [ 130 ] - xx [ 81 ] * xx [ 761 ] ;
xx [ 430 ] = xx [ 761 ] * xx [ 130 ] + xx [ 81 ] * xx [ 764 ] ; xx [ 435 ] =
xx [ 81 ] * xx [ 762 ] + xx [ 763 ] * xx [ 130 ] ; xx [ 556 ] = xx [ 762 ] *
xx [ 130 ] - xx [ 81 ] * xx [ 763 ] ; xx [ 557 ] = xx [ 279 ] * xx [ 556 ] ;
xx [ 749 ] = - xx [ 435 ] ; xx [ 750 ] = xx [ 556 ] ; xx [ 751 ] = - xx [ 430
] ; xx [ 565 ] = xx [ 435 ] * xx [ 279 ] - xx [ 430 ] * xx [ 55 ] ; xx [ 567
] = xx [ 55 ] * xx [ 556 ] ; xx [ 755 ] = xx [ 557 ] ; xx [ 756 ] = xx [ 565
] ; xx [ 757 ] = - xx [ 567 ] ; pm_math_Vector3_cross_ra ( xx + 749 , xx +
755 , xx + 765 ) ; xx [ 574 ] = xx [ 764 ] * xx [ 133 ] ; xx [ 580 ] = xx [
144 ] * xx [ 764 ] ; xx [ 583 ] = xx [ 144 ] * xx [ 763 ] - xx [ 762 ] * xx [
133 ] ; xx [ 749 ] = - xx [ 574 ] ; xx [ 750 ] = xx [ 580 ] ; xx [ 751 ] = -
xx [ 583 ] ; pm_math_Vector3_cross_ra ( xx + 762 , xx + 749 , xx + 755 ) ;
pm_math_Quaternion_xform_ra ( xx + 612 , xx + 348 , xx + 749 ) ; xx [ 768 ] =
- ( xx [ 105 ] * xx [ 104 ] + xx [ 430 ] * xx [ 110 ] ) ; xx [ 769 ] = xx [
435 ] * xx [ 105 ] - xx [ 110 ] * xx [ 556 ] ; xx [ 770 ] = - ( xx [ 105 ] *
xx [ 556 ] + xx [ 435 ] * xx [ 110 ] ) ; xx [ 771 ] = xx [ 430 ] * xx [ 105 ]
- xx [ 110 ] * xx [ 104 ] ; xx [ 772 ] = xx [ 55 ] + ( xx [ 557 ] * xx [ 104
] + xx [ 765 ] ) * xx [ 6 ] + xx [ 144 ] + ( xx [ 755 ] - xx [ 761 ] * xx [
574 ] ) * xx [ 6 ] + xx [ 749 ] + xx [ 0 ] ; xx [ 773 ] = ( xx [ 565 ] * xx [
104 ] + xx [ 766 ] ) * xx [ 6 ] + ( xx [ 761 ] * xx [ 580 ] + xx [ 756 ] ) *
xx [ 6 ] + xx [ 133 ] + xx [ 750 ] + xx [ 3 ] ; xx [ 774 ] = xx [ 279 ] + xx
[ 6 ] * ( xx [ 767 ] - xx [ 567 ] * xx [ 104 ] ) + xx [ 6 ] * ( xx [ 757 ] -
xx [ 583 ] * xx [ 761 ] ) + xx [ 751 ] + xx [ 5 ] ; xx [ 0 ] =
sm_core_compiler_computeProximityInfoBrickCylinder (
wheel_leg_d9e82f16_1_geometry_5 ( rtdv ) , wheel_leg_d9e82f16_1_geometry_2 (
rtdv ) , ( pm_math_Transform3 * ) ( xx + 622 ) , ( pm_math_Transform3 * ) (
xx + 768 ) , ( pm_math_Vector3 * ) ( xx + 612 ) , ( pm_math_Vector3 * ) ( xx
+ 749 ) , ( pm_math_Vector3 * ) ( xx + 755 ) , ( pm_math_Vector3 * ) ( xx +
761 ) ) ; xx [ 3 ] = xx [ 279 ] * xx [ 416 ] ; xx [ 5 ] = xx [ 341 ] * state
[ 7 ] ; xx [ 104 ] = xx [ 74 ] * state [ 7 ] ; xx [ 764 ] = xx [ 666 ] ; xx [
765 ] = xx [ 319 ] - xx [ 5 ] ; xx [ 766 ] = xx [ 104 ] + xx [ 668 ] ;
pm_math_Quaternion_inverseXform_ra ( xx + 629 , xx + 764 , xx + 665 ) ; xx [
319 ] = xx [ 8 ] * xx [ 133 ] ; xx [ 430 ] = xx [ 665 ] - xx [ 319 ] ; xx [
435 ] = xx [ 8 ] * xx [ 144 ] ; xx [ 556 ] = xx [ 63 ] * state [ 19 ] ; xx [
557 ] = xx [ 435 ] + xx [ 666 ] + xx [ 556 ] ; xx [ 565 ] = xx [ 557 ] * xx [
130 ] ; xx [ 567 ] = xx [ 430 ] * xx [ 130 ] ; xx [ 574 ] = xx [ 3 ] + xx [
430 ] + xx [ 6 ] * ( xx [ 81 ] * xx [ 565 ] - xx [ 567 ] * xx [ 130 ] ) ; xx
[ 430 ] = xx [ 50 ] * xx [ 55 ] - xx [ 274 ] * xx [ 279 ] ; xx [ 580 ] = xx [
55 ] * state [ 21 ] ; xx [ 583 ] = xx [ 430 ] + xx [ 557 ] - ( xx [ 81 ] * xx
[ 567 ] + xx [ 565 ] * xx [ 130 ] ) * xx [ 6 ] + xx [ 580 ] ; xx [ 557 ] = xx
[ 583 ] * xx [ 110 ] ; xx [ 565 ] = xx [ 574 ] * xx [ 110 ] ; xx [ 567 ] = xx
[ 144 ] * xx [ 249 ] - xx [ 133 ] * xx [ 59 ] ; xx [ 599 ] = xx [ 55 ] * xx [
416 ] ; xx [ 775 ] = xx [ 118 ] ; xx [ 776 ] = xx [ 264 ] ; xx [ 777 ] = xx [
19 ] ; xx [ 778 ] = xx [ 574 ] + xx [ 6 ] * ( xx [ 105 ] * xx [ 557 ] - xx [
565 ] * xx [ 110 ] ) ; xx [ 779 ] = xx [ 583 ] - ( xx [ 105 ] * xx [ 565 ] +
xx [ 557 ] * xx [ 110 ] ) * xx [ 6 ] ; xx [ 780 ] = xx [ 667 ] - xx [ 567 ] -
xx [ 599 ] ; sm_core_compiler_computeContactWrenches ( xx [ 0 ] , xx + 612 ,
xx + 749 , xx + 755 , xx + 761 , xx + 622 , xx + 622 , xx + 622 , xx + 768 ,
NULL , xx + 775 , 0 , 1 , xx [ 283 ] , xx [ 395 ] , xx [ 404 ] , xx [ 425 ] ,
xx [ 561 ] , xx [ 575 ] , NULL , xx + 665 ) ; xx [ 0 ] =
sm_core_compiler_computeProximityInfoCxpolyCylinder (
wheel_leg_d9e82f16_1_geometry_3 ( rtdv ) , wheel_leg_d9e82f16_1_geometry_2 (
rtdv ) , ( pm_math_Transform3 * ) ( xx + 649 ) , ( pm_math_Transform3 * ) (
xx + 768 ) , ( pm_math_Vector3 * ) ( xx + 612 ) , ( pm_math_Vector3 * ) ( xx
+ 749 ) , ( pm_math_Vector3 * ) ( xx + 755 ) , ( pm_math_Vector3 * ) ( xx +
761 ) ) ; sm_core_compiler_computeContactWrenches ( xx [ 0 ] , xx + 612 , xx
+ 749 , xx + 755 , xx + 761 , xx + 649 , xx + 622 , xx + 649 , xx + 768 ,
NULL , xx + 775 , 0 , 1 , xx [ 283 ] , xx [ 395 ] , xx [ 404 ] , xx [ 425 ] ,
xx [ 561 ] , xx [ 575 ] , NULL , xx + 781 ) ; xx [ 0 ] =
sm_core_compiler_computeProximityInfoBrickCylinder (
wheel_leg_d9e82f16_1_geometry_0 ( rtdv ) , wheel_leg_d9e82f16_1_geometry_2 (
rtdv ) , ( pm_math_Transform3 * ) ( xx + 717 ) , ( pm_math_Transform3 * ) (
xx + 768 ) , ( pm_math_Vector3 * ) ( xx + 612 ) , ( pm_math_Vector3 * ) ( xx
+ 649 ) , ( pm_math_Vector3 * ) ( xx + 652 ) , ( pm_math_Vector3 * ) ( xx +
749 ) ) ; sm_core_compiler_computeContactWrenches ( xx [ 0 ] , xx + 612 , xx
+ 649 , xx + 652 , xx + 749 , xx + 717 , xx + 622 , xx + 717 , xx + 768 ,
NULL , xx + 775 , 0 , 1 , xx [ 283 ] , xx [ 395 ] , xx [ 404 ] , xx [ 425 ] ,
xx [ 561 ] , xx [ 575 ] , NULL , xx + 761 ) ; xx [ 0 ] =
sm_core_compiler_computeProximityInfoCxpolyCylinder (
wheel_leg_d9e82f16_1_geometry_4 ( rtdv ) , wheel_leg_d9e82f16_1_geometry_2 (
rtdv ) , ( pm_math_Transform3 * ) ( xx + 730 ) , ( pm_math_Transform3 * ) (
xx + 768 ) , ( pm_math_Vector3 * ) ( xx + 612 ) , ( pm_math_Vector3 * ) ( xx
+ 649 ) , ( pm_math_Vector3 * ) ( xx + 652 ) , ( pm_math_Vector3 * ) ( xx +
717 ) ) ; sm_core_compiler_computeContactWrenches ( xx [ 0 ] , xx + 612 , xx
+ 649 , xx + 652 , xx + 717 , xx + 730 , xx + 622 , xx + 730 , xx + 768 ,
NULL , xx + 775 , 0 , 1 , xx [ 283 ] , xx [ 395 ] , xx [ 404 ] , xx [ 425 ] ,
xx [ 561 ] , xx [ 575 ] , NULL , xx + 787 ) ; xx [ 0 ] = xx [ 758 ] - xx [
665 ] - xx [ 781 ] - xx [ 761 ] - xx [ 787 ] + xx [ 21 ] * xx [ 264 ] * state
[ 23 ] ; xx [ 19 ] = xx [ 759 ] - xx [ 666 ] - xx [ 782 ] - xx [ 762 ] - xx [
788 ] - xx [ 21 ] * xx [ 118 ] * state [ 23 ] ; xx [ 118 ] = xx [ 110 ] * xx
[ 19 ] ; xx [ 264 ] = xx [ 0 ] * xx [ 110 ] ; xx [ 612 ] = xx [ 3 ] ; xx [
613 ] = xx [ 430 ] ; xx [ 614 ] = - xx [ 599 ] ; pm_math_Vector3_cross_ra (
xx + 743 , xx + 612 , xx + 622 ) ; xx [ 3 ] = xx [ 622 ] * xx [ 110 ] ; xx [
283 ] = xx [ 623 ] * xx [ 110 ] ; xx [ 395 ] = xx [ 103 ] * ( xx [ 623 ] - (
xx [ 105 ] * xx [ 3 ] + xx [ 283 ] * xx [ 110 ] ) * xx [ 6 ] ) - ( xx [ 669 ]
+ xx [ 785 ] + xx [ 765 ] + xx [ 791 ] ) ; xx [ 404 ] = ( xx [ 622 ] + xx [ 6
] * ( xx [ 105 ] * xx [ 283 ] - xx [ 3 ] * xx [ 110 ] ) ) * xx [ 103 ] - ( xx
[ 668 ] + xx [ 784 ] + xx [ 764 ] + xx [ 790 ] ) ; xx [ 3 ] = xx [ 110 ] * xx
[ 404 ] ; xx [ 283 ] = xx [ 110 ] * xx [ 395 ] ; xx [ 425 ] = xx [ 395 ] + xx
[ 6 ] * ( xx [ 105 ] * xx [ 3 ] - xx [ 283 ] * xx [ 110 ] ) ; xx [ 395 ] = xx
[ 416 ] * state [ 21 ] ; xx [ 416 ] = xx [ 274 ] * state [ 21 ] ; xx [ 612 ]
= - xx [ 319 ] ; xx [ 613 ] = xx [ 435 ] ; xx [ 614 ] = - xx [ 567 ] ;
pm_math_Vector3_cross_ra ( xx + 684 , xx + 612 , xx + 625 ) ; xx [ 319 ] = xx
[ 626 ] * xx [ 130 ] ; xx [ 430 ] = xx [ 625 ] * xx [ 130 ] ; xx [ 435 ] = xx
[ 625 ] + xx [ 6 ] * ( xx [ 81 ] * xx [ 319 ] - xx [ 430 ] * xx [ 130 ] ) - (
xx [ 8 ] + xx [ 50 ] ) * xx [ 580 ] ; xx [ 50 ] = xx [ 626 ] - ( xx [ 81 ] *
xx [ 430 ] + xx [ 319 ] * xx [ 130 ] ) * xx [ 6 ] ; xx [ 319 ] = xx [ 752 ] +
xx [ 0 ] - ( xx [ 105 ] * xx [ 118 ] + xx [ 264 ] * xx [ 110 ] ) * xx [ 6 ] -
xx [ 425 ] * xx [ 279 ] + xx [ 433 ] * xx [ 395 ] - xx [ 416 ] * xx [ 22 ] -
( xx [ 322 ] * xx [ 435 ] + xx [ 320 ] * xx [ 50 ] ) ; xx [ 0 ] = xx [ 760 ]
- xx [ 667 ] - xx [ 783 ] - xx [ 763 ] - xx [ 789 ] ; xx [ 22 ] = ( input [ 1
] - xx [ 0 ] ) / xx [ 121 ] ; xx [ 121 ] = xx [ 754 ] + xx [ 0 ] + xx [ 119 ]
* xx [ 22 ] + xx [ 425 ] * xx [ 55 ] + xx [ 116 ] * xx [ 435 ] + xx [ 109 ] *
xx [ 50 ] - ( xx [ 106 ] * xx [ 395 ] + xx [ 416 ] * xx [ 328 ] ) ; xx [ 0 ]
= xx [ 425 ] + xx [ 435 ] * xx [ 135 ] + xx [ 112 ] * xx [ 50 ] - ( xx [ 320
] * xx [ 395 ] + xx [ 416 ] * xx [ 327 ] ) ; xx [ 106 ] = xx [ 121 ] + xx [ 0
] * xx [ 55 ] ; xx [ 109 ] = xx [ 106 ] / xx [ 127 ] ; xx [ 112 ] = xx [ 319
] + xx [ 321 ] * xx [ 109 ] ; xx [ 116 ] = xx [ 404 ] - ( xx [ 105 ] * xx [
283 ] + xx [ 3 ] * xx [ 110 ] ) * xx [ 6 ] ; xx [ 3 ] = xx [ 103 ] * xx [ 624
] - ( xx [ 670 ] + xx [ 786 ] + xx [ 766 ] + xx [ 792 ] ) ; xx [ 119 ] = ( xx
[ 274 ] + xx [ 274 ] ) * xx [ 580 ] + xx [ 627 ] ; xx [ 135 ] = xx [ 753 ] +
xx [ 19 ] + xx [ 6 ] * ( xx [ 105 ] * xx [ 264 ] - xx [ 118 ] * xx [ 110 ] )
+ xx [ 279 ] * xx [ 116 ] - xx [ 55 ] * xx [ 3 ] + xx [ 395 ] * xx [ 475 ] -
xx [ 473 ] * xx [ 416 ] + xx [ 326 ] * xx [ 435 ] + xx [ 327 ] * xx [ 50 ] -
xx [ 119 ] * xx [ 342 ] ; xx [ 19 ] = xx [ 135 ] - xx [ 329 ] * xx [ 109 ] ;
xx [ 105 ] = xx [ 130 ] * xx [ 19 ] ; xx [ 110 ] = xx [ 112 ] * xx [ 130 ] ;
xx [ 118 ] = xx [ 3 ] + xx [ 416 ] * xx [ 342 ] + xx [ 123 ] * xx [ 119 ] ;
xx [ 3 ] = xx [ 118 ] * xx [ 133 ] ; xx [ 119 ] = xx [ 249 ] * state [ 19 ] ;
xx [ 123 ] = xx [ 59 ] * state [ 19 ] ; xx [ 249 ] = xx [ 104 ] * state [ 7 ]
; xx [ 104 ] = xx [ 72 ] * xx [ 249 ] ; xx [ 264 ] = xx [ 5 ] * state [ 7 ] ;
xx [ 5 ] = xx [ 104 ] - xx [ 70 ] * xx [ 264 ] ; xx [ 274 ] = xx [ 72 ] * xx
[ 264 ] ; xx [ 612 ] = xx [ 5 ] ; xx [ 613 ] = xx [ 274 ] ; xx [ 614 ] = - xx
[ 104 ] ; pm_math_Vector3_cross_ra ( xx + 577 , xx + 612 , xx + 622 ) ; xx [
279 ] = xx [ 6 ] * ( xx [ 622 ] - xx [ 70 ] * xx [ 5 ] ) - ( xx [ 53 ] + xx [
8 ] ) * xx [ 556 ] ; xx [ 5 ] = xx [ 6 ] * ( xx [ 623 ] - xx [ 70 ] * xx [
274 ] ) - xx [ 249 ] ; xx [ 8 ] = ( xx [ 59 ] + xx [ 59 ] ) * xx [ 556 ] + (
xx [ 70 ] * xx [ 104 ] + xx [ 624 ] ) * xx [ 6 ] - xx [ 264 ] ; xx [ 53 ] =
xx [ 477 ] * xx [ 119 ] - xx [ 343 ] * xx [ 123 ] + xx [ 279 ] * xx [ 333 ] +
xx [ 324 ] * xx [ 5 ] + xx [ 8 ] * xx [ 345 ] ; xx [ 59 ] = state [ 18 ] ; if
( xx [ 284 ] < xx [ 59 ] ) xx [ 59 ] = xx [ 284 ] ; xx [ 104 ] = - ( xx [ 59
] / xx [ 386 ] ) ; if ( xx [ 7 ] < xx [ 104 ] ) xx [ 104 ] = xx [ 7 ] ; xx [
249 ] = xx [ 412 ] * state [ 19 ] ; xx [ 264 ] = xx [ 104 ] * xx [ 104 ] * (
xx [ 566 ] - xx [ 6 ] * xx [ 104 ] ) * ( ( - xx [ 59 ] == xx [ 284 ] ? xx [
284 ] : - xx [ 249 ] ) - xx [ 434 ] * xx [ 59 ] ) ; if ( xx [ 284 ] > xx [
264 ] ) xx [ 264 ] = xx [ 284 ] ; xx [ 59 ] = state [ 18 ] - xx [ 52 ] ; if (
xx [ 284 ] > xx [ 59 ] ) xx [ 59 ] = xx [ 284 ] ; xx [ 104 ] = xx [ 59 ] / xx
[ 386 ] ; if ( xx [ 7 ] < xx [ 104 ] ) xx [ 104 ] = xx [ 7 ] ; xx [ 274 ] = (
xx [ 223 ] * xx [ 59 ] + ( xx [ 59 ] == xx [ 284 ] ? xx [ 284 ] : xx [ 249 ]
) ) * xx [ 104 ] * xx [ 104 ] * ( xx [ 566 ] - xx [ 6 ] * xx [ 104 ] ) ; if (
xx [ 284 ] > xx [ 274 ] ) xx [ 274 ] = xx [ 284 ] ; xx [ 59 ] = input [ 4 ] +
xx [ 264 ] - xx [ 274 ] ; xx [ 104 ] = xx [ 0 ] - xx [ 126 ] * xx [ 109 ] ;
xx [ 249 ] = xx [ 116 ] + xx [ 114 ] * xx [ 435 ] + xx [ 50 ] * xx [ 108 ] -
( xx [ 322 ] * xx [ 395 ] + xx [ 326 ] * xx [ 416 ] ) ; xx [ 50 ] = xx [ 249
] - xx [ 117 ] * xx [ 109 ] ; xx [ 108 ] = xx [ 130 ] * xx [ 50 ] ; xx [ 114
] = xx [ 130 ] * xx [ 104 ] ; xx [ 116 ] = xx [ 104 ] + xx [ 6 ] * ( xx [ 81
] * xx [ 108 ] - xx [ 114 ] * xx [ 130 ] ) ; xx [ 104 ] = xx [ 50 ] - ( xx [
81 ] * xx [ 114 ] + xx [ 108 ] * xx [ 130 ] ) * xx [ 6 ] ; xx [ 50 ] = xx [
335 ] * xx [ 119 ] - xx [ 330 ] * xx [ 123 ] + xx [ 137 ] * xx [ 279 ] + xx [
82 ] * xx [ 5 ] ; xx [ 82 ] = xx [ 748 ] + xx [ 121 ] - xx [ 125 ] * xx [ 109
] + xx [ 144 ] * xx [ 116 ] - xx [ 133 ] * xx [ 104 ] + xx [ 50 ] ; xx [ 108
] = xx [ 119 ] * xx [ 324 ] - xx [ 325 ] * xx [ 123 ] + xx [ 129 ] * xx [ 279
] + xx [ 134 ] * xx [ 5 ] ; xx [ 114 ] = xx [ 116 ] + xx [ 108 ] ; xx [ 116 ]
= ( xx [ 59 ] - ( xx [ 82 ] + xx [ 114 ] * xx [ 63 ] ) ) / xx [ 141 ] ; xx [
129 ] = xx [ 144 ] * xx [ 118 ] ; xx [ 134 ] = xx [ 480 ] * xx [ 119 ] - xx [
107 ] * xx [ 123 ] + xx [ 346 ] * xx [ 279 ] + xx [ 325 ] * xx [ 5 ] - xx [ 8
] * xx [ 153 ] ; xx [ 324 ] = xx [ 746 ] + xx [ 112 ] - ( xx [ 81 ] * xx [
105 ] + xx [ 110 ] * xx [ 130 ] ) * xx [ 6 ] + xx [ 3 ] + xx [ 53 ] + xx [
339 ] * xx [ 116 ] ; xx [ 325 ] = xx [ 747 ] + xx [ 19 ] + xx [ 6 ] * ( xx [
81 ] * xx [ 110 ] - xx [ 105 ] * xx [ 130 ] ) - xx [ 129 ] + xx [ 134 ] + xx
[ 332 ] * xx [ 116 ] ; xx [ 326 ] = xx [ 82 ] + xx [ 124 ] * xx [ 116 ] ;
pm_math_Quaternion_xform_ra ( xx + 629 , xx + 324 , xx + 612 ) ; xx [ 19 ] =
xx [ 119 ] * xx [ 333 ] - xx [ 346 ] * xx [ 123 ] + xx [ 138 ] * xx [ 279 ] +
xx [ 111 ] * xx [ 5 ] ; xx [ 82 ] = xx [ 118 ] + xx [ 119 ] * xx [ 345 ] + xx
[ 153 ] * xx [ 123 ] + xx [ 154 ] * xx [ 8 ] ; xx [ 110 ] = xx [ 104 ] + xx [
19 ] + xx [ 146 ] * xx [ 116 ] ; xx [ 111 ] = xx [ 114 ] + xx [ 140 ] * xx [
116 ] ; xx [ 112 ] = xx [ 82 ] ; pm_math_Quaternion_xform_ra ( xx + 629 , xx
+ 110 , xx + 324 ) ; pm_math_Vector3_cross_ra ( xx + 348 , xx + 324 , xx +
110 ) ; xx [ 8 ] = xx [ 149 ] * state [ 7 ] ; xx [ 104 ] = xx [ 151 ] * state
[ 7 ] ; xx [ 105 ] = ( xx [ 149 ] * xx [ 8 ] + xx [ 151 ] * xx [ 104 ] ) * xx
[ 6 ] ; xx [ 107 ] = state [ 7 ] - xx [ 105 ] ; xx [ 111 ] = xx [ 6 ] * ( xx
[ 151 ] * xx [ 8 ] - xx [ 149 ] * xx [ 104 ] ) ; xx [ 8 ] = xx [ 105 ] +
state [ 15 ] ; xx [ 613 ] = xx [ 107 ] ; xx [ 614 ] = xx [ 111 ] ; xx [ 615 ]
= xx [ 8 ] ; xx [ 622 ] = xx [ 219 ] * xx [ 107 ] ; xx [ 623 ] = xx [ 419 ] *
xx [ 111 ] ; xx [ 624 ] = xx [ 8 ] * xx [ 65 ] ; pm_math_Vector3_cross_ra (
xx + 613 , xx + 622 , xx + 625 ) ; xx [ 104 ] = xx [ 111 ] * xx [ 184 ] ; xx
[ 112 ] = xx [ 184 ] * xx [ 107 ] ; xx [ 114 ] = xx [ 107 ] + xx [ 6 ] * ( xx
[ 162 ] * xx [ 104 ] - xx [ 112 ] * xx [ 184 ] ) ; xx [ 118 ] = xx [ 111 ] -
( xx [ 162 ] * xx [ 112 ] + xx [ 104 ] * xx [ 184 ] ) * xx [ 6 ] ; xx [ 104 ]
= xx [ 8 ] + state [ 17 ] ; xx [ 622 ] = xx [ 114 ] ; xx [ 623 ] = xx [ 118 ]
; xx [ 624 ] = xx [ 104 ] ; xx [ 649 ] = xx [ 114 ] * xx [ 293 ] ; xx [ 650 ]
= xx [ 429 ] * xx [ 118 ] ; xx [ 651 ] = xx [ 104 ] * xx [ 51 ] ;
pm_math_Vector3_cross_ra ( xx + 622 , xx + 649 , xx + 652 ) ; xx [ 112 ] = xx
[ 118 ] * state [ 17 ] ; xx [ 118 ] = xx [ 652 ] + xx [ 293 ] * xx [ 112 ] ;
xx [ 137 ] = xx [ 114 ] * state [ 17 ] ; xx [ 138 ] = xx [ 653 ] - xx [ 429 ]
* xx [ 137 ] ; xx [ 153 ] = xx [ 184 ] * xx [ 138 ] ; xx [ 154 ] = xx [ 118 ]
* xx [ 184 ] ; xx [ 264 ] = xx [ 55 ] * state [ 17 ] ; xx [ 622 ] = - ( xx [
8 ] * xx [ 193 ] ) ; xx [ 623 ] = xx [ 8 ] * xx [ 195 ] ; xx [ 624 ] = - ( xx
[ 195 ] * xx [ 111 ] - xx [ 193 ] * xx [ 107 ] ) ; pm_math_Vector3_cross_ra (
xx + 613 , xx + 622 , xx + 649 ) ; xx [ 274 ] = ( ( xx [ 114 ] + xx [ 114 ] )
* xx [ 264 ] + xx [ 651 ] ) * xx [ 35 ] ; xx [ 114 ] = xx [ 111 ] * state [
15 ] ; xx [ 111 ] = xx [ 107 ] * state [ 15 ] ; xx [ 283 ] = xx [ 63 ] *
state [ 15 ] ; xx [ 293 ] = xx [ 208 ] * state [ 7 ] * state [ 7 ] ; xx [ 320
] = xx [ 151 ] * xx [ 293 ] ; xx [ 322 ] = xx [ 357 ] * state [ 7 ] * state [
7 ] ; xx [ 327 ] = xx [ 149 ] * xx [ 322 ] - xx [ 320 ] ; xx [ 328 ] = xx [
151 ] * xx [ 322 ] ; xx [ 613 ] = xx [ 327 ] ; xx [ 614 ] = - xx [ 328 ] ; xx
[ 615 ] = xx [ 320 ] ; pm_math_Vector3_cross_ra ( xx + 571 , xx + 613 , xx +
622 ) ; xx [ 330 ] = ( xx [ 107 ] + xx [ 107 ] ) * xx [ 283 ] + ( xx [ 149 ]
* xx [ 320 ] + xx [ 624 ] ) * xx [ 6 ] - xx [ 322 ] ; xx [ 107 ] = xx [ 625 ]
+ xx [ 118 ] - ( xx [ 162 ] * xx [ 153 ] + xx [ 154 ] * xx [ 184 ] ) * xx [ 6
] + xx [ 274 ] * xx [ 193 ] + xx [ 152 ] * xx [ 114 ] - xx [ 323 ] * xx [ 111
] + xx [ 330 ] * xx [ 76 ] ; xx [ 118 ] = xx [ 626 ] + xx [ 138 ] + xx [ 6 ]
* ( xx [ 162 ] * xx [ 154 ] - xx [ 153 ] * xx [ 184 ] ) - xx [ 195 ] * xx [
274 ] + xx [ 331 ] * xx [ 114 ] - xx [ 136 ] * xx [ 111 ] - xx [ 330 ] * xx [
351 ] ; xx [ 136 ] = xx [ 649 ] * xx [ 184 ] ; xx [ 138 ] = xx [ 650 ] * xx [
184 ] ; xx [ 152 ] = xx [ 35 ] * ( xx [ 650 ] - ( xx [ 162 ] * xx [ 136 ] +
xx [ 138 ] * xx [ 184 ] ) * xx [ 6 ] ) ; xx [ 153 ] = xx [ 654 ] + xx [ 55 ]
* xx [ 152 ] ; xx [ 154 ] = xx [ 153 ] / xx [ 44 ] ; xx [ 320 ] = xx [ 152 ]
- xx [ 43 ] * xx [ 154 ] ; xx [ 322 ] = xx [ 35 ] * ( xx [ 649 ] + xx [ 6 ] *
( xx [ 162 ] * xx [ 138 ] - xx [ 136 ] * xx [ 184 ] ) - ( xx [ 8 ] + xx [ 104
] ) * xx [ 264 ] ) ; xx [ 35 ] = xx [ 322 ] * xx [ 184 ] ; xx [ 104 ] = xx [
162 ] * xx [ 35 ] ; xx [ 136 ] = xx [ 184 ] * xx [ 320 ] ; xx [ 138 ] = xx [
320 ] + xx [ 6 ] * ( xx [ 104 ] - xx [ 136 ] * xx [ 184 ] ) ; xx [ 264 ] = xx
[ 35 ] * xx [ 184 ] ; xx [ 35 ] = xx [ 322 ] - ( xx [ 162 ] * xx [ 136 ] + xx
[ 264 ] ) * xx [ 6 ] ; xx [ 136 ] = xx [ 6 ] * ( xx [ 622 ] + xx [ 149 ] * xx
[ 327 ] ) - ( xx [ 105 ] + xx [ 8 ] ) * xx [ 283 ] ; xx [ 8 ] = xx [ 6 ] * (
xx [ 623 ] - xx [ 149 ] * xx [ 328 ] ) - xx [ 293 ] ; xx [ 105 ] = xx [ 189 ]
* xx [ 136 ] + xx [ 191 ] * xx [ 8 ] ; xx [ 189 ] = xx [ 627 ] + xx [ 654 ] -
xx [ 51 ] * xx [ 154 ] + xx [ 195 ] * xx [ 138 ] - xx [ 193 ] * xx [ 35 ] +
xx [ 105 ] ; xx [ 191 ] = state [ 14 ] + xx [ 52 ] ; if ( xx [ 284 ] < xx [
191 ] ) xx [ 191 ] = xx [ 284 ] ; xx [ 283 ] = - ( xx [ 191 ] / xx [ 386 ] )
; if ( xx [ 7 ] < xx [ 283 ] ) xx [ 283 ] = xx [ 7 ] ; xx [ 293 ] = xx [ 412
] * state [ 15 ] ; xx [ 320 ] = xx [ 283 ] * xx [ 283 ] * ( xx [ 566 ] - xx [
6 ] * xx [ 283 ] ) * ( ( - xx [ 191 ] == xx [ 284 ] ? xx [ 284 ] : - xx [ 293
] ) - xx [ 434 ] * xx [ 191 ] ) ; if ( xx [ 284 ] > xx [ 320 ] ) xx [ 320 ] =
xx [ 284 ] ; xx [ 191 ] = state [ 14 ] ; if ( xx [ 284 ] > xx [ 191 ] ) xx [
191 ] = xx [ 284 ] ; xx [ 283 ] = xx [ 191 ] / xx [ 386 ] ; if ( xx [ 7 ] <
xx [ 283 ] ) xx [ 283 ] = xx [ 7 ] ; xx [ 323 ] = ( xx [ 223 ] * xx [ 191 ] +
( xx [ 191 ] == xx [ 284 ] ? xx [ 284 ] : xx [ 293 ] ) ) * xx [ 283 ] * xx [
283 ] * ( xx [ 566 ] - xx [ 6 ] * xx [ 283 ] ) ; if ( xx [ 284 ] > xx [ 323 ]
) xx [ 323 ] = xx [ 284 ] ; xx [ 191 ] = input [ 3 ] + xx [ 320 ] - xx [ 323
] ; xx [ 283 ] = xx [ 136 ] * xx [ 196 ] + xx [ 197 ] * xx [ 8 ] ; xx [ 196 ]
= xx [ 138 ] + xx [ 283 ] ; xx [ 138 ] = ( xx [ 191 ] - ( xx [ 189 ] + xx [
196 ] * xx [ 63 ] ) ) / xx [ 199 ] ; xx [ 613 ] = xx [ 107 ] ; xx [ 614 ] =
xx [ 118 ] ; xx [ 615 ] = xx [ 189 ] + xx [ 183 ] * xx [ 138 ] ;
pm_math_Quaternion_xform_ra ( xx + 605 , xx + 613 , xx + 622 ) ; xx [ 189 ] =
xx [ 190 ] * xx [ 136 ] + xx [ 8 ] * xx [ 187 ] ; xx [ 187 ] = xx [ 274 ] +
xx [ 351 ] * xx [ 111 ] + xx [ 76 ] * xx [ 114 ] + xx [ 192 ] * xx [ 330 ] ;
xx [ 613 ] = xx [ 35 ] + xx [ 189 ] + xx [ 200 ] * xx [ 138 ] ; xx [ 614 ] =
xx [ 196 ] + xx [ 163 ] * xx [ 138 ] ; xx [ 615 ] = xx [ 187 ] ;
pm_math_Quaternion_xform_ra ( xx + 605 , xx + 613 , xx + 623 ) ;
pm_math_Vector3_cross_ra ( xx + 377 , xx + 623 , xx + 613 ) ; xx [ 649 ] = xx
[ 219 ] * xx [ 147 ] ; xx [ 650 ] = xx [ 419 ] * xx [ 167 ] ; xx [ 651 ] = xx
[ 48 ] * xx [ 65 ] ; pm_math_Vector3_cross_ra ( xx + 602 , xx + 649 , xx +
665 ) ; xx [ 35 ] = xx [ 609 ] - xx [ 678 ] - xx [ 687 ] - xx [ 693 ] - xx [
699 ] - xx [ 705 ] - xx [ 711 ] - xx [ 724 ] - xx [ 737 ] + xx [ 21 ] * xx [
303 ] * state [ 13 ] ; xx [ 65 ] = xx [ 610 ] - xx [ 679 ] - xx [ 688 ] - xx
[ 694 ] - xx [ 700 ] - xx [ 706 ] - xx [ 712 ] - xx [ 725 ] - xx [ 738 ] - xx
[ 21 ] * xx [ 292 ] * state [ 13 ] ; xx [ 21 ] = xx [ 243 ] * xx [ 65 ] ; xx
[ 76 ] = xx [ 35 ] * xx [ 243 ] ; xx [ 190 ] = xx [ 592 ] + xx [ 35 ] - ( xx
[ 238 ] * xx [ 21 ] + xx [ 76 ] * xx [ 243 ] ) * xx [ 6 ] + xx [ 374 ] * xx [
352 ] + xx [ 301 ] * xx [ 337 ] - xx [ 276 ] * xx [ 155 ] + xx [ 358 ] * xx [
380 ] + xx [ 381 ] * xx [ 54 ] ; xx [ 35 ] = xx [ 190 ] - xx [ 382 ] * xx [
242 ] ; xx [ 155 ] = xx [ 338 ] - ( xx [ 238 ] * xx [ 336 ] + xx [ 120 ] * xx
[ 243 ] ) * xx [ 6 ] ; xx [ 120 ] = xx [ 103 ] * xx [ 658 ] - ( xx [ 683 ] +
xx [ 692 ] + xx [ 698 ] + xx [ 704 ] + xx [ 710 ] + xx [ 716 ] + xx [ 729 ] +
xx [ 742 ] ) ; xx [ 103 ] = ( xx [ 240 ] + xx [ 240 ] ) * xx [ 452 ] + xx [
664 ] ; xx [ 192 ] = xx [ 593 ] + xx [ 65 ] + xx [ 6 ] * ( xx [ 238 ] * xx [
76 ] - xx [ 21 ] * xx [ 243 ] ) - ( xx [ 352 ] * xx [ 155 ] + xx [ 55 ] * xx
[ 120 ] ) + xx [ 337 ] * xx [ 347 ] - xx [ 237 ] * xx [ 276 ] - ( xx [ 408 ]
* xx [ 380 ] + xx [ 405 ] * xx [ 54 ] + xx [ 103 ] * xx [ 420 ] ) ; xx [ 21 ]
= xx [ 192 ] + xx [ 407 ] * xx [ 242 ] ; xx [ 65 ] = xx [ 259 ] * xx [ 21 ] ;
xx [ 76 ] = xx [ 35 ] * xx [ 259 ] ; xx [ 196 ] = xx [ 120 ] + xx [ 276 ] *
xx [ 420 ] + xx [ 252 ] * xx [ 103 ] ; xx [ 103 ] = xx [ 196 ] * xx [ 262 ] ;
xx [ 120 ] = xx [ 167 ] * state [ 9 ] ; xx [ 167 ] = xx [ 147 ] * state [ 9 ]
; xx [ 197 ] = xx [ 334 ] * state [ 7 ] ; xx [ 219 ] = xx [ 354 ] * state [ 7
] ; xx [ 237 ] = xx [ 207 ] * xx [ 219 ] ; xx [ 238 ] = xx [ 205 ] * xx [ 197
] - xx [ 237 ] ; xx [ 240 ] = xx [ 207 ] * xx [ 197 ] ; xx [ 333 ] = xx [ 238
] ; xx [ 334 ] = - xx [ 240 ] ; xx [ 335 ] = xx [ 237 ] ;
pm_math_Vector3_cross_ra ( xx + 553 , xx + 333 , xx + 345 ) ; xx [ 243 ] = xx
[ 6 ] * ( xx [ 345 ] + xx [ 205 ] * xx [ 238 ] ) - ( xx [ 113 ] + xx [ 48 ] )
* xx [ 383 ] ; xx [ 48 ] = xx [ 6 ] * ( xx [ 346 ] - xx [ 205 ] * xx [ 240 ]
) - xx [ 219 ] ; xx [ 113 ] = ( xx [ 147 ] + xx [ 147 ] ) * xx [ 383 ] + ( xx
[ 205 ] * xx [ 237 ] + xx [ 347 ] ) * xx [ 6 ] - xx [ 197 ] ; xx [ 147 ] = xx
[ 359 ] * xx [ 120 ] - xx [ 310 ] * xx [ 167 ] + xx [ 243 ] * xx [ 411 ] + xx
[ 246 ] * xx [ 48 ] + xx [ 113 ] * xx [ 423 ] ; xx [ 197 ] = state [ 8 ] ; if
( xx [ 284 ] < xx [ 197 ] ) xx [ 197 ] = xx [ 284 ] ; xx [ 219 ] = - ( xx [
197 ] / xx [ 386 ] ) ; if ( xx [ 7 ] < xx [ 219 ] ) xx [ 219 ] = xx [ 7 ] ;
xx [ 237 ] = xx [ 412 ] * state [ 9 ] ; xx [ 238 ] = xx [ 219 ] * xx [ 219 ]
* ( xx [ 566 ] - xx [ 6 ] * xx [ 219 ] ) * ( ( - xx [ 197 ] == xx [ 284 ] ?
xx [ 284 ] : - xx [ 237 ] ) - xx [ 434 ] * xx [ 197 ] ) ; if ( xx [ 284 ] >
xx [ 238 ] ) xx [ 238 ] = xx [ 284 ] ; xx [ 197 ] = state [ 8 ] - xx [ 52 ] ;
if ( xx [ 284 ] > xx [ 197 ] ) xx [ 197 ] = xx [ 284 ] ; xx [ 52 ] = xx [ 197
] / xx [ 386 ] ; if ( xx [ 7 ] < xx [ 52 ] ) xx [ 52 ] = xx [ 7 ] ; xx [ 7 ]
= ( xx [ 223 ] * xx [ 197 ] + ( xx [ 197 ] == xx [ 284 ] ? xx [ 284 ] : xx [
237 ] ) ) * xx [ 52 ] * xx [ 52 ] * ( xx [ 566 ] - xx [ 6 ] * xx [ 52 ] ) ;
if ( xx [ 284 ] > xx [ 7 ] ) xx [ 7 ] = xx [ 284 ] ; xx [ 52 ] = input [ 2 ]
+ xx [ 238 ] - xx [ 7 ] ; xx [ 7 ] = xx [ 1 ] - xx [ 255 ] * xx [ 242 ] ; xx
[ 197 ] = xx [ 155 ] + xx [ 247 ] * xx [ 380 ] + xx [ 54 ] * xx [ 241 ] + xx
[ 358 ] * xx [ 337 ] + xx [ 408 ] * xx [ 276 ] ; xx [ 54 ] = xx [ 197 ] - xx
[ 250 ] * xx [ 242 ] ; xx [ 155 ] = xx [ 259 ] * xx [ 54 ] ; xx [ 219 ] = xx
[ 259 ] * xx [ 7 ] ; xx [ 223 ] = xx [ 7 ] + xx [ 6 ] * ( xx [ 216 ] * xx [
155 ] - xx [ 219 ] * xx [ 259 ] ) ; xx [ 7 ] = xx [ 54 ] - ( xx [ 216 ] * xx
[ 219 ] + xx [ 155 ] * xx [ 259 ] ) * xx [ 6 ] ; xx [ 54 ] = xx [ 344 ] * xx
[ 120 ] - xx [ 368 ] * xx [ 167 ] + xx [ 266 ] * xx [ 243 ] + xx [ 217 ] * xx
[ 48 ] ; xx [ 155 ] = xx [ 667 ] + xx [ 355 ] - xx [ 254 ] * xx [ 242 ] + xx
[ 273 ] * xx [ 223 ] - xx [ 262 ] * xx [ 7 ] + xx [ 54 ] ; xx [ 217 ] = xx [
120 ] * xx [ 246 ] - xx [ 385 ] * xx [ 167 ] + xx [ 258 ] * xx [ 243 ] + xx [
263 ] * xx [ 48 ] ; xx [ 219 ] = xx [ 223 ] + xx [ 217 ] ; xx [ 223 ] = ( xx
[ 52 ] - ( xx [ 155 ] + xx [ 219 ] * xx [ 63 ] ) ) / xx [ 270 ] ; xx [ 237 ]
= xx [ 273 ] * xx [ 196 ] ; xx [ 238 ] = xx [ 421 ] * xx [ 120 ] - xx [ 158 ]
* xx [ 167 ] + xx [ 424 ] * xx [ 243 ] + xx [ 385 ] * xx [ 48 ] - xx [ 113 ]
* xx [ 102 ] ; xx [ 333 ] = xx [ 665 ] + xx [ 35 ] - ( xx [ 216 ] * xx [ 65 ]
+ xx [ 76 ] * xx [ 259 ] ) * xx [ 6 ] + xx [ 103 ] + xx [ 147 ] + xx [ 417 ]
* xx [ 223 ] ; xx [ 334 ] = xx [ 666 ] + xx [ 21 ] + xx [ 6 ] * ( xx [ 216 ]
* xx [ 76 ] - xx [ 65 ] * xx [ 259 ] ) - xx [ 237 ] + xx [ 238 ] + xx [ 410 ]
* xx [ 223 ] ; xx [ 335 ] = xx [ 155 ] + xx [ 253 ] * xx [ 223 ] ;
pm_math_Quaternion_xform_ra ( xx + 595 , xx + 333 , xx + 336 ) ; xx [ 21 ] =
xx [ 120 ] * xx [ 411 ] - xx [ 424 ] * xx [ 167 ] + xx [ 267 ] * xx [ 243 ] +
xx [ 244 ] * xx [ 48 ] ; xx [ 35 ] = xx [ 196 ] + xx [ 120 ] * xx [ 423 ] +
xx [ 102 ] * xx [ 167 ] + xx [ 282 ] * xx [ 113 ] ; xx [ 333 ] = xx [ 7 ] +
xx [ 21 ] + xx [ 275 ] * xx [ 223 ] ; xx [ 334 ] = xx [ 219 ] + xx [ 269 ] *
xx [ 223 ] ; xx [ 335 ] = xx [ 35 ] ; pm_math_Quaternion_xform_ra ( xx + 595
, xx + 333 , xx + 342 ) ; pm_math_Vector3_cross_ra ( xx + 426 , xx + 342 , xx
+ 333 ) ; xx [ 7 ] = xx [ 316 ] * state [ 7 ] ; xx [ 65 ] = xx [ 85 ] * xx [
7 ] ; xx [ 76 ] = xx [ 225 ] * xx [ 7 ] ; xx [ 102 ] = xx [ 674 ] + xx [ 325
] + xx [ 624 ] + xx [ 343 ] + xx [ 76 ] ; xx [ 113 ] = ( xx [ 672 ] + xx [
659 ] + xx [ 612 ] + xx [ 110 ] + xx [ 622 ] + xx [ 613 ] + xx [ 336 ] + xx [
333 ] + xx [ 65 ] + xx [ 102 ] * xx [ 84 ] ) / xx [ 168 ] ; xx [ 110 ] = xx [
302 ] * xx [ 7 ] ; xx [ 155 ] = xx [ 169 ] * xx [ 7 ] ; xx [ 301 ] = xx [ 673
] + xx [ 324 ] + xx [ 623 ] + xx [ 342 ] + xx [ 110 ] - xx [ 165 ] * xx [ 113
] ; xx [ 302 ] = xx [ 102 ] - xx [ 87 ] * xx [ 113 ] ; xx [ 303 ] = xx [ 675
] + xx [ 326 ] + xx [ 625 ] + xx [ 344 ] + xx [ 155 ] - xx [ 90 ] * xx [ 113
] ; pm_math_Quaternion_xform_ra ( xx + 294 , xx + 301 , xx + 323 ) ; xx [ 301
] = - xx [ 323 ] ; xx [ 302 ] = - xx [ 324 ] ; xx [ 303 ] = - xx [ 325 ] ;
solveSymmetricPosDef ( xx + 544 , xx + 301 , 3 , 1 , xx + 323 , xx + 326 ) ;
xx [ 102 ] = 9.806650000000001 ; xx [ 158 ] = xx [ 305 ] - xx [ 314 ] + xx [
363 ] - xx [ 372 ] + xx [ 390 ] - xx [ 399 ] + xx [ 441 ] - xx [ 450 ] ; xx [
169 ] = xx [ 467 ] - xx [ 29 ] - xx [ 27 ] - xx [ 458 ] + xx [ 484 ] - xx [
95 ] - xx [ 93 ] - xx [ 493 ] + xx [ 511 ] - xx [ 176 ] - xx [ 174 ] - xx [
502 ] + xx [ 520 ] - xx [ 230 ] - xx [ 228 ] - xx [ 529 ] + xx [ 158 ] * xx [
84 ] ; xx [ 26 ] = xx [ 308 ] - xx [ 315 ] + xx [ 366 ] - xx [ 373 ] + xx [
393 ] - xx [ 400 ] + xx [ 444 ] - xx [ 451 ] ; xx [ 27 ] = xx [ 470 ] - xx [
32 ] - xx [ 28 ] - xx [ 461 ] + xx [ 487 ] - xx [ 98 ] - xx [ 94 ] - xx [ 496
] + xx [ 514 ] - xx [ 179 ] - xx [ 175 ] - xx [ 505 ] + xx [ 523 ] - xx [ 233
] - xx [ 229 ] - xx [ 532 ] + xx [ 26 ] * xx [ 84 ] ; xx [ 92 ] = xx [ 73 ] -
xx [ 265 ] * xx [ 222 ] ; xx [ 93 ] = xx [ 188 ] - xx [ 265 ] * xx [ 220 ] ;
xx [ 94 ] = xx [ 85 ] - xx [ 265 ] * xx [ 91 ] ; xx [ 95 ] = xx [ 304 ] - xx
[ 311 ] + xx [ 362 ] - xx [ 369 ] + xx [ 389 ] - xx [ 396 ] + xx [ 440 ] - xx
[ 447 ] - xx [ 169 ] * xx [ 222 ] ; xx [ 96 ] = xx [ 158 ] - xx [ 169 ] * xx
[ 220 ] ; xx [ 97 ] = xx [ 306 ] - xx [ 317 ] + xx [ 364 ] - xx [ 375 ] + xx
[ 391 ] - xx [ 402 ] + xx [ 442 ] - xx [ 453 ] - xx [ 169 ] * xx [ 91 ] ; xx
[ 98 ] = xx [ 307 ] - xx [ 312 ] + xx [ 365 ] - xx [ 370 ] + xx [ 392 ] - xx
[ 397 ] + xx [ 443 ] - xx [ 448 ] - xx [ 27 ] * xx [ 222 ] ; xx [ 99 ] = xx [
26 ] - xx [ 27 ] * xx [ 220 ] ; xx [ 100 ] = xx [ 309 ] - xx [ 318 ] + xx [
367 ] - xx [ 376 ] + xx [ 394 ] - xx [ 403 ] + xx [ 445 ] - xx [ 454 ] - xx [
27 ] * xx [ 91 ] ; pm_math_Matrix3x3_composeTranspose_ra ( xx + 92 , xx + 10
, xx + 26 ) ; pm_math_Matrix3x3_compose_ra ( xx + 10 , xx + 26 , xx + 91 ) ;
xx [ 10 ] = - xx [ 449 ] ; xx [ 11 ] = xx [ 413 ] ; xx [ 12 ] = - xx [ 476 ]
; pm_math_Matrix3x3_postCross_ra ( xx + 535 , xx + 10 , xx + 26 ) ; xx [ 447
] = xx [ 91 ] - xx [ 26 ] ; xx [ 448 ] = xx [ 92 ] - xx [ 29 ] ; xx [ 449 ] =
xx [ 93 ] - xx [ 32 ] ; xx [ 450 ] = xx [ 94 ] - xx [ 27 ] ; xx [ 451 ] = xx
[ 95 ] - xx [ 30 ] ; xx [ 452 ] = xx [ 96 ] - xx [ 33 ] ; xx [ 453 ] = xx [
97 ] - xx [ 28 ] ; xx [ 454 ] = xx [ 98 ] - xx [ 31 ] ; xx [ 455 ] = xx [ 99
] - xx [ 34 ] ; xx [ 456 ] = xx [ 535 ] ; xx [ 457 ] = xx [ 536 ] ; xx [ 458
] = xx [ 537 ] ; xx [ 459 ] = xx [ 538 ] ; xx [ 460 ] = xx [ 539 ] ; xx [ 461
] = xx [ 540 ] ; xx [ 462 ] = xx [ 541 ] ; xx [ 463 ] = xx [ 542 ] ; xx [ 464
] = xx [ 543 ] ; solveSymmetricPosDef ( xx + 544 , xx + 447 , 3 , 6 , xx +
480 , xx + 10 ) ; xx [ 10 ] = 1.77635683940025e-15 ; xx [ 11 ] = xx [ 102 ] *
xx [ 489 ] + xx [ 10 ] * xx [ 495 ] ; xx [ 12 ] = xx [ 102 ] * xx [ 490 ] +
xx [ 10 ] * xx [ 496 ] ; xx [ 13 ] = xx [ 102 ] * xx [ 491 ] + xx [ 10 ] * xx
[ 497 ] ; xx [ 14 ] = xx [ 323 ] + xx [ 11 ] - xx [ 102 ] ; xx [ 15 ] = xx [
324 ] + xx [ 12 ] ; xx [ 16 ] = xx [ 325 ] + xx [ 13 ] - xx [ 10 ] ;
pm_math_Quaternion_inverseXform_ra ( xx + 294 , xx + 14 , xx + 26 ) ; xx [ 14
] = xx [ 113 ] + pm_math_Vector3_dot_ra ( xx + 589 , xx + 26 ) ; xx [ 15 ] =
xx [ 14 ] * xx [ 205 ] ; xx [ 16 ] = xx [ 14 ] * xx [ 207 ] ; xx [ 17 ] = (
xx [ 15 ] * xx [ 205 ] + xx [ 16 ] * xx [ 207 ] ) * xx [ 6 ] ; xx [ 18 ] = xx
[ 17 ] - xx [ 14 ] ; xx [ 29 ] = xx [ 18 ] + xx [ 120 ] ; xx [ 30 ] = xx [ 6
] * ( xx [ 16 ] * xx [ 205 ] - xx [ 15 ] * xx [ 207 ] ) ; xx [ 15 ] = xx [ 30
] - xx [ 167 ] ; xx [ 16 ] = xx [ 259 ] * xx [ 15 ] ; xx [ 31 ] = xx [ 29 ] *
xx [ 259 ] ; xx [ 32 ] = xx [ 18 ] ; xx [ 33 ] = xx [ 30 ] ; xx [ 34 ] = - xx
[ 17 ] ; xx [ 18 ] = xx [ 27 ] - xx [ 14 ] * xx [ 84 ] ; xx [ 27 ] = xx [ 28
] + xx [ 7 ] ; xx [ 91 ] = xx [ 26 ] ; xx [ 92 ] = xx [ 18 ] + xx [ 14 ] * xx
[ 418 ] ; xx [ 93 ] = xx [ 27 ] - xx [ 14 ] * xx [ 209 ] ;
pm_math_Quaternion_inverseXform_ra ( xx + 595 , xx + 91 , xx + 94 ) ; xx [ 28
] = xx [ 223 ] - ( pm_math_Vector3_dot_ra ( xx + 562 , xx + 32 ) + xx [ 271 ]
* xx [ 94 ] + xx [ 277 ] * xx [ 95 ] ) ; xx [ 30 ] = xx [ 28 ] - xx [ 17 ] ;
xx [ 32 ] = xx [ 29 ] + xx [ 6 ] * ( xx [ 216 ] * xx [ 16 ] - xx [ 31 ] * xx
[ 259 ] ) ; xx [ 33 ] = xx [ 15 ] - ( xx [ 216 ] * xx [ 31 ] + xx [ 16 ] * xx
[ 259 ] ) * xx [ 6 ] ; xx [ 34 ] = xx [ 30 ] ; xx [ 15 ] = xx [ 94 ] + xx [
243 ] - xx [ 262 ] * xx [ 30 ] ; xx [ 16 ] = xx [ 95 ] + xx [ 63 ] * xx [ 28
] + xx [ 48 ] + xx [ 273 ] * xx [ 30 ] ; xx [ 17 ] = xx [ 16 ] * xx [ 259 ] ;
xx [ 29 ] = xx [ 15 ] * xx [ 259 ] ; xx [ 30 ] = xx [ 242 ] +
pm_math_Vector3_dot_ra ( xx + 584 , xx + 32 ) + ( xx [ 15 ] + xx [ 6 ] * ( xx
[ 216 ] * xx [ 17 ] - xx [ 29 ] * xx [ 259 ] ) ) * xx [ 257 ] + xx [ 248 ] *
( xx [ 16 ] - ( xx [ 216 ] * xx [ 29 ] + xx [ 17 ] * xx [ 259 ] ) * xx [ 6 ]
) ; xx [ 15 ] = state [ 11 ] * state [ 11 ] ; xx [ 16 ] = xx [ 15 ] * ( xx [
272 ] - xx [ 55 ] ) ; xx [ 17 ] = xx [ 207 ] * xx [ 16 ] ; xx [ 29 ] = xx [
262 ] * xx [ 15 ] ; xx [ 15 ] = xx [ 207 ] * xx [ 29 ] ; xx [ 31 ] = xx [ 15
] + xx [ 205 ] * xx [ 16 ] ; xx [ 32 ] = - xx [ 15 ] ; xx [ 33 ] = - xx [ 17
] ; xx [ 34 ] = xx [ 31 ] ; pm_math_Vector3_cross_ra ( xx + 553 , xx + 32 ,
xx + 91 ) ; xx [ 15 ] = xx [ 214 ] * state [ 9 ] * state [ 9 ] ; xx [ 16 ] =
xx [ 6 ] * xx [ 213 ] * state [ 9 ] * state [ 9 ] ; xx [ 32 ] = state [ 9 ] *
state [ 9 ] ; xx [ 33 ] = xx [ 6 ] * xx [ 261 ] * state [ 11 ] * state [ 9 ]
; xx [ 34 ] = xx [ 207 ] * xx [ 33 ] ; xx [ 73 ] = xx [ 414 ] * state [ 11 ]
* state [ 9 ] ; xx [ 85 ] = xx [ 207 ] * xx [ 73 ] ; xx [ 94 ] = xx [ 34 ] -
xx [ 205 ] * xx [ 73 ] ; xx [ 95 ] = - xx [ 34 ] ; xx [ 96 ] = xx [ 85 ] ; xx
[ 97 ] = xx [ 94 ] ; pm_math_Vector3_cross_ra ( xx + 553 , xx + 95 , xx + 98
) ; xx [ 34 ] = 4.0 ; xx [ 73 ] = xx [ 236 ] * state [ 11 ] * state [ 11 ] ;
xx [ 95 ] = xx [ 6 ] * xx [ 206 ] * state [ 11 ] * state [ 11 ] ; xx [ 96 ] =
state [ 17 ] * state [ 17 ] ; xx [ 97 ] = xx [ 96 ] * ( xx [ 194 ] - xx [ 55
] ) ; xx [ 113 ] = xx [ 151 ] * xx [ 97 ] ; xx [ 158 ] = xx [ 193 ] * xx [ 96
] ; xx [ 96 ] = xx [ 151 ] * xx [ 158 ] ; xx [ 169 ] = xx [ 96 ] + xx [ 149 ]
* xx [ 97 ] ; xx [ 173 ] = - xx [ 96 ] ; xx [ 174 ] = - xx [ 113 ] ; xx [ 175
] = xx [ 169 ] ; pm_math_Vector3_cross_ra ( xx + 571 , xx + 173 , xx + 176 )
; xx [ 96 ] = xx [ 160 ] * state [ 15 ] * state [ 15 ] ; xx [ 97 ] = xx [ 6 ]
* xx [ 159 ] * state [ 15 ] * state [ 15 ] ; xx [ 159 ] = state [ 15 ] *
state [ 15 ] ; xx [ 173 ] = xx [ 6 ] * xx [ 198 ] * state [ 17 ] * state [ 15
] ; xx [ 174 ] = xx [ 151 ] * xx [ 173 ] ; xx [ 175 ] = xx [ 157 ] * state [
17 ] * state [ 15 ] ; xx [ 157 ] = xx [ 151 ] * xx [ 175 ] ; xx [ 179 ] = xx
[ 174 ] - xx [ 149 ] * xx [ 175 ] ; xx [ 227 ] = - xx [ 174 ] ; xx [ 228 ] =
xx [ 157 ] ; xx [ 229 ] = xx [ 179 ] ; pm_math_Vector3_cross_ra ( xx + 571 ,
xx + 227 , xx + 230 ) ; xx [ 174 ] = xx [ 182 ] * state [ 17 ] * state [ 17 ]
; xx [ 175 ] = xx [ 6 ] * xx [ 150 ] * state [ 17 ] * state [ 17 ] ; xx [ 227
] = xx [ 26 ] ; xx [ 228 ] = xx [ 18 ] + xx [ 14 ] * xx [ 357 ] ; xx [ 229 ]
= xx [ 27 ] - xx [ 14 ] * xx [ 208 ] ; pm_math_Quaternion_inverseXform_ra (
xx + 605 , xx + 227 , xx + 233 ) ; xx [ 150 ] = ( xx [ 14 ] * xx [ 149 ] * xx
[ 149 ] + xx [ 14 ] * xx [ 151 ] * xx [ 151 ] ) * xx [ 6 ] ; xx [ 180 ] = xx
[ 138 ] - ( xx [ 202 ] * xx [ 233 ] + xx [ 204 ] * xx [ 234 ] - xx [ 139 ] *
xx [ 150 ] ) ; xx [ 138 ] = xx [ 180 ] - xx [ 150 ] ; xx [ 150 ] = xx [ 234 ]
+ xx [ 63 ] * xx [ 180 ] + xx [ 8 ] + xx [ 195 ] * xx [ 138 ] ; xx [ 181 ] =
xx [ 154 ] + xx [ 201 ] * xx [ 138 ] + xx [ 186 ] * ( xx [ 150 ] - ( xx [ 162
] * ( xx [ 233 ] + xx [ 136 ] - xx [ 193 ] * xx [ 138 ] ) * xx [ 184 ] + xx [
150 ] * xx [ 184 ] * xx [ 184 ] ) * xx [ 6 ] ) ; xx [ 138 ] = xx [ 6 ] * xx [
215 ] * state [ 9 ] * state [ 9 ] ; xx [ 91 ] = xx [ 6 ] * xx [ 161 ] * state
[ 15 ] * state [ 15 ] ; xx [ 98 ] = xx [ 14 ] * xx [ 70 ] ; xx [ 150 ] = xx [
14 ] * xx [ 72 ] ; xx [ 154 ] = ( xx [ 98 ] * xx [ 70 ] + xx [ 150 ] * xx [
72 ] ) * xx [ 6 ] ; xx [ 161 ] = xx [ 154 ] - xx [ 14 ] ; xx [ 176 ] = xx [
161 ] + xx [ 119 ] ; xx [ 188 ] = xx [ 6 ] * ( xx [ 150 ] * xx [ 70 ] - xx [
98 ] * xx [ 72 ] ) ; xx [ 98 ] = xx [ 188 ] - xx [ 123 ] ; xx [ 150 ] = xx [
130 ] * xx [ 98 ] ; xx [ 194 ] = xx [ 176 ] * xx [ 130 ] ; xx [ 227 ] = xx [
161 ] ; xx [ 228 ] = xx [ 188 ] ; xx [ 229 ] = - xx [ 154 ] ; xx [ 233 ] = xx
[ 26 ] ; xx [ 234 ] = xx [ 18 ] + xx [ 14 ] * xx [ 341 ] ; xx [ 235 ] = xx [
27 ] - xx [ 14 ] * xx [ 74 ] ; pm_math_Quaternion_inverseXform_ra ( xx + 629
, xx + 233 , xx + 240 ) ; xx [ 161 ] = xx [ 116 ] - ( pm_math_Vector3_dot_ra
( xx + 616 , xx + 227 ) + xx [ 142 ] * xx [ 240 ] + xx [ 148 ] * xx [ 241 ] )
; xx [ 116 ] = xx [ 161 ] - xx [ 154 ] ; xx [ 227 ] = xx [ 176 ] + xx [ 6 ] *
( xx [ 81 ] * xx [ 150 ] - xx [ 194 ] * xx [ 130 ] ) ; xx [ 228 ] = xx [ 98 ]
- ( xx [ 81 ] * xx [ 194 ] + xx [ 150 ] * xx [ 130 ] ) * xx [ 6 ] ; xx [ 229
] = xx [ 116 ] ; xx [ 98 ] = xx [ 240 ] + xx [ 279 ] - xx [ 133 ] * xx [ 116
] ; xx [ 150 ] = xx [ 241 ] + xx [ 63 ] * xx [ 161 ] + xx [ 5 ] + xx [ 144 ]
* xx [ 116 ] ; xx [ 116 ] = xx [ 150 ] * xx [ 130 ] ; xx [ 154 ] = xx [ 98 ]
* xx [ 130 ] ; xx [ 176 ] = xx [ 109 ] + pm_math_Vector3_dot_ra ( xx + 619 ,
xx + 227 ) + ( xx [ 98 ] + xx [ 6 ] * ( xx [ 81 ] * xx [ 116 ] - xx [ 154 ] *
xx [ 130 ] ) ) * xx [ 128 ] + xx [ 115 ] * ( xx [ 150 ] - ( xx [ 81 ] * xx [
154 ] + xx [ 116 ] * xx [ 130 ] ) * xx [ 6 ] ) ; xx [ 98 ] = xx [ 6 ] * xx [
78 ] * state [ 19 ] * state [ 19 ] ; xx [ 78 ] = xx [ 79 ] * state [ 19 ] *
state [ 19 ] ; xx [ 109 ] = xx [ 101 ] * state [ 21 ] * state [ 21 ] ; xx [
116 ] = xx [ 6 ] * xx [ 71 ] * state [ 21 ] * state [ 21 ] ; xx [ 71 ] =
state [ 21 ] * state [ 21 ] ; xx [ 150 ] = xx [ 71 ] * ( xx [ 143 ] - xx [ 55
] ) ; xx [ 143 ] = xx [ 72 ] * xx [ 150 ] ; xx [ 154 ] = xx [ 133 ] * xx [ 71
] ; xx [ 71 ] = xx [ 72 ] * xx [ 154 ] ; xx [ 188 ] = xx [ 71 ] + xx [ 70 ] *
xx [ 150 ] ; xx [ 227 ] = xx [ 71 ] ; xx [ 228 ] = xx [ 143 ] ; xx [ 229 ] =
- xx [ 188 ] ; pm_math_Vector3_cross_ra ( xx + 577 , xx + 227 , xx + 233 ) ;
xx [ 71 ] = state [ 19 ] * state [ 19 ] ; xx [ 150 ] = xx [ 6 ] * xx [ 132 ]
* state [ 21 ] * state [ 19 ] ; xx [ 132 ] = xx [ 72 ] * xx [ 150 ] ; xx [
194 ] = xx [ 281 ] * state [ 21 ] * state [ 19 ] ; xx [ 196 ] = xx [ 72 ] *
xx [ 194 ] ; xx [ 198 ] = xx [ 70 ] * xx [ 194 ] - xx [ 132 ] ; xx [ 227 ] =
xx [ 132 ] ; xx [ 228 ] = - xx [ 196 ] ; xx [ 229 ] = xx [ 198 ] ;
pm_math_Vector3_cross_ra ( xx + 577 , xx + 227 , xx + 240 ) ; xx [ 132 ] = xx
[ 6 ] * xx [ 23 ] * state [ 25 ] * state [ 25 ] ; xx [ 23 ] = xx [ 24 ] *
state [ 25 ] * state [ 25 ] ; xx [ 194 ] = xx [ 39 ] * state [ 27 ] * state [
27 ] ; xx [ 206 ] = xx [ 6 ] * xx [ 36 ] * state [ 27 ] * state [ 27 ] ; xx [
36 ] = state [ 27 ] * state [ 27 ] ; xx [ 213 ] = xx [ 36 ] * ( xx [ 57 ] -
xx [ 55 ] ) ; xx [ 57 ] = xx [ 9 ] * xx [ 213 ] ; xx [ 215 ] = xx [ 56 ] * xx
[ 36 ] ; xx [ 36 ] = xx [ 9 ] * xx [ 215 ] ; xx [ 219 ] = xx [ 36 ] + xx [ 4
] * xx [ 213 ] ; xx [ 227 ] = xx [ 36 ] ; xx [ 228 ] = xx [ 57 ] ; xx [ 229 ]
= - xx [ 219 ] ; pm_math_Vector3_cross_ra ( xx + 298 , xx + 227 , xx + 265 )
; xx [ 36 ] = state [ 25 ] * state [ 25 ] ; xx [ 213 ] = xx [ 6 ] * xx [ 61 ]
* state [ 27 ] * state [ 25 ] ; xx [ 61 ] = xx [ 9 ] * xx [ 213 ] ; xx [ 220
] = xx [ 166 ] * state [ 27 ] * state [ 25 ] ; xx [ 166 ] = xx [ 9 ] * xx [
220 ] ; xx [ 222 ] = xx [ 4 ] * xx [ 220 ] - xx [ 61 ] ; xx [ 227 ] = xx [ 61
] ; xx [ 228 ] = - xx [ 166 ] ; xx [ 229 ] = xx [ 222 ] ;
pm_math_Vector3_cross_ra ( xx + 298 , xx + 227 , xx + 301 ) ; xx [ 227 ] = xx
[ 26 ] ; xx [ 228 ] = xx [ 18 ] + xx [ 14 ] * xx [ 288 ] ; xx [ 229 ] = xx [
27 ] - xx [ 14 ] * xx [ 285 ] ; pm_math_Quaternion_inverseXform_ra ( xx + 436
, xx + 227 , xx + 298 ) ; xx [ 18 ] = ( xx [ 14 ] * xx [ 4 ] * xx [ 4 ] + xx
[ 14 ] * xx [ 9 ] * xx [ 9 ] ) * xx [ 6 ] ; xx [ 14 ] = xx [ 60 ] - ( xx [ 67
] * xx [ 298 ] + xx [ 69 ] * xx [ 299 ] - xx [ 432 ] * xx [ 18 ] ) ; xx [ 26
] = xx [ 14 ] - xx [ 18 ] ; xx [ 18 ] = xx [ 299 ] + xx [ 63 ] * xx [ 14 ] +
xx [ 83 ] + xx [ 58 ] * xx [ 26 ] ; xx [ 27 ] = xx [ 224 ] + xx [ 66 ] * xx [
26 ] + xx [ 46 ] * ( xx [ 18 ] - ( xx [ 37 ] * ( xx [ 298 ] + xx [ 388 ] - xx
[ 56 ] * xx [ 26 ] ) * xx [ 41 ] + xx [ 18 ] * xx [ 41 ] * xx [ 41 ] ) * xx [
6 ] ) ; xx [ 18 ] = xx [ 6 ] * xx [ 80 ] * state [ 19 ] * state [ 19 ] ; xx [
26 ] = xx [ 6 ] * xx [ 25 ] * state [ 25 ] * state [ 25 ] ; xx [ 227 ] = xx [
30 ] * xx [ 384 ] - ( ( xx [ 205 ] * xx [ 17 ] + xx [ 92 ] ) * xx [ 6 ] - xx
[ 29 ] - ( xx [ 262 ] * xx [ 15 ] + xx [ 273 ] * xx [ 16 ] + xx [ 212 ] * xx
[ 32 ] ) + xx [ 6 ] * ( xx [ 6 ] * ( xx [ 99 ] - xx [ 205 ] * xx [ 85 ] ) -
xx [ 33 ] ) - xx [ 55 ] * ( xx [ 34 ] * ( xx [ 415 ] * xx [ 371 ] + xx [ 409
] * xx [ 371 ] ) * state [ 9 ] * state [ 11 ] + xx [ 16 ] * xx [ 236 ] + xx [
260 ] * xx [ 15 ] + xx [ 210 ] * xx [ 73 ] + xx [ 95 ] * xx [ 214 ] ) - ( (
xx [ 149 ] * xx [ 113 ] + xx [ 177 ] ) * xx [ 6 ] - xx [ 158 ] - ( xx [ 193 ]
* xx [ 96 ] + xx [ 195 ] * xx [ 97 ] + xx [ 353 ] * xx [ 159 ] ) + xx [ 6 ] *
( xx [ 6 ] * ( xx [ 231 ] - xx [ 149 ] * xx [ 157 ] ) - xx [ 173 ] ) - xx [
55 ] * ( xx [ 34 ] * ( xx [ 581 ] * xx [ 446 ] + xx [ 576 ] * xx [ 446 ] ) *
state [ 15 ] * state [ 17 ] + xx [ 97 ] * xx [ 182 ] + xx [ 185 ] * xx [ 96 ]
+ xx [ 156 ] * xx [ 174 ] + xx [ 175 ] * xx [ 160 ] ) ) + xx [ 422 ] * xx [
28 ] ) + xx [ 474 ] * xx [ 180 ] - xx [ 181 ] * xx [ 582 ] ; xx [ 228 ] = xx
[ 30 ] * xx [ 398 ] - ( xx [ 278 ] * xx [ 32 ] - ( xx [ 262 ] * xx [ 16 ] +
xx [ 273 ] * xx [ 138 ] ) + ( xx [ 93 ] - xx [ 205 ] * xx [ 31 ] ) * xx [ 6 ]
+ ( xx [ 100 ] - xx [ 205 ] * xx [ 94 ] ) * xx [ 34 ] - ( xx [ 138 ] * xx [
236 ] + xx [ 260 ] * xx [ 16 ] + xx [ 203 ] * xx [ 73 ] + xx [ 210 ] * xx [
95 ] + xx [ 34 ] * ( xx [ 415 ] * xx [ 409 ] - xx [ 371 ] * xx [ 371 ] ) *
state [ 9 ] * state [ 11 ] ) * xx [ 55 ] - ( xx [ 356 ] * xx [ 159 ] - ( xx [
193 ] * xx [ 97 ] + xx [ 195 ] * xx [ 91 ] ) + ( xx [ 178 ] - xx [ 149 ] * xx
[ 169 ] ) * xx [ 6 ] + ( xx [ 232 ] - xx [ 149 ] * xx [ 179 ] ) * xx [ 34 ] -
( xx [ 91 ] * xx [ 182 ] + xx [ 185 ] * xx [ 97 ] + xx [ 145 ] * xx [ 174 ] +
xx [ 156 ] * xx [ 175 ] + xx [ 34 ] * ( xx [ 581 ] * xx [ 576 ] - xx [ 446 ]
* xx [ 446 ] ) * state [ 15 ] * state [ 17 ] ) * xx [ 55 ] ) + xx [ 401 ] *
xx [ 28 ] ) + xx [ 568 ] * xx [ 180 ] - xx [ 181 ] * xx [ 478 ] ; xx [ 229 ]
= xx [ 176 ] * xx [ 601 ] - ( xx [ 55 ] * ( xx [ 34 ] * ( xx [ 600 ] * xx [
559 ] + xx [ 587 ] * xx [ 559 ] ) * state [ 19 ] * state [ 21 ] - ( xx [ 98 ]
* xx [ 101 ] + xx [ 131 ] * xx [ 78 ] + xx [ 75 ] * xx [ 109 ] + xx [ 116 ] *
xx [ 79 ] ) ) + ( xx [ 70 ] * xx [ 143 ] + xx [ 234 ] ) * xx [ 6 ] - xx [ 154
] - ( xx [ 133 ] * xx [ 78 ] + xx [ 144 ] * xx [ 98 ] + xx [ 77 ] * xx [ 71 ]
) + xx [ 6 ] * ( xx [ 6 ] * ( xx [ 241 ] - xx [ 70 ] * xx [ 196 ] ) - xx [
150 ] ) - ( xx [ 55 ] * ( xx [ 34 ] * ( xx [ 360 ] * xx [ 479 ] + xx [ 313 ]
* xx [ 479 ] ) * state [ 25 ] * state [ 27 ] - ( xx [ 132 ] * xx [ 39 ] + xx
[ 42 ] * xx [ 23 ] + xx [ 20 ] * xx [ 194 ] + xx [ 206 ] * xx [ 24 ] ) ) + (
xx [ 4 ] * xx [ 57 ] + xx [ 266 ] ) * xx [ 6 ] - xx [ 215 ] - ( xx [ 56 ] *
xx [ 23 ] + xx [ 58 ] * xx [ 132 ] + xx [ 280 ] * xx [ 36 ] ) + xx [ 6 ] * (
xx [ 6 ] * ( xx [ 302 ] - xx [ 4 ] * xx [ 166 ] ) - xx [ 213 ] ) ) + xx [ 570
] * xx [ 161 ] ) + xx [ 268 ] * xx [ 14 ] - xx [ 27 ] * xx [ 361 ] ; xx [ 230
] = xx [ 176 ] * xx [ 569 ] - ( xx [ 340 ] * xx [ 71 ] - ( xx [ 133 ] * xx [
98 ] + xx [ 144 ] * xx [ 18 ] ) + ( xx [ 235 ] - xx [ 70 ] * xx [ 188 ] ) *
xx [ 6 ] + ( xx [ 70 ] * xx [ 198 ] + xx [ 242 ] ) * xx [ 34 ] - ( xx [ 18 ]
* xx [ 101 ] + xx [ 131 ] * xx [ 98 ] + xx [ 68 ] * xx [ 109 ] + xx [ 75 ] *
xx [ 116 ] + xx [ 34 ] * ( xx [ 600 ] * xx [ 587 ] - xx [ 559 ] * xx [ 559 ]
) * state [ 19 ] * state [ 21 ] ) * xx [ 55 ] - ( xx [ 287 ] * xx [ 36 ] - (
xx [ 56 ] * xx [ 132 ] + xx [ 58 ] * xx [ 26 ] ) + ( xx [ 267 ] - xx [ 4 ] *
xx [ 219 ] ) * xx [ 6 ] + ( xx [ 4 ] * xx [ 222 ] + xx [ 303 ] ) * xx [ 34 ]
- ( xx [ 26 ] * xx [ 39 ] + xx [ 42 ] * xx [ 132 ] + xx [ 2 ] * xx [ 194 ] +
xx [ 20 ] * xx [ 206 ] + xx [ 34 ] * ( xx [ 360 ] * xx [ 313 ] - xx [ 479 ] *
xx [ 479 ] ) * state [ 25 ] * state [ 27 ] ) * xx [ 55 ] ) + xx [ 588 ] * xx
[ 161 ] ) + xx [ 560 ] * xx [ 14 ] - xx [ 27 ] * xx [ 226 ] ; memcpy ( xx +
447 , xx + 633 , 16 * sizeof ( double ) ) ; factorAndSolveSymmetric ( xx +
447 , 4 , xx + 23 , ii + 0 , xx + 227 , xx + 14 , xx + 480 ) ; xx [ 2 ] = (
xx [ 86 ] + xx [ 361 ] * xx [ 16 ] + xx [ 17 ] * xx [ 226 ] ) / xx [ 45 ] ;
xx [ 18 ] = xx [ 221 ] - xx [ 43 ] * xx [ 2 ] ; xx [ 20 ] = xx [ 41 ] * xx [
18 ] ; xx [ 23 ] = xx [ 218 ] - ( xx [ 37 ] * xx [ 20 ] + xx [ 245 ] ) * xx [
6 ] ; xx [ 24 ] = xx [ 18 ] + xx [ 6 ] * ( xx [ 387 ] - xx [ 20 ] * xx [ 41 ]
) ; xx [ 18 ] = xx [ 671 ] + xx [ 558 ] - xx [ 51 ] * xx [ 2 ] + xx [ 58 ] *
xx [ 24 ] - xx [ 56 ] * xx [ 23 ] + xx [ 88 ] ; xx [ 20 ] = xx [ 24 ] + xx [
431 ] ; xx [ 24 ] = ( xx [ 406 ] - ( xx [ 18 ] + xx [ 20 ] * xx [ 63 ] ) - (
xx [ 268 ] * xx [ 16 ] + xx [ 17 ] * xx [ 560 ] ) ) / xx [ 62 ] ; xx [ 25 ] =
xx [ 23 ] + xx [ 49 ] + xx [ 64 ] * xx [ 24 ] ; xx [ 26 ] = xx [ 20 ] + xx [
38 ] * xx [ 24 ] ; xx [ 27 ] = xx [ 47 ] ; pm_math_Quaternion_xform_ra ( xx +
436 , xx + 25 , xx + 28 ) ; xx [ 20 ] = ( xx [ 601 ] * xx [ 16 ] + xx [ 17 ]
* xx [ 569 ] - xx [ 106 ] ) / xx [ 127 ] ; xx [ 23 ] = xx [ 249 ] + xx [ 117
] * xx [ 20 ] ; xx [ 25 ] = xx [ 0 ] + xx [ 126 ] * xx [ 20 ] ; xx [ 0 ] = xx
[ 25 ] * xx [ 130 ] ; xx [ 26 ] = xx [ 23 ] * xx [ 130 ] ; xx [ 27 ] = xx [
23 ] - ( xx [ 81 ] * xx [ 0 ] + xx [ 26 ] * xx [ 130 ] ) * xx [ 6 ] ; xx [ 23
] = xx [ 25 ] + xx [ 6 ] * ( xx [ 81 ] * xx [ 26 ] - xx [ 0 ] * xx [ 130 ] )
; xx [ 0 ] = xx [ 748 ] + xx [ 121 ] + xx [ 125 ] * xx [ 20 ] + xx [ 23 ] *
xx [ 144 ] - xx [ 133 ] * xx [ 27 ] + xx [ 50 ] ; xx [ 25 ] = xx [ 23 ] + xx
[ 108 ] ; xx [ 23 ] = ( xx [ 59 ] - ( xx [ 0 ] + xx [ 25 ] * xx [ 63 ] ) + xx
[ 570 ] * xx [ 16 ] + xx [ 17 ] * xx [ 588 ] ) / xx [ 141 ] ; xx [ 31 ] = xx
[ 27 ] + xx [ 19 ] + xx [ 146 ] * xx [ 23 ] ; xx [ 32 ] = xx [ 25 ] + xx [
140 ] * xx [ 23 ] ; xx [ 33 ] = xx [ 82 ] ; pm_math_Quaternion_xform_ra ( xx
+ 629 , xx + 31 , xx + 25 ) ; xx [ 16 ] = ( xx [ 153 ] + xx [ 582 ] * xx [ 14
] + xx [ 15 ] * xx [ 478 ] ) / xx [ 44 ] ; xx [ 17 ] = xx [ 152 ] - xx [ 43 ]
* xx [ 16 ] ; xx [ 19 ] = xx [ 184 ] * xx [ 17 ] ; xx [ 31 ] = xx [ 322 ] - (
xx [ 162 ] * xx [ 19 ] + xx [ 264 ] ) * xx [ 6 ] ; xx [ 32 ] = xx [ 17 ] + xx
[ 6 ] * ( xx [ 104 ] - xx [ 19 ] * xx [ 184 ] ) ; xx [ 17 ] = xx [ 627 ] + xx
[ 654 ] - xx [ 51 ] * xx [ 16 ] + xx [ 195 ] * xx [ 32 ] - xx [ 193 ] * xx [
31 ] + xx [ 105 ] ; xx [ 19 ] = xx [ 32 ] + xx [ 283 ] ; xx [ 32 ] = ( xx [
191 ] - ( xx [ 17 ] + xx [ 19 ] * xx [ 63 ] ) - ( xx [ 474 ] * xx [ 14 ] + xx
[ 15 ] * xx [ 568 ] ) ) / xx [ 199 ] ; xx [ 42 ] = xx [ 31 ] + xx [ 189 ] +
xx [ 200 ] * xx [ 32 ] ; xx [ 43 ] = xx [ 19 ] + xx [ 163 ] * xx [ 32 ] ; xx
[ 44 ] = xx [ 187 ] ; pm_math_Quaternion_xform_ra ( xx + 605 , xx + 42 , xx +
49 ) ; xx [ 19 ] = ( xx [ 384 ] * xx [ 14 ] + xx [ 15 ] * xx [ 398 ] - xx [
239 ] ) / xx [ 256 ] ; xx [ 31 ] = xx [ 197 ] + xx [ 250 ] * xx [ 19 ] ; xx [
33 ] = xx [ 1 ] + xx [ 255 ] * xx [ 19 ] ; xx [ 1 ] = xx [ 33 ] * xx [ 259 ]
; xx [ 34 ] = xx [ 31 ] * xx [ 259 ] ; xx [ 36 ] = xx [ 31 ] - ( xx [ 216 ] *
xx [ 1 ] + xx [ 34 ] * xx [ 259 ] ) * xx [ 6 ] ; xx [ 31 ] = xx [ 33 ] + xx [
6 ] * ( xx [ 216 ] * xx [ 34 ] - xx [ 1 ] * xx [ 259 ] ) ; xx [ 1 ] = xx [
667 ] + xx [ 355 ] + xx [ 254 ] * xx [ 19 ] + xx [ 31 ] * xx [ 273 ] - xx [
262 ] * xx [ 36 ] + xx [ 54 ] ; xx [ 33 ] = xx [ 31 ] + xx [ 217 ] ; xx [ 31
] = ( xx [ 52 ] - ( xx [ 1 ] + xx [ 33 ] * xx [ 63 ] ) + xx [ 422 ] * xx [ 14
] + xx [ 15 ] * xx [ 401 ] ) / xx [ 270 ] ; xx [ 42 ] = xx [ 36 ] + xx [ 21 ]
+ xx [ 275 ] * xx [ 31 ] ; xx [ 43 ] = xx [ 33 ] + xx [ 269 ] * xx [ 31 ] ;
xx [ 44 ] = xx [ 35 ] ; pm_math_Quaternion_xform_ra ( xx + 595 , xx + 42 , xx
+ 33 ) ; xx [ 42 ] = xx [ 89 ] ; xx [ 43 ] = xx [ 171 ] ; xx [ 44 ] = xx [ 18
] + xx [ 40 ] * xx [ 24 ] ; pm_math_Quaternion_xform_ra ( xx + 436 , xx + 42
, xx + 38 ) ; pm_math_Vector3_cross_ra ( xx + 289 , xx + 28 , xx + 42 ) ; xx
[ 14 ] = xx [ 319 ] - xx [ 321 ] * xx [ 20 ] ; xx [ 15 ] = xx [ 135 ] + xx [
329 ] * xx [ 20 ] ; xx [ 18 ] = xx [ 15 ] * xx [ 130 ] ; xx [ 21 ] = xx [ 130
] * xx [ 14 ] ; xx [ 43 ] = xx [ 746 ] + xx [ 14 ] - ( xx [ 81 ] * xx [ 18 ]
+ xx [ 21 ] * xx [ 130 ] ) * xx [ 6 ] + xx [ 3 ] + xx [ 53 ] + xx [ 339 ] *
xx [ 23 ] ; xx [ 44 ] = xx [ 747 ] + xx [ 15 ] + xx [ 6 ] * ( xx [ 81 ] * xx
[ 21 ] - xx [ 18 ] * xx [ 130 ] ) - xx [ 129 ] + xx [ 134 ] + xx [ 332 ] * xx
[ 23 ] ; xx [ 45 ] = xx [ 0 ] + xx [ 124 ] * xx [ 23 ] ;
pm_math_Quaternion_xform_ra ( xx + 629 , xx + 43 , xx + 52 ) ;
pm_math_Vector3_cross_ra ( xx + 348 , xx + 25 , xx + 43 ) ; xx [ 53 ] = xx [
107 ] ; xx [ 54 ] = xx [ 118 ] ; xx [ 55 ] = xx [ 17 ] + xx [ 183 ] * xx [ 32
] ; pm_math_Quaternion_xform_ra ( xx + 605 , xx + 53 , xx + 59 ) ;
pm_math_Vector3_cross_ra ( xx + 377 , xx + 49 , xx + 53 ) ; xx [ 0 ] = xx [
190 ] + xx [ 382 ] * xx [ 19 ] ; xx [ 3 ] = xx [ 192 ] - xx [ 407 ] * xx [ 19
] ; xx [ 14 ] = xx [ 3 ] * xx [ 259 ] ; xx [ 15 ] = xx [ 259 ] * xx [ 0 ] ;
xx [ 60 ] = xx [ 665 ] + xx [ 0 ] - ( xx [ 216 ] * xx [ 14 ] + xx [ 15 ] * xx
[ 259 ] ) * xx [ 6 ] + xx [ 103 ] + xx [ 147 ] + xx [ 417 ] * xx [ 31 ] ; xx
[ 61 ] = xx [ 666 ] + xx [ 3 ] + xx [ 6 ] * ( xx [ 216 ] * xx [ 15 ] - xx [
14 ] * xx [ 259 ] ) - xx [ 237 ] + xx [ 238 ] + xx [ 410 ] * xx [ 31 ] ; xx [
62 ] = xx [ 1 ] + xx [ 253 ] * xx [ 31 ] ; pm_math_Quaternion_xform_ra ( xx +
595 , xx + 60 , xx + 77 ) ; pm_math_Vector3_cross_ra ( xx + 426 , xx + 33 ,
xx + 60 ) ; xx [ 0 ] = xx [ 29 ] + xx [ 26 ] + xx [ 50 ] + xx [ 34 ] + xx [
76 ] ; xx [ 1 ] = ( xx [ 38 ] + xx [ 42 ] + xx [ 52 ] + xx [ 43 ] + xx [ 59 ]
+ xx [ 53 ] + xx [ 77 ] + xx [ 60 ] + xx [ 65 ] + xx [ 0 ] * xx [ 84 ] ) / xx
[ 168 ] ; xx [ 38 ] = xx [ 28 ] + xx [ 25 ] + xx [ 49 ] + xx [ 33 ] + xx [
110 ] - xx [ 165 ] * xx [ 1 ] ; xx [ 39 ] = xx [ 0 ] - xx [ 87 ] * xx [ 1 ] ;
xx [ 40 ] = xx [ 30 ] + xx [ 27 ] + xx [ 51 ] + xx [ 35 ] + xx [ 155 ] - xx [
90 ] * xx [ 1 ] ; pm_math_Quaternion_xform_ra ( xx + 294 , xx + 38 , xx + 25
) ; xx [ 28 ] = - xx [ 25 ] ; xx [ 29 ] = - xx [ 26 ] ; xx [ 30 ] = - xx [ 27
] ; solveSymmetricPosDef ( xx + 544 , xx + 28 , 3 , 1 , xx + 25 , xx + 33 ) ;
xx [ 0 ] = xx [ 25 ] + xx [ 11 ] ; xx [ 3 ] = xx [ 26 ] + xx [ 12 ] ; xx [ 11
] = xx [ 27 ] + xx [ 13 ] ; xx [ 12 ] = xx [ 0 ] - xx [ 102 ] ; xx [ 13 ] =
xx [ 3 ] ; xx [ 14 ] = xx [ 11 ] - xx [ 10 ] ;
pm_math_Quaternion_inverseXform_ra ( xx + 294 , xx + 12 , xx + 25 ) ; xx [ 10
] = xx [ 1 ] + pm_math_Vector3_dot_ra ( xx + 589 , xx + 25 ) ; xx [ 1 ] = xx
[ 10 ] * xx [ 205 ] ; xx [ 12 ] = xx [ 10 ] * xx [ 207 ] ; xx [ 13 ] = ( xx [
1 ] * xx [ 205 ] + xx [ 12 ] * xx [ 207 ] ) * xx [ 6 ] ; xx [ 14 ] = xx [ 13
] - xx [ 10 ] ; xx [ 15 ] = xx [ 6 ] * ( xx [ 12 ] * xx [ 205 ] - xx [ 1 ] *
xx [ 207 ] ) ; xx [ 28 ] = xx [ 14 ] ; xx [ 29 ] = xx [ 15 ] ; xx [ 30 ] = -
xx [ 13 ] ; xx [ 1 ] = xx [ 26 ] - xx [ 10 ] * xx [ 84 ] ; xx [ 12 ] = xx [
27 ] + xx [ 7 ] ; xx [ 33 ] = xx [ 25 ] ; xx [ 34 ] = xx [ 1 ] + xx [ 10 ] *
xx [ 418 ] ; xx [ 35 ] = xx [ 12 ] - xx [ 10 ] * xx [ 209 ] ;
pm_math_Quaternion_inverseXform_ra ( xx + 595 , xx + 33 , xx + 38 ) ; xx [ 7
] = xx [ 31 ] - ( pm_math_Vector3_dot_ra ( xx + 562 , xx + 28 ) + xx [ 271 ]
* xx [ 38 ] + xx [ 277 ] * xx [ 39 ] ) ; xx [ 17 ] = xx [ 14 ] + xx [ 120 ] ;
xx [ 14 ] = xx [ 15 ] - xx [ 167 ] ; xx [ 15 ] = xx [ 259 ] * xx [ 14 ] ; xx
[ 18 ] = xx [ 17 ] * xx [ 259 ] ; xx [ 21 ] = xx [ 7 ] - xx [ 13 ] ; xx [ 26
] = xx [ 17 ] + xx [ 6 ] * ( xx [ 216 ] * xx [ 15 ] - xx [ 18 ] * xx [ 259 ]
) ; xx [ 27 ] = xx [ 14 ] - ( xx [ 216 ] * xx [ 18 ] + xx [ 15 ] * xx [ 259 ]
) * xx [ 6 ] ; xx [ 28 ] = xx [ 21 ] ; xx [ 13 ] = xx [ 38 ] + xx [ 243 ] -
xx [ 262 ] * xx [ 21 ] ; xx [ 14 ] = xx [ 39 ] + xx [ 63 ] * xx [ 7 ] + xx [
48 ] + xx [ 273 ] * xx [ 21 ] ; xx [ 15 ] = xx [ 14 ] * xx [ 259 ] ; xx [ 17
] = xx [ 13 ] * xx [ 259 ] ; xx [ 18 ] = xx [ 19 ] - ( pm_math_Vector3_dot_ra
( xx + 584 , xx + 26 ) + ( xx [ 13 ] + xx [ 6 ] * ( xx [ 216 ] * xx [ 15 ] -
xx [ 17 ] * xx [ 259 ] ) ) * xx [ 257 ] + xx [ 248 ] * ( xx [ 14 ] - ( xx [
216 ] * xx [ 17 ] + xx [ 15 ] * xx [ 259 ] ) * xx [ 6 ] ) ) ; xx [ 13 ] = xx
[ 21 ] + xx [ 18 ] ; xx [ 26 ] = xx [ 25 ] ; xx [ 27 ] = xx [ 1 ] + xx [ 10 ]
* xx [ 357 ] ; xx [ 28 ] = xx [ 12 ] - xx [ 10 ] * xx [ 208 ] ;
pm_math_Quaternion_inverseXform_ra ( xx + 605 , xx + 26 , xx + 29 ) ; xx [ 14
] = xx [ 10 ] * xx [ 149 ] ; xx [ 15 ] = xx [ 10 ] * xx [ 151 ] ; xx [ 17 ] =
( xx [ 14 ] * xx [ 149 ] + xx [ 15 ] * xx [ 151 ] ) * xx [ 6 ] ; xx [ 19 ] =
xx [ 32 ] - ( xx [ 202 ] * xx [ 29 ] + xx [ 204 ] * xx [ 30 ] - xx [ 139 ] *
xx [ 17 ] ) ; xx [ 21 ] = xx [ 19 ] - xx [ 17 ] ; xx [ 26 ] = xx [ 30 ] + xx
[ 63 ] * xx [ 19 ] + xx [ 8 ] + xx [ 195 ] * xx [ 21 ] ; xx [ 8 ] = xx [ 16 ]
+ xx [ 201 ] * xx [ 21 ] + xx [ 186 ] * ( xx [ 26 ] - ( xx [ 162 ] * ( xx [
29 ] + xx [ 136 ] - xx [ 193 ] * xx [ 21 ] ) * xx [ 184 ] + xx [ 26 ] * xx [
184 ] * xx [ 184 ] ) * xx [ 6 ] ) ; xx [ 16 ] = xx [ 10 ] * xx [ 70 ] ; xx [
26 ] = xx [ 10 ] * xx [ 72 ] ; xx [ 27 ] = ( xx [ 16 ] * xx [ 70 ] + xx [ 26
] * xx [ 72 ] ) * xx [ 6 ] ; xx [ 28 ] = xx [ 27 ] - xx [ 10 ] ; xx [ 29 ] =
xx [ 6 ] * ( xx [ 26 ] * xx [ 70 ] - xx [ 16 ] * xx [ 72 ] ) ; xx [ 30 ] = xx
[ 28 ] ; xx [ 31 ] = xx [ 29 ] ; xx [ 32 ] = - xx [ 27 ] ; xx [ 33 ] = xx [
25 ] ; xx [ 34 ] = xx [ 1 ] + xx [ 10 ] * xx [ 341 ] ; xx [ 35 ] = xx [ 12 ]
- xx [ 10 ] * xx [ 74 ] ; pm_math_Quaternion_inverseXform_ra ( xx + 629 , xx
+ 33 , xx + 38 ) ; xx [ 16 ] = xx [ 23 ] - ( pm_math_Vector3_dot_ra ( xx +
616 , xx + 30 ) + xx [ 142 ] * xx [ 38 ] + xx [ 148 ] * xx [ 39 ] ) ; xx [ 23
] = xx [ 28 ] + xx [ 119 ] ; xx [ 26 ] = xx [ 29 ] - xx [ 123 ] ; xx [ 28 ] =
xx [ 130 ] * xx [ 26 ] ; xx [ 29 ] = xx [ 23 ] * xx [ 130 ] ; xx [ 30 ] = xx
[ 16 ] - xx [ 27 ] ; xx [ 31 ] = xx [ 23 ] + xx [ 6 ] * ( xx [ 81 ] * xx [ 28
] - xx [ 29 ] * xx [ 130 ] ) ; xx [ 32 ] = xx [ 26 ] - ( xx [ 81 ] * xx [ 29
] + xx [ 28 ] * xx [ 130 ] ) * xx [ 6 ] ; xx [ 33 ] = xx [ 30 ] ; xx [ 23 ] =
xx [ 38 ] + xx [ 279 ] - xx [ 133 ] * xx [ 30 ] ; xx [ 26 ] = xx [ 39 ] + xx
[ 63 ] * xx [ 16 ] + xx [ 5 ] + xx [ 144 ] * xx [ 30 ] ; xx [ 5 ] = xx [ 26 ]
* xx [ 130 ] ; xx [ 27 ] = xx [ 23 ] * xx [ 130 ] ; xx [ 28 ] = xx [ 20 ] - (
pm_math_Vector3_dot_ra ( xx + 619 , xx + 31 ) + ( xx [ 23 ] + xx [ 6 ] * ( xx
[ 81 ] * xx [ 5 ] - xx [ 27 ] * xx [ 130 ] ) ) * xx [ 128 ] + xx [ 115 ] * (
xx [ 26 ] - ( xx [ 81 ] * xx [ 27 ] + xx [ 5 ] * xx [ 130 ] ) * xx [ 6 ] ) )
; xx [ 5 ] = xx [ 30 ] + xx [ 28 ] ; xx [ 29 ] = xx [ 25 ] ; xx [ 30 ] = xx [
1 ] + xx [ 10 ] * xx [ 288 ] ; xx [ 31 ] = xx [ 12 ] - xx [ 10 ] * xx [ 285 ]
; pm_math_Quaternion_inverseXform_ra ( xx + 436 , xx + 29 , xx + 25 ) ; xx [
1 ] = xx [ 10 ] * xx [ 4 ] ; xx [ 12 ] = xx [ 10 ] * xx [ 9 ] ; xx [ 20 ] = (
xx [ 1 ] * xx [ 4 ] + xx [ 12 ] * xx [ 9 ] ) * xx [ 6 ] ; xx [ 23 ] = xx [ 24
] - ( xx [ 67 ] * xx [ 25 ] + xx [ 69 ] * xx [ 26 ] - xx [ 432 ] * xx [ 20 ]
) ; xx [ 24 ] = xx [ 23 ] - xx [ 20 ] ; xx [ 27 ] = xx [ 26 ] + xx [ 63 ] *
xx [ 23 ] + xx [ 83 ] + xx [ 58 ] * xx [ 24 ] ; xx [ 26 ] = xx [ 2 ] + xx [
66 ] * xx [ 24 ] + xx [ 46 ] * ( xx [ 27 ] - ( xx [ 37 ] * ( xx [ 25 ] + xx [
388 ] - xx [ 56 ] * xx [ 24 ] ) * xx [ 41 ] + xx [ 27 ] * xx [ 41 ] * xx [ 41
] ) * xx [ 6 ] ) ; xx [ 29 ] = xx [ 446 ] ; xx [ 30 ] = xx [ 581 ] ; xx [ 31
] = xx [ 446 ] ; xx [ 32 ] = xx [ 576 ] ; xx [ 33 ] = xx [ 371 ] ; xx [ 34 ]
= xx [ 415 ] ; xx [ 35 ] = xx [ 371 ] ; xx [ 36 ] = xx [ 409 ] ;
pm_math_Quaternion_inverseCompose_ra ( xx + 29 , xx + 33 , xx + 42 ) ; xx [ 2
] = xx [ 17 ] - xx [ 10 ] + xx [ 114 ] ; xx [ 17 ] = xx [ 6 ] * ( xx [ 15 ] *
xx [ 149 ] - xx [ 14 ] * xx [ 151 ] ) - xx [ 111 ] ; xx [ 14 ] = xx [ 184 ] *
xx [ 17 ] ; xx [ 15 ] = xx [ 2 ] * xx [ 184 ] ; xx [ 29 ] = xx [ 2 ] + xx [ 6
] * ( xx [ 162 ] * xx [ 14 ] - xx [ 15 ] * xx [ 184 ] ) + xx [ 112 ] ; xx [
30 ] = xx [ 17 ] - ( xx [ 162 ] * xx [ 15 ] + xx [ 14 ] * xx [ 184 ] ) * xx [
6 ] - xx [ 137 ] ; xx [ 31 ] = xx [ 21 ] - xx [ 8 ] ;
pm_math_Quaternion_inverseXform_ra ( xx + 42 , xx + 29 , xx + 32 ) ; xx [ 29
] = xx [ 479 ] ; xx [ 30 ] = - xx [ 360 ] ; xx [ 31 ] = xx [ 479 ] ; xx [ 32
] = - xx [ 313 ] ; xx [ 42 ] = xx [ 559 ] ; xx [ 43 ] = - xx [ 600 ] ; xx [
44 ] = xx [ 559 ] ; xx [ 45 ] = - xx [ 587 ] ;
pm_math_Quaternion_inverseCompose_ra ( xx + 29 , xx + 42 , xx + 46 ) ; xx [ 2
] = xx [ 20 ] - xx [ 10 ] + xx [ 211 ] ; xx [ 14 ] = xx [ 6 ] * ( xx [ 12 ] *
xx [ 4 ] - xx [ 1 ] * xx [ 9 ] ) - xx [ 164 ] ; xx [ 1 ] = xx [ 41 ] * xx [
14 ] ; xx [ 4 ] = xx [ 2 ] * xx [ 41 ] ; xx [ 29 ] = xx [ 2 ] + xx [ 6 ] * (
xx [ 37 ] * xx [ 1 ] - xx [ 4 ] * xx [ 41 ] ) + xx [ 170 ] ; xx [ 30 ] = xx [
14 ] - ( xx [ 37 ] * xx [ 4 ] + xx [ 1 ] * xx [ 41 ] ) * xx [ 6 ] - xx [ 172
] ; xx [ 31 ] = xx [ 24 ] - xx [ 26 ] ; pm_math_Quaternion_inverseXform_ra (
xx + 46 , xx + 29 , xx + 35 ) ; deriv [ 0 ] = state [ 3 ] ; deriv [ 1 ] =
state [ 4 ] ; deriv [ 2 ] = state [ 5 ] ; deriv [ 3 ] = xx [ 0 ] ; deriv [ 4
] = xx [ 3 ] ; deriv [ 5 ] = xx [ 11 ] ; deriv [ 6 ] = state [ 7 ] ; deriv [
7 ] = - xx [ 10 ] ; deriv [ 8 ] = state [ 9 ] ; deriv [ 9 ] = xx [ 7 ] ;
deriv [ 10 ] = state [ 11 ] ; deriv [ 11 ] = xx [ 18 ] ; deriv [ 12 ] = state
[ 13 ] ; deriv [ 13 ] = xx [ 286 ] - xx [ 13 ] * xx [ 251 ] ; deriv [ 14 ] =
state [ 15 ] ; deriv [ 15 ] = xx [ 19 ] ; deriv [ 16 ] = state [ 17 ] ; deriv
[ 17 ] = - xx [ 8 ] ; deriv [ 18 ] = state [ 19 ] ; deriv [ 19 ] = xx [ 16 ]
; deriv [ 20 ] = state [ 21 ] ; deriv [ 21 ] = xx [ 28 ] ; deriv [ 22 ] =
state [ 23 ] ; deriv [ 23 ] = xx [ 22 ] - xx [ 5 ] * xx [ 122 ] ; deriv [ 24
] = state [ 25 ] ; deriv [ 25 ] = xx [ 23 ] ; deriv [ 26 ] = state [ 27 ] ;
deriv [ 27 ] = - xx [ 26 ] ; deriv [ 28 ] = state [ 29 ] ; deriv [ 29 ] = xx
[ 13 ] - xx [ 34 ] ; deriv [ 30 ] = state [ 31 ] ; deriv [ 31 ] = xx [ 5 ] -
xx [ 37 ] ; errorResult [ 0 ] = xx [ 284 ] ; return NULL ; } PmfMessageId
wheel_leg_d9e82f16_1_numJacPerturbLoBounds ( const RuntimeDerivedValuesBundle
* rtdv , const int * eqnEnableFlags , const double * state , const int *
modeVector , const double * input , const double * inputDot , const double *
inputDdot , const double * discreteState , double * bounds , double *
errorResult , NeuDiagnosticManager * neDiagMgr ) { const double * rtdvd =
rtdv -> mDoubles . mValues ; const int * rtdvi = rtdv -> mInts . mValues ;
double xx [ 2 ] ; ( void ) rtdvd ; ( void ) rtdvi ; ( void ) eqnEnableFlags ;
( void ) state ; ( void ) modeVector ; ( void ) input ; ( void ) inputDot ; (
void ) inputDdot ; ( void ) discreteState ; ( void ) neDiagMgr ; xx [ 0 ] =
1.0e-9 ; xx [ 1 ] = 1.0e-8 ; bounds [ 0 ] = xx [ 0 ] ; bounds [ 1 ] = xx [ 0
] ; bounds [ 2 ] = xx [ 0 ] ; bounds [ 3 ] = xx [ 0 ] ; bounds [ 4 ] = xx [ 0
] ; bounds [ 5 ] = xx [ 0 ] ; bounds [ 6 ] = xx [ 1 ] ; bounds [ 7 ] = xx [ 1
] ; bounds [ 8 ] = xx [ 1 ] ; bounds [ 9 ] = xx [ 1 ] ; bounds [ 10 ] = xx [
1 ] ; bounds [ 11 ] = xx [ 1 ] ; bounds [ 12 ] = xx [ 1 ] ; bounds [ 13 ] =
xx [ 1 ] ; bounds [ 14 ] = xx [ 1 ] ; bounds [ 15 ] = xx [ 1 ] ; bounds [ 16
] = xx [ 1 ] ; bounds [ 17 ] = xx [ 1 ] ; bounds [ 18 ] = xx [ 1 ] ; bounds [
19 ] = xx [ 1 ] ; bounds [ 20 ] = xx [ 1 ] ; bounds [ 21 ] = xx [ 1 ] ;
bounds [ 22 ] = xx [ 1 ] ; bounds [ 23 ] = xx [ 1 ] ; bounds [ 24 ] = xx [ 1
] ; bounds [ 25 ] = xx [ 1 ] ; bounds [ 26 ] = xx [ 1 ] ; bounds [ 27 ] = xx
[ 1 ] ; bounds [ 28 ] = xx [ 1 ] ; bounds [ 29 ] = xx [ 1 ] ; bounds [ 30 ] =
xx [ 1 ] ; bounds [ 31 ] = xx [ 1 ] ; errorResult [ 0 ] = 0.0 ; return NULL ;
} PmfMessageId wheel_leg_d9e82f16_1_numJacPerturbHiBounds ( const
RuntimeDerivedValuesBundle * rtdv , const int * eqnEnableFlags , const double
* state , const int * modeVector , const double * input , const double *
inputDot , const double * inputDdot , const double * discreteState , double *
bounds , double * errorResult , NeuDiagnosticManager * neDiagMgr ) { const
double * rtdvd = rtdv -> mDoubles . mValues ; const int * rtdvi = rtdv ->
mInts . mValues ; double xx [ 2 ] ; ( void ) rtdvd ; ( void ) rtdvi ; ( void
) eqnEnableFlags ; ( void ) state ; ( void ) modeVector ; ( void ) input ; (
void ) inputDot ; ( void ) inputDdot ; ( void ) discreteState ; ( void )
neDiagMgr ; xx [ 0 ] = + pmf_get_inf ( ) ; xx [ 1 ] = 1.0 ; bounds [ 0 ] = xx
[ 0 ] ; bounds [ 1 ] = xx [ 0 ] ; bounds [ 2 ] = xx [ 0 ] ; bounds [ 3 ] = xx
[ 0 ] ; bounds [ 4 ] = xx [ 0 ] ; bounds [ 5 ] = xx [ 0 ] ; bounds [ 6 ] = xx
[ 1 ] ; bounds [ 7 ] = xx [ 0 ] ; bounds [ 8 ] = xx [ 1 ] ; bounds [ 9 ] = xx
[ 0 ] ; bounds [ 10 ] = xx [ 1 ] ; bounds [ 11 ] = xx [ 0 ] ; bounds [ 12 ] =
xx [ 1 ] ; bounds [ 13 ] = xx [ 0 ] ; bounds [ 14 ] = xx [ 1 ] ; bounds [ 15
] = xx [ 0 ] ; bounds [ 16 ] = xx [ 1 ] ; bounds [ 17 ] = xx [ 0 ] ; bounds [
18 ] = xx [ 1 ] ; bounds [ 19 ] = xx [ 0 ] ; bounds [ 20 ] = xx [ 1 ] ;
bounds [ 21 ] = xx [ 0 ] ; bounds [ 22 ] = xx [ 1 ] ; bounds [ 23 ] = xx [ 0
] ; bounds [ 24 ] = xx [ 1 ] ; bounds [ 25 ] = xx [ 0 ] ; bounds [ 26 ] = xx
[ 1 ] ; bounds [ 27 ] = xx [ 0 ] ; bounds [ 28 ] = xx [ 1 ] ; bounds [ 29 ] =
xx [ 0 ] ; bounds [ 30 ] = xx [ 1 ] ; bounds [ 31 ] = xx [ 0 ] ; errorResult
[ 0 ] = 0.0 ; return NULL ; }
