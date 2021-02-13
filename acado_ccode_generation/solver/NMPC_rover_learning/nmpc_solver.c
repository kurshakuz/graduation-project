/*
 *    This file was auto-generated using the ACADO Toolkit.
 *    
 *    While ACADO Toolkit is free software released under the terms of
 *    the GNU Lesser General Public License (LGPL), the generated code
 *    as such remains the property of the user who used ACADO Toolkit
 *    to generate this code. In particular, user dependent data of the code
 *    do not inherit the GNU LGPL license. On the other hand, parts of the
 *    generated code that are a direct copy of source code from the
 *    ACADO Toolkit or the software tools it is based on, remain, as derived
 *    work, automatically covered by the LGPL license.
 *    
 *    ACADO Toolkit is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *    
 */


#include "nmpc_common.h"




/******************************************************************************/
/*                                                                            */
/* ACADO code generation                                                      */
/*                                                                            */
/******************************************************************************/


int nmpc_modelSimulation(  )
{
int ret;

int lRun1;
ret = 0;
for (lRun1 = 0; lRun1 < 30; ++lRun1)
{
nmpcWorkspace.state[0] = nmpcVariables.x[lRun1 * 5];
nmpcWorkspace.state[1] = nmpcVariables.x[lRun1 * 5 + 1];
nmpcWorkspace.state[2] = nmpcVariables.x[lRun1 * 5 + 2];
nmpcWorkspace.state[3] = nmpcVariables.x[lRun1 * 5 + 3];
nmpcWorkspace.state[4] = nmpcVariables.x[lRun1 * 5 + 4];

nmpcWorkspace.state[40] = nmpcVariables.u[lRun1 * 2];
nmpcWorkspace.state[41] = nmpcVariables.u[lRun1 * 2 + 1];
nmpcWorkspace.state[42] = nmpcVariables.od[lRun1 * 2];
nmpcWorkspace.state[43] = nmpcVariables.od[lRun1 * 2 + 1];

ret = nmpc_integrate(nmpcWorkspace.state, 1);

nmpcWorkspace.d[lRun1 * 5] = nmpcWorkspace.state[0] - nmpcVariables.x[lRun1 * 5 + 5];
nmpcWorkspace.d[lRun1 * 5 + 1] = nmpcWorkspace.state[1] - nmpcVariables.x[lRun1 * 5 + 6];
nmpcWorkspace.d[lRun1 * 5 + 2] = nmpcWorkspace.state[2] - nmpcVariables.x[lRun1 * 5 + 7];
nmpcWorkspace.d[lRun1 * 5 + 3] = nmpcWorkspace.state[3] - nmpcVariables.x[lRun1 * 5 + 8];
nmpcWorkspace.d[lRun1 * 5 + 4] = nmpcWorkspace.state[4] - nmpcVariables.x[lRun1 * 5 + 9];

nmpcWorkspace.evGx[lRun1 * 25] = nmpcWorkspace.state[5];
nmpcWorkspace.evGx[lRun1 * 25 + 1] = nmpcWorkspace.state[6];
nmpcWorkspace.evGx[lRun1 * 25 + 2] = nmpcWorkspace.state[7];
nmpcWorkspace.evGx[lRun1 * 25 + 3] = nmpcWorkspace.state[8];
nmpcWorkspace.evGx[lRun1 * 25 + 4] = nmpcWorkspace.state[9];
nmpcWorkspace.evGx[lRun1 * 25 + 5] = nmpcWorkspace.state[10];
nmpcWorkspace.evGx[lRun1 * 25 + 6] = nmpcWorkspace.state[11];
nmpcWorkspace.evGx[lRun1 * 25 + 7] = nmpcWorkspace.state[12];
nmpcWorkspace.evGx[lRun1 * 25 + 8] = nmpcWorkspace.state[13];
nmpcWorkspace.evGx[lRun1 * 25 + 9] = nmpcWorkspace.state[14];
nmpcWorkspace.evGx[lRun1 * 25 + 10] = nmpcWorkspace.state[15];
nmpcWorkspace.evGx[lRun1 * 25 + 11] = nmpcWorkspace.state[16];
nmpcWorkspace.evGx[lRun1 * 25 + 12] = nmpcWorkspace.state[17];
nmpcWorkspace.evGx[lRun1 * 25 + 13] = nmpcWorkspace.state[18];
nmpcWorkspace.evGx[lRun1 * 25 + 14] = nmpcWorkspace.state[19];
nmpcWorkspace.evGx[lRun1 * 25 + 15] = nmpcWorkspace.state[20];
nmpcWorkspace.evGx[lRun1 * 25 + 16] = nmpcWorkspace.state[21];
nmpcWorkspace.evGx[lRun1 * 25 + 17] = nmpcWorkspace.state[22];
nmpcWorkspace.evGx[lRun1 * 25 + 18] = nmpcWorkspace.state[23];
nmpcWorkspace.evGx[lRun1 * 25 + 19] = nmpcWorkspace.state[24];
nmpcWorkspace.evGx[lRun1 * 25 + 20] = nmpcWorkspace.state[25];
nmpcWorkspace.evGx[lRun1 * 25 + 21] = nmpcWorkspace.state[26];
nmpcWorkspace.evGx[lRun1 * 25 + 22] = nmpcWorkspace.state[27];
nmpcWorkspace.evGx[lRun1 * 25 + 23] = nmpcWorkspace.state[28];
nmpcWorkspace.evGx[lRun1 * 25 + 24] = nmpcWorkspace.state[29];

nmpcWorkspace.evGu[lRun1 * 10] = nmpcWorkspace.state[30];
nmpcWorkspace.evGu[lRun1 * 10 + 1] = nmpcWorkspace.state[31];
nmpcWorkspace.evGu[lRun1 * 10 + 2] = nmpcWorkspace.state[32];
nmpcWorkspace.evGu[lRun1 * 10 + 3] = nmpcWorkspace.state[33];
nmpcWorkspace.evGu[lRun1 * 10 + 4] = nmpcWorkspace.state[34];
nmpcWorkspace.evGu[lRun1 * 10 + 5] = nmpcWorkspace.state[35];
nmpcWorkspace.evGu[lRun1 * 10 + 6] = nmpcWorkspace.state[36];
nmpcWorkspace.evGu[lRun1 * 10 + 7] = nmpcWorkspace.state[37];
nmpcWorkspace.evGu[lRun1 * 10 + 8] = nmpcWorkspace.state[38];
nmpcWorkspace.evGu[lRun1 * 10 + 9] = nmpcWorkspace.state[39];
}
return ret;
}

void nmpc_evaluateLSQ(const real_t* in, real_t* out)
{
const real_t* xd = in;
const real_t* u = in + 5;

/* Compute outputs: */
out[0] = xd[0];
out[1] = xd[1];
out[2] = xd[2];
out[3] = xd[3];
out[4] = xd[4];
out[5] = u[0];
out[6] = u[1];
}

void nmpc_evaluateLSQEndTerm(const real_t* in, real_t* out)
{
const real_t* xd = in;

/* Compute outputs: */
out[0] = xd[0];
out[1] = xd[1];
out[2] = xd[2];
out[3] = xd[3];
out[4] = xd[4];
}

void nmpc_setObjQ1Q2( real_t* const tmpObjS, real_t* const tmpQ1, real_t* const tmpQ2 )
{
tmpQ2[0] = +tmpObjS[0];
tmpQ2[1] = +tmpObjS[1];
tmpQ2[2] = +tmpObjS[2];
tmpQ2[3] = +tmpObjS[3];
tmpQ2[4] = +tmpObjS[4];
tmpQ2[5] = +tmpObjS[5];
tmpQ2[6] = +tmpObjS[6];
tmpQ2[7] = +tmpObjS[7];
tmpQ2[8] = +tmpObjS[8];
tmpQ2[9] = +tmpObjS[9];
tmpQ2[10] = +tmpObjS[10];
tmpQ2[11] = +tmpObjS[11];
tmpQ2[12] = +tmpObjS[12];
tmpQ2[13] = +tmpObjS[13];
tmpQ2[14] = +tmpObjS[14];
tmpQ2[15] = +tmpObjS[15];
tmpQ2[16] = +tmpObjS[16];
tmpQ2[17] = +tmpObjS[17];
tmpQ2[18] = +tmpObjS[18];
tmpQ2[19] = +tmpObjS[19];
tmpQ2[20] = +tmpObjS[20];
tmpQ2[21] = +tmpObjS[21];
tmpQ2[22] = +tmpObjS[22];
tmpQ2[23] = +tmpObjS[23];
tmpQ2[24] = +tmpObjS[24];
tmpQ2[25] = +tmpObjS[25];
tmpQ2[26] = +tmpObjS[26];
tmpQ2[27] = +tmpObjS[27];
tmpQ2[28] = +tmpObjS[28];
tmpQ2[29] = +tmpObjS[29];
tmpQ2[30] = +tmpObjS[30];
tmpQ2[31] = +tmpObjS[31];
tmpQ2[32] = +tmpObjS[32];
tmpQ2[33] = +tmpObjS[33];
tmpQ2[34] = +tmpObjS[34];
tmpQ1[0] = + tmpQ2[0];
tmpQ1[1] = + tmpQ2[1];
tmpQ1[2] = + tmpQ2[2];
tmpQ1[3] = + tmpQ2[3];
tmpQ1[4] = + tmpQ2[4];
tmpQ1[5] = + tmpQ2[7];
tmpQ1[6] = + tmpQ2[8];
tmpQ1[7] = + tmpQ2[9];
tmpQ1[8] = + tmpQ2[10];
tmpQ1[9] = + tmpQ2[11];
tmpQ1[10] = + tmpQ2[14];
tmpQ1[11] = + tmpQ2[15];
tmpQ1[12] = + tmpQ2[16];
tmpQ1[13] = + tmpQ2[17];
tmpQ1[14] = + tmpQ2[18];
tmpQ1[15] = + tmpQ2[21];
tmpQ1[16] = + tmpQ2[22];
tmpQ1[17] = + tmpQ2[23];
tmpQ1[18] = + tmpQ2[24];
tmpQ1[19] = + tmpQ2[25];
tmpQ1[20] = + tmpQ2[28];
tmpQ1[21] = + tmpQ2[29];
tmpQ1[22] = + tmpQ2[30];
tmpQ1[23] = + tmpQ2[31];
tmpQ1[24] = + tmpQ2[32];
}

void nmpc_setObjR1R2( real_t* const tmpObjS, real_t* const tmpR1, real_t* const tmpR2 )
{
tmpR2[0] = +tmpObjS[35];
tmpR2[1] = +tmpObjS[36];
tmpR2[2] = +tmpObjS[37];
tmpR2[3] = +tmpObjS[38];
tmpR2[4] = +tmpObjS[39];
tmpR2[5] = +tmpObjS[40];
tmpR2[6] = +tmpObjS[41];
tmpR2[7] = +tmpObjS[42];
tmpR2[8] = +tmpObjS[43];
tmpR2[9] = +tmpObjS[44];
tmpR2[10] = +tmpObjS[45];
tmpR2[11] = +tmpObjS[46];
tmpR2[12] = +tmpObjS[47];
tmpR2[13] = +tmpObjS[48];
tmpR1[0] = + tmpR2[5];
tmpR1[1] = + tmpR2[6];
tmpR1[2] = + tmpR2[12];
tmpR1[3] = + tmpR2[13];
}

void nmpc_setObjQN1QN2( real_t* const tmpObjSEndTerm, real_t* const tmpQN1, real_t* const tmpQN2 )
{
tmpQN2[0] = +tmpObjSEndTerm[0];
tmpQN2[1] = +tmpObjSEndTerm[1];
tmpQN2[2] = +tmpObjSEndTerm[2];
tmpQN2[3] = +tmpObjSEndTerm[3];
tmpQN2[4] = +tmpObjSEndTerm[4];
tmpQN2[5] = +tmpObjSEndTerm[5];
tmpQN2[6] = +tmpObjSEndTerm[6];
tmpQN2[7] = +tmpObjSEndTerm[7];
tmpQN2[8] = +tmpObjSEndTerm[8];
tmpQN2[9] = +tmpObjSEndTerm[9];
tmpQN2[10] = +tmpObjSEndTerm[10];
tmpQN2[11] = +tmpObjSEndTerm[11];
tmpQN2[12] = +tmpObjSEndTerm[12];
tmpQN2[13] = +tmpObjSEndTerm[13];
tmpQN2[14] = +tmpObjSEndTerm[14];
tmpQN2[15] = +tmpObjSEndTerm[15];
tmpQN2[16] = +tmpObjSEndTerm[16];
tmpQN2[17] = +tmpObjSEndTerm[17];
tmpQN2[18] = +tmpObjSEndTerm[18];
tmpQN2[19] = +tmpObjSEndTerm[19];
tmpQN2[20] = +tmpObjSEndTerm[20];
tmpQN2[21] = +tmpObjSEndTerm[21];
tmpQN2[22] = +tmpObjSEndTerm[22];
tmpQN2[23] = +tmpObjSEndTerm[23];
tmpQN2[24] = +tmpObjSEndTerm[24];
tmpQN1[0] = + tmpQN2[0];
tmpQN1[1] = + tmpQN2[1];
tmpQN1[2] = + tmpQN2[2];
tmpQN1[3] = + tmpQN2[3];
tmpQN1[4] = + tmpQN2[4];
tmpQN1[5] = + tmpQN2[5];
tmpQN1[6] = + tmpQN2[6];
tmpQN1[7] = + tmpQN2[7];
tmpQN1[8] = + tmpQN2[8];
tmpQN1[9] = + tmpQN2[9];
tmpQN1[10] = + tmpQN2[10];
tmpQN1[11] = + tmpQN2[11];
tmpQN1[12] = + tmpQN2[12];
tmpQN1[13] = + tmpQN2[13];
tmpQN1[14] = + tmpQN2[14];
tmpQN1[15] = + tmpQN2[15];
tmpQN1[16] = + tmpQN2[16];
tmpQN1[17] = + tmpQN2[17];
tmpQN1[18] = + tmpQN2[18];
tmpQN1[19] = + tmpQN2[19];
tmpQN1[20] = + tmpQN2[20];
tmpQN1[21] = + tmpQN2[21];
tmpQN1[22] = + tmpQN2[22];
tmpQN1[23] = + tmpQN2[23];
tmpQN1[24] = + tmpQN2[24];
}

void nmpc_evaluateObjective(  )
{
int runObj;
for (runObj = 0; runObj < 30; ++runObj)
{
nmpcWorkspace.objValueIn[0] = nmpcVariables.x[runObj * 5];
nmpcWorkspace.objValueIn[1] = nmpcVariables.x[runObj * 5 + 1];
nmpcWorkspace.objValueIn[2] = nmpcVariables.x[runObj * 5 + 2];
nmpcWorkspace.objValueIn[3] = nmpcVariables.x[runObj * 5 + 3];
nmpcWorkspace.objValueIn[4] = nmpcVariables.x[runObj * 5 + 4];
nmpcWorkspace.objValueIn[5] = nmpcVariables.u[runObj * 2];
nmpcWorkspace.objValueIn[6] = nmpcVariables.u[runObj * 2 + 1];
nmpcWorkspace.objValueIn[7] = nmpcVariables.od[runObj * 2];
nmpcWorkspace.objValueIn[8] = nmpcVariables.od[runObj * 2 + 1];

nmpc_evaluateLSQ( nmpcWorkspace.objValueIn, nmpcWorkspace.objValueOut );
nmpcWorkspace.Dy[runObj * 7] = nmpcWorkspace.objValueOut[0];
nmpcWorkspace.Dy[runObj * 7 + 1] = nmpcWorkspace.objValueOut[1];
nmpcWorkspace.Dy[runObj * 7 + 2] = nmpcWorkspace.objValueOut[2];
nmpcWorkspace.Dy[runObj * 7 + 3] = nmpcWorkspace.objValueOut[3];
nmpcWorkspace.Dy[runObj * 7 + 4] = nmpcWorkspace.objValueOut[4];
nmpcWorkspace.Dy[runObj * 7 + 5] = nmpcWorkspace.objValueOut[5];
nmpcWorkspace.Dy[runObj * 7 + 6] = nmpcWorkspace.objValueOut[6];

nmpc_setObjQ1Q2( nmpcVariables.W, &(nmpcWorkspace.Q1[ runObj * 25 ]), &(nmpcWorkspace.Q2[ runObj * 35 ]) );

nmpc_setObjR1R2( nmpcVariables.W, &(nmpcWorkspace.R1[ runObj * 4 ]), &(nmpcWorkspace.R2[ runObj * 14 ]) );

}
nmpcWorkspace.objValueIn[0] = nmpcVariables.x[150];
nmpcWorkspace.objValueIn[1] = nmpcVariables.x[151];
nmpcWorkspace.objValueIn[2] = nmpcVariables.x[152];
nmpcWorkspace.objValueIn[3] = nmpcVariables.x[153];
nmpcWorkspace.objValueIn[4] = nmpcVariables.x[154];
nmpcWorkspace.objValueIn[5] = nmpcVariables.od[60];
nmpcWorkspace.objValueIn[6] = nmpcVariables.od[61];
nmpc_evaluateLSQEndTerm( nmpcWorkspace.objValueIn, nmpcWorkspace.objValueOut );

nmpcWorkspace.DyN[0] = nmpcWorkspace.objValueOut[0];
nmpcWorkspace.DyN[1] = nmpcWorkspace.objValueOut[1];
nmpcWorkspace.DyN[2] = nmpcWorkspace.objValueOut[2];
nmpcWorkspace.DyN[3] = nmpcWorkspace.objValueOut[3];
nmpcWorkspace.DyN[4] = nmpcWorkspace.objValueOut[4];

nmpc_setObjQN1QN2( nmpcVariables.WN, nmpcWorkspace.QN1, nmpcWorkspace.QN2 );

}

void nmpc_multGxd( real_t* const dOld, real_t* const Gx1, real_t* const dNew )
{
dNew[0] += + Gx1[0]*dOld[0] + Gx1[1]*dOld[1] + Gx1[2]*dOld[2] + Gx1[3]*dOld[3] + Gx1[4]*dOld[4];
dNew[1] += + Gx1[5]*dOld[0] + Gx1[6]*dOld[1] + Gx1[7]*dOld[2] + Gx1[8]*dOld[3] + Gx1[9]*dOld[4];
dNew[2] += + Gx1[10]*dOld[0] + Gx1[11]*dOld[1] + Gx1[12]*dOld[2] + Gx1[13]*dOld[3] + Gx1[14]*dOld[4];
dNew[3] += + Gx1[15]*dOld[0] + Gx1[16]*dOld[1] + Gx1[17]*dOld[2] + Gx1[18]*dOld[3] + Gx1[19]*dOld[4];
dNew[4] += + Gx1[20]*dOld[0] + Gx1[21]*dOld[1] + Gx1[22]*dOld[2] + Gx1[23]*dOld[3] + Gx1[24]*dOld[4];
}

void nmpc_moveGxT( real_t* const Gx1, real_t* const Gx2 )
{
Gx2[0] = Gx1[0];
Gx2[1] = Gx1[1];
Gx2[2] = Gx1[2];
Gx2[3] = Gx1[3];
Gx2[4] = Gx1[4];
Gx2[5] = Gx1[5];
Gx2[6] = Gx1[6];
Gx2[7] = Gx1[7];
Gx2[8] = Gx1[8];
Gx2[9] = Gx1[9];
Gx2[10] = Gx1[10];
Gx2[11] = Gx1[11];
Gx2[12] = Gx1[12];
Gx2[13] = Gx1[13];
Gx2[14] = Gx1[14];
Gx2[15] = Gx1[15];
Gx2[16] = Gx1[16];
Gx2[17] = Gx1[17];
Gx2[18] = Gx1[18];
Gx2[19] = Gx1[19];
Gx2[20] = Gx1[20];
Gx2[21] = Gx1[21];
Gx2[22] = Gx1[22];
Gx2[23] = Gx1[23];
Gx2[24] = Gx1[24];
}

void nmpc_multGxGx( real_t* const Gx1, real_t* const Gx2, real_t* const Gx3 )
{
Gx3[0] = + Gx1[0]*Gx2[0] + Gx1[1]*Gx2[5] + Gx1[2]*Gx2[10] + Gx1[3]*Gx2[15] + Gx1[4]*Gx2[20];
Gx3[1] = + Gx1[0]*Gx2[1] + Gx1[1]*Gx2[6] + Gx1[2]*Gx2[11] + Gx1[3]*Gx2[16] + Gx1[4]*Gx2[21];
Gx3[2] = + Gx1[0]*Gx2[2] + Gx1[1]*Gx2[7] + Gx1[2]*Gx2[12] + Gx1[3]*Gx2[17] + Gx1[4]*Gx2[22];
Gx3[3] = + Gx1[0]*Gx2[3] + Gx1[1]*Gx2[8] + Gx1[2]*Gx2[13] + Gx1[3]*Gx2[18] + Gx1[4]*Gx2[23];
Gx3[4] = + Gx1[0]*Gx2[4] + Gx1[1]*Gx2[9] + Gx1[2]*Gx2[14] + Gx1[3]*Gx2[19] + Gx1[4]*Gx2[24];
Gx3[5] = + Gx1[5]*Gx2[0] + Gx1[6]*Gx2[5] + Gx1[7]*Gx2[10] + Gx1[8]*Gx2[15] + Gx1[9]*Gx2[20];
Gx3[6] = + Gx1[5]*Gx2[1] + Gx1[6]*Gx2[6] + Gx1[7]*Gx2[11] + Gx1[8]*Gx2[16] + Gx1[9]*Gx2[21];
Gx3[7] = + Gx1[5]*Gx2[2] + Gx1[6]*Gx2[7] + Gx1[7]*Gx2[12] + Gx1[8]*Gx2[17] + Gx1[9]*Gx2[22];
Gx3[8] = + Gx1[5]*Gx2[3] + Gx1[6]*Gx2[8] + Gx1[7]*Gx2[13] + Gx1[8]*Gx2[18] + Gx1[9]*Gx2[23];
Gx3[9] = + Gx1[5]*Gx2[4] + Gx1[6]*Gx2[9] + Gx1[7]*Gx2[14] + Gx1[8]*Gx2[19] + Gx1[9]*Gx2[24];
Gx3[10] = + Gx1[10]*Gx2[0] + Gx1[11]*Gx2[5] + Gx1[12]*Gx2[10] + Gx1[13]*Gx2[15] + Gx1[14]*Gx2[20];
Gx3[11] = + Gx1[10]*Gx2[1] + Gx1[11]*Gx2[6] + Gx1[12]*Gx2[11] + Gx1[13]*Gx2[16] + Gx1[14]*Gx2[21];
Gx3[12] = + Gx1[10]*Gx2[2] + Gx1[11]*Gx2[7] + Gx1[12]*Gx2[12] + Gx1[13]*Gx2[17] + Gx1[14]*Gx2[22];
Gx3[13] = + Gx1[10]*Gx2[3] + Gx1[11]*Gx2[8] + Gx1[12]*Gx2[13] + Gx1[13]*Gx2[18] + Gx1[14]*Gx2[23];
Gx3[14] = + Gx1[10]*Gx2[4] + Gx1[11]*Gx2[9] + Gx1[12]*Gx2[14] + Gx1[13]*Gx2[19] + Gx1[14]*Gx2[24];
Gx3[15] = + Gx1[15]*Gx2[0] + Gx1[16]*Gx2[5] + Gx1[17]*Gx2[10] + Gx1[18]*Gx2[15] + Gx1[19]*Gx2[20];
Gx3[16] = + Gx1[15]*Gx2[1] + Gx1[16]*Gx2[6] + Gx1[17]*Gx2[11] + Gx1[18]*Gx2[16] + Gx1[19]*Gx2[21];
Gx3[17] = + Gx1[15]*Gx2[2] + Gx1[16]*Gx2[7] + Gx1[17]*Gx2[12] + Gx1[18]*Gx2[17] + Gx1[19]*Gx2[22];
Gx3[18] = + Gx1[15]*Gx2[3] + Gx1[16]*Gx2[8] + Gx1[17]*Gx2[13] + Gx1[18]*Gx2[18] + Gx1[19]*Gx2[23];
Gx3[19] = + Gx1[15]*Gx2[4] + Gx1[16]*Gx2[9] + Gx1[17]*Gx2[14] + Gx1[18]*Gx2[19] + Gx1[19]*Gx2[24];
Gx3[20] = + Gx1[20]*Gx2[0] + Gx1[21]*Gx2[5] + Gx1[22]*Gx2[10] + Gx1[23]*Gx2[15] + Gx1[24]*Gx2[20];
Gx3[21] = + Gx1[20]*Gx2[1] + Gx1[21]*Gx2[6] + Gx1[22]*Gx2[11] + Gx1[23]*Gx2[16] + Gx1[24]*Gx2[21];
Gx3[22] = + Gx1[20]*Gx2[2] + Gx1[21]*Gx2[7] + Gx1[22]*Gx2[12] + Gx1[23]*Gx2[17] + Gx1[24]*Gx2[22];
Gx3[23] = + Gx1[20]*Gx2[3] + Gx1[21]*Gx2[8] + Gx1[22]*Gx2[13] + Gx1[23]*Gx2[18] + Gx1[24]*Gx2[23];
Gx3[24] = + Gx1[20]*Gx2[4] + Gx1[21]*Gx2[9] + Gx1[22]*Gx2[14] + Gx1[23]*Gx2[19] + Gx1[24]*Gx2[24];
}

void nmpc_multGxGu( real_t* const Gx1, real_t* const Gu1, real_t* const Gu2 )
{
Gu2[0] = + Gx1[0]*Gu1[0] + Gx1[1]*Gu1[2] + Gx1[2]*Gu1[4] + Gx1[3]*Gu1[6] + Gx1[4]*Gu1[8];
Gu2[1] = + Gx1[0]*Gu1[1] + Gx1[1]*Gu1[3] + Gx1[2]*Gu1[5] + Gx1[3]*Gu1[7] + Gx1[4]*Gu1[9];
Gu2[2] = + Gx1[5]*Gu1[0] + Gx1[6]*Gu1[2] + Gx1[7]*Gu1[4] + Gx1[8]*Gu1[6] + Gx1[9]*Gu1[8];
Gu2[3] = + Gx1[5]*Gu1[1] + Gx1[6]*Gu1[3] + Gx1[7]*Gu1[5] + Gx1[8]*Gu1[7] + Gx1[9]*Gu1[9];
Gu2[4] = + Gx1[10]*Gu1[0] + Gx1[11]*Gu1[2] + Gx1[12]*Gu1[4] + Gx1[13]*Gu1[6] + Gx1[14]*Gu1[8];
Gu2[5] = + Gx1[10]*Gu1[1] + Gx1[11]*Gu1[3] + Gx1[12]*Gu1[5] + Gx1[13]*Gu1[7] + Gx1[14]*Gu1[9];
Gu2[6] = + Gx1[15]*Gu1[0] + Gx1[16]*Gu1[2] + Gx1[17]*Gu1[4] + Gx1[18]*Gu1[6] + Gx1[19]*Gu1[8];
Gu2[7] = + Gx1[15]*Gu1[1] + Gx1[16]*Gu1[3] + Gx1[17]*Gu1[5] + Gx1[18]*Gu1[7] + Gx1[19]*Gu1[9];
Gu2[8] = + Gx1[20]*Gu1[0] + Gx1[21]*Gu1[2] + Gx1[22]*Gu1[4] + Gx1[23]*Gu1[6] + Gx1[24]*Gu1[8];
Gu2[9] = + Gx1[20]*Gu1[1] + Gx1[21]*Gu1[3] + Gx1[22]*Gu1[5] + Gx1[23]*Gu1[7] + Gx1[24]*Gu1[9];
}

void nmpc_moveGuE( real_t* const Gu1, real_t* const Gu2 )
{
Gu2[0] = Gu1[0];
Gu2[1] = Gu1[1];
Gu2[2] = Gu1[2];
Gu2[3] = Gu1[3];
Gu2[4] = Gu1[4];
Gu2[5] = Gu1[5];
Gu2[6] = Gu1[6];
Gu2[7] = Gu1[7];
Gu2[8] = Gu1[8];
Gu2[9] = Gu1[9];
}

void nmpc_setBlockH11( int iRow, int iCol, real_t* const Gu1, real_t* const Gu2 )
{
nmpcWorkspace.H[(iRow * 130 + 325) + (iCol * 2 + 5)] += + Gu1[0]*Gu2[0] + Gu1[2]*Gu2[2] + Gu1[4]*Gu2[4] + Gu1[6]*Gu2[6] + Gu1[8]*Gu2[8];
nmpcWorkspace.H[(iRow * 130 + 325) + (iCol * 2 + 6)] += + Gu1[0]*Gu2[1] + Gu1[2]*Gu2[3] + Gu1[4]*Gu2[5] + Gu1[6]*Gu2[7] + Gu1[8]*Gu2[9];
nmpcWorkspace.H[(iRow * 130 + 390) + (iCol * 2 + 5)] += + Gu1[1]*Gu2[0] + Gu1[3]*Gu2[2] + Gu1[5]*Gu2[4] + Gu1[7]*Gu2[6] + Gu1[9]*Gu2[8];
nmpcWorkspace.H[(iRow * 130 + 390) + (iCol * 2 + 6)] += + Gu1[1]*Gu2[1] + Gu1[3]*Gu2[3] + Gu1[5]*Gu2[5] + Gu1[7]*Gu2[7] + Gu1[9]*Gu2[9];
}

void nmpc_setBlockH11_R1( int iRow, int iCol, real_t* const R11 )
{
nmpcWorkspace.H[(iRow * 130 + 325) + (iCol * 2 + 5)] = R11[0];
nmpcWorkspace.H[(iRow * 130 + 325) + (iCol * 2 + 6)] = R11[1];
nmpcWorkspace.H[(iRow * 130 + 390) + (iCol * 2 + 5)] = R11[2];
nmpcWorkspace.H[(iRow * 130 + 390) + (iCol * 2 + 6)] = R11[3];
}

void nmpc_zeroBlockH11( int iRow, int iCol )
{
nmpcWorkspace.H[(iRow * 130 + 325) + (iCol * 2 + 5)] = 0.0000000000000000e+00;
nmpcWorkspace.H[(iRow * 130 + 325) + (iCol * 2 + 6)] = 0.0000000000000000e+00;
nmpcWorkspace.H[(iRow * 130 + 390) + (iCol * 2 + 5)] = 0.0000000000000000e+00;
nmpcWorkspace.H[(iRow * 130 + 390) + (iCol * 2 + 6)] = 0.0000000000000000e+00;
}

void nmpc_copyHTH( int iRow, int iCol )
{
nmpcWorkspace.H[(iRow * 130 + 325) + (iCol * 2 + 5)] = nmpcWorkspace.H[(iCol * 130 + 325) + (iRow * 2 + 5)];
nmpcWorkspace.H[(iRow * 130 + 325) + (iCol * 2 + 6)] = nmpcWorkspace.H[(iCol * 130 + 390) + (iRow * 2 + 5)];
nmpcWorkspace.H[(iRow * 130 + 390) + (iCol * 2 + 5)] = nmpcWorkspace.H[(iCol * 130 + 325) + (iRow * 2 + 6)];
nmpcWorkspace.H[(iRow * 130 + 390) + (iCol * 2 + 6)] = nmpcWorkspace.H[(iCol * 130 + 390) + (iRow * 2 + 6)];
}

void nmpc_multQ1d( real_t* const Gx1, real_t* const dOld, real_t* const dNew )
{
dNew[0] = + Gx1[0]*dOld[0] + Gx1[1]*dOld[1] + Gx1[2]*dOld[2] + Gx1[3]*dOld[3] + Gx1[4]*dOld[4];
dNew[1] = + Gx1[5]*dOld[0] + Gx1[6]*dOld[1] + Gx1[7]*dOld[2] + Gx1[8]*dOld[3] + Gx1[9]*dOld[4];
dNew[2] = + Gx1[10]*dOld[0] + Gx1[11]*dOld[1] + Gx1[12]*dOld[2] + Gx1[13]*dOld[3] + Gx1[14]*dOld[4];
dNew[3] = + Gx1[15]*dOld[0] + Gx1[16]*dOld[1] + Gx1[17]*dOld[2] + Gx1[18]*dOld[3] + Gx1[19]*dOld[4];
dNew[4] = + Gx1[20]*dOld[0] + Gx1[21]*dOld[1] + Gx1[22]*dOld[2] + Gx1[23]*dOld[3] + Gx1[24]*dOld[4];
}

void nmpc_multQN1d( real_t* const QN1, real_t* const dOld, real_t* const dNew )
{
dNew[0] = + nmpcWorkspace.QN1[0]*dOld[0] + nmpcWorkspace.QN1[1]*dOld[1] + nmpcWorkspace.QN1[2]*dOld[2] + nmpcWorkspace.QN1[3]*dOld[3] + nmpcWorkspace.QN1[4]*dOld[4];
dNew[1] = + nmpcWorkspace.QN1[5]*dOld[0] + nmpcWorkspace.QN1[6]*dOld[1] + nmpcWorkspace.QN1[7]*dOld[2] + nmpcWorkspace.QN1[8]*dOld[3] + nmpcWorkspace.QN1[9]*dOld[4];
dNew[2] = + nmpcWorkspace.QN1[10]*dOld[0] + nmpcWorkspace.QN1[11]*dOld[1] + nmpcWorkspace.QN1[12]*dOld[2] + nmpcWorkspace.QN1[13]*dOld[3] + nmpcWorkspace.QN1[14]*dOld[4];
dNew[3] = + nmpcWorkspace.QN1[15]*dOld[0] + nmpcWorkspace.QN1[16]*dOld[1] + nmpcWorkspace.QN1[17]*dOld[2] + nmpcWorkspace.QN1[18]*dOld[3] + nmpcWorkspace.QN1[19]*dOld[4];
dNew[4] = + nmpcWorkspace.QN1[20]*dOld[0] + nmpcWorkspace.QN1[21]*dOld[1] + nmpcWorkspace.QN1[22]*dOld[2] + nmpcWorkspace.QN1[23]*dOld[3] + nmpcWorkspace.QN1[24]*dOld[4];
}

void nmpc_multRDy( real_t* const R2, real_t* const Dy1, real_t* const RDy1 )
{
RDy1[0] = + R2[0]*Dy1[0] + R2[1]*Dy1[1] + R2[2]*Dy1[2] + R2[3]*Dy1[3] + R2[4]*Dy1[4] + R2[5]*Dy1[5] + R2[6]*Dy1[6];
RDy1[1] = + R2[7]*Dy1[0] + R2[8]*Dy1[1] + R2[9]*Dy1[2] + R2[10]*Dy1[3] + R2[11]*Dy1[4] + R2[12]*Dy1[5] + R2[13]*Dy1[6];
}

void nmpc_multQDy( real_t* const Q2, real_t* const Dy1, real_t* const QDy1 )
{
QDy1[0] = + Q2[0]*Dy1[0] + Q2[1]*Dy1[1] + Q2[2]*Dy1[2] + Q2[3]*Dy1[3] + Q2[4]*Dy1[4] + Q2[5]*Dy1[5] + Q2[6]*Dy1[6];
QDy1[1] = + Q2[7]*Dy1[0] + Q2[8]*Dy1[1] + Q2[9]*Dy1[2] + Q2[10]*Dy1[3] + Q2[11]*Dy1[4] + Q2[12]*Dy1[5] + Q2[13]*Dy1[6];
QDy1[2] = + Q2[14]*Dy1[0] + Q2[15]*Dy1[1] + Q2[16]*Dy1[2] + Q2[17]*Dy1[3] + Q2[18]*Dy1[4] + Q2[19]*Dy1[5] + Q2[20]*Dy1[6];
QDy1[3] = + Q2[21]*Dy1[0] + Q2[22]*Dy1[1] + Q2[23]*Dy1[2] + Q2[24]*Dy1[3] + Q2[25]*Dy1[4] + Q2[26]*Dy1[5] + Q2[27]*Dy1[6];
QDy1[4] = + Q2[28]*Dy1[0] + Q2[29]*Dy1[1] + Q2[30]*Dy1[2] + Q2[31]*Dy1[3] + Q2[32]*Dy1[4] + Q2[33]*Dy1[5] + Q2[34]*Dy1[6];
}

void nmpc_multEQDy( real_t* const E1, real_t* const QDy1, real_t* const U1 )
{
U1[0] += + E1[0]*QDy1[0] + E1[2]*QDy1[1] + E1[4]*QDy1[2] + E1[6]*QDy1[3] + E1[8]*QDy1[4];
U1[1] += + E1[1]*QDy1[0] + E1[3]*QDy1[1] + E1[5]*QDy1[2] + E1[7]*QDy1[3] + E1[9]*QDy1[4];
}

void nmpc_multQETGx( real_t* const E1, real_t* const Gx1, real_t* const H101 )
{
H101[0] += + E1[0]*Gx1[0] + E1[2]*Gx1[5] + E1[4]*Gx1[10] + E1[6]*Gx1[15] + E1[8]*Gx1[20];
H101[1] += + E1[0]*Gx1[1] + E1[2]*Gx1[6] + E1[4]*Gx1[11] + E1[6]*Gx1[16] + E1[8]*Gx1[21];
H101[2] += + E1[0]*Gx1[2] + E1[2]*Gx1[7] + E1[4]*Gx1[12] + E1[6]*Gx1[17] + E1[8]*Gx1[22];
H101[3] += + E1[0]*Gx1[3] + E1[2]*Gx1[8] + E1[4]*Gx1[13] + E1[6]*Gx1[18] + E1[8]*Gx1[23];
H101[4] += + E1[0]*Gx1[4] + E1[2]*Gx1[9] + E1[4]*Gx1[14] + E1[6]*Gx1[19] + E1[8]*Gx1[24];
H101[5] += + E1[1]*Gx1[0] + E1[3]*Gx1[5] + E1[5]*Gx1[10] + E1[7]*Gx1[15] + E1[9]*Gx1[20];
H101[6] += + E1[1]*Gx1[1] + E1[3]*Gx1[6] + E1[5]*Gx1[11] + E1[7]*Gx1[16] + E1[9]*Gx1[21];
H101[7] += + E1[1]*Gx1[2] + E1[3]*Gx1[7] + E1[5]*Gx1[12] + E1[7]*Gx1[17] + E1[9]*Gx1[22];
H101[8] += + E1[1]*Gx1[3] + E1[3]*Gx1[8] + E1[5]*Gx1[13] + E1[7]*Gx1[18] + E1[9]*Gx1[23];
H101[9] += + E1[1]*Gx1[4] + E1[3]*Gx1[9] + E1[5]*Gx1[14] + E1[7]*Gx1[19] + E1[9]*Gx1[24];
}

void nmpc_zeroBlockH10( real_t* const H101 )
{
{ int lCopy; for (lCopy = 0; lCopy < 10; lCopy++) H101[ lCopy ] = 0; }
}

void nmpc_multEDu( real_t* const E1, real_t* const U1, real_t* const dNew )
{
dNew[0] += + E1[0]*U1[0] + E1[1]*U1[1];
dNew[1] += + E1[2]*U1[0] + E1[3]*U1[1];
dNew[2] += + E1[4]*U1[0] + E1[5]*U1[1];
dNew[3] += + E1[6]*U1[0] + E1[7]*U1[1];
dNew[4] += + E1[8]*U1[0] + E1[9]*U1[1];
}

void nmpc_zeroBlockH00(  )
{
nmpcWorkspace.H[0] = 0.0000000000000000e+00;
nmpcWorkspace.H[1] = 0.0000000000000000e+00;
nmpcWorkspace.H[2] = 0.0000000000000000e+00;
nmpcWorkspace.H[3] = 0.0000000000000000e+00;
nmpcWorkspace.H[4] = 0.0000000000000000e+00;
nmpcWorkspace.H[65] = 0.0000000000000000e+00;
nmpcWorkspace.H[66] = 0.0000000000000000e+00;
nmpcWorkspace.H[67] = 0.0000000000000000e+00;
nmpcWorkspace.H[68] = 0.0000000000000000e+00;
nmpcWorkspace.H[69] = 0.0000000000000000e+00;
nmpcWorkspace.H[130] = 0.0000000000000000e+00;
nmpcWorkspace.H[131] = 0.0000000000000000e+00;
nmpcWorkspace.H[132] = 0.0000000000000000e+00;
nmpcWorkspace.H[133] = 0.0000000000000000e+00;
nmpcWorkspace.H[134] = 0.0000000000000000e+00;
nmpcWorkspace.H[195] = 0.0000000000000000e+00;
nmpcWorkspace.H[196] = 0.0000000000000000e+00;
nmpcWorkspace.H[197] = 0.0000000000000000e+00;
nmpcWorkspace.H[198] = 0.0000000000000000e+00;
nmpcWorkspace.H[199] = 0.0000000000000000e+00;
nmpcWorkspace.H[260] = 0.0000000000000000e+00;
nmpcWorkspace.H[261] = 0.0000000000000000e+00;
nmpcWorkspace.H[262] = 0.0000000000000000e+00;
nmpcWorkspace.H[263] = 0.0000000000000000e+00;
nmpcWorkspace.H[264] = 0.0000000000000000e+00;
}

void nmpc_multCTQC( real_t* const Gx1, real_t* const Gx2 )
{
nmpcWorkspace.H[0] += + Gx1[0]*Gx2[0] + Gx1[5]*Gx2[5] + Gx1[10]*Gx2[10] + Gx1[15]*Gx2[15] + Gx1[20]*Gx2[20];
nmpcWorkspace.H[1] += + Gx1[0]*Gx2[1] + Gx1[5]*Gx2[6] + Gx1[10]*Gx2[11] + Gx1[15]*Gx2[16] + Gx1[20]*Gx2[21];
nmpcWorkspace.H[2] += + Gx1[0]*Gx2[2] + Gx1[5]*Gx2[7] + Gx1[10]*Gx2[12] + Gx1[15]*Gx2[17] + Gx1[20]*Gx2[22];
nmpcWorkspace.H[3] += + Gx1[0]*Gx2[3] + Gx1[5]*Gx2[8] + Gx1[10]*Gx2[13] + Gx1[15]*Gx2[18] + Gx1[20]*Gx2[23];
nmpcWorkspace.H[4] += + Gx1[0]*Gx2[4] + Gx1[5]*Gx2[9] + Gx1[10]*Gx2[14] + Gx1[15]*Gx2[19] + Gx1[20]*Gx2[24];
nmpcWorkspace.H[65] += + Gx1[1]*Gx2[0] + Gx1[6]*Gx2[5] + Gx1[11]*Gx2[10] + Gx1[16]*Gx2[15] + Gx1[21]*Gx2[20];
nmpcWorkspace.H[66] += + Gx1[1]*Gx2[1] + Gx1[6]*Gx2[6] + Gx1[11]*Gx2[11] + Gx1[16]*Gx2[16] + Gx1[21]*Gx2[21];
nmpcWorkspace.H[67] += + Gx1[1]*Gx2[2] + Gx1[6]*Gx2[7] + Gx1[11]*Gx2[12] + Gx1[16]*Gx2[17] + Gx1[21]*Gx2[22];
nmpcWorkspace.H[68] += + Gx1[1]*Gx2[3] + Gx1[6]*Gx2[8] + Gx1[11]*Gx2[13] + Gx1[16]*Gx2[18] + Gx1[21]*Gx2[23];
nmpcWorkspace.H[69] += + Gx1[1]*Gx2[4] + Gx1[6]*Gx2[9] + Gx1[11]*Gx2[14] + Gx1[16]*Gx2[19] + Gx1[21]*Gx2[24];
nmpcWorkspace.H[130] += + Gx1[2]*Gx2[0] + Gx1[7]*Gx2[5] + Gx1[12]*Gx2[10] + Gx1[17]*Gx2[15] + Gx1[22]*Gx2[20];
nmpcWorkspace.H[131] += + Gx1[2]*Gx2[1] + Gx1[7]*Gx2[6] + Gx1[12]*Gx2[11] + Gx1[17]*Gx2[16] + Gx1[22]*Gx2[21];
nmpcWorkspace.H[132] += + Gx1[2]*Gx2[2] + Gx1[7]*Gx2[7] + Gx1[12]*Gx2[12] + Gx1[17]*Gx2[17] + Gx1[22]*Gx2[22];
nmpcWorkspace.H[133] += + Gx1[2]*Gx2[3] + Gx1[7]*Gx2[8] + Gx1[12]*Gx2[13] + Gx1[17]*Gx2[18] + Gx1[22]*Gx2[23];
nmpcWorkspace.H[134] += + Gx1[2]*Gx2[4] + Gx1[7]*Gx2[9] + Gx1[12]*Gx2[14] + Gx1[17]*Gx2[19] + Gx1[22]*Gx2[24];
nmpcWorkspace.H[195] += + Gx1[3]*Gx2[0] + Gx1[8]*Gx2[5] + Gx1[13]*Gx2[10] + Gx1[18]*Gx2[15] + Gx1[23]*Gx2[20];
nmpcWorkspace.H[196] += + Gx1[3]*Gx2[1] + Gx1[8]*Gx2[6] + Gx1[13]*Gx2[11] + Gx1[18]*Gx2[16] + Gx1[23]*Gx2[21];
nmpcWorkspace.H[197] += + Gx1[3]*Gx2[2] + Gx1[8]*Gx2[7] + Gx1[13]*Gx2[12] + Gx1[18]*Gx2[17] + Gx1[23]*Gx2[22];
nmpcWorkspace.H[198] += + Gx1[3]*Gx2[3] + Gx1[8]*Gx2[8] + Gx1[13]*Gx2[13] + Gx1[18]*Gx2[18] + Gx1[23]*Gx2[23];
nmpcWorkspace.H[199] += + Gx1[3]*Gx2[4] + Gx1[8]*Gx2[9] + Gx1[13]*Gx2[14] + Gx1[18]*Gx2[19] + Gx1[23]*Gx2[24];
nmpcWorkspace.H[260] += + Gx1[4]*Gx2[0] + Gx1[9]*Gx2[5] + Gx1[14]*Gx2[10] + Gx1[19]*Gx2[15] + Gx1[24]*Gx2[20];
nmpcWorkspace.H[261] += + Gx1[4]*Gx2[1] + Gx1[9]*Gx2[6] + Gx1[14]*Gx2[11] + Gx1[19]*Gx2[16] + Gx1[24]*Gx2[21];
nmpcWorkspace.H[262] += + Gx1[4]*Gx2[2] + Gx1[9]*Gx2[7] + Gx1[14]*Gx2[12] + Gx1[19]*Gx2[17] + Gx1[24]*Gx2[22];
nmpcWorkspace.H[263] += + Gx1[4]*Gx2[3] + Gx1[9]*Gx2[8] + Gx1[14]*Gx2[13] + Gx1[19]*Gx2[18] + Gx1[24]*Gx2[23];
nmpcWorkspace.H[264] += + Gx1[4]*Gx2[4] + Gx1[9]*Gx2[9] + Gx1[14]*Gx2[14] + Gx1[19]*Gx2[19] + Gx1[24]*Gx2[24];
}

void nmpc_macCTSlx( real_t* const C0, real_t* const g0 )
{
g0[0] += 0.0;
;
g0[1] += 0.0;
;
g0[2] += 0.0;
;
g0[3] += 0.0;
;
g0[4] += 0.0;
;
}

void nmpc_macETSlu( real_t* const E0, real_t* const g1 )
{
g1[0] += 0.0;
;
g1[1] += 0.0;
;
}

void nmpc_condensePrep(  )
{
int lRun1;
int lRun2;
int lRun3;
int lRun4;
int lRun5;
/** Row vector of size: 60 */
static const int xBoundIndices[ 60 ] = 
{ 8, 9, 13, 14, 18, 19, 23, 24, 28, 29, 33, 34, 38, 39, 43, 44, 48, 49, 53, 54, 58, 59, 63, 64, 68, 69, 73, 74, 78, 79, 83, 84, 88, 89, 93, 94, 98, 99, 103, 104, 108, 109, 113, 114, 118, 119, 123, 124, 128, 129, 133, 134, 138, 139, 143, 144, 148, 149, 153, 154 };
nmpc_moveGuE( nmpcWorkspace.evGu, nmpcWorkspace.E );
for (lRun1 = 1; lRun1 < 30; ++lRun1)
{
nmpc_moveGxT( &(nmpcWorkspace.evGx[ lRun1 * 25 ]), nmpcWorkspace.T );
nmpc_multGxd( &(nmpcWorkspace.d[ lRun1 * 5-5 ]), &(nmpcWorkspace.evGx[ lRun1 * 25 ]), &(nmpcWorkspace.d[ lRun1 * 5 ]) );
nmpc_multGxGx( nmpcWorkspace.T, &(nmpcWorkspace.evGx[ lRun1 * 25-25 ]), &(nmpcWorkspace.evGx[ lRun1 * 25 ]) );
for (lRun2 = 0; lRun2 < lRun1; ++lRun2)
{
lRun4 = (((lRun1) * (lRun1-1)) / (2)) + (lRun2);
lRun3 = (((lRun1 + 1) * (lRun1)) / (2)) + (lRun2);
nmpc_multGxGu( nmpcWorkspace.T, &(nmpcWorkspace.E[ lRun4 * 10 ]), &(nmpcWorkspace.E[ lRun3 * 10 ]) );
}
lRun3 = (((lRun1 + 1) * (lRun1)) / (2)) + (lRun2);
nmpc_moveGuE( &(nmpcWorkspace.evGu[ lRun1 * 10 ]), &(nmpcWorkspace.E[ lRun3 * 10 ]) );
}

nmpc_multGxGx( &(nmpcWorkspace.Q1[ 25 ]), nmpcWorkspace.evGx, nmpcWorkspace.QGx );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 50 ]), &(nmpcWorkspace.evGx[ 25 ]), &(nmpcWorkspace.QGx[ 25 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 75 ]), &(nmpcWorkspace.evGx[ 50 ]), &(nmpcWorkspace.QGx[ 50 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 100 ]), &(nmpcWorkspace.evGx[ 75 ]), &(nmpcWorkspace.QGx[ 75 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 125 ]), &(nmpcWorkspace.evGx[ 100 ]), &(nmpcWorkspace.QGx[ 100 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 150 ]), &(nmpcWorkspace.evGx[ 125 ]), &(nmpcWorkspace.QGx[ 125 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 175 ]), &(nmpcWorkspace.evGx[ 150 ]), &(nmpcWorkspace.QGx[ 150 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 200 ]), &(nmpcWorkspace.evGx[ 175 ]), &(nmpcWorkspace.QGx[ 175 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 225 ]), &(nmpcWorkspace.evGx[ 200 ]), &(nmpcWorkspace.QGx[ 200 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 250 ]), &(nmpcWorkspace.evGx[ 225 ]), &(nmpcWorkspace.QGx[ 225 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 275 ]), &(nmpcWorkspace.evGx[ 250 ]), &(nmpcWorkspace.QGx[ 250 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 300 ]), &(nmpcWorkspace.evGx[ 275 ]), &(nmpcWorkspace.QGx[ 275 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 325 ]), &(nmpcWorkspace.evGx[ 300 ]), &(nmpcWorkspace.QGx[ 300 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 350 ]), &(nmpcWorkspace.evGx[ 325 ]), &(nmpcWorkspace.QGx[ 325 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 375 ]), &(nmpcWorkspace.evGx[ 350 ]), &(nmpcWorkspace.QGx[ 350 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 400 ]), &(nmpcWorkspace.evGx[ 375 ]), &(nmpcWorkspace.QGx[ 375 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 425 ]), &(nmpcWorkspace.evGx[ 400 ]), &(nmpcWorkspace.QGx[ 400 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 450 ]), &(nmpcWorkspace.evGx[ 425 ]), &(nmpcWorkspace.QGx[ 425 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 475 ]), &(nmpcWorkspace.evGx[ 450 ]), &(nmpcWorkspace.QGx[ 450 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 500 ]), &(nmpcWorkspace.evGx[ 475 ]), &(nmpcWorkspace.QGx[ 475 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 525 ]), &(nmpcWorkspace.evGx[ 500 ]), &(nmpcWorkspace.QGx[ 500 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 550 ]), &(nmpcWorkspace.evGx[ 525 ]), &(nmpcWorkspace.QGx[ 525 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 575 ]), &(nmpcWorkspace.evGx[ 550 ]), &(nmpcWorkspace.QGx[ 550 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 600 ]), &(nmpcWorkspace.evGx[ 575 ]), &(nmpcWorkspace.QGx[ 575 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 625 ]), &(nmpcWorkspace.evGx[ 600 ]), &(nmpcWorkspace.QGx[ 600 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 650 ]), &(nmpcWorkspace.evGx[ 625 ]), &(nmpcWorkspace.QGx[ 625 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 675 ]), &(nmpcWorkspace.evGx[ 650 ]), &(nmpcWorkspace.QGx[ 650 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 700 ]), &(nmpcWorkspace.evGx[ 675 ]), &(nmpcWorkspace.QGx[ 675 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 725 ]), &(nmpcWorkspace.evGx[ 700 ]), &(nmpcWorkspace.QGx[ 700 ]) );
nmpc_multGxGx( nmpcWorkspace.QN1, &(nmpcWorkspace.evGx[ 725 ]), &(nmpcWorkspace.QGx[ 725 ]) );

for (lRun1 = 0; lRun1 < 29; ++lRun1)
{
for (lRun2 = 0; lRun2 < lRun1 + 1; ++lRun2)
{
lRun3 = (((lRun1 + 1) * (lRun1)) / (2)) + (lRun2);
nmpc_multGxGu( &(nmpcWorkspace.Q1[ lRun1 * 25 + 25 ]), &(nmpcWorkspace.E[ lRun3 * 10 ]), &(nmpcWorkspace.QE[ lRun3 * 10 ]) );
}
}

for (lRun2 = 0; lRun2 < lRun1 + 1; ++lRun2)
{
lRun3 = (((lRun1 + 1) * (lRun1)) / (2)) + (lRun2);
nmpc_multGxGu( nmpcWorkspace.QN1, &(nmpcWorkspace.E[ lRun3 * 10 ]), &(nmpcWorkspace.QE[ lRun3 * 10 ]) );
}

nmpc_zeroBlockH00(  );
nmpc_multCTQC( nmpcWorkspace.evGx, nmpcWorkspace.QGx );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 25 ]), &(nmpcWorkspace.QGx[ 25 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 50 ]), &(nmpcWorkspace.QGx[ 50 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 75 ]), &(nmpcWorkspace.QGx[ 75 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 100 ]), &(nmpcWorkspace.QGx[ 100 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 125 ]), &(nmpcWorkspace.QGx[ 125 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 150 ]), &(nmpcWorkspace.QGx[ 150 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 175 ]), &(nmpcWorkspace.QGx[ 175 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 200 ]), &(nmpcWorkspace.QGx[ 200 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 225 ]), &(nmpcWorkspace.QGx[ 225 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 250 ]), &(nmpcWorkspace.QGx[ 250 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 275 ]), &(nmpcWorkspace.QGx[ 275 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 300 ]), &(nmpcWorkspace.QGx[ 300 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 325 ]), &(nmpcWorkspace.QGx[ 325 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 350 ]), &(nmpcWorkspace.QGx[ 350 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 375 ]), &(nmpcWorkspace.QGx[ 375 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 400 ]), &(nmpcWorkspace.QGx[ 400 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 425 ]), &(nmpcWorkspace.QGx[ 425 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 450 ]), &(nmpcWorkspace.QGx[ 450 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 475 ]), &(nmpcWorkspace.QGx[ 475 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 500 ]), &(nmpcWorkspace.QGx[ 500 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 525 ]), &(nmpcWorkspace.QGx[ 525 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 550 ]), &(nmpcWorkspace.QGx[ 550 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 575 ]), &(nmpcWorkspace.QGx[ 575 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 600 ]), &(nmpcWorkspace.QGx[ 600 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 625 ]), &(nmpcWorkspace.QGx[ 625 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 650 ]), &(nmpcWorkspace.QGx[ 650 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 675 ]), &(nmpcWorkspace.QGx[ 675 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 700 ]), &(nmpcWorkspace.QGx[ 700 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 725 ]), &(nmpcWorkspace.QGx[ 725 ]) );

for (lRun1 = 0; lRun1 < 30; ++lRun1)
{
nmpc_zeroBlockH10( &(nmpcWorkspace.H10[ lRun1 * 10 ]) );
for (lRun2 = lRun1; lRun2 < 30; ++lRun2)
{
lRun3 = (((lRun2 + 1) * (lRun2)) / (2)) + (lRun1);
nmpc_multQETGx( &(nmpcWorkspace.QE[ lRun3 * 10 ]), &(nmpcWorkspace.evGx[ lRun2 * 25 ]), &(nmpcWorkspace.H10[ lRun1 * 10 ]) );
}
}

for (lRun1 = 0;lRun1 < 5; ++lRun1)
for (lRun2 = 0;lRun2 < 60; ++lRun2)
nmpcWorkspace.H[(lRun1 * 65) + (lRun2 + 5)] = nmpcWorkspace.H10[(lRun2 * 5) + (lRun1)];

for (lRun1 = 0; lRun1 < 30; ++lRun1)
{
nmpc_setBlockH11_R1( lRun1, lRun1, &(nmpcWorkspace.R1[ lRun1 * 4 ]) );
lRun2 = lRun1;
for (lRun3 = lRun1; lRun3 < 30; ++lRun3)
{
lRun4 = (((lRun3 + 1) * (lRun3)) / (2)) + (lRun1);
lRun5 = (((lRun3 + 1) * (lRun3)) / (2)) + (lRun2);
nmpc_setBlockH11( lRun1, lRun2, &(nmpcWorkspace.E[ lRun4 * 10 ]), &(nmpcWorkspace.QE[ lRun5 * 10 ]) );
}
for (lRun2 = lRun1 + 1; lRun2 < 30; ++lRun2)
{
nmpc_zeroBlockH11( lRun1, lRun2 );
for (lRun3 = lRun2; lRun3 < 30; ++lRun3)
{
lRun4 = (((lRun3 + 1) * (lRun3)) / (2)) + (lRun1);
lRun5 = (((lRun3 + 1) * (lRun3)) / (2)) + (lRun2);
nmpc_setBlockH11( lRun1, lRun2, &(nmpcWorkspace.E[ lRun4 * 10 ]), &(nmpcWorkspace.QE[ lRun5 * 10 ]) );
}
}
}

for (lRun1 = 0; lRun1 < 30; ++lRun1)
{
for (lRun2 = 0; lRun2 < lRun1; ++lRun2)
{
nmpc_copyHTH( lRun1, lRun2 );
}
}

for (lRun1 = 0;lRun1 < 60; ++lRun1)
for (lRun2 = 0;lRun2 < 5; ++lRun2)
nmpcWorkspace.H[(lRun1 * 65 + 325) + (lRun2)] = nmpcWorkspace.H10[(lRun1 * 5) + (lRun2)];

nmpc_multQ1d( &(nmpcWorkspace.Q1[ 25 ]), nmpcWorkspace.d, nmpcWorkspace.Qd );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 50 ]), &(nmpcWorkspace.d[ 5 ]), &(nmpcWorkspace.Qd[ 5 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 75 ]), &(nmpcWorkspace.d[ 10 ]), &(nmpcWorkspace.Qd[ 10 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 100 ]), &(nmpcWorkspace.d[ 15 ]), &(nmpcWorkspace.Qd[ 15 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 125 ]), &(nmpcWorkspace.d[ 20 ]), &(nmpcWorkspace.Qd[ 20 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 150 ]), &(nmpcWorkspace.d[ 25 ]), &(nmpcWorkspace.Qd[ 25 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 175 ]), &(nmpcWorkspace.d[ 30 ]), &(nmpcWorkspace.Qd[ 30 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 200 ]), &(nmpcWorkspace.d[ 35 ]), &(nmpcWorkspace.Qd[ 35 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 225 ]), &(nmpcWorkspace.d[ 40 ]), &(nmpcWorkspace.Qd[ 40 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 250 ]), &(nmpcWorkspace.d[ 45 ]), &(nmpcWorkspace.Qd[ 45 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 275 ]), &(nmpcWorkspace.d[ 50 ]), &(nmpcWorkspace.Qd[ 50 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 300 ]), &(nmpcWorkspace.d[ 55 ]), &(nmpcWorkspace.Qd[ 55 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 325 ]), &(nmpcWorkspace.d[ 60 ]), &(nmpcWorkspace.Qd[ 60 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 350 ]), &(nmpcWorkspace.d[ 65 ]), &(nmpcWorkspace.Qd[ 65 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 375 ]), &(nmpcWorkspace.d[ 70 ]), &(nmpcWorkspace.Qd[ 70 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 400 ]), &(nmpcWorkspace.d[ 75 ]), &(nmpcWorkspace.Qd[ 75 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 425 ]), &(nmpcWorkspace.d[ 80 ]), &(nmpcWorkspace.Qd[ 80 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 450 ]), &(nmpcWorkspace.d[ 85 ]), &(nmpcWorkspace.Qd[ 85 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 475 ]), &(nmpcWorkspace.d[ 90 ]), &(nmpcWorkspace.Qd[ 90 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 500 ]), &(nmpcWorkspace.d[ 95 ]), &(nmpcWorkspace.Qd[ 95 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 525 ]), &(nmpcWorkspace.d[ 100 ]), &(nmpcWorkspace.Qd[ 100 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 550 ]), &(nmpcWorkspace.d[ 105 ]), &(nmpcWorkspace.Qd[ 105 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 575 ]), &(nmpcWorkspace.d[ 110 ]), &(nmpcWorkspace.Qd[ 110 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 600 ]), &(nmpcWorkspace.d[ 115 ]), &(nmpcWorkspace.Qd[ 115 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 625 ]), &(nmpcWorkspace.d[ 120 ]), &(nmpcWorkspace.Qd[ 120 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 650 ]), &(nmpcWorkspace.d[ 125 ]), &(nmpcWorkspace.Qd[ 125 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 675 ]), &(nmpcWorkspace.d[ 130 ]), &(nmpcWorkspace.Qd[ 130 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 700 ]), &(nmpcWorkspace.d[ 135 ]), &(nmpcWorkspace.Qd[ 135 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 725 ]), &(nmpcWorkspace.d[ 140 ]), &(nmpcWorkspace.Qd[ 140 ]) );
nmpc_multQN1d( nmpcWorkspace.QN1, &(nmpcWorkspace.d[ 145 ]), &(nmpcWorkspace.Qd[ 145 ]) );

nmpc_macCTSlx( nmpcWorkspace.evGx, nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 25 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 50 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 75 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 100 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 125 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 150 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 175 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 200 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 225 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 250 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 275 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 300 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 325 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 350 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 375 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 400 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 425 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 450 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 475 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 500 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 525 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 550 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 575 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 600 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 625 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 650 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 675 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 700 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 725 ]), nmpcWorkspace.g );
for (lRun1 = 0; lRun1 < 30; ++lRun1)
{
for (lRun2 = lRun1; lRun2 < 30; ++lRun2)
{
lRun3 = (((lRun2 + 1) * (lRun2)) / (2)) + (lRun1);
nmpc_macETSlu( &(nmpcWorkspace.QE[ lRun3 * 10 ]), &(nmpcWorkspace.g[ lRun1 * 2 + 5 ]) );
}
}
nmpcWorkspace.lb[5] = (real_t)-2.0000000000000000e+00 - nmpcVariables.u[0];
nmpcWorkspace.lb[6] = (real_t)-2.0000000000000000e+00 - nmpcVariables.u[1];
nmpcWorkspace.lb[7] = (real_t)-2.0000000000000000e+00 - nmpcVariables.u[2];
nmpcWorkspace.lb[8] = (real_t)-2.0000000000000000e+00 - nmpcVariables.u[3];
nmpcWorkspace.lb[9] = (real_t)-2.0000000000000000e+00 - nmpcVariables.u[4];
nmpcWorkspace.lb[10] = (real_t)-2.0000000000000000e+00 - nmpcVariables.u[5];
nmpcWorkspace.lb[11] = (real_t)-2.0000000000000000e+00 - nmpcVariables.u[6];
nmpcWorkspace.lb[12] = (real_t)-2.0000000000000000e+00 - nmpcVariables.u[7];
nmpcWorkspace.lb[13] = (real_t)-2.0000000000000000e+00 - nmpcVariables.u[8];
nmpcWorkspace.lb[14] = (real_t)-2.0000000000000000e+00 - nmpcVariables.u[9];
nmpcWorkspace.lb[15] = (real_t)-2.0000000000000000e+00 - nmpcVariables.u[10];
nmpcWorkspace.lb[16] = (real_t)-2.0000000000000000e+00 - nmpcVariables.u[11];
nmpcWorkspace.lb[17] = (real_t)-2.0000000000000000e+00 - nmpcVariables.u[12];
nmpcWorkspace.lb[18] = (real_t)-2.0000000000000000e+00 - nmpcVariables.u[13];
nmpcWorkspace.lb[19] = (real_t)-2.0000000000000000e+00 - nmpcVariables.u[14];
nmpcWorkspace.lb[20] = (real_t)-2.0000000000000000e+00 - nmpcVariables.u[15];
nmpcWorkspace.lb[21] = (real_t)-2.0000000000000000e+00 - nmpcVariables.u[16];
nmpcWorkspace.lb[22] = (real_t)-2.0000000000000000e+00 - nmpcVariables.u[17];
nmpcWorkspace.lb[23] = (real_t)-2.0000000000000000e+00 - nmpcVariables.u[18];
nmpcWorkspace.lb[24] = (real_t)-2.0000000000000000e+00 - nmpcVariables.u[19];
nmpcWorkspace.lb[25] = (real_t)-2.0000000000000000e+00 - nmpcVariables.u[20];
nmpcWorkspace.lb[26] = (real_t)-2.0000000000000000e+00 - nmpcVariables.u[21];
nmpcWorkspace.lb[27] = (real_t)-2.0000000000000000e+00 - nmpcVariables.u[22];
nmpcWorkspace.lb[28] = (real_t)-2.0000000000000000e+00 - nmpcVariables.u[23];
nmpcWorkspace.lb[29] = (real_t)-2.0000000000000000e+00 - nmpcVariables.u[24];
nmpcWorkspace.lb[30] = (real_t)-2.0000000000000000e+00 - nmpcVariables.u[25];
nmpcWorkspace.lb[31] = (real_t)-2.0000000000000000e+00 - nmpcVariables.u[26];
nmpcWorkspace.lb[32] = (real_t)-2.0000000000000000e+00 - nmpcVariables.u[27];
nmpcWorkspace.lb[33] = (real_t)-2.0000000000000000e+00 - nmpcVariables.u[28];
nmpcWorkspace.lb[34] = (real_t)-2.0000000000000000e+00 - nmpcVariables.u[29];
nmpcWorkspace.lb[35] = (real_t)-2.0000000000000000e+00 - nmpcVariables.u[30];
nmpcWorkspace.lb[36] = (real_t)-2.0000000000000000e+00 - nmpcVariables.u[31];
nmpcWorkspace.lb[37] = (real_t)-2.0000000000000000e+00 - nmpcVariables.u[32];
nmpcWorkspace.lb[38] = (real_t)-2.0000000000000000e+00 - nmpcVariables.u[33];
nmpcWorkspace.lb[39] = (real_t)-2.0000000000000000e+00 - nmpcVariables.u[34];
nmpcWorkspace.lb[40] = (real_t)-2.0000000000000000e+00 - nmpcVariables.u[35];
nmpcWorkspace.lb[41] = (real_t)-2.0000000000000000e+00 - nmpcVariables.u[36];
nmpcWorkspace.lb[42] = (real_t)-2.0000000000000000e+00 - nmpcVariables.u[37];
nmpcWorkspace.lb[43] = (real_t)-2.0000000000000000e+00 - nmpcVariables.u[38];
nmpcWorkspace.lb[44] = (real_t)-2.0000000000000000e+00 - nmpcVariables.u[39];
nmpcWorkspace.lb[45] = (real_t)-2.0000000000000000e+00 - nmpcVariables.u[40];
nmpcWorkspace.lb[46] = (real_t)-2.0000000000000000e+00 - nmpcVariables.u[41];
nmpcWorkspace.lb[47] = (real_t)-2.0000000000000000e+00 - nmpcVariables.u[42];
nmpcWorkspace.lb[48] = (real_t)-2.0000000000000000e+00 - nmpcVariables.u[43];
nmpcWorkspace.lb[49] = (real_t)-2.0000000000000000e+00 - nmpcVariables.u[44];
nmpcWorkspace.lb[50] = (real_t)-2.0000000000000000e+00 - nmpcVariables.u[45];
nmpcWorkspace.lb[51] = (real_t)-2.0000000000000000e+00 - nmpcVariables.u[46];
nmpcWorkspace.lb[52] = (real_t)-2.0000000000000000e+00 - nmpcVariables.u[47];
nmpcWorkspace.lb[53] = (real_t)-2.0000000000000000e+00 - nmpcVariables.u[48];
nmpcWorkspace.lb[54] = (real_t)-2.0000000000000000e+00 - nmpcVariables.u[49];
nmpcWorkspace.lb[55] = (real_t)-2.0000000000000000e+00 - nmpcVariables.u[50];
nmpcWorkspace.lb[56] = (real_t)-2.0000000000000000e+00 - nmpcVariables.u[51];
nmpcWorkspace.lb[57] = (real_t)-2.0000000000000000e+00 - nmpcVariables.u[52];
nmpcWorkspace.lb[58] = (real_t)-2.0000000000000000e+00 - nmpcVariables.u[53];
nmpcWorkspace.lb[59] = (real_t)-2.0000000000000000e+00 - nmpcVariables.u[54];
nmpcWorkspace.lb[60] = (real_t)-2.0000000000000000e+00 - nmpcVariables.u[55];
nmpcWorkspace.lb[61] = (real_t)-2.0000000000000000e+00 - nmpcVariables.u[56];
nmpcWorkspace.lb[62] = (real_t)-2.0000000000000000e+00 - nmpcVariables.u[57];
nmpcWorkspace.lb[63] = (real_t)-2.0000000000000000e+00 - nmpcVariables.u[58];
nmpcWorkspace.lb[64] = (real_t)-2.0000000000000000e+00 - nmpcVariables.u[59];
nmpcWorkspace.ub[5] = (real_t)2.0000000000000000e+00 - nmpcVariables.u[0];
nmpcWorkspace.ub[6] = (real_t)2.0000000000000000e+00 - nmpcVariables.u[1];
nmpcWorkspace.ub[7] = (real_t)2.0000000000000000e+00 - nmpcVariables.u[2];
nmpcWorkspace.ub[8] = (real_t)2.0000000000000000e+00 - nmpcVariables.u[3];
nmpcWorkspace.ub[9] = (real_t)2.0000000000000000e+00 - nmpcVariables.u[4];
nmpcWorkspace.ub[10] = (real_t)2.0000000000000000e+00 - nmpcVariables.u[5];
nmpcWorkspace.ub[11] = (real_t)2.0000000000000000e+00 - nmpcVariables.u[6];
nmpcWorkspace.ub[12] = (real_t)2.0000000000000000e+00 - nmpcVariables.u[7];
nmpcWorkspace.ub[13] = (real_t)2.0000000000000000e+00 - nmpcVariables.u[8];
nmpcWorkspace.ub[14] = (real_t)2.0000000000000000e+00 - nmpcVariables.u[9];
nmpcWorkspace.ub[15] = (real_t)2.0000000000000000e+00 - nmpcVariables.u[10];
nmpcWorkspace.ub[16] = (real_t)2.0000000000000000e+00 - nmpcVariables.u[11];
nmpcWorkspace.ub[17] = (real_t)2.0000000000000000e+00 - nmpcVariables.u[12];
nmpcWorkspace.ub[18] = (real_t)2.0000000000000000e+00 - nmpcVariables.u[13];
nmpcWorkspace.ub[19] = (real_t)2.0000000000000000e+00 - nmpcVariables.u[14];
nmpcWorkspace.ub[20] = (real_t)2.0000000000000000e+00 - nmpcVariables.u[15];
nmpcWorkspace.ub[21] = (real_t)2.0000000000000000e+00 - nmpcVariables.u[16];
nmpcWorkspace.ub[22] = (real_t)2.0000000000000000e+00 - nmpcVariables.u[17];
nmpcWorkspace.ub[23] = (real_t)2.0000000000000000e+00 - nmpcVariables.u[18];
nmpcWorkspace.ub[24] = (real_t)2.0000000000000000e+00 - nmpcVariables.u[19];
nmpcWorkspace.ub[25] = (real_t)2.0000000000000000e+00 - nmpcVariables.u[20];
nmpcWorkspace.ub[26] = (real_t)2.0000000000000000e+00 - nmpcVariables.u[21];
nmpcWorkspace.ub[27] = (real_t)2.0000000000000000e+00 - nmpcVariables.u[22];
nmpcWorkspace.ub[28] = (real_t)2.0000000000000000e+00 - nmpcVariables.u[23];
nmpcWorkspace.ub[29] = (real_t)2.0000000000000000e+00 - nmpcVariables.u[24];
nmpcWorkspace.ub[30] = (real_t)2.0000000000000000e+00 - nmpcVariables.u[25];
nmpcWorkspace.ub[31] = (real_t)2.0000000000000000e+00 - nmpcVariables.u[26];
nmpcWorkspace.ub[32] = (real_t)2.0000000000000000e+00 - nmpcVariables.u[27];
nmpcWorkspace.ub[33] = (real_t)2.0000000000000000e+00 - nmpcVariables.u[28];
nmpcWorkspace.ub[34] = (real_t)2.0000000000000000e+00 - nmpcVariables.u[29];
nmpcWorkspace.ub[35] = (real_t)2.0000000000000000e+00 - nmpcVariables.u[30];
nmpcWorkspace.ub[36] = (real_t)2.0000000000000000e+00 - nmpcVariables.u[31];
nmpcWorkspace.ub[37] = (real_t)2.0000000000000000e+00 - nmpcVariables.u[32];
nmpcWorkspace.ub[38] = (real_t)2.0000000000000000e+00 - nmpcVariables.u[33];
nmpcWorkspace.ub[39] = (real_t)2.0000000000000000e+00 - nmpcVariables.u[34];
nmpcWorkspace.ub[40] = (real_t)2.0000000000000000e+00 - nmpcVariables.u[35];
nmpcWorkspace.ub[41] = (real_t)2.0000000000000000e+00 - nmpcVariables.u[36];
nmpcWorkspace.ub[42] = (real_t)2.0000000000000000e+00 - nmpcVariables.u[37];
nmpcWorkspace.ub[43] = (real_t)2.0000000000000000e+00 - nmpcVariables.u[38];
nmpcWorkspace.ub[44] = (real_t)2.0000000000000000e+00 - nmpcVariables.u[39];
nmpcWorkspace.ub[45] = (real_t)2.0000000000000000e+00 - nmpcVariables.u[40];
nmpcWorkspace.ub[46] = (real_t)2.0000000000000000e+00 - nmpcVariables.u[41];
nmpcWorkspace.ub[47] = (real_t)2.0000000000000000e+00 - nmpcVariables.u[42];
nmpcWorkspace.ub[48] = (real_t)2.0000000000000000e+00 - nmpcVariables.u[43];
nmpcWorkspace.ub[49] = (real_t)2.0000000000000000e+00 - nmpcVariables.u[44];
nmpcWorkspace.ub[50] = (real_t)2.0000000000000000e+00 - nmpcVariables.u[45];
nmpcWorkspace.ub[51] = (real_t)2.0000000000000000e+00 - nmpcVariables.u[46];
nmpcWorkspace.ub[52] = (real_t)2.0000000000000000e+00 - nmpcVariables.u[47];
nmpcWorkspace.ub[53] = (real_t)2.0000000000000000e+00 - nmpcVariables.u[48];
nmpcWorkspace.ub[54] = (real_t)2.0000000000000000e+00 - nmpcVariables.u[49];
nmpcWorkspace.ub[55] = (real_t)2.0000000000000000e+00 - nmpcVariables.u[50];
nmpcWorkspace.ub[56] = (real_t)2.0000000000000000e+00 - nmpcVariables.u[51];
nmpcWorkspace.ub[57] = (real_t)2.0000000000000000e+00 - nmpcVariables.u[52];
nmpcWorkspace.ub[58] = (real_t)2.0000000000000000e+00 - nmpcVariables.u[53];
nmpcWorkspace.ub[59] = (real_t)2.0000000000000000e+00 - nmpcVariables.u[54];
nmpcWorkspace.ub[60] = (real_t)2.0000000000000000e+00 - nmpcVariables.u[55];
nmpcWorkspace.ub[61] = (real_t)2.0000000000000000e+00 - nmpcVariables.u[56];
nmpcWorkspace.ub[62] = (real_t)2.0000000000000000e+00 - nmpcVariables.u[57];
nmpcWorkspace.ub[63] = (real_t)2.0000000000000000e+00 - nmpcVariables.u[58];
nmpcWorkspace.ub[64] = (real_t)2.0000000000000000e+00 - nmpcVariables.u[59];

for (lRun1 = 0; lRun1 < 60; ++lRun1)
{
lRun3 = xBoundIndices[ lRun1 ] - 5;
lRun4 = ((lRun3) / (5)) + (1);
nmpcWorkspace.A[lRun1 * 65] = nmpcWorkspace.evGx[lRun3 * 5];
nmpcWorkspace.A[lRun1 * 65 + 1] = nmpcWorkspace.evGx[lRun3 * 5 + 1];
nmpcWorkspace.A[lRun1 * 65 + 2] = nmpcWorkspace.evGx[lRun3 * 5 + 2];
nmpcWorkspace.A[lRun1 * 65 + 3] = nmpcWorkspace.evGx[lRun3 * 5 + 3];
nmpcWorkspace.A[lRun1 * 65 + 4] = nmpcWorkspace.evGx[lRun3 * 5 + 4];
for (lRun2 = 0; lRun2 < lRun4; ++lRun2)
{
lRun5 = (((((lRun4) * (lRun4-1)) / (2)) + (lRun2)) * (5)) + ((lRun3) % (5));
nmpcWorkspace.A[(lRun1 * 65) + (lRun2 * 2 + 5)] = nmpcWorkspace.E[lRun5 * 2];
nmpcWorkspace.A[(lRun1 * 65) + (lRun2 * 2 + 6)] = nmpcWorkspace.E[lRun5 * 2 + 1];
}
}

}

void nmpc_condenseFdb(  )
{
int lRun1;
int lRun2;
int lRun3;
real_t tmp;

nmpcWorkspace.Dx0[0] = nmpcVariables.x0[0] - nmpcVariables.x[0];
nmpcWorkspace.Dx0[1] = nmpcVariables.x0[1] - nmpcVariables.x[1];
nmpcWorkspace.Dx0[2] = nmpcVariables.x0[2] - nmpcVariables.x[2];
nmpcWorkspace.Dx0[3] = nmpcVariables.x0[3] - nmpcVariables.x[3];
nmpcWorkspace.Dx0[4] = nmpcVariables.x0[4] - nmpcVariables.x[4];

for (lRun2 = 0; lRun2 < 210; ++lRun2)
nmpcWorkspace.Dy[lRun2] -= nmpcVariables.y[lRun2];

nmpcWorkspace.DyN[0] -= nmpcVariables.yN[0];
nmpcWorkspace.DyN[1] -= nmpcVariables.yN[1];
nmpcWorkspace.DyN[2] -= nmpcVariables.yN[2];
nmpcWorkspace.DyN[3] -= nmpcVariables.yN[3];
nmpcWorkspace.DyN[4] -= nmpcVariables.yN[4];

nmpc_multRDy( nmpcWorkspace.R2, nmpcWorkspace.Dy, &(nmpcWorkspace.g[ 5 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 14 ]), &(nmpcWorkspace.Dy[ 7 ]), &(nmpcWorkspace.g[ 7 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 28 ]), &(nmpcWorkspace.Dy[ 14 ]), &(nmpcWorkspace.g[ 9 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 42 ]), &(nmpcWorkspace.Dy[ 21 ]), &(nmpcWorkspace.g[ 11 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 56 ]), &(nmpcWorkspace.Dy[ 28 ]), &(nmpcWorkspace.g[ 13 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 70 ]), &(nmpcWorkspace.Dy[ 35 ]), &(nmpcWorkspace.g[ 15 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 84 ]), &(nmpcWorkspace.Dy[ 42 ]), &(nmpcWorkspace.g[ 17 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 98 ]), &(nmpcWorkspace.Dy[ 49 ]), &(nmpcWorkspace.g[ 19 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 112 ]), &(nmpcWorkspace.Dy[ 56 ]), &(nmpcWorkspace.g[ 21 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 126 ]), &(nmpcWorkspace.Dy[ 63 ]), &(nmpcWorkspace.g[ 23 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 140 ]), &(nmpcWorkspace.Dy[ 70 ]), &(nmpcWorkspace.g[ 25 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 154 ]), &(nmpcWorkspace.Dy[ 77 ]), &(nmpcWorkspace.g[ 27 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 168 ]), &(nmpcWorkspace.Dy[ 84 ]), &(nmpcWorkspace.g[ 29 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 182 ]), &(nmpcWorkspace.Dy[ 91 ]), &(nmpcWorkspace.g[ 31 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 196 ]), &(nmpcWorkspace.Dy[ 98 ]), &(nmpcWorkspace.g[ 33 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 210 ]), &(nmpcWorkspace.Dy[ 105 ]), &(nmpcWorkspace.g[ 35 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 224 ]), &(nmpcWorkspace.Dy[ 112 ]), &(nmpcWorkspace.g[ 37 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 238 ]), &(nmpcWorkspace.Dy[ 119 ]), &(nmpcWorkspace.g[ 39 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 252 ]), &(nmpcWorkspace.Dy[ 126 ]), &(nmpcWorkspace.g[ 41 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 266 ]), &(nmpcWorkspace.Dy[ 133 ]), &(nmpcWorkspace.g[ 43 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 280 ]), &(nmpcWorkspace.Dy[ 140 ]), &(nmpcWorkspace.g[ 45 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 294 ]), &(nmpcWorkspace.Dy[ 147 ]), &(nmpcWorkspace.g[ 47 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 308 ]), &(nmpcWorkspace.Dy[ 154 ]), &(nmpcWorkspace.g[ 49 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 322 ]), &(nmpcWorkspace.Dy[ 161 ]), &(nmpcWorkspace.g[ 51 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 336 ]), &(nmpcWorkspace.Dy[ 168 ]), &(nmpcWorkspace.g[ 53 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 350 ]), &(nmpcWorkspace.Dy[ 175 ]), &(nmpcWorkspace.g[ 55 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 364 ]), &(nmpcWorkspace.Dy[ 182 ]), &(nmpcWorkspace.g[ 57 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 378 ]), &(nmpcWorkspace.Dy[ 189 ]), &(nmpcWorkspace.g[ 59 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 392 ]), &(nmpcWorkspace.Dy[ 196 ]), &(nmpcWorkspace.g[ 61 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 406 ]), &(nmpcWorkspace.Dy[ 203 ]), &(nmpcWorkspace.g[ 63 ]) );

nmpc_multQDy( nmpcWorkspace.Q2, nmpcWorkspace.Dy, nmpcWorkspace.QDy );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 35 ]), &(nmpcWorkspace.Dy[ 7 ]), &(nmpcWorkspace.QDy[ 5 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 70 ]), &(nmpcWorkspace.Dy[ 14 ]), &(nmpcWorkspace.QDy[ 10 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 105 ]), &(nmpcWorkspace.Dy[ 21 ]), &(nmpcWorkspace.QDy[ 15 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 140 ]), &(nmpcWorkspace.Dy[ 28 ]), &(nmpcWorkspace.QDy[ 20 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 175 ]), &(nmpcWorkspace.Dy[ 35 ]), &(nmpcWorkspace.QDy[ 25 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 210 ]), &(nmpcWorkspace.Dy[ 42 ]), &(nmpcWorkspace.QDy[ 30 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 245 ]), &(nmpcWorkspace.Dy[ 49 ]), &(nmpcWorkspace.QDy[ 35 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 280 ]), &(nmpcWorkspace.Dy[ 56 ]), &(nmpcWorkspace.QDy[ 40 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 315 ]), &(nmpcWorkspace.Dy[ 63 ]), &(nmpcWorkspace.QDy[ 45 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 350 ]), &(nmpcWorkspace.Dy[ 70 ]), &(nmpcWorkspace.QDy[ 50 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 385 ]), &(nmpcWorkspace.Dy[ 77 ]), &(nmpcWorkspace.QDy[ 55 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 420 ]), &(nmpcWorkspace.Dy[ 84 ]), &(nmpcWorkspace.QDy[ 60 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 455 ]), &(nmpcWorkspace.Dy[ 91 ]), &(nmpcWorkspace.QDy[ 65 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 490 ]), &(nmpcWorkspace.Dy[ 98 ]), &(nmpcWorkspace.QDy[ 70 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 525 ]), &(nmpcWorkspace.Dy[ 105 ]), &(nmpcWorkspace.QDy[ 75 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 560 ]), &(nmpcWorkspace.Dy[ 112 ]), &(nmpcWorkspace.QDy[ 80 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 595 ]), &(nmpcWorkspace.Dy[ 119 ]), &(nmpcWorkspace.QDy[ 85 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 630 ]), &(nmpcWorkspace.Dy[ 126 ]), &(nmpcWorkspace.QDy[ 90 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 665 ]), &(nmpcWorkspace.Dy[ 133 ]), &(nmpcWorkspace.QDy[ 95 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 700 ]), &(nmpcWorkspace.Dy[ 140 ]), &(nmpcWorkspace.QDy[ 100 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 735 ]), &(nmpcWorkspace.Dy[ 147 ]), &(nmpcWorkspace.QDy[ 105 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 770 ]), &(nmpcWorkspace.Dy[ 154 ]), &(nmpcWorkspace.QDy[ 110 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 805 ]), &(nmpcWorkspace.Dy[ 161 ]), &(nmpcWorkspace.QDy[ 115 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 840 ]), &(nmpcWorkspace.Dy[ 168 ]), &(nmpcWorkspace.QDy[ 120 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 875 ]), &(nmpcWorkspace.Dy[ 175 ]), &(nmpcWorkspace.QDy[ 125 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 910 ]), &(nmpcWorkspace.Dy[ 182 ]), &(nmpcWorkspace.QDy[ 130 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 945 ]), &(nmpcWorkspace.Dy[ 189 ]), &(nmpcWorkspace.QDy[ 135 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 980 ]), &(nmpcWorkspace.Dy[ 196 ]), &(nmpcWorkspace.QDy[ 140 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 1015 ]), &(nmpcWorkspace.Dy[ 203 ]), &(nmpcWorkspace.QDy[ 145 ]) );

nmpcWorkspace.QDy[150] = + nmpcWorkspace.QN2[0]*nmpcWorkspace.DyN[0] + nmpcWorkspace.QN2[1]*nmpcWorkspace.DyN[1] + nmpcWorkspace.QN2[2]*nmpcWorkspace.DyN[2] + nmpcWorkspace.QN2[3]*nmpcWorkspace.DyN[3] + nmpcWorkspace.QN2[4]*nmpcWorkspace.DyN[4];
nmpcWorkspace.QDy[151] = + nmpcWorkspace.QN2[5]*nmpcWorkspace.DyN[0] + nmpcWorkspace.QN2[6]*nmpcWorkspace.DyN[1] + nmpcWorkspace.QN2[7]*nmpcWorkspace.DyN[2] + nmpcWorkspace.QN2[8]*nmpcWorkspace.DyN[3] + nmpcWorkspace.QN2[9]*nmpcWorkspace.DyN[4];
nmpcWorkspace.QDy[152] = + nmpcWorkspace.QN2[10]*nmpcWorkspace.DyN[0] + nmpcWorkspace.QN2[11]*nmpcWorkspace.DyN[1] + nmpcWorkspace.QN2[12]*nmpcWorkspace.DyN[2] + nmpcWorkspace.QN2[13]*nmpcWorkspace.DyN[3] + nmpcWorkspace.QN2[14]*nmpcWorkspace.DyN[4];
nmpcWorkspace.QDy[153] = + nmpcWorkspace.QN2[15]*nmpcWorkspace.DyN[0] + nmpcWorkspace.QN2[16]*nmpcWorkspace.DyN[1] + nmpcWorkspace.QN2[17]*nmpcWorkspace.DyN[2] + nmpcWorkspace.QN2[18]*nmpcWorkspace.DyN[3] + nmpcWorkspace.QN2[19]*nmpcWorkspace.DyN[4];
nmpcWorkspace.QDy[154] = + nmpcWorkspace.QN2[20]*nmpcWorkspace.DyN[0] + nmpcWorkspace.QN2[21]*nmpcWorkspace.DyN[1] + nmpcWorkspace.QN2[22]*nmpcWorkspace.DyN[2] + nmpcWorkspace.QN2[23]*nmpcWorkspace.DyN[3] + nmpcWorkspace.QN2[24]*nmpcWorkspace.DyN[4];

for (lRun2 = 0; lRun2 < 150; ++lRun2)
nmpcWorkspace.QDy[lRun2 + 5] += nmpcWorkspace.Qd[lRun2];


nmpcWorkspace.g[0] = + nmpcWorkspace.evGx[0]*nmpcWorkspace.QDy[5] + nmpcWorkspace.evGx[5]*nmpcWorkspace.QDy[6] + nmpcWorkspace.evGx[10]*nmpcWorkspace.QDy[7] + nmpcWorkspace.evGx[15]*nmpcWorkspace.QDy[8] + nmpcWorkspace.evGx[20]*nmpcWorkspace.QDy[9] + nmpcWorkspace.evGx[25]*nmpcWorkspace.QDy[10] + nmpcWorkspace.evGx[30]*nmpcWorkspace.QDy[11] + nmpcWorkspace.evGx[35]*nmpcWorkspace.QDy[12] + nmpcWorkspace.evGx[40]*nmpcWorkspace.QDy[13] + nmpcWorkspace.evGx[45]*nmpcWorkspace.QDy[14] + nmpcWorkspace.evGx[50]*nmpcWorkspace.QDy[15] + nmpcWorkspace.evGx[55]*nmpcWorkspace.QDy[16] + nmpcWorkspace.evGx[60]*nmpcWorkspace.QDy[17] + nmpcWorkspace.evGx[65]*nmpcWorkspace.QDy[18] + nmpcWorkspace.evGx[70]*nmpcWorkspace.QDy[19] + nmpcWorkspace.evGx[75]*nmpcWorkspace.QDy[20] + nmpcWorkspace.evGx[80]*nmpcWorkspace.QDy[21] + nmpcWorkspace.evGx[85]*nmpcWorkspace.QDy[22] + nmpcWorkspace.evGx[90]*nmpcWorkspace.QDy[23] + nmpcWorkspace.evGx[95]*nmpcWorkspace.QDy[24] + nmpcWorkspace.evGx[100]*nmpcWorkspace.QDy[25] + nmpcWorkspace.evGx[105]*nmpcWorkspace.QDy[26] + nmpcWorkspace.evGx[110]*nmpcWorkspace.QDy[27] + nmpcWorkspace.evGx[115]*nmpcWorkspace.QDy[28] + nmpcWorkspace.evGx[120]*nmpcWorkspace.QDy[29] + nmpcWorkspace.evGx[125]*nmpcWorkspace.QDy[30] + nmpcWorkspace.evGx[130]*nmpcWorkspace.QDy[31] + nmpcWorkspace.evGx[135]*nmpcWorkspace.QDy[32] + nmpcWorkspace.evGx[140]*nmpcWorkspace.QDy[33] + nmpcWorkspace.evGx[145]*nmpcWorkspace.QDy[34] + nmpcWorkspace.evGx[150]*nmpcWorkspace.QDy[35] + nmpcWorkspace.evGx[155]*nmpcWorkspace.QDy[36] + nmpcWorkspace.evGx[160]*nmpcWorkspace.QDy[37] + nmpcWorkspace.evGx[165]*nmpcWorkspace.QDy[38] + nmpcWorkspace.evGx[170]*nmpcWorkspace.QDy[39] + nmpcWorkspace.evGx[175]*nmpcWorkspace.QDy[40] + nmpcWorkspace.evGx[180]*nmpcWorkspace.QDy[41] + nmpcWorkspace.evGx[185]*nmpcWorkspace.QDy[42] + nmpcWorkspace.evGx[190]*nmpcWorkspace.QDy[43] + nmpcWorkspace.evGx[195]*nmpcWorkspace.QDy[44] + nmpcWorkspace.evGx[200]*nmpcWorkspace.QDy[45] + nmpcWorkspace.evGx[205]*nmpcWorkspace.QDy[46] + nmpcWorkspace.evGx[210]*nmpcWorkspace.QDy[47] + nmpcWorkspace.evGx[215]*nmpcWorkspace.QDy[48] + nmpcWorkspace.evGx[220]*nmpcWorkspace.QDy[49] + nmpcWorkspace.evGx[225]*nmpcWorkspace.QDy[50] + nmpcWorkspace.evGx[230]*nmpcWorkspace.QDy[51] + nmpcWorkspace.evGx[235]*nmpcWorkspace.QDy[52] + nmpcWorkspace.evGx[240]*nmpcWorkspace.QDy[53] + nmpcWorkspace.evGx[245]*nmpcWorkspace.QDy[54] + nmpcWorkspace.evGx[250]*nmpcWorkspace.QDy[55] + nmpcWorkspace.evGx[255]*nmpcWorkspace.QDy[56] + nmpcWorkspace.evGx[260]*nmpcWorkspace.QDy[57] + nmpcWorkspace.evGx[265]*nmpcWorkspace.QDy[58] + nmpcWorkspace.evGx[270]*nmpcWorkspace.QDy[59] + nmpcWorkspace.evGx[275]*nmpcWorkspace.QDy[60] + nmpcWorkspace.evGx[280]*nmpcWorkspace.QDy[61] + nmpcWorkspace.evGx[285]*nmpcWorkspace.QDy[62] + nmpcWorkspace.evGx[290]*nmpcWorkspace.QDy[63] + nmpcWorkspace.evGx[295]*nmpcWorkspace.QDy[64] + nmpcWorkspace.evGx[300]*nmpcWorkspace.QDy[65] + nmpcWorkspace.evGx[305]*nmpcWorkspace.QDy[66] + nmpcWorkspace.evGx[310]*nmpcWorkspace.QDy[67] + nmpcWorkspace.evGx[315]*nmpcWorkspace.QDy[68] + nmpcWorkspace.evGx[320]*nmpcWorkspace.QDy[69] + nmpcWorkspace.evGx[325]*nmpcWorkspace.QDy[70] + nmpcWorkspace.evGx[330]*nmpcWorkspace.QDy[71] + nmpcWorkspace.evGx[335]*nmpcWorkspace.QDy[72] + nmpcWorkspace.evGx[340]*nmpcWorkspace.QDy[73] + nmpcWorkspace.evGx[345]*nmpcWorkspace.QDy[74] + nmpcWorkspace.evGx[350]*nmpcWorkspace.QDy[75] + nmpcWorkspace.evGx[355]*nmpcWorkspace.QDy[76] + nmpcWorkspace.evGx[360]*nmpcWorkspace.QDy[77] + nmpcWorkspace.evGx[365]*nmpcWorkspace.QDy[78] + nmpcWorkspace.evGx[370]*nmpcWorkspace.QDy[79] + nmpcWorkspace.evGx[375]*nmpcWorkspace.QDy[80] + nmpcWorkspace.evGx[380]*nmpcWorkspace.QDy[81] + nmpcWorkspace.evGx[385]*nmpcWorkspace.QDy[82] + nmpcWorkspace.evGx[390]*nmpcWorkspace.QDy[83] + nmpcWorkspace.evGx[395]*nmpcWorkspace.QDy[84] + nmpcWorkspace.evGx[400]*nmpcWorkspace.QDy[85] + nmpcWorkspace.evGx[405]*nmpcWorkspace.QDy[86] + nmpcWorkspace.evGx[410]*nmpcWorkspace.QDy[87] + nmpcWorkspace.evGx[415]*nmpcWorkspace.QDy[88] + nmpcWorkspace.evGx[420]*nmpcWorkspace.QDy[89] + nmpcWorkspace.evGx[425]*nmpcWorkspace.QDy[90] + nmpcWorkspace.evGx[430]*nmpcWorkspace.QDy[91] + nmpcWorkspace.evGx[435]*nmpcWorkspace.QDy[92] + nmpcWorkspace.evGx[440]*nmpcWorkspace.QDy[93] + nmpcWorkspace.evGx[445]*nmpcWorkspace.QDy[94] + nmpcWorkspace.evGx[450]*nmpcWorkspace.QDy[95] + nmpcWorkspace.evGx[455]*nmpcWorkspace.QDy[96] + nmpcWorkspace.evGx[460]*nmpcWorkspace.QDy[97] + nmpcWorkspace.evGx[465]*nmpcWorkspace.QDy[98] + nmpcWorkspace.evGx[470]*nmpcWorkspace.QDy[99] + nmpcWorkspace.evGx[475]*nmpcWorkspace.QDy[100] + nmpcWorkspace.evGx[480]*nmpcWorkspace.QDy[101] + nmpcWorkspace.evGx[485]*nmpcWorkspace.QDy[102] + nmpcWorkspace.evGx[490]*nmpcWorkspace.QDy[103] + nmpcWorkspace.evGx[495]*nmpcWorkspace.QDy[104] + nmpcWorkspace.evGx[500]*nmpcWorkspace.QDy[105] + nmpcWorkspace.evGx[505]*nmpcWorkspace.QDy[106] + nmpcWorkspace.evGx[510]*nmpcWorkspace.QDy[107] + nmpcWorkspace.evGx[515]*nmpcWorkspace.QDy[108] + nmpcWorkspace.evGx[520]*nmpcWorkspace.QDy[109] + nmpcWorkspace.evGx[525]*nmpcWorkspace.QDy[110] + nmpcWorkspace.evGx[530]*nmpcWorkspace.QDy[111] + nmpcWorkspace.evGx[535]*nmpcWorkspace.QDy[112] + nmpcWorkspace.evGx[540]*nmpcWorkspace.QDy[113] + nmpcWorkspace.evGx[545]*nmpcWorkspace.QDy[114] + nmpcWorkspace.evGx[550]*nmpcWorkspace.QDy[115] + nmpcWorkspace.evGx[555]*nmpcWorkspace.QDy[116] + nmpcWorkspace.evGx[560]*nmpcWorkspace.QDy[117] + nmpcWorkspace.evGx[565]*nmpcWorkspace.QDy[118] + nmpcWorkspace.evGx[570]*nmpcWorkspace.QDy[119] + nmpcWorkspace.evGx[575]*nmpcWorkspace.QDy[120] + nmpcWorkspace.evGx[580]*nmpcWorkspace.QDy[121] + nmpcWorkspace.evGx[585]*nmpcWorkspace.QDy[122] + nmpcWorkspace.evGx[590]*nmpcWorkspace.QDy[123] + nmpcWorkspace.evGx[595]*nmpcWorkspace.QDy[124] + nmpcWorkspace.evGx[600]*nmpcWorkspace.QDy[125] + nmpcWorkspace.evGx[605]*nmpcWorkspace.QDy[126] + nmpcWorkspace.evGx[610]*nmpcWorkspace.QDy[127] + nmpcWorkspace.evGx[615]*nmpcWorkspace.QDy[128] + nmpcWorkspace.evGx[620]*nmpcWorkspace.QDy[129] + nmpcWorkspace.evGx[625]*nmpcWorkspace.QDy[130] + nmpcWorkspace.evGx[630]*nmpcWorkspace.QDy[131] + nmpcWorkspace.evGx[635]*nmpcWorkspace.QDy[132] + nmpcWorkspace.evGx[640]*nmpcWorkspace.QDy[133] + nmpcWorkspace.evGx[645]*nmpcWorkspace.QDy[134] + nmpcWorkspace.evGx[650]*nmpcWorkspace.QDy[135] + nmpcWorkspace.evGx[655]*nmpcWorkspace.QDy[136] + nmpcWorkspace.evGx[660]*nmpcWorkspace.QDy[137] + nmpcWorkspace.evGx[665]*nmpcWorkspace.QDy[138] + nmpcWorkspace.evGx[670]*nmpcWorkspace.QDy[139] + nmpcWorkspace.evGx[675]*nmpcWorkspace.QDy[140] + nmpcWorkspace.evGx[680]*nmpcWorkspace.QDy[141] + nmpcWorkspace.evGx[685]*nmpcWorkspace.QDy[142] + nmpcWorkspace.evGx[690]*nmpcWorkspace.QDy[143] + nmpcWorkspace.evGx[695]*nmpcWorkspace.QDy[144] + nmpcWorkspace.evGx[700]*nmpcWorkspace.QDy[145] + nmpcWorkspace.evGx[705]*nmpcWorkspace.QDy[146] + nmpcWorkspace.evGx[710]*nmpcWorkspace.QDy[147] + nmpcWorkspace.evGx[715]*nmpcWorkspace.QDy[148] + nmpcWorkspace.evGx[720]*nmpcWorkspace.QDy[149] + nmpcWorkspace.evGx[725]*nmpcWorkspace.QDy[150] + nmpcWorkspace.evGx[730]*nmpcWorkspace.QDy[151] + nmpcWorkspace.evGx[735]*nmpcWorkspace.QDy[152] + nmpcWorkspace.evGx[740]*nmpcWorkspace.QDy[153] + nmpcWorkspace.evGx[745]*nmpcWorkspace.QDy[154];
nmpcWorkspace.g[1] = + nmpcWorkspace.evGx[1]*nmpcWorkspace.QDy[5] + nmpcWorkspace.evGx[6]*nmpcWorkspace.QDy[6] + nmpcWorkspace.evGx[11]*nmpcWorkspace.QDy[7] + nmpcWorkspace.evGx[16]*nmpcWorkspace.QDy[8] + nmpcWorkspace.evGx[21]*nmpcWorkspace.QDy[9] + nmpcWorkspace.evGx[26]*nmpcWorkspace.QDy[10] + nmpcWorkspace.evGx[31]*nmpcWorkspace.QDy[11] + nmpcWorkspace.evGx[36]*nmpcWorkspace.QDy[12] + nmpcWorkspace.evGx[41]*nmpcWorkspace.QDy[13] + nmpcWorkspace.evGx[46]*nmpcWorkspace.QDy[14] + nmpcWorkspace.evGx[51]*nmpcWorkspace.QDy[15] + nmpcWorkspace.evGx[56]*nmpcWorkspace.QDy[16] + nmpcWorkspace.evGx[61]*nmpcWorkspace.QDy[17] + nmpcWorkspace.evGx[66]*nmpcWorkspace.QDy[18] + nmpcWorkspace.evGx[71]*nmpcWorkspace.QDy[19] + nmpcWorkspace.evGx[76]*nmpcWorkspace.QDy[20] + nmpcWorkspace.evGx[81]*nmpcWorkspace.QDy[21] + nmpcWorkspace.evGx[86]*nmpcWorkspace.QDy[22] + nmpcWorkspace.evGx[91]*nmpcWorkspace.QDy[23] + nmpcWorkspace.evGx[96]*nmpcWorkspace.QDy[24] + nmpcWorkspace.evGx[101]*nmpcWorkspace.QDy[25] + nmpcWorkspace.evGx[106]*nmpcWorkspace.QDy[26] + nmpcWorkspace.evGx[111]*nmpcWorkspace.QDy[27] + nmpcWorkspace.evGx[116]*nmpcWorkspace.QDy[28] + nmpcWorkspace.evGx[121]*nmpcWorkspace.QDy[29] + nmpcWorkspace.evGx[126]*nmpcWorkspace.QDy[30] + nmpcWorkspace.evGx[131]*nmpcWorkspace.QDy[31] + nmpcWorkspace.evGx[136]*nmpcWorkspace.QDy[32] + nmpcWorkspace.evGx[141]*nmpcWorkspace.QDy[33] + nmpcWorkspace.evGx[146]*nmpcWorkspace.QDy[34] + nmpcWorkspace.evGx[151]*nmpcWorkspace.QDy[35] + nmpcWorkspace.evGx[156]*nmpcWorkspace.QDy[36] + nmpcWorkspace.evGx[161]*nmpcWorkspace.QDy[37] + nmpcWorkspace.evGx[166]*nmpcWorkspace.QDy[38] + nmpcWorkspace.evGx[171]*nmpcWorkspace.QDy[39] + nmpcWorkspace.evGx[176]*nmpcWorkspace.QDy[40] + nmpcWorkspace.evGx[181]*nmpcWorkspace.QDy[41] + nmpcWorkspace.evGx[186]*nmpcWorkspace.QDy[42] + nmpcWorkspace.evGx[191]*nmpcWorkspace.QDy[43] + nmpcWorkspace.evGx[196]*nmpcWorkspace.QDy[44] + nmpcWorkspace.evGx[201]*nmpcWorkspace.QDy[45] + nmpcWorkspace.evGx[206]*nmpcWorkspace.QDy[46] + nmpcWorkspace.evGx[211]*nmpcWorkspace.QDy[47] + nmpcWorkspace.evGx[216]*nmpcWorkspace.QDy[48] + nmpcWorkspace.evGx[221]*nmpcWorkspace.QDy[49] + nmpcWorkspace.evGx[226]*nmpcWorkspace.QDy[50] + nmpcWorkspace.evGx[231]*nmpcWorkspace.QDy[51] + nmpcWorkspace.evGx[236]*nmpcWorkspace.QDy[52] + nmpcWorkspace.evGx[241]*nmpcWorkspace.QDy[53] + nmpcWorkspace.evGx[246]*nmpcWorkspace.QDy[54] + nmpcWorkspace.evGx[251]*nmpcWorkspace.QDy[55] + nmpcWorkspace.evGx[256]*nmpcWorkspace.QDy[56] + nmpcWorkspace.evGx[261]*nmpcWorkspace.QDy[57] + nmpcWorkspace.evGx[266]*nmpcWorkspace.QDy[58] + nmpcWorkspace.evGx[271]*nmpcWorkspace.QDy[59] + nmpcWorkspace.evGx[276]*nmpcWorkspace.QDy[60] + nmpcWorkspace.evGx[281]*nmpcWorkspace.QDy[61] + nmpcWorkspace.evGx[286]*nmpcWorkspace.QDy[62] + nmpcWorkspace.evGx[291]*nmpcWorkspace.QDy[63] + nmpcWorkspace.evGx[296]*nmpcWorkspace.QDy[64] + nmpcWorkspace.evGx[301]*nmpcWorkspace.QDy[65] + nmpcWorkspace.evGx[306]*nmpcWorkspace.QDy[66] + nmpcWorkspace.evGx[311]*nmpcWorkspace.QDy[67] + nmpcWorkspace.evGx[316]*nmpcWorkspace.QDy[68] + nmpcWorkspace.evGx[321]*nmpcWorkspace.QDy[69] + nmpcWorkspace.evGx[326]*nmpcWorkspace.QDy[70] + nmpcWorkspace.evGx[331]*nmpcWorkspace.QDy[71] + nmpcWorkspace.evGx[336]*nmpcWorkspace.QDy[72] + nmpcWorkspace.evGx[341]*nmpcWorkspace.QDy[73] + nmpcWorkspace.evGx[346]*nmpcWorkspace.QDy[74] + nmpcWorkspace.evGx[351]*nmpcWorkspace.QDy[75] + nmpcWorkspace.evGx[356]*nmpcWorkspace.QDy[76] + nmpcWorkspace.evGx[361]*nmpcWorkspace.QDy[77] + nmpcWorkspace.evGx[366]*nmpcWorkspace.QDy[78] + nmpcWorkspace.evGx[371]*nmpcWorkspace.QDy[79] + nmpcWorkspace.evGx[376]*nmpcWorkspace.QDy[80] + nmpcWorkspace.evGx[381]*nmpcWorkspace.QDy[81] + nmpcWorkspace.evGx[386]*nmpcWorkspace.QDy[82] + nmpcWorkspace.evGx[391]*nmpcWorkspace.QDy[83] + nmpcWorkspace.evGx[396]*nmpcWorkspace.QDy[84] + nmpcWorkspace.evGx[401]*nmpcWorkspace.QDy[85] + nmpcWorkspace.evGx[406]*nmpcWorkspace.QDy[86] + nmpcWorkspace.evGx[411]*nmpcWorkspace.QDy[87] + nmpcWorkspace.evGx[416]*nmpcWorkspace.QDy[88] + nmpcWorkspace.evGx[421]*nmpcWorkspace.QDy[89] + nmpcWorkspace.evGx[426]*nmpcWorkspace.QDy[90] + nmpcWorkspace.evGx[431]*nmpcWorkspace.QDy[91] + nmpcWorkspace.evGx[436]*nmpcWorkspace.QDy[92] + nmpcWorkspace.evGx[441]*nmpcWorkspace.QDy[93] + nmpcWorkspace.evGx[446]*nmpcWorkspace.QDy[94] + nmpcWorkspace.evGx[451]*nmpcWorkspace.QDy[95] + nmpcWorkspace.evGx[456]*nmpcWorkspace.QDy[96] + nmpcWorkspace.evGx[461]*nmpcWorkspace.QDy[97] + nmpcWorkspace.evGx[466]*nmpcWorkspace.QDy[98] + nmpcWorkspace.evGx[471]*nmpcWorkspace.QDy[99] + nmpcWorkspace.evGx[476]*nmpcWorkspace.QDy[100] + nmpcWorkspace.evGx[481]*nmpcWorkspace.QDy[101] + nmpcWorkspace.evGx[486]*nmpcWorkspace.QDy[102] + nmpcWorkspace.evGx[491]*nmpcWorkspace.QDy[103] + nmpcWorkspace.evGx[496]*nmpcWorkspace.QDy[104] + nmpcWorkspace.evGx[501]*nmpcWorkspace.QDy[105] + nmpcWorkspace.evGx[506]*nmpcWorkspace.QDy[106] + nmpcWorkspace.evGx[511]*nmpcWorkspace.QDy[107] + nmpcWorkspace.evGx[516]*nmpcWorkspace.QDy[108] + nmpcWorkspace.evGx[521]*nmpcWorkspace.QDy[109] + nmpcWorkspace.evGx[526]*nmpcWorkspace.QDy[110] + nmpcWorkspace.evGx[531]*nmpcWorkspace.QDy[111] + nmpcWorkspace.evGx[536]*nmpcWorkspace.QDy[112] + nmpcWorkspace.evGx[541]*nmpcWorkspace.QDy[113] + nmpcWorkspace.evGx[546]*nmpcWorkspace.QDy[114] + nmpcWorkspace.evGx[551]*nmpcWorkspace.QDy[115] + nmpcWorkspace.evGx[556]*nmpcWorkspace.QDy[116] + nmpcWorkspace.evGx[561]*nmpcWorkspace.QDy[117] + nmpcWorkspace.evGx[566]*nmpcWorkspace.QDy[118] + nmpcWorkspace.evGx[571]*nmpcWorkspace.QDy[119] + nmpcWorkspace.evGx[576]*nmpcWorkspace.QDy[120] + nmpcWorkspace.evGx[581]*nmpcWorkspace.QDy[121] + nmpcWorkspace.evGx[586]*nmpcWorkspace.QDy[122] + nmpcWorkspace.evGx[591]*nmpcWorkspace.QDy[123] + nmpcWorkspace.evGx[596]*nmpcWorkspace.QDy[124] + nmpcWorkspace.evGx[601]*nmpcWorkspace.QDy[125] + nmpcWorkspace.evGx[606]*nmpcWorkspace.QDy[126] + nmpcWorkspace.evGx[611]*nmpcWorkspace.QDy[127] + nmpcWorkspace.evGx[616]*nmpcWorkspace.QDy[128] + nmpcWorkspace.evGx[621]*nmpcWorkspace.QDy[129] + nmpcWorkspace.evGx[626]*nmpcWorkspace.QDy[130] + nmpcWorkspace.evGx[631]*nmpcWorkspace.QDy[131] + nmpcWorkspace.evGx[636]*nmpcWorkspace.QDy[132] + nmpcWorkspace.evGx[641]*nmpcWorkspace.QDy[133] + nmpcWorkspace.evGx[646]*nmpcWorkspace.QDy[134] + nmpcWorkspace.evGx[651]*nmpcWorkspace.QDy[135] + nmpcWorkspace.evGx[656]*nmpcWorkspace.QDy[136] + nmpcWorkspace.evGx[661]*nmpcWorkspace.QDy[137] + nmpcWorkspace.evGx[666]*nmpcWorkspace.QDy[138] + nmpcWorkspace.evGx[671]*nmpcWorkspace.QDy[139] + nmpcWorkspace.evGx[676]*nmpcWorkspace.QDy[140] + nmpcWorkspace.evGx[681]*nmpcWorkspace.QDy[141] + nmpcWorkspace.evGx[686]*nmpcWorkspace.QDy[142] + nmpcWorkspace.evGx[691]*nmpcWorkspace.QDy[143] + nmpcWorkspace.evGx[696]*nmpcWorkspace.QDy[144] + nmpcWorkspace.evGx[701]*nmpcWorkspace.QDy[145] + nmpcWorkspace.evGx[706]*nmpcWorkspace.QDy[146] + nmpcWorkspace.evGx[711]*nmpcWorkspace.QDy[147] + nmpcWorkspace.evGx[716]*nmpcWorkspace.QDy[148] + nmpcWorkspace.evGx[721]*nmpcWorkspace.QDy[149] + nmpcWorkspace.evGx[726]*nmpcWorkspace.QDy[150] + nmpcWorkspace.evGx[731]*nmpcWorkspace.QDy[151] + nmpcWorkspace.evGx[736]*nmpcWorkspace.QDy[152] + nmpcWorkspace.evGx[741]*nmpcWorkspace.QDy[153] + nmpcWorkspace.evGx[746]*nmpcWorkspace.QDy[154];
nmpcWorkspace.g[2] = + nmpcWorkspace.evGx[2]*nmpcWorkspace.QDy[5] + nmpcWorkspace.evGx[7]*nmpcWorkspace.QDy[6] + nmpcWorkspace.evGx[12]*nmpcWorkspace.QDy[7] + nmpcWorkspace.evGx[17]*nmpcWorkspace.QDy[8] + nmpcWorkspace.evGx[22]*nmpcWorkspace.QDy[9] + nmpcWorkspace.evGx[27]*nmpcWorkspace.QDy[10] + nmpcWorkspace.evGx[32]*nmpcWorkspace.QDy[11] + nmpcWorkspace.evGx[37]*nmpcWorkspace.QDy[12] + nmpcWorkspace.evGx[42]*nmpcWorkspace.QDy[13] + nmpcWorkspace.evGx[47]*nmpcWorkspace.QDy[14] + nmpcWorkspace.evGx[52]*nmpcWorkspace.QDy[15] + nmpcWorkspace.evGx[57]*nmpcWorkspace.QDy[16] + nmpcWorkspace.evGx[62]*nmpcWorkspace.QDy[17] + nmpcWorkspace.evGx[67]*nmpcWorkspace.QDy[18] + nmpcWorkspace.evGx[72]*nmpcWorkspace.QDy[19] + nmpcWorkspace.evGx[77]*nmpcWorkspace.QDy[20] + nmpcWorkspace.evGx[82]*nmpcWorkspace.QDy[21] + nmpcWorkspace.evGx[87]*nmpcWorkspace.QDy[22] + nmpcWorkspace.evGx[92]*nmpcWorkspace.QDy[23] + nmpcWorkspace.evGx[97]*nmpcWorkspace.QDy[24] + nmpcWorkspace.evGx[102]*nmpcWorkspace.QDy[25] + nmpcWorkspace.evGx[107]*nmpcWorkspace.QDy[26] + nmpcWorkspace.evGx[112]*nmpcWorkspace.QDy[27] + nmpcWorkspace.evGx[117]*nmpcWorkspace.QDy[28] + nmpcWorkspace.evGx[122]*nmpcWorkspace.QDy[29] + nmpcWorkspace.evGx[127]*nmpcWorkspace.QDy[30] + nmpcWorkspace.evGx[132]*nmpcWorkspace.QDy[31] + nmpcWorkspace.evGx[137]*nmpcWorkspace.QDy[32] + nmpcWorkspace.evGx[142]*nmpcWorkspace.QDy[33] + nmpcWorkspace.evGx[147]*nmpcWorkspace.QDy[34] + nmpcWorkspace.evGx[152]*nmpcWorkspace.QDy[35] + nmpcWorkspace.evGx[157]*nmpcWorkspace.QDy[36] + nmpcWorkspace.evGx[162]*nmpcWorkspace.QDy[37] + nmpcWorkspace.evGx[167]*nmpcWorkspace.QDy[38] + nmpcWorkspace.evGx[172]*nmpcWorkspace.QDy[39] + nmpcWorkspace.evGx[177]*nmpcWorkspace.QDy[40] + nmpcWorkspace.evGx[182]*nmpcWorkspace.QDy[41] + nmpcWorkspace.evGx[187]*nmpcWorkspace.QDy[42] + nmpcWorkspace.evGx[192]*nmpcWorkspace.QDy[43] + nmpcWorkspace.evGx[197]*nmpcWorkspace.QDy[44] + nmpcWorkspace.evGx[202]*nmpcWorkspace.QDy[45] + nmpcWorkspace.evGx[207]*nmpcWorkspace.QDy[46] + nmpcWorkspace.evGx[212]*nmpcWorkspace.QDy[47] + nmpcWorkspace.evGx[217]*nmpcWorkspace.QDy[48] + nmpcWorkspace.evGx[222]*nmpcWorkspace.QDy[49] + nmpcWorkspace.evGx[227]*nmpcWorkspace.QDy[50] + nmpcWorkspace.evGx[232]*nmpcWorkspace.QDy[51] + nmpcWorkspace.evGx[237]*nmpcWorkspace.QDy[52] + nmpcWorkspace.evGx[242]*nmpcWorkspace.QDy[53] + nmpcWorkspace.evGx[247]*nmpcWorkspace.QDy[54] + nmpcWorkspace.evGx[252]*nmpcWorkspace.QDy[55] + nmpcWorkspace.evGx[257]*nmpcWorkspace.QDy[56] + nmpcWorkspace.evGx[262]*nmpcWorkspace.QDy[57] + nmpcWorkspace.evGx[267]*nmpcWorkspace.QDy[58] + nmpcWorkspace.evGx[272]*nmpcWorkspace.QDy[59] + nmpcWorkspace.evGx[277]*nmpcWorkspace.QDy[60] + nmpcWorkspace.evGx[282]*nmpcWorkspace.QDy[61] + nmpcWorkspace.evGx[287]*nmpcWorkspace.QDy[62] + nmpcWorkspace.evGx[292]*nmpcWorkspace.QDy[63] + nmpcWorkspace.evGx[297]*nmpcWorkspace.QDy[64] + nmpcWorkspace.evGx[302]*nmpcWorkspace.QDy[65] + nmpcWorkspace.evGx[307]*nmpcWorkspace.QDy[66] + nmpcWorkspace.evGx[312]*nmpcWorkspace.QDy[67] + nmpcWorkspace.evGx[317]*nmpcWorkspace.QDy[68] + nmpcWorkspace.evGx[322]*nmpcWorkspace.QDy[69] + nmpcWorkspace.evGx[327]*nmpcWorkspace.QDy[70] + nmpcWorkspace.evGx[332]*nmpcWorkspace.QDy[71] + nmpcWorkspace.evGx[337]*nmpcWorkspace.QDy[72] + nmpcWorkspace.evGx[342]*nmpcWorkspace.QDy[73] + nmpcWorkspace.evGx[347]*nmpcWorkspace.QDy[74] + nmpcWorkspace.evGx[352]*nmpcWorkspace.QDy[75] + nmpcWorkspace.evGx[357]*nmpcWorkspace.QDy[76] + nmpcWorkspace.evGx[362]*nmpcWorkspace.QDy[77] + nmpcWorkspace.evGx[367]*nmpcWorkspace.QDy[78] + nmpcWorkspace.evGx[372]*nmpcWorkspace.QDy[79] + nmpcWorkspace.evGx[377]*nmpcWorkspace.QDy[80] + nmpcWorkspace.evGx[382]*nmpcWorkspace.QDy[81] + nmpcWorkspace.evGx[387]*nmpcWorkspace.QDy[82] + nmpcWorkspace.evGx[392]*nmpcWorkspace.QDy[83] + nmpcWorkspace.evGx[397]*nmpcWorkspace.QDy[84] + nmpcWorkspace.evGx[402]*nmpcWorkspace.QDy[85] + nmpcWorkspace.evGx[407]*nmpcWorkspace.QDy[86] + nmpcWorkspace.evGx[412]*nmpcWorkspace.QDy[87] + nmpcWorkspace.evGx[417]*nmpcWorkspace.QDy[88] + nmpcWorkspace.evGx[422]*nmpcWorkspace.QDy[89] + nmpcWorkspace.evGx[427]*nmpcWorkspace.QDy[90] + nmpcWorkspace.evGx[432]*nmpcWorkspace.QDy[91] + nmpcWorkspace.evGx[437]*nmpcWorkspace.QDy[92] + nmpcWorkspace.evGx[442]*nmpcWorkspace.QDy[93] + nmpcWorkspace.evGx[447]*nmpcWorkspace.QDy[94] + nmpcWorkspace.evGx[452]*nmpcWorkspace.QDy[95] + nmpcWorkspace.evGx[457]*nmpcWorkspace.QDy[96] + nmpcWorkspace.evGx[462]*nmpcWorkspace.QDy[97] + nmpcWorkspace.evGx[467]*nmpcWorkspace.QDy[98] + nmpcWorkspace.evGx[472]*nmpcWorkspace.QDy[99] + nmpcWorkspace.evGx[477]*nmpcWorkspace.QDy[100] + nmpcWorkspace.evGx[482]*nmpcWorkspace.QDy[101] + nmpcWorkspace.evGx[487]*nmpcWorkspace.QDy[102] + nmpcWorkspace.evGx[492]*nmpcWorkspace.QDy[103] + nmpcWorkspace.evGx[497]*nmpcWorkspace.QDy[104] + nmpcWorkspace.evGx[502]*nmpcWorkspace.QDy[105] + nmpcWorkspace.evGx[507]*nmpcWorkspace.QDy[106] + nmpcWorkspace.evGx[512]*nmpcWorkspace.QDy[107] + nmpcWorkspace.evGx[517]*nmpcWorkspace.QDy[108] + nmpcWorkspace.evGx[522]*nmpcWorkspace.QDy[109] + nmpcWorkspace.evGx[527]*nmpcWorkspace.QDy[110] + nmpcWorkspace.evGx[532]*nmpcWorkspace.QDy[111] + nmpcWorkspace.evGx[537]*nmpcWorkspace.QDy[112] + nmpcWorkspace.evGx[542]*nmpcWorkspace.QDy[113] + nmpcWorkspace.evGx[547]*nmpcWorkspace.QDy[114] + nmpcWorkspace.evGx[552]*nmpcWorkspace.QDy[115] + nmpcWorkspace.evGx[557]*nmpcWorkspace.QDy[116] + nmpcWorkspace.evGx[562]*nmpcWorkspace.QDy[117] + nmpcWorkspace.evGx[567]*nmpcWorkspace.QDy[118] + nmpcWorkspace.evGx[572]*nmpcWorkspace.QDy[119] + nmpcWorkspace.evGx[577]*nmpcWorkspace.QDy[120] + nmpcWorkspace.evGx[582]*nmpcWorkspace.QDy[121] + nmpcWorkspace.evGx[587]*nmpcWorkspace.QDy[122] + nmpcWorkspace.evGx[592]*nmpcWorkspace.QDy[123] + nmpcWorkspace.evGx[597]*nmpcWorkspace.QDy[124] + nmpcWorkspace.evGx[602]*nmpcWorkspace.QDy[125] + nmpcWorkspace.evGx[607]*nmpcWorkspace.QDy[126] + nmpcWorkspace.evGx[612]*nmpcWorkspace.QDy[127] + nmpcWorkspace.evGx[617]*nmpcWorkspace.QDy[128] + nmpcWorkspace.evGx[622]*nmpcWorkspace.QDy[129] + nmpcWorkspace.evGx[627]*nmpcWorkspace.QDy[130] + nmpcWorkspace.evGx[632]*nmpcWorkspace.QDy[131] + nmpcWorkspace.evGx[637]*nmpcWorkspace.QDy[132] + nmpcWorkspace.evGx[642]*nmpcWorkspace.QDy[133] + nmpcWorkspace.evGx[647]*nmpcWorkspace.QDy[134] + nmpcWorkspace.evGx[652]*nmpcWorkspace.QDy[135] + nmpcWorkspace.evGx[657]*nmpcWorkspace.QDy[136] + nmpcWorkspace.evGx[662]*nmpcWorkspace.QDy[137] + nmpcWorkspace.evGx[667]*nmpcWorkspace.QDy[138] + nmpcWorkspace.evGx[672]*nmpcWorkspace.QDy[139] + nmpcWorkspace.evGx[677]*nmpcWorkspace.QDy[140] + nmpcWorkspace.evGx[682]*nmpcWorkspace.QDy[141] + nmpcWorkspace.evGx[687]*nmpcWorkspace.QDy[142] + nmpcWorkspace.evGx[692]*nmpcWorkspace.QDy[143] + nmpcWorkspace.evGx[697]*nmpcWorkspace.QDy[144] + nmpcWorkspace.evGx[702]*nmpcWorkspace.QDy[145] + nmpcWorkspace.evGx[707]*nmpcWorkspace.QDy[146] + nmpcWorkspace.evGx[712]*nmpcWorkspace.QDy[147] + nmpcWorkspace.evGx[717]*nmpcWorkspace.QDy[148] + nmpcWorkspace.evGx[722]*nmpcWorkspace.QDy[149] + nmpcWorkspace.evGx[727]*nmpcWorkspace.QDy[150] + nmpcWorkspace.evGx[732]*nmpcWorkspace.QDy[151] + nmpcWorkspace.evGx[737]*nmpcWorkspace.QDy[152] + nmpcWorkspace.evGx[742]*nmpcWorkspace.QDy[153] + nmpcWorkspace.evGx[747]*nmpcWorkspace.QDy[154];
nmpcWorkspace.g[3] = + nmpcWorkspace.evGx[3]*nmpcWorkspace.QDy[5] + nmpcWorkspace.evGx[8]*nmpcWorkspace.QDy[6] + nmpcWorkspace.evGx[13]*nmpcWorkspace.QDy[7] + nmpcWorkspace.evGx[18]*nmpcWorkspace.QDy[8] + nmpcWorkspace.evGx[23]*nmpcWorkspace.QDy[9] + nmpcWorkspace.evGx[28]*nmpcWorkspace.QDy[10] + nmpcWorkspace.evGx[33]*nmpcWorkspace.QDy[11] + nmpcWorkspace.evGx[38]*nmpcWorkspace.QDy[12] + nmpcWorkspace.evGx[43]*nmpcWorkspace.QDy[13] + nmpcWorkspace.evGx[48]*nmpcWorkspace.QDy[14] + nmpcWorkspace.evGx[53]*nmpcWorkspace.QDy[15] + nmpcWorkspace.evGx[58]*nmpcWorkspace.QDy[16] + nmpcWorkspace.evGx[63]*nmpcWorkspace.QDy[17] + nmpcWorkspace.evGx[68]*nmpcWorkspace.QDy[18] + nmpcWorkspace.evGx[73]*nmpcWorkspace.QDy[19] + nmpcWorkspace.evGx[78]*nmpcWorkspace.QDy[20] + nmpcWorkspace.evGx[83]*nmpcWorkspace.QDy[21] + nmpcWorkspace.evGx[88]*nmpcWorkspace.QDy[22] + nmpcWorkspace.evGx[93]*nmpcWorkspace.QDy[23] + nmpcWorkspace.evGx[98]*nmpcWorkspace.QDy[24] + nmpcWorkspace.evGx[103]*nmpcWorkspace.QDy[25] + nmpcWorkspace.evGx[108]*nmpcWorkspace.QDy[26] + nmpcWorkspace.evGx[113]*nmpcWorkspace.QDy[27] + nmpcWorkspace.evGx[118]*nmpcWorkspace.QDy[28] + nmpcWorkspace.evGx[123]*nmpcWorkspace.QDy[29] + nmpcWorkspace.evGx[128]*nmpcWorkspace.QDy[30] + nmpcWorkspace.evGx[133]*nmpcWorkspace.QDy[31] + nmpcWorkspace.evGx[138]*nmpcWorkspace.QDy[32] + nmpcWorkspace.evGx[143]*nmpcWorkspace.QDy[33] + nmpcWorkspace.evGx[148]*nmpcWorkspace.QDy[34] + nmpcWorkspace.evGx[153]*nmpcWorkspace.QDy[35] + nmpcWorkspace.evGx[158]*nmpcWorkspace.QDy[36] + nmpcWorkspace.evGx[163]*nmpcWorkspace.QDy[37] + nmpcWorkspace.evGx[168]*nmpcWorkspace.QDy[38] + nmpcWorkspace.evGx[173]*nmpcWorkspace.QDy[39] + nmpcWorkspace.evGx[178]*nmpcWorkspace.QDy[40] + nmpcWorkspace.evGx[183]*nmpcWorkspace.QDy[41] + nmpcWorkspace.evGx[188]*nmpcWorkspace.QDy[42] + nmpcWorkspace.evGx[193]*nmpcWorkspace.QDy[43] + nmpcWorkspace.evGx[198]*nmpcWorkspace.QDy[44] + nmpcWorkspace.evGx[203]*nmpcWorkspace.QDy[45] + nmpcWorkspace.evGx[208]*nmpcWorkspace.QDy[46] + nmpcWorkspace.evGx[213]*nmpcWorkspace.QDy[47] + nmpcWorkspace.evGx[218]*nmpcWorkspace.QDy[48] + nmpcWorkspace.evGx[223]*nmpcWorkspace.QDy[49] + nmpcWorkspace.evGx[228]*nmpcWorkspace.QDy[50] + nmpcWorkspace.evGx[233]*nmpcWorkspace.QDy[51] + nmpcWorkspace.evGx[238]*nmpcWorkspace.QDy[52] + nmpcWorkspace.evGx[243]*nmpcWorkspace.QDy[53] + nmpcWorkspace.evGx[248]*nmpcWorkspace.QDy[54] + nmpcWorkspace.evGx[253]*nmpcWorkspace.QDy[55] + nmpcWorkspace.evGx[258]*nmpcWorkspace.QDy[56] + nmpcWorkspace.evGx[263]*nmpcWorkspace.QDy[57] + nmpcWorkspace.evGx[268]*nmpcWorkspace.QDy[58] + nmpcWorkspace.evGx[273]*nmpcWorkspace.QDy[59] + nmpcWorkspace.evGx[278]*nmpcWorkspace.QDy[60] + nmpcWorkspace.evGx[283]*nmpcWorkspace.QDy[61] + nmpcWorkspace.evGx[288]*nmpcWorkspace.QDy[62] + nmpcWorkspace.evGx[293]*nmpcWorkspace.QDy[63] + nmpcWorkspace.evGx[298]*nmpcWorkspace.QDy[64] + nmpcWorkspace.evGx[303]*nmpcWorkspace.QDy[65] + nmpcWorkspace.evGx[308]*nmpcWorkspace.QDy[66] + nmpcWorkspace.evGx[313]*nmpcWorkspace.QDy[67] + nmpcWorkspace.evGx[318]*nmpcWorkspace.QDy[68] + nmpcWorkspace.evGx[323]*nmpcWorkspace.QDy[69] + nmpcWorkspace.evGx[328]*nmpcWorkspace.QDy[70] + nmpcWorkspace.evGx[333]*nmpcWorkspace.QDy[71] + nmpcWorkspace.evGx[338]*nmpcWorkspace.QDy[72] + nmpcWorkspace.evGx[343]*nmpcWorkspace.QDy[73] + nmpcWorkspace.evGx[348]*nmpcWorkspace.QDy[74] + nmpcWorkspace.evGx[353]*nmpcWorkspace.QDy[75] + nmpcWorkspace.evGx[358]*nmpcWorkspace.QDy[76] + nmpcWorkspace.evGx[363]*nmpcWorkspace.QDy[77] + nmpcWorkspace.evGx[368]*nmpcWorkspace.QDy[78] + nmpcWorkspace.evGx[373]*nmpcWorkspace.QDy[79] + nmpcWorkspace.evGx[378]*nmpcWorkspace.QDy[80] + nmpcWorkspace.evGx[383]*nmpcWorkspace.QDy[81] + nmpcWorkspace.evGx[388]*nmpcWorkspace.QDy[82] + nmpcWorkspace.evGx[393]*nmpcWorkspace.QDy[83] + nmpcWorkspace.evGx[398]*nmpcWorkspace.QDy[84] + nmpcWorkspace.evGx[403]*nmpcWorkspace.QDy[85] + nmpcWorkspace.evGx[408]*nmpcWorkspace.QDy[86] + nmpcWorkspace.evGx[413]*nmpcWorkspace.QDy[87] + nmpcWorkspace.evGx[418]*nmpcWorkspace.QDy[88] + nmpcWorkspace.evGx[423]*nmpcWorkspace.QDy[89] + nmpcWorkspace.evGx[428]*nmpcWorkspace.QDy[90] + nmpcWorkspace.evGx[433]*nmpcWorkspace.QDy[91] + nmpcWorkspace.evGx[438]*nmpcWorkspace.QDy[92] + nmpcWorkspace.evGx[443]*nmpcWorkspace.QDy[93] + nmpcWorkspace.evGx[448]*nmpcWorkspace.QDy[94] + nmpcWorkspace.evGx[453]*nmpcWorkspace.QDy[95] + nmpcWorkspace.evGx[458]*nmpcWorkspace.QDy[96] + nmpcWorkspace.evGx[463]*nmpcWorkspace.QDy[97] + nmpcWorkspace.evGx[468]*nmpcWorkspace.QDy[98] + nmpcWorkspace.evGx[473]*nmpcWorkspace.QDy[99] + nmpcWorkspace.evGx[478]*nmpcWorkspace.QDy[100] + nmpcWorkspace.evGx[483]*nmpcWorkspace.QDy[101] + nmpcWorkspace.evGx[488]*nmpcWorkspace.QDy[102] + nmpcWorkspace.evGx[493]*nmpcWorkspace.QDy[103] + nmpcWorkspace.evGx[498]*nmpcWorkspace.QDy[104] + nmpcWorkspace.evGx[503]*nmpcWorkspace.QDy[105] + nmpcWorkspace.evGx[508]*nmpcWorkspace.QDy[106] + nmpcWorkspace.evGx[513]*nmpcWorkspace.QDy[107] + nmpcWorkspace.evGx[518]*nmpcWorkspace.QDy[108] + nmpcWorkspace.evGx[523]*nmpcWorkspace.QDy[109] + nmpcWorkspace.evGx[528]*nmpcWorkspace.QDy[110] + nmpcWorkspace.evGx[533]*nmpcWorkspace.QDy[111] + nmpcWorkspace.evGx[538]*nmpcWorkspace.QDy[112] + nmpcWorkspace.evGx[543]*nmpcWorkspace.QDy[113] + nmpcWorkspace.evGx[548]*nmpcWorkspace.QDy[114] + nmpcWorkspace.evGx[553]*nmpcWorkspace.QDy[115] + nmpcWorkspace.evGx[558]*nmpcWorkspace.QDy[116] + nmpcWorkspace.evGx[563]*nmpcWorkspace.QDy[117] + nmpcWorkspace.evGx[568]*nmpcWorkspace.QDy[118] + nmpcWorkspace.evGx[573]*nmpcWorkspace.QDy[119] + nmpcWorkspace.evGx[578]*nmpcWorkspace.QDy[120] + nmpcWorkspace.evGx[583]*nmpcWorkspace.QDy[121] + nmpcWorkspace.evGx[588]*nmpcWorkspace.QDy[122] + nmpcWorkspace.evGx[593]*nmpcWorkspace.QDy[123] + nmpcWorkspace.evGx[598]*nmpcWorkspace.QDy[124] + nmpcWorkspace.evGx[603]*nmpcWorkspace.QDy[125] + nmpcWorkspace.evGx[608]*nmpcWorkspace.QDy[126] + nmpcWorkspace.evGx[613]*nmpcWorkspace.QDy[127] + nmpcWorkspace.evGx[618]*nmpcWorkspace.QDy[128] + nmpcWorkspace.evGx[623]*nmpcWorkspace.QDy[129] + nmpcWorkspace.evGx[628]*nmpcWorkspace.QDy[130] + nmpcWorkspace.evGx[633]*nmpcWorkspace.QDy[131] + nmpcWorkspace.evGx[638]*nmpcWorkspace.QDy[132] + nmpcWorkspace.evGx[643]*nmpcWorkspace.QDy[133] + nmpcWorkspace.evGx[648]*nmpcWorkspace.QDy[134] + nmpcWorkspace.evGx[653]*nmpcWorkspace.QDy[135] + nmpcWorkspace.evGx[658]*nmpcWorkspace.QDy[136] + nmpcWorkspace.evGx[663]*nmpcWorkspace.QDy[137] + nmpcWorkspace.evGx[668]*nmpcWorkspace.QDy[138] + nmpcWorkspace.evGx[673]*nmpcWorkspace.QDy[139] + nmpcWorkspace.evGx[678]*nmpcWorkspace.QDy[140] + nmpcWorkspace.evGx[683]*nmpcWorkspace.QDy[141] + nmpcWorkspace.evGx[688]*nmpcWorkspace.QDy[142] + nmpcWorkspace.evGx[693]*nmpcWorkspace.QDy[143] + nmpcWorkspace.evGx[698]*nmpcWorkspace.QDy[144] + nmpcWorkspace.evGx[703]*nmpcWorkspace.QDy[145] + nmpcWorkspace.evGx[708]*nmpcWorkspace.QDy[146] + nmpcWorkspace.evGx[713]*nmpcWorkspace.QDy[147] + nmpcWorkspace.evGx[718]*nmpcWorkspace.QDy[148] + nmpcWorkspace.evGx[723]*nmpcWorkspace.QDy[149] + nmpcWorkspace.evGx[728]*nmpcWorkspace.QDy[150] + nmpcWorkspace.evGx[733]*nmpcWorkspace.QDy[151] + nmpcWorkspace.evGx[738]*nmpcWorkspace.QDy[152] + nmpcWorkspace.evGx[743]*nmpcWorkspace.QDy[153] + nmpcWorkspace.evGx[748]*nmpcWorkspace.QDy[154];
nmpcWorkspace.g[4] = + nmpcWorkspace.evGx[4]*nmpcWorkspace.QDy[5] + nmpcWorkspace.evGx[9]*nmpcWorkspace.QDy[6] + nmpcWorkspace.evGx[14]*nmpcWorkspace.QDy[7] + nmpcWorkspace.evGx[19]*nmpcWorkspace.QDy[8] + nmpcWorkspace.evGx[24]*nmpcWorkspace.QDy[9] + nmpcWorkspace.evGx[29]*nmpcWorkspace.QDy[10] + nmpcWorkspace.evGx[34]*nmpcWorkspace.QDy[11] + nmpcWorkspace.evGx[39]*nmpcWorkspace.QDy[12] + nmpcWorkspace.evGx[44]*nmpcWorkspace.QDy[13] + nmpcWorkspace.evGx[49]*nmpcWorkspace.QDy[14] + nmpcWorkspace.evGx[54]*nmpcWorkspace.QDy[15] + nmpcWorkspace.evGx[59]*nmpcWorkspace.QDy[16] + nmpcWorkspace.evGx[64]*nmpcWorkspace.QDy[17] + nmpcWorkspace.evGx[69]*nmpcWorkspace.QDy[18] + nmpcWorkspace.evGx[74]*nmpcWorkspace.QDy[19] + nmpcWorkspace.evGx[79]*nmpcWorkspace.QDy[20] + nmpcWorkspace.evGx[84]*nmpcWorkspace.QDy[21] + nmpcWorkspace.evGx[89]*nmpcWorkspace.QDy[22] + nmpcWorkspace.evGx[94]*nmpcWorkspace.QDy[23] + nmpcWorkspace.evGx[99]*nmpcWorkspace.QDy[24] + nmpcWorkspace.evGx[104]*nmpcWorkspace.QDy[25] + nmpcWorkspace.evGx[109]*nmpcWorkspace.QDy[26] + nmpcWorkspace.evGx[114]*nmpcWorkspace.QDy[27] + nmpcWorkspace.evGx[119]*nmpcWorkspace.QDy[28] + nmpcWorkspace.evGx[124]*nmpcWorkspace.QDy[29] + nmpcWorkspace.evGx[129]*nmpcWorkspace.QDy[30] + nmpcWorkspace.evGx[134]*nmpcWorkspace.QDy[31] + nmpcWorkspace.evGx[139]*nmpcWorkspace.QDy[32] + nmpcWorkspace.evGx[144]*nmpcWorkspace.QDy[33] + nmpcWorkspace.evGx[149]*nmpcWorkspace.QDy[34] + nmpcWorkspace.evGx[154]*nmpcWorkspace.QDy[35] + nmpcWorkspace.evGx[159]*nmpcWorkspace.QDy[36] + nmpcWorkspace.evGx[164]*nmpcWorkspace.QDy[37] + nmpcWorkspace.evGx[169]*nmpcWorkspace.QDy[38] + nmpcWorkspace.evGx[174]*nmpcWorkspace.QDy[39] + nmpcWorkspace.evGx[179]*nmpcWorkspace.QDy[40] + nmpcWorkspace.evGx[184]*nmpcWorkspace.QDy[41] + nmpcWorkspace.evGx[189]*nmpcWorkspace.QDy[42] + nmpcWorkspace.evGx[194]*nmpcWorkspace.QDy[43] + nmpcWorkspace.evGx[199]*nmpcWorkspace.QDy[44] + nmpcWorkspace.evGx[204]*nmpcWorkspace.QDy[45] + nmpcWorkspace.evGx[209]*nmpcWorkspace.QDy[46] + nmpcWorkspace.evGx[214]*nmpcWorkspace.QDy[47] + nmpcWorkspace.evGx[219]*nmpcWorkspace.QDy[48] + nmpcWorkspace.evGx[224]*nmpcWorkspace.QDy[49] + nmpcWorkspace.evGx[229]*nmpcWorkspace.QDy[50] + nmpcWorkspace.evGx[234]*nmpcWorkspace.QDy[51] + nmpcWorkspace.evGx[239]*nmpcWorkspace.QDy[52] + nmpcWorkspace.evGx[244]*nmpcWorkspace.QDy[53] + nmpcWorkspace.evGx[249]*nmpcWorkspace.QDy[54] + nmpcWorkspace.evGx[254]*nmpcWorkspace.QDy[55] + nmpcWorkspace.evGx[259]*nmpcWorkspace.QDy[56] + nmpcWorkspace.evGx[264]*nmpcWorkspace.QDy[57] + nmpcWorkspace.evGx[269]*nmpcWorkspace.QDy[58] + nmpcWorkspace.evGx[274]*nmpcWorkspace.QDy[59] + nmpcWorkspace.evGx[279]*nmpcWorkspace.QDy[60] + nmpcWorkspace.evGx[284]*nmpcWorkspace.QDy[61] + nmpcWorkspace.evGx[289]*nmpcWorkspace.QDy[62] + nmpcWorkspace.evGx[294]*nmpcWorkspace.QDy[63] + nmpcWorkspace.evGx[299]*nmpcWorkspace.QDy[64] + nmpcWorkspace.evGx[304]*nmpcWorkspace.QDy[65] + nmpcWorkspace.evGx[309]*nmpcWorkspace.QDy[66] + nmpcWorkspace.evGx[314]*nmpcWorkspace.QDy[67] + nmpcWorkspace.evGx[319]*nmpcWorkspace.QDy[68] + nmpcWorkspace.evGx[324]*nmpcWorkspace.QDy[69] + nmpcWorkspace.evGx[329]*nmpcWorkspace.QDy[70] + nmpcWorkspace.evGx[334]*nmpcWorkspace.QDy[71] + nmpcWorkspace.evGx[339]*nmpcWorkspace.QDy[72] + nmpcWorkspace.evGx[344]*nmpcWorkspace.QDy[73] + nmpcWorkspace.evGx[349]*nmpcWorkspace.QDy[74] + nmpcWorkspace.evGx[354]*nmpcWorkspace.QDy[75] + nmpcWorkspace.evGx[359]*nmpcWorkspace.QDy[76] + nmpcWorkspace.evGx[364]*nmpcWorkspace.QDy[77] + nmpcWorkspace.evGx[369]*nmpcWorkspace.QDy[78] + nmpcWorkspace.evGx[374]*nmpcWorkspace.QDy[79] + nmpcWorkspace.evGx[379]*nmpcWorkspace.QDy[80] + nmpcWorkspace.evGx[384]*nmpcWorkspace.QDy[81] + nmpcWorkspace.evGx[389]*nmpcWorkspace.QDy[82] + nmpcWorkspace.evGx[394]*nmpcWorkspace.QDy[83] + nmpcWorkspace.evGx[399]*nmpcWorkspace.QDy[84] + nmpcWorkspace.evGx[404]*nmpcWorkspace.QDy[85] + nmpcWorkspace.evGx[409]*nmpcWorkspace.QDy[86] + nmpcWorkspace.evGx[414]*nmpcWorkspace.QDy[87] + nmpcWorkspace.evGx[419]*nmpcWorkspace.QDy[88] + nmpcWorkspace.evGx[424]*nmpcWorkspace.QDy[89] + nmpcWorkspace.evGx[429]*nmpcWorkspace.QDy[90] + nmpcWorkspace.evGx[434]*nmpcWorkspace.QDy[91] + nmpcWorkspace.evGx[439]*nmpcWorkspace.QDy[92] + nmpcWorkspace.evGx[444]*nmpcWorkspace.QDy[93] + nmpcWorkspace.evGx[449]*nmpcWorkspace.QDy[94] + nmpcWorkspace.evGx[454]*nmpcWorkspace.QDy[95] + nmpcWorkspace.evGx[459]*nmpcWorkspace.QDy[96] + nmpcWorkspace.evGx[464]*nmpcWorkspace.QDy[97] + nmpcWorkspace.evGx[469]*nmpcWorkspace.QDy[98] + nmpcWorkspace.evGx[474]*nmpcWorkspace.QDy[99] + nmpcWorkspace.evGx[479]*nmpcWorkspace.QDy[100] + nmpcWorkspace.evGx[484]*nmpcWorkspace.QDy[101] + nmpcWorkspace.evGx[489]*nmpcWorkspace.QDy[102] + nmpcWorkspace.evGx[494]*nmpcWorkspace.QDy[103] + nmpcWorkspace.evGx[499]*nmpcWorkspace.QDy[104] + nmpcWorkspace.evGx[504]*nmpcWorkspace.QDy[105] + nmpcWorkspace.evGx[509]*nmpcWorkspace.QDy[106] + nmpcWorkspace.evGx[514]*nmpcWorkspace.QDy[107] + nmpcWorkspace.evGx[519]*nmpcWorkspace.QDy[108] + nmpcWorkspace.evGx[524]*nmpcWorkspace.QDy[109] + nmpcWorkspace.evGx[529]*nmpcWorkspace.QDy[110] + nmpcWorkspace.evGx[534]*nmpcWorkspace.QDy[111] + nmpcWorkspace.evGx[539]*nmpcWorkspace.QDy[112] + nmpcWorkspace.evGx[544]*nmpcWorkspace.QDy[113] + nmpcWorkspace.evGx[549]*nmpcWorkspace.QDy[114] + nmpcWorkspace.evGx[554]*nmpcWorkspace.QDy[115] + nmpcWorkspace.evGx[559]*nmpcWorkspace.QDy[116] + nmpcWorkspace.evGx[564]*nmpcWorkspace.QDy[117] + nmpcWorkspace.evGx[569]*nmpcWorkspace.QDy[118] + nmpcWorkspace.evGx[574]*nmpcWorkspace.QDy[119] + nmpcWorkspace.evGx[579]*nmpcWorkspace.QDy[120] + nmpcWorkspace.evGx[584]*nmpcWorkspace.QDy[121] + nmpcWorkspace.evGx[589]*nmpcWorkspace.QDy[122] + nmpcWorkspace.evGx[594]*nmpcWorkspace.QDy[123] + nmpcWorkspace.evGx[599]*nmpcWorkspace.QDy[124] + nmpcWorkspace.evGx[604]*nmpcWorkspace.QDy[125] + nmpcWorkspace.evGx[609]*nmpcWorkspace.QDy[126] + nmpcWorkspace.evGx[614]*nmpcWorkspace.QDy[127] + nmpcWorkspace.evGx[619]*nmpcWorkspace.QDy[128] + nmpcWorkspace.evGx[624]*nmpcWorkspace.QDy[129] + nmpcWorkspace.evGx[629]*nmpcWorkspace.QDy[130] + nmpcWorkspace.evGx[634]*nmpcWorkspace.QDy[131] + nmpcWorkspace.evGx[639]*nmpcWorkspace.QDy[132] + nmpcWorkspace.evGx[644]*nmpcWorkspace.QDy[133] + nmpcWorkspace.evGx[649]*nmpcWorkspace.QDy[134] + nmpcWorkspace.evGx[654]*nmpcWorkspace.QDy[135] + nmpcWorkspace.evGx[659]*nmpcWorkspace.QDy[136] + nmpcWorkspace.evGx[664]*nmpcWorkspace.QDy[137] + nmpcWorkspace.evGx[669]*nmpcWorkspace.QDy[138] + nmpcWorkspace.evGx[674]*nmpcWorkspace.QDy[139] + nmpcWorkspace.evGx[679]*nmpcWorkspace.QDy[140] + nmpcWorkspace.evGx[684]*nmpcWorkspace.QDy[141] + nmpcWorkspace.evGx[689]*nmpcWorkspace.QDy[142] + nmpcWorkspace.evGx[694]*nmpcWorkspace.QDy[143] + nmpcWorkspace.evGx[699]*nmpcWorkspace.QDy[144] + nmpcWorkspace.evGx[704]*nmpcWorkspace.QDy[145] + nmpcWorkspace.evGx[709]*nmpcWorkspace.QDy[146] + nmpcWorkspace.evGx[714]*nmpcWorkspace.QDy[147] + nmpcWorkspace.evGx[719]*nmpcWorkspace.QDy[148] + nmpcWorkspace.evGx[724]*nmpcWorkspace.QDy[149] + nmpcWorkspace.evGx[729]*nmpcWorkspace.QDy[150] + nmpcWorkspace.evGx[734]*nmpcWorkspace.QDy[151] + nmpcWorkspace.evGx[739]*nmpcWorkspace.QDy[152] + nmpcWorkspace.evGx[744]*nmpcWorkspace.QDy[153] + nmpcWorkspace.evGx[749]*nmpcWorkspace.QDy[154];


for (lRun1 = 0; lRun1 < 30; ++lRun1)
{
for (lRun2 = lRun1; lRun2 < 30; ++lRun2)
{
lRun3 = (((lRun2 + 1) * (lRun2)) / (2)) + (lRun1);
nmpc_multEQDy( &(nmpcWorkspace.E[ lRun3 * 10 ]), &(nmpcWorkspace.QDy[ lRun2 * 5 + 5 ]), &(nmpcWorkspace.g[ lRun1 * 2 + 5 ]) );
}
}

nmpcWorkspace.lb[0] = nmpcWorkspace.Dx0[0];
nmpcWorkspace.lb[1] = nmpcWorkspace.Dx0[1];
nmpcWorkspace.lb[2] = nmpcWorkspace.Dx0[2];
nmpcWorkspace.lb[3] = nmpcWorkspace.Dx0[3];
nmpcWorkspace.lb[4] = nmpcWorkspace.Dx0[4];
nmpcWorkspace.ub[0] = nmpcWorkspace.Dx0[0];
nmpcWorkspace.ub[1] = nmpcWorkspace.Dx0[1];
nmpcWorkspace.ub[2] = nmpcWorkspace.Dx0[2];
nmpcWorkspace.ub[3] = nmpcWorkspace.Dx0[3];
nmpcWorkspace.ub[4] = nmpcWorkspace.Dx0[4];
tmp = nmpcVariables.x[8] + nmpcWorkspace.d[3];
nmpcWorkspace.lbA[0] = (real_t)-5.9999999999999998e-01 - tmp;
nmpcWorkspace.ubA[0] = (real_t)5.9999999999999998e-01 - tmp;
tmp = nmpcVariables.x[9] + nmpcWorkspace.d[4];
nmpcWorkspace.lbA[1] = (real_t)-5.9999999999999998e-01 - tmp;
nmpcWorkspace.ubA[1] = (real_t)5.9999999999999998e-01 - tmp;
tmp = nmpcVariables.x[13] + nmpcWorkspace.d[8];
nmpcWorkspace.lbA[2] = (real_t)-5.9999999999999998e-01 - tmp;
nmpcWorkspace.ubA[2] = (real_t)5.9999999999999998e-01 - tmp;
tmp = nmpcVariables.x[14] + nmpcWorkspace.d[9];
nmpcWorkspace.lbA[3] = (real_t)-5.9999999999999998e-01 - tmp;
nmpcWorkspace.ubA[3] = (real_t)5.9999999999999998e-01 - tmp;
tmp = nmpcVariables.x[18] + nmpcWorkspace.d[13];
nmpcWorkspace.lbA[4] = (real_t)-5.9999999999999998e-01 - tmp;
nmpcWorkspace.ubA[4] = (real_t)5.9999999999999998e-01 - tmp;
tmp = nmpcVariables.x[19] + nmpcWorkspace.d[14];
nmpcWorkspace.lbA[5] = (real_t)-5.9999999999999998e-01 - tmp;
nmpcWorkspace.ubA[5] = (real_t)5.9999999999999998e-01 - tmp;
tmp = nmpcVariables.x[23] + nmpcWorkspace.d[18];
nmpcWorkspace.lbA[6] = (real_t)-5.9999999999999998e-01 - tmp;
nmpcWorkspace.ubA[6] = (real_t)5.9999999999999998e-01 - tmp;
tmp = nmpcVariables.x[24] + nmpcWorkspace.d[19];
nmpcWorkspace.lbA[7] = (real_t)-5.9999999999999998e-01 - tmp;
nmpcWorkspace.ubA[7] = (real_t)5.9999999999999998e-01 - tmp;
tmp = nmpcVariables.x[28] + nmpcWorkspace.d[23];
nmpcWorkspace.lbA[8] = (real_t)-5.9999999999999998e-01 - tmp;
nmpcWorkspace.ubA[8] = (real_t)5.9999999999999998e-01 - tmp;
tmp = nmpcVariables.x[29] + nmpcWorkspace.d[24];
nmpcWorkspace.lbA[9] = (real_t)-5.9999999999999998e-01 - tmp;
nmpcWorkspace.ubA[9] = (real_t)5.9999999999999998e-01 - tmp;
tmp = nmpcVariables.x[33] + nmpcWorkspace.d[28];
nmpcWorkspace.lbA[10] = (real_t)-5.9999999999999998e-01 - tmp;
nmpcWorkspace.ubA[10] = (real_t)5.9999999999999998e-01 - tmp;
tmp = nmpcVariables.x[34] + nmpcWorkspace.d[29];
nmpcWorkspace.lbA[11] = (real_t)-5.9999999999999998e-01 - tmp;
nmpcWorkspace.ubA[11] = (real_t)5.9999999999999998e-01 - tmp;
tmp = nmpcVariables.x[38] + nmpcWorkspace.d[33];
nmpcWorkspace.lbA[12] = (real_t)-5.9999999999999998e-01 - tmp;
nmpcWorkspace.ubA[12] = (real_t)5.9999999999999998e-01 - tmp;
tmp = nmpcVariables.x[39] + nmpcWorkspace.d[34];
nmpcWorkspace.lbA[13] = (real_t)-5.9999999999999998e-01 - tmp;
nmpcWorkspace.ubA[13] = (real_t)5.9999999999999998e-01 - tmp;
tmp = nmpcVariables.x[43] + nmpcWorkspace.d[38];
nmpcWorkspace.lbA[14] = (real_t)-5.9999999999999998e-01 - tmp;
nmpcWorkspace.ubA[14] = (real_t)5.9999999999999998e-01 - tmp;
tmp = nmpcVariables.x[44] + nmpcWorkspace.d[39];
nmpcWorkspace.lbA[15] = (real_t)-5.9999999999999998e-01 - tmp;
nmpcWorkspace.ubA[15] = (real_t)5.9999999999999998e-01 - tmp;
tmp = nmpcVariables.x[48] + nmpcWorkspace.d[43];
nmpcWorkspace.lbA[16] = (real_t)-5.9999999999999998e-01 - tmp;
nmpcWorkspace.ubA[16] = (real_t)5.9999999999999998e-01 - tmp;
tmp = nmpcVariables.x[49] + nmpcWorkspace.d[44];
nmpcWorkspace.lbA[17] = (real_t)-5.9999999999999998e-01 - tmp;
nmpcWorkspace.ubA[17] = (real_t)5.9999999999999998e-01 - tmp;
tmp = nmpcVariables.x[53] + nmpcWorkspace.d[48];
nmpcWorkspace.lbA[18] = (real_t)-5.9999999999999998e-01 - tmp;
nmpcWorkspace.ubA[18] = (real_t)5.9999999999999998e-01 - tmp;
tmp = nmpcVariables.x[54] + nmpcWorkspace.d[49];
nmpcWorkspace.lbA[19] = (real_t)-5.9999999999999998e-01 - tmp;
nmpcWorkspace.ubA[19] = (real_t)5.9999999999999998e-01 - tmp;
tmp = nmpcVariables.x[58] + nmpcWorkspace.d[53];
nmpcWorkspace.lbA[20] = (real_t)-5.9999999999999998e-01 - tmp;
nmpcWorkspace.ubA[20] = (real_t)5.9999999999999998e-01 - tmp;
tmp = nmpcVariables.x[59] + nmpcWorkspace.d[54];
nmpcWorkspace.lbA[21] = (real_t)-5.9999999999999998e-01 - tmp;
nmpcWorkspace.ubA[21] = (real_t)5.9999999999999998e-01 - tmp;
tmp = nmpcVariables.x[63] + nmpcWorkspace.d[58];
nmpcWorkspace.lbA[22] = (real_t)-5.9999999999999998e-01 - tmp;
nmpcWorkspace.ubA[22] = (real_t)5.9999999999999998e-01 - tmp;
tmp = nmpcVariables.x[64] + nmpcWorkspace.d[59];
nmpcWorkspace.lbA[23] = (real_t)-5.9999999999999998e-01 - tmp;
nmpcWorkspace.ubA[23] = (real_t)5.9999999999999998e-01 - tmp;
tmp = nmpcVariables.x[68] + nmpcWorkspace.d[63];
nmpcWorkspace.lbA[24] = (real_t)-5.9999999999999998e-01 - tmp;
nmpcWorkspace.ubA[24] = (real_t)5.9999999999999998e-01 - tmp;
tmp = nmpcVariables.x[69] + nmpcWorkspace.d[64];
nmpcWorkspace.lbA[25] = (real_t)-5.9999999999999998e-01 - tmp;
nmpcWorkspace.ubA[25] = (real_t)5.9999999999999998e-01 - tmp;
tmp = nmpcVariables.x[73] + nmpcWorkspace.d[68];
nmpcWorkspace.lbA[26] = (real_t)-5.9999999999999998e-01 - tmp;
nmpcWorkspace.ubA[26] = (real_t)5.9999999999999998e-01 - tmp;
tmp = nmpcVariables.x[74] + nmpcWorkspace.d[69];
nmpcWorkspace.lbA[27] = (real_t)-5.9999999999999998e-01 - tmp;
nmpcWorkspace.ubA[27] = (real_t)5.9999999999999998e-01 - tmp;
tmp = nmpcVariables.x[78] + nmpcWorkspace.d[73];
nmpcWorkspace.lbA[28] = (real_t)-5.9999999999999998e-01 - tmp;
nmpcWorkspace.ubA[28] = (real_t)5.9999999999999998e-01 - tmp;
tmp = nmpcVariables.x[79] + nmpcWorkspace.d[74];
nmpcWorkspace.lbA[29] = (real_t)-5.9999999999999998e-01 - tmp;
nmpcWorkspace.ubA[29] = (real_t)5.9999999999999998e-01 - tmp;
tmp = nmpcVariables.x[83] + nmpcWorkspace.d[78];
nmpcWorkspace.lbA[30] = (real_t)-5.9999999999999998e-01 - tmp;
nmpcWorkspace.ubA[30] = (real_t)5.9999999999999998e-01 - tmp;
tmp = nmpcVariables.x[84] + nmpcWorkspace.d[79];
nmpcWorkspace.lbA[31] = (real_t)-5.9999999999999998e-01 - tmp;
nmpcWorkspace.ubA[31] = (real_t)5.9999999999999998e-01 - tmp;
tmp = nmpcVariables.x[88] + nmpcWorkspace.d[83];
nmpcWorkspace.lbA[32] = (real_t)-5.9999999999999998e-01 - tmp;
nmpcWorkspace.ubA[32] = (real_t)5.9999999999999998e-01 - tmp;
tmp = nmpcVariables.x[89] + nmpcWorkspace.d[84];
nmpcWorkspace.lbA[33] = (real_t)-5.9999999999999998e-01 - tmp;
nmpcWorkspace.ubA[33] = (real_t)5.9999999999999998e-01 - tmp;
tmp = nmpcVariables.x[93] + nmpcWorkspace.d[88];
nmpcWorkspace.lbA[34] = (real_t)-5.9999999999999998e-01 - tmp;
nmpcWorkspace.ubA[34] = (real_t)5.9999999999999998e-01 - tmp;
tmp = nmpcVariables.x[94] + nmpcWorkspace.d[89];
nmpcWorkspace.lbA[35] = (real_t)-5.9999999999999998e-01 - tmp;
nmpcWorkspace.ubA[35] = (real_t)5.9999999999999998e-01 - tmp;
tmp = nmpcVariables.x[98] + nmpcWorkspace.d[93];
nmpcWorkspace.lbA[36] = (real_t)-5.9999999999999998e-01 - tmp;
nmpcWorkspace.ubA[36] = (real_t)5.9999999999999998e-01 - tmp;
tmp = nmpcVariables.x[99] + nmpcWorkspace.d[94];
nmpcWorkspace.lbA[37] = (real_t)-5.9999999999999998e-01 - tmp;
nmpcWorkspace.ubA[37] = (real_t)5.9999999999999998e-01 - tmp;
tmp = nmpcVariables.x[103] + nmpcWorkspace.d[98];
nmpcWorkspace.lbA[38] = (real_t)-5.9999999999999998e-01 - tmp;
nmpcWorkspace.ubA[38] = (real_t)5.9999999999999998e-01 - tmp;
tmp = nmpcVariables.x[104] + nmpcWorkspace.d[99];
nmpcWorkspace.lbA[39] = (real_t)-5.9999999999999998e-01 - tmp;
nmpcWorkspace.ubA[39] = (real_t)5.9999999999999998e-01 - tmp;
tmp = nmpcVariables.x[108] + nmpcWorkspace.d[103];
nmpcWorkspace.lbA[40] = (real_t)-5.9999999999999998e-01 - tmp;
nmpcWorkspace.ubA[40] = (real_t)5.9999999999999998e-01 - tmp;
tmp = nmpcVariables.x[109] + nmpcWorkspace.d[104];
nmpcWorkspace.lbA[41] = (real_t)-5.9999999999999998e-01 - tmp;
nmpcWorkspace.ubA[41] = (real_t)5.9999999999999998e-01 - tmp;
tmp = nmpcVariables.x[113] + nmpcWorkspace.d[108];
nmpcWorkspace.lbA[42] = (real_t)-5.9999999999999998e-01 - tmp;
nmpcWorkspace.ubA[42] = (real_t)5.9999999999999998e-01 - tmp;
tmp = nmpcVariables.x[114] + nmpcWorkspace.d[109];
nmpcWorkspace.lbA[43] = (real_t)-5.9999999999999998e-01 - tmp;
nmpcWorkspace.ubA[43] = (real_t)5.9999999999999998e-01 - tmp;
tmp = nmpcVariables.x[118] + nmpcWorkspace.d[113];
nmpcWorkspace.lbA[44] = (real_t)-5.9999999999999998e-01 - tmp;
nmpcWorkspace.ubA[44] = (real_t)5.9999999999999998e-01 - tmp;
tmp = nmpcVariables.x[119] + nmpcWorkspace.d[114];
nmpcWorkspace.lbA[45] = (real_t)-5.9999999999999998e-01 - tmp;
nmpcWorkspace.ubA[45] = (real_t)5.9999999999999998e-01 - tmp;
tmp = nmpcVariables.x[123] + nmpcWorkspace.d[118];
nmpcWorkspace.lbA[46] = (real_t)-5.9999999999999998e-01 - tmp;
nmpcWorkspace.ubA[46] = (real_t)5.9999999999999998e-01 - tmp;
tmp = nmpcVariables.x[124] + nmpcWorkspace.d[119];
nmpcWorkspace.lbA[47] = (real_t)-5.9999999999999998e-01 - tmp;
nmpcWorkspace.ubA[47] = (real_t)5.9999999999999998e-01 - tmp;
tmp = nmpcVariables.x[128] + nmpcWorkspace.d[123];
nmpcWorkspace.lbA[48] = (real_t)-5.9999999999999998e-01 - tmp;
nmpcWorkspace.ubA[48] = (real_t)5.9999999999999998e-01 - tmp;
tmp = nmpcVariables.x[129] + nmpcWorkspace.d[124];
nmpcWorkspace.lbA[49] = (real_t)-5.9999999999999998e-01 - tmp;
nmpcWorkspace.ubA[49] = (real_t)5.9999999999999998e-01 - tmp;
tmp = nmpcVariables.x[133] + nmpcWorkspace.d[128];
nmpcWorkspace.lbA[50] = (real_t)-5.9999999999999998e-01 - tmp;
nmpcWorkspace.ubA[50] = (real_t)5.9999999999999998e-01 - tmp;
tmp = nmpcVariables.x[134] + nmpcWorkspace.d[129];
nmpcWorkspace.lbA[51] = (real_t)-5.9999999999999998e-01 - tmp;
nmpcWorkspace.ubA[51] = (real_t)5.9999999999999998e-01 - tmp;
tmp = nmpcVariables.x[138] + nmpcWorkspace.d[133];
nmpcWorkspace.lbA[52] = (real_t)-5.9999999999999998e-01 - tmp;
nmpcWorkspace.ubA[52] = (real_t)5.9999999999999998e-01 - tmp;
tmp = nmpcVariables.x[139] + nmpcWorkspace.d[134];
nmpcWorkspace.lbA[53] = (real_t)-5.9999999999999998e-01 - tmp;
nmpcWorkspace.ubA[53] = (real_t)5.9999999999999998e-01 - tmp;
tmp = nmpcVariables.x[143] + nmpcWorkspace.d[138];
nmpcWorkspace.lbA[54] = (real_t)-5.9999999999999998e-01 - tmp;
nmpcWorkspace.ubA[54] = (real_t)5.9999999999999998e-01 - tmp;
tmp = nmpcVariables.x[144] + nmpcWorkspace.d[139];
nmpcWorkspace.lbA[55] = (real_t)-5.9999999999999998e-01 - tmp;
nmpcWorkspace.ubA[55] = (real_t)5.9999999999999998e-01 - tmp;
tmp = nmpcVariables.x[148] + nmpcWorkspace.d[143];
nmpcWorkspace.lbA[56] = (real_t)-5.9999999999999998e-01 - tmp;
nmpcWorkspace.ubA[56] = (real_t)5.9999999999999998e-01 - tmp;
tmp = nmpcVariables.x[149] + nmpcWorkspace.d[144];
nmpcWorkspace.lbA[57] = (real_t)-5.9999999999999998e-01 - tmp;
nmpcWorkspace.ubA[57] = (real_t)5.9999999999999998e-01 - tmp;
tmp = nmpcVariables.x[153] + nmpcWorkspace.d[148];
nmpcWorkspace.lbA[58] = (real_t)-5.9999999999999998e-01 - tmp;
nmpcWorkspace.ubA[58] = (real_t)5.9999999999999998e-01 - tmp;
tmp = nmpcVariables.x[154] + nmpcWorkspace.d[149];
nmpcWorkspace.lbA[59] = (real_t)-5.9999999999999998e-01 - tmp;
nmpcWorkspace.ubA[59] = (real_t)5.9999999999999998e-01 - tmp;

}

void nmpc_expand(  )
{
int lRun1;
int lRun2;
int lRun3;
nmpcVariables.x[0] += nmpcWorkspace.x[0];
nmpcVariables.x[1] += nmpcWorkspace.x[1];
nmpcVariables.x[2] += nmpcWorkspace.x[2];
nmpcVariables.x[3] += nmpcWorkspace.x[3];
nmpcVariables.x[4] += nmpcWorkspace.x[4];

nmpcVariables.u[0] += nmpcWorkspace.x[5];
nmpcVariables.u[1] += nmpcWorkspace.x[6];
nmpcVariables.u[2] += nmpcWorkspace.x[7];
nmpcVariables.u[3] += nmpcWorkspace.x[8];
nmpcVariables.u[4] += nmpcWorkspace.x[9];
nmpcVariables.u[5] += nmpcWorkspace.x[10];
nmpcVariables.u[6] += nmpcWorkspace.x[11];
nmpcVariables.u[7] += nmpcWorkspace.x[12];
nmpcVariables.u[8] += nmpcWorkspace.x[13];
nmpcVariables.u[9] += nmpcWorkspace.x[14];
nmpcVariables.u[10] += nmpcWorkspace.x[15];
nmpcVariables.u[11] += nmpcWorkspace.x[16];
nmpcVariables.u[12] += nmpcWorkspace.x[17];
nmpcVariables.u[13] += nmpcWorkspace.x[18];
nmpcVariables.u[14] += nmpcWorkspace.x[19];
nmpcVariables.u[15] += nmpcWorkspace.x[20];
nmpcVariables.u[16] += nmpcWorkspace.x[21];
nmpcVariables.u[17] += nmpcWorkspace.x[22];
nmpcVariables.u[18] += nmpcWorkspace.x[23];
nmpcVariables.u[19] += nmpcWorkspace.x[24];
nmpcVariables.u[20] += nmpcWorkspace.x[25];
nmpcVariables.u[21] += nmpcWorkspace.x[26];
nmpcVariables.u[22] += nmpcWorkspace.x[27];
nmpcVariables.u[23] += nmpcWorkspace.x[28];
nmpcVariables.u[24] += nmpcWorkspace.x[29];
nmpcVariables.u[25] += nmpcWorkspace.x[30];
nmpcVariables.u[26] += nmpcWorkspace.x[31];
nmpcVariables.u[27] += nmpcWorkspace.x[32];
nmpcVariables.u[28] += nmpcWorkspace.x[33];
nmpcVariables.u[29] += nmpcWorkspace.x[34];
nmpcVariables.u[30] += nmpcWorkspace.x[35];
nmpcVariables.u[31] += nmpcWorkspace.x[36];
nmpcVariables.u[32] += nmpcWorkspace.x[37];
nmpcVariables.u[33] += nmpcWorkspace.x[38];
nmpcVariables.u[34] += nmpcWorkspace.x[39];
nmpcVariables.u[35] += nmpcWorkspace.x[40];
nmpcVariables.u[36] += nmpcWorkspace.x[41];
nmpcVariables.u[37] += nmpcWorkspace.x[42];
nmpcVariables.u[38] += nmpcWorkspace.x[43];
nmpcVariables.u[39] += nmpcWorkspace.x[44];
nmpcVariables.u[40] += nmpcWorkspace.x[45];
nmpcVariables.u[41] += nmpcWorkspace.x[46];
nmpcVariables.u[42] += nmpcWorkspace.x[47];
nmpcVariables.u[43] += nmpcWorkspace.x[48];
nmpcVariables.u[44] += nmpcWorkspace.x[49];
nmpcVariables.u[45] += nmpcWorkspace.x[50];
nmpcVariables.u[46] += nmpcWorkspace.x[51];
nmpcVariables.u[47] += nmpcWorkspace.x[52];
nmpcVariables.u[48] += nmpcWorkspace.x[53];
nmpcVariables.u[49] += nmpcWorkspace.x[54];
nmpcVariables.u[50] += nmpcWorkspace.x[55];
nmpcVariables.u[51] += nmpcWorkspace.x[56];
nmpcVariables.u[52] += nmpcWorkspace.x[57];
nmpcVariables.u[53] += nmpcWorkspace.x[58];
nmpcVariables.u[54] += nmpcWorkspace.x[59];
nmpcVariables.u[55] += nmpcWorkspace.x[60];
nmpcVariables.u[56] += nmpcWorkspace.x[61];
nmpcVariables.u[57] += nmpcWorkspace.x[62];
nmpcVariables.u[58] += nmpcWorkspace.x[63];
nmpcVariables.u[59] += nmpcWorkspace.x[64];

nmpcVariables.x[5] += + nmpcWorkspace.evGx[0]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[1]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[2]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[3]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[4]*nmpcWorkspace.x[4] + nmpcWorkspace.d[0];
nmpcVariables.x[6] += + nmpcWorkspace.evGx[5]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[6]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[7]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[8]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[9]*nmpcWorkspace.x[4] + nmpcWorkspace.d[1];
nmpcVariables.x[7] += + nmpcWorkspace.evGx[10]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[11]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[12]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[13]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[14]*nmpcWorkspace.x[4] + nmpcWorkspace.d[2];
nmpcVariables.x[8] += + nmpcWorkspace.evGx[15]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[16]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[17]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[18]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[19]*nmpcWorkspace.x[4] + nmpcWorkspace.d[3];
nmpcVariables.x[9] += + nmpcWorkspace.evGx[20]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[21]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[22]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[23]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[24]*nmpcWorkspace.x[4] + nmpcWorkspace.d[4];
nmpcVariables.x[10] += + nmpcWorkspace.evGx[25]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[26]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[27]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[28]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[29]*nmpcWorkspace.x[4] + nmpcWorkspace.d[5];
nmpcVariables.x[11] += + nmpcWorkspace.evGx[30]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[31]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[32]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[33]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[34]*nmpcWorkspace.x[4] + nmpcWorkspace.d[6];
nmpcVariables.x[12] += + nmpcWorkspace.evGx[35]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[36]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[37]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[38]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[39]*nmpcWorkspace.x[4] + nmpcWorkspace.d[7];
nmpcVariables.x[13] += + nmpcWorkspace.evGx[40]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[41]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[42]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[43]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[44]*nmpcWorkspace.x[4] + nmpcWorkspace.d[8];
nmpcVariables.x[14] += + nmpcWorkspace.evGx[45]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[46]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[47]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[48]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[49]*nmpcWorkspace.x[4] + nmpcWorkspace.d[9];
nmpcVariables.x[15] += + nmpcWorkspace.evGx[50]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[51]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[52]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[53]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[54]*nmpcWorkspace.x[4] + nmpcWorkspace.d[10];
nmpcVariables.x[16] += + nmpcWorkspace.evGx[55]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[56]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[57]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[58]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[59]*nmpcWorkspace.x[4] + nmpcWorkspace.d[11];
nmpcVariables.x[17] += + nmpcWorkspace.evGx[60]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[61]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[62]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[63]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[64]*nmpcWorkspace.x[4] + nmpcWorkspace.d[12];
nmpcVariables.x[18] += + nmpcWorkspace.evGx[65]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[66]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[67]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[68]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[69]*nmpcWorkspace.x[4] + nmpcWorkspace.d[13];
nmpcVariables.x[19] += + nmpcWorkspace.evGx[70]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[71]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[72]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[73]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[74]*nmpcWorkspace.x[4] + nmpcWorkspace.d[14];
nmpcVariables.x[20] += + nmpcWorkspace.evGx[75]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[76]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[77]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[78]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[79]*nmpcWorkspace.x[4] + nmpcWorkspace.d[15];
nmpcVariables.x[21] += + nmpcWorkspace.evGx[80]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[81]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[82]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[83]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[84]*nmpcWorkspace.x[4] + nmpcWorkspace.d[16];
nmpcVariables.x[22] += + nmpcWorkspace.evGx[85]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[86]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[87]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[88]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[89]*nmpcWorkspace.x[4] + nmpcWorkspace.d[17];
nmpcVariables.x[23] += + nmpcWorkspace.evGx[90]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[91]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[92]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[93]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[94]*nmpcWorkspace.x[4] + nmpcWorkspace.d[18];
nmpcVariables.x[24] += + nmpcWorkspace.evGx[95]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[96]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[97]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[98]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[99]*nmpcWorkspace.x[4] + nmpcWorkspace.d[19];
nmpcVariables.x[25] += + nmpcWorkspace.evGx[100]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[101]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[102]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[103]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[104]*nmpcWorkspace.x[4] + nmpcWorkspace.d[20];
nmpcVariables.x[26] += + nmpcWorkspace.evGx[105]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[106]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[107]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[108]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[109]*nmpcWorkspace.x[4] + nmpcWorkspace.d[21];
nmpcVariables.x[27] += + nmpcWorkspace.evGx[110]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[111]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[112]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[113]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[114]*nmpcWorkspace.x[4] + nmpcWorkspace.d[22];
nmpcVariables.x[28] += + nmpcWorkspace.evGx[115]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[116]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[117]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[118]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[119]*nmpcWorkspace.x[4] + nmpcWorkspace.d[23];
nmpcVariables.x[29] += + nmpcWorkspace.evGx[120]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[121]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[122]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[123]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[124]*nmpcWorkspace.x[4] + nmpcWorkspace.d[24];
nmpcVariables.x[30] += + nmpcWorkspace.evGx[125]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[126]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[127]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[128]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[129]*nmpcWorkspace.x[4] + nmpcWorkspace.d[25];
nmpcVariables.x[31] += + nmpcWorkspace.evGx[130]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[131]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[132]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[133]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[134]*nmpcWorkspace.x[4] + nmpcWorkspace.d[26];
nmpcVariables.x[32] += + nmpcWorkspace.evGx[135]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[136]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[137]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[138]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[139]*nmpcWorkspace.x[4] + nmpcWorkspace.d[27];
nmpcVariables.x[33] += + nmpcWorkspace.evGx[140]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[141]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[142]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[143]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[144]*nmpcWorkspace.x[4] + nmpcWorkspace.d[28];
nmpcVariables.x[34] += + nmpcWorkspace.evGx[145]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[146]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[147]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[148]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[149]*nmpcWorkspace.x[4] + nmpcWorkspace.d[29];
nmpcVariables.x[35] += + nmpcWorkspace.evGx[150]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[151]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[152]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[153]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[154]*nmpcWorkspace.x[4] + nmpcWorkspace.d[30];
nmpcVariables.x[36] += + nmpcWorkspace.evGx[155]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[156]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[157]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[158]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[159]*nmpcWorkspace.x[4] + nmpcWorkspace.d[31];
nmpcVariables.x[37] += + nmpcWorkspace.evGx[160]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[161]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[162]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[163]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[164]*nmpcWorkspace.x[4] + nmpcWorkspace.d[32];
nmpcVariables.x[38] += + nmpcWorkspace.evGx[165]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[166]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[167]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[168]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[169]*nmpcWorkspace.x[4] + nmpcWorkspace.d[33];
nmpcVariables.x[39] += + nmpcWorkspace.evGx[170]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[171]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[172]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[173]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[174]*nmpcWorkspace.x[4] + nmpcWorkspace.d[34];
nmpcVariables.x[40] += + nmpcWorkspace.evGx[175]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[176]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[177]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[178]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[179]*nmpcWorkspace.x[4] + nmpcWorkspace.d[35];
nmpcVariables.x[41] += + nmpcWorkspace.evGx[180]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[181]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[182]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[183]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[184]*nmpcWorkspace.x[4] + nmpcWorkspace.d[36];
nmpcVariables.x[42] += + nmpcWorkspace.evGx[185]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[186]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[187]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[188]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[189]*nmpcWorkspace.x[4] + nmpcWorkspace.d[37];
nmpcVariables.x[43] += + nmpcWorkspace.evGx[190]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[191]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[192]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[193]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[194]*nmpcWorkspace.x[4] + nmpcWorkspace.d[38];
nmpcVariables.x[44] += + nmpcWorkspace.evGx[195]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[196]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[197]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[198]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[199]*nmpcWorkspace.x[4] + nmpcWorkspace.d[39];
nmpcVariables.x[45] += + nmpcWorkspace.evGx[200]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[201]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[202]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[203]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[204]*nmpcWorkspace.x[4] + nmpcWorkspace.d[40];
nmpcVariables.x[46] += + nmpcWorkspace.evGx[205]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[206]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[207]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[208]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[209]*nmpcWorkspace.x[4] + nmpcWorkspace.d[41];
nmpcVariables.x[47] += + nmpcWorkspace.evGx[210]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[211]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[212]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[213]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[214]*nmpcWorkspace.x[4] + nmpcWorkspace.d[42];
nmpcVariables.x[48] += + nmpcWorkspace.evGx[215]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[216]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[217]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[218]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[219]*nmpcWorkspace.x[4] + nmpcWorkspace.d[43];
nmpcVariables.x[49] += + nmpcWorkspace.evGx[220]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[221]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[222]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[223]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[224]*nmpcWorkspace.x[4] + nmpcWorkspace.d[44];
nmpcVariables.x[50] += + nmpcWorkspace.evGx[225]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[226]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[227]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[228]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[229]*nmpcWorkspace.x[4] + nmpcWorkspace.d[45];
nmpcVariables.x[51] += + nmpcWorkspace.evGx[230]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[231]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[232]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[233]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[234]*nmpcWorkspace.x[4] + nmpcWorkspace.d[46];
nmpcVariables.x[52] += + nmpcWorkspace.evGx[235]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[236]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[237]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[238]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[239]*nmpcWorkspace.x[4] + nmpcWorkspace.d[47];
nmpcVariables.x[53] += + nmpcWorkspace.evGx[240]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[241]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[242]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[243]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[244]*nmpcWorkspace.x[4] + nmpcWorkspace.d[48];
nmpcVariables.x[54] += + nmpcWorkspace.evGx[245]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[246]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[247]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[248]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[249]*nmpcWorkspace.x[4] + nmpcWorkspace.d[49];
nmpcVariables.x[55] += + nmpcWorkspace.evGx[250]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[251]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[252]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[253]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[254]*nmpcWorkspace.x[4] + nmpcWorkspace.d[50];
nmpcVariables.x[56] += + nmpcWorkspace.evGx[255]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[256]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[257]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[258]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[259]*nmpcWorkspace.x[4] + nmpcWorkspace.d[51];
nmpcVariables.x[57] += + nmpcWorkspace.evGx[260]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[261]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[262]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[263]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[264]*nmpcWorkspace.x[4] + nmpcWorkspace.d[52];
nmpcVariables.x[58] += + nmpcWorkspace.evGx[265]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[266]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[267]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[268]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[269]*nmpcWorkspace.x[4] + nmpcWorkspace.d[53];
nmpcVariables.x[59] += + nmpcWorkspace.evGx[270]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[271]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[272]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[273]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[274]*nmpcWorkspace.x[4] + nmpcWorkspace.d[54];
nmpcVariables.x[60] += + nmpcWorkspace.evGx[275]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[276]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[277]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[278]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[279]*nmpcWorkspace.x[4] + nmpcWorkspace.d[55];
nmpcVariables.x[61] += + nmpcWorkspace.evGx[280]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[281]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[282]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[283]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[284]*nmpcWorkspace.x[4] + nmpcWorkspace.d[56];
nmpcVariables.x[62] += + nmpcWorkspace.evGx[285]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[286]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[287]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[288]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[289]*nmpcWorkspace.x[4] + nmpcWorkspace.d[57];
nmpcVariables.x[63] += + nmpcWorkspace.evGx[290]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[291]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[292]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[293]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[294]*nmpcWorkspace.x[4] + nmpcWorkspace.d[58];
nmpcVariables.x[64] += + nmpcWorkspace.evGx[295]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[296]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[297]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[298]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[299]*nmpcWorkspace.x[4] + nmpcWorkspace.d[59];
nmpcVariables.x[65] += + nmpcWorkspace.evGx[300]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[301]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[302]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[303]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[304]*nmpcWorkspace.x[4] + nmpcWorkspace.d[60];
nmpcVariables.x[66] += + nmpcWorkspace.evGx[305]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[306]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[307]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[308]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[309]*nmpcWorkspace.x[4] + nmpcWorkspace.d[61];
nmpcVariables.x[67] += + nmpcWorkspace.evGx[310]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[311]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[312]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[313]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[314]*nmpcWorkspace.x[4] + nmpcWorkspace.d[62];
nmpcVariables.x[68] += + nmpcWorkspace.evGx[315]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[316]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[317]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[318]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[319]*nmpcWorkspace.x[4] + nmpcWorkspace.d[63];
nmpcVariables.x[69] += + nmpcWorkspace.evGx[320]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[321]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[322]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[323]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[324]*nmpcWorkspace.x[4] + nmpcWorkspace.d[64];
nmpcVariables.x[70] += + nmpcWorkspace.evGx[325]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[326]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[327]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[328]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[329]*nmpcWorkspace.x[4] + nmpcWorkspace.d[65];
nmpcVariables.x[71] += + nmpcWorkspace.evGx[330]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[331]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[332]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[333]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[334]*nmpcWorkspace.x[4] + nmpcWorkspace.d[66];
nmpcVariables.x[72] += + nmpcWorkspace.evGx[335]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[336]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[337]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[338]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[339]*nmpcWorkspace.x[4] + nmpcWorkspace.d[67];
nmpcVariables.x[73] += + nmpcWorkspace.evGx[340]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[341]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[342]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[343]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[344]*nmpcWorkspace.x[4] + nmpcWorkspace.d[68];
nmpcVariables.x[74] += + nmpcWorkspace.evGx[345]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[346]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[347]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[348]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[349]*nmpcWorkspace.x[4] + nmpcWorkspace.d[69];
nmpcVariables.x[75] += + nmpcWorkspace.evGx[350]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[351]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[352]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[353]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[354]*nmpcWorkspace.x[4] + nmpcWorkspace.d[70];
nmpcVariables.x[76] += + nmpcWorkspace.evGx[355]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[356]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[357]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[358]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[359]*nmpcWorkspace.x[4] + nmpcWorkspace.d[71];
nmpcVariables.x[77] += + nmpcWorkspace.evGx[360]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[361]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[362]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[363]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[364]*nmpcWorkspace.x[4] + nmpcWorkspace.d[72];
nmpcVariables.x[78] += + nmpcWorkspace.evGx[365]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[366]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[367]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[368]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[369]*nmpcWorkspace.x[4] + nmpcWorkspace.d[73];
nmpcVariables.x[79] += + nmpcWorkspace.evGx[370]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[371]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[372]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[373]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[374]*nmpcWorkspace.x[4] + nmpcWorkspace.d[74];
nmpcVariables.x[80] += + nmpcWorkspace.evGx[375]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[376]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[377]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[378]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[379]*nmpcWorkspace.x[4] + nmpcWorkspace.d[75];
nmpcVariables.x[81] += + nmpcWorkspace.evGx[380]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[381]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[382]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[383]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[384]*nmpcWorkspace.x[4] + nmpcWorkspace.d[76];
nmpcVariables.x[82] += + nmpcWorkspace.evGx[385]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[386]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[387]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[388]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[389]*nmpcWorkspace.x[4] + nmpcWorkspace.d[77];
nmpcVariables.x[83] += + nmpcWorkspace.evGx[390]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[391]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[392]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[393]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[394]*nmpcWorkspace.x[4] + nmpcWorkspace.d[78];
nmpcVariables.x[84] += + nmpcWorkspace.evGx[395]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[396]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[397]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[398]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[399]*nmpcWorkspace.x[4] + nmpcWorkspace.d[79];
nmpcVariables.x[85] += + nmpcWorkspace.evGx[400]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[401]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[402]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[403]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[404]*nmpcWorkspace.x[4] + nmpcWorkspace.d[80];
nmpcVariables.x[86] += + nmpcWorkspace.evGx[405]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[406]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[407]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[408]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[409]*nmpcWorkspace.x[4] + nmpcWorkspace.d[81];
nmpcVariables.x[87] += + nmpcWorkspace.evGx[410]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[411]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[412]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[413]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[414]*nmpcWorkspace.x[4] + nmpcWorkspace.d[82];
nmpcVariables.x[88] += + nmpcWorkspace.evGx[415]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[416]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[417]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[418]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[419]*nmpcWorkspace.x[4] + nmpcWorkspace.d[83];
nmpcVariables.x[89] += + nmpcWorkspace.evGx[420]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[421]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[422]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[423]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[424]*nmpcWorkspace.x[4] + nmpcWorkspace.d[84];
nmpcVariables.x[90] += + nmpcWorkspace.evGx[425]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[426]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[427]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[428]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[429]*nmpcWorkspace.x[4] + nmpcWorkspace.d[85];
nmpcVariables.x[91] += + nmpcWorkspace.evGx[430]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[431]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[432]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[433]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[434]*nmpcWorkspace.x[4] + nmpcWorkspace.d[86];
nmpcVariables.x[92] += + nmpcWorkspace.evGx[435]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[436]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[437]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[438]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[439]*nmpcWorkspace.x[4] + nmpcWorkspace.d[87];
nmpcVariables.x[93] += + nmpcWorkspace.evGx[440]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[441]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[442]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[443]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[444]*nmpcWorkspace.x[4] + nmpcWorkspace.d[88];
nmpcVariables.x[94] += + nmpcWorkspace.evGx[445]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[446]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[447]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[448]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[449]*nmpcWorkspace.x[4] + nmpcWorkspace.d[89];
nmpcVariables.x[95] += + nmpcWorkspace.evGx[450]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[451]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[452]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[453]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[454]*nmpcWorkspace.x[4] + nmpcWorkspace.d[90];
nmpcVariables.x[96] += + nmpcWorkspace.evGx[455]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[456]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[457]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[458]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[459]*nmpcWorkspace.x[4] + nmpcWorkspace.d[91];
nmpcVariables.x[97] += + nmpcWorkspace.evGx[460]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[461]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[462]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[463]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[464]*nmpcWorkspace.x[4] + nmpcWorkspace.d[92];
nmpcVariables.x[98] += + nmpcWorkspace.evGx[465]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[466]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[467]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[468]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[469]*nmpcWorkspace.x[4] + nmpcWorkspace.d[93];
nmpcVariables.x[99] += + nmpcWorkspace.evGx[470]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[471]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[472]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[473]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[474]*nmpcWorkspace.x[4] + nmpcWorkspace.d[94];
nmpcVariables.x[100] += + nmpcWorkspace.evGx[475]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[476]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[477]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[478]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[479]*nmpcWorkspace.x[4] + nmpcWorkspace.d[95];
nmpcVariables.x[101] += + nmpcWorkspace.evGx[480]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[481]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[482]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[483]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[484]*nmpcWorkspace.x[4] + nmpcWorkspace.d[96];
nmpcVariables.x[102] += + nmpcWorkspace.evGx[485]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[486]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[487]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[488]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[489]*nmpcWorkspace.x[4] + nmpcWorkspace.d[97];
nmpcVariables.x[103] += + nmpcWorkspace.evGx[490]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[491]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[492]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[493]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[494]*nmpcWorkspace.x[4] + nmpcWorkspace.d[98];
nmpcVariables.x[104] += + nmpcWorkspace.evGx[495]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[496]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[497]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[498]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[499]*nmpcWorkspace.x[4] + nmpcWorkspace.d[99];
nmpcVariables.x[105] += + nmpcWorkspace.evGx[500]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[501]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[502]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[503]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[504]*nmpcWorkspace.x[4] + nmpcWorkspace.d[100];
nmpcVariables.x[106] += + nmpcWorkspace.evGx[505]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[506]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[507]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[508]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[509]*nmpcWorkspace.x[4] + nmpcWorkspace.d[101];
nmpcVariables.x[107] += + nmpcWorkspace.evGx[510]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[511]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[512]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[513]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[514]*nmpcWorkspace.x[4] + nmpcWorkspace.d[102];
nmpcVariables.x[108] += + nmpcWorkspace.evGx[515]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[516]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[517]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[518]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[519]*nmpcWorkspace.x[4] + nmpcWorkspace.d[103];
nmpcVariables.x[109] += + nmpcWorkspace.evGx[520]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[521]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[522]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[523]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[524]*nmpcWorkspace.x[4] + nmpcWorkspace.d[104];
nmpcVariables.x[110] += + nmpcWorkspace.evGx[525]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[526]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[527]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[528]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[529]*nmpcWorkspace.x[4] + nmpcWorkspace.d[105];
nmpcVariables.x[111] += + nmpcWorkspace.evGx[530]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[531]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[532]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[533]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[534]*nmpcWorkspace.x[4] + nmpcWorkspace.d[106];
nmpcVariables.x[112] += + nmpcWorkspace.evGx[535]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[536]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[537]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[538]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[539]*nmpcWorkspace.x[4] + nmpcWorkspace.d[107];
nmpcVariables.x[113] += + nmpcWorkspace.evGx[540]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[541]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[542]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[543]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[544]*nmpcWorkspace.x[4] + nmpcWorkspace.d[108];
nmpcVariables.x[114] += + nmpcWorkspace.evGx[545]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[546]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[547]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[548]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[549]*nmpcWorkspace.x[4] + nmpcWorkspace.d[109];
nmpcVariables.x[115] += + nmpcWorkspace.evGx[550]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[551]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[552]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[553]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[554]*nmpcWorkspace.x[4] + nmpcWorkspace.d[110];
nmpcVariables.x[116] += + nmpcWorkspace.evGx[555]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[556]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[557]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[558]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[559]*nmpcWorkspace.x[4] + nmpcWorkspace.d[111];
nmpcVariables.x[117] += + nmpcWorkspace.evGx[560]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[561]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[562]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[563]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[564]*nmpcWorkspace.x[4] + nmpcWorkspace.d[112];
nmpcVariables.x[118] += + nmpcWorkspace.evGx[565]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[566]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[567]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[568]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[569]*nmpcWorkspace.x[4] + nmpcWorkspace.d[113];
nmpcVariables.x[119] += + nmpcWorkspace.evGx[570]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[571]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[572]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[573]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[574]*nmpcWorkspace.x[4] + nmpcWorkspace.d[114];
nmpcVariables.x[120] += + nmpcWorkspace.evGx[575]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[576]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[577]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[578]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[579]*nmpcWorkspace.x[4] + nmpcWorkspace.d[115];
nmpcVariables.x[121] += + nmpcWorkspace.evGx[580]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[581]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[582]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[583]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[584]*nmpcWorkspace.x[4] + nmpcWorkspace.d[116];
nmpcVariables.x[122] += + nmpcWorkspace.evGx[585]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[586]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[587]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[588]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[589]*nmpcWorkspace.x[4] + nmpcWorkspace.d[117];
nmpcVariables.x[123] += + nmpcWorkspace.evGx[590]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[591]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[592]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[593]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[594]*nmpcWorkspace.x[4] + nmpcWorkspace.d[118];
nmpcVariables.x[124] += + nmpcWorkspace.evGx[595]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[596]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[597]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[598]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[599]*nmpcWorkspace.x[4] + nmpcWorkspace.d[119];
nmpcVariables.x[125] += + nmpcWorkspace.evGx[600]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[601]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[602]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[603]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[604]*nmpcWorkspace.x[4] + nmpcWorkspace.d[120];
nmpcVariables.x[126] += + nmpcWorkspace.evGx[605]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[606]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[607]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[608]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[609]*nmpcWorkspace.x[4] + nmpcWorkspace.d[121];
nmpcVariables.x[127] += + nmpcWorkspace.evGx[610]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[611]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[612]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[613]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[614]*nmpcWorkspace.x[4] + nmpcWorkspace.d[122];
nmpcVariables.x[128] += + nmpcWorkspace.evGx[615]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[616]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[617]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[618]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[619]*nmpcWorkspace.x[4] + nmpcWorkspace.d[123];
nmpcVariables.x[129] += + nmpcWorkspace.evGx[620]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[621]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[622]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[623]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[624]*nmpcWorkspace.x[4] + nmpcWorkspace.d[124];
nmpcVariables.x[130] += + nmpcWorkspace.evGx[625]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[626]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[627]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[628]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[629]*nmpcWorkspace.x[4] + nmpcWorkspace.d[125];
nmpcVariables.x[131] += + nmpcWorkspace.evGx[630]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[631]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[632]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[633]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[634]*nmpcWorkspace.x[4] + nmpcWorkspace.d[126];
nmpcVariables.x[132] += + nmpcWorkspace.evGx[635]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[636]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[637]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[638]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[639]*nmpcWorkspace.x[4] + nmpcWorkspace.d[127];
nmpcVariables.x[133] += + nmpcWorkspace.evGx[640]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[641]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[642]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[643]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[644]*nmpcWorkspace.x[4] + nmpcWorkspace.d[128];
nmpcVariables.x[134] += + nmpcWorkspace.evGx[645]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[646]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[647]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[648]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[649]*nmpcWorkspace.x[4] + nmpcWorkspace.d[129];
nmpcVariables.x[135] += + nmpcWorkspace.evGx[650]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[651]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[652]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[653]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[654]*nmpcWorkspace.x[4] + nmpcWorkspace.d[130];
nmpcVariables.x[136] += + nmpcWorkspace.evGx[655]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[656]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[657]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[658]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[659]*nmpcWorkspace.x[4] + nmpcWorkspace.d[131];
nmpcVariables.x[137] += + nmpcWorkspace.evGx[660]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[661]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[662]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[663]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[664]*nmpcWorkspace.x[4] + nmpcWorkspace.d[132];
nmpcVariables.x[138] += + nmpcWorkspace.evGx[665]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[666]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[667]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[668]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[669]*nmpcWorkspace.x[4] + nmpcWorkspace.d[133];
nmpcVariables.x[139] += + nmpcWorkspace.evGx[670]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[671]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[672]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[673]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[674]*nmpcWorkspace.x[4] + nmpcWorkspace.d[134];
nmpcVariables.x[140] += + nmpcWorkspace.evGx[675]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[676]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[677]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[678]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[679]*nmpcWorkspace.x[4] + nmpcWorkspace.d[135];
nmpcVariables.x[141] += + nmpcWorkspace.evGx[680]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[681]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[682]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[683]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[684]*nmpcWorkspace.x[4] + nmpcWorkspace.d[136];
nmpcVariables.x[142] += + nmpcWorkspace.evGx[685]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[686]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[687]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[688]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[689]*nmpcWorkspace.x[4] + nmpcWorkspace.d[137];
nmpcVariables.x[143] += + nmpcWorkspace.evGx[690]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[691]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[692]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[693]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[694]*nmpcWorkspace.x[4] + nmpcWorkspace.d[138];
nmpcVariables.x[144] += + nmpcWorkspace.evGx[695]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[696]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[697]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[698]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[699]*nmpcWorkspace.x[4] + nmpcWorkspace.d[139];
nmpcVariables.x[145] += + nmpcWorkspace.evGx[700]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[701]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[702]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[703]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[704]*nmpcWorkspace.x[4] + nmpcWorkspace.d[140];
nmpcVariables.x[146] += + nmpcWorkspace.evGx[705]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[706]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[707]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[708]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[709]*nmpcWorkspace.x[4] + nmpcWorkspace.d[141];
nmpcVariables.x[147] += + nmpcWorkspace.evGx[710]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[711]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[712]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[713]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[714]*nmpcWorkspace.x[4] + nmpcWorkspace.d[142];
nmpcVariables.x[148] += + nmpcWorkspace.evGx[715]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[716]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[717]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[718]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[719]*nmpcWorkspace.x[4] + nmpcWorkspace.d[143];
nmpcVariables.x[149] += + nmpcWorkspace.evGx[720]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[721]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[722]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[723]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[724]*nmpcWorkspace.x[4] + nmpcWorkspace.d[144];
nmpcVariables.x[150] += + nmpcWorkspace.evGx[725]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[726]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[727]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[728]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[729]*nmpcWorkspace.x[4] + nmpcWorkspace.d[145];
nmpcVariables.x[151] += + nmpcWorkspace.evGx[730]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[731]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[732]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[733]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[734]*nmpcWorkspace.x[4] + nmpcWorkspace.d[146];
nmpcVariables.x[152] += + nmpcWorkspace.evGx[735]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[736]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[737]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[738]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[739]*nmpcWorkspace.x[4] + nmpcWorkspace.d[147];
nmpcVariables.x[153] += + nmpcWorkspace.evGx[740]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[741]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[742]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[743]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[744]*nmpcWorkspace.x[4] + nmpcWorkspace.d[148];
nmpcVariables.x[154] += + nmpcWorkspace.evGx[745]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[746]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[747]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[748]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[749]*nmpcWorkspace.x[4] + nmpcWorkspace.d[149];

for (lRun1 = 0; lRun1 < 30; ++lRun1)
{
for (lRun2 = 0; lRun2 < lRun1 + 1; ++lRun2)
{
lRun3 = (((lRun1 + 1) * (lRun1)) / (2)) + (lRun2);
nmpc_multEDu( &(nmpcWorkspace.E[ lRun3 * 10 ]), &(nmpcWorkspace.x[ lRun2 * 2 + 5 ]), &(nmpcVariables.x[ lRun1 * 5 + 5 ]) );
}
}
}

int nmpc_preparationStep(  )
{
int ret;

ret = nmpc_modelSimulation();
nmpc_evaluateObjective(  );
nmpc_condensePrep(  );
return ret;
}

int nmpc_feedbackStep(  )
{
int tmp;

nmpc_condenseFdb(  );

tmp = nmpc_solve( );

nmpc_expand(  );
return tmp;
}

int nmpc_initializeSolver(  )
{
int ret;

/* This is a function which must be called once before any other function call! */


ret = 0;

memset(&nmpcWorkspace, 0, sizeof( nmpcWorkspace ));
return ret;
}

void nmpc_initializeNodesByForwardSimulation(  )
{
int index;
for (index = 0; index < 30; ++index)
{
nmpcWorkspace.state[0] = nmpcVariables.x[index * 5];
nmpcWorkspace.state[1] = nmpcVariables.x[index * 5 + 1];
nmpcWorkspace.state[2] = nmpcVariables.x[index * 5 + 2];
nmpcWorkspace.state[3] = nmpcVariables.x[index * 5 + 3];
nmpcWorkspace.state[4] = nmpcVariables.x[index * 5 + 4];
nmpcWorkspace.state[40] = nmpcVariables.u[index * 2];
nmpcWorkspace.state[41] = nmpcVariables.u[index * 2 + 1];
nmpcWorkspace.state[42] = nmpcVariables.od[index * 2];
nmpcWorkspace.state[43] = nmpcVariables.od[index * 2 + 1];

nmpc_integrate(nmpcWorkspace.state, index == 0);

nmpcVariables.x[index * 5 + 5] = nmpcWorkspace.state[0];
nmpcVariables.x[index * 5 + 6] = nmpcWorkspace.state[1];
nmpcVariables.x[index * 5 + 7] = nmpcWorkspace.state[2];
nmpcVariables.x[index * 5 + 8] = nmpcWorkspace.state[3];
nmpcVariables.x[index * 5 + 9] = nmpcWorkspace.state[4];
}
}

void nmpc_shiftStates( int strategy, real_t* const xEnd, real_t* const uEnd )
{
int index;
for (index = 0; index < 30; ++index)
{
nmpcVariables.x[index * 5] = nmpcVariables.x[index * 5 + 5];
nmpcVariables.x[index * 5 + 1] = nmpcVariables.x[index * 5 + 6];
nmpcVariables.x[index * 5 + 2] = nmpcVariables.x[index * 5 + 7];
nmpcVariables.x[index * 5 + 3] = nmpcVariables.x[index * 5 + 8];
nmpcVariables.x[index * 5 + 4] = nmpcVariables.x[index * 5 + 9];
}

if (strategy == 1 && xEnd != 0)
{
nmpcVariables.x[150] = xEnd[0];
nmpcVariables.x[151] = xEnd[1];
nmpcVariables.x[152] = xEnd[2];
nmpcVariables.x[153] = xEnd[3];
nmpcVariables.x[154] = xEnd[4];
}
else if (strategy == 2) 
{
nmpcWorkspace.state[0] = nmpcVariables.x[150];
nmpcWorkspace.state[1] = nmpcVariables.x[151];
nmpcWorkspace.state[2] = nmpcVariables.x[152];
nmpcWorkspace.state[3] = nmpcVariables.x[153];
nmpcWorkspace.state[4] = nmpcVariables.x[154];
if (uEnd != 0)
{
nmpcWorkspace.state[40] = uEnd[0];
nmpcWorkspace.state[41] = uEnd[1];
}
else
{
nmpcWorkspace.state[40] = nmpcVariables.u[58];
nmpcWorkspace.state[41] = nmpcVariables.u[59];
}
nmpcWorkspace.state[42] = nmpcVariables.od[60];
nmpcWorkspace.state[43] = nmpcVariables.od[61];

nmpc_integrate(nmpcWorkspace.state, 1);

nmpcVariables.x[150] = nmpcWorkspace.state[0];
nmpcVariables.x[151] = nmpcWorkspace.state[1];
nmpcVariables.x[152] = nmpcWorkspace.state[2];
nmpcVariables.x[153] = nmpcWorkspace.state[3];
nmpcVariables.x[154] = nmpcWorkspace.state[4];
}
}

void nmpc_shiftControls( real_t* const uEnd )
{
int index;
for (index = 0; index < 29; ++index)
{
nmpcVariables.u[index * 2] = nmpcVariables.u[index * 2 + 2];
nmpcVariables.u[index * 2 + 1] = nmpcVariables.u[index * 2 + 3];
}

if (uEnd != 0)
{
nmpcVariables.u[58] = uEnd[0];
nmpcVariables.u[59] = uEnd[1];
}
}

real_t nmpc_getKKT(  )
{
real_t kkt;

int index;
real_t prd;

kkt = + nmpcWorkspace.g[0]*nmpcWorkspace.x[0] + nmpcWorkspace.g[1]*nmpcWorkspace.x[1] + nmpcWorkspace.g[2]*nmpcWorkspace.x[2] + nmpcWorkspace.g[3]*nmpcWorkspace.x[3] + nmpcWorkspace.g[4]*nmpcWorkspace.x[4] + nmpcWorkspace.g[5]*nmpcWorkspace.x[5] + nmpcWorkspace.g[6]*nmpcWorkspace.x[6] + nmpcWorkspace.g[7]*nmpcWorkspace.x[7] + nmpcWorkspace.g[8]*nmpcWorkspace.x[8] + nmpcWorkspace.g[9]*nmpcWorkspace.x[9] + nmpcWorkspace.g[10]*nmpcWorkspace.x[10] + nmpcWorkspace.g[11]*nmpcWorkspace.x[11] + nmpcWorkspace.g[12]*nmpcWorkspace.x[12] + nmpcWorkspace.g[13]*nmpcWorkspace.x[13] + nmpcWorkspace.g[14]*nmpcWorkspace.x[14] + nmpcWorkspace.g[15]*nmpcWorkspace.x[15] + nmpcWorkspace.g[16]*nmpcWorkspace.x[16] + nmpcWorkspace.g[17]*nmpcWorkspace.x[17] + nmpcWorkspace.g[18]*nmpcWorkspace.x[18] + nmpcWorkspace.g[19]*nmpcWorkspace.x[19] + nmpcWorkspace.g[20]*nmpcWorkspace.x[20] + nmpcWorkspace.g[21]*nmpcWorkspace.x[21] + nmpcWorkspace.g[22]*nmpcWorkspace.x[22] + nmpcWorkspace.g[23]*nmpcWorkspace.x[23] + nmpcWorkspace.g[24]*nmpcWorkspace.x[24] + nmpcWorkspace.g[25]*nmpcWorkspace.x[25] + nmpcWorkspace.g[26]*nmpcWorkspace.x[26] + nmpcWorkspace.g[27]*nmpcWorkspace.x[27] + nmpcWorkspace.g[28]*nmpcWorkspace.x[28] + nmpcWorkspace.g[29]*nmpcWorkspace.x[29] + nmpcWorkspace.g[30]*nmpcWorkspace.x[30] + nmpcWorkspace.g[31]*nmpcWorkspace.x[31] + nmpcWorkspace.g[32]*nmpcWorkspace.x[32] + nmpcWorkspace.g[33]*nmpcWorkspace.x[33] + nmpcWorkspace.g[34]*nmpcWorkspace.x[34] + nmpcWorkspace.g[35]*nmpcWorkspace.x[35] + nmpcWorkspace.g[36]*nmpcWorkspace.x[36] + nmpcWorkspace.g[37]*nmpcWorkspace.x[37] + nmpcWorkspace.g[38]*nmpcWorkspace.x[38] + nmpcWorkspace.g[39]*nmpcWorkspace.x[39] + nmpcWorkspace.g[40]*nmpcWorkspace.x[40] + nmpcWorkspace.g[41]*nmpcWorkspace.x[41] + nmpcWorkspace.g[42]*nmpcWorkspace.x[42] + nmpcWorkspace.g[43]*nmpcWorkspace.x[43] + nmpcWorkspace.g[44]*nmpcWorkspace.x[44] + nmpcWorkspace.g[45]*nmpcWorkspace.x[45] + nmpcWorkspace.g[46]*nmpcWorkspace.x[46] + nmpcWorkspace.g[47]*nmpcWorkspace.x[47] + nmpcWorkspace.g[48]*nmpcWorkspace.x[48] + nmpcWorkspace.g[49]*nmpcWorkspace.x[49] + nmpcWorkspace.g[50]*nmpcWorkspace.x[50] + nmpcWorkspace.g[51]*nmpcWorkspace.x[51] + nmpcWorkspace.g[52]*nmpcWorkspace.x[52] + nmpcWorkspace.g[53]*nmpcWorkspace.x[53] + nmpcWorkspace.g[54]*nmpcWorkspace.x[54] + nmpcWorkspace.g[55]*nmpcWorkspace.x[55] + nmpcWorkspace.g[56]*nmpcWorkspace.x[56] + nmpcWorkspace.g[57]*nmpcWorkspace.x[57] + nmpcWorkspace.g[58]*nmpcWorkspace.x[58] + nmpcWorkspace.g[59]*nmpcWorkspace.x[59] + nmpcWorkspace.g[60]*nmpcWorkspace.x[60] + nmpcWorkspace.g[61]*nmpcWorkspace.x[61] + nmpcWorkspace.g[62]*nmpcWorkspace.x[62] + nmpcWorkspace.g[63]*nmpcWorkspace.x[63] + nmpcWorkspace.g[64]*nmpcWorkspace.x[64];
kkt = fabs( kkt );
for (index = 0; index < 65; ++index)
{
prd = nmpcWorkspace.y[index];
if (prd > 1e-12)
kkt += fabs(nmpcWorkspace.lb[index] * prd);
else if (prd < -1e-12)
kkt += fabs(nmpcWorkspace.ub[index] * prd);
}
for (index = 0; index < 60; ++index)
{
prd = nmpcWorkspace.y[index + 65];
if (prd > 1e-12)
kkt += fabs(nmpcWorkspace.lbA[index] * prd);
else if (prd < -1e-12)
kkt += fabs(nmpcWorkspace.ubA[index] * prd);
}
return kkt;
}

real_t nmpc_getObjective(  )
{
real_t objVal;

int lRun1;
/** Row vector of size: 7 */
real_t tmpDy[ 7 ];

/** Row vector of size: 5 */
real_t tmpDyN[ 5 ];

for (lRun1 = 0; lRun1 < 30; ++lRun1)
{
nmpcWorkspace.objValueIn[0] = nmpcVariables.x[lRun1 * 5];
nmpcWorkspace.objValueIn[1] = nmpcVariables.x[lRun1 * 5 + 1];
nmpcWorkspace.objValueIn[2] = nmpcVariables.x[lRun1 * 5 + 2];
nmpcWorkspace.objValueIn[3] = nmpcVariables.x[lRun1 * 5 + 3];
nmpcWorkspace.objValueIn[4] = nmpcVariables.x[lRun1 * 5 + 4];
nmpcWorkspace.objValueIn[5] = nmpcVariables.u[lRun1 * 2];
nmpcWorkspace.objValueIn[6] = nmpcVariables.u[lRun1 * 2 + 1];
nmpcWorkspace.objValueIn[7] = nmpcVariables.od[lRun1 * 2];
nmpcWorkspace.objValueIn[8] = nmpcVariables.od[lRun1 * 2 + 1];

nmpc_evaluateLSQ( nmpcWorkspace.objValueIn, nmpcWorkspace.objValueOut );
nmpcWorkspace.Dy[lRun1 * 7] = nmpcWorkspace.objValueOut[0] - nmpcVariables.y[lRun1 * 7];
nmpcWorkspace.Dy[lRun1 * 7 + 1] = nmpcWorkspace.objValueOut[1] - nmpcVariables.y[lRun1 * 7 + 1];
nmpcWorkspace.Dy[lRun1 * 7 + 2] = nmpcWorkspace.objValueOut[2] - nmpcVariables.y[lRun1 * 7 + 2];
nmpcWorkspace.Dy[lRun1 * 7 + 3] = nmpcWorkspace.objValueOut[3] - nmpcVariables.y[lRun1 * 7 + 3];
nmpcWorkspace.Dy[lRun1 * 7 + 4] = nmpcWorkspace.objValueOut[4] - nmpcVariables.y[lRun1 * 7 + 4];
nmpcWorkspace.Dy[lRun1 * 7 + 5] = nmpcWorkspace.objValueOut[5] - nmpcVariables.y[lRun1 * 7 + 5];
nmpcWorkspace.Dy[lRun1 * 7 + 6] = nmpcWorkspace.objValueOut[6] - nmpcVariables.y[lRun1 * 7 + 6];
}
nmpcWorkspace.objValueIn[0] = nmpcVariables.x[150];
nmpcWorkspace.objValueIn[1] = nmpcVariables.x[151];
nmpcWorkspace.objValueIn[2] = nmpcVariables.x[152];
nmpcWorkspace.objValueIn[3] = nmpcVariables.x[153];
nmpcWorkspace.objValueIn[4] = nmpcVariables.x[154];
nmpcWorkspace.objValueIn[5] = nmpcVariables.od[60];
nmpcWorkspace.objValueIn[6] = nmpcVariables.od[61];
nmpc_evaluateLSQEndTerm( nmpcWorkspace.objValueIn, nmpcWorkspace.objValueOut );
nmpcWorkspace.DyN[0] = nmpcWorkspace.objValueOut[0] - nmpcVariables.yN[0];
nmpcWorkspace.DyN[1] = nmpcWorkspace.objValueOut[1] - nmpcVariables.yN[1];
nmpcWorkspace.DyN[2] = nmpcWorkspace.objValueOut[2] - nmpcVariables.yN[2];
nmpcWorkspace.DyN[3] = nmpcWorkspace.objValueOut[3] - nmpcVariables.yN[3];
nmpcWorkspace.DyN[4] = nmpcWorkspace.objValueOut[4] - nmpcVariables.yN[4];
objVal = 0.0000000000000000e+00;
for (lRun1 = 0; lRun1 < 30; ++lRun1)
{
tmpDy[0] = + nmpcWorkspace.Dy[lRun1 * 7]*nmpcVariables.W[0];
tmpDy[1] = + nmpcWorkspace.Dy[lRun1 * 7 + 1]*nmpcVariables.W[8];
tmpDy[2] = + nmpcWorkspace.Dy[lRun1 * 7 + 2]*nmpcVariables.W[16];
tmpDy[3] = + nmpcWorkspace.Dy[lRun1 * 7 + 3]*nmpcVariables.W[24];
tmpDy[4] = + nmpcWorkspace.Dy[lRun1 * 7 + 4]*nmpcVariables.W[32];
tmpDy[5] = + nmpcWorkspace.Dy[lRun1 * 7 + 5]*nmpcVariables.W[40];
tmpDy[6] = + nmpcWorkspace.Dy[lRun1 * 7 + 6]*nmpcVariables.W[48];
objVal += + nmpcWorkspace.Dy[lRun1 * 7]*tmpDy[0] + nmpcWorkspace.Dy[lRun1 * 7 + 1]*tmpDy[1] + nmpcWorkspace.Dy[lRun1 * 7 + 2]*tmpDy[2] + nmpcWorkspace.Dy[lRun1 * 7 + 3]*tmpDy[3] + nmpcWorkspace.Dy[lRun1 * 7 + 4]*tmpDy[4] + nmpcWorkspace.Dy[lRun1 * 7 + 5]*tmpDy[5] + nmpcWorkspace.Dy[lRun1 * 7 + 6]*tmpDy[6];
}

tmpDyN[0] = + nmpcWorkspace.DyN[0]*nmpcVariables.WN[0];
tmpDyN[1] = + nmpcWorkspace.DyN[1]*nmpcVariables.WN[6];
tmpDyN[2] = + nmpcWorkspace.DyN[2]*nmpcVariables.WN[12];
tmpDyN[3] = + nmpcWorkspace.DyN[3]*nmpcVariables.WN[18];
tmpDyN[4] = + nmpcWorkspace.DyN[4]*nmpcVariables.WN[24];
objVal += + nmpcWorkspace.DyN[0]*tmpDyN[0] + nmpcWorkspace.DyN[1]*tmpDyN[1] + nmpcWorkspace.DyN[2]*tmpDyN[2] + nmpcWorkspace.DyN[3]*tmpDyN[3] + nmpcWorkspace.DyN[4]*tmpDyN[4];

objVal *= 0.5;
return objVal;
}

