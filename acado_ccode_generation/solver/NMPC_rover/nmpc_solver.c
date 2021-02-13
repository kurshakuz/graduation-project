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
nmpcWorkspace.state[0] = nmpcVariables.x[lRun1 * 4];
nmpcWorkspace.state[1] = nmpcVariables.x[lRun1 * 4 + 1];
nmpcWorkspace.state[2] = nmpcVariables.x[lRun1 * 4 + 2];
nmpcWorkspace.state[3] = nmpcVariables.x[lRun1 * 4 + 3];

nmpcWorkspace.state[28] = nmpcVariables.u[lRun1 * 2];
nmpcWorkspace.state[29] = nmpcVariables.u[lRun1 * 2 + 1];
nmpcWorkspace.state[30] = nmpcVariables.od[lRun1 * 2];
nmpcWorkspace.state[31] = nmpcVariables.od[lRun1 * 2 + 1];

ret = nmpc_integrate(nmpcWorkspace.state, 1);

nmpcWorkspace.d[lRun1 * 4] = nmpcWorkspace.state[0] - nmpcVariables.x[lRun1 * 4 + 4];
nmpcWorkspace.d[lRun1 * 4 + 1] = nmpcWorkspace.state[1] - nmpcVariables.x[lRun1 * 4 + 5];
nmpcWorkspace.d[lRun1 * 4 + 2] = nmpcWorkspace.state[2] - nmpcVariables.x[lRun1 * 4 + 6];
nmpcWorkspace.d[lRun1 * 4 + 3] = nmpcWorkspace.state[3] - nmpcVariables.x[lRun1 * 4 + 7];

nmpcWorkspace.evGx[lRun1 * 16] = nmpcWorkspace.state[4];
nmpcWorkspace.evGx[lRun1 * 16 + 1] = nmpcWorkspace.state[5];
nmpcWorkspace.evGx[lRun1 * 16 + 2] = nmpcWorkspace.state[6];
nmpcWorkspace.evGx[lRun1 * 16 + 3] = nmpcWorkspace.state[7];
nmpcWorkspace.evGx[lRun1 * 16 + 4] = nmpcWorkspace.state[8];
nmpcWorkspace.evGx[lRun1 * 16 + 5] = nmpcWorkspace.state[9];
nmpcWorkspace.evGx[lRun1 * 16 + 6] = nmpcWorkspace.state[10];
nmpcWorkspace.evGx[lRun1 * 16 + 7] = nmpcWorkspace.state[11];
nmpcWorkspace.evGx[lRun1 * 16 + 8] = nmpcWorkspace.state[12];
nmpcWorkspace.evGx[lRun1 * 16 + 9] = nmpcWorkspace.state[13];
nmpcWorkspace.evGx[lRun1 * 16 + 10] = nmpcWorkspace.state[14];
nmpcWorkspace.evGx[lRun1 * 16 + 11] = nmpcWorkspace.state[15];
nmpcWorkspace.evGx[lRun1 * 16 + 12] = nmpcWorkspace.state[16];
nmpcWorkspace.evGx[lRun1 * 16 + 13] = nmpcWorkspace.state[17];
nmpcWorkspace.evGx[lRun1 * 16 + 14] = nmpcWorkspace.state[18];
nmpcWorkspace.evGx[lRun1 * 16 + 15] = nmpcWorkspace.state[19];

nmpcWorkspace.evGu[lRun1 * 8] = nmpcWorkspace.state[20];
nmpcWorkspace.evGu[lRun1 * 8 + 1] = nmpcWorkspace.state[21];
nmpcWorkspace.evGu[lRun1 * 8 + 2] = nmpcWorkspace.state[22];
nmpcWorkspace.evGu[lRun1 * 8 + 3] = nmpcWorkspace.state[23];
nmpcWorkspace.evGu[lRun1 * 8 + 4] = nmpcWorkspace.state[24];
nmpcWorkspace.evGu[lRun1 * 8 + 5] = nmpcWorkspace.state[25];
nmpcWorkspace.evGu[lRun1 * 8 + 6] = nmpcWorkspace.state[26];
nmpcWorkspace.evGu[lRun1 * 8 + 7] = nmpcWorkspace.state[27];
}
return ret;
}

void nmpc_evaluateLSQ(const real_t* in, real_t* out)
{
const real_t* xd = in;
const real_t* u = in + 4;

/* Compute outputs: */
out[0] = xd[0];
out[1] = xd[1];
out[2] = xd[2];
out[3] = xd[3];
out[4] = u[0];
out[5] = u[1];
}

void nmpc_evaluateLSQEndTerm(const real_t* in, real_t* out)
{
const real_t* xd = in;

/* Compute outputs: */
out[0] = xd[0];
out[1] = xd[1];
out[2] = xd[2];
out[3] = xd[3];
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
tmpQ1[0] = + tmpQ2[0];
tmpQ1[1] = + tmpQ2[1];
tmpQ1[2] = + tmpQ2[2];
tmpQ1[3] = + tmpQ2[3];
tmpQ1[4] = + tmpQ2[6];
tmpQ1[5] = + tmpQ2[7];
tmpQ1[6] = + tmpQ2[8];
tmpQ1[7] = + tmpQ2[9];
tmpQ1[8] = + tmpQ2[12];
tmpQ1[9] = + tmpQ2[13];
tmpQ1[10] = + tmpQ2[14];
tmpQ1[11] = + tmpQ2[15];
tmpQ1[12] = + tmpQ2[18];
tmpQ1[13] = + tmpQ2[19];
tmpQ1[14] = + tmpQ2[20];
tmpQ1[15] = + tmpQ2[21];
}

void nmpc_setObjR1R2( real_t* const tmpObjS, real_t* const tmpR1, real_t* const tmpR2 )
{
tmpR2[0] = +tmpObjS[24];
tmpR2[1] = +tmpObjS[25];
tmpR2[2] = +tmpObjS[26];
tmpR2[3] = +tmpObjS[27];
tmpR2[4] = +tmpObjS[28];
tmpR2[5] = +tmpObjS[29];
tmpR2[6] = +tmpObjS[30];
tmpR2[7] = +tmpObjS[31];
tmpR2[8] = +tmpObjS[32];
tmpR2[9] = +tmpObjS[33];
tmpR2[10] = +tmpObjS[34];
tmpR2[11] = +tmpObjS[35];
tmpR1[0] = + tmpR2[4];
tmpR1[1] = + tmpR2[5];
tmpR1[2] = + tmpR2[10];
tmpR1[3] = + tmpR2[11];
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
}

void nmpc_evaluateObjective(  )
{
int runObj;
for (runObj = 0; runObj < 30; ++runObj)
{
nmpcWorkspace.objValueIn[0] = nmpcVariables.x[runObj * 4];
nmpcWorkspace.objValueIn[1] = nmpcVariables.x[runObj * 4 + 1];
nmpcWorkspace.objValueIn[2] = nmpcVariables.x[runObj * 4 + 2];
nmpcWorkspace.objValueIn[3] = nmpcVariables.x[runObj * 4 + 3];
nmpcWorkspace.objValueIn[4] = nmpcVariables.u[runObj * 2];
nmpcWorkspace.objValueIn[5] = nmpcVariables.u[runObj * 2 + 1];
nmpcWorkspace.objValueIn[6] = nmpcVariables.od[runObj * 2];
nmpcWorkspace.objValueIn[7] = nmpcVariables.od[runObj * 2 + 1];

nmpc_evaluateLSQ( nmpcWorkspace.objValueIn, nmpcWorkspace.objValueOut );
nmpcWorkspace.Dy[runObj * 6] = nmpcWorkspace.objValueOut[0];
nmpcWorkspace.Dy[runObj * 6 + 1] = nmpcWorkspace.objValueOut[1];
nmpcWorkspace.Dy[runObj * 6 + 2] = nmpcWorkspace.objValueOut[2];
nmpcWorkspace.Dy[runObj * 6 + 3] = nmpcWorkspace.objValueOut[3];
nmpcWorkspace.Dy[runObj * 6 + 4] = nmpcWorkspace.objValueOut[4];
nmpcWorkspace.Dy[runObj * 6 + 5] = nmpcWorkspace.objValueOut[5];

nmpc_setObjQ1Q2( nmpcVariables.W, &(nmpcWorkspace.Q1[ runObj * 16 ]), &(nmpcWorkspace.Q2[ runObj * 24 ]) );

nmpc_setObjR1R2( nmpcVariables.W, &(nmpcWorkspace.R1[ runObj * 4 ]), &(nmpcWorkspace.R2[ runObj * 12 ]) );

}
nmpcWorkspace.objValueIn[0] = nmpcVariables.x[120];
nmpcWorkspace.objValueIn[1] = nmpcVariables.x[121];
nmpcWorkspace.objValueIn[2] = nmpcVariables.x[122];
nmpcWorkspace.objValueIn[3] = nmpcVariables.x[123];
nmpcWorkspace.objValueIn[4] = nmpcVariables.od[60];
nmpcWorkspace.objValueIn[5] = nmpcVariables.od[61];
nmpc_evaluateLSQEndTerm( nmpcWorkspace.objValueIn, nmpcWorkspace.objValueOut );

nmpcWorkspace.DyN[0] = nmpcWorkspace.objValueOut[0];
nmpcWorkspace.DyN[1] = nmpcWorkspace.objValueOut[1];
nmpcWorkspace.DyN[2] = nmpcWorkspace.objValueOut[2];
nmpcWorkspace.DyN[3] = nmpcWorkspace.objValueOut[3];

nmpc_setObjQN1QN2( nmpcVariables.WN, nmpcWorkspace.QN1, nmpcWorkspace.QN2 );

}

void nmpc_multGxd( real_t* const dOld, real_t* const Gx1, real_t* const dNew )
{
dNew[0] += + Gx1[0]*dOld[0] + Gx1[1]*dOld[1] + Gx1[2]*dOld[2] + Gx1[3]*dOld[3];
dNew[1] += + Gx1[4]*dOld[0] + Gx1[5]*dOld[1] + Gx1[6]*dOld[2] + Gx1[7]*dOld[3];
dNew[2] += + Gx1[8]*dOld[0] + Gx1[9]*dOld[1] + Gx1[10]*dOld[2] + Gx1[11]*dOld[3];
dNew[3] += + Gx1[12]*dOld[0] + Gx1[13]*dOld[1] + Gx1[14]*dOld[2] + Gx1[15]*dOld[3];
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
}

void nmpc_multGxGx( real_t* const Gx1, real_t* const Gx2, real_t* const Gx3 )
{
Gx3[0] = + Gx1[0]*Gx2[0] + Gx1[1]*Gx2[4] + Gx1[2]*Gx2[8] + Gx1[3]*Gx2[12];
Gx3[1] = + Gx1[0]*Gx2[1] + Gx1[1]*Gx2[5] + Gx1[2]*Gx2[9] + Gx1[3]*Gx2[13];
Gx3[2] = + Gx1[0]*Gx2[2] + Gx1[1]*Gx2[6] + Gx1[2]*Gx2[10] + Gx1[3]*Gx2[14];
Gx3[3] = + Gx1[0]*Gx2[3] + Gx1[1]*Gx2[7] + Gx1[2]*Gx2[11] + Gx1[3]*Gx2[15];
Gx3[4] = + Gx1[4]*Gx2[0] + Gx1[5]*Gx2[4] + Gx1[6]*Gx2[8] + Gx1[7]*Gx2[12];
Gx3[5] = + Gx1[4]*Gx2[1] + Gx1[5]*Gx2[5] + Gx1[6]*Gx2[9] + Gx1[7]*Gx2[13];
Gx3[6] = + Gx1[4]*Gx2[2] + Gx1[5]*Gx2[6] + Gx1[6]*Gx2[10] + Gx1[7]*Gx2[14];
Gx3[7] = + Gx1[4]*Gx2[3] + Gx1[5]*Gx2[7] + Gx1[6]*Gx2[11] + Gx1[7]*Gx2[15];
Gx3[8] = + Gx1[8]*Gx2[0] + Gx1[9]*Gx2[4] + Gx1[10]*Gx2[8] + Gx1[11]*Gx2[12];
Gx3[9] = + Gx1[8]*Gx2[1] + Gx1[9]*Gx2[5] + Gx1[10]*Gx2[9] + Gx1[11]*Gx2[13];
Gx3[10] = + Gx1[8]*Gx2[2] + Gx1[9]*Gx2[6] + Gx1[10]*Gx2[10] + Gx1[11]*Gx2[14];
Gx3[11] = + Gx1[8]*Gx2[3] + Gx1[9]*Gx2[7] + Gx1[10]*Gx2[11] + Gx1[11]*Gx2[15];
Gx3[12] = + Gx1[12]*Gx2[0] + Gx1[13]*Gx2[4] + Gx1[14]*Gx2[8] + Gx1[15]*Gx2[12];
Gx3[13] = + Gx1[12]*Gx2[1] + Gx1[13]*Gx2[5] + Gx1[14]*Gx2[9] + Gx1[15]*Gx2[13];
Gx3[14] = + Gx1[12]*Gx2[2] + Gx1[13]*Gx2[6] + Gx1[14]*Gx2[10] + Gx1[15]*Gx2[14];
Gx3[15] = + Gx1[12]*Gx2[3] + Gx1[13]*Gx2[7] + Gx1[14]*Gx2[11] + Gx1[15]*Gx2[15];
}

void nmpc_multGxGu( real_t* const Gx1, real_t* const Gu1, real_t* const Gu2 )
{
Gu2[0] = + Gx1[0]*Gu1[0] + Gx1[1]*Gu1[2] + Gx1[2]*Gu1[4] + Gx1[3]*Gu1[6];
Gu2[1] = + Gx1[0]*Gu1[1] + Gx1[1]*Gu1[3] + Gx1[2]*Gu1[5] + Gx1[3]*Gu1[7];
Gu2[2] = + Gx1[4]*Gu1[0] + Gx1[5]*Gu1[2] + Gx1[6]*Gu1[4] + Gx1[7]*Gu1[6];
Gu2[3] = + Gx1[4]*Gu1[1] + Gx1[5]*Gu1[3] + Gx1[6]*Gu1[5] + Gx1[7]*Gu1[7];
Gu2[4] = + Gx1[8]*Gu1[0] + Gx1[9]*Gu1[2] + Gx1[10]*Gu1[4] + Gx1[11]*Gu1[6];
Gu2[5] = + Gx1[8]*Gu1[1] + Gx1[9]*Gu1[3] + Gx1[10]*Gu1[5] + Gx1[11]*Gu1[7];
Gu2[6] = + Gx1[12]*Gu1[0] + Gx1[13]*Gu1[2] + Gx1[14]*Gu1[4] + Gx1[15]*Gu1[6];
Gu2[7] = + Gx1[12]*Gu1[1] + Gx1[13]*Gu1[3] + Gx1[14]*Gu1[5] + Gx1[15]*Gu1[7];
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
}

void nmpc_setBlockH11( int iRow, int iCol, real_t* const Gu1, real_t* const Gu2 )
{
nmpcWorkspace.H[(iRow * 128 + 256) + (iCol * 2 + 4)] += + Gu1[0]*Gu2[0] + Gu1[2]*Gu2[2] + Gu1[4]*Gu2[4] + Gu1[6]*Gu2[6];
nmpcWorkspace.H[(iRow * 128 + 256) + (iCol * 2 + 5)] += + Gu1[0]*Gu2[1] + Gu1[2]*Gu2[3] + Gu1[4]*Gu2[5] + Gu1[6]*Gu2[7];
nmpcWorkspace.H[(iRow * 128 + 320) + (iCol * 2 + 4)] += + Gu1[1]*Gu2[0] + Gu1[3]*Gu2[2] + Gu1[5]*Gu2[4] + Gu1[7]*Gu2[6];
nmpcWorkspace.H[(iRow * 128 + 320) + (iCol * 2 + 5)] += + Gu1[1]*Gu2[1] + Gu1[3]*Gu2[3] + Gu1[5]*Gu2[5] + Gu1[7]*Gu2[7];
}

void nmpc_setBlockH11_R1( int iRow, int iCol, real_t* const R11 )
{
nmpcWorkspace.H[(iRow * 128 + 256) + (iCol * 2 + 4)] = R11[0];
nmpcWorkspace.H[(iRow * 128 + 256) + (iCol * 2 + 5)] = R11[1];
nmpcWorkspace.H[(iRow * 128 + 320) + (iCol * 2 + 4)] = R11[2];
nmpcWorkspace.H[(iRow * 128 + 320) + (iCol * 2 + 5)] = R11[3];
}

void nmpc_zeroBlockH11( int iRow, int iCol )
{
nmpcWorkspace.H[(iRow * 128 + 256) + (iCol * 2 + 4)] = 0.0000000000000000e+00;
nmpcWorkspace.H[(iRow * 128 + 256) + (iCol * 2 + 5)] = 0.0000000000000000e+00;
nmpcWorkspace.H[(iRow * 128 + 320) + (iCol * 2 + 4)] = 0.0000000000000000e+00;
nmpcWorkspace.H[(iRow * 128 + 320) + (iCol * 2 + 5)] = 0.0000000000000000e+00;
}

void nmpc_copyHTH( int iRow, int iCol )
{
nmpcWorkspace.H[(iRow * 128 + 256) + (iCol * 2 + 4)] = nmpcWorkspace.H[(iCol * 128 + 256) + (iRow * 2 + 4)];
nmpcWorkspace.H[(iRow * 128 + 256) + (iCol * 2 + 5)] = nmpcWorkspace.H[(iCol * 128 + 320) + (iRow * 2 + 4)];
nmpcWorkspace.H[(iRow * 128 + 320) + (iCol * 2 + 4)] = nmpcWorkspace.H[(iCol * 128 + 256) + (iRow * 2 + 5)];
nmpcWorkspace.H[(iRow * 128 + 320) + (iCol * 2 + 5)] = nmpcWorkspace.H[(iCol * 128 + 320) + (iRow * 2 + 5)];
}

void nmpc_multQ1d( real_t* const Gx1, real_t* const dOld, real_t* const dNew )
{
dNew[0] = + Gx1[0]*dOld[0] + Gx1[1]*dOld[1] + Gx1[2]*dOld[2] + Gx1[3]*dOld[3];
dNew[1] = + Gx1[4]*dOld[0] + Gx1[5]*dOld[1] + Gx1[6]*dOld[2] + Gx1[7]*dOld[3];
dNew[2] = + Gx1[8]*dOld[0] + Gx1[9]*dOld[1] + Gx1[10]*dOld[2] + Gx1[11]*dOld[3];
dNew[3] = + Gx1[12]*dOld[0] + Gx1[13]*dOld[1] + Gx1[14]*dOld[2] + Gx1[15]*dOld[3];
}

void nmpc_multQN1d( real_t* const QN1, real_t* const dOld, real_t* const dNew )
{
dNew[0] = + nmpcWorkspace.QN1[0]*dOld[0] + nmpcWorkspace.QN1[1]*dOld[1] + nmpcWorkspace.QN1[2]*dOld[2] + nmpcWorkspace.QN1[3]*dOld[3];
dNew[1] = + nmpcWorkspace.QN1[4]*dOld[0] + nmpcWorkspace.QN1[5]*dOld[1] + nmpcWorkspace.QN1[6]*dOld[2] + nmpcWorkspace.QN1[7]*dOld[3];
dNew[2] = + nmpcWorkspace.QN1[8]*dOld[0] + nmpcWorkspace.QN1[9]*dOld[1] + nmpcWorkspace.QN1[10]*dOld[2] + nmpcWorkspace.QN1[11]*dOld[3];
dNew[3] = + nmpcWorkspace.QN1[12]*dOld[0] + nmpcWorkspace.QN1[13]*dOld[1] + nmpcWorkspace.QN1[14]*dOld[2] + nmpcWorkspace.QN1[15]*dOld[3];
}

void nmpc_multRDy( real_t* const R2, real_t* const Dy1, real_t* const RDy1 )
{
RDy1[0] = + R2[0]*Dy1[0] + R2[1]*Dy1[1] + R2[2]*Dy1[2] + R2[3]*Dy1[3] + R2[4]*Dy1[4] + R2[5]*Dy1[5];
RDy1[1] = + R2[6]*Dy1[0] + R2[7]*Dy1[1] + R2[8]*Dy1[2] + R2[9]*Dy1[3] + R2[10]*Dy1[4] + R2[11]*Dy1[5];
}

void nmpc_multQDy( real_t* const Q2, real_t* const Dy1, real_t* const QDy1 )
{
QDy1[0] = + Q2[0]*Dy1[0] + Q2[1]*Dy1[1] + Q2[2]*Dy1[2] + Q2[3]*Dy1[3] + Q2[4]*Dy1[4] + Q2[5]*Dy1[5];
QDy1[1] = + Q2[6]*Dy1[0] + Q2[7]*Dy1[1] + Q2[8]*Dy1[2] + Q2[9]*Dy1[3] + Q2[10]*Dy1[4] + Q2[11]*Dy1[5];
QDy1[2] = + Q2[12]*Dy1[0] + Q2[13]*Dy1[1] + Q2[14]*Dy1[2] + Q2[15]*Dy1[3] + Q2[16]*Dy1[4] + Q2[17]*Dy1[5];
QDy1[3] = + Q2[18]*Dy1[0] + Q2[19]*Dy1[1] + Q2[20]*Dy1[2] + Q2[21]*Dy1[3] + Q2[22]*Dy1[4] + Q2[23]*Dy1[5];
}

void nmpc_multEQDy( real_t* const E1, real_t* const QDy1, real_t* const U1 )
{
U1[0] += + E1[0]*QDy1[0] + E1[2]*QDy1[1] + E1[4]*QDy1[2] + E1[6]*QDy1[3];
U1[1] += + E1[1]*QDy1[0] + E1[3]*QDy1[1] + E1[5]*QDy1[2] + E1[7]*QDy1[3];
}

void nmpc_multQETGx( real_t* const E1, real_t* const Gx1, real_t* const H101 )
{
H101[0] += + E1[0]*Gx1[0] + E1[2]*Gx1[4] + E1[4]*Gx1[8] + E1[6]*Gx1[12];
H101[1] += + E1[0]*Gx1[1] + E1[2]*Gx1[5] + E1[4]*Gx1[9] + E1[6]*Gx1[13];
H101[2] += + E1[0]*Gx1[2] + E1[2]*Gx1[6] + E1[4]*Gx1[10] + E1[6]*Gx1[14];
H101[3] += + E1[0]*Gx1[3] + E1[2]*Gx1[7] + E1[4]*Gx1[11] + E1[6]*Gx1[15];
H101[4] += + E1[1]*Gx1[0] + E1[3]*Gx1[4] + E1[5]*Gx1[8] + E1[7]*Gx1[12];
H101[5] += + E1[1]*Gx1[1] + E1[3]*Gx1[5] + E1[5]*Gx1[9] + E1[7]*Gx1[13];
H101[6] += + E1[1]*Gx1[2] + E1[3]*Gx1[6] + E1[5]*Gx1[10] + E1[7]*Gx1[14];
H101[7] += + E1[1]*Gx1[3] + E1[3]*Gx1[7] + E1[5]*Gx1[11] + E1[7]*Gx1[15];
}

void nmpc_zeroBlockH10( real_t* const H101 )
{
{ int lCopy; for (lCopy = 0; lCopy < 8; lCopy++) H101[ lCopy ] = 0; }
}

void nmpc_multEDu( real_t* const E1, real_t* const U1, real_t* const dNew )
{
dNew[0] += + E1[0]*U1[0] + E1[1]*U1[1];
dNew[1] += + E1[2]*U1[0] + E1[3]*U1[1];
dNew[2] += + E1[4]*U1[0] + E1[5]*U1[1];
dNew[3] += + E1[6]*U1[0] + E1[7]*U1[1];
}

void nmpc_zeroBlockH00(  )
{
nmpcWorkspace.H[0] = 0.0000000000000000e+00;
nmpcWorkspace.H[1] = 0.0000000000000000e+00;
nmpcWorkspace.H[2] = 0.0000000000000000e+00;
nmpcWorkspace.H[3] = 0.0000000000000000e+00;
nmpcWorkspace.H[64] = 0.0000000000000000e+00;
nmpcWorkspace.H[65] = 0.0000000000000000e+00;
nmpcWorkspace.H[66] = 0.0000000000000000e+00;
nmpcWorkspace.H[67] = 0.0000000000000000e+00;
nmpcWorkspace.H[128] = 0.0000000000000000e+00;
nmpcWorkspace.H[129] = 0.0000000000000000e+00;
nmpcWorkspace.H[130] = 0.0000000000000000e+00;
nmpcWorkspace.H[131] = 0.0000000000000000e+00;
nmpcWorkspace.H[192] = 0.0000000000000000e+00;
nmpcWorkspace.H[193] = 0.0000000000000000e+00;
nmpcWorkspace.H[194] = 0.0000000000000000e+00;
nmpcWorkspace.H[195] = 0.0000000000000000e+00;
}

void nmpc_multCTQC( real_t* const Gx1, real_t* const Gx2 )
{
nmpcWorkspace.H[0] += + Gx1[0]*Gx2[0] + Gx1[4]*Gx2[4] + Gx1[8]*Gx2[8] + Gx1[12]*Gx2[12];
nmpcWorkspace.H[1] += + Gx1[0]*Gx2[1] + Gx1[4]*Gx2[5] + Gx1[8]*Gx2[9] + Gx1[12]*Gx2[13];
nmpcWorkspace.H[2] += + Gx1[0]*Gx2[2] + Gx1[4]*Gx2[6] + Gx1[8]*Gx2[10] + Gx1[12]*Gx2[14];
nmpcWorkspace.H[3] += + Gx1[0]*Gx2[3] + Gx1[4]*Gx2[7] + Gx1[8]*Gx2[11] + Gx1[12]*Gx2[15];
nmpcWorkspace.H[64] += + Gx1[1]*Gx2[0] + Gx1[5]*Gx2[4] + Gx1[9]*Gx2[8] + Gx1[13]*Gx2[12];
nmpcWorkspace.H[65] += + Gx1[1]*Gx2[1] + Gx1[5]*Gx2[5] + Gx1[9]*Gx2[9] + Gx1[13]*Gx2[13];
nmpcWorkspace.H[66] += + Gx1[1]*Gx2[2] + Gx1[5]*Gx2[6] + Gx1[9]*Gx2[10] + Gx1[13]*Gx2[14];
nmpcWorkspace.H[67] += + Gx1[1]*Gx2[3] + Gx1[5]*Gx2[7] + Gx1[9]*Gx2[11] + Gx1[13]*Gx2[15];
nmpcWorkspace.H[128] += + Gx1[2]*Gx2[0] + Gx1[6]*Gx2[4] + Gx1[10]*Gx2[8] + Gx1[14]*Gx2[12];
nmpcWorkspace.H[129] += + Gx1[2]*Gx2[1] + Gx1[6]*Gx2[5] + Gx1[10]*Gx2[9] + Gx1[14]*Gx2[13];
nmpcWorkspace.H[130] += + Gx1[2]*Gx2[2] + Gx1[6]*Gx2[6] + Gx1[10]*Gx2[10] + Gx1[14]*Gx2[14];
nmpcWorkspace.H[131] += + Gx1[2]*Gx2[3] + Gx1[6]*Gx2[7] + Gx1[10]*Gx2[11] + Gx1[14]*Gx2[15];
nmpcWorkspace.H[192] += + Gx1[3]*Gx2[0] + Gx1[7]*Gx2[4] + Gx1[11]*Gx2[8] + Gx1[15]*Gx2[12];
nmpcWorkspace.H[193] += + Gx1[3]*Gx2[1] + Gx1[7]*Gx2[5] + Gx1[11]*Gx2[9] + Gx1[15]*Gx2[13];
nmpcWorkspace.H[194] += + Gx1[3]*Gx2[2] + Gx1[7]*Gx2[6] + Gx1[11]*Gx2[10] + Gx1[15]*Gx2[14];
nmpcWorkspace.H[195] += + Gx1[3]*Gx2[3] + Gx1[7]*Gx2[7] + Gx1[11]*Gx2[11] + Gx1[15]*Gx2[15];
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
nmpc_moveGuE( nmpcWorkspace.evGu, nmpcWorkspace.E );
for (lRun1 = 1; lRun1 < 30; ++lRun1)
{
nmpc_moveGxT( &(nmpcWorkspace.evGx[ lRun1 * 16 ]), nmpcWorkspace.T );
nmpc_multGxd( &(nmpcWorkspace.d[ lRun1 * 4-4 ]), &(nmpcWorkspace.evGx[ lRun1 * 16 ]), &(nmpcWorkspace.d[ lRun1 * 4 ]) );
nmpc_multGxGx( nmpcWorkspace.T, &(nmpcWorkspace.evGx[ lRun1 * 16-16 ]), &(nmpcWorkspace.evGx[ lRun1 * 16 ]) );
for (lRun2 = 0; lRun2 < lRun1; ++lRun2)
{
lRun4 = (((lRun1) * (lRun1-1)) / (2)) + (lRun2);
lRun3 = (((lRun1 + 1) * (lRun1)) / (2)) + (lRun2);
nmpc_multGxGu( nmpcWorkspace.T, &(nmpcWorkspace.E[ lRun4 * 8 ]), &(nmpcWorkspace.E[ lRun3 * 8 ]) );
}
lRun3 = (((lRun1 + 1) * (lRun1)) / (2)) + (lRun2);
nmpc_moveGuE( &(nmpcWorkspace.evGu[ lRun1 * 8 ]), &(nmpcWorkspace.E[ lRun3 * 8 ]) );
}

nmpc_multGxGx( &(nmpcWorkspace.Q1[ 16 ]), nmpcWorkspace.evGx, nmpcWorkspace.QGx );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 32 ]), &(nmpcWorkspace.evGx[ 16 ]), &(nmpcWorkspace.QGx[ 16 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 48 ]), &(nmpcWorkspace.evGx[ 32 ]), &(nmpcWorkspace.QGx[ 32 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 64 ]), &(nmpcWorkspace.evGx[ 48 ]), &(nmpcWorkspace.QGx[ 48 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 80 ]), &(nmpcWorkspace.evGx[ 64 ]), &(nmpcWorkspace.QGx[ 64 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 96 ]), &(nmpcWorkspace.evGx[ 80 ]), &(nmpcWorkspace.QGx[ 80 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 112 ]), &(nmpcWorkspace.evGx[ 96 ]), &(nmpcWorkspace.QGx[ 96 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 128 ]), &(nmpcWorkspace.evGx[ 112 ]), &(nmpcWorkspace.QGx[ 112 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 144 ]), &(nmpcWorkspace.evGx[ 128 ]), &(nmpcWorkspace.QGx[ 128 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 160 ]), &(nmpcWorkspace.evGx[ 144 ]), &(nmpcWorkspace.QGx[ 144 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 176 ]), &(nmpcWorkspace.evGx[ 160 ]), &(nmpcWorkspace.QGx[ 160 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 192 ]), &(nmpcWorkspace.evGx[ 176 ]), &(nmpcWorkspace.QGx[ 176 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 208 ]), &(nmpcWorkspace.evGx[ 192 ]), &(nmpcWorkspace.QGx[ 192 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 224 ]), &(nmpcWorkspace.evGx[ 208 ]), &(nmpcWorkspace.QGx[ 208 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 240 ]), &(nmpcWorkspace.evGx[ 224 ]), &(nmpcWorkspace.QGx[ 224 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 256 ]), &(nmpcWorkspace.evGx[ 240 ]), &(nmpcWorkspace.QGx[ 240 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 272 ]), &(nmpcWorkspace.evGx[ 256 ]), &(nmpcWorkspace.QGx[ 256 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 288 ]), &(nmpcWorkspace.evGx[ 272 ]), &(nmpcWorkspace.QGx[ 272 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 304 ]), &(nmpcWorkspace.evGx[ 288 ]), &(nmpcWorkspace.QGx[ 288 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 320 ]), &(nmpcWorkspace.evGx[ 304 ]), &(nmpcWorkspace.QGx[ 304 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 336 ]), &(nmpcWorkspace.evGx[ 320 ]), &(nmpcWorkspace.QGx[ 320 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 352 ]), &(nmpcWorkspace.evGx[ 336 ]), &(nmpcWorkspace.QGx[ 336 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 368 ]), &(nmpcWorkspace.evGx[ 352 ]), &(nmpcWorkspace.QGx[ 352 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 384 ]), &(nmpcWorkspace.evGx[ 368 ]), &(nmpcWorkspace.QGx[ 368 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 400 ]), &(nmpcWorkspace.evGx[ 384 ]), &(nmpcWorkspace.QGx[ 384 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 416 ]), &(nmpcWorkspace.evGx[ 400 ]), &(nmpcWorkspace.QGx[ 400 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 432 ]), &(nmpcWorkspace.evGx[ 416 ]), &(nmpcWorkspace.QGx[ 416 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 448 ]), &(nmpcWorkspace.evGx[ 432 ]), &(nmpcWorkspace.QGx[ 432 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 464 ]), &(nmpcWorkspace.evGx[ 448 ]), &(nmpcWorkspace.QGx[ 448 ]) );
nmpc_multGxGx( nmpcWorkspace.QN1, &(nmpcWorkspace.evGx[ 464 ]), &(nmpcWorkspace.QGx[ 464 ]) );

for (lRun1 = 0; lRun1 < 29; ++lRun1)
{
for (lRun2 = 0; lRun2 < lRun1 + 1; ++lRun2)
{
lRun3 = (((lRun1 + 1) * (lRun1)) / (2)) + (lRun2);
nmpc_multGxGu( &(nmpcWorkspace.Q1[ lRun1 * 16 + 16 ]), &(nmpcWorkspace.E[ lRun3 * 8 ]), &(nmpcWorkspace.QE[ lRun3 * 8 ]) );
}
}

for (lRun2 = 0; lRun2 < lRun1 + 1; ++lRun2)
{
lRun3 = (((lRun1 + 1) * (lRun1)) / (2)) + (lRun2);
nmpc_multGxGu( nmpcWorkspace.QN1, &(nmpcWorkspace.E[ lRun3 * 8 ]), &(nmpcWorkspace.QE[ lRun3 * 8 ]) );
}

nmpc_zeroBlockH00(  );
nmpc_multCTQC( nmpcWorkspace.evGx, nmpcWorkspace.QGx );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 16 ]), &(nmpcWorkspace.QGx[ 16 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 32 ]), &(nmpcWorkspace.QGx[ 32 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 48 ]), &(nmpcWorkspace.QGx[ 48 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 64 ]), &(nmpcWorkspace.QGx[ 64 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 80 ]), &(nmpcWorkspace.QGx[ 80 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 96 ]), &(nmpcWorkspace.QGx[ 96 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 112 ]), &(nmpcWorkspace.QGx[ 112 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 128 ]), &(nmpcWorkspace.QGx[ 128 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 144 ]), &(nmpcWorkspace.QGx[ 144 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 160 ]), &(nmpcWorkspace.QGx[ 160 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 176 ]), &(nmpcWorkspace.QGx[ 176 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 192 ]), &(nmpcWorkspace.QGx[ 192 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 208 ]), &(nmpcWorkspace.QGx[ 208 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 224 ]), &(nmpcWorkspace.QGx[ 224 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 240 ]), &(nmpcWorkspace.QGx[ 240 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 256 ]), &(nmpcWorkspace.QGx[ 256 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 272 ]), &(nmpcWorkspace.QGx[ 272 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 288 ]), &(nmpcWorkspace.QGx[ 288 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 304 ]), &(nmpcWorkspace.QGx[ 304 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 320 ]), &(nmpcWorkspace.QGx[ 320 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 336 ]), &(nmpcWorkspace.QGx[ 336 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 352 ]), &(nmpcWorkspace.QGx[ 352 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 368 ]), &(nmpcWorkspace.QGx[ 368 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 384 ]), &(nmpcWorkspace.QGx[ 384 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 400 ]), &(nmpcWorkspace.QGx[ 400 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 416 ]), &(nmpcWorkspace.QGx[ 416 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 432 ]), &(nmpcWorkspace.QGx[ 432 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 448 ]), &(nmpcWorkspace.QGx[ 448 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 464 ]), &(nmpcWorkspace.QGx[ 464 ]) );

for (lRun1 = 0; lRun1 < 30; ++lRun1)
{
nmpc_zeroBlockH10( &(nmpcWorkspace.H10[ lRun1 * 8 ]) );
for (lRun2 = lRun1; lRun2 < 30; ++lRun2)
{
lRun3 = (((lRun2 + 1) * (lRun2)) / (2)) + (lRun1);
nmpc_multQETGx( &(nmpcWorkspace.QE[ lRun3 * 8 ]), &(nmpcWorkspace.evGx[ lRun2 * 16 ]), &(nmpcWorkspace.H10[ lRun1 * 8 ]) );
}
}

for (lRun1 = 0;lRun1 < 4; ++lRun1)
for (lRun2 = 0;lRun2 < 60; ++lRun2)
nmpcWorkspace.H[(lRun1 * 64) + (lRun2 + 4)] = nmpcWorkspace.H10[(lRun2 * 4) + (lRun1)];

for (lRun1 = 0; lRun1 < 30; ++lRun1)
{
nmpc_setBlockH11_R1( lRun1, lRun1, &(nmpcWorkspace.R1[ lRun1 * 4 ]) );
lRun2 = lRun1;
for (lRun3 = lRun1; lRun3 < 30; ++lRun3)
{
lRun4 = (((lRun3 + 1) * (lRun3)) / (2)) + (lRun1);
lRun5 = (((lRun3 + 1) * (lRun3)) / (2)) + (lRun2);
nmpc_setBlockH11( lRun1, lRun2, &(nmpcWorkspace.E[ lRun4 * 8 ]), &(nmpcWorkspace.QE[ lRun5 * 8 ]) );
}
for (lRun2 = lRun1 + 1; lRun2 < 30; ++lRun2)
{
nmpc_zeroBlockH11( lRun1, lRun2 );
for (lRun3 = lRun2; lRun3 < 30; ++lRun3)
{
lRun4 = (((lRun3 + 1) * (lRun3)) / (2)) + (lRun1);
lRun5 = (((lRun3 + 1) * (lRun3)) / (2)) + (lRun2);
nmpc_setBlockH11( lRun1, lRun2, &(nmpcWorkspace.E[ lRun4 * 8 ]), &(nmpcWorkspace.QE[ lRun5 * 8 ]) );
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
for (lRun2 = 0;lRun2 < 4; ++lRun2)
nmpcWorkspace.H[(lRun1 * 64 + 256) + (lRun2)] = nmpcWorkspace.H10[(lRun1 * 4) + (lRun2)];

nmpc_multQ1d( &(nmpcWorkspace.Q1[ 16 ]), nmpcWorkspace.d, nmpcWorkspace.Qd );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 32 ]), &(nmpcWorkspace.d[ 4 ]), &(nmpcWorkspace.Qd[ 4 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 48 ]), &(nmpcWorkspace.d[ 8 ]), &(nmpcWorkspace.Qd[ 8 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 64 ]), &(nmpcWorkspace.d[ 12 ]), &(nmpcWorkspace.Qd[ 12 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 80 ]), &(nmpcWorkspace.d[ 16 ]), &(nmpcWorkspace.Qd[ 16 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 96 ]), &(nmpcWorkspace.d[ 20 ]), &(nmpcWorkspace.Qd[ 20 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 112 ]), &(nmpcWorkspace.d[ 24 ]), &(nmpcWorkspace.Qd[ 24 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 128 ]), &(nmpcWorkspace.d[ 28 ]), &(nmpcWorkspace.Qd[ 28 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 144 ]), &(nmpcWorkspace.d[ 32 ]), &(nmpcWorkspace.Qd[ 32 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 160 ]), &(nmpcWorkspace.d[ 36 ]), &(nmpcWorkspace.Qd[ 36 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 176 ]), &(nmpcWorkspace.d[ 40 ]), &(nmpcWorkspace.Qd[ 40 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 192 ]), &(nmpcWorkspace.d[ 44 ]), &(nmpcWorkspace.Qd[ 44 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 208 ]), &(nmpcWorkspace.d[ 48 ]), &(nmpcWorkspace.Qd[ 48 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 224 ]), &(nmpcWorkspace.d[ 52 ]), &(nmpcWorkspace.Qd[ 52 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 240 ]), &(nmpcWorkspace.d[ 56 ]), &(nmpcWorkspace.Qd[ 56 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 256 ]), &(nmpcWorkspace.d[ 60 ]), &(nmpcWorkspace.Qd[ 60 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 272 ]), &(nmpcWorkspace.d[ 64 ]), &(nmpcWorkspace.Qd[ 64 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 288 ]), &(nmpcWorkspace.d[ 68 ]), &(nmpcWorkspace.Qd[ 68 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 304 ]), &(nmpcWorkspace.d[ 72 ]), &(nmpcWorkspace.Qd[ 72 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 320 ]), &(nmpcWorkspace.d[ 76 ]), &(nmpcWorkspace.Qd[ 76 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 336 ]), &(nmpcWorkspace.d[ 80 ]), &(nmpcWorkspace.Qd[ 80 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 352 ]), &(nmpcWorkspace.d[ 84 ]), &(nmpcWorkspace.Qd[ 84 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 368 ]), &(nmpcWorkspace.d[ 88 ]), &(nmpcWorkspace.Qd[ 88 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 384 ]), &(nmpcWorkspace.d[ 92 ]), &(nmpcWorkspace.Qd[ 92 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 400 ]), &(nmpcWorkspace.d[ 96 ]), &(nmpcWorkspace.Qd[ 96 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 416 ]), &(nmpcWorkspace.d[ 100 ]), &(nmpcWorkspace.Qd[ 100 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 432 ]), &(nmpcWorkspace.d[ 104 ]), &(nmpcWorkspace.Qd[ 104 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 448 ]), &(nmpcWorkspace.d[ 108 ]), &(nmpcWorkspace.Qd[ 108 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 464 ]), &(nmpcWorkspace.d[ 112 ]), &(nmpcWorkspace.Qd[ 112 ]) );
nmpc_multQN1d( nmpcWorkspace.QN1, &(nmpcWorkspace.d[ 116 ]), &(nmpcWorkspace.Qd[ 116 ]) );

nmpc_macCTSlx( nmpcWorkspace.evGx, nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 16 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 32 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 48 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 64 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 80 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 96 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 112 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 128 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 144 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 160 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 176 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 192 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 208 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 224 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 240 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 256 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 272 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 288 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 304 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 320 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 336 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 352 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 368 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 384 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 400 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 416 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 432 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 448 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 464 ]), nmpcWorkspace.g );
for (lRun1 = 0; lRun1 < 30; ++lRun1)
{
for (lRun2 = lRun1; lRun2 < 30; ++lRun2)
{
lRun3 = (((lRun2 + 1) * (lRun2)) / (2)) + (lRun1);
nmpc_macETSlu( &(nmpcWorkspace.QE[ lRun3 * 8 ]), &(nmpcWorkspace.g[ lRun1 * 2 + 4 ]) );
}
}
nmpcWorkspace.lb[4] = (real_t)-1.0000000000000000e+12 - nmpcVariables.u[0];
nmpcWorkspace.lb[5] = (real_t)-1.0000000000000000e+12 - nmpcVariables.u[1];
nmpcWorkspace.lb[6] = (real_t)-1.0000000000000000e+12 - nmpcVariables.u[2];
nmpcWorkspace.lb[7] = (real_t)-1.0000000000000000e+12 - nmpcVariables.u[3];
nmpcWorkspace.lb[8] = (real_t)-1.0000000000000000e+12 - nmpcVariables.u[4];
nmpcWorkspace.lb[9] = (real_t)-1.0000000000000000e+12 - nmpcVariables.u[5];
nmpcWorkspace.lb[10] = (real_t)-1.0000000000000000e+12 - nmpcVariables.u[6];
nmpcWorkspace.lb[11] = (real_t)-1.0000000000000000e+12 - nmpcVariables.u[7];
nmpcWorkspace.lb[12] = (real_t)-1.0000000000000000e+12 - nmpcVariables.u[8];
nmpcWorkspace.lb[13] = (real_t)-1.0000000000000000e+12 - nmpcVariables.u[9];
nmpcWorkspace.lb[14] = (real_t)-1.0000000000000000e+12 - nmpcVariables.u[10];
nmpcWorkspace.lb[15] = (real_t)-1.0000000000000000e+12 - nmpcVariables.u[11];
nmpcWorkspace.lb[16] = (real_t)-1.0000000000000000e+12 - nmpcVariables.u[12];
nmpcWorkspace.lb[17] = (real_t)-1.0000000000000000e+12 - nmpcVariables.u[13];
nmpcWorkspace.lb[18] = (real_t)-1.0000000000000000e+12 - nmpcVariables.u[14];
nmpcWorkspace.lb[19] = (real_t)-1.0000000000000000e+12 - nmpcVariables.u[15];
nmpcWorkspace.lb[20] = (real_t)-1.0000000000000000e+12 - nmpcVariables.u[16];
nmpcWorkspace.lb[21] = (real_t)-1.0000000000000000e+12 - nmpcVariables.u[17];
nmpcWorkspace.lb[22] = (real_t)-1.0000000000000000e+12 - nmpcVariables.u[18];
nmpcWorkspace.lb[23] = (real_t)-1.0000000000000000e+12 - nmpcVariables.u[19];
nmpcWorkspace.lb[24] = (real_t)-1.0000000000000000e+12 - nmpcVariables.u[20];
nmpcWorkspace.lb[25] = (real_t)-1.0000000000000000e+12 - nmpcVariables.u[21];
nmpcWorkspace.lb[26] = (real_t)-1.0000000000000000e+12 - nmpcVariables.u[22];
nmpcWorkspace.lb[27] = (real_t)-1.0000000000000000e+12 - nmpcVariables.u[23];
nmpcWorkspace.lb[28] = (real_t)-1.0000000000000000e+12 - nmpcVariables.u[24];
nmpcWorkspace.lb[29] = (real_t)-1.0000000000000000e+12 - nmpcVariables.u[25];
nmpcWorkspace.lb[30] = (real_t)-1.0000000000000000e+12 - nmpcVariables.u[26];
nmpcWorkspace.lb[31] = (real_t)-1.0000000000000000e+12 - nmpcVariables.u[27];
nmpcWorkspace.lb[32] = (real_t)-1.0000000000000000e+12 - nmpcVariables.u[28];
nmpcWorkspace.lb[33] = (real_t)-1.0000000000000000e+12 - nmpcVariables.u[29];
nmpcWorkspace.lb[34] = (real_t)-1.0000000000000000e+12 - nmpcVariables.u[30];
nmpcWorkspace.lb[35] = (real_t)-1.0000000000000000e+12 - nmpcVariables.u[31];
nmpcWorkspace.lb[36] = (real_t)-1.0000000000000000e+12 - nmpcVariables.u[32];
nmpcWorkspace.lb[37] = (real_t)-1.0000000000000000e+12 - nmpcVariables.u[33];
nmpcWorkspace.lb[38] = (real_t)-1.0000000000000000e+12 - nmpcVariables.u[34];
nmpcWorkspace.lb[39] = (real_t)-1.0000000000000000e+12 - nmpcVariables.u[35];
nmpcWorkspace.lb[40] = (real_t)-1.0000000000000000e+12 - nmpcVariables.u[36];
nmpcWorkspace.lb[41] = (real_t)-1.0000000000000000e+12 - nmpcVariables.u[37];
nmpcWorkspace.lb[42] = (real_t)-1.0000000000000000e+12 - nmpcVariables.u[38];
nmpcWorkspace.lb[43] = (real_t)-1.0000000000000000e+12 - nmpcVariables.u[39];
nmpcWorkspace.lb[44] = (real_t)-1.0000000000000000e+12 - nmpcVariables.u[40];
nmpcWorkspace.lb[45] = (real_t)-1.0000000000000000e+12 - nmpcVariables.u[41];
nmpcWorkspace.lb[46] = (real_t)-1.0000000000000000e+12 - nmpcVariables.u[42];
nmpcWorkspace.lb[47] = (real_t)-1.0000000000000000e+12 - nmpcVariables.u[43];
nmpcWorkspace.lb[48] = (real_t)-1.0000000000000000e+12 - nmpcVariables.u[44];
nmpcWorkspace.lb[49] = (real_t)-1.0000000000000000e+12 - nmpcVariables.u[45];
nmpcWorkspace.lb[50] = (real_t)-1.0000000000000000e+12 - nmpcVariables.u[46];
nmpcWorkspace.lb[51] = (real_t)-1.0000000000000000e+12 - nmpcVariables.u[47];
nmpcWorkspace.lb[52] = (real_t)-1.0000000000000000e+12 - nmpcVariables.u[48];
nmpcWorkspace.lb[53] = (real_t)-1.0000000000000000e+12 - nmpcVariables.u[49];
nmpcWorkspace.lb[54] = (real_t)-1.0000000000000000e+12 - nmpcVariables.u[50];
nmpcWorkspace.lb[55] = (real_t)-1.0000000000000000e+12 - nmpcVariables.u[51];
nmpcWorkspace.lb[56] = (real_t)-1.0000000000000000e+12 - nmpcVariables.u[52];
nmpcWorkspace.lb[57] = (real_t)-1.0000000000000000e+12 - nmpcVariables.u[53];
nmpcWorkspace.lb[58] = (real_t)-1.0000000000000000e+12 - nmpcVariables.u[54];
nmpcWorkspace.lb[59] = (real_t)-1.0000000000000000e+12 - nmpcVariables.u[55];
nmpcWorkspace.lb[60] = (real_t)-1.0000000000000000e+12 - nmpcVariables.u[56];
nmpcWorkspace.lb[61] = (real_t)-1.0000000000000000e+12 - nmpcVariables.u[57];
nmpcWorkspace.lb[62] = (real_t)-1.0000000000000000e+12 - nmpcVariables.u[58];
nmpcWorkspace.lb[63] = (real_t)-1.0000000000000000e+12 - nmpcVariables.u[59];
nmpcWorkspace.ub[4] = (real_t)1.0000000000000000e+12 - nmpcVariables.u[0];
nmpcWorkspace.ub[5] = (real_t)1.0000000000000000e+12 - nmpcVariables.u[1];
nmpcWorkspace.ub[6] = (real_t)1.0000000000000000e+12 - nmpcVariables.u[2];
nmpcWorkspace.ub[7] = (real_t)1.0000000000000000e+12 - nmpcVariables.u[3];
nmpcWorkspace.ub[8] = (real_t)1.0000000000000000e+12 - nmpcVariables.u[4];
nmpcWorkspace.ub[9] = (real_t)1.0000000000000000e+12 - nmpcVariables.u[5];
nmpcWorkspace.ub[10] = (real_t)1.0000000000000000e+12 - nmpcVariables.u[6];
nmpcWorkspace.ub[11] = (real_t)1.0000000000000000e+12 - nmpcVariables.u[7];
nmpcWorkspace.ub[12] = (real_t)1.0000000000000000e+12 - nmpcVariables.u[8];
nmpcWorkspace.ub[13] = (real_t)1.0000000000000000e+12 - nmpcVariables.u[9];
nmpcWorkspace.ub[14] = (real_t)1.0000000000000000e+12 - nmpcVariables.u[10];
nmpcWorkspace.ub[15] = (real_t)1.0000000000000000e+12 - nmpcVariables.u[11];
nmpcWorkspace.ub[16] = (real_t)1.0000000000000000e+12 - nmpcVariables.u[12];
nmpcWorkspace.ub[17] = (real_t)1.0000000000000000e+12 - nmpcVariables.u[13];
nmpcWorkspace.ub[18] = (real_t)1.0000000000000000e+12 - nmpcVariables.u[14];
nmpcWorkspace.ub[19] = (real_t)1.0000000000000000e+12 - nmpcVariables.u[15];
nmpcWorkspace.ub[20] = (real_t)1.0000000000000000e+12 - nmpcVariables.u[16];
nmpcWorkspace.ub[21] = (real_t)1.0000000000000000e+12 - nmpcVariables.u[17];
nmpcWorkspace.ub[22] = (real_t)1.0000000000000000e+12 - nmpcVariables.u[18];
nmpcWorkspace.ub[23] = (real_t)1.0000000000000000e+12 - nmpcVariables.u[19];
nmpcWorkspace.ub[24] = (real_t)1.0000000000000000e+12 - nmpcVariables.u[20];
nmpcWorkspace.ub[25] = (real_t)1.0000000000000000e+12 - nmpcVariables.u[21];
nmpcWorkspace.ub[26] = (real_t)1.0000000000000000e+12 - nmpcVariables.u[22];
nmpcWorkspace.ub[27] = (real_t)1.0000000000000000e+12 - nmpcVariables.u[23];
nmpcWorkspace.ub[28] = (real_t)1.0000000000000000e+12 - nmpcVariables.u[24];
nmpcWorkspace.ub[29] = (real_t)1.0000000000000000e+12 - nmpcVariables.u[25];
nmpcWorkspace.ub[30] = (real_t)1.0000000000000000e+12 - nmpcVariables.u[26];
nmpcWorkspace.ub[31] = (real_t)1.0000000000000000e+12 - nmpcVariables.u[27];
nmpcWorkspace.ub[32] = (real_t)1.0000000000000000e+12 - nmpcVariables.u[28];
nmpcWorkspace.ub[33] = (real_t)1.0000000000000000e+12 - nmpcVariables.u[29];
nmpcWorkspace.ub[34] = (real_t)1.0000000000000000e+12 - nmpcVariables.u[30];
nmpcWorkspace.ub[35] = (real_t)1.0000000000000000e+12 - nmpcVariables.u[31];
nmpcWorkspace.ub[36] = (real_t)1.0000000000000000e+12 - nmpcVariables.u[32];
nmpcWorkspace.ub[37] = (real_t)1.0000000000000000e+12 - nmpcVariables.u[33];
nmpcWorkspace.ub[38] = (real_t)1.0000000000000000e+12 - nmpcVariables.u[34];
nmpcWorkspace.ub[39] = (real_t)1.0000000000000000e+12 - nmpcVariables.u[35];
nmpcWorkspace.ub[40] = (real_t)1.0000000000000000e+12 - nmpcVariables.u[36];
nmpcWorkspace.ub[41] = (real_t)1.0000000000000000e+12 - nmpcVariables.u[37];
nmpcWorkspace.ub[42] = (real_t)1.0000000000000000e+12 - nmpcVariables.u[38];
nmpcWorkspace.ub[43] = (real_t)1.0000000000000000e+12 - nmpcVariables.u[39];
nmpcWorkspace.ub[44] = (real_t)1.0000000000000000e+12 - nmpcVariables.u[40];
nmpcWorkspace.ub[45] = (real_t)1.0000000000000000e+12 - nmpcVariables.u[41];
nmpcWorkspace.ub[46] = (real_t)1.0000000000000000e+12 - nmpcVariables.u[42];
nmpcWorkspace.ub[47] = (real_t)1.0000000000000000e+12 - nmpcVariables.u[43];
nmpcWorkspace.ub[48] = (real_t)1.0000000000000000e+12 - nmpcVariables.u[44];
nmpcWorkspace.ub[49] = (real_t)1.0000000000000000e+12 - nmpcVariables.u[45];
nmpcWorkspace.ub[50] = (real_t)1.0000000000000000e+12 - nmpcVariables.u[46];
nmpcWorkspace.ub[51] = (real_t)1.0000000000000000e+12 - nmpcVariables.u[47];
nmpcWorkspace.ub[52] = (real_t)1.0000000000000000e+12 - nmpcVariables.u[48];
nmpcWorkspace.ub[53] = (real_t)1.0000000000000000e+12 - nmpcVariables.u[49];
nmpcWorkspace.ub[54] = (real_t)1.0000000000000000e+12 - nmpcVariables.u[50];
nmpcWorkspace.ub[55] = (real_t)1.0000000000000000e+12 - nmpcVariables.u[51];
nmpcWorkspace.ub[56] = (real_t)1.0000000000000000e+12 - nmpcVariables.u[52];
nmpcWorkspace.ub[57] = (real_t)1.0000000000000000e+12 - nmpcVariables.u[53];
nmpcWorkspace.ub[58] = (real_t)1.0000000000000000e+12 - nmpcVariables.u[54];
nmpcWorkspace.ub[59] = (real_t)1.0000000000000000e+12 - nmpcVariables.u[55];
nmpcWorkspace.ub[60] = (real_t)1.0000000000000000e+12 - nmpcVariables.u[56];
nmpcWorkspace.ub[61] = (real_t)1.0000000000000000e+12 - nmpcVariables.u[57];
nmpcWorkspace.ub[62] = (real_t)1.0000000000000000e+12 - nmpcVariables.u[58];
nmpcWorkspace.ub[63] = (real_t)1.0000000000000000e+12 - nmpcVariables.u[59];

}

void nmpc_condenseFdb(  )
{
int lRun1;
int lRun2;
int lRun3;
nmpcWorkspace.Dx0[0] = nmpcVariables.x0[0] - nmpcVariables.x[0];
nmpcWorkspace.Dx0[1] = nmpcVariables.x0[1] - nmpcVariables.x[1];
nmpcWorkspace.Dx0[2] = nmpcVariables.x0[2] - nmpcVariables.x[2];
nmpcWorkspace.Dx0[3] = nmpcVariables.x0[3] - nmpcVariables.x[3];

for (lRun2 = 0; lRun2 < 180; ++lRun2)
nmpcWorkspace.Dy[lRun2] -= nmpcVariables.y[lRun2];

nmpcWorkspace.DyN[0] -= nmpcVariables.yN[0];
nmpcWorkspace.DyN[1] -= nmpcVariables.yN[1];
nmpcWorkspace.DyN[2] -= nmpcVariables.yN[2];
nmpcWorkspace.DyN[3] -= nmpcVariables.yN[3];

nmpc_multRDy( nmpcWorkspace.R2, nmpcWorkspace.Dy, &(nmpcWorkspace.g[ 4 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 12 ]), &(nmpcWorkspace.Dy[ 6 ]), &(nmpcWorkspace.g[ 6 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 24 ]), &(nmpcWorkspace.Dy[ 12 ]), &(nmpcWorkspace.g[ 8 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 36 ]), &(nmpcWorkspace.Dy[ 18 ]), &(nmpcWorkspace.g[ 10 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 48 ]), &(nmpcWorkspace.Dy[ 24 ]), &(nmpcWorkspace.g[ 12 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 60 ]), &(nmpcWorkspace.Dy[ 30 ]), &(nmpcWorkspace.g[ 14 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 72 ]), &(nmpcWorkspace.Dy[ 36 ]), &(nmpcWorkspace.g[ 16 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 84 ]), &(nmpcWorkspace.Dy[ 42 ]), &(nmpcWorkspace.g[ 18 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 96 ]), &(nmpcWorkspace.Dy[ 48 ]), &(nmpcWorkspace.g[ 20 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 108 ]), &(nmpcWorkspace.Dy[ 54 ]), &(nmpcWorkspace.g[ 22 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 120 ]), &(nmpcWorkspace.Dy[ 60 ]), &(nmpcWorkspace.g[ 24 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 132 ]), &(nmpcWorkspace.Dy[ 66 ]), &(nmpcWorkspace.g[ 26 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 144 ]), &(nmpcWorkspace.Dy[ 72 ]), &(nmpcWorkspace.g[ 28 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 156 ]), &(nmpcWorkspace.Dy[ 78 ]), &(nmpcWorkspace.g[ 30 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 168 ]), &(nmpcWorkspace.Dy[ 84 ]), &(nmpcWorkspace.g[ 32 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 180 ]), &(nmpcWorkspace.Dy[ 90 ]), &(nmpcWorkspace.g[ 34 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 192 ]), &(nmpcWorkspace.Dy[ 96 ]), &(nmpcWorkspace.g[ 36 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 204 ]), &(nmpcWorkspace.Dy[ 102 ]), &(nmpcWorkspace.g[ 38 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 216 ]), &(nmpcWorkspace.Dy[ 108 ]), &(nmpcWorkspace.g[ 40 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 228 ]), &(nmpcWorkspace.Dy[ 114 ]), &(nmpcWorkspace.g[ 42 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 240 ]), &(nmpcWorkspace.Dy[ 120 ]), &(nmpcWorkspace.g[ 44 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 252 ]), &(nmpcWorkspace.Dy[ 126 ]), &(nmpcWorkspace.g[ 46 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 264 ]), &(nmpcWorkspace.Dy[ 132 ]), &(nmpcWorkspace.g[ 48 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 276 ]), &(nmpcWorkspace.Dy[ 138 ]), &(nmpcWorkspace.g[ 50 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 288 ]), &(nmpcWorkspace.Dy[ 144 ]), &(nmpcWorkspace.g[ 52 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 300 ]), &(nmpcWorkspace.Dy[ 150 ]), &(nmpcWorkspace.g[ 54 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 312 ]), &(nmpcWorkspace.Dy[ 156 ]), &(nmpcWorkspace.g[ 56 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 324 ]), &(nmpcWorkspace.Dy[ 162 ]), &(nmpcWorkspace.g[ 58 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 336 ]), &(nmpcWorkspace.Dy[ 168 ]), &(nmpcWorkspace.g[ 60 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 348 ]), &(nmpcWorkspace.Dy[ 174 ]), &(nmpcWorkspace.g[ 62 ]) );

nmpc_multQDy( nmpcWorkspace.Q2, nmpcWorkspace.Dy, nmpcWorkspace.QDy );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 24 ]), &(nmpcWorkspace.Dy[ 6 ]), &(nmpcWorkspace.QDy[ 4 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 48 ]), &(nmpcWorkspace.Dy[ 12 ]), &(nmpcWorkspace.QDy[ 8 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 72 ]), &(nmpcWorkspace.Dy[ 18 ]), &(nmpcWorkspace.QDy[ 12 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 96 ]), &(nmpcWorkspace.Dy[ 24 ]), &(nmpcWorkspace.QDy[ 16 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 120 ]), &(nmpcWorkspace.Dy[ 30 ]), &(nmpcWorkspace.QDy[ 20 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 144 ]), &(nmpcWorkspace.Dy[ 36 ]), &(nmpcWorkspace.QDy[ 24 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 168 ]), &(nmpcWorkspace.Dy[ 42 ]), &(nmpcWorkspace.QDy[ 28 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 192 ]), &(nmpcWorkspace.Dy[ 48 ]), &(nmpcWorkspace.QDy[ 32 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 216 ]), &(nmpcWorkspace.Dy[ 54 ]), &(nmpcWorkspace.QDy[ 36 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 240 ]), &(nmpcWorkspace.Dy[ 60 ]), &(nmpcWorkspace.QDy[ 40 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 264 ]), &(nmpcWorkspace.Dy[ 66 ]), &(nmpcWorkspace.QDy[ 44 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 288 ]), &(nmpcWorkspace.Dy[ 72 ]), &(nmpcWorkspace.QDy[ 48 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 312 ]), &(nmpcWorkspace.Dy[ 78 ]), &(nmpcWorkspace.QDy[ 52 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 336 ]), &(nmpcWorkspace.Dy[ 84 ]), &(nmpcWorkspace.QDy[ 56 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 360 ]), &(nmpcWorkspace.Dy[ 90 ]), &(nmpcWorkspace.QDy[ 60 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 384 ]), &(nmpcWorkspace.Dy[ 96 ]), &(nmpcWorkspace.QDy[ 64 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 408 ]), &(nmpcWorkspace.Dy[ 102 ]), &(nmpcWorkspace.QDy[ 68 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 432 ]), &(nmpcWorkspace.Dy[ 108 ]), &(nmpcWorkspace.QDy[ 72 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 456 ]), &(nmpcWorkspace.Dy[ 114 ]), &(nmpcWorkspace.QDy[ 76 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 480 ]), &(nmpcWorkspace.Dy[ 120 ]), &(nmpcWorkspace.QDy[ 80 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 504 ]), &(nmpcWorkspace.Dy[ 126 ]), &(nmpcWorkspace.QDy[ 84 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 528 ]), &(nmpcWorkspace.Dy[ 132 ]), &(nmpcWorkspace.QDy[ 88 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 552 ]), &(nmpcWorkspace.Dy[ 138 ]), &(nmpcWorkspace.QDy[ 92 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 576 ]), &(nmpcWorkspace.Dy[ 144 ]), &(nmpcWorkspace.QDy[ 96 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 600 ]), &(nmpcWorkspace.Dy[ 150 ]), &(nmpcWorkspace.QDy[ 100 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 624 ]), &(nmpcWorkspace.Dy[ 156 ]), &(nmpcWorkspace.QDy[ 104 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 648 ]), &(nmpcWorkspace.Dy[ 162 ]), &(nmpcWorkspace.QDy[ 108 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 672 ]), &(nmpcWorkspace.Dy[ 168 ]), &(nmpcWorkspace.QDy[ 112 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 696 ]), &(nmpcWorkspace.Dy[ 174 ]), &(nmpcWorkspace.QDy[ 116 ]) );

nmpcWorkspace.QDy[120] = + nmpcWorkspace.QN2[0]*nmpcWorkspace.DyN[0] + nmpcWorkspace.QN2[1]*nmpcWorkspace.DyN[1] + nmpcWorkspace.QN2[2]*nmpcWorkspace.DyN[2] + nmpcWorkspace.QN2[3]*nmpcWorkspace.DyN[3];
nmpcWorkspace.QDy[121] = + nmpcWorkspace.QN2[4]*nmpcWorkspace.DyN[0] + nmpcWorkspace.QN2[5]*nmpcWorkspace.DyN[1] + nmpcWorkspace.QN2[6]*nmpcWorkspace.DyN[2] + nmpcWorkspace.QN2[7]*nmpcWorkspace.DyN[3];
nmpcWorkspace.QDy[122] = + nmpcWorkspace.QN2[8]*nmpcWorkspace.DyN[0] + nmpcWorkspace.QN2[9]*nmpcWorkspace.DyN[1] + nmpcWorkspace.QN2[10]*nmpcWorkspace.DyN[2] + nmpcWorkspace.QN2[11]*nmpcWorkspace.DyN[3];
nmpcWorkspace.QDy[123] = + nmpcWorkspace.QN2[12]*nmpcWorkspace.DyN[0] + nmpcWorkspace.QN2[13]*nmpcWorkspace.DyN[1] + nmpcWorkspace.QN2[14]*nmpcWorkspace.DyN[2] + nmpcWorkspace.QN2[15]*nmpcWorkspace.DyN[3];

nmpcWorkspace.QDy[4] += nmpcWorkspace.Qd[0];
nmpcWorkspace.QDy[5] += nmpcWorkspace.Qd[1];
nmpcWorkspace.QDy[6] += nmpcWorkspace.Qd[2];
nmpcWorkspace.QDy[7] += nmpcWorkspace.Qd[3];
nmpcWorkspace.QDy[8] += nmpcWorkspace.Qd[4];
nmpcWorkspace.QDy[9] += nmpcWorkspace.Qd[5];
nmpcWorkspace.QDy[10] += nmpcWorkspace.Qd[6];
nmpcWorkspace.QDy[11] += nmpcWorkspace.Qd[7];
nmpcWorkspace.QDy[12] += nmpcWorkspace.Qd[8];
nmpcWorkspace.QDy[13] += nmpcWorkspace.Qd[9];
nmpcWorkspace.QDy[14] += nmpcWorkspace.Qd[10];
nmpcWorkspace.QDy[15] += nmpcWorkspace.Qd[11];
nmpcWorkspace.QDy[16] += nmpcWorkspace.Qd[12];
nmpcWorkspace.QDy[17] += nmpcWorkspace.Qd[13];
nmpcWorkspace.QDy[18] += nmpcWorkspace.Qd[14];
nmpcWorkspace.QDy[19] += nmpcWorkspace.Qd[15];
nmpcWorkspace.QDy[20] += nmpcWorkspace.Qd[16];
nmpcWorkspace.QDy[21] += nmpcWorkspace.Qd[17];
nmpcWorkspace.QDy[22] += nmpcWorkspace.Qd[18];
nmpcWorkspace.QDy[23] += nmpcWorkspace.Qd[19];
nmpcWorkspace.QDy[24] += nmpcWorkspace.Qd[20];
nmpcWorkspace.QDy[25] += nmpcWorkspace.Qd[21];
nmpcWorkspace.QDy[26] += nmpcWorkspace.Qd[22];
nmpcWorkspace.QDy[27] += nmpcWorkspace.Qd[23];
nmpcWorkspace.QDy[28] += nmpcWorkspace.Qd[24];
nmpcWorkspace.QDy[29] += nmpcWorkspace.Qd[25];
nmpcWorkspace.QDy[30] += nmpcWorkspace.Qd[26];
nmpcWorkspace.QDy[31] += nmpcWorkspace.Qd[27];
nmpcWorkspace.QDy[32] += nmpcWorkspace.Qd[28];
nmpcWorkspace.QDy[33] += nmpcWorkspace.Qd[29];
nmpcWorkspace.QDy[34] += nmpcWorkspace.Qd[30];
nmpcWorkspace.QDy[35] += nmpcWorkspace.Qd[31];
nmpcWorkspace.QDy[36] += nmpcWorkspace.Qd[32];
nmpcWorkspace.QDy[37] += nmpcWorkspace.Qd[33];
nmpcWorkspace.QDy[38] += nmpcWorkspace.Qd[34];
nmpcWorkspace.QDy[39] += nmpcWorkspace.Qd[35];
nmpcWorkspace.QDy[40] += nmpcWorkspace.Qd[36];
nmpcWorkspace.QDy[41] += nmpcWorkspace.Qd[37];
nmpcWorkspace.QDy[42] += nmpcWorkspace.Qd[38];
nmpcWorkspace.QDy[43] += nmpcWorkspace.Qd[39];
nmpcWorkspace.QDy[44] += nmpcWorkspace.Qd[40];
nmpcWorkspace.QDy[45] += nmpcWorkspace.Qd[41];
nmpcWorkspace.QDy[46] += nmpcWorkspace.Qd[42];
nmpcWorkspace.QDy[47] += nmpcWorkspace.Qd[43];
nmpcWorkspace.QDy[48] += nmpcWorkspace.Qd[44];
nmpcWorkspace.QDy[49] += nmpcWorkspace.Qd[45];
nmpcWorkspace.QDy[50] += nmpcWorkspace.Qd[46];
nmpcWorkspace.QDy[51] += nmpcWorkspace.Qd[47];
nmpcWorkspace.QDy[52] += nmpcWorkspace.Qd[48];
nmpcWorkspace.QDy[53] += nmpcWorkspace.Qd[49];
nmpcWorkspace.QDy[54] += nmpcWorkspace.Qd[50];
nmpcWorkspace.QDy[55] += nmpcWorkspace.Qd[51];
nmpcWorkspace.QDy[56] += nmpcWorkspace.Qd[52];
nmpcWorkspace.QDy[57] += nmpcWorkspace.Qd[53];
nmpcWorkspace.QDy[58] += nmpcWorkspace.Qd[54];
nmpcWorkspace.QDy[59] += nmpcWorkspace.Qd[55];
nmpcWorkspace.QDy[60] += nmpcWorkspace.Qd[56];
nmpcWorkspace.QDy[61] += nmpcWorkspace.Qd[57];
nmpcWorkspace.QDy[62] += nmpcWorkspace.Qd[58];
nmpcWorkspace.QDy[63] += nmpcWorkspace.Qd[59];
nmpcWorkspace.QDy[64] += nmpcWorkspace.Qd[60];
nmpcWorkspace.QDy[65] += nmpcWorkspace.Qd[61];
nmpcWorkspace.QDy[66] += nmpcWorkspace.Qd[62];
nmpcWorkspace.QDy[67] += nmpcWorkspace.Qd[63];
nmpcWorkspace.QDy[68] += nmpcWorkspace.Qd[64];
nmpcWorkspace.QDy[69] += nmpcWorkspace.Qd[65];
nmpcWorkspace.QDy[70] += nmpcWorkspace.Qd[66];
nmpcWorkspace.QDy[71] += nmpcWorkspace.Qd[67];
nmpcWorkspace.QDy[72] += nmpcWorkspace.Qd[68];
nmpcWorkspace.QDy[73] += nmpcWorkspace.Qd[69];
nmpcWorkspace.QDy[74] += nmpcWorkspace.Qd[70];
nmpcWorkspace.QDy[75] += nmpcWorkspace.Qd[71];
nmpcWorkspace.QDy[76] += nmpcWorkspace.Qd[72];
nmpcWorkspace.QDy[77] += nmpcWorkspace.Qd[73];
nmpcWorkspace.QDy[78] += nmpcWorkspace.Qd[74];
nmpcWorkspace.QDy[79] += nmpcWorkspace.Qd[75];
nmpcWorkspace.QDy[80] += nmpcWorkspace.Qd[76];
nmpcWorkspace.QDy[81] += nmpcWorkspace.Qd[77];
nmpcWorkspace.QDy[82] += nmpcWorkspace.Qd[78];
nmpcWorkspace.QDy[83] += nmpcWorkspace.Qd[79];
nmpcWorkspace.QDy[84] += nmpcWorkspace.Qd[80];
nmpcWorkspace.QDy[85] += nmpcWorkspace.Qd[81];
nmpcWorkspace.QDy[86] += nmpcWorkspace.Qd[82];
nmpcWorkspace.QDy[87] += nmpcWorkspace.Qd[83];
nmpcWorkspace.QDy[88] += nmpcWorkspace.Qd[84];
nmpcWorkspace.QDy[89] += nmpcWorkspace.Qd[85];
nmpcWorkspace.QDy[90] += nmpcWorkspace.Qd[86];
nmpcWorkspace.QDy[91] += nmpcWorkspace.Qd[87];
nmpcWorkspace.QDy[92] += nmpcWorkspace.Qd[88];
nmpcWorkspace.QDy[93] += nmpcWorkspace.Qd[89];
nmpcWorkspace.QDy[94] += nmpcWorkspace.Qd[90];
nmpcWorkspace.QDy[95] += nmpcWorkspace.Qd[91];
nmpcWorkspace.QDy[96] += nmpcWorkspace.Qd[92];
nmpcWorkspace.QDy[97] += nmpcWorkspace.Qd[93];
nmpcWorkspace.QDy[98] += nmpcWorkspace.Qd[94];
nmpcWorkspace.QDy[99] += nmpcWorkspace.Qd[95];
nmpcWorkspace.QDy[100] += nmpcWorkspace.Qd[96];
nmpcWorkspace.QDy[101] += nmpcWorkspace.Qd[97];
nmpcWorkspace.QDy[102] += nmpcWorkspace.Qd[98];
nmpcWorkspace.QDy[103] += nmpcWorkspace.Qd[99];
nmpcWorkspace.QDy[104] += nmpcWorkspace.Qd[100];
nmpcWorkspace.QDy[105] += nmpcWorkspace.Qd[101];
nmpcWorkspace.QDy[106] += nmpcWorkspace.Qd[102];
nmpcWorkspace.QDy[107] += nmpcWorkspace.Qd[103];
nmpcWorkspace.QDy[108] += nmpcWorkspace.Qd[104];
nmpcWorkspace.QDy[109] += nmpcWorkspace.Qd[105];
nmpcWorkspace.QDy[110] += nmpcWorkspace.Qd[106];
nmpcWorkspace.QDy[111] += nmpcWorkspace.Qd[107];
nmpcWorkspace.QDy[112] += nmpcWorkspace.Qd[108];
nmpcWorkspace.QDy[113] += nmpcWorkspace.Qd[109];
nmpcWorkspace.QDy[114] += nmpcWorkspace.Qd[110];
nmpcWorkspace.QDy[115] += nmpcWorkspace.Qd[111];
nmpcWorkspace.QDy[116] += nmpcWorkspace.Qd[112];
nmpcWorkspace.QDy[117] += nmpcWorkspace.Qd[113];
nmpcWorkspace.QDy[118] += nmpcWorkspace.Qd[114];
nmpcWorkspace.QDy[119] += nmpcWorkspace.Qd[115];
nmpcWorkspace.QDy[120] += nmpcWorkspace.Qd[116];
nmpcWorkspace.QDy[121] += nmpcWorkspace.Qd[117];
nmpcWorkspace.QDy[122] += nmpcWorkspace.Qd[118];
nmpcWorkspace.QDy[123] += nmpcWorkspace.Qd[119];

nmpcWorkspace.g[0] = + nmpcWorkspace.evGx[0]*nmpcWorkspace.QDy[4] + nmpcWorkspace.evGx[4]*nmpcWorkspace.QDy[5] + nmpcWorkspace.evGx[8]*nmpcWorkspace.QDy[6] + nmpcWorkspace.evGx[12]*nmpcWorkspace.QDy[7] + nmpcWorkspace.evGx[16]*nmpcWorkspace.QDy[8] + nmpcWorkspace.evGx[20]*nmpcWorkspace.QDy[9] + nmpcWorkspace.evGx[24]*nmpcWorkspace.QDy[10] + nmpcWorkspace.evGx[28]*nmpcWorkspace.QDy[11] + nmpcWorkspace.evGx[32]*nmpcWorkspace.QDy[12] + nmpcWorkspace.evGx[36]*nmpcWorkspace.QDy[13] + nmpcWorkspace.evGx[40]*nmpcWorkspace.QDy[14] + nmpcWorkspace.evGx[44]*nmpcWorkspace.QDy[15] + nmpcWorkspace.evGx[48]*nmpcWorkspace.QDy[16] + nmpcWorkspace.evGx[52]*nmpcWorkspace.QDy[17] + nmpcWorkspace.evGx[56]*nmpcWorkspace.QDy[18] + nmpcWorkspace.evGx[60]*nmpcWorkspace.QDy[19] + nmpcWorkspace.evGx[64]*nmpcWorkspace.QDy[20] + nmpcWorkspace.evGx[68]*nmpcWorkspace.QDy[21] + nmpcWorkspace.evGx[72]*nmpcWorkspace.QDy[22] + nmpcWorkspace.evGx[76]*nmpcWorkspace.QDy[23] + nmpcWorkspace.evGx[80]*nmpcWorkspace.QDy[24] + nmpcWorkspace.evGx[84]*nmpcWorkspace.QDy[25] + nmpcWorkspace.evGx[88]*nmpcWorkspace.QDy[26] + nmpcWorkspace.evGx[92]*nmpcWorkspace.QDy[27] + nmpcWorkspace.evGx[96]*nmpcWorkspace.QDy[28] + nmpcWorkspace.evGx[100]*nmpcWorkspace.QDy[29] + nmpcWorkspace.evGx[104]*nmpcWorkspace.QDy[30] + nmpcWorkspace.evGx[108]*nmpcWorkspace.QDy[31] + nmpcWorkspace.evGx[112]*nmpcWorkspace.QDy[32] + nmpcWorkspace.evGx[116]*nmpcWorkspace.QDy[33] + nmpcWorkspace.evGx[120]*nmpcWorkspace.QDy[34] + nmpcWorkspace.evGx[124]*nmpcWorkspace.QDy[35] + nmpcWorkspace.evGx[128]*nmpcWorkspace.QDy[36] + nmpcWorkspace.evGx[132]*nmpcWorkspace.QDy[37] + nmpcWorkspace.evGx[136]*nmpcWorkspace.QDy[38] + nmpcWorkspace.evGx[140]*nmpcWorkspace.QDy[39] + nmpcWorkspace.evGx[144]*nmpcWorkspace.QDy[40] + nmpcWorkspace.evGx[148]*nmpcWorkspace.QDy[41] + nmpcWorkspace.evGx[152]*nmpcWorkspace.QDy[42] + nmpcWorkspace.evGx[156]*nmpcWorkspace.QDy[43] + nmpcWorkspace.evGx[160]*nmpcWorkspace.QDy[44] + nmpcWorkspace.evGx[164]*nmpcWorkspace.QDy[45] + nmpcWorkspace.evGx[168]*nmpcWorkspace.QDy[46] + nmpcWorkspace.evGx[172]*nmpcWorkspace.QDy[47] + nmpcWorkspace.evGx[176]*nmpcWorkspace.QDy[48] + nmpcWorkspace.evGx[180]*nmpcWorkspace.QDy[49] + nmpcWorkspace.evGx[184]*nmpcWorkspace.QDy[50] + nmpcWorkspace.evGx[188]*nmpcWorkspace.QDy[51] + nmpcWorkspace.evGx[192]*nmpcWorkspace.QDy[52] + nmpcWorkspace.evGx[196]*nmpcWorkspace.QDy[53] + nmpcWorkspace.evGx[200]*nmpcWorkspace.QDy[54] + nmpcWorkspace.evGx[204]*nmpcWorkspace.QDy[55] + nmpcWorkspace.evGx[208]*nmpcWorkspace.QDy[56] + nmpcWorkspace.evGx[212]*nmpcWorkspace.QDy[57] + nmpcWorkspace.evGx[216]*nmpcWorkspace.QDy[58] + nmpcWorkspace.evGx[220]*nmpcWorkspace.QDy[59] + nmpcWorkspace.evGx[224]*nmpcWorkspace.QDy[60] + nmpcWorkspace.evGx[228]*nmpcWorkspace.QDy[61] + nmpcWorkspace.evGx[232]*nmpcWorkspace.QDy[62] + nmpcWorkspace.evGx[236]*nmpcWorkspace.QDy[63] + nmpcWorkspace.evGx[240]*nmpcWorkspace.QDy[64] + nmpcWorkspace.evGx[244]*nmpcWorkspace.QDy[65] + nmpcWorkspace.evGx[248]*nmpcWorkspace.QDy[66] + nmpcWorkspace.evGx[252]*nmpcWorkspace.QDy[67] + nmpcWorkspace.evGx[256]*nmpcWorkspace.QDy[68] + nmpcWorkspace.evGx[260]*nmpcWorkspace.QDy[69] + nmpcWorkspace.evGx[264]*nmpcWorkspace.QDy[70] + nmpcWorkspace.evGx[268]*nmpcWorkspace.QDy[71] + nmpcWorkspace.evGx[272]*nmpcWorkspace.QDy[72] + nmpcWorkspace.evGx[276]*nmpcWorkspace.QDy[73] + nmpcWorkspace.evGx[280]*nmpcWorkspace.QDy[74] + nmpcWorkspace.evGx[284]*nmpcWorkspace.QDy[75] + nmpcWorkspace.evGx[288]*nmpcWorkspace.QDy[76] + nmpcWorkspace.evGx[292]*nmpcWorkspace.QDy[77] + nmpcWorkspace.evGx[296]*nmpcWorkspace.QDy[78] + nmpcWorkspace.evGx[300]*nmpcWorkspace.QDy[79] + nmpcWorkspace.evGx[304]*nmpcWorkspace.QDy[80] + nmpcWorkspace.evGx[308]*nmpcWorkspace.QDy[81] + nmpcWorkspace.evGx[312]*nmpcWorkspace.QDy[82] + nmpcWorkspace.evGx[316]*nmpcWorkspace.QDy[83] + nmpcWorkspace.evGx[320]*nmpcWorkspace.QDy[84] + nmpcWorkspace.evGx[324]*nmpcWorkspace.QDy[85] + nmpcWorkspace.evGx[328]*nmpcWorkspace.QDy[86] + nmpcWorkspace.evGx[332]*nmpcWorkspace.QDy[87] + nmpcWorkspace.evGx[336]*nmpcWorkspace.QDy[88] + nmpcWorkspace.evGx[340]*nmpcWorkspace.QDy[89] + nmpcWorkspace.evGx[344]*nmpcWorkspace.QDy[90] + nmpcWorkspace.evGx[348]*nmpcWorkspace.QDy[91] + nmpcWorkspace.evGx[352]*nmpcWorkspace.QDy[92] + nmpcWorkspace.evGx[356]*nmpcWorkspace.QDy[93] + nmpcWorkspace.evGx[360]*nmpcWorkspace.QDy[94] + nmpcWorkspace.evGx[364]*nmpcWorkspace.QDy[95] + nmpcWorkspace.evGx[368]*nmpcWorkspace.QDy[96] + nmpcWorkspace.evGx[372]*nmpcWorkspace.QDy[97] + nmpcWorkspace.evGx[376]*nmpcWorkspace.QDy[98] + nmpcWorkspace.evGx[380]*nmpcWorkspace.QDy[99] + nmpcWorkspace.evGx[384]*nmpcWorkspace.QDy[100] + nmpcWorkspace.evGx[388]*nmpcWorkspace.QDy[101] + nmpcWorkspace.evGx[392]*nmpcWorkspace.QDy[102] + nmpcWorkspace.evGx[396]*nmpcWorkspace.QDy[103] + nmpcWorkspace.evGx[400]*nmpcWorkspace.QDy[104] + nmpcWorkspace.evGx[404]*nmpcWorkspace.QDy[105] + nmpcWorkspace.evGx[408]*nmpcWorkspace.QDy[106] + nmpcWorkspace.evGx[412]*nmpcWorkspace.QDy[107] + nmpcWorkspace.evGx[416]*nmpcWorkspace.QDy[108] + nmpcWorkspace.evGx[420]*nmpcWorkspace.QDy[109] + nmpcWorkspace.evGx[424]*nmpcWorkspace.QDy[110] + nmpcWorkspace.evGx[428]*nmpcWorkspace.QDy[111] + nmpcWorkspace.evGx[432]*nmpcWorkspace.QDy[112] + nmpcWorkspace.evGx[436]*nmpcWorkspace.QDy[113] + nmpcWorkspace.evGx[440]*nmpcWorkspace.QDy[114] + nmpcWorkspace.evGx[444]*nmpcWorkspace.QDy[115] + nmpcWorkspace.evGx[448]*nmpcWorkspace.QDy[116] + nmpcWorkspace.evGx[452]*nmpcWorkspace.QDy[117] + nmpcWorkspace.evGx[456]*nmpcWorkspace.QDy[118] + nmpcWorkspace.evGx[460]*nmpcWorkspace.QDy[119] + nmpcWorkspace.evGx[464]*nmpcWorkspace.QDy[120] + nmpcWorkspace.evGx[468]*nmpcWorkspace.QDy[121] + nmpcWorkspace.evGx[472]*nmpcWorkspace.QDy[122] + nmpcWorkspace.evGx[476]*nmpcWorkspace.QDy[123];
nmpcWorkspace.g[1] = + nmpcWorkspace.evGx[1]*nmpcWorkspace.QDy[4] + nmpcWorkspace.evGx[5]*nmpcWorkspace.QDy[5] + nmpcWorkspace.evGx[9]*nmpcWorkspace.QDy[6] + nmpcWorkspace.evGx[13]*nmpcWorkspace.QDy[7] + nmpcWorkspace.evGx[17]*nmpcWorkspace.QDy[8] + nmpcWorkspace.evGx[21]*nmpcWorkspace.QDy[9] + nmpcWorkspace.evGx[25]*nmpcWorkspace.QDy[10] + nmpcWorkspace.evGx[29]*nmpcWorkspace.QDy[11] + nmpcWorkspace.evGx[33]*nmpcWorkspace.QDy[12] + nmpcWorkspace.evGx[37]*nmpcWorkspace.QDy[13] + nmpcWorkspace.evGx[41]*nmpcWorkspace.QDy[14] + nmpcWorkspace.evGx[45]*nmpcWorkspace.QDy[15] + nmpcWorkspace.evGx[49]*nmpcWorkspace.QDy[16] + nmpcWorkspace.evGx[53]*nmpcWorkspace.QDy[17] + nmpcWorkspace.evGx[57]*nmpcWorkspace.QDy[18] + nmpcWorkspace.evGx[61]*nmpcWorkspace.QDy[19] + nmpcWorkspace.evGx[65]*nmpcWorkspace.QDy[20] + nmpcWorkspace.evGx[69]*nmpcWorkspace.QDy[21] + nmpcWorkspace.evGx[73]*nmpcWorkspace.QDy[22] + nmpcWorkspace.evGx[77]*nmpcWorkspace.QDy[23] + nmpcWorkspace.evGx[81]*nmpcWorkspace.QDy[24] + nmpcWorkspace.evGx[85]*nmpcWorkspace.QDy[25] + nmpcWorkspace.evGx[89]*nmpcWorkspace.QDy[26] + nmpcWorkspace.evGx[93]*nmpcWorkspace.QDy[27] + nmpcWorkspace.evGx[97]*nmpcWorkspace.QDy[28] + nmpcWorkspace.evGx[101]*nmpcWorkspace.QDy[29] + nmpcWorkspace.evGx[105]*nmpcWorkspace.QDy[30] + nmpcWorkspace.evGx[109]*nmpcWorkspace.QDy[31] + nmpcWorkspace.evGx[113]*nmpcWorkspace.QDy[32] + nmpcWorkspace.evGx[117]*nmpcWorkspace.QDy[33] + nmpcWorkspace.evGx[121]*nmpcWorkspace.QDy[34] + nmpcWorkspace.evGx[125]*nmpcWorkspace.QDy[35] + nmpcWorkspace.evGx[129]*nmpcWorkspace.QDy[36] + nmpcWorkspace.evGx[133]*nmpcWorkspace.QDy[37] + nmpcWorkspace.evGx[137]*nmpcWorkspace.QDy[38] + nmpcWorkspace.evGx[141]*nmpcWorkspace.QDy[39] + nmpcWorkspace.evGx[145]*nmpcWorkspace.QDy[40] + nmpcWorkspace.evGx[149]*nmpcWorkspace.QDy[41] + nmpcWorkspace.evGx[153]*nmpcWorkspace.QDy[42] + nmpcWorkspace.evGx[157]*nmpcWorkspace.QDy[43] + nmpcWorkspace.evGx[161]*nmpcWorkspace.QDy[44] + nmpcWorkspace.evGx[165]*nmpcWorkspace.QDy[45] + nmpcWorkspace.evGx[169]*nmpcWorkspace.QDy[46] + nmpcWorkspace.evGx[173]*nmpcWorkspace.QDy[47] + nmpcWorkspace.evGx[177]*nmpcWorkspace.QDy[48] + nmpcWorkspace.evGx[181]*nmpcWorkspace.QDy[49] + nmpcWorkspace.evGx[185]*nmpcWorkspace.QDy[50] + nmpcWorkspace.evGx[189]*nmpcWorkspace.QDy[51] + nmpcWorkspace.evGx[193]*nmpcWorkspace.QDy[52] + nmpcWorkspace.evGx[197]*nmpcWorkspace.QDy[53] + nmpcWorkspace.evGx[201]*nmpcWorkspace.QDy[54] + nmpcWorkspace.evGx[205]*nmpcWorkspace.QDy[55] + nmpcWorkspace.evGx[209]*nmpcWorkspace.QDy[56] + nmpcWorkspace.evGx[213]*nmpcWorkspace.QDy[57] + nmpcWorkspace.evGx[217]*nmpcWorkspace.QDy[58] + nmpcWorkspace.evGx[221]*nmpcWorkspace.QDy[59] + nmpcWorkspace.evGx[225]*nmpcWorkspace.QDy[60] + nmpcWorkspace.evGx[229]*nmpcWorkspace.QDy[61] + nmpcWorkspace.evGx[233]*nmpcWorkspace.QDy[62] + nmpcWorkspace.evGx[237]*nmpcWorkspace.QDy[63] + nmpcWorkspace.evGx[241]*nmpcWorkspace.QDy[64] + nmpcWorkspace.evGx[245]*nmpcWorkspace.QDy[65] + nmpcWorkspace.evGx[249]*nmpcWorkspace.QDy[66] + nmpcWorkspace.evGx[253]*nmpcWorkspace.QDy[67] + nmpcWorkspace.evGx[257]*nmpcWorkspace.QDy[68] + nmpcWorkspace.evGx[261]*nmpcWorkspace.QDy[69] + nmpcWorkspace.evGx[265]*nmpcWorkspace.QDy[70] + nmpcWorkspace.evGx[269]*nmpcWorkspace.QDy[71] + nmpcWorkspace.evGx[273]*nmpcWorkspace.QDy[72] + nmpcWorkspace.evGx[277]*nmpcWorkspace.QDy[73] + nmpcWorkspace.evGx[281]*nmpcWorkspace.QDy[74] + nmpcWorkspace.evGx[285]*nmpcWorkspace.QDy[75] + nmpcWorkspace.evGx[289]*nmpcWorkspace.QDy[76] + nmpcWorkspace.evGx[293]*nmpcWorkspace.QDy[77] + nmpcWorkspace.evGx[297]*nmpcWorkspace.QDy[78] + nmpcWorkspace.evGx[301]*nmpcWorkspace.QDy[79] + nmpcWorkspace.evGx[305]*nmpcWorkspace.QDy[80] + nmpcWorkspace.evGx[309]*nmpcWorkspace.QDy[81] + nmpcWorkspace.evGx[313]*nmpcWorkspace.QDy[82] + nmpcWorkspace.evGx[317]*nmpcWorkspace.QDy[83] + nmpcWorkspace.evGx[321]*nmpcWorkspace.QDy[84] + nmpcWorkspace.evGx[325]*nmpcWorkspace.QDy[85] + nmpcWorkspace.evGx[329]*nmpcWorkspace.QDy[86] + nmpcWorkspace.evGx[333]*nmpcWorkspace.QDy[87] + nmpcWorkspace.evGx[337]*nmpcWorkspace.QDy[88] + nmpcWorkspace.evGx[341]*nmpcWorkspace.QDy[89] + nmpcWorkspace.evGx[345]*nmpcWorkspace.QDy[90] + nmpcWorkspace.evGx[349]*nmpcWorkspace.QDy[91] + nmpcWorkspace.evGx[353]*nmpcWorkspace.QDy[92] + nmpcWorkspace.evGx[357]*nmpcWorkspace.QDy[93] + nmpcWorkspace.evGx[361]*nmpcWorkspace.QDy[94] + nmpcWorkspace.evGx[365]*nmpcWorkspace.QDy[95] + nmpcWorkspace.evGx[369]*nmpcWorkspace.QDy[96] + nmpcWorkspace.evGx[373]*nmpcWorkspace.QDy[97] + nmpcWorkspace.evGx[377]*nmpcWorkspace.QDy[98] + nmpcWorkspace.evGx[381]*nmpcWorkspace.QDy[99] + nmpcWorkspace.evGx[385]*nmpcWorkspace.QDy[100] + nmpcWorkspace.evGx[389]*nmpcWorkspace.QDy[101] + nmpcWorkspace.evGx[393]*nmpcWorkspace.QDy[102] + nmpcWorkspace.evGx[397]*nmpcWorkspace.QDy[103] + nmpcWorkspace.evGx[401]*nmpcWorkspace.QDy[104] + nmpcWorkspace.evGx[405]*nmpcWorkspace.QDy[105] + nmpcWorkspace.evGx[409]*nmpcWorkspace.QDy[106] + nmpcWorkspace.evGx[413]*nmpcWorkspace.QDy[107] + nmpcWorkspace.evGx[417]*nmpcWorkspace.QDy[108] + nmpcWorkspace.evGx[421]*nmpcWorkspace.QDy[109] + nmpcWorkspace.evGx[425]*nmpcWorkspace.QDy[110] + nmpcWorkspace.evGx[429]*nmpcWorkspace.QDy[111] + nmpcWorkspace.evGx[433]*nmpcWorkspace.QDy[112] + nmpcWorkspace.evGx[437]*nmpcWorkspace.QDy[113] + nmpcWorkspace.evGx[441]*nmpcWorkspace.QDy[114] + nmpcWorkspace.evGx[445]*nmpcWorkspace.QDy[115] + nmpcWorkspace.evGx[449]*nmpcWorkspace.QDy[116] + nmpcWorkspace.evGx[453]*nmpcWorkspace.QDy[117] + nmpcWorkspace.evGx[457]*nmpcWorkspace.QDy[118] + nmpcWorkspace.evGx[461]*nmpcWorkspace.QDy[119] + nmpcWorkspace.evGx[465]*nmpcWorkspace.QDy[120] + nmpcWorkspace.evGx[469]*nmpcWorkspace.QDy[121] + nmpcWorkspace.evGx[473]*nmpcWorkspace.QDy[122] + nmpcWorkspace.evGx[477]*nmpcWorkspace.QDy[123];
nmpcWorkspace.g[2] = + nmpcWorkspace.evGx[2]*nmpcWorkspace.QDy[4] + nmpcWorkspace.evGx[6]*nmpcWorkspace.QDy[5] + nmpcWorkspace.evGx[10]*nmpcWorkspace.QDy[6] + nmpcWorkspace.evGx[14]*nmpcWorkspace.QDy[7] + nmpcWorkspace.evGx[18]*nmpcWorkspace.QDy[8] + nmpcWorkspace.evGx[22]*nmpcWorkspace.QDy[9] + nmpcWorkspace.evGx[26]*nmpcWorkspace.QDy[10] + nmpcWorkspace.evGx[30]*nmpcWorkspace.QDy[11] + nmpcWorkspace.evGx[34]*nmpcWorkspace.QDy[12] + nmpcWorkspace.evGx[38]*nmpcWorkspace.QDy[13] + nmpcWorkspace.evGx[42]*nmpcWorkspace.QDy[14] + nmpcWorkspace.evGx[46]*nmpcWorkspace.QDy[15] + nmpcWorkspace.evGx[50]*nmpcWorkspace.QDy[16] + nmpcWorkspace.evGx[54]*nmpcWorkspace.QDy[17] + nmpcWorkspace.evGx[58]*nmpcWorkspace.QDy[18] + nmpcWorkspace.evGx[62]*nmpcWorkspace.QDy[19] + nmpcWorkspace.evGx[66]*nmpcWorkspace.QDy[20] + nmpcWorkspace.evGx[70]*nmpcWorkspace.QDy[21] + nmpcWorkspace.evGx[74]*nmpcWorkspace.QDy[22] + nmpcWorkspace.evGx[78]*nmpcWorkspace.QDy[23] + nmpcWorkspace.evGx[82]*nmpcWorkspace.QDy[24] + nmpcWorkspace.evGx[86]*nmpcWorkspace.QDy[25] + nmpcWorkspace.evGx[90]*nmpcWorkspace.QDy[26] + nmpcWorkspace.evGx[94]*nmpcWorkspace.QDy[27] + nmpcWorkspace.evGx[98]*nmpcWorkspace.QDy[28] + nmpcWorkspace.evGx[102]*nmpcWorkspace.QDy[29] + nmpcWorkspace.evGx[106]*nmpcWorkspace.QDy[30] + nmpcWorkspace.evGx[110]*nmpcWorkspace.QDy[31] + nmpcWorkspace.evGx[114]*nmpcWorkspace.QDy[32] + nmpcWorkspace.evGx[118]*nmpcWorkspace.QDy[33] + nmpcWorkspace.evGx[122]*nmpcWorkspace.QDy[34] + nmpcWorkspace.evGx[126]*nmpcWorkspace.QDy[35] + nmpcWorkspace.evGx[130]*nmpcWorkspace.QDy[36] + nmpcWorkspace.evGx[134]*nmpcWorkspace.QDy[37] + nmpcWorkspace.evGx[138]*nmpcWorkspace.QDy[38] + nmpcWorkspace.evGx[142]*nmpcWorkspace.QDy[39] + nmpcWorkspace.evGx[146]*nmpcWorkspace.QDy[40] + nmpcWorkspace.evGx[150]*nmpcWorkspace.QDy[41] + nmpcWorkspace.evGx[154]*nmpcWorkspace.QDy[42] + nmpcWorkspace.evGx[158]*nmpcWorkspace.QDy[43] + nmpcWorkspace.evGx[162]*nmpcWorkspace.QDy[44] + nmpcWorkspace.evGx[166]*nmpcWorkspace.QDy[45] + nmpcWorkspace.evGx[170]*nmpcWorkspace.QDy[46] + nmpcWorkspace.evGx[174]*nmpcWorkspace.QDy[47] + nmpcWorkspace.evGx[178]*nmpcWorkspace.QDy[48] + nmpcWorkspace.evGx[182]*nmpcWorkspace.QDy[49] + nmpcWorkspace.evGx[186]*nmpcWorkspace.QDy[50] + nmpcWorkspace.evGx[190]*nmpcWorkspace.QDy[51] + nmpcWorkspace.evGx[194]*nmpcWorkspace.QDy[52] + nmpcWorkspace.evGx[198]*nmpcWorkspace.QDy[53] + nmpcWorkspace.evGx[202]*nmpcWorkspace.QDy[54] + nmpcWorkspace.evGx[206]*nmpcWorkspace.QDy[55] + nmpcWorkspace.evGx[210]*nmpcWorkspace.QDy[56] + nmpcWorkspace.evGx[214]*nmpcWorkspace.QDy[57] + nmpcWorkspace.evGx[218]*nmpcWorkspace.QDy[58] + nmpcWorkspace.evGx[222]*nmpcWorkspace.QDy[59] + nmpcWorkspace.evGx[226]*nmpcWorkspace.QDy[60] + nmpcWorkspace.evGx[230]*nmpcWorkspace.QDy[61] + nmpcWorkspace.evGx[234]*nmpcWorkspace.QDy[62] + nmpcWorkspace.evGx[238]*nmpcWorkspace.QDy[63] + nmpcWorkspace.evGx[242]*nmpcWorkspace.QDy[64] + nmpcWorkspace.evGx[246]*nmpcWorkspace.QDy[65] + nmpcWorkspace.evGx[250]*nmpcWorkspace.QDy[66] + nmpcWorkspace.evGx[254]*nmpcWorkspace.QDy[67] + nmpcWorkspace.evGx[258]*nmpcWorkspace.QDy[68] + nmpcWorkspace.evGx[262]*nmpcWorkspace.QDy[69] + nmpcWorkspace.evGx[266]*nmpcWorkspace.QDy[70] + nmpcWorkspace.evGx[270]*nmpcWorkspace.QDy[71] + nmpcWorkspace.evGx[274]*nmpcWorkspace.QDy[72] + nmpcWorkspace.evGx[278]*nmpcWorkspace.QDy[73] + nmpcWorkspace.evGx[282]*nmpcWorkspace.QDy[74] + nmpcWorkspace.evGx[286]*nmpcWorkspace.QDy[75] + nmpcWorkspace.evGx[290]*nmpcWorkspace.QDy[76] + nmpcWorkspace.evGx[294]*nmpcWorkspace.QDy[77] + nmpcWorkspace.evGx[298]*nmpcWorkspace.QDy[78] + nmpcWorkspace.evGx[302]*nmpcWorkspace.QDy[79] + nmpcWorkspace.evGx[306]*nmpcWorkspace.QDy[80] + nmpcWorkspace.evGx[310]*nmpcWorkspace.QDy[81] + nmpcWorkspace.evGx[314]*nmpcWorkspace.QDy[82] + nmpcWorkspace.evGx[318]*nmpcWorkspace.QDy[83] + nmpcWorkspace.evGx[322]*nmpcWorkspace.QDy[84] + nmpcWorkspace.evGx[326]*nmpcWorkspace.QDy[85] + nmpcWorkspace.evGx[330]*nmpcWorkspace.QDy[86] + nmpcWorkspace.evGx[334]*nmpcWorkspace.QDy[87] + nmpcWorkspace.evGx[338]*nmpcWorkspace.QDy[88] + nmpcWorkspace.evGx[342]*nmpcWorkspace.QDy[89] + nmpcWorkspace.evGx[346]*nmpcWorkspace.QDy[90] + nmpcWorkspace.evGx[350]*nmpcWorkspace.QDy[91] + nmpcWorkspace.evGx[354]*nmpcWorkspace.QDy[92] + nmpcWorkspace.evGx[358]*nmpcWorkspace.QDy[93] + nmpcWorkspace.evGx[362]*nmpcWorkspace.QDy[94] + nmpcWorkspace.evGx[366]*nmpcWorkspace.QDy[95] + nmpcWorkspace.evGx[370]*nmpcWorkspace.QDy[96] + nmpcWorkspace.evGx[374]*nmpcWorkspace.QDy[97] + nmpcWorkspace.evGx[378]*nmpcWorkspace.QDy[98] + nmpcWorkspace.evGx[382]*nmpcWorkspace.QDy[99] + nmpcWorkspace.evGx[386]*nmpcWorkspace.QDy[100] + nmpcWorkspace.evGx[390]*nmpcWorkspace.QDy[101] + nmpcWorkspace.evGx[394]*nmpcWorkspace.QDy[102] + nmpcWorkspace.evGx[398]*nmpcWorkspace.QDy[103] + nmpcWorkspace.evGx[402]*nmpcWorkspace.QDy[104] + nmpcWorkspace.evGx[406]*nmpcWorkspace.QDy[105] + nmpcWorkspace.evGx[410]*nmpcWorkspace.QDy[106] + nmpcWorkspace.evGx[414]*nmpcWorkspace.QDy[107] + nmpcWorkspace.evGx[418]*nmpcWorkspace.QDy[108] + nmpcWorkspace.evGx[422]*nmpcWorkspace.QDy[109] + nmpcWorkspace.evGx[426]*nmpcWorkspace.QDy[110] + nmpcWorkspace.evGx[430]*nmpcWorkspace.QDy[111] + nmpcWorkspace.evGx[434]*nmpcWorkspace.QDy[112] + nmpcWorkspace.evGx[438]*nmpcWorkspace.QDy[113] + nmpcWorkspace.evGx[442]*nmpcWorkspace.QDy[114] + nmpcWorkspace.evGx[446]*nmpcWorkspace.QDy[115] + nmpcWorkspace.evGx[450]*nmpcWorkspace.QDy[116] + nmpcWorkspace.evGx[454]*nmpcWorkspace.QDy[117] + nmpcWorkspace.evGx[458]*nmpcWorkspace.QDy[118] + nmpcWorkspace.evGx[462]*nmpcWorkspace.QDy[119] + nmpcWorkspace.evGx[466]*nmpcWorkspace.QDy[120] + nmpcWorkspace.evGx[470]*nmpcWorkspace.QDy[121] + nmpcWorkspace.evGx[474]*nmpcWorkspace.QDy[122] + nmpcWorkspace.evGx[478]*nmpcWorkspace.QDy[123];
nmpcWorkspace.g[3] = + nmpcWorkspace.evGx[3]*nmpcWorkspace.QDy[4] + nmpcWorkspace.evGx[7]*nmpcWorkspace.QDy[5] + nmpcWorkspace.evGx[11]*nmpcWorkspace.QDy[6] + nmpcWorkspace.evGx[15]*nmpcWorkspace.QDy[7] + nmpcWorkspace.evGx[19]*nmpcWorkspace.QDy[8] + nmpcWorkspace.evGx[23]*nmpcWorkspace.QDy[9] + nmpcWorkspace.evGx[27]*nmpcWorkspace.QDy[10] + nmpcWorkspace.evGx[31]*nmpcWorkspace.QDy[11] + nmpcWorkspace.evGx[35]*nmpcWorkspace.QDy[12] + nmpcWorkspace.evGx[39]*nmpcWorkspace.QDy[13] + nmpcWorkspace.evGx[43]*nmpcWorkspace.QDy[14] + nmpcWorkspace.evGx[47]*nmpcWorkspace.QDy[15] + nmpcWorkspace.evGx[51]*nmpcWorkspace.QDy[16] + nmpcWorkspace.evGx[55]*nmpcWorkspace.QDy[17] + nmpcWorkspace.evGx[59]*nmpcWorkspace.QDy[18] + nmpcWorkspace.evGx[63]*nmpcWorkspace.QDy[19] + nmpcWorkspace.evGx[67]*nmpcWorkspace.QDy[20] + nmpcWorkspace.evGx[71]*nmpcWorkspace.QDy[21] + nmpcWorkspace.evGx[75]*nmpcWorkspace.QDy[22] + nmpcWorkspace.evGx[79]*nmpcWorkspace.QDy[23] + nmpcWorkspace.evGx[83]*nmpcWorkspace.QDy[24] + nmpcWorkspace.evGx[87]*nmpcWorkspace.QDy[25] + nmpcWorkspace.evGx[91]*nmpcWorkspace.QDy[26] + nmpcWorkspace.evGx[95]*nmpcWorkspace.QDy[27] + nmpcWorkspace.evGx[99]*nmpcWorkspace.QDy[28] + nmpcWorkspace.evGx[103]*nmpcWorkspace.QDy[29] + nmpcWorkspace.evGx[107]*nmpcWorkspace.QDy[30] + nmpcWorkspace.evGx[111]*nmpcWorkspace.QDy[31] + nmpcWorkspace.evGx[115]*nmpcWorkspace.QDy[32] + nmpcWorkspace.evGx[119]*nmpcWorkspace.QDy[33] + nmpcWorkspace.evGx[123]*nmpcWorkspace.QDy[34] + nmpcWorkspace.evGx[127]*nmpcWorkspace.QDy[35] + nmpcWorkspace.evGx[131]*nmpcWorkspace.QDy[36] + nmpcWorkspace.evGx[135]*nmpcWorkspace.QDy[37] + nmpcWorkspace.evGx[139]*nmpcWorkspace.QDy[38] + nmpcWorkspace.evGx[143]*nmpcWorkspace.QDy[39] + nmpcWorkspace.evGx[147]*nmpcWorkspace.QDy[40] + nmpcWorkspace.evGx[151]*nmpcWorkspace.QDy[41] + nmpcWorkspace.evGx[155]*nmpcWorkspace.QDy[42] + nmpcWorkspace.evGx[159]*nmpcWorkspace.QDy[43] + nmpcWorkspace.evGx[163]*nmpcWorkspace.QDy[44] + nmpcWorkspace.evGx[167]*nmpcWorkspace.QDy[45] + nmpcWorkspace.evGx[171]*nmpcWorkspace.QDy[46] + nmpcWorkspace.evGx[175]*nmpcWorkspace.QDy[47] + nmpcWorkspace.evGx[179]*nmpcWorkspace.QDy[48] + nmpcWorkspace.evGx[183]*nmpcWorkspace.QDy[49] + nmpcWorkspace.evGx[187]*nmpcWorkspace.QDy[50] + nmpcWorkspace.evGx[191]*nmpcWorkspace.QDy[51] + nmpcWorkspace.evGx[195]*nmpcWorkspace.QDy[52] + nmpcWorkspace.evGx[199]*nmpcWorkspace.QDy[53] + nmpcWorkspace.evGx[203]*nmpcWorkspace.QDy[54] + nmpcWorkspace.evGx[207]*nmpcWorkspace.QDy[55] + nmpcWorkspace.evGx[211]*nmpcWorkspace.QDy[56] + nmpcWorkspace.evGx[215]*nmpcWorkspace.QDy[57] + nmpcWorkspace.evGx[219]*nmpcWorkspace.QDy[58] + nmpcWorkspace.evGx[223]*nmpcWorkspace.QDy[59] + nmpcWorkspace.evGx[227]*nmpcWorkspace.QDy[60] + nmpcWorkspace.evGx[231]*nmpcWorkspace.QDy[61] + nmpcWorkspace.evGx[235]*nmpcWorkspace.QDy[62] + nmpcWorkspace.evGx[239]*nmpcWorkspace.QDy[63] + nmpcWorkspace.evGx[243]*nmpcWorkspace.QDy[64] + nmpcWorkspace.evGx[247]*nmpcWorkspace.QDy[65] + nmpcWorkspace.evGx[251]*nmpcWorkspace.QDy[66] + nmpcWorkspace.evGx[255]*nmpcWorkspace.QDy[67] + nmpcWorkspace.evGx[259]*nmpcWorkspace.QDy[68] + nmpcWorkspace.evGx[263]*nmpcWorkspace.QDy[69] + nmpcWorkspace.evGx[267]*nmpcWorkspace.QDy[70] + nmpcWorkspace.evGx[271]*nmpcWorkspace.QDy[71] + nmpcWorkspace.evGx[275]*nmpcWorkspace.QDy[72] + nmpcWorkspace.evGx[279]*nmpcWorkspace.QDy[73] + nmpcWorkspace.evGx[283]*nmpcWorkspace.QDy[74] + nmpcWorkspace.evGx[287]*nmpcWorkspace.QDy[75] + nmpcWorkspace.evGx[291]*nmpcWorkspace.QDy[76] + nmpcWorkspace.evGx[295]*nmpcWorkspace.QDy[77] + nmpcWorkspace.evGx[299]*nmpcWorkspace.QDy[78] + nmpcWorkspace.evGx[303]*nmpcWorkspace.QDy[79] + nmpcWorkspace.evGx[307]*nmpcWorkspace.QDy[80] + nmpcWorkspace.evGx[311]*nmpcWorkspace.QDy[81] + nmpcWorkspace.evGx[315]*nmpcWorkspace.QDy[82] + nmpcWorkspace.evGx[319]*nmpcWorkspace.QDy[83] + nmpcWorkspace.evGx[323]*nmpcWorkspace.QDy[84] + nmpcWorkspace.evGx[327]*nmpcWorkspace.QDy[85] + nmpcWorkspace.evGx[331]*nmpcWorkspace.QDy[86] + nmpcWorkspace.evGx[335]*nmpcWorkspace.QDy[87] + nmpcWorkspace.evGx[339]*nmpcWorkspace.QDy[88] + nmpcWorkspace.evGx[343]*nmpcWorkspace.QDy[89] + nmpcWorkspace.evGx[347]*nmpcWorkspace.QDy[90] + nmpcWorkspace.evGx[351]*nmpcWorkspace.QDy[91] + nmpcWorkspace.evGx[355]*nmpcWorkspace.QDy[92] + nmpcWorkspace.evGx[359]*nmpcWorkspace.QDy[93] + nmpcWorkspace.evGx[363]*nmpcWorkspace.QDy[94] + nmpcWorkspace.evGx[367]*nmpcWorkspace.QDy[95] + nmpcWorkspace.evGx[371]*nmpcWorkspace.QDy[96] + nmpcWorkspace.evGx[375]*nmpcWorkspace.QDy[97] + nmpcWorkspace.evGx[379]*nmpcWorkspace.QDy[98] + nmpcWorkspace.evGx[383]*nmpcWorkspace.QDy[99] + nmpcWorkspace.evGx[387]*nmpcWorkspace.QDy[100] + nmpcWorkspace.evGx[391]*nmpcWorkspace.QDy[101] + nmpcWorkspace.evGx[395]*nmpcWorkspace.QDy[102] + nmpcWorkspace.evGx[399]*nmpcWorkspace.QDy[103] + nmpcWorkspace.evGx[403]*nmpcWorkspace.QDy[104] + nmpcWorkspace.evGx[407]*nmpcWorkspace.QDy[105] + nmpcWorkspace.evGx[411]*nmpcWorkspace.QDy[106] + nmpcWorkspace.evGx[415]*nmpcWorkspace.QDy[107] + nmpcWorkspace.evGx[419]*nmpcWorkspace.QDy[108] + nmpcWorkspace.evGx[423]*nmpcWorkspace.QDy[109] + nmpcWorkspace.evGx[427]*nmpcWorkspace.QDy[110] + nmpcWorkspace.evGx[431]*nmpcWorkspace.QDy[111] + nmpcWorkspace.evGx[435]*nmpcWorkspace.QDy[112] + nmpcWorkspace.evGx[439]*nmpcWorkspace.QDy[113] + nmpcWorkspace.evGx[443]*nmpcWorkspace.QDy[114] + nmpcWorkspace.evGx[447]*nmpcWorkspace.QDy[115] + nmpcWorkspace.evGx[451]*nmpcWorkspace.QDy[116] + nmpcWorkspace.evGx[455]*nmpcWorkspace.QDy[117] + nmpcWorkspace.evGx[459]*nmpcWorkspace.QDy[118] + nmpcWorkspace.evGx[463]*nmpcWorkspace.QDy[119] + nmpcWorkspace.evGx[467]*nmpcWorkspace.QDy[120] + nmpcWorkspace.evGx[471]*nmpcWorkspace.QDy[121] + nmpcWorkspace.evGx[475]*nmpcWorkspace.QDy[122] + nmpcWorkspace.evGx[479]*nmpcWorkspace.QDy[123];


for (lRun1 = 0; lRun1 < 30; ++lRun1)
{
for (lRun2 = lRun1; lRun2 < 30; ++lRun2)
{
lRun3 = (((lRun2 + 1) * (lRun2)) / (2)) + (lRun1);
nmpc_multEQDy( &(nmpcWorkspace.E[ lRun3 * 8 ]), &(nmpcWorkspace.QDy[ lRun2 * 4 + 4 ]), &(nmpcWorkspace.g[ lRun1 * 2 + 4 ]) );
}
}

nmpcWorkspace.lb[0] = nmpcWorkspace.Dx0[0];
nmpcWorkspace.lb[1] = nmpcWorkspace.Dx0[1];
nmpcWorkspace.lb[2] = nmpcWorkspace.Dx0[2];
nmpcWorkspace.lb[3] = nmpcWorkspace.Dx0[3];
nmpcWorkspace.ub[0] = nmpcWorkspace.Dx0[0];
nmpcWorkspace.ub[1] = nmpcWorkspace.Dx0[1];
nmpcWorkspace.ub[2] = nmpcWorkspace.Dx0[2];
nmpcWorkspace.ub[3] = nmpcWorkspace.Dx0[3];
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

nmpcVariables.u[0] += nmpcWorkspace.x[4];
nmpcVariables.u[1] += nmpcWorkspace.x[5];
nmpcVariables.u[2] += nmpcWorkspace.x[6];
nmpcVariables.u[3] += nmpcWorkspace.x[7];
nmpcVariables.u[4] += nmpcWorkspace.x[8];
nmpcVariables.u[5] += nmpcWorkspace.x[9];
nmpcVariables.u[6] += nmpcWorkspace.x[10];
nmpcVariables.u[7] += nmpcWorkspace.x[11];
nmpcVariables.u[8] += nmpcWorkspace.x[12];
nmpcVariables.u[9] += nmpcWorkspace.x[13];
nmpcVariables.u[10] += nmpcWorkspace.x[14];
nmpcVariables.u[11] += nmpcWorkspace.x[15];
nmpcVariables.u[12] += nmpcWorkspace.x[16];
nmpcVariables.u[13] += nmpcWorkspace.x[17];
nmpcVariables.u[14] += nmpcWorkspace.x[18];
nmpcVariables.u[15] += nmpcWorkspace.x[19];
nmpcVariables.u[16] += nmpcWorkspace.x[20];
nmpcVariables.u[17] += nmpcWorkspace.x[21];
nmpcVariables.u[18] += nmpcWorkspace.x[22];
nmpcVariables.u[19] += nmpcWorkspace.x[23];
nmpcVariables.u[20] += nmpcWorkspace.x[24];
nmpcVariables.u[21] += nmpcWorkspace.x[25];
nmpcVariables.u[22] += nmpcWorkspace.x[26];
nmpcVariables.u[23] += nmpcWorkspace.x[27];
nmpcVariables.u[24] += nmpcWorkspace.x[28];
nmpcVariables.u[25] += nmpcWorkspace.x[29];
nmpcVariables.u[26] += nmpcWorkspace.x[30];
nmpcVariables.u[27] += nmpcWorkspace.x[31];
nmpcVariables.u[28] += nmpcWorkspace.x[32];
nmpcVariables.u[29] += nmpcWorkspace.x[33];
nmpcVariables.u[30] += nmpcWorkspace.x[34];
nmpcVariables.u[31] += nmpcWorkspace.x[35];
nmpcVariables.u[32] += nmpcWorkspace.x[36];
nmpcVariables.u[33] += nmpcWorkspace.x[37];
nmpcVariables.u[34] += nmpcWorkspace.x[38];
nmpcVariables.u[35] += nmpcWorkspace.x[39];
nmpcVariables.u[36] += nmpcWorkspace.x[40];
nmpcVariables.u[37] += nmpcWorkspace.x[41];
nmpcVariables.u[38] += nmpcWorkspace.x[42];
nmpcVariables.u[39] += nmpcWorkspace.x[43];
nmpcVariables.u[40] += nmpcWorkspace.x[44];
nmpcVariables.u[41] += nmpcWorkspace.x[45];
nmpcVariables.u[42] += nmpcWorkspace.x[46];
nmpcVariables.u[43] += nmpcWorkspace.x[47];
nmpcVariables.u[44] += nmpcWorkspace.x[48];
nmpcVariables.u[45] += nmpcWorkspace.x[49];
nmpcVariables.u[46] += nmpcWorkspace.x[50];
nmpcVariables.u[47] += nmpcWorkspace.x[51];
nmpcVariables.u[48] += nmpcWorkspace.x[52];
nmpcVariables.u[49] += nmpcWorkspace.x[53];
nmpcVariables.u[50] += nmpcWorkspace.x[54];
nmpcVariables.u[51] += nmpcWorkspace.x[55];
nmpcVariables.u[52] += nmpcWorkspace.x[56];
nmpcVariables.u[53] += nmpcWorkspace.x[57];
nmpcVariables.u[54] += nmpcWorkspace.x[58];
nmpcVariables.u[55] += nmpcWorkspace.x[59];
nmpcVariables.u[56] += nmpcWorkspace.x[60];
nmpcVariables.u[57] += nmpcWorkspace.x[61];
nmpcVariables.u[58] += nmpcWorkspace.x[62];
nmpcVariables.u[59] += nmpcWorkspace.x[63];

nmpcVariables.x[4] += + nmpcWorkspace.evGx[0]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[1]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[2]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[3]*nmpcWorkspace.x[3] + nmpcWorkspace.d[0];
nmpcVariables.x[5] += + nmpcWorkspace.evGx[4]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[5]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[6]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[7]*nmpcWorkspace.x[3] + nmpcWorkspace.d[1];
nmpcVariables.x[6] += + nmpcWorkspace.evGx[8]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[9]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[10]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[11]*nmpcWorkspace.x[3] + nmpcWorkspace.d[2];
nmpcVariables.x[7] += + nmpcWorkspace.evGx[12]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[13]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[14]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[15]*nmpcWorkspace.x[3] + nmpcWorkspace.d[3];
nmpcVariables.x[8] += + nmpcWorkspace.evGx[16]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[17]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[18]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[19]*nmpcWorkspace.x[3] + nmpcWorkspace.d[4];
nmpcVariables.x[9] += + nmpcWorkspace.evGx[20]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[21]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[22]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[23]*nmpcWorkspace.x[3] + nmpcWorkspace.d[5];
nmpcVariables.x[10] += + nmpcWorkspace.evGx[24]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[25]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[26]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[27]*nmpcWorkspace.x[3] + nmpcWorkspace.d[6];
nmpcVariables.x[11] += + nmpcWorkspace.evGx[28]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[29]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[30]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[31]*nmpcWorkspace.x[3] + nmpcWorkspace.d[7];
nmpcVariables.x[12] += + nmpcWorkspace.evGx[32]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[33]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[34]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[35]*nmpcWorkspace.x[3] + nmpcWorkspace.d[8];
nmpcVariables.x[13] += + nmpcWorkspace.evGx[36]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[37]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[38]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[39]*nmpcWorkspace.x[3] + nmpcWorkspace.d[9];
nmpcVariables.x[14] += + nmpcWorkspace.evGx[40]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[41]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[42]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[43]*nmpcWorkspace.x[3] + nmpcWorkspace.d[10];
nmpcVariables.x[15] += + nmpcWorkspace.evGx[44]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[45]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[46]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[47]*nmpcWorkspace.x[3] + nmpcWorkspace.d[11];
nmpcVariables.x[16] += + nmpcWorkspace.evGx[48]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[49]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[50]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[51]*nmpcWorkspace.x[3] + nmpcWorkspace.d[12];
nmpcVariables.x[17] += + nmpcWorkspace.evGx[52]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[53]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[54]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[55]*nmpcWorkspace.x[3] + nmpcWorkspace.d[13];
nmpcVariables.x[18] += + nmpcWorkspace.evGx[56]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[57]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[58]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[59]*nmpcWorkspace.x[3] + nmpcWorkspace.d[14];
nmpcVariables.x[19] += + nmpcWorkspace.evGx[60]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[61]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[62]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[63]*nmpcWorkspace.x[3] + nmpcWorkspace.d[15];
nmpcVariables.x[20] += + nmpcWorkspace.evGx[64]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[65]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[66]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[67]*nmpcWorkspace.x[3] + nmpcWorkspace.d[16];
nmpcVariables.x[21] += + nmpcWorkspace.evGx[68]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[69]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[70]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[71]*nmpcWorkspace.x[3] + nmpcWorkspace.d[17];
nmpcVariables.x[22] += + nmpcWorkspace.evGx[72]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[73]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[74]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[75]*nmpcWorkspace.x[3] + nmpcWorkspace.d[18];
nmpcVariables.x[23] += + nmpcWorkspace.evGx[76]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[77]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[78]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[79]*nmpcWorkspace.x[3] + nmpcWorkspace.d[19];
nmpcVariables.x[24] += + nmpcWorkspace.evGx[80]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[81]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[82]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[83]*nmpcWorkspace.x[3] + nmpcWorkspace.d[20];
nmpcVariables.x[25] += + nmpcWorkspace.evGx[84]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[85]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[86]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[87]*nmpcWorkspace.x[3] + nmpcWorkspace.d[21];
nmpcVariables.x[26] += + nmpcWorkspace.evGx[88]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[89]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[90]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[91]*nmpcWorkspace.x[3] + nmpcWorkspace.d[22];
nmpcVariables.x[27] += + nmpcWorkspace.evGx[92]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[93]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[94]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[95]*nmpcWorkspace.x[3] + nmpcWorkspace.d[23];
nmpcVariables.x[28] += + nmpcWorkspace.evGx[96]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[97]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[98]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[99]*nmpcWorkspace.x[3] + nmpcWorkspace.d[24];
nmpcVariables.x[29] += + nmpcWorkspace.evGx[100]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[101]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[102]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[103]*nmpcWorkspace.x[3] + nmpcWorkspace.d[25];
nmpcVariables.x[30] += + nmpcWorkspace.evGx[104]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[105]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[106]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[107]*nmpcWorkspace.x[3] + nmpcWorkspace.d[26];
nmpcVariables.x[31] += + nmpcWorkspace.evGx[108]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[109]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[110]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[111]*nmpcWorkspace.x[3] + nmpcWorkspace.d[27];
nmpcVariables.x[32] += + nmpcWorkspace.evGx[112]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[113]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[114]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[115]*nmpcWorkspace.x[3] + nmpcWorkspace.d[28];
nmpcVariables.x[33] += + nmpcWorkspace.evGx[116]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[117]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[118]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[119]*nmpcWorkspace.x[3] + nmpcWorkspace.d[29];
nmpcVariables.x[34] += + nmpcWorkspace.evGx[120]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[121]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[122]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[123]*nmpcWorkspace.x[3] + nmpcWorkspace.d[30];
nmpcVariables.x[35] += + nmpcWorkspace.evGx[124]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[125]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[126]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[127]*nmpcWorkspace.x[3] + nmpcWorkspace.d[31];
nmpcVariables.x[36] += + nmpcWorkspace.evGx[128]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[129]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[130]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[131]*nmpcWorkspace.x[3] + nmpcWorkspace.d[32];
nmpcVariables.x[37] += + nmpcWorkspace.evGx[132]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[133]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[134]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[135]*nmpcWorkspace.x[3] + nmpcWorkspace.d[33];
nmpcVariables.x[38] += + nmpcWorkspace.evGx[136]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[137]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[138]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[139]*nmpcWorkspace.x[3] + nmpcWorkspace.d[34];
nmpcVariables.x[39] += + nmpcWorkspace.evGx[140]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[141]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[142]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[143]*nmpcWorkspace.x[3] + nmpcWorkspace.d[35];
nmpcVariables.x[40] += + nmpcWorkspace.evGx[144]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[145]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[146]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[147]*nmpcWorkspace.x[3] + nmpcWorkspace.d[36];
nmpcVariables.x[41] += + nmpcWorkspace.evGx[148]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[149]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[150]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[151]*nmpcWorkspace.x[3] + nmpcWorkspace.d[37];
nmpcVariables.x[42] += + nmpcWorkspace.evGx[152]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[153]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[154]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[155]*nmpcWorkspace.x[3] + nmpcWorkspace.d[38];
nmpcVariables.x[43] += + nmpcWorkspace.evGx[156]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[157]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[158]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[159]*nmpcWorkspace.x[3] + nmpcWorkspace.d[39];
nmpcVariables.x[44] += + nmpcWorkspace.evGx[160]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[161]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[162]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[163]*nmpcWorkspace.x[3] + nmpcWorkspace.d[40];
nmpcVariables.x[45] += + nmpcWorkspace.evGx[164]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[165]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[166]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[167]*nmpcWorkspace.x[3] + nmpcWorkspace.d[41];
nmpcVariables.x[46] += + nmpcWorkspace.evGx[168]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[169]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[170]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[171]*nmpcWorkspace.x[3] + nmpcWorkspace.d[42];
nmpcVariables.x[47] += + nmpcWorkspace.evGx[172]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[173]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[174]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[175]*nmpcWorkspace.x[3] + nmpcWorkspace.d[43];
nmpcVariables.x[48] += + nmpcWorkspace.evGx[176]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[177]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[178]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[179]*nmpcWorkspace.x[3] + nmpcWorkspace.d[44];
nmpcVariables.x[49] += + nmpcWorkspace.evGx[180]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[181]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[182]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[183]*nmpcWorkspace.x[3] + nmpcWorkspace.d[45];
nmpcVariables.x[50] += + nmpcWorkspace.evGx[184]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[185]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[186]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[187]*nmpcWorkspace.x[3] + nmpcWorkspace.d[46];
nmpcVariables.x[51] += + nmpcWorkspace.evGx[188]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[189]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[190]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[191]*nmpcWorkspace.x[3] + nmpcWorkspace.d[47];
nmpcVariables.x[52] += + nmpcWorkspace.evGx[192]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[193]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[194]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[195]*nmpcWorkspace.x[3] + nmpcWorkspace.d[48];
nmpcVariables.x[53] += + nmpcWorkspace.evGx[196]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[197]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[198]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[199]*nmpcWorkspace.x[3] + nmpcWorkspace.d[49];
nmpcVariables.x[54] += + nmpcWorkspace.evGx[200]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[201]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[202]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[203]*nmpcWorkspace.x[3] + nmpcWorkspace.d[50];
nmpcVariables.x[55] += + nmpcWorkspace.evGx[204]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[205]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[206]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[207]*nmpcWorkspace.x[3] + nmpcWorkspace.d[51];
nmpcVariables.x[56] += + nmpcWorkspace.evGx[208]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[209]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[210]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[211]*nmpcWorkspace.x[3] + nmpcWorkspace.d[52];
nmpcVariables.x[57] += + nmpcWorkspace.evGx[212]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[213]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[214]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[215]*nmpcWorkspace.x[3] + nmpcWorkspace.d[53];
nmpcVariables.x[58] += + nmpcWorkspace.evGx[216]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[217]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[218]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[219]*nmpcWorkspace.x[3] + nmpcWorkspace.d[54];
nmpcVariables.x[59] += + nmpcWorkspace.evGx[220]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[221]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[222]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[223]*nmpcWorkspace.x[3] + nmpcWorkspace.d[55];
nmpcVariables.x[60] += + nmpcWorkspace.evGx[224]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[225]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[226]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[227]*nmpcWorkspace.x[3] + nmpcWorkspace.d[56];
nmpcVariables.x[61] += + nmpcWorkspace.evGx[228]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[229]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[230]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[231]*nmpcWorkspace.x[3] + nmpcWorkspace.d[57];
nmpcVariables.x[62] += + nmpcWorkspace.evGx[232]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[233]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[234]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[235]*nmpcWorkspace.x[3] + nmpcWorkspace.d[58];
nmpcVariables.x[63] += + nmpcWorkspace.evGx[236]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[237]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[238]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[239]*nmpcWorkspace.x[3] + nmpcWorkspace.d[59];
nmpcVariables.x[64] += + nmpcWorkspace.evGx[240]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[241]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[242]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[243]*nmpcWorkspace.x[3] + nmpcWorkspace.d[60];
nmpcVariables.x[65] += + nmpcWorkspace.evGx[244]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[245]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[246]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[247]*nmpcWorkspace.x[3] + nmpcWorkspace.d[61];
nmpcVariables.x[66] += + nmpcWorkspace.evGx[248]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[249]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[250]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[251]*nmpcWorkspace.x[3] + nmpcWorkspace.d[62];
nmpcVariables.x[67] += + nmpcWorkspace.evGx[252]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[253]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[254]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[255]*nmpcWorkspace.x[3] + nmpcWorkspace.d[63];
nmpcVariables.x[68] += + nmpcWorkspace.evGx[256]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[257]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[258]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[259]*nmpcWorkspace.x[3] + nmpcWorkspace.d[64];
nmpcVariables.x[69] += + nmpcWorkspace.evGx[260]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[261]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[262]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[263]*nmpcWorkspace.x[3] + nmpcWorkspace.d[65];
nmpcVariables.x[70] += + nmpcWorkspace.evGx[264]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[265]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[266]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[267]*nmpcWorkspace.x[3] + nmpcWorkspace.d[66];
nmpcVariables.x[71] += + nmpcWorkspace.evGx[268]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[269]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[270]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[271]*nmpcWorkspace.x[3] + nmpcWorkspace.d[67];
nmpcVariables.x[72] += + nmpcWorkspace.evGx[272]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[273]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[274]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[275]*nmpcWorkspace.x[3] + nmpcWorkspace.d[68];
nmpcVariables.x[73] += + nmpcWorkspace.evGx[276]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[277]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[278]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[279]*nmpcWorkspace.x[3] + nmpcWorkspace.d[69];
nmpcVariables.x[74] += + nmpcWorkspace.evGx[280]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[281]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[282]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[283]*nmpcWorkspace.x[3] + nmpcWorkspace.d[70];
nmpcVariables.x[75] += + nmpcWorkspace.evGx[284]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[285]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[286]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[287]*nmpcWorkspace.x[3] + nmpcWorkspace.d[71];
nmpcVariables.x[76] += + nmpcWorkspace.evGx[288]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[289]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[290]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[291]*nmpcWorkspace.x[3] + nmpcWorkspace.d[72];
nmpcVariables.x[77] += + nmpcWorkspace.evGx[292]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[293]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[294]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[295]*nmpcWorkspace.x[3] + nmpcWorkspace.d[73];
nmpcVariables.x[78] += + nmpcWorkspace.evGx[296]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[297]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[298]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[299]*nmpcWorkspace.x[3] + nmpcWorkspace.d[74];
nmpcVariables.x[79] += + nmpcWorkspace.evGx[300]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[301]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[302]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[303]*nmpcWorkspace.x[3] + nmpcWorkspace.d[75];
nmpcVariables.x[80] += + nmpcWorkspace.evGx[304]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[305]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[306]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[307]*nmpcWorkspace.x[3] + nmpcWorkspace.d[76];
nmpcVariables.x[81] += + nmpcWorkspace.evGx[308]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[309]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[310]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[311]*nmpcWorkspace.x[3] + nmpcWorkspace.d[77];
nmpcVariables.x[82] += + nmpcWorkspace.evGx[312]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[313]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[314]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[315]*nmpcWorkspace.x[3] + nmpcWorkspace.d[78];
nmpcVariables.x[83] += + nmpcWorkspace.evGx[316]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[317]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[318]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[319]*nmpcWorkspace.x[3] + nmpcWorkspace.d[79];
nmpcVariables.x[84] += + nmpcWorkspace.evGx[320]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[321]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[322]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[323]*nmpcWorkspace.x[3] + nmpcWorkspace.d[80];
nmpcVariables.x[85] += + nmpcWorkspace.evGx[324]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[325]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[326]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[327]*nmpcWorkspace.x[3] + nmpcWorkspace.d[81];
nmpcVariables.x[86] += + nmpcWorkspace.evGx[328]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[329]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[330]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[331]*nmpcWorkspace.x[3] + nmpcWorkspace.d[82];
nmpcVariables.x[87] += + nmpcWorkspace.evGx[332]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[333]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[334]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[335]*nmpcWorkspace.x[3] + nmpcWorkspace.d[83];
nmpcVariables.x[88] += + nmpcWorkspace.evGx[336]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[337]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[338]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[339]*nmpcWorkspace.x[3] + nmpcWorkspace.d[84];
nmpcVariables.x[89] += + nmpcWorkspace.evGx[340]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[341]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[342]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[343]*nmpcWorkspace.x[3] + nmpcWorkspace.d[85];
nmpcVariables.x[90] += + nmpcWorkspace.evGx[344]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[345]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[346]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[347]*nmpcWorkspace.x[3] + nmpcWorkspace.d[86];
nmpcVariables.x[91] += + nmpcWorkspace.evGx[348]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[349]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[350]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[351]*nmpcWorkspace.x[3] + nmpcWorkspace.d[87];
nmpcVariables.x[92] += + nmpcWorkspace.evGx[352]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[353]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[354]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[355]*nmpcWorkspace.x[3] + nmpcWorkspace.d[88];
nmpcVariables.x[93] += + nmpcWorkspace.evGx[356]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[357]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[358]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[359]*nmpcWorkspace.x[3] + nmpcWorkspace.d[89];
nmpcVariables.x[94] += + nmpcWorkspace.evGx[360]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[361]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[362]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[363]*nmpcWorkspace.x[3] + nmpcWorkspace.d[90];
nmpcVariables.x[95] += + nmpcWorkspace.evGx[364]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[365]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[366]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[367]*nmpcWorkspace.x[3] + nmpcWorkspace.d[91];
nmpcVariables.x[96] += + nmpcWorkspace.evGx[368]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[369]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[370]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[371]*nmpcWorkspace.x[3] + nmpcWorkspace.d[92];
nmpcVariables.x[97] += + nmpcWorkspace.evGx[372]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[373]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[374]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[375]*nmpcWorkspace.x[3] + nmpcWorkspace.d[93];
nmpcVariables.x[98] += + nmpcWorkspace.evGx[376]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[377]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[378]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[379]*nmpcWorkspace.x[3] + nmpcWorkspace.d[94];
nmpcVariables.x[99] += + nmpcWorkspace.evGx[380]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[381]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[382]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[383]*nmpcWorkspace.x[3] + nmpcWorkspace.d[95];
nmpcVariables.x[100] += + nmpcWorkspace.evGx[384]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[385]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[386]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[387]*nmpcWorkspace.x[3] + nmpcWorkspace.d[96];
nmpcVariables.x[101] += + nmpcWorkspace.evGx[388]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[389]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[390]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[391]*nmpcWorkspace.x[3] + nmpcWorkspace.d[97];
nmpcVariables.x[102] += + nmpcWorkspace.evGx[392]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[393]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[394]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[395]*nmpcWorkspace.x[3] + nmpcWorkspace.d[98];
nmpcVariables.x[103] += + nmpcWorkspace.evGx[396]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[397]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[398]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[399]*nmpcWorkspace.x[3] + nmpcWorkspace.d[99];
nmpcVariables.x[104] += + nmpcWorkspace.evGx[400]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[401]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[402]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[403]*nmpcWorkspace.x[3] + nmpcWorkspace.d[100];
nmpcVariables.x[105] += + nmpcWorkspace.evGx[404]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[405]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[406]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[407]*nmpcWorkspace.x[3] + nmpcWorkspace.d[101];
nmpcVariables.x[106] += + nmpcWorkspace.evGx[408]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[409]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[410]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[411]*nmpcWorkspace.x[3] + nmpcWorkspace.d[102];
nmpcVariables.x[107] += + nmpcWorkspace.evGx[412]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[413]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[414]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[415]*nmpcWorkspace.x[3] + nmpcWorkspace.d[103];
nmpcVariables.x[108] += + nmpcWorkspace.evGx[416]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[417]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[418]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[419]*nmpcWorkspace.x[3] + nmpcWorkspace.d[104];
nmpcVariables.x[109] += + nmpcWorkspace.evGx[420]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[421]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[422]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[423]*nmpcWorkspace.x[3] + nmpcWorkspace.d[105];
nmpcVariables.x[110] += + nmpcWorkspace.evGx[424]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[425]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[426]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[427]*nmpcWorkspace.x[3] + nmpcWorkspace.d[106];
nmpcVariables.x[111] += + nmpcWorkspace.evGx[428]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[429]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[430]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[431]*nmpcWorkspace.x[3] + nmpcWorkspace.d[107];
nmpcVariables.x[112] += + nmpcWorkspace.evGx[432]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[433]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[434]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[435]*nmpcWorkspace.x[3] + nmpcWorkspace.d[108];
nmpcVariables.x[113] += + nmpcWorkspace.evGx[436]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[437]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[438]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[439]*nmpcWorkspace.x[3] + nmpcWorkspace.d[109];
nmpcVariables.x[114] += + nmpcWorkspace.evGx[440]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[441]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[442]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[443]*nmpcWorkspace.x[3] + nmpcWorkspace.d[110];
nmpcVariables.x[115] += + nmpcWorkspace.evGx[444]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[445]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[446]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[447]*nmpcWorkspace.x[3] + nmpcWorkspace.d[111];
nmpcVariables.x[116] += + nmpcWorkspace.evGx[448]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[449]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[450]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[451]*nmpcWorkspace.x[3] + nmpcWorkspace.d[112];
nmpcVariables.x[117] += + nmpcWorkspace.evGx[452]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[453]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[454]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[455]*nmpcWorkspace.x[3] + nmpcWorkspace.d[113];
nmpcVariables.x[118] += + nmpcWorkspace.evGx[456]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[457]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[458]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[459]*nmpcWorkspace.x[3] + nmpcWorkspace.d[114];
nmpcVariables.x[119] += + nmpcWorkspace.evGx[460]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[461]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[462]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[463]*nmpcWorkspace.x[3] + nmpcWorkspace.d[115];
nmpcVariables.x[120] += + nmpcWorkspace.evGx[464]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[465]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[466]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[467]*nmpcWorkspace.x[3] + nmpcWorkspace.d[116];
nmpcVariables.x[121] += + nmpcWorkspace.evGx[468]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[469]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[470]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[471]*nmpcWorkspace.x[3] + nmpcWorkspace.d[117];
nmpcVariables.x[122] += + nmpcWorkspace.evGx[472]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[473]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[474]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[475]*nmpcWorkspace.x[3] + nmpcWorkspace.d[118];
nmpcVariables.x[123] += + nmpcWorkspace.evGx[476]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[477]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[478]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[479]*nmpcWorkspace.x[3] + nmpcWorkspace.d[119];

for (lRun1 = 0; lRun1 < 30; ++lRun1)
{
for (lRun2 = 0; lRun2 < lRun1 + 1; ++lRun2)
{
lRun3 = (((lRun1 + 1) * (lRun1)) / (2)) + (lRun2);
nmpc_multEDu( &(nmpcWorkspace.E[ lRun3 * 8 ]), &(nmpcWorkspace.x[ lRun2 * 2 + 4 ]), &(nmpcVariables.x[ lRun1 * 4 + 4 ]) );
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
nmpcWorkspace.state[0] = nmpcVariables.x[index * 4];
nmpcWorkspace.state[1] = nmpcVariables.x[index * 4 + 1];
nmpcWorkspace.state[2] = nmpcVariables.x[index * 4 + 2];
nmpcWorkspace.state[3] = nmpcVariables.x[index * 4 + 3];
nmpcWorkspace.state[28] = nmpcVariables.u[index * 2];
nmpcWorkspace.state[29] = nmpcVariables.u[index * 2 + 1];
nmpcWorkspace.state[30] = nmpcVariables.od[index * 2];
nmpcWorkspace.state[31] = nmpcVariables.od[index * 2 + 1];

nmpc_integrate(nmpcWorkspace.state, index == 0);

nmpcVariables.x[index * 4 + 4] = nmpcWorkspace.state[0];
nmpcVariables.x[index * 4 + 5] = nmpcWorkspace.state[1];
nmpcVariables.x[index * 4 + 6] = nmpcWorkspace.state[2];
nmpcVariables.x[index * 4 + 7] = nmpcWorkspace.state[3];
}
}

void nmpc_shiftStates( int strategy, real_t* const xEnd, real_t* const uEnd )
{
int index;
for (index = 0; index < 30; ++index)
{
nmpcVariables.x[index * 4] = nmpcVariables.x[index * 4 + 4];
nmpcVariables.x[index * 4 + 1] = nmpcVariables.x[index * 4 + 5];
nmpcVariables.x[index * 4 + 2] = nmpcVariables.x[index * 4 + 6];
nmpcVariables.x[index * 4 + 3] = nmpcVariables.x[index * 4 + 7];
}

if (strategy == 1 && xEnd != 0)
{
nmpcVariables.x[120] = xEnd[0];
nmpcVariables.x[121] = xEnd[1];
nmpcVariables.x[122] = xEnd[2];
nmpcVariables.x[123] = xEnd[3];
}
else if (strategy == 2) 
{
nmpcWorkspace.state[0] = nmpcVariables.x[120];
nmpcWorkspace.state[1] = nmpcVariables.x[121];
nmpcWorkspace.state[2] = nmpcVariables.x[122];
nmpcWorkspace.state[3] = nmpcVariables.x[123];
if (uEnd != 0)
{
nmpcWorkspace.state[28] = uEnd[0];
nmpcWorkspace.state[29] = uEnd[1];
}
else
{
nmpcWorkspace.state[28] = nmpcVariables.u[58];
nmpcWorkspace.state[29] = nmpcVariables.u[59];
}
nmpcWorkspace.state[30] = nmpcVariables.od[60];
nmpcWorkspace.state[31] = nmpcVariables.od[61];

nmpc_integrate(nmpcWorkspace.state, 1);

nmpcVariables.x[120] = nmpcWorkspace.state[0];
nmpcVariables.x[121] = nmpcWorkspace.state[1];
nmpcVariables.x[122] = nmpcWorkspace.state[2];
nmpcVariables.x[123] = nmpcWorkspace.state[3];
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

kkt = + nmpcWorkspace.g[0]*nmpcWorkspace.x[0] + nmpcWorkspace.g[1]*nmpcWorkspace.x[1] + nmpcWorkspace.g[2]*nmpcWorkspace.x[2] + nmpcWorkspace.g[3]*nmpcWorkspace.x[3] + nmpcWorkspace.g[4]*nmpcWorkspace.x[4] + nmpcWorkspace.g[5]*nmpcWorkspace.x[5] + nmpcWorkspace.g[6]*nmpcWorkspace.x[6] + nmpcWorkspace.g[7]*nmpcWorkspace.x[7] + nmpcWorkspace.g[8]*nmpcWorkspace.x[8] + nmpcWorkspace.g[9]*nmpcWorkspace.x[9] + nmpcWorkspace.g[10]*nmpcWorkspace.x[10] + nmpcWorkspace.g[11]*nmpcWorkspace.x[11] + nmpcWorkspace.g[12]*nmpcWorkspace.x[12] + nmpcWorkspace.g[13]*nmpcWorkspace.x[13] + nmpcWorkspace.g[14]*nmpcWorkspace.x[14] + nmpcWorkspace.g[15]*nmpcWorkspace.x[15] + nmpcWorkspace.g[16]*nmpcWorkspace.x[16] + nmpcWorkspace.g[17]*nmpcWorkspace.x[17] + nmpcWorkspace.g[18]*nmpcWorkspace.x[18] + nmpcWorkspace.g[19]*nmpcWorkspace.x[19] + nmpcWorkspace.g[20]*nmpcWorkspace.x[20] + nmpcWorkspace.g[21]*nmpcWorkspace.x[21] + nmpcWorkspace.g[22]*nmpcWorkspace.x[22] + nmpcWorkspace.g[23]*nmpcWorkspace.x[23] + nmpcWorkspace.g[24]*nmpcWorkspace.x[24] + nmpcWorkspace.g[25]*nmpcWorkspace.x[25] + nmpcWorkspace.g[26]*nmpcWorkspace.x[26] + nmpcWorkspace.g[27]*nmpcWorkspace.x[27] + nmpcWorkspace.g[28]*nmpcWorkspace.x[28] + nmpcWorkspace.g[29]*nmpcWorkspace.x[29] + nmpcWorkspace.g[30]*nmpcWorkspace.x[30] + nmpcWorkspace.g[31]*nmpcWorkspace.x[31] + nmpcWorkspace.g[32]*nmpcWorkspace.x[32] + nmpcWorkspace.g[33]*nmpcWorkspace.x[33] + nmpcWorkspace.g[34]*nmpcWorkspace.x[34] + nmpcWorkspace.g[35]*nmpcWorkspace.x[35] + nmpcWorkspace.g[36]*nmpcWorkspace.x[36] + nmpcWorkspace.g[37]*nmpcWorkspace.x[37] + nmpcWorkspace.g[38]*nmpcWorkspace.x[38] + nmpcWorkspace.g[39]*nmpcWorkspace.x[39] + nmpcWorkspace.g[40]*nmpcWorkspace.x[40] + nmpcWorkspace.g[41]*nmpcWorkspace.x[41] + nmpcWorkspace.g[42]*nmpcWorkspace.x[42] + nmpcWorkspace.g[43]*nmpcWorkspace.x[43] + nmpcWorkspace.g[44]*nmpcWorkspace.x[44] + nmpcWorkspace.g[45]*nmpcWorkspace.x[45] + nmpcWorkspace.g[46]*nmpcWorkspace.x[46] + nmpcWorkspace.g[47]*nmpcWorkspace.x[47] + nmpcWorkspace.g[48]*nmpcWorkspace.x[48] + nmpcWorkspace.g[49]*nmpcWorkspace.x[49] + nmpcWorkspace.g[50]*nmpcWorkspace.x[50] + nmpcWorkspace.g[51]*nmpcWorkspace.x[51] + nmpcWorkspace.g[52]*nmpcWorkspace.x[52] + nmpcWorkspace.g[53]*nmpcWorkspace.x[53] + nmpcWorkspace.g[54]*nmpcWorkspace.x[54] + nmpcWorkspace.g[55]*nmpcWorkspace.x[55] + nmpcWorkspace.g[56]*nmpcWorkspace.x[56] + nmpcWorkspace.g[57]*nmpcWorkspace.x[57] + nmpcWorkspace.g[58]*nmpcWorkspace.x[58] + nmpcWorkspace.g[59]*nmpcWorkspace.x[59] + nmpcWorkspace.g[60]*nmpcWorkspace.x[60] + nmpcWorkspace.g[61]*nmpcWorkspace.x[61] + nmpcWorkspace.g[62]*nmpcWorkspace.x[62] + nmpcWorkspace.g[63]*nmpcWorkspace.x[63];
kkt = fabs( kkt );
for (index = 0; index < 64; ++index)
{
prd = nmpcWorkspace.y[index];
if (prd > 1e-12)
kkt += fabs(nmpcWorkspace.lb[index] * prd);
else if (prd < -1e-12)
kkt += fabs(nmpcWorkspace.ub[index] * prd);
}
return kkt;
}

real_t nmpc_getObjective(  )
{
real_t objVal;

int lRun1;
/** Row vector of size: 6 */
real_t tmpDy[ 6 ];

/** Row vector of size: 4 */
real_t tmpDyN[ 4 ];

for (lRun1 = 0; lRun1 < 30; ++lRun1)
{
nmpcWorkspace.objValueIn[0] = nmpcVariables.x[lRun1 * 4];
nmpcWorkspace.objValueIn[1] = nmpcVariables.x[lRun1 * 4 + 1];
nmpcWorkspace.objValueIn[2] = nmpcVariables.x[lRun1 * 4 + 2];
nmpcWorkspace.objValueIn[3] = nmpcVariables.x[lRun1 * 4 + 3];
nmpcWorkspace.objValueIn[4] = nmpcVariables.u[lRun1 * 2];
nmpcWorkspace.objValueIn[5] = nmpcVariables.u[lRun1 * 2 + 1];
nmpcWorkspace.objValueIn[6] = nmpcVariables.od[lRun1 * 2];
nmpcWorkspace.objValueIn[7] = nmpcVariables.od[lRun1 * 2 + 1];

nmpc_evaluateLSQ( nmpcWorkspace.objValueIn, nmpcWorkspace.objValueOut );
nmpcWorkspace.Dy[lRun1 * 6] = nmpcWorkspace.objValueOut[0] - nmpcVariables.y[lRun1 * 6];
nmpcWorkspace.Dy[lRun1 * 6 + 1] = nmpcWorkspace.objValueOut[1] - nmpcVariables.y[lRun1 * 6 + 1];
nmpcWorkspace.Dy[lRun1 * 6 + 2] = nmpcWorkspace.objValueOut[2] - nmpcVariables.y[lRun1 * 6 + 2];
nmpcWorkspace.Dy[lRun1 * 6 + 3] = nmpcWorkspace.objValueOut[3] - nmpcVariables.y[lRun1 * 6 + 3];
nmpcWorkspace.Dy[lRun1 * 6 + 4] = nmpcWorkspace.objValueOut[4] - nmpcVariables.y[lRun1 * 6 + 4];
nmpcWorkspace.Dy[lRun1 * 6 + 5] = nmpcWorkspace.objValueOut[5] - nmpcVariables.y[lRun1 * 6 + 5];
}
nmpcWorkspace.objValueIn[0] = nmpcVariables.x[120];
nmpcWorkspace.objValueIn[1] = nmpcVariables.x[121];
nmpcWorkspace.objValueIn[2] = nmpcVariables.x[122];
nmpcWorkspace.objValueIn[3] = nmpcVariables.x[123];
nmpcWorkspace.objValueIn[4] = nmpcVariables.od[60];
nmpcWorkspace.objValueIn[5] = nmpcVariables.od[61];
nmpc_evaluateLSQEndTerm( nmpcWorkspace.objValueIn, nmpcWorkspace.objValueOut );
nmpcWorkspace.DyN[0] = nmpcWorkspace.objValueOut[0] - nmpcVariables.yN[0];
nmpcWorkspace.DyN[1] = nmpcWorkspace.objValueOut[1] - nmpcVariables.yN[1];
nmpcWorkspace.DyN[2] = nmpcWorkspace.objValueOut[2] - nmpcVariables.yN[2];
nmpcWorkspace.DyN[3] = nmpcWorkspace.objValueOut[3] - nmpcVariables.yN[3];
objVal = 0.0000000000000000e+00;
for (lRun1 = 0; lRun1 < 30; ++lRun1)
{
tmpDy[0] = + nmpcWorkspace.Dy[lRun1 * 6]*nmpcVariables.W[0];
tmpDy[1] = + nmpcWorkspace.Dy[lRun1 * 6 + 1]*nmpcVariables.W[7];
tmpDy[2] = + nmpcWorkspace.Dy[lRun1 * 6 + 2]*nmpcVariables.W[14];
tmpDy[3] = + nmpcWorkspace.Dy[lRun1 * 6 + 3]*nmpcVariables.W[21];
tmpDy[4] = + nmpcWorkspace.Dy[lRun1 * 6 + 4]*nmpcVariables.W[28];
tmpDy[5] = + nmpcWorkspace.Dy[lRun1 * 6 + 5]*nmpcVariables.W[35];
objVal += + nmpcWorkspace.Dy[lRun1 * 6]*tmpDy[0] + nmpcWorkspace.Dy[lRun1 * 6 + 1]*tmpDy[1] + nmpcWorkspace.Dy[lRun1 * 6 + 2]*tmpDy[2] + nmpcWorkspace.Dy[lRun1 * 6 + 3]*tmpDy[3] + nmpcWorkspace.Dy[lRun1 * 6 + 4]*tmpDy[4] + nmpcWorkspace.Dy[lRun1 * 6 + 5]*tmpDy[5];
}

tmpDyN[0] = + nmpcWorkspace.DyN[0]*nmpcVariables.WN[0];
tmpDyN[1] = + nmpcWorkspace.DyN[1]*nmpcVariables.WN[5];
tmpDyN[2] = + nmpcWorkspace.DyN[2]*nmpcVariables.WN[10];
tmpDyN[3] = + nmpcWorkspace.DyN[3]*nmpcVariables.WN[15];
objVal += + nmpcWorkspace.DyN[0]*tmpDyN[0] + nmpcWorkspace.DyN[1]*tmpDyN[1] + nmpcWorkspace.DyN[2]*tmpDyN[2] + nmpcWorkspace.DyN[3]*tmpDyN[3];

objVal *= 0.5;
return objVal;
}

