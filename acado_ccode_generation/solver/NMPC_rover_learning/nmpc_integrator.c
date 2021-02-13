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


void nmpc_rhs_forw(const real_t* in, real_t* out)
{
const real_t* xd = in;
const real_t* u = in + 40;
const real_t* od = in + 42;
/* Vector of auxiliary variables; number of elements: 75. */
real_t* a = nmpcWorkspace.rhs_aux;

/* Compute intermediate quantities: */
a[0] = ((((od[0]*(real_t)(-2.1500000000000000e-01))*xd[3])-((od[1]*(real_t)(2.1500000000000000e-01))*xd[4]))/(real_t)(-4.2999999999999999e-01));
a[1] = (cos(xd[2]));
a[2] = (sin(xd[2]));
a[3] = (((od[0]*xd[3])-(od[1]*xd[4]))/(real_t)(-4.2999999999999999e-01));
a[4] = (sin(xd[2]));
a[5] = (cos(xd[2]));
a[6] = ((real_t)(1.0000000000000000e+00)/(real_t)(-4.2999999999999999e-01));
a[7] = ((((od[0]*(real_t)(-2.1500000000000000e-01))*xd[20])-((od[1]*(real_t)(2.1500000000000000e-01))*xd[25]))*a[6]);
a[8] = ((real_t)(-1.0000000000000000e+00)*(sin(xd[2])));
a[9] = (xd[15]*a[8]);
a[10] = (cos(xd[2]));
a[11] = (xd[15]*a[10]);
a[12] = ((real_t)(1.0000000000000000e+00)/(real_t)(-4.2999999999999999e-01));
a[13] = (((od[0]*xd[20])-(od[1]*xd[25]))*a[12]);
a[14] = ((((od[0]*(real_t)(-2.1500000000000000e-01))*xd[21])-((od[1]*(real_t)(2.1500000000000000e-01))*xd[26]))*a[6]);
a[15] = (xd[16]*a[8]);
a[16] = (xd[16]*a[10]);
a[17] = (((od[0]*xd[21])-(od[1]*xd[26]))*a[12]);
a[18] = ((((od[0]*(real_t)(-2.1500000000000000e-01))*xd[22])-((od[1]*(real_t)(2.1500000000000000e-01))*xd[27]))*a[6]);
a[19] = (xd[17]*a[8]);
a[20] = (xd[17]*a[10]);
a[21] = (((od[0]*xd[22])-(od[1]*xd[27]))*a[12]);
a[22] = ((((od[0]*(real_t)(-2.1500000000000000e-01))*xd[23])-((od[1]*(real_t)(2.1500000000000000e-01))*xd[28]))*a[6]);
a[23] = (xd[18]*a[8]);
a[24] = (xd[18]*a[10]);
a[25] = (((od[0]*xd[23])-(od[1]*xd[28]))*a[12]);
a[26] = ((((od[0]*(real_t)(-2.1500000000000000e-01))*xd[24])-((od[1]*(real_t)(2.1500000000000000e-01))*xd[29]))*a[6]);
a[27] = (xd[19]*a[8]);
a[28] = (xd[19]*a[10]);
a[29] = (((od[0]*xd[24])-(od[1]*xd[29]))*a[12]);
a[30] = ((((od[0]*(real_t)(-2.1500000000000000e-01))*xd[20])-((od[1]*(real_t)(2.1500000000000000e-01))*xd[25]))*a[6]);
a[31] = (cos(xd[2]));
a[32] = (xd[15]*a[31]);
a[33] = ((real_t)(-1.0000000000000000e+00)*(sin(xd[2])));
a[34] = (xd[15]*a[33]);
a[35] = (((od[0]*xd[20])-(od[1]*xd[25]))*a[12]);
a[36] = ((((od[0]*(real_t)(-2.1500000000000000e-01))*xd[21])-((od[1]*(real_t)(2.1500000000000000e-01))*xd[26]))*a[6]);
a[37] = (xd[16]*a[31]);
a[38] = (xd[16]*a[33]);
a[39] = (((od[0]*xd[21])-(od[1]*xd[26]))*a[12]);
a[40] = ((((od[0]*(real_t)(-2.1500000000000000e-01))*xd[22])-((od[1]*(real_t)(2.1500000000000000e-01))*xd[27]))*a[6]);
a[41] = (xd[17]*a[31]);
a[42] = (xd[17]*a[33]);
a[43] = (((od[0]*xd[22])-(od[1]*xd[27]))*a[12]);
a[44] = ((((od[0]*(real_t)(-2.1500000000000000e-01))*xd[23])-((od[1]*(real_t)(2.1500000000000000e-01))*xd[28]))*a[6]);
a[45] = (xd[18]*a[31]);
a[46] = (xd[18]*a[33]);
a[47] = (((od[0]*xd[23])-(od[1]*xd[28]))*a[12]);
a[48] = ((((od[0]*(real_t)(-2.1500000000000000e-01))*xd[24])-((od[1]*(real_t)(2.1500000000000000e-01))*xd[29]))*a[6]);
a[49] = (xd[19]*a[31]);
a[50] = (xd[19]*a[33]);
a[51] = (((od[0]*xd[24])-(od[1]*xd[29]))*a[12]);
a[52] = (((od[0]*xd[20])-(od[1]*xd[25]))*a[12]);
a[53] = (((od[0]*xd[21])-(od[1]*xd[26]))*a[12]);
a[54] = (((od[0]*xd[22])-(od[1]*xd[27]))*a[12]);
a[55] = (((od[0]*xd[23])-(od[1]*xd[28]))*a[12]);
a[56] = (((od[0]*xd[24])-(od[1]*xd[29]))*a[12]);
a[57] = ((((od[0]*(real_t)(-2.1500000000000000e-01))*xd[36])-((od[1]*(real_t)(2.1500000000000000e-01))*xd[38]))*a[6]);
a[58] = (xd[34]*a[8]);
a[59] = (xd[34]*a[10]);
a[60] = (((od[0]*xd[36])-(od[1]*xd[38]))*a[12]);
a[61] = ((((od[0]*(real_t)(-2.1500000000000000e-01))*xd[37])-((od[1]*(real_t)(2.1500000000000000e-01))*xd[39]))*a[6]);
a[62] = (xd[35]*a[8]);
a[63] = (xd[35]*a[10]);
a[64] = (((od[0]*xd[37])-(od[1]*xd[39]))*a[12]);
a[65] = ((((od[0]*(real_t)(-2.1500000000000000e-01))*xd[36])-((od[1]*(real_t)(2.1500000000000000e-01))*xd[38]))*a[6]);
a[66] = (xd[34]*a[31]);
a[67] = (xd[34]*a[33]);
a[68] = (((od[0]*xd[36])-(od[1]*xd[38]))*a[12]);
a[69] = ((((od[0]*(real_t)(-2.1500000000000000e-01))*xd[37])-((od[1]*(real_t)(2.1500000000000000e-01))*xd[39]))*a[6]);
a[70] = (xd[35]*a[31]);
a[71] = (xd[35]*a[33]);
a[72] = (((od[0]*xd[37])-(od[1]*xd[39]))*a[12]);
a[73] = (((od[0]*xd[36])-(od[1]*xd[38]))*a[12]);
a[74] = (((od[0]*xd[37])-(od[1]*xd[39]))*a[12]);

/* Compute outputs: */
out[0] = ((a[0]*a[1])+(((real_t)(2.1400000000000000e-01)*a[2])*a[3]));
out[1] = ((a[0]*a[4])-(((real_t)(2.1400000000000000e-01)*a[5])*a[3]));
out[2] = a[3];
out[3] = u[0];
out[4] = u[1];
out[5] = (((a[7]*a[1])+(a[0]*a[9]))+((((real_t)(2.1400000000000000e-01)*a[11])*a[3])+(((real_t)(2.1400000000000000e-01)*a[2])*a[13])));
out[6] = (((a[14]*a[1])+(a[0]*a[15]))+((((real_t)(2.1400000000000000e-01)*a[16])*a[3])+(((real_t)(2.1400000000000000e-01)*a[2])*a[17])));
out[7] = (((a[18]*a[1])+(a[0]*a[19]))+((((real_t)(2.1400000000000000e-01)*a[20])*a[3])+(((real_t)(2.1400000000000000e-01)*a[2])*a[21])));
out[8] = (((a[22]*a[1])+(a[0]*a[23]))+((((real_t)(2.1400000000000000e-01)*a[24])*a[3])+(((real_t)(2.1400000000000000e-01)*a[2])*a[25])));
out[9] = (((a[26]*a[1])+(a[0]*a[27]))+((((real_t)(2.1400000000000000e-01)*a[28])*a[3])+(((real_t)(2.1400000000000000e-01)*a[2])*a[29])));
out[10] = (((a[30]*a[4])+(a[0]*a[32]))-((((real_t)(2.1400000000000000e-01)*a[34])*a[3])+(((real_t)(2.1400000000000000e-01)*a[5])*a[35])));
out[11] = (((a[36]*a[4])+(a[0]*a[37]))-((((real_t)(2.1400000000000000e-01)*a[38])*a[3])+(((real_t)(2.1400000000000000e-01)*a[5])*a[39])));
out[12] = (((a[40]*a[4])+(a[0]*a[41]))-((((real_t)(2.1400000000000000e-01)*a[42])*a[3])+(((real_t)(2.1400000000000000e-01)*a[5])*a[43])));
out[13] = (((a[44]*a[4])+(a[0]*a[45]))-((((real_t)(2.1400000000000000e-01)*a[46])*a[3])+(((real_t)(2.1400000000000000e-01)*a[5])*a[47])));
out[14] = (((a[48]*a[4])+(a[0]*a[49]))-((((real_t)(2.1400000000000000e-01)*a[50])*a[3])+(((real_t)(2.1400000000000000e-01)*a[5])*a[51])));
out[15] = a[52];
out[16] = a[53];
out[17] = a[54];
out[18] = a[55];
out[19] = a[56];
out[20] = (real_t)(0.0000000000000000e+00);
out[21] = (real_t)(0.0000000000000000e+00);
out[22] = (real_t)(0.0000000000000000e+00);
out[23] = (real_t)(0.0000000000000000e+00);
out[24] = (real_t)(0.0000000000000000e+00);
out[25] = (real_t)(0.0000000000000000e+00);
out[26] = (real_t)(0.0000000000000000e+00);
out[27] = (real_t)(0.0000000000000000e+00);
out[28] = (real_t)(0.0000000000000000e+00);
out[29] = (real_t)(0.0000000000000000e+00);
out[30] = (((a[57]*a[1])+(a[0]*a[58]))+((((real_t)(2.1400000000000000e-01)*a[59])*a[3])+(((real_t)(2.1400000000000000e-01)*a[2])*a[60])));
out[31] = (((a[61]*a[1])+(a[0]*a[62]))+((((real_t)(2.1400000000000000e-01)*a[63])*a[3])+(((real_t)(2.1400000000000000e-01)*a[2])*a[64])));
out[32] = (((a[65]*a[4])+(a[0]*a[66]))-((((real_t)(2.1400000000000000e-01)*a[67])*a[3])+(((real_t)(2.1400000000000000e-01)*a[5])*a[68])));
out[33] = (((a[69]*a[4])+(a[0]*a[70]))-((((real_t)(2.1400000000000000e-01)*a[71])*a[3])+(((real_t)(2.1400000000000000e-01)*a[5])*a[72])));
out[34] = a[73];
out[35] = a[74];
out[36] = (real_t)(1.0000000000000000e+00);
out[37] = (real_t)(0.0000000000000000e+00);
out[38] = (real_t)(0.0000000000000000e+00);
out[39] = (real_t)(1.0000000000000000e+00);
}

/* Fixed step size:0.005 */
int nmpc_integrate( real_t* const rk_eta, int resetIntegrator )
{
int error;

int run1;
nmpcWorkspace.rk_ttt = 0.0000000000000000e+00;
rk_eta[5] = 1.0000000000000000e+00;
rk_eta[6] = 0.0000000000000000e+00;
rk_eta[7] = 0.0000000000000000e+00;
rk_eta[8] = 0.0000000000000000e+00;
rk_eta[9] = 0.0000000000000000e+00;
rk_eta[10] = 0.0000000000000000e+00;
rk_eta[11] = 1.0000000000000000e+00;
rk_eta[12] = 0.0000000000000000e+00;
rk_eta[13] = 0.0000000000000000e+00;
rk_eta[14] = 0.0000000000000000e+00;
rk_eta[15] = 0.0000000000000000e+00;
rk_eta[16] = 0.0000000000000000e+00;
rk_eta[17] = 1.0000000000000000e+00;
rk_eta[18] = 0.0000000000000000e+00;
rk_eta[19] = 0.0000000000000000e+00;
rk_eta[20] = 0.0000000000000000e+00;
rk_eta[21] = 0.0000000000000000e+00;
rk_eta[22] = 0.0000000000000000e+00;
rk_eta[23] = 1.0000000000000000e+00;
rk_eta[24] = 0.0000000000000000e+00;
rk_eta[25] = 0.0000000000000000e+00;
rk_eta[26] = 0.0000000000000000e+00;
rk_eta[27] = 0.0000000000000000e+00;
rk_eta[28] = 0.0000000000000000e+00;
rk_eta[29] = 1.0000000000000000e+00;
rk_eta[30] = 0.0000000000000000e+00;
rk_eta[31] = 0.0000000000000000e+00;
rk_eta[32] = 0.0000000000000000e+00;
rk_eta[33] = 0.0000000000000000e+00;
rk_eta[34] = 0.0000000000000000e+00;
rk_eta[35] = 0.0000000000000000e+00;
rk_eta[36] = 0.0000000000000000e+00;
rk_eta[37] = 0.0000000000000000e+00;
rk_eta[38] = 0.0000000000000000e+00;
rk_eta[39] = 0.0000000000000000e+00;
nmpcWorkspace.rk_xxx[40] = rk_eta[40];
nmpcWorkspace.rk_xxx[41] = rk_eta[41];
nmpcWorkspace.rk_xxx[42] = rk_eta[42];
nmpcWorkspace.rk_xxx[43] = rk_eta[43];

for (run1 = 0; run1 < 2; ++run1)
{
nmpcWorkspace.rk_xxx[0] = + rk_eta[0];
nmpcWorkspace.rk_xxx[1] = + rk_eta[1];
nmpcWorkspace.rk_xxx[2] = + rk_eta[2];
nmpcWorkspace.rk_xxx[3] = + rk_eta[3];
nmpcWorkspace.rk_xxx[4] = + rk_eta[4];
nmpcWorkspace.rk_xxx[5] = + rk_eta[5];
nmpcWorkspace.rk_xxx[6] = + rk_eta[6];
nmpcWorkspace.rk_xxx[7] = + rk_eta[7];
nmpcWorkspace.rk_xxx[8] = + rk_eta[8];
nmpcWorkspace.rk_xxx[9] = + rk_eta[9];
nmpcWorkspace.rk_xxx[10] = + rk_eta[10];
nmpcWorkspace.rk_xxx[11] = + rk_eta[11];
nmpcWorkspace.rk_xxx[12] = + rk_eta[12];
nmpcWorkspace.rk_xxx[13] = + rk_eta[13];
nmpcWorkspace.rk_xxx[14] = + rk_eta[14];
nmpcWorkspace.rk_xxx[15] = + rk_eta[15];
nmpcWorkspace.rk_xxx[16] = + rk_eta[16];
nmpcWorkspace.rk_xxx[17] = + rk_eta[17];
nmpcWorkspace.rk_xxx[18] = + rk_eta[18];
nmpcWorkspace.rk_xxx[19] = + rk_eta[19];
nmpcWorkspace.rk_xxx[20] = + rk_eta[20];
nmpcWorkspace.rk_xxx[21] = + rk_eta[21];
nmpcWorkspace.rk_xxx[22] = + rk_eta[22];
nmpcWorkspace.rk_xxx[23] = + rk_eta[23];
nmpcWorkspace.rk_xxx[24] = + rk_eta[24];
nmpcWorkspace.rk_xxx[25] = + rk_eta[25];
nmpcWorkspace.rk_xxx[26] = + rk_eta[26];
nmpcWorkspace.rk_xxx[27] = + rk_eta[27];
nmpcWorkspace.rk_xxx[28] = + rk_eta[28];
nmpcWorkspace.rk_xxx[29] = + rk_eta[29];
nmpcWorkspace.rk_xxx[30] = + rk_eta[30];
nmpcWorkspace.rk_xxx[31] = + rk_eta[31];
nmpcWorkspace.rk_xxx[32] = + rk_eta[32];
nmpcWorkspace.rk_xxx[33] = + rk_eta[33];
nmpcWorkspace.rk_xxx[34] = + rk_eta[34];
nmpcWorkspace.rk_xxx[35] = + rk_eta[35];
nmpcWorkspace.rk_xxx[36] = + rk_eta[36];
nmpcWorkspace.rk_xxx[37] = + rk_eta[37];
nmpcWorkspace.rk_xxx[38] = + rk_eta[38];
nmpcWorkspace.rk_xxx[39] = + rk_eta[39];
nmpc_rhs_forw( nmpcWorkspace.rk_xxx, nmpcWorkspace.rk_kkk );
nmpcWorkspace.rk_xxx[0] = + (real_t)2.5000000000000001e-03*nmpcWorkspace.rk_kkk[0] + rk_eta[0];
nmpcWorkspace.rk_xxx[1] = + (real_t)2.5000000000000001e-03*nmpcWorkspace.rk_kkk[1] + rk_eta[1];
nmpcWorkspace.rk_xxx[2] = + (real_t)2.5000000000000001e-03*nmpcWorkspace.rk_kkk[2] + rk_eta[2];
nmpcWorkspace.rk_xxx[3] = + (real_t)2.5000000000000001e-03*nmpcWorkspace.rk_kkk[3] + rk_eta[3];
nmpcWorkspace.rk_xxx[4] = + (real_t)2.5000000000000001e-03*nmpcWorkspace.rk_kkk[4] + rk_eta[4];
nmpcWorkspace.rk_xxx[5] = + (real_t)2.5000000000000001e-03*nmpcWorkspace.rk_kkk[5] + rk_eta[5];
nmpcWorkspace.rk_xxx[6] = + (real_t)2.5000000000000001e-03*nmpcWorkspace.rk_kkk[6] + rk_eta[6];
nmpcWorkspace.rk_xxx[7] = + (real_t)2.5000000000000001e-03*nmpcWorkspace.rk_kkk[7] + rk_eta[7];
nmpcWorkspace.rk_xxx[8] = + (real_t)2.5000000000000001e-03*nmpcWorkspace.rk_kkk[8] + rk_eta[8];
nmpcWorkspace.rk_xxx[9] = + (real_t)2.5000000000000001e-03*nmpcWorkspace.rk_kkk[9] + rk_eta[9];
nmpcWorkspace.rk_xxx[10] = + (real_t)2.5000000000000001e-03*nmpcWorkspace.rk_kkk[10] + rk_eta[10];
nmpcWorkspace.rk_xxx[11] = + (real_t)2.5000000000000001e-03*nmpcWorkspace.rk_kkk[11] + rk_eta[11];
nmpcWorkspace.rk_xxx[12] = + (real_t)2.5000000000000001e-03*nmpcWorkspace.rk_kkk[12] + rk_eta[12];
nmpcWorkspace.rk_xxx[13] = + (real_t)2.5000000000000001e-03*nmpcWorkspace.rk_kkk[13] + rk_eta[13];
nmpcWorkspace.rk_xxx[14] = + (real_t)2.5000000000000001e-03*nmpcWorkspace.rk_kkk[14] + rk_eta[14];
nmpcWorkspace.rk_xxx[15] = + (real_t)2.5000000000000001e-03*nmpcWorkspace.rk_kkk[15] + rk_eta[15];
nmpcWorkspace.rk_xxx[16] = + (real_t)2.5000000000000001e-03*nmpcWorkspace.rk_kkk[16] + rk_eta[16];
nmpcWorkspace.rk_xxx[17] = + (real_t)2.5000000000000001e-03*nmpcWorkspace.rk_kkk[17] + rk_eta[17];
nmpcWorkspace.rk_xxx[18] = + (real_t)2.5000000000000001e-03*nmpcWorkspace.rk_kkk[18] + rk_eta[18];
nmpcWorkspace.rk_xxx[19] = + (real_t)2.5000000000000001e-03*nmpcWorkspace.rk_kkk[19] + rk_eta[19];
nmpcWorkspace.rk_xxx[20] = + (real_t)2.5000000000000001e-03*nmpcWorkspace.rk_kkk[20] + rk_eta[20];
nmpcWorkspace.rk_xxx[21] = + (real_t)2.5000000000000001e-03*nmpcWorkspace.rk_kkk[21] + rk_eta[21];
nmpcWorkspace.rk_xxx[22] = + (real_t)2.5000000000000001e-03*nmpcWorkspace.rk_kkk[22] + rk_eta[22];
nmpcWorkspace.rk_xxx[23] = + (real_t)2.5000000000000001e-03*nmpcWorkspace.rk_kkk[23] + rk_eta[23];
nmpcWorkspace.rk_xxx[24] = + (real_t)2.5000000000000001e-03*nmpcWorkspace.rk_kkk[24] + rk_eta[24];
nmpcWorkspace.rk_xxx[25] = + (real_t)2.5000000000000001e-03*nmpcWorkspace.rk_kkk[25] + rk_eta[25];
nmpcWorkspace.rk_xxx[26] = + (real_t)2.5000000000000001e-03*nmpcWorkspace.rk_kkk[26] + rk_eta[26];
nmpcWorkspace.rk_xxx[27] = + (real_t)2.5000000000000001e-03*nmpcWorkspace.rk_kkk[27] + rk_eta[27];
nmpcWorkspace.rk_xxx[28] = + (real_t)2.5000000000000001e-03*nmpcWorkspace.rk_kkk[28] + rk_eta[28];
nmpcWorkspace.rk_xxx[29] = + (real_t)2.5000000000000001e-03*nmpcWorkspace.rk_kkk[29] + rk_eta[29];
nmpcWorkspace.rk_xxx[30] = + (real_t)2.5000000000000001e-03*nmpcWorkspace.rk_kkk[30] + rk_eta[30];
nmpcWorkspace.rk_xxx[31] = + (real_t)2.5000000000000001e-03*nmpcWorkspace.rk_kkk[31] + rk_eta[31];
nmpcWorkspace.rk_xxx[32] = + (real_t)2.5000000000000001e-03*nmpcWorkspace.rk_kkk[32] + rk_eta[32];
nmpcWorkspace.rk_xxx[33] = + (real_t)2.5000000000000001e-03*nmpcWorkspace.rk_kkk[33] + rk_eta[33];
nmpcWorkspace.rk_xxx[34] = + (real_t)2.5000000000000001e-03*nmpcWorkspace.rk_kkk[34] + rk_eta[34];
nmpcWorkspace.rk_xxx[35] = + (real_t)2.5000000000000001e-03*nmpcWorkspace.rk_kkk[35] + rk_eta[35];
nmpcWorkspace.rk_xxx[36] = + (real_t)2.5000000000000001e-03*nmpcWorkspace.rk_kkk[36] + rk_eta[36];
nmpcWorkspace.rk_xxx[37] = + (real_t)2.5000000000000001e-03*nmpcWorkspace.rk_kkk[37] + rk_eta[37];
nmpcWorkspace.rk_xxx[38] = + (real_t)2.5000000000000001e-03*nmpcWorkspace.rk_kkk[38] + rk_eta[38];
nmpcWorkspace.rk_xxx[39] = + (real_t)2.5000000000000001e-03*nmpcWorkspace.rk_kkk[39] + rk_eta[39];
nmpc_rhs_forw( nmpcWorkspace.rk_xxx, &(nmpcWorkspace.rk_kkk[ 40 ]) );
nmpcWorkspace.rk_xxx[0] = + (real_t)2.5000000000000001e-03*nmpcWorkspace.rk_kkk[40] + rk_eta[0];
nmpcWorkspace.rk_xxx[1] = + (real_t)2.5000000000000001e-03*nmpcWorkspace.rk_kkk[41] + rk_eta[1];
nmpcWorkspace.rk_xxx[2] = + (real_t)2.5000000000000001e-03*nmpcWorkspace.rk_kkk[42] + rk_eta[2];
nmpcWorkspace.rk_xxx[3] = + (real_t)2.5000000000000001e-03*nmpcWorkspace.rk_kkk[43] + rk_eta[3];
nmpcWorkspace.rk_xxx[4] = + (real_t)2.5000000000000001e-03*nmpcWorkspace.rk_kkk[44] + rk_eta[4];
nmpcWorkspace.rk_xxx[5] = + (real_t)2.5000000000000001e-03*nmpcWorkspace.rk_kkk[45] + rk_eta[5];
nmpcWorkspace.rk_xxx[6] = + (real_t)2.5000000000000001e-03*nmpcWorkspace.rk_kkk[46] + rk_eta[6];
nmpcWorkspace.rk_xxx[7] = + (real_t)2.5000000000000001e-03*nmpcWorkspace.rk_kkk[47] + rk_eta[7];
nmpcWorkspace.rk_xxx[8] = + (real_t)2.5000000000000001e-03*nmpcWorkspace.rk_kkk[48] + rk_eta[8];
nmpcWorkspace.rk_xxx[9] = + (real_t)2.5000000000000001e-03*nmpcWorkspace.rk_kkk[49] + rk_eta[9];
nmpcWorkspace.rk_xxx[10] = + (real_t)2.5000000000000001e-03*nmpcWorkspace.rk_kkk[50] + rk_eta[10];
nmpcWorkspace.rk_xxx[11] = + (real_t)2.5000000000000001e-03*nmpcWorkspace.rk_kkk[51] + rk_eta[11];
nmpcWorkspace.rk_xxx[12] = + (real_t)2.5000000000000001e-03*nmpcWorkspace.rk_kkk[52] + rk_eta[12];
nmpcWorkspace.rk_xxx[13] = + (real_t)2.5000000000000001e-03*nmpcWorkspace.rk_kkk[53] + rk_eta[13];
nmpcWorkspace.rk_xxx[14] = + (real_t)2.5000000000000001e-03*nmpcWorkspace.rk_kkk[54] + rk_eta[14];
nmpcWorkspace.rk_xxx[15] = + (real_t)2.5000000000000001e-03*nmpcWorkspace.rk_kkk[55] + rk_eta[15];
nmpcWorkspace.rk_xxx[16] = + (real_t)2.5000000000000001e-03*nmpcWorkspace.rk_kkk[56] + rk_eta[16];
nmpcWorkspace.rk_xxx[17] = + (real_t)2.5000000000000001e-03*nmpcWorkspace.rk_kkk[57] + rk_eta[17];
nmpcWorkspace.rk_xxx[18] = + (real_t)2.5000000000000001e-03*nmpcWorkspace.rk_kkk[58] + rk_eta[18];
nmpcWorkspace.rk_xxx[19] = + (real_t)2.5000000000000001e-03*nmpcWorkspace.rk_kkk[59] + rk_eta[19];
nmpcWorkspace.rk_xxx[20] = + (real_t)2.5000000000000001e-03*nmpcWorkspace.rk_kkk[60] + rk_eta[20];
nmpcWorkspace.rk_xxx[21] = + (real_t)2.5000000000000001e-03*nmpcWorkspace.rk_kkk[61] + rk_eta[21];
nmpcWorkspace.rk_xxx[22] = + (real_t)2.5000000000000001e-03*nmpcWorkspace.rk_kkk[62] + rk_eta[22];
nmpcWorkspace.rk_xxx[23] = + (real_t)2.5000000000000001e-03*nmpcWorkspace.rk_kkk[63] + rk_eta[23];
nmpcWorkspace.rk_xxx[24] = + (real_t)2.5000000000000001e-03*nmpcWorkspace.rk_kkk[64] + rk_eta[24];
nmpcWorkspace.rk_xxx[25] = + (real_t)2.5000000000000001e-03*nmpcWorkspace.rk_kkk[65] + rk_eta[25];
nmpcWorkspace.rk_xxx[26] = + (real_t)2.5000000000000001e-03*nmpcWorkspace.rk_kkk[66] + rk_eta[26];
nmpcWorkspace.rk_xxx[27] = + (real_t)2.5000000000000001e-03*nmpcWorkspace.rk_kkk[67] + rk_eta[27];
nmpcWorkspace.rk_xxx[28] = + (real_t)2.5000000000000001e-03*nmpcWorkspace.rk_kkk[68] + rk_eta[28];
nmpcWorkspace.rk_xxx[29] = + (real_t)2.5000000000000001e-03*nmpcWorkspace.rk_kkk[69] + rk_eta[29];
nmpcWorkspace.rk_xxx[30] = + (real_t)2.5000000000000001e-03*nmpcWorkspace.rk_kkk[70] + rk_eta[30];
nmpcWorkspace.rk_xxx[31] = + (real_t)2.5000000000000001e-03*nmpcWorkspace.rk_kkk[71] + rk_eta[31];
nmpcWorkspace.rk_xxx[32] = + (real_t)2.5000000000000001e-03*nmpcWorkspace.rk_kkk[72] + rk_eta[32];
nmpcWorkspace.rk_xxx[33] = + (real_t)2.5000000000000001e-03*nmpcWorkspace.rk_kkk[73] + rk_eta[33];
nmpcWorkspace.rk_xxx[34] = + (real_t)2.5000000000000001e-03*nmpcWorkspace.rk_kkk[74] + rk_eta[34];
nmpcWorkspace.rk_xxx[35] = + (real_t)2.5000000000000001e-03*nmpcWorkspace.rk_kkk[75] + rk_eta[35];
nmpcWorkspace.rk_xxx[36] = + (real_t)2.5000000000000001e-03*nmpcWorkspace.rk_kkk[76] + rk_eta[36];
nmpcWorkspace.rk_xxx[37] = + (real_t)2.5000000000000001e-03*nmpcWorkspace.rk_kkk[77] + rk_eta[37];
nmpcWorkspace.rk_xxx[38] = + (real_t)2.5000000000000001e-03*nmpcWorkspace.rk_kkk[78] + rk_eta[38];
nmpcWorkspace.rk_xxx[39] = + (real_t)2.5000000000000001e-03*nmpcWorkspace.rk_kkk[79] + rk_eta[39];
nmpc_rhs_forw( nmpcWorkspace.rk_xxx, &(nmpcWorkspace.rk_kkk[ 80 ]) );
nmpcWorkspace.rk_xxx[0] = + (real_t)5.0000000000000001e-03*nmpcWorkspace.rk_kkk[80] + rk_eta[0];
nmpcWorkspace.rk_xxx[1] = + (real_t)5.0000000000000001e-03*nmpcWorkspace.rk_kkk[81] + rk_eta[1];
nmpcWorkspace.rk_xxx[2] = + (real_t)5.0000000000000001e-03*nmpcWorkspace.rk_kkk[82] + rk_eta[2];
nmpcWorkspace.rk_xxx[3] = + (real_t)5.0000000000000001e-03*nmpcWorkspace.rk_kkk[83] + rk_eta[3];
nmpcWorkspace.rk_xxx[4] = + (real_t)5.0000000000000001e-03*nmpcWorkspace.rk_kkk[84] + rk_eta[4];
nmpcWorkspace.rk_xxx[5] = + (real_t)5.0000000000000001e-03*nmpcWorkspace.rk_kkk[85] + rk_eta[5];
nmpcWorkspace.rk_xxx[6] = + (real_t)5.0000000000000001e-03*nmpcWorkspace.rk_kkk[86] + rk_eta[6];
nmpcWorkspace.rk_xxx[7] = + (real_t)5.0000000000000001e-03*nmpcWorkspace.rk_kkk[87] + rk_eta[7];
nmpcWorkspace.rk_xxx[8] = + (real_t)5.0000000000000001e-03*nmpcWorkspace.rk_kkk[88] + rk_eta[8];
nmpcWorkspace.rk_xxx[9] = + (real_t)5.0000000000000001e-03*nmpcWorkspace.rk_kkk[89] + rk_eta[9];
nmpcWorkspace.rk_xxx[10] = + (real_t)5.0000000000000001e-03*nmpcWorkspace.rk_kkk[90] + rk_eta[10];
nmpcWorkspace.rk_xxx[11] = + (real_t)5.0000000000000001e-03*nmpcWorkspace.rk_kkk[91] + rk_eta[11];
nmpcWorkspace.rk_xxx[12] = + (real_t)5.0000000000000001e-03*nmpcWorkspace.rk_kkk[92] + rk_eta[12];
nmpcWorkspace.rk_xxx[13] = + (real_t)5.0000000000000001e-03*nmpcWorkspace.rk_kkk[93] + rk_eta[13];
nmpcWorkspace.rk_xxx[14] = + (real_t)5.0000000000000001e-03*nmpcWorkspace.rk_kkk[94] + rk_eta[14];
nmpcWorkspace.rk_xxx[15] = + (real_t)5.0000000000000001e-03*nmpcWorkspace.rk_kkk[95] + rk_eta[15];
nmpcWorkspace.rk_xxx[16] = + (real_t)5.0000000000000001e-03*nmpcWorkspace.rk_kkk[96] + rk_eta[16];
nmpcWorkspace.rk_xxx[17] = + (real_t)5.0000000000000001e-03*nmpcWorkspace.rk_kkk[97] + rk_eta[17];
nmpcWorkspace.rk_xxx[18] = + (real_t)5.0000000000000001e-03*nmpcWorkspace.rk_kkk[98] + rk_eta[18];
nmpcWorkspace.rk_xxx[19] = + (real_t)5.0000000000000001e-03*nmpcWorkspace.rk_kkk[99] + rk_eta[19];
nmpcWorkspace.rk_xxx[20] = + (real_t)5.0000000000000001e-03*nmpcWorkspace.rk_kkk[100] + rk_eta[20];
nmpcWorkspace.rk_xxx[21] = + (real_t)5.0000000000000001e-03*nmpcWorkspace.rk_kkk[101] + rk_eta[21];
nmpcWorkspace.rk_xxx[22] = + (real_t)5.0000000000000001e-03*nmpcWorkspace.rk_kkk[102] + rk_eta[22];
nmpcWorkspace.rk_xxx[23] = + (real_t)5.0000000000000001e-03*nmpcWorkspace.rk_kkk[103] + rk_eta[23];
nmpcWorkspace.rk_xxx[24] = + (real_t)5.0000000000000001e-03*nmpcWorkspace.rk_kkk[104] + rk_eta[24];
nmpcWorkspace.rk_xxx[25] = + (real_t)5.0000000000000001e-03*nmpcWorkspace.rk_kkk[105] + rk_eta[25];
nmpcWorkspace.rk_xxx[26] = + (real_t)5.0000000000000001e-03*nmpcWorkspace.rk_kkk[106] + rk_eta[26];
nmpcWorkspace.rk_xxx[27] = + (real_t)5.0000000000000001e-03*nmpcWorkspace.rk_kkk[107] + rk_eta[27];
nmpcWorkspace.rk_xxx[28] = + (real_t)5.0000000000000001e-03*nmpcWorkspace.rk_kkk[108] + rk_eta[28];
nmpcWorkspace.rk_xxx[29] = + (real_t)5.0000000000000001e-03*nmpcWorkspace.rk_kkk[109] + rk_eta[29];
nmpcWorkspace.rk_xxx[30] = + (real_t)5.0000000000000001e-03*nmpcWorkspace.rk_kkk[110] + rk_eta[30];
nmpcWorkspace.rk_xxx[31] = + (real_t)5.0000000000000001e-03*nmpcWorkspace.rk_kkk[111] + rk_eta[31];
nmpcWorkspace.rk_xxx[32] = + (real_t)5.0000000000000001e-03*nmpcWorkspace.rk_kkk[112] + rk_eta[32];
nmpcWorkspace.rk_xxx[33] = + (real_t)5.0000000000000001e-03*nmpcWorkspace.rk_kkk[113] + rk_eta[33];
nmpcWorkspace.rk_xxx[34] = + (real_t)5.0000000000000001e-03*nmpcWorkspace.rk_kkk[114] + rk_eta[34];
nmpcWorkspace.rk_xxx[35] = + (real_t)5.0000000000000001e-03*nmpcWorkspace.rk_kkk[115] + rk_eta[35];
nmpcWorkspace.rk_xxx[36] = + (real_t)5.0000000000000001e-03*nmpcWorkspace.rk_kkk[116] + rk_eta[36];
nmpcWorkspace.rk_xxx[37] = + (real_t)5.0000000000000001e-03*nmpcWorkspace.rk_kkk[117] + rk_eta[37];
nmpcWorkspace.rk_xxx[38] = + (real_t)5.0000000000000001e-03*nmpcWorkspace.rk_kkk[118] + rk_eta[38];
nmpcWorkspace.rk_xxx[39] = + (real_t)5.0000000000000001e-03*nmpcWorkspace.rk_kkk[119] + rk_eta[39];
nmpc_rhs_forw( nmpcWorkspace.rk_xxx, &(nmpcWorkspace.rk_kkk[ 120 ]) );
rk_eta[0] += + (real_t)8.3333333333333328e-04*nmpcWorkspace.rk_kkk[0] + (real_t)1.6666666666666666e-03*nmpcWorkspace.rk_kkk[40] + (real_t)1.6666666666666666e-03*nmpcWorkspace.rk_kkk[80] + (real_t)8.3333333333333328e-04*nmpcWorkspace.rk_kkk[120];
rk_eta[1] += + (real_t)8.3333333333333328e-04*nmpcWorkspace.rk_kkk[1] + (real_t)1.6666666666666666e-03*nmpcWorkspace.rk_kkk[41] + (real_t)1.6666666666666666e-03*nmpcWorkspace.rk_kkk[81] + (real_t)8.3333333333333328e-04*nmpcWorkspace.rk_kkk[121];
rk_eta[2] += + (real_t)8.3333333333333328e-04*nmpcWorkspace.rk_kkk[2] + (real_t)1.6666666666666666e-03*nmpcWorkspace.rk_kkk[42] + (real_t)1.6666666666666666e-03*nmpcWorkspace.rk_kkk[82] + (real_t)8.3333333333333328e-04*nmpcWorkspace.rk_kkk[122];
rk_eta[3] += + (real_t)8.3333333333333328e-04*nmpcWorkspace.rk_kkk[3] + (real_t)1.6666666666666666e-03*nmpcWorkspace.rk_kkk[43] + (real_t)1.6666666666666666e-03*nmpcWorkspace.rk_kkk[83] + (real_t)8.3333333333333328e-04*nmpcWorkspace.rk_kkk[123];
rk_eta[4] += + (real_t)8.3333333333333328e-04*nmpcWorkspace.rk_kkk[4] + (real_t)1.6666666666666666e-03*nmpcWorkspace.rk_kkk[44] + (real_t)1.6666666666666666e-03*nmpcWorkspace.rk_kkk[84] + (real_t)8.3333333333333328e-04*nmpcWorkspace.rk_kkk[124];
rk_eta[5] += + (real_t)8.3333333333333328e-04*nmpcWorkspace.rk_kkk[5] + (real_t)1.6666666666666666e-03*nmpcWorkspace.rk_kkk[45] + (real_t)1.6666666666666666e-03*nmpcWorkspace.rk_kkk[85] + (real_t)8.3333333333333328e-04*nmpcWorkspace.rk_kkk[125];
rk_eta[6] += + (real_t)8.3333333333333328e-04*nmpcWorkspace.rk_kkk[6] + (real_t)1.6666666666666666e-03*nmpcWorkspace.rk_kkk[46] + (real_t)1.6666666666666666e-03*nmpcWorkspace.rk_kkk[86] + (real_t)8.3333333333333328e-04*nmpcWorkspace.rk_kkk[126];
rk_eta[7] += + (real_t)8.3333333333333328e-04*nmpcWorkspace.rk_kkk[7] + (real_t)1.6666666666666666e-03*nmpcWorkspace.rk_kkk[47] + (real_t)1.6666666666666666e-03*nmpcWorkspace.rk_kkk[87] + (real_t)8.3333333333333328e-04*nmpcWorkspace.rk_kkk[127];
rk_eta[8] += + (real_t)8.3333333333333328e-04*nmpcWorkspace.rk_kkk[8] + (real_t)1.6666666666666666e-03*nmpcWorkspace.rk_kkk[48] + (real_t)1.6666666666666666e-03*nmpcWorkspace.rk_kkk[88] + (real_t)8.3333333333333328e-04*nmpcWorkspace.rk_kkk[128];
rk_eta[9] += + (real_t)8.3333333333333328e-04*nmpcWorkspace.rk_kkk[9] + (real_t)1.6666666666666666e-03*nmpcWorkspace.rk_kkk[49] + (real_t)1.6666666666666666e-03*nmpcWorkspace.rk_kkk[89] + (real_t)8.3333333333333328e-04*nmpcWorkspace.rk_kkk[129];
rk_eta[10] += + (real_t)8.3333333333333328e-04*nmpcWorkspace.rk_kkk[10] + (real_t)1.6666666666666666e-03*nmpcWorkspace.rk_kkk[50] + (real_t)1.6666666666666666e-03*nmpcWorkspace.rk_kkk[90] + (real_t)8.3333333333333328e-04*nmpcWorkspace.rk_kkk[130];
rk_eta[11] += + (real_t)8.3333333333333328e-04*nmpcWorkspace.rk_kkk[11] + (real_t)1.6666666666666666e-03*nmpcWorkspace.rk_kkk[51] + (real_t)1.6666666666666666e-03*nmpcWorkspace.rk_kkk[91] + (real_t)8.3333333333333328e-04*nmpcWorkspace.rk_kkk[131];
rk_eta[12] += + (real_t)8.3333333333333328e-04*nmpcWorkspace.rk_kkk[12] + (real_t)1.6666666666666666e-03*nmpcWorkspace.rk_kkk[52] + (real_t)1.6666666666666666e-03*nmpcWorkspace.rk_kkk[92] + (real_t)8.3333333333333328e-04*nmpcWorkspace.rk_kkk[132];
rk_eta[13] += + (real_t)8.3333333333333328e-04*nmpcWorkspace.rk_kkk[13] + (real_t)1.6666666666666666e-03*nmpcWorkspace.rk_kkk[53] + (real_t)1.6666666666666666e-03*nmpcWorkspace.rk_kkk[93] + (real_t)8.3333333333333328e-04*nmpcWorkspace.rk_kkk[133];
rk_eta[14] += + (real_t)8.3333333333333328e-04*nmpcWorkspace.rk_kkk[14] + (real_t)1.6666666666666666e-03*nmpcWorkspace.rk_kkk[54] + (real_t)1.6666666666666666e-03*nmpcWorkspace.rk_kkk[94] + (real_t)8.3333333333333328e-04*nmpcWorkspace.rk_kkk[134];
rk_eta[15] += + (real_t)8.3333333333333328e-04*nmpcWorkspace.rk_kkk[15] + (real_t)1.6666666666666666e-03*nmpcWorkspace.rk_kkk[55] + (real_t)1.6666666666666666e-03*nmpcWorkspace.rk_kkk[95] + (real_t)8.3333333333333328e-04*nmpcWorkspace.rk_kkk[135];
rk_eta[16] += + (real_t)8.3333333333333328e-04*nmpcWorkspace.rk_kkk[16] + (real_t)1.6666666666666666e-03*nmpcWorkspace.rk_kkk[56] + (real_t)1.6666666666666666e-03*nmpcWorkspace.rk_kkk[96] + (real_t)8.3333333333333328e-04*nmpcWorkspace.rk_kkk[136];
rk_eta[17] += + (real_t)8.3333333333333328e-04*nmpcWorkspace.rk_kkk[17] + (real_t)1.6666666666666666e-03*nmpcWorkspace.rk_kkk[57] + (real_t)1.6666666666666666e-03*nmpcWorkspace.rk_kkk[97] + (real_t)8.3333333333333328e-04*nmpcWorkspace.rk_kkk[137];
rk_eta[18] += + (real_t)8.3333333333333328e-04*nmpcWorkspace.rk_kkk[18] + (real_t)1.6666666666666666e-03*nmpcWorkspace.rk_kkk[58] + (real_t)1.6666666666666666e-03*nmpcWorkspace.rk_kkk[98] + (real_t)8.3333333333333328e-04*nmpcWorkspace.rk_kkk[138];
rk_eta[19] += + (real_t)8.3333333333333328e-04*nmpcWorkspace.rk_kkk[19] + (real_t)1.6666666666666666e-03*nmpcWorkspace.rk_kkk[59] + (real_t)1.6666666666666666e-03*nmpcWorkspace.rk_kkk[99] + (real_t)8.3333333333333328e-04*nmpcWorkspace.rk_kkk[139];
rk_eta[20] += + (real_t)8.3333333333333328e-04*nmpcWorkspace.rk_kkk[20] + (real_t)1.6666666666666666e-03*nmpcWorkspace.rk_kkk[60] + (real_t)1.6666666666666666e-03*nmpcWorkspace.rk_kkk[100] + (real_t)8.3333333333333328e-04*nmpcWorkspace.rk_kkk[140];
rk_eta[21] += + (real_t)8.3333333333333328e-04*nmpcWorkspace.rk_kkk[21] + (real_t)1.6666666666666666e-03*nmpcWorkspace.rk_kkk[61] + (real_t)1.6666666666666666e-03*nmpcWorkspace.rk_kkk[101] + (real_t)8.3333333333333328e-04*nmpcWorkspace.rk_kkk[141];
rk_eta[22] += + (real_t)8.3333333333333328e-04*nmpcWorkspace.rk_kkk[22] + (real_t)1.6666666666666666e-03*nmpcWorkspace.rk_kkk[62] + (real_t)1.6666666666666666e-03*nmpcWorkspace.rk_kkk[102] + (real_t)8.3333333333333328e-04*nmpcWorkspace.rk_kkk[142];
rk_eta[23] += + (real_t)8.3333333333333328e-04*nmpcWorkspace.rk_kkk[23] + (real_t)1.6666666666666666e-03*nmpcWorkspace.rk_kkk[63] + (real_t)1.6666666666666666e-03*nmpcWorkspace.rk_kkk[103] + (real_t)8.3333333333333328e-04*nmpcWorkspace.rk_kkk[143];
rk_eta[24] += + (real_t)8.3333333333333328e-04*nmpcWorkspace.rk_kkk[24] + (real_t)1.6666666666666666e-03*nmpcWorkspace.rk_kkk[64] + (real_t)1.6666666666666666e-03*nmpcWorkspace.rk_kkk[104] + (real_t)8.3333333333333328e-04*nmpcWorkspace.rk_kkk[144];
rk_eta[25] += + (real_t)8.3333333333333328e-04*nmpcWorkspace.rk_kkk[25] + (real_t)1.6666666666666666e-03*nmpcWorkspace.rk_kkk[65] + (real_t)1.6666666666666666e-03*nmpcWorkspace.rk_kkk[105] + (real_t)8.3333333333333328e-04*nmpcWorkspace.rk_kkk[145];
rk_eta[26] += + (real_t)8.3333333333333328e-04*nmpcWorkspace.rk_kkk[26] + (real_t)1.6666666666666666e-03*nmpcWorkspace.rk_kkk[66] + (real_t)1.6666666666666666e-03*nmpcWorkspace.rk_kkk[106] + (real_t)8.3333333333333328e-04*nmpcWorkspace.rk_kkk[146];
rk_eta[27] += + (real_t)8.3333333333333328e-04*nmpcWorkspace.rk_kkk[27] + (real_t)1.6666666666666666e-03*nmpcWorkspace.rk_kkk[67] + (real_t)1.6666666666666666e-03*nmpcWorkspace.rk_kkk[107] + (real_t)8.3333333333333328e-04*nmpcWorkspace.rk_kkk[147];
rk_eta[28] += + (real_t)8.3333333333333328e-04*nmpcWorkspace.rk_kkk[28] + (real_t)1.6666666666666666e-03*nmpcWorkspace.rk_kkk[68] + (real_t)1.6666666666666666e-03*nmpcWorkspace.rk_kkk[108] + (real_t)8.3333333333333328e-04*nmpcWorkspace.rk_kkk[148];
rk_eta[29] += + (real_t)8.3333333333333328e-04*nmpcWorkspace.rk_kkk[29] + (real_t)1.6666666666666666e-03*nmpcWorkspace.rk_kkk[69] + (real_t)1.6666666666666666e-03*nmpcWorkspace.rk_kkk[109] + (real_t)8.3333333333333328e-04*nmpcWorkspace.rk_kkk[149];
rk_eta[30] += + (real_t)8.3333333333333328e-04*nmpcWorkspace.rk_kkk[30] + (real_t)1.6666666666666666e-03*nmpcWorkspace.rk_kkk[70] + (real_t)1.6666666666666666e-03*nmpcWorkspace.rk_kkk[110] + (real_t)8.3333333333333328e-04*nmpcWorkspace.rk_kkk[150];
rk_eta[31] += + (real_t)8.3333333333333328e-04*nmpcWorkspace.rk_kkk[31] + (real_t)1.6666666666666666e-03*nmpcWorkspace.rk_kkk[71] + (real_t)1.6666666666666666e-03*nmpcWorkspace.rk_kkk[111] + (real_t)8.3333333333333328e-04*nmpcWorkspace.rk_kkk[151];
rk_eta[32] += + (real_t)8.3333333333333328e-04*nmpcWorkspace.rk_kkk[32] + (real_t)1.6666666666666666e-03*nmpcWorkspace.rk_kkk[72] + (real_t)1.6666666666666666e-03*nmpcWorkspace.rk_kkk[112] + (real_t)8.3333333333333328e-04*nmpcWorkspace.rk_kkk[152];
rk_eta[33] += + (real_t)8.3333333333333328e-04*nmpcWorkspace.rk_kkk[33] + (real_t)1.6666666666666666e-03*nmpcWorkspace.rk_kkk[73] + (real_t)1.6666666666666666e-03*nmpcWorkspace.rk_kkk[113] + (real_t)8.3333333333333328e-04*nmpcWorkspace.rk_kkk[153];
rk_eta[34] += + (real_t)8.3333333333333328e-04*nmpcWorkspace.rk_kkk[34] + (real_t)1.6666666666666666e-03*nmpcWorkspace.rk_kkk[74] + (real_t)1.6666666666666666e-03*nmpcWorkspace.rk_kkk[114] + (real_t)8.3333333333333328e-04*nmpcWorkspace.rk_kkk[154];
rk_eta[35] += + (real_t)8.3333333333333328e-04*nmpcWorkspace.rk_kkk[35] + (real_t)1.6666666666666666e-03*nmpcWorkspace.rk_kkk[75] + (real_t)1.6666666666666666e-03*nmpcWorkspace.rk_kkk[115] + (real_t)8.3333333333333328e-04*nmpcWorkspace.rk_kkk[155];
rk_eta[36] += + (real_t)8.3333333333333328e-04*nmpcWorkspace.rk_kkk[36] + (real_t)1.6666666666666666e-03*nmpcWorkspace.rk_kkk[76] + (real_t)1.6666666666666666e-03*nmpcWorkspace.rk_kkk[116] + (real_t)8.3333333333333328e-04*nmpcWorkspace.rk_kkk[156];
rk_eta[37] += + (real_t)8.3333333333333328e-04*nmpcWorkspace.rk_kkk[37] + (real_t)1.6666666666666666e-03*nmpcWorkspace.rk_kkk[77] + (real_t)1.6666666666666666e-03*nmpcWorkspace.rk_kkk[117] + (real_t)8.3333333333333328e-04*nmpcWorkspace.rk_kkk[157];
rk_eta[38] += + (real_t)8.3333333333333328e-04*nmpcWorkspace.rk_kkk[38] + (real_t)1.6666666666666666e-03*nmpcWorkspace.rk_kkk[78] + (real_t)1.6666666666666666e-03*nmpcWorkspace.rk_kkk[118] + (real_t)8.3333333333333328e-04*nmpcWorkspace.rk_kkk[158];
rk_eta[39] += + (real_t)8.3333333333333328e-04*nmpcWorkspace.rk_kkk[39] + (real_t)1.6666666666666666e-03*nmpcWorkspace.rk_kkk[79] + (real_t)1.6666666666666666e-03*nmpcWorkspace.rk_kkk[119] + (real_t)8.3333333333333328e-04*nmpcWorkspace.rk_kkk[159];
nmpcWorkspace.rk_ttt += 5.0000000000000000e-01;
}
error = 0;
return error;
}

