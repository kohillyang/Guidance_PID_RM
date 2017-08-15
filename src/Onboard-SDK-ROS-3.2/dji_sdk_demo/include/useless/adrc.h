#ifndef __ADRC_H__
#define __ADRC_H__

#include<stdio.h>
#include<stdlib.h>

#if defined(MATLAB_MEX_FILE)
#include "tmwtypes.h"
#include "simstruc_types.h"
#else
#define real_T  float
#endif



class CADRC{
public:
	CADRC();
	~CADRC();
private:
	real_T NF_wc,ESO_w0;
	real_T Sys_b;
public:
	real_T TD_r, TD_delta;
	real_T NF_belta1,NF_belta2,NF_alpha1,NF_alpha2,NF_b,NF_delta,NF_limited;
	real_T ESO_belta1,ESO_belta2,ESO_belta3,ESO_alpha1,ESO_alpha2,ESO_alpha3,ESO_b,ESO_delta;
	real_T Sys_a1,Sys_a2,Sys_T;	//总共22个需要整定的参数。。

	real_T TD_x1,TD_x2;
	real_T ESO_z1,ESO_z2,ESO_z3; 
	real_T NF_cmd;
	FILE *fp;

public:	
	int SetParamsAll(const real_T p_TD_r,const  real_T p_TD_delta,const  real_T p_NF_belta1,const  real_T p_NF_belta2,const  real_T p_NF_alpha1, const  real_T p_NF_alpha2,const  real_T p_NF_b,const  real_T p_NF_delta,const  real_T p_NF_limited,const  real_T p_ESO_belta1,const  real_T p_ESO_belta2,const real_T p_ESO_belta3,const  real_T p_ESO_alpha1,const  real_T p_ESO_alpha2,const real_T p_ESO_alpha3,const  real_T p_ESO_b,const  real_T p_ESO_delta,const  real_T p_Sys_a1,const  real_T p_Sys_a2,const  real_T p_Sys_b,const  real_T p_Sys_T);	
	int SetParamsQuick(const  real_T p_TD_r,const real_T p_TD_delta,const real_T p_NF_wc, const real_T p_NF_alpha1,const real_T p_NF_alpha2,const real_T p_NF_b,const real_T p_NF_delta,const real_T p_NF_limited,const real_T p_ESO_w0,const real_T p_ESO_alpha1,const real_T p_ESO_alpha2,const real_T p_ESO_alpha3,const real_T p_ESO_b,const real_T p_ESO_delta,const real_T p_Sys_a1,const real_T p_Sys_a2,const real_T p_Sys_b,const real_T p_Sys_T);		
	int ADRC_Once(real_T pos_aim,real_T pos_feedback,real_T *vel_cmd);
	int ADRC_RK4(real_T pos_aim,real_T pos_feedback,real_T *vel_cmd);
	int ADRC_RK42(real_T pos_aim,real_T pos_feedback,real_T vel_feedback,real_T *vel_cmd);
	real_T FST2(real_T x1,real_T x2,real_T v,real_T r,real_T d);
	real_T FAL(real_T e,real_T alpha,real_T delta);
	int Setw0wc(real_T p_ESO_w0,real_T p_NF_wc);
	int Setb(real_T b);
	int ADRC_Diff(const real_T u[2],const real_T x[5], real_T dx[5]);
	int ADRC_Diff2(const real_T u[3],const real_T x[5], real_T dx[5]);
	int ADRC_Update(real_T x[5],const real_T dx[5]);
	int ADRC_Output(const real_T x[5], real_T *y);
	int ADRC_Output2(const real_T x[5], real_T *y);  
	int Open(const char *filename);
	long long GetTime(void);
};

#endif
