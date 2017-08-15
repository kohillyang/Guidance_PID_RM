#include"wqf_math.h"
void Quaternion_To_Euler(float q0,float q1,float q2,float q3,  float *yaw,float *pitch,float *roll)
{
		float r11,r12,r21,r31,r32,r1,r2,r3;
		
    	r11 = 2.0f *(q1 * q2 + q0 * q3);
    	r12 = q0 * q0 + q1 * q1 - q2 * q2  - q3 * q3 ;
    	r21 = -2.0f * (q1 * q3 - q0 * q2);
    	r31 = 2.0f *( q2 * q3 + q0  * q1);
    	r32 = q0 * q0 - q1 * q1  - q2 * q2 + q3 * q3 ;
    	/*float yaw= atan2( r11, r12 );
    	float pitch = asin( r21 );
    	float roll = atan2( r31, r32 );
           */    	
    	*yaw= atan2( r11, r12 );
        *pitch = asin( r21 );
        *roll = atan2( r31, r32 );
    	/*cvmSet(att, 0, 0, roll);
    	cvmSet(att, 1, 0, pitch);
    	cvmSet(att, 2, 0, yaw);*/
//printf("%f  \n",asin(r21));
//printf("%f  \n",r21);
//printf("hhhhhhhhhhhhhhhhhhhhhhhh\n");
}
