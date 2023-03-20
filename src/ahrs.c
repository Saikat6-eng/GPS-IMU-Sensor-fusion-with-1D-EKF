#include <matrix.h>
#include <nav.h>
#include <ahrs.h>	
#include <string.h>
#include <math.h>

static float q[4] = {1.0f,0.0f,0.0f,0.0f};
static float integralFBx = 0.0f;
static float integralFBy = 0.0f;
static float integralFBz = 0.0f;          // integral error terms scaled by Ki
static float R_mat[9];

static struct kelman struct_north,struct_east,struct_alt;
static hpf lpf_n,lpf_e,lpf_d;
struct data estimated_data;
struct cfg ekf_cfg;
char quat_updated;
float dt_ahrs;
float freq_ahrs;

void ahrs_init(void)
{
	float d;
	dt_ahrs = 0.005f;
	
	ekf_cfg.standerddeviation_latlon = 2.5f;
	ekf_cfg.standerddeviation_alt    = 1.518522f;
	ekf_cfg.standerddeviation_accl_N = Actual_gravity * 0.015;
	ekf_cfg.standerddeviation_accl_E = Actual_gravity * 0.015;
	ekf_cfg.standerddeviation_accl_Z = Actual_gravity * 0.015;
	
	//latitude to meters
	d = latlon_toDistance(0.0f,0.0f,struct_home_data.Lat_home,0.0f);		
	if(struct_home_data.Lat_home<0.0)
		d*=-1;
	struct_north.cur_state[0][0] = d;
	struct_north.cur_state[1][0] = 0.0f; 	 //ekf_cfg velocity
	struct_north.Q[0][0] = ekf_cfg.standerddeviation_accl_N * ekf_cfg.standerddeviation_accl_N;
	struct_north.Q[1][1] = ekf_cfg.standerddeviation_accl_N * ekf_cfg.standerddeviation_accl_N;
	struct_north.R[0][0] = ekf_cfg.standerddeviation_latlon * ekf_cfg.standerddeviation_latlon;
	struct_north.R[1][1] = ekf_cfg.standerddeviation_latlon * ekf_cfg.standerddeviation_latlon;
	struct_north.H[0][0] = 1.0;
	struct_north.H[1][1] = 1.0;
	struct_north.P[0][0] = 1.0;
	struct_north.P[1][1] = 1.0;
	struct_north.A[0][0] = 1.0;
	struct_north.A[0][1] = dt_ahrs;
	struct_north.A[1][0] = 0;
	struct_north.A[1][1] = 1.0;	
	struct_north.B[0][0] = 0.5*dt_ahrs*dt_ahrs;
	struct_north.B[1][0] = dt_ahrs;

	//longitude to meters
	d = latlon_toDistance(0.0f,0.0f,0.0f,struct_home_data.Lon_home);
	if(struct_home_data.Lon_home<0.0)
		d*=-1;
	struct_east.cur_state[0][0] = d;   //ekf_cfg position
	struct_east.cur_state[1][0] = 0.0f;    //ekf_cfg velocity	
	struct_east.Q[0][0] = ekf_cfg.standerddeviation_accl_E * ekf_cfg.standerddeviation_accl_E;
	struct_east.Q[1][1] = ekf_cfg.standerddeviation_accl_E * ekf_cfg.standerddeviation_accl_E;
	struct_east.R[0][0] = ekf_cfg.standerddeviation_latlon * ekf_cfg.standerddeviation_latlon;
	struct_east.R[1][1] = ekf_cfg.standerddeviation_latlon * ekf_cfg.standerddeviation_latlon;
	struct_east.H[0][0] = 1.0;
	struct_east.H[1][1]	= 1.0;
	struct_east.P[0][0] = 1.0;
	struct_east.P[1][1]	= 1.0;
	struct_east.A[0][0]	= 1.0;
	struct_east.A[0][1]	= dt_ahrs;
	struct_east.A[1][0]	= 0.0;
	struct_east.A[1][1]	= 1.0;	
	struct_east.B[0][0]	= 0.5*dt_ahrs*dt_ahrs;
	struct_east.B[1][0]	= dt_ahrs;
 
	//altitude in meter
	struct_alt.cur_state[0][0]  = struct_home_data.alt_home;   //ekf_cfg position
	struct_alt.cur_state[1][0]  = 1.0f;            //ekf_cfg velocity
	struct_alt.Q[0][0] 	= ekf_cfg.standerddeviation_accl_Z * ekf_cfg.standerddeviation_accl_Z;
	struct_alt.Q[1][1] 	= ekf_cfg.standerddeviation_accl_Z * ekf_cfg.standerddeviation_accl_Z;
	struct_alt.R[0][0] 	= ekf_cfg.standerddeviation_alt * ekf_cfg.standerddeviation_alt;
	struct_alt.R[1][1] 	= ekf_cfg.standerddeviation_alt * ekf_cfg.standerddeviation_alt;
	struct_alt.H[0][0] 	= 1.0;
	struct_alt.H[1][1]	= 1.0;
	struct_alt.P[0][0]  = 1.0;
	struct_alt.P[1][1]	= 1.0;
	struct_alt.A[0][0] 	= 1.0;
	struct_alt.A[0][1] 	= dt_ahrs;
	struct_alt.A[1][0] 	= 0;
	struct_alt.A[1][1] 	= 1.0;	
	struct_alt.B[0][0]	= 0.5*dt_ahrs*dt_ahrs;
	struct_alt.B[1][0]	= dt_ahrs;
	
	lpf_n.c1 = 0.062;
	lpf_n.c2 = 1.25e-6;
	lpf_e.c1 = 0.062;
	lpf_e.c2 = 1.25e-6;
	lpf_d.c1 = 0.025;
    lpf_d.c2 = 1.25e-6;
}


void ahrs_update(struct kelman *updateThis,double pos_This,double vel_This)
{
	double temp_y[2][1] = {0.0,0.0};
	double temp_s[2][2] = {0.0,0.0,0.0,0.0};
	double s_inv[2][2] = {0.0,0.0,0.0,0.0};
	
	updateThis->z[0][0] = pos_This;
	updateThis->z[1][0] = vel_This;
	
	updateThis->R[0][0] = ekf_cfg.pos_error * ekf_cfg.pos_error;	
	updateThis->R[1][1] = ekf_cfg.vel_error * ekf_cfg.vel_error;
	
	temp_y[0][0] = updateThis->z[0][0] - updateThis->cur_state[0][0];
	temp_y[1][0] = updateThis->z[1][0] - updateThis->cur_state[1][0];
	
	temp_s[0][0] = updateThis->P[0][0] + updateThis->R[0][0]; 
	temp_s[0][1] = updateThis->P[0][1] + updateThis->R[0][1];
	temp_s[1][0] = updateThis->P[1][0] + updateThis->R[1][0];
	temp_s[1][1] = updateThis->P[1][1] + updateThis->R[1][1];
	
	double fo = (temp_s[0][0]*temp_s[1][1]) - (temp_s[0][1]*temp_s[1][0]);
	if(fo==0.0) 
	{
		printf("No inverse for S\n");
		return;
	}
	double det = 1.0/fo;
	s_inv[0][0] = temp_s[1][1]*det;
	s_inv[0][1] = -temp_s[0][1]*det;
	s_inv[1][0] = -temp_s[1][0]*det;
	s_inv[1][1] = temp_s[0][0]*det;
	
	//kelman gain
	updateThis->K[0][0] = updateThis->P[0][0] * s_inv[0][0] + updateThis->P[0][1] * s_inv[1][0];
	updateThis->K[0][1] = updateThis->P[0][0] * s_inv[0][1] + updateThis->P[0][1] * s_inv[1][1];
	updateThis->K[1][0] = updateThis->P[1][0] * s_inv[0][0] + updateThis->P[1][1] * s_inv[1][0];
	updateThis->K[1][1] = updateThis->P[1][0] * s_inv[0][1] + updateThis->P[1][1] * s_inv[1][1];
																					 
	updateThis->cur_state[0][0] = updateThis->cur_state[0][0] + (updateThis->K[0][0]*temp_y[0][0] + updateThis->K[0][1]*temp_y[1][0]);
	updateThis->cur_state[1][0] = updateThis->cur_state[1][0] + (updateThis->K[1][0]*temp_y[0][0] + updateThis->K[1][1]*temp_y[1][0]);

	temp_s[0][0] = (1.0f-updateThis->K[0][0]);
	temp_s[0][1] = (0.0f-updateThis->K[0][1]);
	temp_s[1][0] = (0.0f-updateThis->K[1][0]);
	temp_s[1][1] = (1.0f-updateThis->K[1][1]);

	updateThis->P[0][0] = temp_s[0][0]*updateThis->P[0][0] + temp_s[0][1]*updateThis->P[1][0];
	updateThis->P[0][1] = temp_s[0][0]*updateThis->P[0][1] + temp_s[0][1]*updateThis->P[1][1];
	updateThis->P[1][0] = temp_s[1][0]*updateThis->P[0][0] + temp_s[1][1]*updateThis->P[1][0];
	updateThis->P[1][1] = temp_s[1][0]*updateThis->P[0][1] + temp_s[1][1]*updateThis->P[1][1];
}

void ahrs_predict(struct kelman *predictThis,double accl_toaxis,float dt)
{
	double AX[2][1] = {0.0,0.0};
	double mul_AP[2][2] = {0.0,0.0,0.0,0.0};
	
	predictThis->B[0][0] = 0.5 * (double)(dt * dt);
	predictThis->B[1][0] = (double)dt; 
	
	predictThis->A[0][0] = 1.0;
	predictThis->A[0][1] = (double)dt;
	predictThis->A[1][0] = 0.0;
	predictThis->A[1][1] = 1.0;
	
	predictThis->u[0][0] = accl_toaxis;
	
	//2x1 matrix	, 2x2 * 2x1 = 2x1
	//State*A+B*u
	AX[0][0] = predictThis->A[0][0]*predictThis->cur_state[0][0] + predictThis->A[0][1]*predictThis->cur_state[1][0];
	AX[1][0] = predictThis->A[1][0]*predictThis->cur_state[0][0] + predictThis->A[1][1]*predictThis->cur_state[1][0];
		
	//2x1 * 1x1 = 2x1
	predictThis->cur_state[0][0] = AX[0][0] + (predictThis->B[0][0]*predictThis->u[0][0]);
	predictThis->cur_state[1][0] = AX[1][0] + (predictThis->B[1][0]*predictThis->u[0][0]);
	//2x2 matrix
	//((A*P)*A^t)+Q
	mul_AP[0][0] = predictThis->A[0][0]*predictThis->P[0][0] + predictThis->A[0][1]*predictThis->P[1][0];
	mul_AP[0][1] = predictThis->A[0][0]*predictThis->P[0][1] + predictThis->A[0][1]*predictThis->P[1][1];
	mul_AP[1][0] = predictThis->A[1][0]*predictThis->P[0][0] + predictThis->A[1][1]*predictThis->P[1][0];
	mul_AP[1][1] = predictThis->A[1][0]*predictThis->P[0][1] + predictThis->A[1][1]*predictThis->P[1][1];

	predictThis->P[0][0] = (mul_AP[0][0]*predictThis->A[0][0] + mul_AP[0][1]*predictThis->A[0][1]) + predictThis->Q[0][0];
  predictThis->P[0][1] = (mul_AP[0][0]*predictThis->A[1][0] + mul_AP[0][1]*predictThis->A[1][1]) + predictThis->Q[0][1];
  predictThis->P[1][0] = (mul_AP[1][0]*predictThis->A[0][0] + mul_AP[1][1]*predictThis->A[0][1]) + predictThis->Q[1][0];
  predictThis->P[1][1] = (mul_AP[1][0]*predictThis->A[1][0] + mul_AP[1][1]*predictThis->A[1][1]) + predictThis->Q[1][1];
}
/******************************
sample_flag = 0x01 - gps data present
sample_flag = 0x04 - estimation done
Accl[3] - calibrated raw accelerometer data in bodyframe
gps_latlon[3] - lat_deg,lon_deg,altitude_meter(baro/gps)
gps_vel_ned[3] - ned velocity from gps

# Calling freq same as IMU data read freq 
******************************/
void loop(uint8_t sample_flag,float *Accl, double *gps_latlon, double *gps_vel_ned, float pos_dev)
{
	(void)argument;
	static uint32_t us;
	double d;
	static double pos_ned[3];
	static double avg_posNed[3][10];
	static double pos_sum[3];
	
	int cnt=0,i=0;
		
	dt_ahrs = (get_us() - us)*1e-6f;
	us = get_us(); 
		
	freq_ahrs = 1/dt_ahrs;
		
		if(quat_updated)
		{
			quat_updated = 0;
			sample_flag &= 0xFB;
		
			get_accl_ned(&estimated_data,Accl);
			
			ahrs_predict(&struct_north,estimated_data.E_AccNED[0],dt_ahrs);
			ahrs_predict(&struct_east,estimated_data.E_AccNED[1],dt_ahrs);
			ahrs_predict(&struct_alt,estimated_data.E_AccNED[2],dt_ahrs);
		
			if(sample_flag & 0x01)
			{
				  sample_flag = 0;
				  ekf_cfg.pos_error = pos_dev;
				  ekf_cfg.vel_error = 0.15f+ekf_cfg.pos_error/100.0f;
				
					d = latlon_toDistance(gps_latlon[0],0.0,0.0,0.0);	
					if(struct_gps_data.Latitude<0.0) d*=-1;
					ahrs_update(&struct_north,d,gps_vel_ned[0]);
				
					d = latlon_toDistance(0.0,gps_latlon[1],0.0,0.0);
					if(struct_gps_data.Longitude<0.0) d*=-1;
					ahrs_update(&struct_east,d,gps_vel_ned[1]);
				
					d = gps_latlon[2];
					ahrs_update(&struct_alt,d,gps_vel_ned[2]);
			}

		mtrsToGeopoint(struct_north.cur_state[0][0],struct_east.cur_state[0][0],pos_ned);
					
		pos_ned[2] = struct_alt.cur_state[0][0];

		estimated_data.E_VelNED[0] = doResonantLPF(&lpf_n,struct_north.cur_state[1][0]);
		estimated_data.E_VelNED[1] = doResonantLPF(&lpf_e,struct_east.cur_state[1][0]);
		estimated_data.E_VelNED[2] = doResonantLPF(&lpf_d,struct_alt.cur_state[1][0]);
			
		double v_north = estimated_data.E_VelNED[0] * cos(struct_imu_abs.Angles_IMU[Yaw]*DEG_2rad);
		double v_east = estimated_data.E_VelNED[1] * sin(struct_imu_abs.Angles_IMU[Yaw]*DEG_2rad);
		estimated_data.E_speed = sqrt(v_north * v_north + v_east * v_east);
		
		avg_posNed[0][cnt] = pos_ned[0];
		avg_posNed[1][cnt] = pos_ned[1];
		avg_posNed[2][cnt] = pos_ned[2];
		cnt = (cnt+1)%10;

		pos_sum[0] = 0; pos_sum[1] = 0; pos_sum[2] = 0;
		for(i=0;i<10;i++)
		{
			pos_sum[0] += avg_posNed[0][i];
			pos_sum[1] += avg_posNed[1][i];
			pos_sum[2] += avg_posNed[2][i];
		}
				
		estimated_data.E_LatLon[0] = pos_sum[0]/10.0;
		estimated_data.E_LatLon[1] = pos_sum[1]/10.0;
		estimated_data.E_alt = pos_sum[2]/10.0;
		
		sample_flag |= 0x04;
	 }
}

void update_mahony_quat(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz)
{
	float recipNorm;
  float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;  
	float hx, hy, bx, bz;
	float halfvx, halfvy, halfvz, halfwx, halfwy, halfwz;
	float halfex, halfey, halfez;
	float qa, qb, qc;

	// Normalise accelerometer measurement
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;     

		// Normalise magnetometer measurement
		recipNorm = invSqrt(mx * mx + my * my + mz * mz);
		mx *= recipNorm;
		my *= recipNorm;
		mz *= recipNorm;   

    // Auxiliary variables to avoid repeated arithmetic
    q0q0 = q[0] * q[0];
    q0q1 = q[0] * q[1];
    q0q2 = q[0] * q[2];
    q0q3 = q[0] * q[3];
    q1q1 = q[1] * q[1];
    q1q2 = q[1] * q[2];
    q1q3 = q[1] * q[3];
    q2q2 = q[2] * q[2];
    q2q3 = q[2] * q[3];
    q3q3 = q[3] * q[3];   

    // Reference direction of Earth's magnetic field
    hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
    hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
    bx = sqrt(hx * hx + hy * hy);
    bz = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));

		// Estimated direction of gravity and magnetic field
		halfvx = q1q3 - q0q2;
		halfvy = q0q1 + q2q3;
		halfvz = q0q0 - 0.5f + q3q3;
    halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
    halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
    halfwz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);  

		// Error is sum of cross product between estimated direction and measured direction of field vectors
		halfex = (ay * halfvz - az * halfvy) + (my * halfwz - mz * halfwy);
		halfey = (az * halfvx - ax * halfvz) + (mz * halfwx - mx * halfwz);
		halfez = (ax * halfvy - ay * halfvx) + (mx * halfwy - my * halfwx);

		if(twoKi > 0.0f) 
		{
			integralFBx += twoKi * halfex * dt_control;	// integral error scaled by Ki
			integralFBy += twoKi * halfey * dt_control;
			integralFBz += twoKi * halfez * dt_control;
			gx += integralFBx;	// apply integral feedback
			gy += integralFBy;
			gz += integralFBz;
		}
		else 
		{
			integralFBx = 0.0f;	// prevent integral windup
			integralFBy = 0.0f;
			integralFBz = 0.0f;
		}

		// Apply proportional feedback
		gx += twoKp * halfex;
		gy += twoKp * halfey;
		gz += twoKp * halfez;
	
    // Integrate rate of change of quaternion
    gx *= (0.5f * dt_control);		// pre-multiply common factors
    gy *= (0.5f * dt_control);
    gz *= (0.5f * dt_control);
    qa = q[0];
    qb = q[1];
    qc = q[2];
    q[0] += (-qb * gx - qc * gy - q[3] * gz);
    q[1] += (qa * gx + qc * gz - q[3] * gy);
    q[2] += (qa * gy - qb * gz + q[3] * gx);
    q[3] += (qa * gz + qb * gy - qc * gx); 
    
    // Normalise quaternion
    recipNorm = invSqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    q[0] *= recipNorm;
    q[1] *= recipNorm;
    q[2] *= recipNorm;
    q[3] *= recipNorm;
    
	quat_updated = 1;
}


void update_quat2_eul(float *euler)
{//w, x, y, z
  R_mat[0] = 1.0f - 2.0F * (q[2]*q[2] + q[3]*q[3]);
  R_mat[1] = 2.0F * (q[1]*q[2] - q[0]*q[3]);
  R_mat[2] = 2.0F * (q[0]*q[2] + q[1]*q[3]);
  R_mat[3] = 2.0F * (q[1]*q[2] + q[0]*q[3]);
  R_mat[4] = 1.0f - 2.0F * (q[1]*q[1] + q[3]*q[3]);
  R_mat[5] = 2.0F * (q[2]*q[3] - q[0]*q[1]);
  R_mat[6] = 2.0F * (q[1]*q[3] - q[0]*q[2]);
  R_mat[7] = 2.0F * (q[0]*q[1] + q[2]*q[3]); 
  R_mat[8] = 1.0f - 2.0F * (q[1]*q[1] + q[2]*q[2]);

  euler[0] = atan2f(R_mat[7], R_mat[8]);
  euler[1] = -asin(R_mat[6]);
  euler[2] = atan2f(R_mat[3], R_mat[0]);

  if (euler[2] < 0.0f)
    euler[2] += Pi_2;

}

void NED_2Body(float *ned, float *body, float yaw_rad)
{
	float vel_body[3];
	vel_body[0] = ned[0]*cos(struct_imu_abs.Angles_IMU[Yaw]*DEG_2rad)+ned[1]*sin(struct_imu_abs.Angles_IMU[Yaw]*DEG_2rad);
	vel_body[1] = -ned[0]*sin(struct_imu_abs.Angles_IMU[Yaw]*DEG_2rad)+ned[1]*cos(struct_imu_abs.Angles_IMU[Yaw]*DEG_2rad);
	vel_body[2] = ned[2];
	
    if (isnan(vel_body[0]) || isinf(vel_body[0]) || isnan(vel_body[1]) || isinf(vel_body[1]) || isnan(vel_body[2]) || isinf(vel_body[2]))  
	{
      vel_body[0] = 0; vel_body[1] = 0; vel_body[2] = 0;
    }
	
	body[0] = vel_body[0];
	body[1] = vel_body[1];
	body[2] = vel_body[2];
}
void acc_relative(struct data *data_rel,float *accl)
{
	float Acc[3]={0.0f,0.0f,0.0f};
	
	/*remove gravity vector to get actual accelaration*/
	Acc[0] = -1 * (accl[0] - (2.0f * (q[0]*q[2] - q[3]*q[1])));          //forward
	Acc[1] = -1 * (accl[1] - (2.0f * (q[0]*q[1] + q[2]*q[3])));	         //right
	Acc[2] = -1 * (accl[2] - (1.0f - 2.0f * (q[1]*q[1]* + q[2]*q[2])));	 //upword

	data_rel->Acc_forward = (cos(struct_imu_abs.Angles_IMU[Pitch]*DEG_2rad) * Acc[0] -
															sin(struct_imu_abs.Angles_IMU[Pitch]*DEG_2rad) * Acc[2]);
		
	data_rel->Acc_upword = (cos(struct_imu_abs.Angles_IMU[Pitch]*DEG_2rad) * Acc[2] +
															sin(struct_imu_abs.Angles_IMU[Pitch]*DEG_2rad) * Acc[0]);

}
void get_accl_ned(struct data *data_abs, float *accl)
{
	float norm_accl[3];
	static float accelerationEast ;
	static float accelerationNorth;
    static float accelerationUp   ;
	
	float Norm = invSqrt(accl[0]*accl[0]+accl[1]*accl[1]+accl[2]*accl[2]);
	
	norm_accl[0] = Norm*accl[0];
	norm_accl[1] = Norm*accl[1];
	norm_accl[2] = Norm*accl[2];
	
// tilt-compensated 
	accelerationNorth = R_mat[0]*norm_accl[0]+R_mat[1]*norm_accl[1]+R_mat[2]*norm_accl[2];
    accelerationEast = R_mat[3]*norm_accl[0]+R_mat[4]*norm_accl[1]+R_mat[5]*norm_accl[2];
	accelerationUp = R_mat[6]*norm_accl[0]+R_mat[7]*norm_accl[1]+R_mat[8]*norm_accl[2];
//remove gravity
	accelerationNorth -= 0.0f;
	accelerationEast -= 0.0f;
	accelerationUp -= 1.0f;
	
    float sinMagneticDeclination = sin(Declination);
	float easternNorthComponent = sinMagneticDeclination * accelerationEast;
	float northernEasternComponent = -sinMagneticDeclination * accelerationNorth;

	accelerationNorth += northernEasternComponent;
	accelerationEast += easternNorthComponent;
//linear acceleration	
	data_abs->E_AccNED[0] = Actual_gravity * accelerationNorth;
	data_abs->E_AccNED[1] = Actual_gravity * accelerationEast;
	data_abs->E_AccNED[2] = Actual_gravity * accelerationUp;	

}

double doResonantHPF(hpf *cfg,double input)
{
    cfg->lastOutput += cfg->momentum - cfg->lastInput + input;
    cfg->lastInput = input;
	
    cfg->momentum = cfg->momentum * cfg->c1 - cfg->lastOutput * cfg->c2; 
    return cfg->lastOutput;
}

double doResonantLPF(hpf *cfg,double input) 
{
    double distanceToGo = input - cfg->lastOutput;
    cfg->momentum += distanceToGo * cfg->c2; 
    return cfg->lastOutput += cfg->momentum + distanceToGo * cfg->c1; 
}

double get_est_lat(void)
{
	static double lat;
	if(sample_flag & 0x04)
		lat = estimated_data.E_LatLon[0];
	return lat;
}
double get_est_lon(void)
{
	static double lon;
	if(sample_flag & 0x04)
		 lon = estimated_data.E_LatLon[1];
	return lon;
}
float get_est_alt(void)
{
	static float alt;
	if(sample_flag & 0x04)
		 alt = estimated_data.E_alt;	
	return alt;
}
float get_est_velN(void)
{
	static float vel_N;
	if(sample_flag & 0x04)
		vel_N = estimated_data.E_VelNED[0];
	return vel_N;
}
float get_est_velE(void)
{
	static float vel_E;
	if(sample_flag & 0x04)
		vel_E = estimated_data.E_VelNED[1];

	return vel_E;
}
float get_est_velD(void)
{
	static float vel_d;
	if(sample_flag & 0x04)
			vel_d = estimated_data.E_VelNED[2];
	return vel_d;
}

float get_est_velR(void)
{
	static float vel_r;
	if(sample_flag & 0x04)
		vel_r = estimated_data.E_speed;
	return vel_r;
}

float get_est_acclN(void)
{
	static float accl_N;
	if(sample_flag & 0x04)
		accl_N = estimated_data.E_AccNED[0];
	return accl_N;
}
float get_est_acclE(void)
{
	static float accl_E;
	if(sample_flag & 0x04)
		accl_E = estimated_data.E_AccNED[1];
	return accl_E;
}
float get_est_acclD(void)
{
	static float accl_D;
	if(sample_flag & 0x04)
		accl_D = estimated_data.E_AccNED[2];
	return accl_D;
}
