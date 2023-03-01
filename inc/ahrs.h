#ifndef _ahrs_h
#define _ahrs_h


#define twoKp 1.0f		      // 2 * proportional gain (Kp)
#define twoKi 1e-6f
#define RAD_2deg 57.29577951f
#define DEG_2rad 0.017453296f
#define Pi 3.14159265359f
#define Pi_2 6.2831853f
#define Rad_earth 6378137		//in mtr
#define Actual_gravity 9.81f

struct kelman
{
	double cur_state[2][1];	
	double u[1][1];					//input control matrix
	double z[2][1];					//input measurment matrix
	double K[2][2];                 //kelman gain
	double H[2][2];					//transformation matrix
	double P[2][2];					//covarience matrix
	double Q[2][2];					//process error varience
	double R[2][2];					//measurement error varience
	double A[2][2];					//state transition matrix
	double B[2][1];					//control matrix
};

struct data
{
	double E_LatLon[2];
	double E_AccNED[3];
	double E_VelNED[3];
	double E_speed;
	double E_alt;
	double Acc_forward;
	double Acc_upword;
};

struct cfg
{
	double standerddeviation_latlon,
				 standerddeviation_alt,
				 standerddeviation_accl_N,
				 standerddeviation_accl_E,
				 standerddeviation_accl_Z;	
	float pos_error,alt_error,vel_error;
	float ground_vel[3];
};

typedef struct filt
{
	double c1;
	double c2;

	double lastInput;
	double lastOutput;
	double momentum;
}hpf;

void ahrs_init(void);
void ahrs_update(struct kelman *updateThis,double pos_This,double vel_This);
void ahrs_predict(struct kelman *predictThis,double accl_toaxis,float dt);

void accl_relative(struct data *,float *accl);
void get_accl_ned(struct data *,float *accl);
void loop(uint8_t sample_flag,float *Accl,double *gps_latlon, double *gps_vel_ned,float pos_dev);
void update_mahony_quat(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz);
void update_quat2_eul(float *eul);
void NED_2Body(float *ned, float *body, float yaw_rad);
double doResonantHPF(hpf *cfg,double input);
double doResonantLPF(hpf *cfg,double input) ;
double get_est_lat(void);
double get_est_lon(void);
float get_est_alt(void);
float get_est_velN(void);
float get_est_velE(void);
float get_est_velD(void);
float get_est_velR(void);
float get_est_acclN(void);
float get_est_acclE(void);
float get_est_acclD(void);

#endif
