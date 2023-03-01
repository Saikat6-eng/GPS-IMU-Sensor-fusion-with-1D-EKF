#include <nav.h>
#include <ahrs.h>

bool gps_flag = false;

int main()
{
	double gps_coord[3];
	float gps_vel_ned[3];
	float pos_dev;
	float accl[3],gyro_rad[3],mag[3];
	float euler_rad[3];
	bool sample_flag;
	ahrs_init();
	
	while(1)
	{
		//get imu_data	
		//must calibrated and filterd accl and gyro data
		
		update_mahony_quat(accl[0],accl[1],accl[2],gyro_rad[0],gyro_rad[1],gyro_rad[2],mag[1],-mag[0],mag[2]);
		
		update_quat2_eul(euler_deg);
		
		if(gps_flag==true)
		{
			gps_flag = false;
			//get  gps_coord[3] in deg
			//get  gps_vel_ned[3]
			//get  pos_dev;
			sample_flag  = true
		}
		else
		{
			sample_flag  = false;
		}
		
		loop(sample_flag,accl,gps_coord,gps_vel_ned,pos_dev);
		
		printf("lat - %lf",get_est_lat());
	    printf("lon - %lf",get_est_lon());
		printf("alt - %lf\n",get_est_alt());
		printf("vel_N - %f",get_est_velN());
		printf("vel_E - %f",get_est_velE());
		printf("vel_D - %f",get_est_velD());
		printf("vel_R - %f\n",get_est_velR());
		printf("accl_N - %f",get_est_acclN());
		printf("accl_E - %f",get_est_acclE());
		printf("accl_D - %f\n",get_est_acclD());
	
	
	    msleep(10);
	}
	
	
	return 0;
}