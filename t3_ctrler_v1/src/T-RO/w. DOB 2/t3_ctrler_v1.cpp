#include <ros/ros.h>

#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <iostream>
#include <math.h>
#include <chrono>

#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/MultiArrayDimension.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int16.h>

#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>

#include <geometry_msgs/Vector3.h>

//About time==============================================

double freq=200;//controller loop frequency

std::chrono::duration<double> delta_t; 


auto end_vicon  =std::chrono::high_resolution_clock::now();
auto start_vicon=std::chrono::high_resolution_clock::now();
std::chrono::duration<double> delta_t_vicon; 

auto end_imu  =std::chrono::high_resolution_clock::now();
auto start_imu=std::chrono::high_resolution_clock::now();
std::chrono::duration<double> delta_t_imu; 
//--------------------------------------------------------

//Flags===================================================

int imu_recv_flag=0;
int rc_recv_flag=0;
int vicon_recv_flag=0;

int vicon_pos_bias_capture_flag=0;
//--------------------------------------------------------


//Sensor_readings=========================================

//RC_readings
int arr[7];//0:roll, 1:pitch, 2:yaw, 3:thrust, 4:3-step switch
double voltage;//battery voltage
double voltage_old;

//IMU_readings
sensor_msgs::Imu imu_data;
geometry_msgs::Vector3 TGP_rpy;
geometry_msgs::Vector3 TGP_ang_vel;
geometry_msgs::Vector3 TGP_lin_acc;
int yaw_count=0;//counts number of yaw rotation
double yaw_prev=0;//yaw value of previous step

//IMU_NAV_readings
sensor_msgs::Imu imu_nav_data;
geometry_msgs::Vector3 FP_rpy;
geometry_msgs::Vector3 FP_ang_vel;
geometry_msgs::Vector3 FP_lin_acc;
int yaw_nav_count=0;
double yaw_nav_prev=0;

//VICON_readings
geometry_msgs::Vector3 vicon_pos;
geometry_msgs::Vector3 vicon_pos_bias;
geometry_msgs::Vector3 vicon_vel;
geometry_msgs::Vector3 vicon_att;
//--------------------------------------------------------

//Logs====================================================
std_msgs::Float32MultiArray rpyT_cmd;
std_msgs::Float32MultiArray xyz_cmd;
//--------------------------------------------------------

//Commands================================================

//Servo_cmd
double theta_r_s=0;//desired roll servo angle  [DYNAMIXEL XH430-W210-R id-1]
double theta_p_s=0;//desired pitch servo angle [DYNAMIXEL XH430-W210-R id-2]

//Thruster_cmd
double F1=0;//desired propeller 1 force
double F2=0;//desired propeller 2 force
double F3=0;//desired propeller 3 force
double F4=0;//desired propeller 4 force

double tau_r_d=0;//roll  desired torque (N.m)
double tau_p_d=0;//pitch desired torque(N.m)

std_msgs::Int16MultiArray PWMs_cmd;

//Pos_cmd
double X_d=0;
double Y_d=0;
double Z_d=0;

//Acc_cmd
double X_ddot_des=0;
double Y_ddot_des=0;
double Z_ddot_des=0;
double X_ddot_tilde_des=0;
double Y_ddot_tilde_des=0;
double Z_ddot_tilde_des=0;


//ud_cmd
double r_d=0;//desired roll angle
double p_d=0;//desired pitch angle
double y_d=0;//desired yaw angle
double y_d_tangent=0;//yaw increment tangent
double T_d=0;//desired thrust
//--------------------------------------------------------


//T3_V2_parameters========================================

//General dimensions
static double l_arm=0.3035;//(m), [motor center] to [fp center] length
static double mass=(double)3.844;//3.4;//(Kg)
static double r_F=0.05;//0.07;//(m)
static double m_F=2.540645;//(Kg) 

//Multirotor parameters
static double J_T1=0.02;//0.015;
static double J_T2=0.02;//0.015;
static double J_T3=0.1;

static double J_F1=0.03;
static double J_F2=0.03;
static double J_F3=0.3;

//Servo mechnism-Roll(mm)
static double r_r_u=63.25;//[u-joint center] to [servo surface] length(vertical)
static double r_r_s=80;//[u-joint center] to [servo surface] length(horizontal)
static double r_r_h=50;//[servo horn] length
static double r_r_r=58;//62.9;//[push rod] length
static double r_r_t=70;//[u-joint centor] to [lower stem] length(horizontal)

//Servo mechanism-Pitch(mm)
static double r_p_u=52.25;
static double r_p_s=64.75;
static double r_p_h=50;
static double r_p_r=58.7;
static double r_p_t=115;

//Propeller constants(DJI E800(3510 motors + 620S ESCs))
static double b_over_k_ratio=0.01;//F_i=k*(omega_i)^2, M_i=b*(omega_i)^2
//--------------------------------------------------------


//General parameters======================================

static double pi=3.141592;//(rad)
static double g=9.80665;//(m/s^2)

static double XY_ddot_limit=2;//2(m/s^2)
static double rp_limit=0.4;//(rad)
static double rel_atti_limit=0.3;//(rad)
static double y_vel_limit=0.01;//(rad/s)
static double y_d_tangent_deadzone=(double)0.05*y_vel_limit;//(rad/s)
static double T_limit=60;//(N)

static double xy_limit=1.2;//(m)
static double  z_limit=2;//(m)
//--------------------------------------------------------


//Function declaration====================================

void arrayCallback(const std_msgs::Int16MultiArray::ConstPtr &array);
void imu_updateCallback(const sensor_msgs::Imu::ConstPtr &imu);
void imu_nav_updateCallback(const sensor_msgs::Imu::ConstPtr &imu);
void vicon_pos_updateCallback(const geometry_msgs::Vector3::ConstPtr &pos);
void vicon_vel_updateCallback(const geometry_msgs::Vector3::ConstPtr &vel);
void vicon_att_updateCallback(const geometry_msgs::Vector3::ConstPtr &att);
void rel_atti_to_servo_ang(double rel_atti_r, double rel_atti_p);
sensor_msgs::JointState servo_msg_create(double t_r_s, double t_p_s);
void ud_to_PWMs(double tau_r_des, double tau_p_des, double tau_y_des, double Thrust_des);
double Force_to_PWM(double F);
void rpyT_ctrl_manual(double roll_d, double pitch_d, double yaw_d, double Thrust_d);
void rpyT_ctrl_auto(double roll_d, double pitch_d, double yaw_d, double Thrust_d);
//--------------------------------------------------------


//Control gains===========================================

//integratior(PID) limitation
double integ_limit=2;
double XY_integ_limit=2;
double Z_integ_limit=100;

//errors
double e_r=0;
double e_p=0;
double e_y=0;

double e_X=0;
double e_Y=0;
double e_Z=0;

//integrations
double e_r_i=0;//roll error integration
double e_p_i=0;//pitch error integration

double e_X_i=0;
double e_Y_i=0;
double e_Z_i=0;//height error integration


//Roll, Pitch PID gains
double Pa=4;//4
double Ia=1;//0.01
double Da=1;//0.5

//Roll, Pitch PID gains (FP fix mode)
double Pa_fp=4;//3
double Ia_fp=0;//0.3;//1;//0.01
double Da_fp=0.7;//0.5

//Yaw PID gains
double Py=1;//1;
double Dy=0.1;//0.01;

//XYZ PID Gains
double Pxy=3;//5;//5;
double Ixy=0.5;//0;//0;
double Dxy=2;//4;//3;

double Pz=2;
double Iz=1;//0.5;
double Dz=5;

//DOB!!!!!
double fQ_cutoff=8;//50;

//TP roll DOB
double xT_r1=0;
double xT_r2=0;
double xTd_r1=0;
double xTd_r2=0;

double yT_r1=0;
double yT_r2=0;
double yTd_r1=0;
double yTd_r2=0;

double tautilde_r_d=0;

//TP pitch DOB
double xT_p1=0;
double xT_p2=0;
double xTd_p1=0;
double xTd_p2=0;

double yT_p1=0;
double yT_p2=0;
double yTd_p1=0;
double yTd_p2=0;

double tautilde_p_d=0;
//--------------------------------------------------------


//Main====================================================

int main(int argc, char **argv){
	vicon_pos.z=-10000;//Deliverately set wierd z value to check vicon's availability
	
	ros::init(argc, argv, "t3_ctrler");
	ros::NodeHandle nh;

	//Publisher
	ros::Publisher PWMs=nh.advertise<std_msgs::Int16MultiArray>("PWMs", 100);
	ros::Publisher servo_angle=nh.advertise<sensor_msgs::JointState>("goal_dynamixel_position", 100);
	ros::Publisher rpyT_cmd_log=nh.advertise<std_msgs::Float32MultiArray>("rpyT_cmd",100);
	ros::Publisher xyz_cmd_log=nh.advertise<std_msgs::Float32MultiArray>("xyz_cmd",100);

	//Subscriber
	ros::Subscriber RC_readings=nh.subscribe("/RC_readings", 100, &arrayCallback);
	//single imu:microstrain_25.launch, double imu:microstrain_25_nav.launch
	//ros::Subscriber imu_attitude=nh.subscribe("imu/data", 100, &imu_updateCallback);
	ros::Subscriber imu_attitude=nh.subscribe("imu/imu/data", 100, &imu_updateCallback);
	ros::Subscriber imu_nav_attitude=nh.subscribe("nav/imu/data", 100, &imu_nav_updateCallback);
	ros::Subscriber VICON_pos=nh.subscribe("/pos", 100, &vicon_pos_updateCallback);
	ros::Subscriber VICON_vel=nh.subscribe("/vel", 100, &vicon_vel_updateCallback);
	ros::Subscriber VICON_att=nh.subscribe("/att", 100, &vicon_att_updateCallback);

	ros::Rate loop_rate(freq);

	//Initialize
	int flag_imu=0;//monitoring imu's availability
	auto end  =std::chrono::high_resolution_clock::now();
	auto start=std::chrono::high_resolution_clock::now();
	vicon_pos_bias.x=0;
	vicon_pos_bias.y=0;
	vicon_pos_bias.z=0;
			
	PWMs_cmd.data.resize(4);
	rpyT_cmd.data.resize(4);
	xyz_cmd.data.resize(3);

	while(ros::ok()){
		end=std::chrono::high_resolution_clock::now();
		delta_t=end-start; 
		start=std::chrono::high_resolution_clock::now();

		//Kill Mode
		if(arr[5]<1300){
			flag_imu=0;
			vicon_pos_bias_capture_flag=0;

			PWMs_cmd.data[0]=1000;
			PWMs_cmd.data[1]=1000;
			PWMs_cmd.data[2]=1000;
			PWMs_cmd.data[3]=1000;

			rel_atti_to_servo_ang((double)0, (double)0);
		
		//Manual Control Mode
		}else if(arr[5]<1700 && arr[5]>1300){
			//Initialize desired yaw
			if(flag_imu!=1){
				//y_d=vicon_att.z;//initial desired yaw setting
				y_d=vicon_att.z;//initial desired yaw setting
				e_r_i=0;//initialize roll integrator
				e_p_i=0;//initialize pitch integrator
				e_Z_i=0;//initialize hight integrator
	
				if(TGP_lin_acc.z<(double)-9)	flag_imu=1;//exits this block if IMU is on-line
			}
	
			//TGP ctrl
			r_d=rp_limit*((arr[0]-(double)1500)/(double)500);
			p_d=rp_limit*((arr[1]-(double)1500)/(double)500);
			y_d_tangent=y_vel_limit*((arr[2]-(double)1500)/(double)500);
			if(fabs(y_d_tangent)<y_d_tangent_deadzone || fabs(y_d_tangent)>y_vel_limit) y_d_tangent=0;
			y_d+=y_d_tangent;
	
			//Manual thrust	
			//T_d=T_limit*((arr[3]-(double)1500)/(double)500);
			
			//Vicon based height contol
			Z_d=-z_limit*((arr[3]-(double)1400)/(double)500);
			e_Z=Z_d-vicon_pos.z;
			if(vicon_recv_flag==1){
				e_Z_i=e_Z_i+e_Z*delta_t.count();
				vicon_recv_flag=0;
			}
			if(fabs(e_Z_i)>Z_integ_limit)	e_Z_i=(e_Z_i/fabs(e_Z_i))*Z_integ_limit;
			double Z_ddot_tilde_des=Pz*e_Z+Iz*e_Z_i-Dz*vicon_vel.z-g;
			T_d=-(mass*Z_ddot_tilde_des)/(cos(TGP_rpy.x)*cos(TGP_rpy.y));

			//Generate motor cmd
			rpyT_ctrl_manual(r_d, p_d, y_d, T_d);	
			rpyT_cmd.data[0]=r_d;
			rpyT_cmd.data[1]=p_d;
			rpyT_cmd.data[2]=y_d;
			rpyT_cmd.data[3]=T_d;
			//std::cout<<r_d<<" "<<p_d<<" "<<y_d<<" "<<T_d<<std::endl;			

			//FP ctrl, arr[4] controls FP function on/off
			if(arr[4]>=1500)	rel_atti_to_servo_ang(TGP_rpy.x,-TGP_rpy.y);	
			else			rel_atti_to_servo_ang((double)0,(double)0);	
		
		//Auto Flight Mode
		}else{	
			//Initialize desired yaw
			if(flag_imu!=1){
				y_d=vicon_att.z;//initial desired yaw setting
				e_r_i=0;//initialize roll integrator
				e_p_i=0;//initialize pitch integrator

				e_X_i=0;
				e_Y_i=0;
				e_Z_i=0;//initialize hight integrator

				xT_r1=0;
				xT_r2=0;
				tautilde_r_d=0;
	
				xT_p1=0;
				xT_p2=0;
				tautilde_p_d=0;	

				if(TGP_lin_acc.z<(double)-9)	flag_imu=1;
			}

			//Position Control
			X_d=-xy_limit*((arr[1]-(double)1500)/(double)500);
			Y_d=xy_limit*((arr[0]-(double)1500)/(double)500);
			Z_d=-z_limit*((arr[3]-(double)1400)/(double)500);
			xyz_cmd.data[0]=X_d;
			xyz_cmd.data[1]=Y_d;
			xyz_cmd.data[2]=Z_d;

			if(Z_d>=1)	Z_d=1;
			e_X=0-vicon_pos.x;
			//e_X=X_d-vicon_pos.x;
			e_Y=0-vicon_pos.y;
			//e_Y=Y_d-vicon_pos.y;
			e_Z=Z_d-vicon_pos.z;
			
			if(vicon_recv_flag==1){
				e_X_i=e_X_i+e_X*delta_t.count();
				e_Y_i=e_Y_i+e_Y*delta_t.count();
				e_Z_i=e_Z_i+e_Z*delta_t.count();
				vicon_recv_flag=0;
			}
			if(fabs(e_X_i)>XY_integ_limit)	e_X_i=(e_X_i/fabs(e_X_i))*XY_integ_limit;
			if(fabs(e_Y_i)>XY_integ_limit)	e_Y_i=(e_Y_i/fabs(e_Y_i))*XY_integ_limit;
			if(fabs(e_Z_i)> Z_integ_limit)	e_Z_i=(e_Z_i/fabs(e_Z_i))* Z_integ_limit;

			//TGP acceleration ctrl
			X_ddot_des=Pxy*e_X+Ixy*e_X_i-Dxy*vicon_vel.x;
			Y_ddot_des=Pxy*e_Y+Ixy*e_Y_i-Dxy*vicon_vel.y;
			Z_ddot_des=Pz *e_Z+Iz *e_Z_i-Dz *vicon_vel.z;		
			//std::cout<<X_ddot_des<<"\t"<<Y_ddot_des<<std::endl;
			if(fabs(X_ddot_des)>XY_ddot_limit)	X_ddot_des=(X_ddot_des/fabs(X_ddot_des))*XY_ddot_limit;
			if(fabs(Y_ddot_des)>XY_ddot_limit)	Y_ddot_des=(Y_ddot_des/fabs(Y_ddot_des))*XY_ddot_limit;
			//std::cout<<X_ddot_des<<"\t"<<Y_ddot_des<<std::endl;
			X_ddot_tilde_des=cos(-vicon_att.z)*X_ddot_des-sin(-vicon_att.z)*Y_ddot_des;
			Y_ddot_tilde_des=sin(-vicon_att.z)*X_ddot_des+cos(-vicon_att.z)*Y_ddot_des;
			Z_ddot_tilde_des=Z_ddot_des-g;		
			//std::cout<<X_ddot_tilde_des<<"\t"<<Y_ddot_tilde_des<<std::endl;
			//std::cout<<vicon_att.z<<std::endl;
			//std::cout<<Z_ddot_tilde_des<<std::endl;
	
			//TGP acc to u conversion
			p_d=atan2(-X_ddot_tilde_des,-Z_ddot_tilde_des);
			r_d=atan2((Y_ddot_tilde_des*cos(p_d)),-Z_ddot_tilde_des);
			T_d=-(mass*Z_ddot_tilde_des)/(cos(TGP_rpy.x)*cos(TGP_rpy.y));
			//std::cout<<r_d<<"\t"<<p_d<<std::endl;

			y_d_tangent=y_vel_limit*((arr[2]-(double)1500)/(double)500);
			if(fabs(y_d_tangent)<y_d_tangent_deadzone || fabs(y_d_tangent)>y_vel_limit) y_d_tangent=0;
			y_d+=y_d_tangent;
			
			//Generate motor cmd
			rpyT_ctrl_auto(r_d, p_d, y_d, T_d);	
			rpyT_cmd.data[0]=r_d;
			rpyT_cmd.data[1]=p_d;
			rpyT_cmd.data[2]=y_d;
			rpyT_cmd.data[3]=T_d;		

			//FP ctrl, arr[4] controls FP function on/off
			double phi_Fdes=((double)0.3/(double)500)*(arr[0]-(double)1500);
			double theta_Fdes=((double)0.3/(double)500)*(arr[1]-(double)1500);
			
			if(arr[4]>=1500)	rel_atti_to_servo_ang(phi_Fdes+TGP_rpy.x,theta_Fdes-TGP_rpy.y);	
			else			rel_atti_to_servo_ang((double)0,(double)0);	
		}

		//Publish data
		PWMs.publish(PWMs_cmd);
		servo_angle.publish(servo_msg_create(theta_r_s, theta_p_s));
		rpyT_cmd_log.publish(rpyT_cmd);
		xyz_cmd_log.publish(xyz_cmd);

		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
//--------------------------------------------------------


//Functions===============================================

void arrayCallback(const std_msgs::Int16MultiArray::ConstPtr &array){
	for(int i=0;i<7;i++){
		arr[i]=array->data[i];
	}
	voltage=arr[6]/(double)1000+0.145;
	double kv=0.01;
	voltage=kv*voltage+((double)1-kv)*voltage_old;
	voltage_old=voltage;
	if(voltage>25.2)	voltage=25.2;	//4.2v/cell*6cell
	if(voltage<21)		voltage=21;	//3.5v/cell*6cell
	//std::cout<<arr[0]<<" "<<arr[1]<<" "<<arr[2]<<" "<<arr[3]<<" "<<arr[4]<<" "<<arr[5]<<std::endl;
	//0:roll
	//1:pitch
	//2:yaw
	//3:thrust
	//4:FP_control sw
	//5:Control_mode sw
	
	//raise recv flag
	rc_recv_flag=1;	
	return;
}

void imu_updateCallback(const sensor_msgs::Imu::ConstPtr &imu){
	end_imu  =std::chrono::high_resolution_clock::now();
	delta_t_imu=end_imu-start_imu; 
	start_imu=std::chrono::high_resolution_clock::now();
	//ROS_INFO("timestep:%lf",delta_t_imu.count());

        geometry_msgs::Quaternion q=imu->orientation;

	//roll
        TGP_rpy.x=atan2((q.y*q.z+q.w*q.x),(double)0.5-(q.x*q.x+q.y*q.y));//roll
	
	//pitch        
	double temp_y=(-(double)2*(q.x*q.z-q.w*q.y));
        if(fabs(temp_y)>0.9999) temp_y=(temp_y/fabs(temp_y))*0.9999;
        TGP_rpy.y=asin(temp_y);//pitch 
	
	//yaw
	double temp_z=atan2((q.x*q.y+q.w*q.z),(double)0.5-(q.y*q.y+q.z*q.z));
	if(fabs(temp_z-yaw_prev)>pi){
		if(temp_z>=0)		yaw_count-=1;
		else if(temp_z<0)	yaw_count+=1;
	}		
        TGP_rpy.z=temp_z+(double)2*pi*(double)yaw_count;//yaw
	yaw_prev=temp_z;
	//Ref: http://marc-b-reynolds.github.io/math/2017/04/18/TaitEuler.html
	//ROS_INFO("temp_z:%lf, yaw:%lf",temp_z, TGP_rpy.z);
	
	//angular velocity
	TGP_ang_vel=imu->angular_velocity;

	//linear acceleration
        TGP_lin_acc=imu->linear_acceleration;

	//raise recv_flag
	imu_recv_flag=1;

	return;
}

void imu_nav_updateCallback(const sensor_msgs::Imu::ConstPtr &imu){

        geometry_msgs::Quaternion q=imu->orientation;

	//roll
        FP_rpy.x=atan2((q.y*q.z+q.w*q.x),(double)0.5-(q.x*q.x+q.y*q.y));//roll
	
	//pitch        
	double temp_y=(-(double)2*(q.x*q.z-q.w*q.y));
        if(fabs(temp_y)>0.9999) temp_y=(temp_y/fabs(temp_y))*0.9999;
        FP_rpy.y=asin(temp_y);//pitch 
	
	//yaw
	double temp_z=atan2((q.x*q.y+q.w*q.z),(double)0.5-(q.y*q.y+q.z*q.z));
	if(fabs(temp_z-yaw_nav_prev)>pi){
		if(temp_z>=0)		yaw_nav_count-=1;
		else if(temp_z<0)	yaw_nav_count+=1;
	}		
        FP_rpy.z=temp_z+(double)2*pi*(double)yaw_nav_count;//yaw
	yaw_nav_prev=temp_z;
	//Ref: http://marc-b-reynolds.github.io/math/2017/04/18/TaitEuler.html
	//ROS_INFO("temp_z:%lf, yaw:%lf",temp_z, FP_rpy.z);

	//angular velocity
	FP_ang_vel=imu->angular_velocity;

	//linear acceleration
        FP_lin_acc=imu->linear_acceleration;

	return;
}

void vicon_pos_updateCallback(const geometry_msgs::Vector3::ConstPtr &pos){
	end_vicon =std::chrono::high_resolution_clock::now();
	delta_t_vicon=end_vicon-start_vicon;
	start_vicon=std::chrono::high_resolution_clock::now();
 
	if(vicon_pos.z!=-10000 && vicon_pos_bias_capture_flag!=1){
		vicon_pos_bias.x=(float)0.001*pos->x;
		vicon_pos_bias.y=(float)0.001*pos->y;
		vicon_pos_bias.z=(float)0.001*pos->z;
		vicon_pos_bias_capture_flag=1;
	}else{	

		vicon_pos.x=(float)0.001*pos->x-vicon_pos_bias.x;
		vicon_pos.y=(float)0.001*pos->y-vicon_pos_bias.y;
		vicon_pos.z=(float)0.001*pos->z-vicon_pos_bias.z;
	}
	
	//raise recv_flag
	vicon_recv_flag=1;

	return;
}

void vicon_vel_updateCallback(const geometry_msgs::Vector3::ConstPtr &vel){
	vicon_vel.x=(float)0.001*vel->x;
	vicon_vel.y=(float)0.001*vel->y;
	vicon_vel.z=(float)0.001*vel->z;
	
	return;
}

void vicon_att_updateCallback(const geometry_msgs::Vector3::ConstPtr &att){
	vicon_att.x=att->x;
	vicon_att.y=att->y;
	vicon_att.z=att->z;
	
	return;
}

void rel_atti_to_servo_ang(double rel_atti_r, double rel_atti_p){

	//Limitation
	if(fabs(rel_atti_r)>=rel_atti_limit)	rel_atti_r=rel_atti_limit*(rel_atti_r/fabs(rel_atti_r));
	if(fabs(rel_atti_p)>=rel_atti_limit)	rel_atti_p=rel_atti_limit*(rel_atti_p/fabs(rel_atti_p));

	//Roll
	double r_r_j=sqrt(pow(r_r_u,2)+pow(r_r_s,2));
	double theta_r_1=atan2(r_r_s,r_r_u);
	double theta_r_2=atan2(r_r_u,r_r_s);
	double k_r=sqrt(pow(r_r_j,2)+pow(r_r_t,2)-2*r_r_j*r_r_t*cos(theta_r_2+rel_atti_r));
	double temp_beta_r=((-pow(r_r_t,2)+pow(r_r_j,2)+pow(k_r,2))/(2*r_r_j*k_r));
	if(fabs(temp_beta_r)>0.9999)    temp_beta_r=(temp_beta_r/fabs(temp_beta_r))*0.9999;
	double beta_r=acos(temp_beta_r);
	double temp_alpha_r=((-pow(k_r,2)+pow(r_r_h,2)+pow(r_r_r,2))/(2*r_r_h*r_r_r));
	if(fabs(temp_alpha_r)>0.9999)   temp_alpha_r=(temp_alpha_r/fabs(temp_alpha_r))*0.9999;
	double alpha_r=acos(temp_alpha_r);
	theta_r_s=asin(r_r_r*sin(alpha_r)/k_r)-theta_r_2-beta_r;

	//Pitch
	double r_p_j=sqrt(pow(r_p_u,2)+pow(r_p_s,2));
	double theta_p_1=atan2(r_p_s,r_p_u);
	double theta_p_2=atan2(r_p_u,r_p_s);
	double k_p=sqrt(pow(r_p_t,2)+pow(r_p_j,2)-2*r_p_t*r_p_j*cos(pi/2-theta_p_1-rel_atti_p));
	double temp_alpha_p=((pow(r_p_j,2)+pow(k_p,2)-pow(r_p_t,2))/(2*r_p_j*k_p));
	if(fabs(temp_alpha_p)>0.9999)   temp_alpha_p=(temp_alpha_p/fabs(temp_alpha_p))*0.9999;
	double alpha_p=acos(temp_alpha_p);
	double beta_p=pi-theta_p_2-alpha_p;
	double temp_theta_p_s=(pow(k_p,2)+pow(r_p_h,2)-pow(r_p_r,2))/(2*k_p*r_p_h);
	if(fabs(temp_theta_p_s)>0.9999) temp_theta_p_s=(temp_theta_p_s/fabs(temp_theta_p_s))*0.9999;
	theta_p_s=acos(temp_theta_p_s)-beta_p;
}

sensor_msgs::JointState servo_msg_create(double t_r_s, double t_p_s){

	sensor_msgs::JointState servo_msg;

	servo_msg.name.resize(2);
	servo_msg.name[0]="id_1";
	servo_msg.name[1]="id_2";

	servo_msg.position.resize(2);
	servo_msg.position[0]=t_r_s;
	servo_msg.position[1]=t_p_s;

	return servo_msg;
}

void rpyT_ctrl_manual(double roll_d, double pitch_d, double yaw_d, double Thrust_d){

	e_r=roll_d-TGP_rpy.x;
	e_p=pitch_d-TGP_rpy.y;
	e_y=yaw_d-vicon_att.z;

	if(imu_recv_flag==1){
		e_r_i+=e_r*delta_t.count();
		e_p_i+=e_p*delta_t.count();
		imu_recv_flag=0;
	}
	if(fabs(e_r_i)>integ_limit)	e_r_i=(e_r_i/fabs(e_r_i))*integ_limit;
	if(fabs(e_p_i)>integ_limit)	e_p_i=(e_p_i/fabs(e_p_i))*integ_limit;

	//PID-FP Ctrl OFF
	if(arr[4]<1500){
		tau_r_d=Pa*e_r+Ia*e_r_i+Da*(-TGP_ang_vel.x)+(double)0.3;
		tau_p_d=Pa*e_p+Ia*e_p_i+Da*(-TGP_ang_vel.y)+(double)0.1;

		tautilde_r_d=tau_r_d;
		tautilde_p_d=tau_p_d;
	}

	//PID-FP Ctrl ON
	else if(arr[4]>=1500){
		tau_r_d=Pa_fp*e_r+Ia_fp*e_r_i+Da_fp*(-TGP_ang_vel.x);//-r_F*m_F*g*TGP_rpy.x+(double)0.3;
		tau_p_d=Pa_fp*e_p+Ia_fp*e_p_i+Da_fp*(-TGP_ang_vel.y);//-r_F*m_F*g*TGP_rpy.y;//+(double)0.2;;

	//TP roll DOB(applied 2nd order butterworth filter)-------------------------
	//Q/Js transfer function to state space
	xTd_r1=-pow(2,0.5)*fQ_cutoff*xT_r1-pow(fQ_cutoff,2)*xT_r2+TGP_ang_vel.x;
	xTd_r2=xT_r1;

	xT_r1=xT_r1+xTd_r1*delta_t.count();
	xT_r2=xT_r2+xTd_r2*delta_t.count();

  	double tauhat_r=J_T1*pow(fQ_cutoff,2)*xT_r1;

    	//Q transfer function to state space
    	yTd_r1=-pow(2,0.5)*fQ_cutoff*yT_r1-pow(fQ_cutoff,2)*yT_r2+tautilde_r_d;
    	yTd_r2=yT_r1;

    	yT_r1=yT_r1+yTd_r1*delta_t.count();
    	yT_r2=yT_r2+yTd_r2*delta_t.count();

	double Qtautilde_r=pow(fQ_cutoff,2)*yT_r2;

	double dhat_r=tauhat_r-Qtautilde_r;

	tautilde_r_d=tau_r_d-dhat_r;
	//--------------------------------------------------------------------------

	//TP pitch DOB(applied 2nd order butterworth filter)-------------------------
	//Q/Js trnsfer function to state space
    xTd_p1=-pow(2,0.5)*fQ_cutoff*xT_p1-pow(fQ_cutoff,2)*xT_p2+TGP_ang_vel.y;
	xTd_p2=xT_p1;

	xT_p1=xT_p1+xTd_p1*delta_t.count();
	xT_p2=xT_p2+xTd_p2*delta_t.count();

	double tauhat_p=J_T2*pow(fQ_cutoff,2)*xT_p1;

    //Q transfer function to state space
    yTd_p1=-pow(2,0.5)*fQ_cutoff*yT_p1-pow(fQ_cutoff,2)*yT_p2+tautilde_p_d;
    yTd_p2=yT_p1;

    yT_p1=yT_p1+yTd_p1*delta_t.count();
    yT_p2=yT_p2+yTd_p2*delta_t.count();

	double Qtautilde_p=pow(fQ_cutoff,2)*yT_p2;

	double dhat_p=tauhat_p-Qtautilde_p;

	tautilde_p_d=tau_p_d-dhat_p;
	//--------------------------------------------------------------------------
	}	
	double tau_y_d=Py*e_y+Dy*(-TGP_ang_vel.z);	



	//ROS_INFO("xvel:%lf, yvel:%lf, zvel:%lf", TGP_ang_vel.x, TGP_ang_vel.y, TGP_ang_vel.z);
	//ROS_INFO("tr:%lf, tp:%lf, ty:%lf, Thrust_d:%lf", tau_r_d, tau_p_d, tau_y_d, Thrust_d);

	ud_to_PWMs(tautilde_r_d, tautilde_p_d, tau_y_d, Thrust_d);
}

void rpyT_ctrl_auto(double roll_d, double pitch_d, double yaw_d, double Thrust_d){

	e_r=roll_d-TGP_rpy.x;
	e_p=pitch_d-TGP_rpy.y;
	e_y=yaw_d-vicon_att.z;

	if(imu_recv_flag==1){
		e_r_i+=e_r*delta_t.count();
		e_p_i+=e_p*delta_t.count();
		imu_recv_flag=0;
	}
	if(fabs(e_r_i)>integ_limit)	e_r_i=(e_r_i/fabs(e_r_i))*integ_limit;
	if(fabs(e_p_i)>integ_limit)	e_p_i=(e_p_i/fabs(e_p_i))*integ_limit;

	//PID-FP_ctrl OFF
	if(arr[4]<1500){
		tau_r_d=Pa*e_r+Ia*e_r_i+Da*(-TGP_ang_vel.x);//+(double)0.3;
		tau_p_d=Pa*e_p+Ia*e_p_i+Da*(-TGP_ang_vel.y);//+(double)0.1;
		
		tautilde_r_d=tau_r_d;
		tautilde_p_d=tau_p_d;
	}
	
	//PID-FP_ctrl On
	else if(arr[4]>=1500){
		tau_r_d=Pa_fp*e_r+Ia_fp*e_r_i+Da_fp*(-TGP_ang_vel.x)-r_F*m_F*g*TGP_rpy.x;//+(double)0.3;
		tau_p_d=Pa_fp*e_p+Ia_fp*e_p_i+Da_fp*(-TGP_ang_vel.y)-r_F*m_F*g*TGP_rpy.y;//+(double)0.2;;

	//TP roll DOB(applied 2nd order butterworth filter)-------------------------
	//Q/Js transfer function to state space
	xTd_r1=-pow(2,0.5)*fQ_cutoff*xT_r1-pow(fQ_cutoff,2)*xT_r2+TGP_ang_vel.x;
	xTd_r2=xT_r1;

	xT_r1=xT_r1+xTd_r1*delta_t.count();
	xT_r2=xT_r2+xTd_r2*delta_t.count();

	double tauhat_r=J_T1*pow(fQ_cutoff,2)*xT_r1;

    	//Q transfer function to state space
    	yTd_r1=-pow(2,0.5)*fQ_cutoff*yT_r1-pow(fQ_cutoff,2)*yT_r2+tautilde_r_d;
    	yTd_r2=yT_r1;

    	yT_r1=yT_r1+yTd_r1*delta_t.count();
    	yT_r2=yT_r2+yTd_r2*delta_t.count();

	double Qtautilde_r=pow(fQ_cutoff,2)*yT_r2;

	double dhat_r=tauhat_r-Qtautilde_r;

	tautilde_r_d=tau_r_d-dhat_r;
	//--------------------------------------------------------------------------

	//TP pitch DOB(applied 2nd order butterworth filter)-------------------------
	//Q/Js trnsfer function to state space
    xTd_p1=-pow(2,0.5)*fQ_cutoff*xT_p1-pow(fQ_cutoff,2)*xT_p2+TGP_ang_vel.y;
	xTd_p2=xT_p1;

	xT_p1=xT_p1+xTd_p1*delta_t.count();
	xT_p2=xT_p2+xTd_p2*delta_t.count();

	double tauhat_p=J_T2*pow(fQ_cutoff,2)*xT_p1;

    //Q transfer function to state space
    yTd_p1=-pow(2,0.5)*fQ_cutoff*yT_p1-pow(fQ_cutoff,2)*yT_p2+tautilde_p_d;
    yTd_p2=yT_p1;

    yT_p1=yT_p1+yTd_p1*delta_t.count();
    yT_p2=yT_p2+yTd_p2*delta_t.count();

	double Qtautilde_p=pow(fQ_cutoff,2)*yT_p2;

	double dhat_p=tauhat_p-Qtautilde_p;

	tautilde_p_d=tau_p_d-dhat_p;
	//--------------------------------------------------------------------------
	}	
	double tau_y_d=Py*e_y+Dy*(-TGP_ang_vel.z);	

	//ROS_INFO("xvel:%lf, yvel:%lf, zvel:%lf", TGP_ang_vel.x, TGP_ang_vel.y, TGP_ang_vel.z);
	//ROS_INFO("tr:%lf, tp:%lf, ty:%lf, Thrust_d:%lf", tau_r_d, tau_p_d, tau_y_d, Thrust_d);

	ud_to_PWMs(tautilde_r_d, tautilde_p_d, tau_y_d, Thrust_d);
}

void ud_to_PWMs(double tau_r_des, double tau_p_des, double tau_y_des, double Thrust_des){

	F1=+((double)0.5/l_arm)*tau_p_des-((double)0.25/b_over_k_ratio)*tau_y_des+(double)0.25*Thrust_des;
	F2=+((double)0.5/l_arm)*tau_r_des+((double)0.25/b_over_k_ratio)*tau_y_des+(double)0.25*Thrust_des;
	F3=-((double)0.5/l_arm)*tau_p_des-((double)0.25/b_over_k_ratio)*tau_y_des+(double)0.25*Thrust_des;
	F4=-((double)0.5/l_arm)*tau_r_des+((double)0.25/b_over_k_ratio)*tau_y_des+(double)0.25*Thrust_des;
	
	//ROS_INFO("F1:%lf, F2:%lf, F3:%lf, F4:%lf", F1, F2, F3, F4);
	PWMs_cmd.data[0]=Force_to_PWM(F1);
	PWMs_cmd.data[1]=Force_to_PWM(F2);
	PWMs_cmd.data[2]=Force_to_PWM(F3);
	PWMs_cmd.data[3]=Force_to_PWM(F4);	
	//ROS_INFO("1:%d, 2:%d, 3:%d, 4:%d",PWMs_cmd.data[0], PWMs_cmd.data[1], PWMs_cmd.data[2], PWMs_cmd.data[3]);
}

double Force_to_PWM(double F){

	double param1=8.0306753702979e-07*pow(voltage,2)-3.24725348836684e-05*voltage+0.000347874552410687;
        double param2=-0.00197341664580309*pow(voltage,2)+0.0801090916883255*voltage-0.856444510479285;
        double param3=1.18666323640693*pow(voltage,2)-48.2871624529186*voltage+514.980272284705;
        double pwm=(sqrt((F-(param3-(pow(param2,2)/(4*param1))))/param1))-(param2/(2*param1));
	
	//double param1=1111.07275742670;
	//double param2=44543.2632092715;
	//double param3=6112.46876873481;
	//double pwm=param1+sqrt(param2*F+param3);

	if(pwm>1880)	pwm=1880;
	
	return pwm;
}
//--------------------------------------------------------
