#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float32MultiArray.h"
#include "nav_msgs/Odometry.h"
#include <iostream>
#include <fstream>
#include <eigen3/Eigen/Dense>

using namespace std;
using namespace Eigen;

/*-------------------------Functions----------------------*/
void odometry_callback(const nav_msgs::Odometry odometry_msg);
void control_callback(const std_msgs::Float32MultiArray control_msg);
Matrix3d QuatToMat(Vector4d Quat);//--Quaternion to matrix
Vector3d R2XYZ(Matrix3d R);//--Rotation to XYZ angles
void save();			//-------Saving data function
/*-------------------------Functions----------------------*/

/*--------------------------Constants-----------------------*/
float pi = 3.1416;	//-----pi
float e  = 0.001;	    //-----epsilon 
float K = 0;		//-----observer gain: 0.01 best
float Ks = 1.; 	//-----hyperbolic tangent gain
/*--------------------------Constants-----------------------*/

/*------------------Parameters of Hexarotor---------------*/
float m = 1.5, body_width = 0.500, body_height = 0.160;

float Ixx = (1/12.) * m * (body_height * body_height + body_width * body_width);
float Iyy = (1/12.) * m * (body_height * body_height + body_width * body_width);
float Izz = (1/12.) * m * (body_width * body_width + body_width * body_width);
/*------------------Parameters of Hexarotor---------------*/

/*--------------------Observer variables------------------*/
float x11g = 0.0, x12g = 0.0, x13g = 0.0, dx11g= 0.0, dx12g= 0.0, dx13g= 0.0;
float x21g = 0.0, x22g = 0.0, x23g = 0.0, dx21g= 0.0, dx22g= 0.0, dx23g= 0.0;
float x31g = 0.0, x32g = 0.0, x33g = 0.0, dx31g= 0.0, dx32g= 0.0, dx33g= 0.0;
float x41g = 0.0, x42g = 0.0, x43g = 0.0, dx41g= 0.0, dx42g= 0.0, dx43g= 0.0;
float x51g = 0.0, x52g = 0.0, x53g = 0.0, dx51g= 0.0, dx52g= 0.0, dx53g= 0.0;
float x61g = 0.0, x62g = 0.0, x63g = 0.0, dx61g= 0.0, dx62g= 0.0, dx63g= 0.0;
float x11,x21,x31,x41,x51,x61;

Vector3d _orientation;
/*--------------------Observer variables------------------*/

/*---------------------ROS variables---------------------*/
std_msgs::Float32MultiArray u;
std_msgs::Float32MultiArray x3g;
std_msgs::Float32MultiArray x1g;
nav_msgs::Odometry odom;
/*---------------------ROS variables---------------------*/

int main(int argc, char **argv)
{
	/*------ROS Setup-------*/
	ros::init(argc, argv,"robust_observer");
	ros::NodeHandle nh;
	ros::Publisher observer_publisher = nh.advertise<std_msgs::Float32MultiArray>("disturbances",0);
	ros::Publisher observer_publisher_states = nh.advertise<std_msgs::Float32MultiArray>("observed_states",0);
	ros::Subscriber feedback = nh.subscribe("/ndt2/odometry", 1, odometry_callback);
	ros::Subscriber control  = nh.subscribe("control_signals", 1, control_callback);
	cout<<"ROS OK!"<<endl;
	ros::Rate loop_rate(1000);
	/*------ROS Setup-------*/
	
	/*---Initialization----*/
	u.data.resize(6);
	x3g.data.resize(6);
	x1g.data.resize(6);
	/*---Initialization----*/
	
	/*------ROS Loop-------*/
	while (ros::ok())
	{
		cout<<"-------------------------------------"<<endl;
		/*-----Feedback-----------*/
		_orientation = R2XYZ( QuatToMat ( Vector4d( odom.pose.pose.orientation.w,  odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, -odom.pose.pose.orientation.z ) ) );
		
		/*----Robust Observer---*/
		
		/*----------X-----------*/
		x11 =  odom.pose.pose.position.x;
		dx11g = x12g + pow(e,-1)*( x11-x11g ) +	K*tanh( Ks*(x11-x11g) );
		dx12g = x13g + pow(e,-2)*( x11-x11g ) +	K*tanh( Ks*(x11-x11g) ) + (1/m)*u.data[0];
		dx13g =        pow(e,-3)*( x11-x11g ) + K*tanh( Ks*(x11-x11g) );
		
		/*----------Y-----------*/
		x21 = -odom.pose.pose.position.y;
		dx21g = x22g + pow(e,-1)*( x21-x21g ) + K*tanh( Ks*(x21-x21g) );
		dx22g = x23g + pow(e,-2)*( x21-x21g ) + K*tanh( Ks*(x21-x21g) ) + (1/m)*u.data[1];
		dx23g =        pow(e,-3)*( x21-x21g ) + K*tanh( Ks*(x21-x21g) );
		
		/*----------Z-----------*/
		x31 = -odom.pose.pose.position.z;
		dx31g = x32g + pow(e,-1)*( x31-x31g ) + K*tanh( Ks*(x31-x31g ) );
		dx32g = x33g + pow(e,-2)*( x31-x31g ) + K*tanh( Ks*(x31-x31g ) ) + (1/m)*u.data[2];
		dx33g =        pow(e,-3)*( x31-x31g ) + K*tanh( Ks*(x31-x31g ) );
		
		/*--------ROLL----------*/
		x41 = _orientation[0];
		dx41g = x42g + pow(e,-1)*( x41-x41g ) + K*tanh( Ks*(x41-x41g) );
		dx42g = x43g + pow(e,-2)*( x41-x41g ) + K*tanh( Ks*(x41-x41g) ) + (1/Ixx)*u.data[3];
		dx43g =    	   pow(e,-3)*( x41-x41g ) + K*tanh( Ks*(x41-x41g) );
		
		/*--------PITCH----------*/
		x51 = _orientation[1];
		dx51g = x52g + pow(e,-1)*( x51-x51g ) + K*tanh( Ks*(x51-x51g) );
		dx52g = x53g + pow(e,-2)*( x51-x51g ) + K*tanh( Ks*(x51-x51g) ) + (1/Iyy)*u.data[4];
		dx53g =    	   pow(e,-3)*( x51-x51g ) + K*tanh( Ks*(x51-x51g) );
		
		/*--------YAW-----------*/
		x61 = _orientation[2];
		dx61g = x62g + pow(e,-1)*( x61-x61g ) + K*tanh( Ks*(x61-x61g) );
		dx62g = x63g + pow(e,-2)*( x61-x61g ) + K*tanh( Ks*(x61-x61g) ) + (1/Izz)*u.data[5];
		dx63g =        pow(e,-3)*( x61-x61g ) + K*tanh( Ks*(x61-x61g) );
		
		/*---Observer Pubilshers--*/
		x3g.data[0] = x13g; x3g.data[1] = x23g; x3g.data[2] = x33g;
		x3g.data[3] = x43g; x3g.data[4] = x53g; x3g.data[5] = x63g;
		
		x1g.data[0] = x11g; x1g.data[1] = x21g; x1g.data[2] = x31g;
		x1g.data[3] = x41g; x1g.data[4] = x51g; x1g.data[5] = x61g;
		
		observer_publisher.publish(x3g);
		observer_publisher_states.publish(x1g);
		
		/*-----Control info-----*/
		cout<<"x3g = \n"<<x3g<<endl;
		cout<<"\n-------------"<<endl;
		
		ros::spinOnce();
		loop_rate.sleep();
		
		x11g = x11g + 0.1*dx11g;
		x12g = x12g + 0.01*dx12g;
		x13g = x13g + 0.001*dx13g;
		
		x21g = x21g + 0.1*dx21g;
		x22g = x22g + 0.01*dx22g;
		x23g = x23g + 0.001*dx23g;
		
		x31g = x31g + 0.1*dx31g;
		x32g = x32g + 0.01*dx32g;
		x33g = x33g + 0.001*dx33g;
		
		x41g = x41g + 0.1*dx41g;
		x42g = x42g + 0.01*dx42g;
		x43g = x43g + 0.001*dx43g;
		
		x51g = x51g + 0.1*dx51g;
		x52g = x52g + 0.01*dx52g;
		x53g = x53g + 0.001*dx53g;
		
		x61g = x61g + 0.1*dx61g;
		x62g = x62g + 0.01*dx62g;
		x63g = x63g + 0.001*dx63g;
		
		save();
	}
	return 0;
}

void odometry_callback(const nav_msgs::Odometry odometry_msg) {
    odom = odometry_msg;
}

void control_callback(const std_msgs::Float32MultiArray control_msg) {
    u = control_msg;    
}

void save(){
	ofstream myfile;
	myfile.open ("resultsSMESO.txt",std::ios::app);
	myfile <<x11g<<","<<x21g<<","<<x31g<<","<<x41g<<","<<x51g<<","<<x61g<<","
	<<x11<<","<<x21<<","<<x31<<","<<x41<<","<<x51<<","<<x61<<endl;
	myfile.close();
}

Matrix3d QuatToMat(Vector4d Quat){
    Matrix3d Rot;
    float _s = Quat[0];
    float _x = Quat[1];
    float _y = Quat[2];
    float _z = Quat[3];
    Rot << 1-2*(_y*_y+_z*_z),2*(_x*_y-_s*_z),2*(_x*_z+_s*_y),
    2*(_x*_y+_s*_z),1-2*(_x*_x+_z*_z),2*(_y*_z-_s*_x),
    2*(_x*_z-_s*_y),2*(_y*_z+_s*_x),1-2*(_x*_x+_y*_y);
    return Rot;
}

Vector3d R2XYZ(Matrix3d R) {
    double _phi=0.0, _theta=0.0, _psi=0.0;
    Vector3d XYZ = Vector3d::Zero();
    
    _theta = asin(R(0,2));
    
    if(fabsf(cos(_theta))>pow(10.0,-10.0))
    {
        _phi=atan2(-R(1,2)/cos(_theta), R(2,2)/cos(_theta));
        _psi=atan2(-R(0,1)/cos(_theta), R(0,0)/cos(_theta));
    }
    else
    {
        if(fabsf(_theta-pi/2.0)<pow(10.0,-5.0))
        {
            _psi = 0.0;
            _phi = atan2(R(1,0), R(2,0));
            _theta = pi/2.0;
        }
        else
        {
            _psi = 0.0;
            _phi = atan2(-R(1,0), R(2,0));
            _theta = -pi/2.0;
        }
    }
    
    XYZ << _phi,_theta,_psi;
    return XYZ;
}

		
