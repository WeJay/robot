/*///////////////////////////////////////////////////////////////////
	complete converage
///////////////////////////////////////////////////////////////////*/
#include "nav_msgs/OccupancyGrid.h"
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include "std_msgs/String.h"
#include <nav_msgs/Odometry.h>				// odom
#include <irobotcreate2/Bumper.h>		// bumper
#include <irobotcreate2/RoombaIR.h>		// ir_bumper cliff
#include <irobotcreate2/IRCharacter.h>	// ir_character
#include <irobotcreate2/OdomSwitch.h>		// play_song
#include <cstdio>
#include <math.h>
#include <iostream>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <test1/behavior.h>

nav_msgs::Odometry odom;
sensor_msgs::LaserScan laser;
geometry_msgs::PoseStamped pose;
nav_msgs::OccupancyGrid mapmsg;
ros::Publisher pubMessageSwitch;
ros::Publisher pubMessage;
ros::Publisher mode_pub;
uint8_t stepsize = 5; //stepsize is step(pixel) per one meter (pixel/meter) 5 is 0.2 meter per pixel
uint8_t celllength = 1;//cell length (meter) of cellmap
uint8_t cellsize = stepsize*celllength; // pixel number of cell, +1 is for axis path
cv::Mat pathmap(cellsize+2,cellsize+2,CV_16S,cv::Scalar(32767));
int start_pose = 0,start_odom = 0,stop_sonar = 0,stop_bumper = 0, stop_laser = 0,start_laser = 0,start_bug1 = 0, slow_laser = 0,cell_status = 0, system_status = 4, odom_switch, switch_publish, reset_odom = 0, scan_match = 0,erode_param,curve_param, turn_status = 1, start_wall_follow = 1;
/////////////////////////wall_follow cross cell parameters///////////////////////////////////////////////////////////
int robot_mode, sweep_mode = 0, next_corner = 1, last_corner, as_wall_sweep_status = 0; 
float offset_bound = 0;  //if go_straight to go_straight, zoom out boundary
float minedge_x = (-(float)celllength),minedge_y = (-(float)celllength/2),maxedge_x = 0,maxedge_y = ((float)celllength/2),crosscell_x,crosscell_y,edgepoint_x,edgepoint_y,robot_body_x = 0.20,robot_body_y = 0.20;  //boundary parameters
int first_bump = -1;
int LastCrossConer = 0, LastCrossBoundary = 0;
int sumCrossPoint = 0;
struct cross_point_info{
    float x;
    float y;
    float MinBoundary_x;
    float MinBoundary_y;
    float MaxBoundary_x;
    float MaxBoundary_y;
    bool  discover;
};
cross_point_info crosspoint[100] = {0};
///////////////////////////////////////////////////////////////////////////////////////////////////////
bool status_set = 0;

float delta_x = 0, delta_y = 0, frame_x = maxedge_x, frame_y = (maxedge_y+minedge_y)/2;

int status1 = 0,status2; //sweep scan twice before go dock
int wall_stop_wf = 1;
test1::behavior behavior;
struct cell_path_info{
	int x;
	int y;
	int pre_x;
	int pre_y;
	int next_x;
	int next_y;
	float tan;
	float next_tan;
	int status;
};
float FindMedian(float* a, int n);
float FindMean(float* a, int n);
void occupancyGridToGrayCvMat(const nav_msgs::OccupancyGrid *map, cv::Mat *im);
void bumperCallback(const irobotcreate2::Bumper msg);
void sendCmdVel(ros::Publisher pub, float linear, float angle);
void odomCallBack(const nav_msgs::Odometry msg);
void poseCallback(const geometry_msgs::PoseStamped& msg);
void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
void imageCallback(const nav_msgs::OccupancyGrid& msg);
void modeCallback(const test1::behavior msg);
int cell_coordinate(cv::Mat opencv_map, uint8_t metersize, const uint16_t laser_shift_x, const uint16_t laser_shift_y);
int sweep_scan(ros::Publisher pub, int frequency , const float path_coordinate_x, const float path_coordinate_y,float kp_ang,float ki_ang,float kd_ang);
void cell_sweepscan(const ros::Publisher pub,const int rate, cell_path_info *sweeppath,float kp_ang,float ki_ang,float kd_ang);
void test_sweepscan(const ros::Publisher pub,const int rate,float kp_ang,float ki_ang,float kd_ang);
void where_to_go(int x,int y,cv::Mat path, cell_path_info *pathpoint);
void bumper_behavior();
void laser_avoidance();
void wall_follow(float minedge_x, float minedge_y, float maxedge_x, float maxedge_y, ros::Publisher pub, int frequency, float kp_ang, float ki_ang, float kd_ang);
void findedge(float minedge_xx, float minedge_yy, float maxedge_xx, float maxedge_yy);
float dis_p2p(float x1, float y1, float x2, float y2);
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
void save_cellmap();
void increment_sweepscan(const ros::Publisher pub,const int rate,float kp_ang,float ki_ang,float kd_ang,cell_path_info *sweeppath);
void maptransform(cv::Mat* opencv_map);
void setpointTF(float& input_x,float& input_y);
int main(int argc, char **argv)
{

  cell_path_info cellpath;
	cellpath.pre_x = 0;
	cellpath.pre_y = 0;
	cellpath.x = 0;//(int16_t)(pose.pose.position.x*stepsize+1);
	cellpath.y = 0;//(int16_t)(pose.pose.position.y*stepsize+1);
	cellpath.next_x = 0;
	cellpath.next_y = 0;
	cellpath.tan = 1;
	cellpath.next_tan = 0;
	cellpath.status = 0;
	double sweep_scan_secs = 0;
	int ros_loop_rate = 30; // Message publishing rate: Hertz
  std::string odom_id;
  double P_vel, I_vel, D_vel;
	pose.pose.position.x = 0;
	ros::init(argc, argv, "complete_coverage");
	ros::NodeHandle n_slam;
  ros::NodeHandle n_param("~");
  n_param.param<double>("p_vel", P_vel, 1.2f);
  n_param.param<double>("i_vel", I_vel, 0.0f);
  n_param.param<double>("d_vel", D_vel, 0.1f);
  n_param.param<int>("odom_switch", odom_switch, 0);
  n_param.param<int>("switch_publish", switch_publish, 1);
  n_param.param<std::string>("odom_id", odom_id, "/odom");
  n_param.param<int>("erode_param", erode_param, 16);
  n_param.param<int>("curve_param", curve_param, 0);  
	ros::Subscriber sub_pose = n_slam.subscribe("pose", 100 , poseCallback);
	ros::Subscriber sub_laser = n_slam.subscribe("scan", 100 , scanCallback);
	ros::Subscriber sub_image = n_slam.subscribe("map", 100, imageCallback);
  ros::Subscriber sub = n_slam.subscribe(odom_id, 1000, odomCallBack);
  ros::Subscriber sub_mode = n_slam.subscribe<test1::behavior>("/behavior", 50 ,modeCallback);
  
  ros::Subscriber bumper_pub = n_slam.subscribe<irobotcreate2::Bumper>("bumper", 50 ,bumperCallback);

  mode_pub = n_slam.advertise<test1::behavior>("/behavior", 100);
  pubMessage = n_slam.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
  pubMessageSwitch = n_slam.advertise<irobotcreate2::OdomSwitch>("/switch_odom", 1000);
  behavior.robotmode = 0;
  mode_pub.publish(behavior);
  ros::Rate loop_rate(50);
  while(n_slam.ok())
  {
     //start_wall_follow = 0;
     if(robot_mode == 0)
     {
       if(start_wall_follow != 0 & start_pose == 1 & start_laser == 1)
       {
         wall_follow(minedge_x, minedge_y, maxedge_x, maxedge_y, pubMessage, ros_loop_rate, P_vel, I_vel, D_vel);
       }
       static bool start_sweep_scan = 0;
       if(start_wall_follow == 0 & !start_sweep_scan & scan_match){
         start_sweep_scan = 1;
         reset_odom = 5;
         save_cellmap();
         for(int i = 0; i < sumCrossPoint; i++)
         {
           printf("cross[%d] = x: %f, y: %f, minx: %f, miny: %f, Maxx: %f, Maxy: %f, discover: %d\n",i,crosspoint[i].x,crosspoint[i].y,crosspoint[i].MinBoundary_x,crosspoint[i].MinBoundary_y,crosspoint[i].MaxBoundary_x,crosspoint[i].MaxBoundary_y,crosspoint[i].discover);
         }
       }
       if(start_sweep_scan & reset_odom!=5)
       {
  			bumper_behavior();
        			if(start_laser)
        				laser_avoidance();
  			  /*if (ros::Time::now().toSec()-sweep_scan_secs > 1.0f/(float)ros_loop_rate & start_pose & cellpath.status != 2 & cell_status){
    				//int sweep_status = sweep_scan(pubMessage,ros_loop_rate,2.0,0,P_vel,I_vel,D_vel);
    				//printf("sweep_scan  secs = %f\n",ros::Time::now().toSec()-sweep_scan_secs);
  			  	//test_sweepscan(pubMessage,ros_loop_rate,P_vel,I_vel,D_vel); 
    				cell_sweepscan(pubMessage, ros_loop_rate, &cellpath,P_vel,I_vel,D_vel);
				    //increment_sweepscan(pubMessage,ros_loop_rate,P_vel,I_vel,D_vel,&cellpath);
  				  sweep_scan_secs = ros::Time::now().toSec();
  			  }*/
           cellpath.status = 2;
           cell_status = 0;
        }
        if (ros::Time::now().toSec()-sweep_scan_secs > 1.0f){
					//printf("cellpath.status = %d\n",cellpath.status);
				  sweep_scan_secs = ros::Time::now().toSec();
			  }
				if(cellpath.status == 2 & cell_status == 0){
					int crosscell_status = sweep_scan(pubMessage,ros_loop_rate,crosscell_x,crosscell_y,P_vel,I_vel,D_vel);
           if(crosscell_status == 1)
           {
						//frame_x = maxedge_x;
						//frame_y = (maxedge_y+minedge_y)/2;
						start_wall_follow = 1;
						start_sweep_scan = 0;
						cellpath.status = 0;
						//status_set = 0;
						//cellpath.x = 0;
						//cellpath.y = 0;
						//printf("save_cellmap ...........................................................\n");
						/*for (int i = 0 ; i < pathmap.rows ; i++){
							for (int j = 0 ; j < pathmap.cols ; j++){
								//printf("%d\t",pathmap.at<int16_t>(cv::Point2i(i,j)));
								pathmap.at<int16_t>(cv::Point2i(i,j)) = 32767;
							}
							//printf("\n");
						}*/
             printf("cross cell success!!\n");
           }
				}
        //go dock
        if(cellpath.status == 2 & behavior.robotmode!=2 & 0)
        {
          if(status1 == 0)
            status1 = sweep_scan(pubMessage,ros_loop_rate,0.20,0.5,P_vel,I_vel,D_vel);
          if(status1 == 1)
          {
            status2 = sweep_scan(pubMessage,ros_loop_rate,0.15,0.5,P_vel,I_vel,D_vel);
            if(status2 == 1){
              printf("dock ready!!");
              behavior.robotmode = 2; 
              mode_pub.publish(behavior);
            }
          }
        }
      }
	ros::spinOnce();
	loop_rate.sleep();
  }
}

void bumper_behavior()
{
	if(stop_bumper >= 1){
		sendCmdVel(pubMessage, -0.1, 0);
		usleep(100000);
		sendCmdVel(pubMessage, -0.15, 0);
		usleep(100000);
		sendCmdVel(pubMessage, -0.2, 0);
		usleep(100000);
		sendCmdVel(pubMessage, -0.25, 0);
		usleep(100000);
		sendCmdVel(pubMessage, -0.2, 0);
		usleep(100000);
		sendCmdVel(pubMessage, -0.15, 0);
		usleep(100000);
		sendCmdVel(pubMessage, -0.05, 0);
		usleep(100000);
		sendCmdVel(pubMessage, 0, 0);
		if (odom_switch == 0)
			printf("stop_bumper = %d,bumper warning in odom(x = %f , y = %f), go to next setpoint\n",stop_bumper,odom.pose.pose.position.x,odom.pose.pose.position.y);
		else
			printf("stop_bumper = %d,bumper warning in pose(x = %f , y = %f), go to next setpoint\n",stop_bumper,pose.pose.position.x,pose.pose.position.y);
	}
}

void modeCallback(const test1::behavior msg)
{
  robot_mode = msg.robotmode;
  //printf("robot_mode = %d\n",robot_mode);
}

void odomCallBack(const nav_msgs::Odometry msg)
{
	odom = msg;
  	start_odom = 1;
}

void poseCallback(const geometry_msgs::PoseStamped& msg)
{
	pose = msg;
  if ((reset_odom!=0)){
    tf::Pose pose_tf;
    tf::poseMsgToTF(pose.pose, pose_tf);
 	  float pose_yaw = tf::getYaw(pose_tf.getRotation());
    if(reset_odom !=5){
      if ((pose_yaw < M_PI/4 & pose_yaw > (-1)*M_PI/4) | pose_yaw < (-3)*M_PI/4 | pose_yaw > 3*M_PI/4)
        reset_odom = 3;
      else
        reset_odom = 2;
    }
    if(fabs(pose.pose.position.x - odom.pose.pose.position.x) < 0.08 & fabs(pose.pose.position.y - odom.pose.pose.position.y) < 0.08 | reset_odom == 1 | reset_odom == 3 | reset_odom == 5){
      irobotcreate2::OdomSwitch Switch_odom;
      Switch_odom.x = pose.pose.position.x;
      Switch_odom.y = pose.pose.position.y;
      Switch_odom.yaw = pose_yaw;
      Switch_odom.status = reset_odom;
      printf("Switch_odom.status=%d,yaw=%f,pose.x=%f,pose.y=%f,odom.x=%f,odom.y=%f,x_diff=%f,y_diff=%f\n",\
      Switch_odom.status,pose_yaw,pose.pose.position.x,pose.pose.position.y,odom.pose.pose.position.x,\
      odom.pose.pose.position.y,fabs(pose.pose.position.x - odom.pose.pose.position.x),\
      fabs(pose.pose.position.y - odom.pose.pose.position.y));
      if (switch_publish | reset_odom==5)
      pubMessageSwitch.publish(Switch_odom);
    }
    reset_odom = 0;
  }
	start_pose = 1;
}

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
        laser = *scan;
	/*static int count_stop_laser = 0;  /////////////////////// Lidar obstacle avoidance ////////////////////////////
	for (int i = 0 ; i<35 ; i+=1)
	{
	printf("stop_laser = %d,count_stop_laser = %d,laser.ranges[%d] = %3.3f\n",stop_laser,count_stop_laser,i,laser.ranges[i]);
		}
	for (int i = 325 ; i<360 ; i+=1)
	{
	printf("stop_laser = %d,count_stop_laser = %d,laser.ranges[%d] = %3.3f\n",stop_laser,count_stop_laser,i,laser.ranges[i]);
	}*/
        start_laser = 1;
} 

void imageCallback(const nav_msgs::OccupancyGrid& msg)
{
    mapmsg = msg;
    //save_cellmap();  
	//printf("scan matching ... , position.x = %f, position.y = %f\n",pose.pose.position.x,pose.pose.position.y);
    if(switch_publish)
      scan_match = 1;
}

void bumperCallback(const irobotcreate2::Bumper msg)
{

  if (msg.left.state == 1 & msg.right.state == 1){
	stop_bumper = 1;
  }
  else if(msg.left.state == 1)
	stop_bumper = 2;
  else if(msg.right.state == 1)
	stop_bumper = 3;
  else
	stop_bumper = 0;
}

void sendCmdVel(ros::Publisher pub, float linear, float angle)
{
   	static geometry_msgs::Twist Twist;
	if(slow_laser == 5)
  		Twist.linear.x = linear*0.8;
	else if(slow_laser == 4)
  		Twist.linear.x = linear*0.6;
	else if(slow_laser == 3)
  		Twist.linear.x = linear*0.4;
	else if(slow_laser == 2)
  		Twist.linear.x = linear*0.2;
	else if(slow_laser == 1)
		Twist.linear.x = linear*0.1;
	else
		Twist.linear.x = linear;
  		Twist.angular.z = angle;
	if(Twist.linear.x > 0 & Twist.linear.x < 0.05)
		Twist.linear.x = 0.05;
  		
	pub.publish(Twist);
	//printf("sendcomdel Twist.linear.x=%3.3f,Twist.angular.z = %3.3f\n ",Twist.linear.x,Twist.angular.z);
}
void laser_avoidance(){
	float frontdatar[5]={0},frontdatal[5]={0},sidedatar1[5]={0},sidedatar2[5]={0},sidedatal1[5]={0},sidedatal2[5]={0};
	float frontdata1[5]={0},frontdata2[5]={0},frontdata3[5]={0},frontdata4[5]={0},frontdata5[5]={0};
	float frontdata6[5]={0},frontdata7[5]={0},frontdata8[5]={0},frontdata9[5]={0},frontdata10[5]={0};
	for (int j = 0 ; j<5 ; j+=1){
		//Right side
		frontdatar[j] = laser.ranges[j+348];
		sidedatar1[j] = laser.ranges[j+323];
		sidedatar2[j] = laser.ranges[j+303];
		//Left side
		frontdatal[j] = laser.ranges[j+3];
		//printf("frontdatal[%d] = %f\n",j,frontdatal[j]);
		sidedatal1[j] = laser.ranges[j+29];
		sidedatal2[j] = laser.ranges[j+53];  
		// angle 
		frontdata1[j] = laser.ranges[j+18];
		frontdata2[j] = laser.ranges[j+20];
		frontdata3[j] = laser.ranges[j+22];
		frontdata4[j] = laser.ranges[j+25];
		frontdata5[j] = laser.ranges[j+28];
		frontdata6[j] = laser.ranges[j+336];
		frontdata7[j] = laser.ranges[j+334];
		frontdata8[j] = laser.ranges[j+333];
		frontdata9[j] = laser.ranges[j+331];
		frontdata10[j] = laser.ranges[j+329];
	}
  	//right side
	float frontdistr = FindMean(frontdatar,5);
  	float sidedistr1 = FindMean(sidedatar1,5);
  	float sidedistr2 = FindMean(sidedatar2,5);
	//left side
	float frontdistl = FindMean(frontdatal,5);
  	float sidedistl1 = FindMean(sidedatal1,5);
  	float sidedistl2 = FindMean(sidedatal2,5);
 	float laser_[6] = {sidedistl2,sidedistl1,frontdistl,frontdistr,sidedistr1,sidedistr2}; 
	float frontdist1 = FindMean(frontdata1,5);
	float frontdist2 = FindMean(frontdata2,5);
	float frontdist3 = FindMean(frontdata3,5);
	float frontdist4 = FindMean(frontdata4,5);
	float frontdist5 = FindMean(frontdata5,5);
	float frontdist6 = FindMean(frontdata6,5);
	float frontdist7 = FindMean(frontdata7,5);
	float frontdist8 = FindMean(frontdata8,5);
	float frontdist9 = FindMean(frontdata9,5);
	float frontdist10 = FindMean(frontdata10,5);
	static int wall_stop = 1;
	bool stop_laser_status = (frontdistr>=0.1 & frontdistr <= 0.32) | (frontdistl>=0.1 & frontdistl <= 0.32) | (frontdist5>=0.1 & frontdist5 <= 0.32) | (frontdist10>=0.1 & frontdist10 <= 0.32) | (frontdist3>=0.1 & frontdist3 <= 0.32) | (frontdist8>=0.1 & frontdist8 <= 0.32) | (frontdist1>=0.1 & frontdist1 <= 0.32) | (frontdist6>=0.1 & frontdist6 <= 0.32);
	  	if(stop_laser_status){
			slow_laser = 1;
		}
		else if((frontdistr>=0.1 & frontdistr <= 0.37) | (frontdistl >= 0.1 & frontdistl <= 0.37) | (frontdist4>=0.1 & frontdist4 <= 0.4) | (frontdist9>=0.1 & frontdist9 <= 0.4))
			slow_laser = 2;
		else if((frontdistr >= 0.1 & frontdistr <= 0.42) | (frontdistl >= 0.1 & frontdistl <= 0.42) | (frontdist3>=0.1 & frontdist3 <= 0.47) | (frontdist8>=0.1 & frontdist8 <= 0.47))
			slow_laser = 3;
		else if((frontdistr >= 0.1 & frontdistr <= 0.47) | (frontdistl >= 0.1 & frontdistl <= 0.47) | (frontdist2>=0.1 & frontdist2 <= 0.51) | (frontdist7>=0.1 & frontdist7 <= 0.51))
			slow_laser = 4;
		else if((frontdistr >= 0.1 & frontdistr <= 0.52) | (frontdistl >= 0.1 & frontdistl <= 0.52) | (frontdist1>=0.1 & frontdist1 <= 0.57) | (frontdist6>=0.1 & frontdist6 <= 0.57))
			slow_laser = 5;
		else
			slow_laser = 0;
  if(start_bug1){
      float smallest = 10;
      int min_index;
      static int side = 0;  //right = 1,left = 2
      if(turn_status == 1)
      {
        printf("laser_ = %f, %f, %f, %f, %f, %f\n",laser_[0],laser_[1],laser_[2],laser_[3],laser_[4],laser_[5]);
        for(int i = 0; i < 6; i++)
        {
          if(laser_[i] < smallest & laser_[i] >= 0.2)
        {
          smallest = laser_[i];
          min_index = i;
        }  
      }
      if(min_index >= 3){
        for (int a = 0 ; a<=5*(5 - min_index); a+=1){
		          usleep(150000);
				      sendCmdVel(pubMessage, 0, 0.8);
		          }
        side = 1;
        }
      if(min_index <= 2){
        for (int a = 0 ; a<=6*(min_index); a+=1){
				      usleep(150000);
				      sendCmdVel(pubMessage, 0, -0.8);
			        }
        side = 2;
        } 
        turn_status = 2;
       }    
      float dist;
      if(side == 1) 
        dist = sidedistr2;
      if(side == 2) 
        dist = sidedistl2;
		  if(stop_laser_status)
		  	turn_status = 1;   
		  else{
			float z_speed = (0.35 - dist)*4;//0.291
			float x_speed = 0.1;
			if(dist == 0)
				z_speed = -0.48;
			if(z_speed >= 0.48)
				z_speed = 0.48;
			if(z_speed <= -0.48)
				z_speed = -0.48;
			if(dist<0.2)
				x_speed = 0.02;
      if(side == 2) 
        z_speed = -z_speed;
			sendCmdVel(pubMessage, 0.1, z_speed);
			wall_stop = 1;
			//printf("x_speed = %f,z_speed = %f\n",x_speed,z_speed);
		  }  
		//printf("side = %d, dist = %f\n",side,dist);
	}
	//printf("fr = %f, fl = %f, fd1 = %f, fd3 = %f,fd5 = %f, fd6 = %f,fd8 = %f, fd10 = %f, slow_laser = %d\n",frontdistr,frontdistl,frontdist1,frontdist3,frontdist5,frontdist6,frontdist8,frontdist10,slow_laser);
}
void setpointTF(float& input_x,float& input_y){
	float angle_TF = M_PI;
  float offset = (float)1/stepsize/2;
	float x_TF = 0.0f+frame_x-offset , y_TF = (float)celllength/2+frame_y-offset;
	input_x = (float)(cos(angle_TF)*input_x-sin(angle_TF)*input_y+x_TF);
	input_y = (float)(sin(angle_TF)*input_x+cos(angle_TF)*input_y+y_TF);
}
void cell_sweepscan(const ros::Publisher pub,const int rate, cell_path_info *sweeppath,float kp_ang,float ki_ang,float kd_ang)
{
	float x_sp = (float)(sweeppath->x-1)/stepsize, y_sp = (float)(sweeppath->y-1)/stepsize-delta_y;
	int status = 1;
	setpointTF(x_sp,y_sp);
  x_sp = x_sp - delta_x;
  y_sp = y_sp - delta_y;
	if (sweeppath->tan != sweeppath->next_tan & status_set){
		status = sweep_scan(pub,rate,x_sp,y_sp,kp_ang,ki_ang,kd_ang);
		//printf("Sweep scan in  x = %d, y = %d, x_sp = %f, y_sp = %f\n",sweeppath->x,sweeppath->y,x_sp,y_sp);
	}	
	if((status==1 & delta_x!=0) | (status==1 & delta_y!=0)){
		delta_x = 0;
		delta_y = 0;
	}
	if(status>=1 & sweeppath->status!=2){
		status_set = 1;      
  		where_to_go(sweeppath->x,sweeppath->y,pathmap,sweeppath);
      if (sweeppath->tan != sweeppath->next_tan & status_set){   
    		printf("pathmap ...........................................................\n");
    		for (int i = 0 ; i < pathmap.rows ; i++){
    			for (int j = 0 ; j < pathmap.cols ; j++){
    				printf("%d\t",pathmap.at<int16_t>(cv::Point2i(i,j)));
    			}
    			printf("\n");
    		}
      }      
	}
	//printf("cell_sweepscan x_pre = %d, y_pre = %d, x = %d, y = %d, x_next = %d, y_next = %d, tan = %f, next_tan = %f\n",sweeppath->pre_x,sweeppath->pre_y,sweeppath->x,sweeppath->y,sweeppath->next_x,sweeppath->next_y,sweeppath->tan,sweeppath->next_tan);
}

void test_sweepscan(const ros::Publisher pub,const int rate,float kp_ang,float ki_ang,float kd_ang)
{
	if (start_pose == 1)
	{
		int path_size=22;//14;
    float path_coordinate_x[path_size] , path_coordinate_y[path_size];
		  float ba = 4.0;
  		path_coordinate_x[0] = 0.0*ba;
  		path_coordinate_x[1] = 0.5*ba;
  		path_coordinate_x[2] = 0.5*ba;
   		path_coordinate_x[3] = 0*ba;
   		path_coordinate_x[4] = 0*ba;
   		path_coordinate_x[5] = 0.5*ba;
   		path_coordinate_x[6] = 0.5*ba;
   		path_coordinate_x[7] = 0*ba;
  		path_coordinate_x[8] = 0*ba;
  		path_coordinate_x[9] = 0.5*ba;
   		path_coordinate_x[10] = 0.5*ba;
   		path_coordinate_x[11] = 0*ba;
   		path_coordinate_x[12] = 0*ba;
   		path_coordinate_x[13] = 0.5*ba;
  		path_coordinate_x[14] = 0.5*ba;
  		path_coordinate_x[15] = 0*ba;
  		path_coordinate_x[16] = 0*ba;
   		path_coordinate_x[17] = 0.5*ba;
   		path_coordinate_x[18] = 0.5*ba;
   		path_coordinate_x[19] = 0*ba;
   		path_coordinate_x[20] = 0*ba;
      		path_coordinate_x[21] = 0.5*ba;
  
  		float bb = 2.0;
  		path_coordinate_y[0] = 0.0*bb;
  		path_coordinate_y[1] = 0.0*bb;
  		path_coordinate_y[2] = 0.1*bb;
  		path_coordinate_y[3] = 0.1*bb;
  		path_coordinate_y[4] = 0.2*bb;
  		path_coordinate_y[5] = 0.2*bb;
  		path_coordinate_y[6] = 0.3*bb;
  		path_coordinate_y[7] = 0.3*bb;
  		path_coordinate_y[8] = 0.4*bb;
  		path_coordinate_y[9] = 0.4*bb;
  		path_coordinate_y[10] = 0.5*bb;
  		path_coordinate_y[11] = 0.5*bb;
  		path_coordinate_y[12] = 0.6*bb;
  		path_coordinate_y[13] = 0.6*bb;
  		path_coordinate_y[14] = 0.7*bb;
  		path_coordinate_y[15] = 0.7*bb;
  		path_coordinate_y[16] = 0.8*bb;
  		path_coordinate_y[17] = 0.8*bb;
  		path_coordinate_y[18] = 0.9*bb;
  		path_coordinate_y[19] = 0.9*bb;
  		path_coordinate_y[20] = 1.0*bb;
      		path_coordinate_y[21] = 1.0*bb;
	// Array of path_coordinate: Meters, Meters
    static int path_counter = 0;//test for sweep scan
		int status = sweep_scan(pub,rate,path_coordinate_x[path_counter],path_coordinate_y[path_counter],kp_ang,ki_ang,kd_ang);
		if (status == 1 & path_counter < path_size-1)
			path_counter++;
		else if (status == 1 & path_counter>=path_size-1)
			path_counter=0;
		//printf("status = %d,i = %d\n",status,i);
	}
}

void increment_sweepscan(const ros::Publisher pub,const int rate,float kp_ang,float ki_ang,float kd_ang,cell_path_info *sweeppath)
{
  if (start_pose == 1){
  	static float cellstep = 0.2f, cellmax = 2.0f, cellmin = 0.0;
	static float x_sp = odom.pose.pose.position.x, y_sp = odom.pose.pose.position.y;
	sweeppath->x = (int16_t)(pose.pose.position.x*stepsize+1+0.5); // 0.5 is for round
	sweeppath->y = (int16_t)(pose.pose.position.y*stepsize+1+0.5);
    ////////////////////// sweep scan in the square //////////////////////
    //cellmax is the side length of square, cellstep is the step in one sweep scan, x_sp, y_sp are the set points in x, y
    static uint8_t sweep_count = 0, xsp_count = 0;
    int status = sweep_scan(pub,rate,x_sp,y_sp,kp_ang,ki_ang,kd_ang);
	//printf("sweeppath.status = %d,x_sp = %f, y_sp = %f, sweeppath.x = %d,sweeppath.y = %d, sweeppath.pre_x = %d, pathpoint.pre_y = %d\n",sweeppath->status,x_sp,y_sp,sweeppath->x,sweeppath->y,sweeppath->pre_x,sweeppath->pre_y);
	/*if ((sweeppath->x != sweeppath->pre_x | sweeppath->y != sweeppath->pre_y) & cell_status == 0 & status >= 1){
		sweeppath->pre_x = sweeppath->x;
		sweeppath->pre_y = sweeppath->y;
		where_to_go(sweeppath->x,sweeppath->y,pathmap,sweeppath);
		if (sweeppath->status == 1){
			printf("movetogoal, x_sp = %f, y_sp = %f\n",(float)(sweeppath->x-1-0.5)/stepsize,(float)(sweeppath->y-1-0.5)/stepsize);
			printf("min min min pose.pose.position.x = %f, pose.pose.position.y = %f\n",pose.pose.position.x,pose.pose.position.y);
		      	x_sp = (float)(sweeppath->x-1-0.5)/stepsize;
		      	y_sp = (float)(sweeppath->y-1-0.5)/stepsize;
			usleep(5000000);
			sweep_count = 0;
			xsp_count = 0;
			printf("finish finish finish x_sp = %f, y_sp = %f, pose.pose.position.x = %f, pose.pose.position.y = %f\n",x_sp,y_sp,pose.pose.position.x,pose.pose.position.y);
			sweeppath->status = 0;
		}
	}
*/
//printf("status = %d, x_sp = %f, y_sp = %f\n",status,x_sp,y_sp);
    ////////////////////// count next set points //////////////////////
    if (status >= 1){
      if (sweep_count%2 == 0){
        if (xsp_count%2 == 0){
          if (status == 2){
            if (odom_switch == 0){
              usleep(500000);
              x_sp = odom.pose.pose.position.x;
              y_sp = odom.pose.pose.position.y;
              printf("x+, odom.pose.pose.position.x = %f, odom.pose.pose.position.y = %f\n",odom.pose.pose.position.x,odom.pose.pose.position.y);
            }
            else {
              usleep(500000);
              x_sp = pose.pose.position.x;
              y_sp = pose.pose.position.y;
              printf("x+, pose.pose.position.x = %f, pose.pose.position.y = %f\n",pose.pose.position.x,pose.pose.position.y);
            }
          }
          x_sp = x_sp + cellmax;
        }
        else{
          if (status == 2){
            if (odom_switch == 0){
              usleep(500000);
              x_sp = odom.pose.pose.position.x;
              y_sp = odom.pose.pose.position.y;
              printf("x-, odom.pose.pose.position.x = %f, odom.pose.pose.position.y = %f\n",odom.pose.pose.position.x,odom.pose.pose.position.y);
            }
            else {
              usleep(500000);
              x_sp = pose.pose.position.x;
              y_sp = pose.pose.position.y;
              printf("x-, pose.pose.position.x = %f, pose.pose.position.y = %f\n",pose.pose.position.x,pose.pose.position.y);
            }
          }
          x_sp = x_sp - cellmax;
        }
          xsp_count++;
      }
      else {
        if (status == 2){
          if (odom_switch == 0){
            usleep(500000);
            x_sp = odom.pose.pose.position.x;
            y_sp = odom.pose.pose.position.y;
            printf("y+, odom.pose.pose.position.x = %f, odom.pose.pose.position.y = %f\n",odom.pose.pose.position.x,odom.pose.pose.position.y);
          }
          else {
            usleep(500000);
            x_sp = pose.pose.position.x;
            y_sp = pose.pose.position.y;
            printf("y+, pose.pose.position.x = %f, pose.pose.position.y = %f\n",pose.pose.position.x,pose.pose.position.y);
          }
        }
        y_sp = y_sp + cellstep;
      }
      
      if (x_sp > cellmax)
        x_sp = cellmax;
      else if (x_sp < cellmin)
        x_sp = cellmin;
      if (y_sp > cellmax)
        y_sp = cellmax;
      else if (y_sp < cellmin)
        y_sp = cellmin;
      
      sweep_count++;
      
      if (x_sp == cellmax & y_sp == cellmax){
        x_sp = 0;
        y_sp = 0;
        sweep_count = 0;
        xsp_count = 0;
      }
    }
  }
}

void where_to_go(int x,int y,cv::Mat path, cell_path_info *pathpoint)
{
    	pathmap.at<int16_t>(cv::Point2i(x,y)) = 32766;
	int start = 32766;
	int next_start = 32766;
	int min = 32766;
		pathpoint->status = 0;
		pathpoint->pre_x = x;
		pathpoint->pre_y = y;
	//printf("where start in x = %d, y = %d\n",x,y);
	if (x > 0 & y > 0){
		if (path.at<int16_t>(cv::Point2d(x-1,y))<start){
			pathpoint->x = x-1;
			pathpoint->y = y;
			start = path.at<int16_t>(cv::Point2d(x-1,y));
		}
		if (path.at<int16_t>(cv::Point2d(x,y-1))<start){
			pathpoint->x = x;
			pathpoint->y = y-1;
			start = path.at<int16_t>(cv::Point2d(x,y-1));
		}
	}
	if (path.at<int16_t>(cv::Point2d(x,y+1))<start){
		pathpoint->x = x;
		pathpoint->y = y+1;
		start = path.at<int16_t>(cv::Point2d(x,y+1));
	}
	if (path.at<int16_t>(cv::Point2d(x+1,y))<start){
		pathpoint->x = x+1;
		pathpoint->y = y;
		start = path.at<int16_t>(cv::Point2d(x+1,y));
	}
	if (start >= 32766){
		pathpoint->status = 1;
		for (int i = 0 ; i < pathmap.rows ; i++){
			for (int j = 0 ; j < pathmap.cols ; j++){
				if (pathmap.at<int16_t>(cv::Point2i(i,j))<min){
					min = pathmap.at<int16_t>(cv::Point2i(i,j));
					pathpoint->x = i;
					pathpoint->y = j;
				}
			}
		}
	  	if (min == 32766 & pathmap.at<int16_t>(cv::Point2i(pathpoint->x,pathpoint->y)) == 32766){
	      		pathpoint->status = 2;
						cell_status = 0;
	      		printf("pathpoint->status = %d\n finish.....\n\n one cell sweep scan.....\n\n",pathpoint->status);
	  	}
	}
	/*printf("min = %d, nowvalue = %d\n",min,pathmap.at<int16_t>(cv::Point2i(pathpoint->x,pathpoint->y)));
	printf("pathmap ...........................................................\n");
	for (int i = 0 ; i < pathmap.rows ; i++){
		for (int j = 0 ; j < pathmap.cols ; j++){
			printf("%d\t",pathmap.at<int16_t>(cv::Point2i(i,j)));
		}
		printf("\n");
	}*/

///////////////////////////////// next point //////////////////////////////////////
	pathpoint->tan = atan2((pathpoint->y-pathpoint->pre_y) , (pathpoint->x-pathpoint->pre_x));

	if (path.at<int16_t>(cv::Point2d(pathpoint->x,pathpoint->y-1))<next_start)
	{
		pathpoint->next_x = pathpoint->x;
		pathpoint->next_y = pathpoint->y-1;
		next_start = path.at<int16_t>(cv::Point2d(pathpoint->x,pathpoint->y-1));
	}
	if (path.at<int16_t>(cv::Point2d(pathpoint->x,pathpoint->y+1))<next_start)
	{
		pathpoint->next_x = pathpoint->x;
		pathpoint->next_y = pathpoint->y+1;
		next_start = path.at<int16_t>(cv::Point2d(pathpoint->x,pathpoint->y+1));
	}
	if (path.at<int16_t>(cv::Point2d(pathpoint->x+1,pathpoint->y))<next_start)
	{
		pathpoint->next_x = pathpoint->x+1;
		pathpoint->next_y = pathpoint->y;
		next_start = path.at<int16_t>(cv::Point2d(pathpoint->x+1,pathpoint->y));
	}
	if (path.at<int16_t>(cv::Point2d(pathpoint->x-1,pathpoint->y))<next_start)
	{
		pathpoint->next_x = pathpoint->x-1;
		pathpoint->next_y = pathpoint->y;
		next_start = path.at<int16_t>(cv::Point2d(pathpoint->x-1,pathpoint->y));
	}
	if (next_start >= 32766)
	{
		pathpoint->next_x = 0;
		pathpoint->next_y = 0;
	}
	pathpoint->next_tan = atan2((pathpoint->next_y-pathpoint->y) , (pathpoint->next_x-pathpoint->x));
	printf("where_to_go,x_pre =%d,y_pre = %d,x = %d,y = %d,x_next = %d,y_next = %d, tan = %f, next_tan = %f\n",pathpoint->pre_x,pathpoint->pre_y,pathpoint->x,pathpoint->y,pathpoint->next_x,pathpoint->next_y,pathpoint->tan,pathpoint->next_tan);
}
void maptransform(cv::Mat* opencv_map){
	//rotation matrix
	cv::Size dst_sz(opencv_map->cols, opencv_map->rows);
	double angle = 180;
	cv::Point2f center((uint16_t)((0-mapmsg.info.origin.position.x)/mapmsg.info.resolution)\
,(uint16_t)((0-mapmsg.info.origin.position.y)/mapmsg.info.resolution));
	cv::Mat rot_mat = cv::getRotationMatrix2D(center, angle, 1.0);
	cv::warpAffine(*opencv_map, *opencv_map, rot_mat, dst_sz);
	//translation matrix
	cv::Mat t_mat =cv::Mat::zeros(2, 3, CV_32FC1);
	t_mat.at<float>(0, 0) = 1;
	t_mat.at<float>(0, 2) = (float)((float)celllength/2+frame_y)*(1/mapmsg.info.resolution);
	t_mat.at<float>(1, 1) = 1;
	t_mat.at<float>(1, 2) = (float)(0.0f+frame_x)*(1/mapmsg.info.resolution);
	printf("celllength = %d, frame_x = %f, frame_y = %f\n",celllength,frame_x,frame_y);
	cv::warpAffine(*opencv_map, *opencv_map, t_mat, dst_sz);
}
int cell_coordinate(cv::Mat opencv_map, uint8_t metersize, const uint16_t laser_shift_x, const uint16_t laser_shift_y)
{
  uint8_t scanrange = 6; // meter
  uint8_t blocksize = 5; // position point size
  uint16_t opencv_map_x_pre = laser_shift_y;
	uint16_t opencv_map_y_pre = laser_shift_x;
	float theta = 0;
	uint16_t opencv_map_x = laser_shift_x+pose.pose.position.y*metersize;
	uint16_t opencv_map_y = laser_shift_y+pose.pose.position.x*metersize;
	float cell_map_ratio = (float)metersize/stepsize;
	cv::Mat cellmap(cellsize,cellsize,CV_16S,cv::Scalar(255));
	cv::Mat pathROI(cellsize,cellsize,CV_16S);
	cv::Mat cellmap_resize(cellsize,cellsize,CV_16S,cv::Scalar(255));
	maptransform(&opencv_map);
	if (opencv_map_x >= opencv_map.rows-blocksize/2) // limit x by the image size
		opencv_map_x = opencv_map.rows-1-blocksize/2;
	else if (opencv_map_x < blocksize/2)
		opencv_map_x = blocksize/2;
	if (opencv_map_y >= opencv_map.cols-blocksize/2) // limit y by the image size
		opencv_map_y = opencv_map.cols-1-blocksize/2;
	else if (opencv_map_y < blocksize/2)
		opencv_map_y = blocksize/2;

	if (pose.pose.orientation.z>=0)
		theta = 2*acos(pose.pose.orientation.w);
	else
		theta = 2*3.142-(2*acos(pose.pose.orientation.w));
	cv::Mat imageROI = opencv_map(cv::Rect(laser_shift_y,laser_shift_x,celllength*metersize,celllength*metersize));
	for (int16_t i = 0 ; i < imageROI.rows; i++) //resize original map to match cellmap
	{
		for (int16_t j = 0 ; j < imageROI.cols ; j++)
		{
			if (imageROI.at<uint8_t>(cv::Point2i(i,j)) == 0)
			{
				cellmap_resize.at<int16_t>(cv::Point2i(j/cell_map_ratio,i/cell_map_ratio))=0;
			}
			else if(imageROI.at<uint8_t>(cv::Point2i(i,j)) == 1)
				cellmap_resize.at<int16_t>(cv::Point2i(j/cell_map_ratio,i/cell_map_ratio))=1;
			else if(imageROI.at<uint8_t>(cv::Point2i(i,j)) == 120)
				cellmap_resize.at<int16_t>(cv::Point2i(j/cell_map_ratio,i/cell_map_ratio))=2;
		}
	}
	/*printf("imageROI ................................................................\n");
	for (int x = 0 ; x < imageROI.rows ; x++)
	{
		for (int y = 0 ; y < imageROI.cols ; y++)
		{
			printf("%d\t",imageROI.at<uint8_t>(cv::Point2i(x,y)));
		}
		printf("\n");
	}
	printf("cellmap_resize ...........................................................\n");
	for (int x = 0 ; x < cellmap_resize.rows ; x++)
	{
		for (int y = 0 ; y < cellmap_resize.cols ; y++)
		{
			printf("%d\t",cellmap_resize.at<int16_t>(cv::Point2i(x,y)));
		}
		printf("\n");
	}*/
	for (int16_t j = 0 ; j< cellsize ; j++ )
   	{
     		for (int16_t i = 0 ; i< cellsize ; i++ )
      		{
			if (j % 2 == 0 )
			{
				if (cellmap_resize.at<int16_t>(cv::Point2i(i,j)) <= 2 | pathmap.at<int16_t>(cv::Point2d(i+1,j+1)) == 32766)
				cellmap.at<int16_t>(cv::Point2i(i,j))=32766;
				else
				cellmap.at<int16_t>(cv::Point2i(i,j))=j*cellsize+i+1;
			}
 		  else{
				if (cellmap_resize.at<int16_t>(cv::Point2i(i,j)) <= 2 | pathmap.at<int16_t>(cv::Point2d(i+1,j+1)) == 32766)
				cellmap.at<int16_t>(cv::Point2i(i,j))=32766;
				else
				cellmap.at<int16_t>(cv::Point2i(i,j)) = j*cellsize+i+1;//(j+1)*cellsize-i;
			}
		}
	}
	/*printf("cellmap in cell coordinate..................................................................\n");
	for (int x = 0 ; x < cellmap.rows ; x++)
	{
		for (int y = 0 ; y < cellmap.cols ; y++)
		{
			printf("%d\t",cellmap.at<int16_t>(cv::Point2i(x,y)));
		}
		printf("\n");
	}*/
	pathROI = pathmap(cv::Rect(1,1,cellsize,cellsize));
	cv::addWeighted(pathROI,0,cellmap,1,0,pathROI);
	//double cellmin , cellmax;
	//cv::Point minPoint , maxPoint;
	//cv::minMaxLoc(cellmap,&cellmin,&cellmax,&minPoint,&maxPoint);
	//printf("cellmin = %f,cellmax = %f,minPoint = (%d,%d),maxPoint = (%d,%d)\n",cellmin,cellmax,minPoint.x,minPoint.y,maxPoint.x,maxPoint.y);
	/*printf("pathmap in cell coordinate...........................................................\n");
	for (int x = 0 ; x < pathmap.rows ; x++)
	{
		for (int y = 0 ; y < pathmap.cols ; y++)
		{
			printf("%d\t",pathmap.at<int16_t>(cv::Point2i(x,y)));
		}
		printf("\n");
	}*/
  //cv::namedWindow("imageROI");
  //cv::namedWindow("graymap");
  //cv::startWindowThread();
	//imshow("imageROI",imageROI);
	//imshow("graymap",opencv_map);
	return 1;
}

int sweep_scan(ros::Publisher pub , int frequency, const float path_coordinate_x, const float path_coordinate_y,float kp_ang,float ki_ang,float kd_ang)
{
	float x_speed = 0,z_speed = 0 , diff_x , diff_y;
	// PID constants for linear controller
	float kp_lin = 0.5;//0.6;
	float ki_lin = 0;//.0001;//0.35;
	float kd_lin = 0;//0.1;
	static float integral_lin = 0;
	static bool first_run = 1;
	// Prior error for Derivative
	static float prior_lin = 0, prior_ang_pos = 0, prior_ang_neg = 0, ang_diff_pos = 0,\
	 ang_diff_neg = 0, ang_diff_max = 0, max_diff;
  static int reset_odom_param = 0;
	// Maximum Roomba speed:
	float min_output_lin,max_output_lin;//0.24
  if(start_wall_follow){
    min_output_lin = -0.14;
    max_output_lin = 0.14;
  }
  else{
    min_output_lin = -0.24;
    max_output_lin = 0.24;
  }
	float min_output_ang = -0.48;//0.48
	float max_output_ang = 0.48;
    	// Calculates relative and total differences in distance as well as their absolute values
	static bool rotate_mode = 1;
  if(next_corner == 1 & start_wall_follow == 1){
      rotate_mode = 1;
    }
  if (odom_switch == 0){
  	diff_x = path_coordinate_x - odom.pose.pose.position.x;
  	diff_y = path_coordinate_y - odom.pose.pose.position.y;
  }
  else {
  	diff_x = path_coordinate_x - pose.pose.position.x;
  	diff_y = path_coordinate_y - pose.pose.position.y;
  }
	float abs_diff_x = fabs(diff_x);
	float abs_diff_y = fabs(diff_y);
	float diff = sqrt(pow(diff_x,2) + pow(diff_y,2));
  if (reset_odom_param == 0 & diff!=0){
    max_diff = diff;
    reset_odom_param = 1;
  }
	float abs_diff = fabs(diff); 
	float ang_diff,abs_ang_diff,ang_desired,derivative,continue_y = 0, yaw;
  static int acc_curve_x = 0;
  static int acc_curve_z = 0;
 	static int step_one = 0;
	static float ang_desired_old = 0;  
    	// If the Roomba is not at it's destination, PID loops will run
  float diff_distance;
    if (curve_param)
 	   diff_distance = 0.1;
    else
  	  diff_distance = 0.03;
    if(!(diff < diff_distance))
    {
      if(as_wall_sweep_status == 1){
    		start_bug1 = 0;
  		  turn_status = 1;
        step_one = 0;
        rotate_mode = 1;
  	    first_run = 1;
     	  prior_lin = 0;
     	  prior_ang_pos = 0;
     	  prior_ang_neg = 0;
        reset_odom_param = 0;
        as_wall_sweep_status = 0;
      }
	    // Conversion from Quaternion to Yaw
  	    tf::Pose pose_tf;
        if (odom_switch)
          tf::poseMsgToTF(pose.pose, pose_tf);
        else
  	      tf::poseMsgToTF(odom.pose.pose, pose_tf);
 	      yaw = tf::getYaw(pose_tf.getRotation());
        if (diff < max_diff/2 & reset_odom_param & scan_match & max_diff > 0.2 & !start_bug1){
          reset_odom = 1;
          reset_odom_param = 2;
          scan_match = 0;
        }
      // printf("waypoint = (%f, %f), pose = (%f, %f)\n",odom.pose.pose.position.x+10*cos(yaw),odom.pose.pose.position.y+10*sin(yaw),odom.pose.pose.position.x,odom.pose.pose.position.y);
      	// Calculates desired angle, difference from current angle and its absolute value 
  	  ang_desired = atan2(diff_y , diff_x);
  	  ang_diff = yaw - ang_desired;
  	  abs_ang_diff = fabs(ang_diff);
	if(first_run){
		ang_desired_old = fabs(ang_desired);
		first_run = 0;
		printf("sweep_scan x_sp = %f, y_sp = %f\n",path_coordinate_x,path_coordinate_y);
	}
	//printf("ang_desired_old = %f,ang_desired = %f,ang_desi_diff = %f,yaw = %f,ang_diff = %f,step_one = %d\n",ang_desired_old,ang_desired,ang_desired_old-fabs(ang_desired),yaw,ang_diff,step_one);
	if((ang_desired_old-fabs(ang_desired) > 0.03 | ang_desired_old-fabs(ang_desired) < -0.03) & step_one == 0 & start_bug1){
		printf("step_one = %d, start bug 1\n",step_one);
    step_one = 1;
   }
	if(step_one == 1 & ang_desired_old-fabs(ang_desired) <= 0.03 & ang_desired_old-fabs(ang_desired) >= -0.03 & start_bug1){
		start_bug1 = 0;
		turn_status = 1;
		step_one = 0;
		rotate_mode = 1;
		first_run = 1;
   printf("step_one = %d, end bug 1\n",step_one);
	}
  if(start_bug1){
    if(diff < 0.15){
  		start_bug1 = 0;
  		step_one = 0;
      rotate_mode = 1;
	    first_run = 1;
   	  prior_lin = 0;
   	  prior_ang_pos = 0;
   	  prior_ang_neg = 0;
      reset_odom_param = 0;
      return 1;
      }
  }
      // Angular PID: only runs if there is an angular offset
        // Normalizes angular difference for easier comparison
        if(ang_diff < 0) ang_diff += 2*M_PI;

        // Checks which direction Roomba should rotate depending on whichever is closer
	      if(ang_diff > M_PI && ang_diff < 2*M_PI)
	      {
	        // Normalizes angular difference for positive rotation 
	        ang_diff = 2*M_PI - ang_diff;

	        // Calculation of the derivative
	        derivative = (ang_diff - prior_ang_pos) * frequency * kd_ang;
	        // Calculation and capping of angular velocity
          if (rotate_mode == 1)
          {
            derivative = 0;
          }
          ang_diff_pos+=ang_diff;
          z_speed = kp_ang * ang_diff + derivative;
	        if(z_speed < min_output_ang)
	          z_speed = min_output_ang;
	        if(z_speed > max_output_ang)
	          z_speed = max_output_ang;
        
	        // Preparation for next derivative calculation
	        prior_ang_pos = ang_diff;
	      }
	      else
	      {
	      // Does the same as above except in the opposite direction

	        derivative = (prior_ang_neg - ang_diff) * frequency * kd_ang;
          if (rotate_mode == 1)
          {
            derivative = 0;
          }
          ang_diff_neg+=ang_diff;
	        z_speed = (kp_ang * ang_diff  + derivative) * -1;
	        if(z_speed < min_output_ang)
	          z_speed = min_output_ang;
	        if(z_speed > max_output_ang)
	          z_speed = max_output_ang;

	        prior_ang_neg = ang_diff;
	      }
      // Linear PID
      // Kicks in when the robot if at most 45 degrees off the correct direction
      // unless the robot is close to the destination, in which case the robot
      // must be in the correct direction to move
      //if(abs_ang_diff < M_PI/4 && ((abs_diff > 0.1) || (abs_diff < 0.1)))
      //if(abs_ang_diff < asin((diff/2) / 0.5))
      if (rotate_mode == 1)
      {
        if ((z_speed == max_output_ang & acc_curve_z <= frequency/2)){
          acc_curve_z++;
          z_speed = max_output_ang*acc_curve_z/(frequency/2);
        }
        if ((z_speed == min_output_ang & acc_curve_z <= frequency/2)){
          acc_curve_z++;
          z_speed = min_output_ang*acc_curve_z/(frequency/2);
        }
        if (abs_ang_diff<0.1 || abs_ang_diff>6.28)
        {
          //sendCmdVel(pub, 0, 0);
          //usleep(1000000);
          rotate_mode = 0;
          acc_curve_x = 0;
          acc_curve_z = 0;
        }
        if (z_speed < 0.1 & z_speed > 0)
          z_speed = 0.1;
        if (z_speed > -0.1 & z_speed < 0)
          z_speed = -0.1;
       	if (curve_param)
          x_speed = 0.05;
        else
          x_speed = 0;
      }
      //if((abs_ang_diff < M_PI/4 && ((abs_diff > 0.1) || (abs_diff < 0.1))) || (abs_ang_diff > M_PI*7/4 && ((abs_diff > 0.1) || (abs_diff < 0.1))))
      if (rotate_mode == 0)
      {
	    if(slow_laser == 1){
    		if(diff>=0.6){
    		    start_bug1 = 1;
    		    if (odom_switch == 0)
    		  	  printf("start bug1 with obstacle in (x = %f , y = %f), (x_sp = %f, y_sp = %f).......\n",odom.pose.pose.position.x,odom.pose.pose.position.y,path_coordinate_x,path_coordinate_y);
    		    else
    		  	  printf("start bug1 with obstacle in (x = %f , y = %f), (x_sp = %f, y_sp = %f).......\n",pose.pose.position.x,pose.pose.position.y,path_coordinate_x,path_coordinate_y);
    		}
    		else {
    			if (odom_switch == 0){
    			    delta_x = path_coordinate_x - odom.pose.pose.position.x;
    			    delta_y = path_coordinate_y - odom.pose.pose.position.y;
    			}
    			else{
    			    delta_x = path_coordinate_x - pose.pose.position.x;
    			    delta_y = path_coordinate_y - pose.pose.position.y;
    			}
  		    printf("delta_x = %f, delta_y = %f\n",delta_x,delta_y);
  		    start_bug1 = 0;
          step_one = 0;
  		    rotate_mode = 1;
  		    first_run = 1;
   	      prior_lin = 0;
     	    prior_ang_pos = 0;
     	    prior_ang_neg = 0;
  		    reset_odom_param = 0;
  		    if (odom_switch == 0)
  		        printf("stop by laser and arrival (x = %f , y = %f), (x_sp = %f, y_sp = %f).......\n",odom.pose.pose.position.x,odom.pose.pose.position.y,path_coordinate_x,path_coordinate_y);
  		    else
  		        printf("stop by laser and arrival (x = %f , y = %f), (x_sp = %f, y_sp = %f).......\n",pose.pose.position.x,pose.pose.position.y,path_coordinate_x,path_coordinate_y);
  		    return 1 ;
      	}
      }
	    // Same as angular PID except in linear direction
	    // Since Roomba is slow, integral only starts building up at closer 
        if (ang_diff_max < ang_diff)
          ang_diff_max = ang_diff;
	      if(abs_diff < 0.3)
	      {
	        integral_lin += abs_diff * ki_lin;
	        if(integral_lin > max_output_lin)
	          integral_lin = max_output_lin/2;
	      }

	      static float derivative_lin = (abs_diff - prior_lin) * frequency;
	
        x_speed = kp_lin * abs_diff + integral_lin + kd_lin * derivative_lin;
        
	      if(x_speed > max_output_lin)
          x_speed= max_output_lin;
        
        if ((x_speed >= max_output_lin/2 & acc_curve_x <= frequency)){
          acc_curve_x++;
          x_speed = max_output_lin*acc_curve_x/(frequency);
        }
        if (x_speed > 0.12 & stop_sonar == 3)
          x_speed = 0.12;
        if (x_speed > 0.06 & stop_sonar == 2)
          x_speed = 0.06;
       	if (x_speed < 0.05)
	  x_speed = 0.05;       
        prior_lin = diff;
      }
    }
      // Destination reached if all path_coordinate are reached,
      // otherwise resets everything for next path_coordinate
    else
    {
  		start_bug1 = 0;
		  step_one = 0;
      rotate_mode = 1;
	    first_run = 1;
   	  prior_lin = 0;
   	  prior_ang_pos = 0;
   	  prior_ang_neg = 0;
      reset_odom_param = 0;
      if (odom_switch == 0)
        printf("arrival (x = %f , y = %f), (x_sp = %f, y_sp = %f).......\n",odom.pose.pose.position.x,odom.pose.pose.position.y,path_coordinate_x,path_coordinate_y);
      else
        printf("arrival (x = %f , y = %f), (x_sp = %f, y_sp = %f).......\n",pose.pose.position.x,pose.pose.position.y,path_coordinate_x,path_coordinate_y);
      return 1 ;
    }
    // Publishes message, increases ROS counter, then sleeps until next iteration
    //printf("float kp_ang = %f,float ki_ang = %f,float kd_ang = %f\n",kp_ang, ki_ang, kd_ang);
    /*if (odom_switch == 0)
      printf("rota=%d,x_sp=%3.3f,y_sp=%3.3f,x=%3.3f,y=%3.3f,x_vel=%3.3f,z_vel=%3.3f,int_lin=%3.3f,ang_dif=%3.3f,yaw=%3.3f,ang_desi=%3.3f,deriv=%3.3f\n",rotate_mode,path_coordinate_x,path_coordinate_y,odom.pose.pose.position.x,odom.pose.pose.position.y,x_speed,z_speed,integral_lin,yaw-ang_desired,yaw,ang_desired,derivative);
    else
      printf("rota=%d,x_sp=%3.3f,y_sp=%3.3f,x=%3.3f,y=%3.3f,x_vel=%3.3f,z_vel=%3.3f,int_lin=%3.3f,ang_dif=%3.3f,yaw=%3.3f,ang_desi=%3.3f,deriv=%3.3f\n",rotate_mode,path_coordinate_x,path_coordinate_y,pose.pose.position.x,pose.pose.position.y,x_speed,z_speed,integral_lin,yaw-ang_desired,yaw,ang_desired,derivative);*/

	if(!start_bug1 | sweep_mode == 1)
    sendCmdVel(pub, x_speed, z_speed);
  return 0 ;
}
void occupancyGridToGrayCvMat(const nav_msgs::OccupancyGrid *map, cv::Mat *im)
{
   uint8_t *data = (uint8_t*) map->data.data(),
            testpoint = data[0];
   bool mapHasPoints = false;

   // transform the map in the same way the map_saver component does
   for (size_t i=0; i<map->data.size(); i++)
   {
     		if (data[i] == 0)
			im->at<uint8_t>(cv::Point2i(i/map->info.height,i%map->info.width)) = 240;//Vec3d sequence == BGR, Scanned area
     		else if (data[i] == 100)
			im->at<uint8_t>(cv::Point2i(i/map->info.height,i%map->info.width)) = 1;// Scanned edge
     		else 
			im->at<uint8_t>(cv::Point2i(i/map->info.height,i%map->info.width)) = 120;// not Scanned area

    // just check if there is actually something in the map
    if (i!=0) mapHasPoints = mapHasPoints || (data[i] != testpoint);
    testpoint = data[i];
  }

  // sanity check
  if (!mapHasPoints) { ROS_WARN("map is empty, ignoring update."); }

}

float FindMedian(float* a, int n){
    float tmp;
    int zerocount = 0;
    for (int i=1; i<n; i++) {
	     int zerocount = 0;
	     for (int j=0; j<n-i; j++){
		     if (a[i]<=0.1)
			zerocount++;
    		     if (zerocount>=1)
			return 0;
		     if (a[j+1] < a[j]) { 
		     tmp = a[j]; 
		     a[j] = a[j+1];
		     a[j+1] = tmp;
		     }
   	     }
    }
    /*for(int ix=0;ix<n;ix++)
    printf("a[%d] = %f\n",ix,a[ix]);
    printf("\n");*/
    return a[n/2];
}

float FindMean(float* a, int n){
  float total = 0;
  int counter = 0;
  int zerocount = 0;
  for(int i = 0; i<n; i++){
    if (a[i]<=0.1)
	zerocount++;
    if (zerocount>=1)
	return 0;
    total = total + a[i];
    if (a[i]>=0.1)
    counter++;
  }
  if(counter == 0)
    counter = 1;
  return total/counter;
}

void wall_follow(float minedge_xx, float minedge_yy, float maxedge_xx, float maxedge_yy, ros::Publisher pub, int frequency, float kp_ang, float ki_ang, float kd_ang){
  static float start_x,start_y;
  static int status = 0;
  static int avoidance_status = 1;
  static double wall_follow_secs = ros::Time::now().toSec(),wf_reset_odom_sec = ros::Time::now().toSec();
  if(dis_p2p(pose.pose.position.x,pose.pose.position.y,odom.pose.pose.position.x,odom.pose.pose.position.y)>0.1 & ros::Time::now().toSec()-wf_reset_odom_sec > 10 & sweep_mode == 0){
    //reset_odom = 1;
    printf("reset odom in wall follow, diff = %f, delta_Time = %f\n",dis_p2p(pose.pose.position.x,pose.pose.position.y,odom.pose.pose.position.x,odom.pose.pose.position.y),ros::Time::now().toSec()-wf_reset_odom_sec);
    wf_reset_odom_sec = ros::Time::now().toSec();
  }
  if(start_wall_follow == 1 & (stop_laser == 1 | first_bump >= 0))
  {
     start_x = pose.pose.position.x;
     start_y = pose.pose.position.y;
     start_wall_follow = 2;
     printf("start wallfollow\n");
  }
  if(dis_p2p(start_x,start_y,pose.pose.position.x,pose.pose.position.y)>=0.6 && start_wall_follow == 2)
  {
    start_wall_follow = 3;
  }
  if(dis_p2p(start_x,start_y,pose.pose.position.x,pose.pose.position.y) <= 0.15 && start_wall_follow == 3)
  {
    if(first_bump >= 0)
    {
	crosscell_x = crosspoint[first_bump].x;
	crosscell_y = crosspoint[first_bump].y;
	minedge_x = crosspoint[first_bump].MinBoundary_x;
	minedge_y = crosspoint[first_bump].MinBoundary_y;
	maxedge_x = crosspoint[first_bump].MaxBoundary_x;
	maxedge_y = crosspoint[first_bump].MaxBoundary_y;
	crosspoint[first_bump].discover = 1;
    }
    else
    {
	for(int i = 0; i < 100; i++)
	{
	   if(crosspoint[i].discover != 0)
	   {
	      	crosscell_x = crosspoint[i].x;
		crosscell_y = crosspoint[i].y;
		minedge_x = crosspoint[i].MinBoundary_x;
		minedge_y = crosspoint[i].MinBoundary_y;
		maxedge_x = crosspoint[i].MaxBoundary_x;
		maxedge_y = crosspoint[i].MaxBoundary_y;
		crosspoint[i].discover = 1;
	   }
	   break;
	}
    }
    LastCrossBoundary = LastCrossConer;
    LastCrossConer = 0;
    start_wall_follow = 0;
    stop_laser = 0;
    next_corner = 3;
    sweep_mode = 0;
    first_bump = -1;
    avoidance_status = 1;
    as_wall_sweep_status = 1;
    printf("minedge_x = %f,minedge_y = %f,maxedge_x = %f,maxedge_y = %f\n",minedge_x,minedge_y,maxedge_x,maxedge_y);
    printf("crosscell_x = %f,crosscell_y = %f\n",crosscell_x,crosscell_y);
    printf("end wallfollow\n");
    return;
  }
  
	float frontdatar[5]={0},sidedatar1[5]={0},sidedatar2[5]={0},
        frontdatal[5]={0},sidedatal1[5]={0},sidedatal2[5]={0};
	float frontdata1[5]={0},frontdata2[5]={0},frontdata3[5]={0},frontdata4[5]={0},frontdata5[5]={0};
	float frontdata6[5]={0},frontdata7[5]={0},frontdata8[5]={0},frontdata9[5]={0},frontdata10[5]={0};
	for (int j = 0 ; j<=4 ; j+=1){
		//Right side
		frontdatar[j] = laser.ranges[j+348];
   if(frontdatar[j]<=0.4 & frontdatar[j]>0.1)
    printf("frontdatar[%d] = %f\n",j,frontdatar[j]);
    sidedatar1[j] = laser.ranges[j+323];
		sidedatar2[j] = laser.ranges[j+303];
		//Left side
		frontdatal[j] = laser.ranges[j+3];
    sidedatal1[j] = laser.ranges[j+29];
		sidedatal2[j] = laser.ranges[j+53];
    //////
		frontdata1[j] = laser.ranges[j+18];
		frontdata2[j] = laser.ranges[j+20];
		frontdata6[j] = laser.ranges[j+336];
		frontdata7[j] = laser.ranges[j+334];        
	}
  //right side
	float frontdistr = FindMean(frontdatar,5);
  float sidedistr1 = FindMedian(sidedatar1,5);
  float sidedistr2 = FindMedian(sidedatar2,5);
  //left side
	float frontdistl = FindMean(frontdatal,5);
  float sidedistl1 = FindMedian(sidedatal1,5);
  float sidedistl2 = FindMedian(sidedatal2,5);
  //
 	float frontdist1 = FindMean(frontdata1,5);
	float frontdist2 = FindMean(frontdata2,5);
 	float frontdist6 = FindMean(frontdata6,5);
	float frontdist7 = FindMean(frontdata7,5);
  float laser_[3] = {sidedistr2,sidedistr1,frontdistr}; 
  int min_index;
  
  ////////////Find min_index////////////////////////////////////////////////////////////////////
  if(avoidance_status == 1){
    float smallest = 10;
    for(int i = 0; i < 3; i++)
    {
      if(laser_[i] < smallest & laser_[i] != 0)
        {
          smallest = laser_[i];
          min_index = i;
        }  
    }
////////////////////find nearest wall to wall  follow/////////////////////////////////////
    if(smallest < 2)
    {
      /*if(min_index > 3){
        for (int a = 0 ; a<=6*(min_index - 3); a+=1){
		          usleep(150000);
				      sendCmdVel(pubMessage, 0, -0.8);
		          }
        }*/
      if(min_index < 2){
        for (int a = 0 ; a<=4*(-min_index+2); a+=1){
				      usleep(150000);
				      sendCmdVel(pubMessage, 0, -0.8);
			        }
        } 
    avoidance_status = 2;    
    }
///////////////////switch detect boundary status/////////////////////////////////
    else
    {
    	wall_follow_secs = ros::Time::now().toSec();
	avoidance_status = 2;	
    }
  }
  /////////////////////////////////////////////////////////////////////////////////////////////////
  if(sweep_mode == 2)
  {
    offset_bound = 0.3;
  }
  else
    offset_bound = 0;
  if(next_corner == 1){
    findedge(minedge_x, minedge_y, maxedge_x, maxedge_y);
  }
  if(sweep_mode == 1){
      status = sweep_scan(pub, frequency, edgepoint_x, edgepoint_y, kp_ang, ki_ang, kd_ang);
      if(status == 1)
      {
        //if(first_bump != 0) //?????????????????
        sweep_mode = 2;
        last_corner = next_corner;
        next_corner = 1;
      }
     }   
    if((frontdistr <= 0.32 & frontdistr >= 0.1) | (frontdistl <= 0.32 & frontdistl >= 0.1) | (frontdist1 <= 0.32 & frontdist1 >= 0.1) | (frontdist6 <= 0.32 & frontdist6 >= 0.1))
      stop_laser = 1;
    else if((frontdistr <= 0.37 & frontdistr >= 0.1) | (frontdistl <= 0.37 & frontdistl >= 0.1))
      slow_laser = 2;
    else if((frontdistr <= 0.42 & frontdistr >= 0.1) | (frontdistl <= 0.42 & frontdistl >= 0.1))
      slow_laser = 3;
    else if((frontdistr <= 0.47 & frontdistr >= 0.1) | (frontdistl <= 0.47 & frontdistl >= 0.1))
      slow_laser = 4;
    else if((frontdistr <= 0.52 & frontdistr >= 0.1) | (frontdistl <= 0.52 & frontdistl >= 0.1))
      slow_laser = 5;
    else
      slow_laser = 0;
    if(!stop_laser & first_bump == -1)
    {
        sendCmdVel(pubMessage, 0.2, 0);
	if (ros::Time::now().toSec()-wall_follow_secs > 4.0f)
        {
          wall_follow_secs = ros::Time::now().toSec();
	  next_corner = 1; 
        }
    }
	  if(avoidance_status == 2 & stop_laser & start_wall_follow != 0){
		  if(frontdistr <= 0.32 & frontdistr>=0.1 & wall_stop_wf == 1){
			  for (int a = 0 ; a<=12 ; a+=1){
				  usleep(150000);
				  sendCmdVel(pubMessage, 0, 0.8);
			    }
          sweep_mode = 0;
			    wall_stop_wf = 0;
         printf("status = %d\n",status);
       if(status < 1){
         as_wall_sweep_status = 1;
         wall_follow_secs = ros::Time::now().toSec();
         }
          //printf("turn 90\n");
	    } 
		  else if((frontdistr>0.32 | frontdistr==0) & sweep_mode == 0){
			  float z_speed = (0.28 - sidedistr2)*8;//0.291
			  float x_speed = 0.14;
			  if(sidedistr2 == 0)
				  z_speed = -0.48;
			  if(z_speed >= 0.48)
				  z_speed = 0.48;
			  if(z_speed <= -0.48)
				  z_speed = -0.48;
			  if(sidedistr2<0.2 & sidedistr2>0.1)
				  x_speed = 0.02;
			  sendCmdVel(pubMessage, x_speed, z_speed);
			  wall_stop_wf = 1;
        if(next_corner != 1){        
          if (ros::Time::now().toSec()-wall_follow_secs > 4.0f){
					//printf("cellpath.status = %d\n",cellpath.status);
				      wall_follow_secs = ros::Time::now().toSec();
              next_corner = 1;                        
			      }
        }
        //printf("wall_follow\n");
		  } 
    }
    //printf("edgepoint_x = %f,edgepoint_y = %f,sweep_mode = %d,next_corner = %d,offset_bound = %f\n",edgepoint_x,edgepoint_y,sweep_mode,next_corner,offset_bound);
}

/// find boundary   update "next_corner" parameter
void findedge(float minedge_xx, float minedge_yy, float maxedge_xx, float maxedge_yy)
{
  float crosscell_x_tmp,crosscell_y_tmp;
  if((odom.pose.pose.position.y > maxedge_yy - offset_bound) & last_corner != 2)
  {
    sweep_mode = 1;
    edgepoint_x = minedge_xx;
    edgepoint_y = maxedge_yy;
    next_corner = 2;
    if(LastCrossBoundary != 4)
    {
    	if(first_bump == -1)
    	{
      		LastCrossConer = next_corner;    
      		crosscell_x_tmp = pose.pose.position.x - robot_body_x;
      		crosscell_y_tmp = maxedge_yy + robot_body_y;
     		crosspoint[sumCrossPoint]={crosscell_x_tmp,crosscell_y_tmp,minedge_xx,maxedge_yy,maxedge_xx,maxedge_yy + celllength,0};
      		first_bump = sumCrossPoint;
      		sumCrossPoint++;
    	}
    	else
    	{    
      		crosscell_x_tmp = pose.pose.position.x - robot_body_x;
      		crosscell_y_tmp = maxedge_yy + robot_body_y;
      		crosspoint[sumCrossPoint]={crosscell_x_tmp,crosscell_y_tmp,minedge_xx,maxedge_yy,maxedge_xx,maxedge_yy + celllength,0};
      		sumCrossPoint++;
    	}
    }
      
  }
  if((odom.pose.pose.position.x < minedge_xx + offset_bound) & last_corner != 3)
  {
    sweep_mode = 1;
    edgepoint_x = minedge_xx;
    edgepoint_y = minedge_yy;
    next_corner = 3;
    if(LastCrossBoundary != 5)
    {
    	if(first_bump == -1)
    	{
      		LastCrossConer = next_corner; 
      		crosscell_x_tmp = minedge_xx - robot_body_x;
      		crosscell_y_tmp = pose.pose.position.y - robot_body_y;
      		crosspoint[sumCrossPoint]={crosscell_x_tmp,crosscell_y_tmp,minedge_xx - celllength,minedge_yy,minedge_xx,maxedge_yy,0};
      		first_bump = sumCrossPoint;
      		sumCrossPoint++;
    	}
    	else
    	{
      		crosscell_x_tmp = minedge_xx - robot_body_x;
      		crosscell_y_tmp = pose.pose.position.y - robot_body_y;
      		crosspoint[sumCrossPoint]={crosscell_x_tmp,crosscell_y_tmp,minedge_xx - celllength,minedge_yy,minedge_xx,maxedge_yy,0};
      		sumCrossPoint++;
    	}
    }
  }
  if((odom.pose.pose.position.y < minedge_yy + offset_bound) & last_corner != 4)
  {
    sweep_mode = 1;
    edgepoint_x = maxedge_xx;
    edgepoint_y = minedge_yy;
    next_corner = 4;
    if(LastCrossBoundary != 2)
    {
    	if(first_bump == -1)
    	{
      		LastCrossConer = next_corner;
      		crosscell_x_tmp = pose.pose.position.x + robot_body_x;
      		crosscell_y_tmp = minedge_yy - robot_body_y;
      		crosspoint[sumCrossPoint]={crosscell_x_tmp,crosscell_y_tmp,minedge_xx,minedge_yy - celllength,maxedge_xx,minedge_yy,0};
      		first_bump = sumCrossPoint;
      		sumCrossPoint++;
    	}
    	else
    	{
      		crosscell_x_tmp = pose.pose.position.x + robot_body_x;
      		crosscell_y_tmp = minedge_yy - robot_body_y;
      		crosspoint[sumCrossPoint]={crosscell_x_tmp,crosscell_y_tmp,minedge_xx,minedge_yy - celllength,maxedge_xx,minedge_yy,0};
      		sumCrossPoint++;
    	}
    }
  }
  if((odom.pose.pose.position.x > maxedge_xx + 0.15 - offset_bound) & last_corner != 5)
  {
    sweep_mode = 1;
    edgepoint_x = maxedge_xx;
    edgepoint_y = maxedge_yy;
    next_corner = 5;
    if(LastCrossBoundary != 3)
    {
    	if(first_bump == -1)
    	{
      		LastCrossConer = next_corner;
      		crosscell_x_tmp = maxedge_xx + robot_body_x;
      		crosscell_y_tmp = pose.pose.position.y + robot_body_y;
      		crosspoint[sumCrossPoint]={crosscell_x_tmp,crosscell_y_tmp,maxedge_xx,minedge_yy,maxedge_xx + celllength,maxedge_yy,0};
      		first_bump = sumCrossPoint;
      		sumCrossPoint++;
    	}    
    	else
    	{
      		crosscell_x_tmp = maxedge_xx + robot_body_x;
      		crosscell_y_tmp = pose.pose.position.y + robot_body_y;
     	 	  crosspoint[sumCrossPoint]={crosscell_x_tmp,crosscell_y_tmp,maxedge_xx,minedge_yy,maxedge_xx + celllength,maxedge_yy,0};
      		sumCrossPoint++;
    	}
    }
  }
  last_corner = 1;
}
float dis_p2p(float x1, float y1, float x2, float y2)
{
	float dis;
	dis = sqrt(pow(x1-x2,2)+pow(y1-y2,2));
	return dis;
}
void save_cellmap(){
	//static bool image_run_onece = 1;
	if (/*image_run_onece == 1 & */!start_wall_follow){
    cv::Mat graymap(mapmsg.info.height, mapmsg.info.width, CV_8UC1);
		occupancyGridToGrayCvMat(&mapmsg,&graymap); 
 		//cv::Mat erodeStruct = getStructuringElement(cv::MORPH_RECT, cv::Size(erode_param, erode_param)); 
		//erode(graymap, graymap, erodeStruct);
		cell_status = cell_coordinate(graymap,(uint8_t)(1/mapmsg.info.resolution),(uint16_t)((0-mapmsg.info.origin.position.x)/mapmsg.info.resolution),(uint16_t)((0-mapmsg.info.origin.position.y)/mapmsg.info.resolution));
		printf("Map resloution = %u, position.x = %f, position.y = %f, width = %u, height = %u\n",(uint8_t)(1/mapmsg.info.resolution),mapmsg.info.origin.position.x,mapmsg.info.origin.position.y,mapmsg.info.width,mapmsg.info.height);
		//printf("imageCallback map.x = %u, map.y = %u, sin = %f, cos = %f\n",(uint16_t)((0-mapmsg.info.origin.position.x)/mapmsg.info.resolution),(uint16_t)((0-mapmsg.info.origin.position.y)/mapmsg.info.resolution),sin(M_PI),cos(M_PI));
		//image_run_onece = 0;
	}
}
