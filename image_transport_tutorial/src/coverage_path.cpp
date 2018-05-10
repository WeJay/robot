/*///////////////////////////////////////////////////////////////////
	complete converage path
///////////////////////////////////////////////////////////////////*/
#include <ros/ros.h>
#include <cstdio>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include "nav_msgs/OccupancyGrid.h"
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include "std_msgs/String.h"
#include <math.h>
#include <iostream>
#include <nav_msgs/Odometry.h>				// odom
#include <nav_msgs/Path.h>
#include <irobotcreate2/Bumper.h>		// bumper
#include <irobotcreate2/RoombaIR.h>		// ir_bumper cliff
#include <irobotcreate2/IRCharacter.h>	// ir_character

//#include <kdl/frames.hpp>


using std::string;
nav_msgs::Path path;
nav_msgs::Odometry odom;
nav_msgs::Path path_astar;
sensor_msgs::LaserScan laser;
geometry_msgs::PoseStamped pose;
int start_pose = 0,start_path = 0,end_path = 0,stop_sonar = 0,stop_bumper = 0, stop_laser = 0,cell_status = 1, system_status = 4,odom_switch,image_run_onece = 1,erode_param;
#define meter2pixel 50 //meter2pixel : pixel number per meter (pixel/meter) that is define by source of map ,20 = 0.05cm
uint8_t stepsize = 5; //stepsize is step(pixel) per one meter (pixel/meter) so 0.2 meter per pixel
uint8_t celllength = 2;//cell length (meter) of cellmap
uint8_t cellsize = stepsize*celllength; // pixel number of cell, +1 is for axis path
cv::Mat pathmap(cellsize+2,cellsize+2,CV_16S,cv::Scalar(32767));
string map_frame_id;
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

void occupancyGridToGrayCvMat(const nav_msgs::OccupancyGrid *map, cv::Mat *im);
void occupancyGridToCvMat(const nav_msgs::OccupancyGrid *map, cv::Mat *im);
void sonarCallback(const irobotcreate2::RoombaIR msg);
void bumperCallback(const irobotcreate2::Bumper msg);
void sendCmdVel(ros::Publisher pub, float linear, float angle);
void drawArrow(cv::Mat& img, cv::Point pStart, cv::Point pEnd, int len, int alpha,cv::Scalar color, int thickness, int lineType);
void showMapInfo(cv::Mat opencv_map, uint8_t metersize, const float laser_shift_x, const float laser_shift_y);
void odomCallBack(const nav_msgs::Odometry msg);
void pathCallBack(const nav_msgs::Path msg);
void poseCallback(const geometry_msgs::PoseStamped& msg);
void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
void imageCallback(const nav_msgs::OccupancyGrid& msg);
int cell_coordinate(cv::Mat opencv_map, uint8_t metersize, const float laser_shift_x, const float laser_shift_y);
int sweep_scan(ros::Publisher pub, int frequency , const float path_coordinate_x, const float path_coordinate_y,float kp_ang,float ki_ang,float kd_ang);
void cell_sweepscan(const ros::Publisher pub,const int rate, cell_path_info *sweeppath);
void test_sweepscan(const ros::Publisher pub,const int rate,float kp_ang,float ki_ang,float kd_ang);
void increment_sweepscan(const ros::Publisher pub,const int rate,float kp_ang,float ki_ang,float kd_ang, cell_path_info *sweeppath);
void where_to_go(int x,int y,cv::Mat path, cell_path_info *pathpoint);
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
void movetogoal(double x, double y, double theta);
cell_path_info cellpath;
bool end_path_record = 1;
int main(int argc, char **argv)
{
	cellpath.x = 1;//(int16_t)(pose.pose.position.x*stepsize+1);
	cellpath.y = 1;//(int16_t)(pose.pose.position.y*stepsize+1);
	cellpath.next_x = 0;
	cellpath.next_y = 0;
	cellpath.tan = 1;
	cellpath.next_tan = 0;
	cellpath.status = 0;
  	double sweep_scan_secs = 0 ;
	int ros_loop_rate = 40; // Message publishing rate: Hertz
	pose.pose.position.x = 0;

	ros::init(argc, argv, "Liteon_image_listener");
	ros::NodeHandle n_slam;
  ros::NodeHandle n_param("~");
  
	ros::Subscriber sub_pose = n_slam.subscribe("pose", 100 , poseCallback);
	ros::Subscriber sub_laser = n_slam.subscribe("scan", 100 , scanCallback);
	ros::Subscriber sub_image = n_slam.subscribe("map", 100, imageCallback);
  ros::Subscriber sub = n_slam.subscribe("odom", 1000, odomCallBack);
  ros::Subscriber sub_path = n_slam.subscribe("path", 1000, pathCallBack);
  ros::Publisher pub_path = n_slam.advertise<nav_msgs::Path>("path", 100);
  ros::Subscriber sonar_pub = n_slam.subscribe<irobotcreate2::RoombaIR>("sonar", 50 ,sonarCallback);
  ros::Subscriber bumper_pub = n_slam.subscribe<irobotcreate2::Bumper>("bumper", 50 ,bumperCallback);
  double P_vel, I_vel, D_vel;
  ros::Publisher pubMessage = n_slam.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
	n_param.param<double>("p_vel", P_vel, 1.2f);
  n_param.param<double>("i_vel", I_vel, 0.0f);
  n_param.param<double>("d_vel", D_vel, 0.1f);
  n_param.param<int>("odom_switch", odom_switch, 0);
  n_param.param<int>("erode_param", erode_param, 10);
  n_param.param<string>("map_frame_id", map_frame_id, "odom");
  ros::Rate loop_rate(30);
//movetogoal(0,0,0);
 		while(n_slam.ok())
		{
			/*if (system_status == 2)
			{	
				int leave_dock_status = sweep_scan(pubMessage,ros_loop_rate, 0.2f, 0.0f,2,0,0.1);
				if (leave_dock_status == 1)
					system_status = 1;
			}
			if (system_status == 1)
			{
				float pose_diff = sqrt(pow(pose.pose.position.x,2) + pow(pose.pose.position.y,2));
				if (pose_diff < 1/(float)stepsize)
					system_status = 0;
			}	
			if (system_status == 0)
			{
				cell_sweepscan(pubMessage, ros_loop_rate, &cellpath);
			}*/
			if (ros::Time::now().toSec()-sweep_scan_secs > 1.0f/(float)ros_loop_rate){
				//printf("sweep_scan  secs = %f\n",ros::Time::now().toSec()-sweep_scan_secs);
				//increment_sweepscan(pubMessage,ros_loop_rate,P_vel,I_vel,D_vel,&cellpath);
        test_sweepscan(pubMessage,ros_loop_rate,P_vel,I_vel,D_vel);
        pub_path.publish(path);
				sweep_scan_secs = ros::Time::now().toSec();
			}
      
      //printf("x = %f/n",path_astar.poses[0].pose.position.x);
      //   
      if (image_run_onece == 0){

      }
			loop_rate.sleep();
      ros::spinOnce();
		}
}

void pathCallBack(const nav_msgs::Path msg)
{
  path_astar = msg;
  start_path = 1;
}
void odomCallBack(const nav_msgs::Odometry msg)
{
	odom = msg;
  start_pose = 1;
}

void poseCallback(const geometry_msgs::PoseStamped& msg)
{
	pose = msg;
	int sweeppath_x = (int16_t)(pose.pose.position.x*stepsize+1);
	int sweeppath_y = (int16_t)(pose.pose.position.y*stepsize+1);
	//if (sweeppath_x>=1 & sweeppath_x <pathmap.rows-1 & sweeppath_y>=1 & sweeppath_y <pathmap.cols-1) //fill the 32766 in pathmap for position
		//pathmap.at<int16_t>(cv::Point2i(sweeppath_x,sweeppath_y))=32766;
	//printf("poseCallback, x_sp = %f, y_sp = %f,sweeppath_x = %d,sweeppath_y = %d\n",pose.pose.position.x,pose.pose.position.y,sweeppath_x,sweeppath_y);
	start_pose = 1;
}

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
	//laser = scan;
        laser = *scan;
////////////////////// For Arima Lidar ////////////////////////////
      /*for(int i=0; i<360; i++)
        {
               	if (i<180)
               		laser.ranges[i] = scan->ranges[i+180];
              	else
                   	laser.ranges[i] = scan->ranges[i-180];
        }*/
////////////////////// For Arima Lidar ////////////////////////////
	/*static int count_stop_laser = 0;  /////////////////////// Lidar obstacle avoidance ////////////////////////////
	for (int i = 90 ; i<=270 ; i+=10)
	{
	//printf("stop_laser = %d,count_stop_laser = %d,laser.ranges[%d] = %3.3f\n",stop_laser,count_stop_laser,i,laser.ranges[i]);
		if (laser.ranges[i]>=0 & laser.ranges[i]<0.21)
		{
			count_stop_laser = 0;
			stop_laser = 1;
		}
		else if (laser.ranges[i]>=0.21 & stop_laser == 1)
			count_stop_laser++;
	}
	if (count_stop_laser >= 180)
		stop_laser = 0;*/
} 

void imageCallback(const nav_msgs::OccupancyGrid& msg)
{
	/*cv::Mat map(msg.info.height, msg.info.width, CV_8UC3, CV_RGB(0,0,0));
	occupancyGridToCvMat(&msg, &map);
	showMapInfo(map,meter2pixel,0,0);*/
	/*static cv::Mat graymap(msg.info.height, msg.info.width, CV_8UC1);
  if (image_run_onece == 1){
	  occupancyGridToGrayCvMat(&msg,&graymap);
 		cv::Mat erodeStruct = getStructuringElement(cv::MORPH_RECT, cv::Size(erode_param, erode_param)); 
		erode(graymap, graymap, erodeStruct);
    cell_status = cell_coordinate(graymap,meter2pixel,0,0);
    image_run_onece = 0;
  }*/
	
	//printf("system_status = %d, position.x = %f, position.y = %f\n",system_status,pose.pose.position.x,pose.pose.position.y);
}

void sonarCallback(const irobotcreate2::RoombaIR msg)
{
  if (msg.signal >=200){
    stop_sonar = 3;
  }
  else if (msg.signal >=150 & msg.signal < 200){
    stop_sonar = 2;
  }
  else if (msg.signal > 0 & msg.signal < 150){
    stop_sonar = 1;
  }
  else
    stop_sonar = 0;
  //printf("sonar : state = %d, signal = %d\n",msg.state,msg.signal);
}

void bumperCallback(const irobotcreate2::Bumper msg)
{
  if (msg.left.state == 1 | msg.right.state == 1)
    stop_bumper = 1;
  else
    stop_bumper = 0;
  //printf("left = %d, right = %d, stop_bumper = %d\n",msg.left.state,msg.right.state,stop_bumper);
}

void sendCmdVel(ros::Publisher pub, float linear, float angle)
{
   	static geometry_msgs::Twist Twist;
    if (0/*stop_bumper == 1 | stop_sonar == 1*/){
      Twist.linear.x = 0;
  		Twist.angular.z = 0;
    }
    else{
  		Twist.linear.x = linear;
  		Twist.angular.z = angle;
    }
	//pub.publish(Twist);
	//printf("sendcomdel Twist.linear.x=%3.3f,Twist.angular.z = %3.3f\n ",Twist.linear.x,Twist.angular.z);
	usleep(100);
}

void cell_sweepscan(const ros::Publisher pub,const int rate, cell_path_info *sweeppath)
{
	static float direction = 0;
	static int status_sweep = 0;
	if (start_pose == 1 & cell_status == 0)
	{
		if (sweeppath->tan != sweeppath->next_tan)
			status_sweep = sweep_scan(pub,rate,((sweeppath->x-1)/(float)stepsize),((sweeppath->y-1)/(float)stepsize),2,0,0.1);	
		if (sweeppath->status == 0 & status_sweep == 1 & sweeppath->x < pathmap.rows-1)
		{
			//where_to_go(sweeppath->x,sweeppath->y,pathmap,sweeppath);
	//printf("pre_x = %d , pre_y = %d , x = %d , y = %d , next_x = %d , next_y = %d , x_go = %3.3f , y_go = %3.3f, point = %d, tan = %3.3f, next_tan = %3.3f\n",sweeppath->pre_x,sweeppath->pre_y,sweeppath->x,sweeppath->y,sweeppath->next_x,sweeppath->next_y,(sweeppath->x-1)/(float)stepsize,(sweeppath->y-1)/(float)stepsize,pathmap.at<int16_t>(cv::Point2d(sweeppath->x,sweeppath->y)),sweeppath->tan,sweeppath->next_tan);
		}
	}
}

void test_sweepscan(const ros::Publisher pub,const int rate,float kp_ang,float ki_ang,float kd_ang)
{
	if (start_pose == 1 && start_path == 1 && end_path == 0)
	{
    static int path_counter = 0, path_number_count = 0;
    if (path_number_count == 0){
		for (int i = 0 ; i < pathmap.rows ; i++){
			for (int j = 0 ; j < pathmap.cols ; j++){
				if (pathmap.at<int16_t>(cv::Point2i(i,j))<32766){
					path_number_count++;
                         printf("path_number_count = %d\n",path_number_count);
				}
			}
		}
    }
		static int path_size = path_number_count;
    float path_coordinate_x[path_size] , path_coordinate_y[path_size];
		  float ba = 2.0;
      path.poses.resize(path_size);
      path.header.frame_id = map_frame_id;//"odom";
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
  		path_coordinate_x[14] = 0*ba;
  		/*path_coordinate_x[15] = 0*ba;
  		path_coordinate_x[16] = 0*ba;
   		path_coordinate_x[17] = 0.5*ba;
   		path_coordinate_x[18] = 0.5*ba;
   		path_coordinate_x[19] = 0*ba;
   		path_coordinate_x[20] = 0*ba;
      path_coordinate_x[21] = 0.5*ba;*/
  
  		float bb = 2;
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
  		path_coordinate_y[14] = 0*bb;
  		/*path_coordinate_y[15] = 0.7*bb;
  		path_coordinate_y[16] = 0.8*bb;
  		path_coordinate_y[17] = 0.8*bb;
  		path_coordinate_y[18] = 0.9*bb;
  		path_coordinate_y[19] = 0.9*bb;
  		path_coordinate_y[20] = 1.0*bb;
      path_coordinate_y[21] = 1.0*bb;*/
	// Array of path_coordinate: Meters, Meters
     /*for(int i = 0 ; i < path_size; i++)
     {
       path.poses[i].pose.position.x = path_coordinate_x[i];
       path.poses[i].pose.position.y = path_coordinate_y[i];
     }*/


  
  
    if (end_path_record & path_counter <= path_size){
      where_to_go(cellpath.x,cellpath.y,pathmap,&cellpath);
      path.poses[path_counter].pose.position.x = (float)(cellpath.x-1)/stepsize;
      path.poses[path_counter].pose.position.y = (float)(cellpath.y-1)/stepsize;

      path_counter++;
      printf("movetogoal, x_sp = %f, y_sp = %f, end_path_record = %d, path_counter = %d\n",(float)(cellpath.x-1)/stepsize,(float)(cellpath.y-1)/stepsize,end_path_record,path_counter);
    }

 
    //printf("path_counter = %d,path.x = %f,path.y = %f\n",path_counter,path_astar.poses[path_counter].pose.position.x,path_astar.poses[path_counter].pose.position.y);
    int status = sweep_scan(pub,rate,path_coordinate_x[path_counter],path_coordinate_y[path_counter],kp_ang,ki_ang,kd_ang);
		//int status = sweep_scan(pub,rate,path_astar.poses[path_counter].pose.position.x,path_astar.poses[path_counter].pose.position.y,kp_ang,ki_ang,kd_ang);
		/*if (status == 1 & path_counter < path_size-1)
			path_counter++;
		else if (status == 1 & path_counter>=path_size-1)
		{	path_counter=0;
      end_path = 1;
    }*/
		//printf("status = %d,i = %d\n",status,i);
	}
}

void increment_sweepscan(const ros::Publisher pub,const int rate,float kp_ang,float ki_ang,float kd_ang,cell_path_info *sweeppath)
{
  if (start_pose == 1){
  	static float cellstep = 0.15f, cellmax = 2.0f, cellmin = 0.0;
	static float x_sp = pose.pose.position.x, y_sp = pose.pose.position.y;
	sweeppath->x = (int16_t)(pose.pose.position.x*stepsize+1); // 0.5 is for round
	sweeppath->y = (int16_t)(pose.pose.position.y*stepsize+1);
    ////////////////////// sweep scan in the square //////////////////////
    //cellmax is the side length of square, cellstep is the step in one sweep scan, x_sp, y_sp are the set points in x, y
    static uint8_t sweep_count = 0, xsp_count = 0;
    int status = sweep_scan(pub,rate,x_sp,y_sp,kp_ang,ki_ang,kd_ang);
	//printf("sweeppath.status = %d,x_sp = %f, y_sp = %f, sweeppath.x = %d,sweeppath.y = %d, sweeppath.pre_x = %d, pathpoint.pre_y = %d\n",sweeppath->status,x_sp,y_sp,sweeppath->x,sweeppath->y,sweeppath->pre_x,sweeppath->pre_y);
	if ((sweeppath->x != sweeppath->pre_x | sweeppath->y != sweeppath->pre_y) & cell_status == 0 & status >= 1){
		sweeppath->pre_x = sweeppath->x;
		sweeppath->pre_y = sweeppath->y;
		//where_to_go(sweeppath->x,sweeppath->y,pathmap,sweeppath);
		if (sweeppath->status == 1){
			printf("movetogoal, x_sp = %f, y_sp = %f\n",(float)(sweeppath->x-1)/stepsize,(float)(sweeppath->y-1)/stepsize);
			printf("min min min pose.pose.position.x = %f, pose.pose.position.y = %f\n",pose.pose.position.x,pose.pose.position.y);
		      	x_sp = (float)(sweeppath->x-1)/stepsize;
		      	y_sp = (float)(sweeppath->y-1)/stepsize;
			movetogoal(x_sp,y_sp,0);
			usleep(5000000);
			sweep_count = 0;
			xsp_count = 0;
			printf("finish finish finish x_sp = %f, y_sp = %f, pose.pose.position.x = %f, pose.pose.position.y = %f\n",x_sp,y_sp,pose.pose.position.x,pose.pose.position.y);
			sweeppath->status = 0;
		}
	}

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
    if (1/*path.at<int16_t>(cv::Point2d(x,y)) != 32766*/){
    pathmap.at<int16_t>(cv::Point2i(x,y)) = 32766;
	int start = 32766;
	int next_start = 32766;
	int min = 32766;
		pathpoint->status = 0;
	printf("where start in x = %d, y = %d\n",x,y);
	if (path.at<int16_t>(cv::Point2d(x,y-1))<start){
		pathpoint->x = x;
		pathpoint->y = y-1;
		start = path.at<int16_t>(cv::Point2d(x,y-1));
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
	if (path.at<int16_t>(cv::Point2d(x-1,y))<start){
		pathpoint->x = x-1;
		pathpoint->y = y;
		start = path.at<int16_t>(cv::Point2d(x-1,y));
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
      end_path_record = 0;
      printf("pathpoint->status = %d , finish...........\n\n\n\n",pathpoint->status);
  	}
	}
	printf("min = %d, nowvalue = %d\n",min,pathmap.at<int16_t>(cv::Point2i(pathpoint->x,pathpoint->y)));
	//printf("pathpoint.status = %d,pathpoint.x = %d,pathpoint.y = %d, pathpoint.next_x = %d, pathpoint.next_y = %d pathpoint.pre_x = %d, pathpoint.pre_y = %d\n",pathpoint->status,pathpoint->x,pathpoint->y,pathpoint->next_x,pathpoint->next_y,pathpoint->pre_x,pathpoint->pre_y);
    }
    else{
	pathpoint->x = x;
	pathpoint->y = y;
    }
		printf("pathmap ...........................................................\n");
		for (int i = 0 ; i < pathmap.rows ; i++){
			for (int j = 0 ; j < pathmap.cols ; j++){
				printf("%d\t",pathmap.at<int16_t>(cv::Point2i(i,j)));
			}
			printf("\n");
		}
	/*pathpoint->tan = atan2((pathpoint->y-pathpoint->pre_y) , (pathpoint->x-pathpoint->pre_x));
	if (pathpoint->status == 0)
		pathmap.at<int16_t>(cv::Point2d(pathpoint->pre_x,pathpoint->pre_y)) = 32766;
/////////////////////////////////next point//////////////////////////////////////
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
	pathpoint->next_tan = atan2((pathpoint->next_y-pathpoint->y) , (pathpoint->next_x-pathpoint->x));*/
}

int cell_coordinate(cv::Mat opencv_map, uint8_t metersize, const float laser_shift_x, const float laser_shift_y)
{
  /*cv::namedWindow("imageROI");
	cv::namedWindow("graymap");
  cv::startWindowThread();*/
  uint8_t scanrange = 6; // meter
  uint8_t blocksize = 5; // position point size
  uint16_t opencv_map_x_pre = opencv_map.rows/2;
	uint16_t opencv_map_y_pre = opencv_map.cols/2;
	float theta = 0;
	uint16_t opencv_map_x = opencv_map.cols/2+pose.pose.position.y*metersize;
	uint16_t opencv_map_y = opencv_map.rows/2+pose.pose.position.x*metersize;
	float cell_map_ratio = (float)metersize/stepsize;
	cv::Mat cellmap(cellsize,cellsize,CV_16S,cv::Scalar(255));
	cv::Mat pathROI(cellsize,cellsize,CV_16S);
	cv::Mat cellmap_resize(cellsize,cellsize,CV_16S,cv::Scalar(255));

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
	cv::Mat imageROI = opencv_map(cv::Rect(opencv_map.rows/2,opencv_map.cols/2,celllength*metersize,celllength*metersize));
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
	/*printf("cellmap ..................................................................\n");
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
	}
	//imshow("imageROI",imageROI);
	//imshow("graymap",opencv_map);*/
	return 0;
}

int sweep_scan(ros::Publisher pub , int frequency, const float path_coordinate_x, const float path_coordinate_y,float kp_ang,float ki_ang,float kd_ang)
{
	float x_speed = 0,z_speed = 0 , radius = 0.5 , diff_x , diff_y;
	static bool correct_direction = false;
	// PID constants for linear controller
	float kp_lin = 0.5;//0.6;
	float ki_lin = 0;//.0001;//0.35;
	float kd_lin = 0;//0.1;
	// PID constants for angular controller
	/*float kp_ang = 2.5;
	float ki_ang = 0;//.1;//2.5;//.1;
	float kd_ang = 0.1;//0.01;//.05;//.05;*/
	// Integrals from PID. Keeps a separate positive and negative integral to avoid large swings
	static float integral_lin = 0, integral_ang_pos = 0, integral_ang_neg = 0;
	// Prior error for Derivative
	static float prior_lin = 0, prior_ang_pos = 0, prior_ang_neg = 0, setpoint_x = 0, setpoint_y = 0, ang_diff_pos = 0, ang_diff_neg = 0, ang_diff_max = 0;
	// Maximum Roomba speed:
	// Linear: (-0.5, 0.5) Meters per second
	// Angular: (-4.25, 4.25) Meters per second
	float min_output_lin = -0.24;//0.24
	float max_output_lin = 0.24;
	float min_output_ang = -0.48;//0.48
	float max_output_ang = 0.48;
    	// Calculates relative and total differences in distance as well as their absolute values
	static bool rotate_mode = 1;
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
	float abs_diff = fabs(diff); 
	float ang_diff,abs_ang_diff,ang_desired,derivative,continue_y = 0, yaw;
  static int acc_curve_x = 0;
  static int acc_curve_z = 0, inverse_y = 0, inverse_x = 0;
    	// If the Roomba is not at it's destination, PID loops will run
    if(!(diff < 0.03))
    {
	    // Conversion from Quaternion to Yaw
  	  tf::Pose pose_tf;
      if (odom_switch == 0)
  	    tf::poseMsgToTF(odom.pose.pose, pose_tf);
      else
        tf::poseMsgToTF(pose.pose, pose_tf);
  	  yaw = tf::getYaw(pose_tf.getRotation());
      	// Calculates desired angle, difference from current angle and its absolute value 
  	  ang_desired = atan2(diff_y , diff_x);
  	  ang_diff = yaw - ang_desired;
  	  abs_ang_diff = fabs(ang_diff);
      // printf("diff < 0.03\n");
      // Angular PID: only runs if there is an angular offset
      //if (!(abs_ang_diff < 0.0005))
        // Normalizes angular difference for easier comparison
        if(ang_diff < 0) ang_diff += 2*M_PI;

        // Checks which direction Roomba should rotate depending on whichever is closer
	      if(ang_diff > M_PI && ang_diff < 2*M_PI)
	      {
	        // Normalizes angular difference for positive rotation 
	        ang_diff = 2*M_PI - ang_diff;

	        // Resets negative angular integral to avoid swings
	        integral_ang_neg = 0;

	        // Calculation and capping of the positive angular integral
	        integral_ang_pos += ang_diff * ki_ang / frequency;
	        if(integral_ang_pos > max_output_ang)
	          integral_ang_pos = max_output_ang;
	        /*else 
		        integral_ang_pos = 0;*/
	        // Calculation of the derivative
	        derivative = (ang_diff - prior_ang_pos) * frequency * kd_ang;
	        //printf("derivative=%3.3f,ang_diff=%3.3f,prior_ang_pos=%3.3f,frequency=%d,kd_ang=%3.3f\n",derivative,ang_diff,prior_ang_pos,frequency,kd_ang);
	        // Calculation and capping of angular velocity
          if (rotate_mode == 1)
          {
            integral_ang_pos = 0;
            derivative = 0;
          }
          ang_diff_pos+=ang_diff;
          z_speed = kp_ang * ang_diff + integral_ang_pos + derivative;
		      //printf("ang_diff=%3.3f,integral_ang_pos=%3.3f,derivative=%3.3f\n",ang_diff,integral_ang_pos,derivative);
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
	        integral_ang_pos = 0;
          
	        integral_ang_neg += ang_diff * ki_ang * -1 / frequency;
	        if(integral_ang_neg < min_output_ang)
	          integral_ang_neg = min_output_ang;
	        /*else
		        integral_ang_neg = 0;*/

	        derivative = (prior_ang_neg - ang_diff) * frequency * kd_ang;
          if (rotate_mode == 1)
          {
            integral_ang_neg = 0;
            derivative = 0;
          }
          ang_diff_neg+=ang_diff;
	        z_speed = (kp_ang * ang_diff + -1 * integral_ang_neg + derivative) * -1;
		      //printf("ang_diff=%3.3f,integral_ang_neg=%3.3f,derivative=%3.3f\n",ang_diff,integral_ang_neg,derivative);
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
      //if(abs_ang_diff < M_PI/4 && ((abs_diff > 0.1) || (abs_diff < 0.1 && correct_direction)))
      //if(abs_ang_diff < asin((diff/2) / radius))
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
          sendCmdVel(pub, 0, 0);
          usleep(1000000);
          integral_ang_pos = 0;
          integral_ang_neg = 0;
          rotate_mode = 0;
          acc_curve_x = 0;
          acc_curve_z = 0;
        }
        if (z_speed < 0.1 & z_speed > 0)
          z_speed = 0.1;
        if (z_speed > -0.1 & z_speed < 0)
          z_speed = -0.1;
       /* float curve_vel_shift = 0;
        if (z_speed>0)
          curve_vel_shift = 0.01;
        else
          curve_vel_shift = 0.002;
          if (setpoint_x == path_coordinate_x){
            if ((setpoint_x-odom.pose.pose.position.x)<0 & inverse_x==0)
              inverse_x = -1;
            else if ((setpoint_x-odom.pose.pose.position.x)>0 & inverse_x==0)
              inverse_x = 1;
            if (path_coordinate_x-odom.pose.pose.position.x>=0)
              x_speed = inverse_x*(curve_vel_shift);
            else
              x_speed = inverse_x*(-1)*curve_vel_shift;
          }
          if (setpoint_y == path_coordinate_y){
            if ((setpoint_y-odom.pose.pose.position.y)<0 & inverse_y==0)
              inverse_y = -1;
            else if ((setpoint_y-odom.pose.pose.position.y)>0 & inverse_y==0)
              inverse_y = 1;
            if (path_coordinate_y-odom.pose.pose.position.y>=0)
              x_speed = inverse_y*(curve_vel_shift);
            else
              x_speed = inverse_y*(-1)*curve_vel_shift;
          }
          if (x_speed < 0)*/
            if (stop_bumper == 1){
              sendCmdVel(pub, -0.2, 0);
              if (odom_switch == 0)
                printf("bumper warning in odom(x = %f , y = %f).................\n",odom.pose.pose.position.x,odom.pose.pose.position.y);
              else
                printf("bumper warning in pose(x = %f , y = %f).................\n",pose.pose.position.x,pose.pose.position.y);
            }
            x_speed = 0;
        //x_speed=0;
      }
      //if((abs_ang_diff < M_PI/4 && ((abs_diff > 0.1) || (abs_diff < 0.1))) || (abs_ang_diff > M_PI*7/4 && ((abs_diff > 0.1) || (abs_diff < 0.1))))
      if (rotate_mode == 0)
      {
        if (stop_sonar == 1){
          rotate_mode = 1;
          sendCmdVel(pub, 0, 0);
          //usleep(200000);
          if (odom_switch == 0)
            printf("sonar warning in odom(x = %f , y = %f), go to next setpoint\n",odom.pose.pose.position.x,odom.pose.pose.position.y);
          else
            printf("sonar warning in pose(x = %f , y = %f), go to next setpoint\n",pose.pose.position.x,pose.pose.position.y);
          return 2 ;
        }
        if (stop_bumper == 1){
          rotate_mode = 1;
          sendCmdVel(pub, -0.2, 0);
          //usleep(400000);
          sendCmdVel(pub, 0, 0);
          if (odom_switch == 0)
            printf("bumper warning in odom(x = %f , y = %f), go to next setpoint\n",odom.pose.pose.position.x,odom.pose.pose.position.y);
          else
            printf("bumper warning in pose(x = %f , y = %f), go to next setpoint\n",pose.pose.position.x,pose.pose.position.y);
          return 2 ;
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
      //integral_lin = 0;
      sendCmdVel(pub, 0, 0);
      usleep(500000);
      rotate_mode = 1;
   	  integral_ang_pos = 0;
   	  integral_ang_neg = 0;
   	  prior_lin = 0;
   	  prior_ang_pos = 0;
   	  prior_ang_neg = 0;
      setpoint_x = path_coordinate_x;
      setpoint_y = path_coordinate_y;
      inverse_x = 0;
      inverse_y = 0;
      printf("arrival (x = %f , y = %f).......\n",pose.pose.position.x,pose.pose.position.y);
      return 1 ;
    }
    // Publishes message, increases ROS counter, then sleeps until next iteration
    //printf("float kp_ang = %f,float ki_ang = %f,float kd_ang = %f\n",kp_ang, ki_ang, kd_ang);
    /*if (odom_switch == 0)
      printf("rota=%d,x_sp=%3.3f,y_sp=%3.3f,x=%3.3f,y=%3.3f,x_vel=%3.3f,z_vel=%3.3f,int_lin=%3.3f,ang_dif=%3.3f,yaw=%3.3f,ang_desi=%3.3f,int_ang_pos=%3.3f,int_ang_neg=%3.3f,deriv=%3.3f\n",rotate_mode,path_coordinate_x,path_coordinate_y,odom.pose.pose.position.x,odom.pose.pose.position.y,x_speed,z_speed,integral_lin,ang_diff,yaw,ang_desired,integral_ang_pos,integral_ang_neg,derivative);
    else
      printf("rota=%d,x_sp=%3.3f,y_sp=%3.3f,x=%3.3f,y=%3.3f,diff=%3.3f,x_vel=%3.3f,z_vel=%3.3f,int_lin=%3.3f,ang_dif=%3.3f,yaw=%3.3f,ang_desi=%3.3f,int_ang_pos=%3.3f,int_ang_neg=%3.3f,deriv=%3.3f\n",rotate_mode,path_coordinate_x,path_coordinate_y,pose.pose.position.x,pose.pose.position.y,diff,x_speed,z_speed,integral_lin,ang_diff,yaw,ang_desired,integral_ang_pos,integral_ang_neg,derivative);
    //printf("rota=%d,x_sp=%3.3f,y_sp=%3.3f,x=%3.3f,y=%3.3f,x_speed=%3.3f,z_speed=%3.3f,ang_diff=%3.3f,integral_ang_pos=%3.3f,integral_ang_neg=%3.3f,derivative=%3.3f,ang_diff_pos=%3.3f,ang_diff_neg=%3.3f,ang_diff_max=%3.3f\n",rotate_mode,path_coordinate_x,path_coordinate_y,odom.pose.pose.position.x,odom.pose.pose.position.y,x_speed,z_speed,ang_diff,integral_ang_pos,integral_ang_neg,derivative,ang_diff_pos,ang_diff_neg,ang_diff_max);
 	  sendCmdVel(pub, x_speed, z_speed);*/
    return 0 ;
}

void showMapInfo(cv::Mat opencv_map, uint8_t metersize, const float laser_shift_x, const float laser_shift_y)
{
  cv::namedWindow("view");
  cv::startWindowThread();
  static cv::Mat DatabufferArea(opencv_map.rows, opencv_map.cols, CV_8UC3, CV_RGB(0,0,0)); // Save data in a cv::Mat
  static cv::Mat DatabufferPath(opencv_map.rows, opencv_map.cols, CV_8UC3, CV_RGB(0,0,0)); // Save data in a cv::Mat
  uint8_t scanrange = 6; // meter
  uint8_t blocksize = 5; // position point size
  static uint16_t opencv_map_x_pre = opencv_map.rows/2;
  static uint16_t opencv_map_y_pre = opencv_map.cols/2;
  static float theta = 0;
  uint16_t opencv_map_x = opencv_map.cols/2+pose.pose.position.y*metersize;
  uint16_t opencv_map_y = opencv_map.rows/2+pose.pose.position.x*metersize;

	if (opencv_map_x >= opencv_map.rows-blocksize/2) // limit x by the image size
		opencv_map_x = opencv_map.rows-1-blocksize/2;
	else if (opencv_map_x < blocksize/2)
		opencv_map_x = blocksize/2;
	if (opencv_map_y >= opencv_map.cols-blocksize/2) // limit y by the image size
		opencv_map_y = opencv_map.cols-1-blocksize/2;
	else if (opencv_map_y < blocksize/2)
		opencv_map_y = blocksize/2;
/// draw scanned area and path , Saving and updating the area and path in data buffer
	if (opencv_map_x_pre!=opencv_map_x | opencv_map_y_pre!=opencv_map_y)
	{
		cv::line(DatabufferArea, cv::Point(opencv_map_x_pre,opencv_map_y_pre), cv::Point(opencv_map_x,opencv_map_y), cv::Scalar(50,50,50), blocksize);
		cv::line(DatabufferPath, cv::Point(opencv_map_x_pre,opencv_map_y_pre), cv::Point(opencv_map_x,opencv_map_y), cv::Scalar(0,0,255), 1);
	}

	opencv_map = opencv_map + DatabufferArea;//Add area buffer in origin map

	for (uint16_t j = metersize ; j < opencv_map.rows ; j+=metersize) // draw the grid line in 1 meter per grid
	{
		cv::line(opencv_map, cv::Point(0,j), cv::Point(opencv_map.rows,j), cv::Scalar(50,50,50), 1);
		cv::line(opencv_map, cv::Point(j,0), cv::Point(j,opencv_map.cols), cv::Scalar(50,50,50), 1);
	}

	opencv_map = opencv_map + DatabufferPath;//Add path buffer in origin map
/// draw home in origin map*/
	for (uint16_t i = 0 ; i < blocksize ; i++)
	{
		opencv_map.at<cv::Vec3b>(cv::Point2d(opencv_map.rows/2-1,opencv_map.cols/2-2+i)) = cv::Vec3d(0,255,0);
		opencv_map.at<cv::Vec3b>(cv::Point2d(opencv_map.rows/2+1,opencv_map.cols/2-2+i)) = cv::Vec3d(0,255,0);
	}
		opencv_map.at<cv::Vec3b>(cv::Point2d(opencv_map.rows/2,opencv_map.cols/2)) = cv::Vec3d(0,255,0);
	opencv_map_x_pre = opencv_map_x;
	opencv_map_y_pre = opencv_map_y;

/// draw arrow for Lidar X direction 
	if (pose.pose.orientation.z>=0)
		theta = 2*acos(pose.pose.orientation.w);
	else
		theta = 2*3.142-(2*acos(pose.pose.orientation.w));
	printf("theta = %f\n", theta*180/3.142);
  drawArrow(opencv_map, cv::Point(opencv_map_x,opencv_map_y), cv::Point(opencv_map_x+18*cos(theta-3.142/2),opencv_map_y-18*sin(theta-3.142/2)),8,40,cv::Scalar(0, 255, 0),1, CV_AA);
/// draw laser data in map 
if (!laser.ranges.empty() & 0)
{
	for (uint16_t i = 0 ; i < 360 ; i++)
	{
	uint16_t lasertheta = i+(uint16_t)(theta*180/3.142);
	cv::Point2d laserpoint;
	if (lasertheta>=360)
	lasertheta = lasertheta - 360;
	if (laser.ranges[i]>=0 & laser.ranges[i]<scanrange)
		if (lasertheta>=0 & lasertheta<90)
		laserpoint = cv::Point2d(opencv_map_x-laser.ranges[i]*metersize*sin(lasertheta*3.142/180)-sin(theta)*laser_shift_x*metersize,opencv_map_y-laser.ranges[i]*metersize*cos(lasertheta*3.142/180)-cos(theta)*laser_shift_y*metersize);
		else if (lasertheta>=90 & lasertheta<180)
		laserpoint = cv::Point2d(opencv_map_x-laser.ranges[i]*metersize*cos((lasertheta-90)*3.142/180)-sin(theta)*laser_shift_x*metersize,opencv_map_y+laser.ranges[i]*metersize*sin((lasertheta-90)*3.142/180)-cos(theta)*laser_shift_y*metersize);
		else if (lasertheta>=180 & lasertheta<270)
		laserpoint = cv::Point2d(opencv_map_x+laser.ranges[i]*metersize*sin((lasertheta-180)*3.142/180)-sin(theta)*laser_shift_x*metersize,opencv_map_y+laser.ranges[i]*metersize*cos((lasertheta-180)*3.142/180)-cos(theta)*laser_shift_y*metersize);
		else
		laserpoint = cv::Point2d(opencv_map_x+laser.ranges[i]*metersize*cos((lasertheta-270)*3.142/180)-sin(theta)*laser_shift_x*metersize,opencv_map_y-laser.ranges[i]*metersize*sin((lasertheta-270)*3.142/180)-cos(theta)*laser_shift_y*metersize);

		if (laserpoint.x>=0 & laserpoint.x<opencv_map.rows & laserpoint.y>=0 & laserpoint.y<opencv_map.cols)
		opencv_map.at<cv::Vec3b>(laserpoint) = cv::Vec3d(0,0,255);
	}
}
	/// draw circle for scan range in 6 meter for radius
  	cv::circle(opencv_map,cv::Point(opencv_map_x,opencv_map_y),scanrange*metersize,cv::Scalar(0,165,255));
  	//cvSetMouseCallback("view",onMouse,NULL);
  	imshow("view",opencv_map);
}

void drawArrow(cv::Mat& img, cv::Point pStart, cv::Point pEnd, int len, int alpha,cv::Scalar color, int thickness, int lineType)
 {
     const double PI = 3.1415926;
     cv::Point arrow;
     double angle = atan2((double)(pStart.y - pEnd.y), (double)(pStart.x - pEnd.x));
     cv::line(img, pStart, pEnd, color, thickness, lineType);

     arrow.x = pEnd.x + len * cos(angle + PI * alpha / 180);
     arrow.y = pEnd.y + len * sin(angle + PI * alpha / 180);
     cv::line(img, pEnd, arrow, color, thickness, lineType);
     arrow.x = pEnd.x + len * cos(angle - PI * alpha / 180);
     arrow.y = pEnd.y + len * sin(angle - PI * alpha / 180);
     cv::line(img, pEnd, arrow, color, thickness, lineType);
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

void occupancyGridToCvMat(const nav_msgs::OccupancyGrid *map, cv::Mat *im)
{
	uint8_t *data = (uint8_t*) map->data.data(), testpoint = data[0];
   	bool mapHasPoints = false;

   	// transform the map in the same way the map_saver component does
   	for (size_t i=0; i<map->data.size(); i++)
   	{
     		if (data[i] == 0)
			im->at<cv::Vec3b>(cv::Point2i(i/map->info.height,i%map->info.width)) = cv::Vec3d(120,120,120);//Vec3d sequence == BGR, Scanned area
     		else if (data[i] == 100)
			im->at<cv::Vec3b>(cv::Point2i(i/map->info.height,i%map->info.width)) = cv::Vec3d(0,0,0);// Scanned edge
     		else 
			im->at<cv::Vec3b>(cv::Point2i(i/map->info.height,i%map->info.width)) = cv::Vec3d(30,30,30);// not Scanned area

    	// just check if there is actually something in the map
    		if (i!=0) mapHasPoints = mapHasPoints || (data[i] != testpoint);
    		testpoint = data[i];
  	}

  	// sanity check
  	if (!mapHasPoints) { ROS_WARN("map is empty, ignoring update."); }
}
void movetogoal(double x, double y, double theta) {
  
	MoveBaseClient ac("move_base", true);

	// Wait 0.5 seconds for the action server to become available
	ROS_INFO("Waiting for the move_base action server");
	ac.waitForServer(ros::Duration(1));

	ROS_INFO("Connected to move base server");

	// Send a goal to move_base
	move_base_msgs::MoveBaseGoal goal;
	goal.target_pose.header.frame_id = "map";
	goal.target_pose.header.stamp = ros::Time::now();

	goal.target_pose.pose.position.x = x;
	goal.target_pose.pose.position.y = y;

	// Convert the Euler angle to quaternion
	double radians = theta * (M_PI/180);
	tf::Quaternion quaternion;
	quaternion = tf::createQuaternionFromYaw(radians);

	geometry_msgs::Quaternion qMsg;
	tf::quaternionTFToMsg(quaternion, qMsg);

	goal.target_pose.pose.orientation = qMsg;

	ROS_INFO("Sending goal to: x = %f, y = %f, theta = %f", x, y, theta);
	ac.sendGoal(goal);

	// Wait for the action to return
	ac.waitForResult();

	if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
		ROS_INFO("You have reached the goal!");
	else
		ROS_INFO("The base failed for some reason");

                ROS_INFO("Finish");

}

