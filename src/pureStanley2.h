#ifndef STANLEY_PLANNER_H_
#define STANLEY_PLANNER_H_


#include <ros/ros.h>
#include <nav_core/base_local_planner.h>
#include <geometry_msgs/PoseStamped.h>
#include <angles/angles.h> 
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Header.h"
#include "std_msgs/Empty.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/Transform.h"
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <math.h>
#include <vector>
#include <string.h>
#include <float.h>
#include <algorithm>  
#include "geometry_msgs/Quaternion.h"  
#include <tf2_ros/transform_listener.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include <cmath>

#include <geometry_msgs/Point.h>
#
#include <geometry_msgs/PolygonStamped.h>


#include <kdl/frames.hpp>

/**
 * Class implements Stanely algorithm for path tracking.
 * Stanely algorithm tracking th e path with constant linear velocity.
 * Maximum angular velocity is limited to -3 rad/s to 3 rad/s.
 *  
 * author: Branko Radoš
 */
namespace local_planner {
class StanleyPlanner{

public:
    /**
      * @brief  Constructor for the planner
      */
	StanleyPlanner();
	/**
       * @brief  Callback method which updates robots current position and computes 
       * velocities(angular/ linear) for Stanley algorithm.
       * @param data is robots odometry position 
       */
	void pose_callback(const nav_msgs::Odometry data);
	/**
       * @brief  Callback method which updates desire path for tracking.
       * @param data array of points which define desire robot path.
       */
	void path_callback(const nav_msgs::Path data);
	KDL::Frame transformToBaseLink(const geometry_msgs::Pose& pose,
                                 const geometry_msgs::Transform& tf);

private:
    /**
       * @brief  private refference to node handler for instantiate new publishers and subscribers 
       */
	ros::NodeHandle n;
	/**
       * @brief private instance of TransformStamped for getting robot to orld transform vector
       */
    geometry_msgs::TransformStamped transformStamped;
    /**
       * @brief  private subscribers for path and odometry topics
       */
	ros::Subscriber pathSub, odometrySub;
	/**
       * @brief  private publishers publish ccomputed velocities as a result of stanley algorithm, and other publishers 
       * like error, angular velocity, x and y component od path point and robots x and y odometry values for
       * plotting data.
       */
	ros::Publisher speedPub, errorPub,omegaPub,modulPub,  pathXPub, pathYPub, robotXPub, robotYPub,kutjedanPub,
		kutpetPub ,
		kutosamPub,
		kutDamjanPub,
		kutja2;

	
	/**
       * @brief  private array which cointains points of desire path
       */
	nav_msgs::Path path;
	/**
       * @brief private Twist message which cointains computed velocities(angular/ linear) for robot
       */
	geometry_msgs::Twist cmd_vel;
	/**
       * @brief  k is angle gain, k2 is main gain, ,velocity constant linear vleocity,
       * pauza is needed for setting up simulator, past Modul need for computing closest point from robot.
       */
	double k, v, k2, yaw, ks,  proslo, pocetno, velocity, pauza, pastModul;
	/**
       * @brief  private integers i and j needed for indexing current point for computing,
       * step increment index for computing curvature of pat 
       */
	int i, j, step, pathSize;
	/**
	 *@brief private vector wich cointains information of robots coordinate frame
	 */
	nav_msgs::Odometry  state;
	/**
	 *@brief private Time which contiains times of current and the last callback of function pose_callback
	 */
	ros::Time now, previous;
	/**
	 * @brief private boolean which tells i we have a desire path(true) or not
	 */
	bool condition;
	/**
	 * @brief string which defines coordinate frames needed for successful transfor and computing
	 */
	std::string childframe, parentframe;
	



};

};
#endif
