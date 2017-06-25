#include "pureStanley2.h"

//PLUGINLIB_EXPORT_CLASS(local_planner::StanleyPlanner, nav_core::BaseLocalPlanner)

using namespace std;
using namespace ros;
using namespace geometry_msgs;
using namespace std_msgs;
using namespace nav_msgs;
using namespace tf2;

#define PI 3.14159265
#define E 2.718281828


namespace local_planner {



void StanleyPlanner::pose_callback(const Odometry msg) {
    //saving times between two callbacks
	proslo = pocetno;
	pocetno = ros::Time::now().toSec();
	if (!condition || (pocetno - proslo) > 1) {
		return;
	}

    //Transfor from robot coordinate frame to global frame
	state = msg;
	tf2_ros::Buffer tfBuffer;
	tf2_ros::TransformListener tfListener(tfBuffer);
	while (true) {
		
		try {
		    


        transformStamped = tfBuffer.lookupTransform("world", "robot0",
					ros::Time(0));
			break;
		} catch (tf2::TransformException &ex) {

			continue;
			

		}
	}
	
	//Find heading angle of robot
		 tf2::Quaternion quaternion(state.pose.pose.orientation.x,
					state.pose.pose.orientation.y,
					state.pose.pose.orientation.z,
					state.pose.pose.orientation.w);
			
			double roll, pitch;

			tf2::Matrix3x3 m(quaternion);
			m.getRPY(roll, pitch, yaw);
			
			
	//getting from x and y coordinate of robot in global frame
	double robot_X =  transformStamped.transform.translation.x;
	double robot_Y = transformStamped.transform.translation.y;
	
	//Find index of closest point on path from robot
	double modulDistanceForI = 0;
	for(int h = i+1; h<path.poses.size(); h++){
	    
	    pastModul = sqrt(pow(path.poses[h-1].pose.position.y-robot_Y, 2)+pow(path.poses[h-1].pose.position.x-robot_X, 2));
	    modulDistanceForI = sqrt(pow(path.poses[h].pose.position.y-robot_Y, 2)+pow(path.poses[h].pose.position.x-robot_X, 2));
	    if(pastModul-modulDistanceForI<0){
	        i = h-1;
	        break;
	    }
	}
	//secure from index out of bounds 
	if(pathSize < i+step){
	    step = pathSize-i;
	    if(step < 0){
	        step =0;
	    }
	}
	//finding cross product of vector1 which is gola point and and second point on path
	//for looking curvature of path and vector2 which is goal point and robot point
	double a11 = path.poses[i + step].pose.position.x - path.poses[i].pose.position.x;
	double a12 = path.poses[i + step].pose.position.y - path.poses[i].pose.position.y;
	double a21 = robot_X - path.poses[i].pose.position.x;
	double a22 = robot_Y - path.poses[i].pose.position.y;
	double Xprod = a11*a22-a12*a21;
	

//########################Zanemariti
	double x1 = -robot_X + path.poses[i].pose.position.x;
	double y1 = -robot_Y + path.poses[i].pose.position.y;
	
	double y2_1=0, x2_1 =0,y2_2=0, x2_2 =0;
	if(x1 == 0){
		x2_1 =1;
		x2_2 =-1;
		y2_1 = 0;
		y2_2 = 0;
	}else{
		y2_1 =1;
		y2_2 =-1;
		x2_1 = -(y1/x1)*y2_1;
		x2_2 = -(y1/x1)*y2_2;

	}
	
	double x2 = path.poses[i + 1].pose.position.x - path.poses[i].pose.position.x;
	double y2 = path.poses[i + 1].pose.position.y - path.poses[i].pose.position.y;
	double kutjedan = atan2(y2,x2);
	double cos1 = (x2_1*x2+y2_1*y2)/abs( (sqrt(pow(x2_1, 2)+pow(y2_1, 2))) * (sqrt(pow(x2, 2)+pow(y2, 2)) ) );
	double cos2 = (x2_2*x2+y2_2*y2)/abs((sqrt(pow(x2_2, 2)+pow(y2_2, 2))) * (sqrt(pow(x2, 2)+pow(y2, 2))));
	double pathAngleCross = 0;
	if(cos1>=0){
		pathAngleCross = atan2(y2_1,x2_1);
	}else{
		pathAngleCross = atan2(y2_2,x2_2);
	}
	
	double cosOnPath = (x1*x2+y1*y2)/abs( (sqrt(pow(x1, 2)+pow(y1, 2))) * (sqrt(pow(x2, 2)+pow(y2, 2)) ) );
	if(abs(cosOnPath-1)<0.004){
		pathAngleCross=0;
	}
//####################Kraj

//###############Kut proba
	KDL::Frame F_bl_end = transformToBaseLink(path.poses[i].pose, transformStamped.transform);

      
      	double rollPath, pitchPath, yawPath;
      	F_bl_end.M.GetRPY(rollPath, pitchPath, yawPath);
      	
//####################Kraj

	//Getting sign which defines if robot if robot is on the right side in regard to path or left side
	//if robot is on the rght side of path than sign is positive because positive anguar velocity is counter clockwise
	//and robot desire direction will intersect with path and otherwise.
	int sign;
	if(Xprod >0){
	    sign =-1;
	}else if(Xprod <0){
	    sign=1;
	}else{
	    sign =0;
	}
	double angleRobot = yaw;
    //Finding curvature of the path
	double anglePath = atan2(
			path.poses[i + step].pose.position.y - path.poses[i].pose.position.y,
			path.poses[i + step].pose.position.x - path.poses[i].pose.position.x);
	double kutosam = atan2(
			path.poses[i + 2*step].pose.position.y - path.poses[i].pose.position.y,
			path.poses[i + 2*step].pose.position.x - path.poses[i].pose.position.x);
			if(anglePath > PI){
			    anglePath -=(2*PI);
			}else if(anglePath < -PI){
			    anglePath +=(2*PI);
			}
			
			if(angleRobot > PI){
			    angleRobot -=(2*PI);
			}else if(angleRobot < -PI){
			    angleRobot +=(2*PI);
			}
    //Finding diference between robots angle and paths angle
	double delta = -angleRobot + anglePath;
       // ROS_INFO("%f", anglePath*(180/3.14159)); 
       //ROS_INFO("%f | %f | %f", pathAngleCross, anglePath, yawPath);   
    //Calcualte closest distance from robot to goal point
	double modulDistance = sqrt(pow(path.poses[i].pose.position.y-robot_Y, 2)+pow(path.poses[i].pose.position.x-robot_X, 2));
         
			//stanley formula
			double angle = k2*(delta + sign*atan2((k * modulDistance)/1.0,1));
            
			double omega = angle/((pocetno - proslo)) ;
            
            
            //reducing linear velocity of robot exponentialy depending on angle difference between robots and paths angle
            //to reduce error
            double kvel = pow(E,-7*abs(delta));
			velocity = 0.2*kvel;//(modulDistance / (pocetno - proslo))*kvel;
			
			//if 
			if(abs(delta)>=PI){
			    velocity = 0;
			}
           
            
            //saturation of angular velocity
            if(omega < -3){
                omega = -3;
            }else if(omega > 3){
                omega = 3;
            }
            
            //setting velocities
			cmd_vel.angular.z =omega;
			cmd_vel.linear.x = 1.0;//velocity;//0.1;//
            
            //uploding data for analyse data
            std_msgs::Float64 msgDelta, msgOmega, msgModulDistance, msgRobotX, msgRobotY, kutjedanD, kutosamD, kutpetD, kutdamjanD, kutja2D;
            msgDelta.data = delta;
            msgOmega.data = omega;
            msgModulDistance.data =modulDistance;
            msgRobotX.data = robot_X ;
            msgRobotY.data = robot_Y;
	    kutjedanD.data=kutjedan;
	    kutosamD.data=kutosam;
	    kutpetD.data =anglePath ;
	    kutdamjanD.data=yawPath;
	    kutja2D.data =-1*pathAngleCross ;
            std_msgs::Float64 msgPathX, msgPathY;

		    msgPathX.data = path.poses[i].pose.position.x;
		    msgPathY.data = path.poses[i].pose.position.y;
		    pathXPub.publish(msgPathX);
		    pathYPub.publish(msgPathY);
		    
			speedPub.publish(cmd_vel);
			errorPub.publish(msgDelta);
		    omegaPub.publish(msgOmega);
		    modulPub.publish(msgModulDistance);
		    robotXPub.publish(msgRobotX);
		    robotYPub.publish(msgRobotY);
		
		kutjedanPub.publish(kutjedanD);
		kutpetPub.publish(kutpetD);
		kutosamPub.publish(kutosamD);
		kutDamjanPub.publish(kutdamjanD);
		kutja2.publish(kutja2D);
			
            

           
			spinOnce();
			



	}

	void StanleyPlanner::path_callback(const nav_msgs::Path msg) {
        //reset path index
	    i =0;
		path = msg;
		//flag telling that we have receive
		condition = true;


		
		tf2_ros::Buffer tfBuffer;
	tf2_ros::TransformListener tfListener(tfBuffer);
	while (true) {
		
		try {
		    
//			transformStamped = tfBuffer.lookupTransform("odom", "base_footprint",
//					ros::Time(0));
        transformStamped = tfBuffer.lookupTransform("world", "robot0",
					ros::Time(0));
			break;
		} catch (tf2::TransformException &ex) {
		    //ROS_INFO("Pao sam");
			continue;
			

		}
	}
		double robot_X =  transformStamped.transform.translation.x;
	double robot_Y = transformStamped.transform.translation.y;
	    int nextStep=0;
	    int indexNext =0;
	    
	    //secure from  wrong path receive
	    pathSize =path.poses.size();
	    if(pathSize==0){
	        condition = false;
	    }
	    
	    //calcualte index which tells what is the difference of indecies between to points which abs distance is approximately 5cm
		for(int r = 0; r<pathSize; r++){
		
	double modulDistance = sqrt(pow(path.poses[r].pose.position.y-path.poses[indexNext].pose.position.y, 2)+pow(path.poses[r].pose.position.x-path.poses[indexNext].pose.position.x, 2));
	    nextStep++;
	if(modulDistance >= 0.03){
	    step = (step+nextStep)/2;
	    nextStep=0;
	    indexNext = r;
	}
		}
	}

	KDL::Frame StanleyPlanner::transformToBaseLink(const geometry_msgs::Pose& pose,
                                            const geometry_msgs::Transform& tf)
{
  // Pose in global (map) frame
  KDL::Frame F_map_pose(KDL::Rotation::Quaternion(pose.orientation.x,
                                                  pose.orientation.y,
                                                  pose.orientation.z,
                                                  pose.orientation.w),
                        KDL::Vector(pose.position.x,
                                    pose.position.y,
                                    pose.position.z));

  // Robot (base_link) in global (map) frame
  KDL::Frame F_map_tf(KDL::Rotation::Quaternion(tf.rotation.x,
                                                tf.rotation.y,
                                                tf.rotation.z,
                                                tf.rotation.w),
                      KDL::Vector(tf.translation.x,
                                  tf.translation.y,
                                  tf.translation.z));

  // TODO: See how the above conversions can be done more elegantly
  // using tf2_kdl and tf2_geometry_msgs

  return F_map_tf.Inverse()*F_map_pose;
}
	
	StanleyPlanner::StanleyPlanner() {


        pathSub = n.subscribe("/move_base/NavfnROS/plan", 1, &StanleyPlanner::path_callback, this);
		
		odometrySub = n.subscribe("/odom", 1,
				&StanleyPlanner::pose_callback, this);

		speedPub = n.advertise < Twist > ("/cmd_vel", 1);
		
		errorPub=n.advertise < Float64 > ("/error", 1);
		omegaPub=n.advertise < Float64 > ("/omega", 1);
		modulPub=n.advertise < Float64 > ("/modul", 1);
		pathXPub=n.advertise < Float64 > ("/pathX", 1);
		pathYPub=n.advertise < Float64 > ("/pathY", 1);
		robotXPub=n.advertise < Float64 > ("/robotX", 1);
		robotYPub=n.advertise < Float64 > ("/robotY", 1);
		kutjedanPub = n.advertise < Float64 > ("/kutjedan", 1);
		kutpetPub = n.advertise < Float64 > ("/kutpet", 1);
		kutosamPub =n.advertise < Float64 > ("/kutosam", 1);
		kutDamjanPub = n.advertise < Float64 > ("/kutdamjan", 1);
		kutja2 = n.advertise < Float64 > ("/kutja", 1);
		k = 0.15;
		v = 0.4;
		k2 = 0.6;
		yaw = 0;
		ks = 10;
		condition = false;
		childframe = "";
		parentframe = "";
		i = 0;
	    j = 0;
	    step = 0;
	    pastModul = 0;
	    pathSize = 0;
	    

	}}
;

	int main(int argc, char **argv)

	{
		init(argc, argv, "Stanley");
		local_planner::StanleyPlanner obj;
		//for loding simulation
		ros::Duration(5.).sleep();

		while(true){
		    spinOnce();
		}

		return 0;

	}

