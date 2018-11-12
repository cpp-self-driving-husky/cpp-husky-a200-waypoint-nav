/*

	simulation latitude longitude

	top left
		49.8999160171 8.90012732623

	top right
		49.8999237892 8.89987663986

	bottom left
		49.9000753632 8.90012531813

	bottom right
		49.9000770425 8.89988743462
	
	49.8999237892 8.89987663986 49.8999160171 8.90012732623 49.9000753632 8.90012531813 49.9000770425 8.89988743462

*/



#include <ros/ros.h>
#include <ros/package.h>
#include <fstream>
#include <utility>
#include <vector>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <robot_localization/navsat_conversions.h>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/Bool.h>
#include <tf/transform_listener.h>
#include <math.h>
//#include <string>
//#include <iostream>

typedef actionlib::SimpleActionClient <move_base_msgs::MoveBaseAction> MoveBaseClient; 


//struct PointXY {
//	double x;
//	double y;
//};



class PathGenerator {


	public:
		PathGenerator() {
			
		}
		
		~PathGenerator() {
			
		}
				
		geometry_msgs::PointStamped latLongtoUTM(double lati, double longi) {
			geometry_msgs::PointStamped utm_point;
			std::string utm_zone;
			RobotLocalization::NavsatConversions::LLtoUTM(
				lati, longi, 
				utm_point.point.y, utm_point.point.x, utm_zone);
			utm_point.header.frame_id = "utm";
			utm_point.header.stamp = ros::Time(0);
			utm_point.point.z = 0.0;
			return utm_point;
		}


		geometry_msgs::PointStamped transformPoint(
			const geometry_msgs::PointStamped& source_point,
			const std::string& target_frame,
			const std::string& source_frame)
		{
			geometry_msgs::PointStamped target_point;
			try {
				tf::TransformListener trans;
				trans.waitForTransform(target_frame, source_frame,
					ros::Time::now(), ros::Duration(3.0));
				trans.transformPoint(target_frame, source_point, target_point);
			} catch(tf::TransformException& ex) {
				ROS_WARN("%s", ex.what());
			}
			return target_point;
		}


		geometry_msgs::PointStamped utmToOdomPoint(const geometry_msgs::PointStamped& utm_point) {
			return transformPoint(utm_point, "odom", "utm");
		}


		std::vector<geometry_msgs::PointStamped> setWaypoints(int argc, char* argv[]) {
			std::vector<geometry_msgs::PointStamped> waypoints;
			for (int i = 1; i < argc; i += 2) {
				double lati = std::atof(argv[i]),
					longi = std::atof(argv[i+1]);
				geometry_msgs::PointStamped utm_point = 
					this->latLongtoUTM(lati, longi);
				//geometry_msgs::PointStamped map_point = 
				//	this->utmToOdomPoint(utm_point);
				waypoints.push_back(utm_point);
			}
			return waypoints;
		}
		
		
		move_base_msgs::MoveBaseGoal generateGoal(
			const std::string& frame_id, 
			const geometry_msgs::PointStamped& point)
		{
			move_base_msgs::MoveBaseGoal goal;
			goal.target_pose.header.frame_id = frame_id;
			goal.target_pose.header.stamp = ros::Time::now();
			goal.target_pose.pose.position.x = point.point.x;
			goal.target_pose.pose.position.y = point.point.y;
			goal.target_pose.pose.orientation.w = 1;
			
			if (this->headingToGoal())
				return goal;
			
			geometry_msgs::PointStamped nPoint = this->utmToOdomPoint(*(this->waypoint_+1));
			// TODO replace with first pose with sensor data?
			//PointXY iPoint = this->beganPath() ? 
			//	this->pose_ : *(this->waypoint_-1);
			
			geometry_msgs::PointStamped iPoint;
			iPoint.point.x = 0.0;
			iPoint.point.y = 0.0;
			if (!this->beganPath()) {
				iPoint =  this->utmToOdomPoint(*this->waypoint_);
			}
			double yaw = std::atan2(
				nPoint.point.y-iPoint.point.y, 
				nPoint.point.x-iPoint.point.x);
			double roll = 0.0, pitch = 0.0;
			
			tf::Matrix3x3 rot_euler;
			tf::Quaternion rot_quat;
			rot_euler.setEulerYPR(yaw, pitch, roll);
			rot_euler.getRotation(rot_quat);
			
			goal.target_pose.pose.orientation.x = rot_quat.getX();
			goal.target_pose.pose.orientation.y = rot_quat.getY();
			goal.target_pose.pose.orientation.z = rot_quat.getZ();
			goal.target_pose.pose.orientation.w = rot_quat.getW();
			
			return goal;
		}
		
		
		bool goalNotReached() {
			return this->waypoint_ != this->path_.end();
		}
		
		
		bool headingToGoal() {
			return this->waypoint_ == this->path_.end()-1;
		}
		
		
		bool beganPath() {
			return this->waypoint_ == this->path_.begin();
		}
		
		
		int run(int argc, char* argv[]) {
			
			ros::init(argc, argv, "global_simple");
			this->path_ = this->setWaypoints(argc, argv);
			this->waypoint_ = this->path_.begin();
			
			MoveBaseClient ac("move_base", true);
			while(!ac.waitForServer(ros::Duration(5.0))) {
				ROS_INFO("move_base action server did not come up");
				ros::shutdown();
			}
			
			// std::cout << "running node" << std::endl;
			ros::Rate r(100);
			while (ros::ok() && this->goalNotReached()) {
				
				ros::spinOnce();
				
				// geometry_msgs::PointStamped point = *this->waypoint_;
				
				geometry_msgs::PointStamped map_point = 
					this->utmToOdomPoint(*this->waypoint_);
				move_base_msgs::MoveBaseGoal goal = 
					this->generateGoal("odom", map_point);
				
				ac.sendGoal(goal);
				ac.waitForResult();
				ROS_INFO("Sending goal...");
				if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
					int goal_num = static_cast<int>(this->waypoint_ - this->path_.begin())+1;
					int total_goals = static_cast<int>(this->path_.end() - this->path_.begin());
					ROS_INFO("Reached goal %d of %d successfully.", goal_num, total_goals);
					++this->waypoint_;
				} else {
					ROS_INFO("Navigation goal failed...");
					ros::shutdown();
				}
				r.sleep();
			}
			ROS_INFO("Navigation completed.");
			return 0;
		}
		
	
	private:
		std::vector<geometry_msgs::PointStamped>::iterator waypoint_;
		std::vector<geometry_msgs::PointStamped> path_;


};


int main(int argc, char* argv[]) {
	if (argc >= 3 && (argc-1) % 2 == 0) {
		PathGenerator path_gen = PathGenerator();
		return path_gen.run(argc,argv);
	} else {
		printf("Invalid number of arguments!");
		return -1;
	}
}

