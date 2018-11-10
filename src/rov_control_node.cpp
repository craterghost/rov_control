#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
//#include <rov_control_node/Adafruit_PWMServoDriver.h>


class ROVControl
{
 public:
   ROVControl();
 private:
   void rovCallback(const geometry_msgs::Twist::ConstPtr &msg);
   //initialize the Rosparts
   ros::Subscriber twistSub;
   ros::NodeHandle nh;
   ros::Timer timeout;
   geometry_msgs::Twist twistMsg;
};


ROVControl::ROVControl() {
	twistSub = nh.subscribe("/cmd_vel", 1000, &ROVControl::rovCallback, this);
}

void ROVControl::rovCallback(const geometry_msgs::Twist::ConstPtr &msg) {
  //Moderate the axes so they fit the PWM
  double linX = (twistMsg.linear.x)*500+1000;
  double linY = (twistMsg.linear.y)*500+1000;
  double linZ = (twistMsg.linear.z)*500+1000;
  double angZ = (twistMsg.angular.z)*500+1000;
  double angY = (twistMsg.angular.y)*500+1000;

  //Add up and constrain multiple axes to get a PWM - value for each motor
  double motor08 = constrain(-linZ, 1100, 1600);
  double motor09 = constrain(-linZ, 1100, 1600);
  double motor13 = constrain(-linZ, 1100, 1600);
  double motor14 = constrain(-linZ, 1100, 1600);
  double motor10 = constrain(-linX+angZ+linY, 1100, 1600);
  double motor11 = constrain(+linX-angZ+linY, 1100, 1600);
  double motor12 = constrain(+linX+angZ-linY, 1100, 1600);
  double motor15 = constrain(-linX-angZ-linY, 1100, 1600);
}




int main(int argc, char** argv) {
	ros::init(argc, argv, "joy_twist_rov");
	ROVControl rov_control_node;
	ros::spin();

	return 0;
}
