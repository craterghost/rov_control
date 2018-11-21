#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <rov_control/Motor_values.h>
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
   ros::Publisher motorPub;
   ros::NodeHandle nh;
   ros::Timer timeout;
   geometry_msgs::Twist twistMsg;
   rov_control::Motor_values motor;
};


ROVControl::ROVControl() {
  motorPub = nh.advertise<rov_control::Motor_values>("/motor_info", 1000);
	twistSub = nh.subscribe("/cmd_vel", 1000, &ROVControl::rovCallback, this);
}

void ROVControl::rovCallback(const geometry_msgs::Twist::ConstPtr &msg) {
  //Moderate the axes so they fit the PWM
  double linX = msg->linear.x;      //Linker Stick Vertikal
  double linY = msg->linear.y;      //Rechter Stick Horizontal
  double linZ = msg->linear.z;      //Rechter Stick Vertikal
  double angZ = msg->angular.z;     //Rechter Stick Horizontal
  double angY = msg->angular.y;     //Rechter AUX

  //Add up and constrain multiple axes to get a PWM - value for each motor
  double motor08 = constrain(linZ*100+1500, 1400, 1600);
  double motor09 = constrain(linZ*100+1500, 1400, 1600);
  double motor13 = constrain(linZ*-100+1500, 1400, 1600);
  double motor14 = constrain(linZ*100+1500, 1400, 1600);
  double motor10 = constrain(linX*100+angZ*-100+linY*-100+1500, 1400, 1600);
  double motor15 = constrain(linX*-100+angZ*-100+linY*-100+1500, 1400, 1600);
  double motor11 = constrain(linX*-100+angZ*100+linY*-100+1500, 1400, 1600);
  double motor12 = constrain(linX*100+angZ*100+linY*-100+1500, 1400, 1600);

  motor.motor08 = int(motor08);
  motor.motor09 = int(motor09);
  motor.motor10 = int(motor10);
  motor.motor11 = int(motor11);
  motor.motor12 = int(motor12);
  motor.motor13 = int(motor13);
  motor.motor14 = int(motor14);
  motor.motor15 = int(motor15);

  motorPub.publish(motor);

}




int main(int argc, char** argv) {
	ros::init(argc, argv, "joy_rov");
	ROVControl rov_control_node;
	ros::spin();

	return 0;
}
