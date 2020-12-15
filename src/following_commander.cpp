#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>

#include <geometry_msgs/Twist.h>

#define I_T_MAX 0.785398 //PI/4
#define V_T_MAX 3.1415

geometry_msgs::Twist cmd_vel;

float x = 0.5;
float image_center = 0.5;
float v = 0, theta = 0;
float Kp_t=0.8, Ki_t=0, Kd_t=0;
bool is_detect = false;

float PID(const float goal, const float present, const float Kp, const float Ki,
          const float Kd, float &I, const float I_Limit, float (&diff)[2]);
void saturate(float &value, const float limit);

void msgCallback(const std_msgs::Float32::ConstPtr& msg){
    x = msg->data;
}

void objectCallback(const std_msgs::Bool::ConstPtr& msg){
    is_detect = msg->data;
}

int main(int argc, char **argv){
    //config
    ros::init(argc,argv,"following_commander");
    ros::NodeHandle n;//make nodehandler

    ros::Publisher pub=n.advertise<geometry_msgs::Twist>("cmd_vel",1);
    ros::Subscriber sub=n.subscribe("BBox_center",1, msgCallback);
    ros::Subscriber sub_objest=n.subscribe("is_detect_object",1, objectCallback);

    ros::Rate loop_rate(30);//looprate(Hz)
    //main
    while(ros::ok()){//keep moving while this node is activated 
    //PID
    static float I_t = 0, v_theta = 0.0;
    static float diff_t[2] = {};
    v_theta =
        PID(x, image_center, Kp_t, Ki_t, Kd_t, I_t, I_T_MAX, diff_t);//for theta
    saturate(v_theta, V_T_MAX);
    if(is_detect){
      cmd_vel.linear.x = 0.0;
      cmd_vel.angular.z = 0.0;
    }
    else{
      cmd_vel.linear.x = 0.15;
      cmd_vel.angular.z = v_theta;
    }
    //ROS_INFO("diff_d, %f %f",diff_d[0],diff_d[1]);
    pub.publish(cmd_vel);  // publish

    ros::spinOnce();//call CallBack function once
    loop_rate.sleep();//sleep to maintain accurate loop rate
    }
    return 0;
}

float PID(const float goal, const float present, const float Kp, const float Ki, const float Kd, float &I, const float I_Limit, float (&diff)[2]){
  diff[0] = diff[1];
  diff[1] = present - goal;

  float P = diff[1];
  I += (diff[1] + diff[0]) / 2.0f / 30.0;
  float D = (diff[1] - diff[0]) * 30.0;

  saturate(I, I_Limit);
  float v = Kp * P + Ki * I + Kd * D;
  return v;
}

void saturate(float &value, const float limit){//prevent I value not to divergent
  if (value > limit) value = limit;
  else if (value < -1*limit) value = -1 * limit;
}