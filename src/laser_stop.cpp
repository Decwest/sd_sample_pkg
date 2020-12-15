#include <iostream>
#include <std_msgs/Bool.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h> // scan 

// クラスLaserStopNodeの定義
class LaserStopNode
{
private:
  ros::NodeHandle nh;

  ros::Subscriber sub_laser;
  ros::Publisher pub;

  // laser scanの測定値を保存する変数の定義
  sensor_msgs::LaserScan latest_scan;

  // 2Hzで制御ループを回す
  const int loop_rate = 2;

  // メッセージの初回受信を確認する変数
  bool is_recieved_scan = false;
  std_msgs::Bool msg;

public:
  LaserStopNode()
  {
    sub_laser = nh.subscribe("/light_sensor/front/scan", 10, &LaserStopNode::laser_callback, this);
    pub = nh.advertise<std_msgs::Bool>("/is_detect_object", 1);
  }

  void laser_callback(const sensor_msgs::LaserScanConstPtr &laser_msg)
  {
    latest_scan = *laser_msg;
    is_recieved_scan = true;
  }

  void mainloop()
  {
    ros::Rate r(loop_rate);

    while (ros::ok())
    {
      ros::spinOnce();

      // 値を取得しないままlatest_scan変数にアクセスするとエラーを起こすので判定
      if(!is_recieved_scan) continue;

      // 複数あるセンサの値のうち、中央にある値を取得する
      // http://docs.ros.org/en/api/sensor_msgs/html/msg/LaserScan.html
      const double center_value = latest_scan.ranges[latest_scan.ranges.size() / 2];

      if (center_value < 0.2)
      {
          msg.data = true;
          pub.publish(msg);

          //ROS_INFO("center laser value %5f : rotate", center_value);
      }
      else
      {
        msg.data = false;
        pub.publish(msg);

        //ROS_INFO("center laser value %8f : move on", center_value);
      }

      // 指定したループ周期になるように調整
      r.sleep();
    }
  }
};

int main(int argc, char **argv)
{

  ros::init(argc, argv, "laser_stop");

  LaserStopNode node = LaserStopNode();

  // メインループを回す
  node.mainloop();

  return 0;
}