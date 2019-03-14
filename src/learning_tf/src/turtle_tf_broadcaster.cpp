#include <ros/ros.h>
#include <tf/transform_broadcaster.h> //tf包的TransformBroadcaster帮助简化发布任务
#include <turtlesim/Pose.h>

std::string turtle_name;



void poseCallback(const turtlesim::PoseConstPtr& msg){//订阅位置信息
  static tf::TransformBroadcaster br; //通过该对象发送转换
  tf::Transform transform;//坐标变换对象
  transform.setOrigin( tf::Vector3(msg->x, msg->y, 0.0) );//将2D龟的坐标信息写入 transform中
  tf::Quaternion q;
  q.setRPY(0, 0, msg->theta); //存入RPY
  transform.setRotation(q);//设置旋转
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", turtle_name));
  //1.变换 2.时间戳 3.链接的父框架 4.链接的子框架
}

int main(int argc, char** argv){
  ros::init(argc, argv, "my_tf_broadcaster");
  if (argc != 2){ROS_ERROR("need turtle name as argument"); return -1;};
  turtle_name = argv[1];

  ros::NodeHandle node;
  ros::Subscriber sub = node.subscribe(turtle_name+"/pose", 10, &poseCallback);//订阅位置信息
 // std::cout<<"乌龟"<<turtle_name<<std::endl;
  ros::spin();
  return 0;
};

