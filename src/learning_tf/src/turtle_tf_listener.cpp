#include <ros/ros.h>
#include <tf/transform_listener.h>//为简化接收任务
#include <geometry_msgs/Twist.h>
#include <turtlesim/Spawn.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "my_tf_listener");

  ros::NodeHandle node;

  // ros::service::waitForService("spawn");
  // ros::ServiceClient add_turtle =
  //   node.serviceClient<turtlesim::Spawn>("spawn");
  // turtlesim::Spawn srv;
  // add_turtle.call(srv);

  ros::Publisher turtle_vel =
    node.advertise<geometry_msgs::Twist>("turtle2/cmd_vel", 10);

  tf::TransformListener listener;//监听器变量需要缓冲10秒左右,所以要让它持久化
  ros::Rate rate(10.0);
  while (node.ok()){
    tf::StampedTransform transform;
    ros::Duration(1.0).sleep();
  // try{
  //    ros::Time now = ros::Time::now();
  //    listener.waitForTransform("/turtle2","/turtle1",now,ros::Duration(3.0));
  //       //1.目标坐标系 2.源坐标系 3.转换时间 4.结果保存对象 目标坐标系是参考系
  //    listener.lookupTransform("/turtle2", "/turtle1",
  //                              now, transform);//ros::Time(0)是leatest available
  //   std::cout<<transform.getOrigin().x()<<" "<<transform.getOrigin().y()<<std::endl;
  //   } //transform进入buffer需要几毫秒的时间 所以不能请求now()
  // try{
  //   ros::Time past = ros::Time::now() - ros::Duration(5.0);
  //   listener.waitForTransform("/turtle2","/turtle1",past,ros::Duration(1.0));
  //   listener.lookupTransform("/turtle2","/turtle1",past,transform);
  //   //两只龟五秒前位置
  // }
  try{
    ros::Time now = ros::Time::now();
    ros::Time past = now - ros::Duration(5.0);
    listener.waitForTransform("/turtle2", now,
                              "/turtle1", past,
                              "/world", ros::Duration(1.0));
    listener.lookupTransform("/turtle2", now,
                             "/turtle1", past,
                             "/world", transform);
  //1.参考坐标系,2.参考系时间,3.源目标系,4.源目标系时间,5.位置不随时间变化的坐标系,6.存储结果变量
  }
    catch (tf::TransformException &ex) {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }
    geometry_msgs::Twist vel_msg;
    vel_msg.angular.z = 1 * atan2(transform.getOrigin().y(),
                                    transform.getOrigin().x());
    vel_msg.linear.x = 0.5 * sqrt(pow(transform.getOrigin().x(), 2) +
                                  pow(transform.getOrigin().y(), 2));//越远越快
    turtle_vel.publish(vel_msg);

    rate.sleep();
  }
  return 0;
};
