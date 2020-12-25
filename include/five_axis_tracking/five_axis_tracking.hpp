#include <ros/ros.h>
#include <ros/console.h>

#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/PointStamped.h>

#include <string>
#include <std_msgs/Float64.h>

class FiveAxisTracking
{
public:
  FiveAxisTracking () ;
  void ballTargetPoint_cb(geometry_msgs::PointStamped targetPoint);

  private:
    bool getPointTransform(const std::string target_frame, const geometry_msgs::PointStamped targetPoint, geometry_msgs::PointStamped& transformed_pt);
    void printTranformationInfo(const std::string msg, const geometry_msgs::PointStamped &transformed_pt, const double &angle);

    ros::NodeHandle nh_;

    ros::Subscriber ball_target_sub_;

    tf2_ros::Buffer tfBuffer_;
    std::shared_ptr<tf2_ros::TransformListener> tfListener_ptr_;

    ros::Publisher a_axis_pub_;
    ros::Publisher c_axis_pub_;

    const bool debug_ = false;
};
