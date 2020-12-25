#include <five_axis_tracking/five_axis_tracking.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

FiveAxisTracking::FiveAxisTracking ()
{
    ball_target_sub_ = nh_.subscribe("ball_target_point", 1, &FiveAxisTracking::ballTargetPoint_cb, this);

    a_axis_pub_ = nh_.advertise<std_msgs::Float64>("/five_axis_robot/joint1_position_controller/command", 30);
    c_axis_pub_ = nh_.advertise<std_msgs::Float64>("/five_axis_robot/joint2_position_controller/command", 30);

    tfListener_ptr_ = std::make_shared<tf2_ros::TransformListener>(tfBuffer_);
}

void FiveAxisTracking::ballTargetPoint_cb(geometry_msgs::PointStamped targetPoint)
{
    double a_axis_angle, c_axis_angle;
    std_msgs::Float64 axis_angle_msg;
    geometry_msgs::PointStamped  a_axis_transformed_pt ;

    if (getPointTransform("a_axis_servo_link", targetPoint, a_axis_transformed_pt))
    {
        a_axis_angle = atan2(
            a_axis_transformed_pt.point.x,
            a_axis_transformed_pt.point.y);

        if (debug_)
            printTranformationInfo("A axis: ", a_axis_transformed_pt, a_axis_angle);

        axis_angle_msg.data = -a_axis_angle;
        a_axis_pub_.publish(axis_angle_msg);
    }

    geometry_msgs::PointStamped  c_axis_transformed_pt ;
    if (getPointTransform("c_axis_servo_link", targetPoint, c_axis_transformed_pt))
    {
        c_axis_angle = atan2(
            -c_axis_transformed_pt.point.y,
            -c_axis_transformed_pt.point.x);

        if (debug_)
            printTranformationInfo("C axis: ", c_axis_transformed_pt, c_axis_angle);

        axis_angle_msg.data = c_axis_angle + 0.1;
        c_axis_pub_.publish(axis_angle_msg);
    }
}

bool FiveAxisTracking::getPointTransform(const std::string target_frame, const geometry_msgs::PointStamped targetPoint, geometry_msgs::PointStamped& transformed_pt)
{
    bool success = true;
    geometry_msgs::TransformStamped transformStamped;
    try
    {
        transformStamped = tfBuffer_.lookupTransform(target_frame, targetPoint.header.frame_id, ros::Time(0), ros::Duration(3.0));
        tf2::doTransform(targetPoint, transformed_pt, transformStamped);
    }
    catch (tf2::TransformException &ex)
    {
      ROS_WARN("%s",ex.what()); ros::Duration(1.0).sleep();
      success = false;
    }
    return success;
}

void FiveAxisTracking::printTranformationInfo(const std::string msg, const geometry_msgs::PointStamped &transformed_pt, const double &angle)
{
    std::cout << msg \
              << transformed_pt.point.x << " "  \
              << transformed_pt.point.y << " " \
              << transformed_pt.point.z << " " \
              << angle << std::endl;

}

int
main (int argc, char **argv)
{
  ros::init (argc, argv, "five_axis_tracking_node");
  FiveAxisTracking fam; //this loads up the node
  ros::spin (); //where she stops nobody knows
  return 0;
}
