#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <gazebo_msgs/ModelStates.h>
#include <tf/transform_broadcaster.h>
#include <random>
#include <memory>
#include <iostream>
#include <math.h>
#include <string>

double subtractAngles(double, double);

class odometry_tf_publisher
{
public:
  odometry_tf_publisher(ros::NodeHandle& node_handle, ros::NodeHandle& private_node_handle)
  {
    this->nh = &node_handle;
    this->pnh = &private_node_handle;

    odom_subscriber = node_handle.subscribe("/gazebo/model_states", 10, &odometry_tf_publisher::gazeboCb, this);

//    gt_odom_publisher = node_handle.advertise<nav_msgs::Odometry>("sim_groundtruth", 10);

    encoder_pub = node_handle.advertise<nav_msgs::Odometry>("encoder_odometry", 10);

    mean = 0;

    if(!pnh->getParam("enc_std", encoder_std))
    {
      std::string error_message =
          ros::this_node::getName() +
          ": failed to get the encoder standard deviation from the launch file, please check";
      ROS_ERROR("%s", error_message.c_str());
      ros::shutdown();
    }

    first_run = true;
  }

  void gazeboCb(const gazebo_msgs::ModelStatesPtr& msg)
  {
      tf::Transform transform;

      nav_msgs::Odometry odom_msg;
      for(int i = 0; i < 3; i++)
      {
          if(msg->name[i] == "summit_xl_a")
          {
              odom_msg.pose.pose = msg->pose[i];
              odom_msg.twist.twist = msg->twist[i];
              odom_msg.header.stamp = ros::Time::now();
              odom_msg.header.frame_id = "summit_xl_a_odom";
              odom_msg.child_frame_id = "summit_xl_a_base_footprint";

              gazebo_gt = odom_msg;

              if(first_run)
              {
                 noisy_output = gazebo_gt;
                 previous_gazebo_gt = gazebo_gt;

                 previous_noisy_x = gazebo_gt.pose.pose.position.x;
                 previous_noisy_y = gazebo_gt.pose.pose.position.y;
                 previous_noisy_theta = tf::getYaw(gazebo_gt.pose.pose.orientation);

                 first_run = false;
              }

              ros::Time now = ros::Time::now();
              double delta_time = now.toSec() - last_publish.toSec();

              addMotionIncrement(gazebo_gt, previous_gazebo_gt, noisy_output);

              double x_increment = getXMotionIncrement(gazebo_gt);
              double y_increment = getYMotionIncrement(gazebo_gt);
              double theta_increment = getThetaMotionIncrement(gazebo_gt);

              bool update = fabs(x_increment) > 0.1 || fabs(y_increment) > 0.1 || fabs(theta_increment) > 0.1;

              if(fabs(x_increment) > 0.1)
              {
                addXNoise();
                previous_noisy_x = gazebo_gt.pose.pose.position.x;
              }
              if(fabs(y_increment) > 0.1)
              {
                addYNoise();
                previous_noisy_y = gazebo_gt.pose.pose.position.y;
              }
              if(fabs(theta_increment) > 0.1)
              {
                addThetaNoise();
                previous_noisy_theta = tf::getYaw(gazebo_gt.pose.pose.orientation);
              }

              if(update) previous_gazebo_gt = gazebo_gt;

              previous_gazebo_gt = gazebo_gt;

              if(delta_time > 0.1)
              {
//              gt_odom_publisher.publish(odom_msg);
                last_publish = now;
                noisy_output.header.stamp = ros::Time::now();
                encoder_pub.publish(noisy_output);
              }

              transform.setOrigin(tf::Vector3(noisy_output.pose.pose.position.x, noisy_output.pose.pose.position.y, noisy_output.pose.pose.position.z));
              transform.setRotation(tf::Quaternion(noisy_output.pose.pose.orientation.x, noisy_output.pose.pose.orientation.y, noisy_output.pose.pose.orientation.z, noisy_output.pose.pose.orientation.w));
              this->br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "summit_xl_a_odom", "summit_xl_a_base_footprint"));
          }
      }
  }

  double getXMotionIncrement(nav_msgs::Odometry current)
  {
    double x_motion = current.pose.pose.position.x - previous_noisy_x;
    return x_motion;
  }

  double getYMotionIncrement(nav_msgs::Odometry current)
  {
    double y_motion = current.pose.pose.position.x - previous_noisy_y;
    return y_motion;
  }

  double getThetaMotionIncrement(nav_msgs::Odometry current)
  {
    double current_theta = tf::getYaw(current.pose.pose.orientation);

    double theta_motion = subtractAngles(current_theta, previous_noisy_theta);

    return theta_motion;
  }

  void addXNoise()
  {
     std::normal_distribution<double> nd_(mean, encoder_std);
     double noise = nd_(generator);
     if(encoder_std != 0)
      noisy_output.pose.pose.position.x+=noise;
  }

  void addYNoise()
  {
     std::normal_distribution<double> nd_(mean, encoder_std);
     double noise = nd_(generator);
     if(encoder_std != 0)
      noisy_output.pose.pose.position.y+=noise;
  }

  void addThetaNoise()
  {
    std::normal_distribution<double> nd_(mean, encoder_std);
    double theta = tf::getYaw(noisy_output.pose.pose.orientation);
    double noise = nd_(generator);
    theta+=noise;
    if(encoder_std != 0)
      noisy_output.pose.pose.orientation = tf::createQuaternionMsgFromYaw(theta);
  }

  void addMotionIncrement(nav_msgs::Odometry current, nav_msgs::Odometry previous, nav_msgs::Odometry& noisy_output)
  {
    double delta_x = current.pose.pose.position.x - previous.pose.pose.position.x;
    double delta_y = current.pose.pose.position.y - previous.pose.pose.position.y;

    double current_theta = tf::getYaw(current.pose.pose.orientation);
    double previous_theta = tf::getYaw(previous.pose.pose.orientation);

    double delta_theta = subtractAngles(current_theta, previous_theta);

    double current_noisy_theta = tf::getYaw(noisy_output.pose.pose.orientation);

    noisy_output.pose.pose.position.x+=delta_x;
    noisy_output.pose.pose.position.y+=delta_y;

    double new_noisy_theta = current_noisy_theta + delta_theta;

    noisy_output.pose.pose.orientation = tf::createQuaternionMsgFromYaw(new_noisy_theta);
  }

private:
  tf::TransformBroadcaster br;
  ros::Publisher gt_odom_publisher;
  ros::Publisher encoder_pub;
  ros::Subscriber odom_subscriber;
  ros::NodeHandle* nh;
  ros::NodeHandle* pnh;

  ros::Time last_publish;

  std::default_random_engine generator;

  double mean;
  double encoder_std;

  nav_msgs::Odometry gazebo_gt;
  nav_msgs::Odometry previous_gazebo_gt;
  nav_msgs::Odometry noisy_output;

  double previous_noisy_x;
  double previous_noisy_y;
  double previous_noisy_theta;

  bool first_run;
};

double subtractAngles(double angle_k, double angle_k_1)
{
  double output;
  angle_k = angle_k * 180.0 / M_PI;
  angle_k_1 = angle_k_1 * 180.0 / M_PI;

  if((angle_k_1 < 90 && angle_k_1 > 0) && (angle_k > -90 && angle_k < 0))
  {
    output = abs(angle_k) + abs(angle_k_1);
    output = -output;
  }
  else if((angle_k < 90 && angle_k > 0) && (angle_k_1 > -90 && angle_k_1 < 0))
  {
    output = abs(angle_k) + abs(angle_k_1);
  }
  else if((angle_k > 90 && angle_k < 180) && (angle_k_1 < -90 && angle_k_1 > -180))
  {
    output = 360 - (abs(angle_k) + abs(angle_k_1));
    output = -output;
  }
  else if((angle_k_1 > 90 && angle_k_1 < 180) && (angle_k < -90 && angle_k > -180))
  {
    output = 360 - (abs(angle_k) + abs(angle_k_1));
  }
  else
  {
    output = angle_k - angle_k_1;
  }

  return output * M_PI / 180.0;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "summit_encoder_model");
    ros::NodeHandle node_handle, private_node_handle("~");

    odometry_tf_publisher tf_publisher(node_handle, private_node_handle);

    ros::spin();

    return 0;
}
