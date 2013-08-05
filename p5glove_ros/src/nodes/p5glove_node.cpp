#include <ros/ros.h>
#include <ros/console.h>
#include <p5glove_ros/GloveData.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Header.h>

#include <p5glove_ros/p5glove.h>
#include <errno.h>
//#include <tf/tf.h>
//#include <tf/LinearMath.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <math.h>

int main(int argc, char **argv)
{
  P5Glove glove;
  glove=p5glove_open(0);
  if (glove == NULL) {
    ROS_FATAL("%s: Can't open glove interface\n", argv[0]);
    return 1;
  }

  ros::init(argc, argv, "p5glove_node");
  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<p5glove_ros::GloveData>("glove_data", 100);
  ros::Publisher pub_pose = nh.advertise<geometry_msgs::PoseStamped>("glove_posestamped", 100);
  ros::Publisher pub_pose2 = nh.advertise<geometry_msgs::PoseStamped>("glove_posestamped2", 100);
  ros::Publisher pub_pose3 = nh.advertise<geometry_msgs::PoseStamped>("glove_posestamped3", 100);



  // variables used for low level data acquisition
  uint32_t buttons;
  double clench;
  double pos[3];
  double axis[3],angle;

  while(ros::ok())
  {
    p5glove_ros::GloveData msg;
    const int err=p5glove_sample(glove, -1);
    if (err < 0 && errno == EAGAIN)
      continue;
    if (err < 0) {
      ROS_ERROR("Glove Failure, detector not finding it, stopped publishing data.");
      continue;
    }

    p5glove_get_buttons(glove,&buttons);
    msg.buttonA = (buttons & P5GLOVE_BUTTON_A) ? 1 : 0;
    msg.buttonB = (buttons & P5GLOVE_BUTTON_B) ? 1 : 0;
    msg.buttonC = (buttons & P5GLOVE_BUTTON_C) ? 1 : 0;
    ROS_DEBUG_STREAM("ButtonA: " << msg.buttonA
                  << "ButtonB: " << msg.buttonB
                  << "ButtonC: " << msg.buttonC);

    for(int i=0; i < 5; ++i)
    {
      p5glove_get_finger(glove,i,&clench);
      msg.fingers.push_back(clench);
      ROS_DEBUG_STREAM("Finger #" << i << " :" << clench);
    }

    p5glove_get_position(glove, pos);
//    msg.pose.position.x = pos[0];
//    msg.pose.position.y = pos[1];
//    msg.pose.position.z = pos[2];

    /* they use weird axis */
    msg.pose.position.x = - pos[2]; // X on gloves axis (see PDF)
    msg.pose.position.y = - pos[0]; // Y
    msg.pose.position.z = pos[1];   // Z
    ROS_DEBUG_STREAM("Position:("
                     << pos[0] << ","
                     << pos[1] << ","
                     << pos[2] << ")");

    p5glove_get_rotation(glove, &angle, axis);
    // from axis-angle representation to Quaternion, but i think this axis is wrong as the position
    tf::Quaternion quat (tf::Vector3(- axis[2], - axis[0], axis[1]), angle * M_PI / 180.0);
    msg.pose.orientation.x = quat.getX();
    msg.pose.orientation.y = quat.getY();
    msg.pose.orientation.z = quat.getZ();
    msg.pose.orientation.w = quat.getW();

    ROS_DEBUG_STREAM("Orientation:("
                     << msg.pose.orientation.x << ","
                     << msg.pose.orientation.y << ","
                     << msg.pose.orientation.z << ","
                     << msg.pose.orientation.w << ")");

    msg.header.stamp = ros::Time::now();
    pub.publish(msg);
    geometry_msgs::PoseStamped posestamped;
    posestamped.pose = msg.pose;
    posestamped.header.stamp= ros::Time::now();
    posestamped.header.frame_id = "base_link";
    pub_pose.publish(posestamped);


    ros::spinOnce();
  }
  p5glove_close(glove);
  return 0;
}
