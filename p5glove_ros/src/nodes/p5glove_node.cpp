#include <ros/ros.h>
#include <p5glove_ros/GloveData.h>

#include <stdio.h>
#include <errno.h>
#include <math.h>
#include <p5glove_ros/p5glove.h>

int main(int argc, char **argv)
{
  P5Glove glove;
  glove=p5glove_open(0);
  if (glove == NULL) {
    fprintf(stderr, "%s: Can't open glove interface\n", argv[0]);
    return 1;
  }

  ros::init(argc, argv, "p5glove_node");
  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<p5glove_ros::GloveData>("data", 100);

  p5glove_ros::GloveData msg;

  // variables used for low level data acquisition
  uint32_t buttons;
  double clench;
  double pos[3];
  double axis[3],angle;

  while(ros::ok())
  {
    const int err=p5glove_sample(glove, -1);
    if (err < 0 && errno == EAGAIN)
      continue;
    if (err < 0) {
      ROS_FATAL("Glove Failure");
      break;
    }

    p5glove_get_buttons(glove,&buttons);
    msg.buttonA = (buttons & P5GLOVE_BUTTON_A) ? 1 : 0;
    msg.buttonB = (buttons & P5GLOVE_BUTTON_B) ? 1 : 0;
    msg.buttonC = (buttons & P5GLOVE_BUTTON_C) ? 1 : 0;

    for(int i=0; i < 5; ++i)
    {
      p5glove_get_finger(glove,i,&clench);
      msg.fingers.push_back(clench);
    }

    p5glove_get_position(glove, pos);
    msg.pose.position.x = pos[0];
    msg.pose.position.y = pos[1];
    msg.pose.position.z = pos[2];

    p5glove_get_rotation(glove, &angle, axis);
    msg.pose.orientation.x = axis[0];
    msg.pose.orientation.y = axis[1];
    msg.pose.orientation.z = axis[2];
    msg.pose.orientation.w = angle;

    msg.header.stamp = ros::Time::now();
    pub.publish(msg);
    ros::spinOnce();
  }
  p5glove_close(glove);
  return 0;
}
