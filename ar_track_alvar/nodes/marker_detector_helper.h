#ifndef MARKER_DETECTOR_HELPER_H
#define MARKER_DETECTOR_HELPER_H

#include <ar_track_alvar_msgs/AlvarMarkers.h>

#include <tf/tf.h>

#include <sstream>

#include <math.h>

#define MAIN_MARKER 1
#define VISIBLE_MARKER 2
#define GHOST_MARKER 3

void getMarkerFrame(int id, std::string& marker_frame)
{
  std::stringstream string_stream;
  string_stream << "ar_marker_" << id;

  marker_frame =string_stream.str();
}

bool toRosPose(Pose& alvar_pose, tf::Transform& ros_pose)
{
  for (size_t i = 0; i < 3; ++i)
  {
    if ( !std::isfinite(alvar_pose.translation[i]) ||
         !std::isfinite(alvar_pose.quaternion[i]) )
    {
      return false;
    }
  }

  if (!std::isfinite(alvar_pose.quaternion[3]))
  {
    return false;
  }

  double px = alvar_pose.translation[0] / 100.0;
  double py = alvar_pose.translation[1] / 100.0;
  double pz = alvar_pose.translation[2] / 100.0;
  double qx = alvar_pose.quaternion[1];
  double qy = alvar_pose.quaternion[2];
  double qz = alvar_pose.quaternion[3];
  double qw = alvar_pose.quaternion[0];

  tf::Quaternion rotation(qx, qy, qz, qw);
  tf::Vector3 origin(px, py, pz);

  ros_pose = tf::Transform(rotation, origin);

  return true;
}

void makeMarkerMsg(int id, const std::string& reference_frame, const ros::Time& stamp, const tf::Transform& marker_wrt_output, int confidence, ar_track_alvar_msgs::AlvarMarker& marker_msg)
{
  tf::poseTFToMsg(marker_wrt_output, marker_msg.pose.pose);
  marker_msg.header.frame_id = reference_frame;
  marker_msg.header.stamp = stamp;
  marker_msg.id = id;
  marker_msg.confidence = confidence;
}

void makeVizMarkerMsg(int type, int id, const std_msgs::Header& header, const tf::Transform& pose,  double marker_size, visualization_msgs::Marker& viz_marker_msg)
{
  tf::poseTFToMsg(pose, viz_marker_msg.pose);
  viz_marker_msg.header.frame_id = header.frame_id;
  viz_marker_msg.header.stamp = header.stamp;
  viz_marker_msg.id = id;
  viz_marker_msg.scale.x = 1.0 * marker_size / 100.0;
  viz_marker_msg.scale.y = 1.0 * marker_size / 100.0;
  viz_marker_msg.scale.z = 0.2 * marker_size / 100.0;
  viz_marker_msg.type = visualization_msgs::Marker::CUBE;
  viz_marker_msg.action = visualization_msgs::Marker::ADD;
  if (type == MAIN_MARKER)
  {
    viz_marker_msg.ns = "main_shapes";
  }
  else
  {
    viz_marker_msg.ns = "basic_shapes";
  }

  if (type == MAIN_MARKER)
  {
    viz_marker_msg.color.r = 1.0f;
    viz_marker_msg.color.g = 0.0f;
    viz_marker_msg.color.b = 0.0f;
    viz_marker_msg.color.a = 1.0;
  }
  else if (type == VISIBLE_MARKER)
  {
    viz_marker_msg.color.r = 0.0f;
    viz_marker_msg.color.g = 1.0f;
    viz_marker_msg.color.b = 0.0f;
    viz_marker_msg.color.a = 0.7;
  }
  else if (type == GHOST_MARKER)
  {
    viz_marker_msg.color.r = 0.0f;
    viz_marker_msg.color.g = 0.0f;
    viz_marker_msg.color.b = 1.0f;
    viz_marker_msg.color.a = 0.5;
  }

  viz_marker_msg.lifetime = ros::Duration (1.0);
}

#endif // MARKER_DETECTOR_HELPER_H