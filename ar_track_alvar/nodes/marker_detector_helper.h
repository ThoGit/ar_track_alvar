#ifndef MARKER_DETECTOR_HELPER_H
#define MARKER_DETECTOR_HELPER_H

#include <ar_track_alvar_msgs/AlvarMarkers.h>

#include <tf/tf.h>

#include <sstream>
#include <map>

#ifdef AR_TRACK_ALVAR_HAVE_YAMLCPP_05
#include <yaml-cpp/yaml.h>
#else
#include <yaml-cpp/node.h>
#include <yaml-cpp/parser.h>
#endif

#include <math.h>

#define MAIN_MARKER 1
#define VISIBLE_MARKER 2
#define GHOST_MARKER 3

typedef std::map<int, size_t> IdxToIdxMap;

struct Bundle
{
  int master_idx;
  tf::Transform pose;
  std::vector<int> marker_idxs;
  std::vector<tf::Transform> marker_pose_constraints;
};

typedef std::vector<Bundle> BundleList;

void getBundleConfiguration(const std::string& configuration_file, Bundle& bundle)
{
  bundle.marker_idxs.clear();
  bundle.marker_pose_constraints.clear();

  YAML::Node configuration_node;
#ifdef AR_TRACK_ALVAR_HAVE_YAMLCPP_05
  configuration_node = YAML::LoadFile(configuration_file.c_str());
#else
  std::ifstream input_file_stream(configuration_file.c_str());
  YAML::Parser parser(input_file_stream);
  parser.GetNextDocument(configuration_node);
#endif

  std::cout << "configuration_node.Type(): " << configuration_node.Type() << std::endl;
  assert(configuration_node.Type() == YAML::NodeType::Map);
  const YAML::Node& master_id_node = configuration_node["master_id"];

#ifdef AR_TRACK_ALVAR_HAVE_YAMLCPP_05
  bundle.master_idx = master_id_node.as<int>();
#else
   master_id_node >> bundle.master_idx;
#endif

  const YAML::Node& bundle_list_node = configuration_node["bundle"];

  for (size_t i = 0; i < bundle_list_node.size(); ++i)
  {
    const YAML::Node& marker_node = bundle_list_node[i];
    const YAML::Node& pose_node = marker_node["pose"];
    const YAML::Node& position_node = pose_node["position"];
    const YAML::Node& orientation_node = pose_node["orientation"];

    int id;
    double x, y, z, roll, pitch, yaw;
#ifdef AR_TRACK_ALVAR_HAVE_YAMLCPP_05
    id = marker_node["id"].as<int>();
    x = position_node["x"].as<double>();
    y = position_node["y"].as<double>();
    z = position_node["z"].as<double>();
    roll = orientation_node["r"].as<double>();
    pitch = orientation_node["p"].as<double>();
    yaw = orientation_node["y"].as<double>();
#else
    marker_node["id"] >> id;
    position_node["x"] >> x;
    position_node["y"] >> y;
    position_node["z"] >> z;
    orientation_node["r"] >> roll;
    orientation_node["p"] >> pitch;
    orientation_node["y"] >> yaw;
#endif

    tf::Vector3 position(x, y, z);
    tf::Quaternion orientation = tf::createQuaternionFromRPY(roll, pitch, yaw);

    bundle.marker_idxs.push_back(id);
    bundle.marker_pose_constraints.push_back(tf::Transform(orientation, position).inverse());
  }
}

void getBundleConfigurations(const std::vector<std::string>& configuration_file_list, BundleList& bundles)
{
  bundles.clear();

  for (size_t i = 0; i < configuration_file_list.size(); ++i)
  {
    Bundle bundle;
    getBundleConfiguration(configuration_file_list[i], bundle);

    bundles.push_back(bundle);
  }
}

void getMarkerFrame(int id, std::string& marker_frame)
{
  std::stringstream string_stream;
  string_stream << "ar_marker_" << id;

  marker_frame =string_stream.str();
}

bool toRosPose(const Pose& alvar_pose, tf::Transform& ros_pose)
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

void toAlvarPose(const tf::Transform& ros_pose, Pose& alvar_pose)
{
  alvar_pose.translation[0] = ros_pose.getOrigin().x() * 100.0;
  alvar_pose.translation[1] = ros_pose.getOrigin().y() * 100.0;
  alvar_pose.translation[2] = ros_pose.getOrigin().z() * 100.0;
  alvar_pose.quaternion[1] = ros_pose.getRotation().x();
  alvar_pose.quaternion[2] = ros_pose.getRotation().y();
  alvar_pose.quaternion[3] = ros_pose.getRotation().z();
  alvar_pose.quaternion[0] = ros_pose.getRotation().w();
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