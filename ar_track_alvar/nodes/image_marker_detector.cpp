/*
 Software License Agreement (BSD License)

 Copyright (c) 2012, Scott Niekum
 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions
 are met:

  * Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.
  * Redistributions in binary form must reproduce the above
    copyright notice, this list of conditions and the following
    disclaimer in the documentation and/or other materials provided
    with the distribution.
  * Neither the name of the Willow Garage nor the names of its
    contributors may be used to endorse or promote products derived
    from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 POSSIBILITY OF SUCH DAMAGE.

 author: Scott Niekum
*/
/*
 * Modidified by thomas.decandia@sri.com
 */


#include "ar_track_alvar/CvTestbed.h"
#include "ar_track_alvar/MarkerDetector.h"
#include "ar_track_alvar/Shared.h"

#include <ar_track_alvar/filter/medianFilter.h>
#include <ar_track_alvar/ParamsConfig.h>

#include <ar_track_alvar_msgs/AlvarMarkers.h>

#include <tf/transform_listener.h>

#include <eigen_conversions/eigen_msg.h>

#include <sensor_msgs/image_encodings.h>

#include <dynamic_reconfigure/server.h>

#include <cv_bridge/cv_bridge.h>

#include "marker_detector_helper.h"

using namespace alvar;
using namespace std;

/*
 * This class is used to detect AR marker whithin an image.
 */
class MarkerDetectorNode
{
public:
  /**
   * Constructor
   * @param parameters detector parameters
   */
  MarkerDetectorNode(std::vector<Bundle>& bundles);

  /**
   * Destructor
   */
  ~MarkerDetectorNode();

  /**
   * Establish communication with ROS and process inputs.
   */
  void run();

private:
  typedef boost::shared_ptr<Camera> CameraPtr;
  typedef boost::shared_ptr<tf::TransformListener> TransformListenerPtr;
  typedef boost::shared_ptr<tf::TransformBroadcaster> TransformBroadcasterPtr;
  typedef boost::shared_ptr<image_transport::ImageTransport> ImageTransportPtr;
  typedef boost::shared_ptr<ar_track_alvar::MedianFilter> MedianFilterPtr;

  void imageCallback(const sensor_msgs::ImageConstPtr& p_image_msg);
  int updateBundle(Bundle& bundle);

  MarkerDetector<MarkerData> marker_detector_;
  std::vector<Bundle> bundles_;
  IdxToIdxMap marker_to_local_idxs_;
  std::vector<tf::Transform> marker_poses_;
  std::vector<bool> detected_markers_;
  ros::NodeHandle node_handle_;
  ImageTransportPtr p_image_transport_;
  CameraPtr p_camera_;
  TransformListenerPtr p_tf_listener_;
  TransformBroadcasterPtr p_tf_broadcaster_;
  ros::Publisher ar_marker_publisher_;
  ros::Publisher viz_marker_publisher_;
  image_transport::Subscriber image_subscriber_;
  double marker_size_; // [cm]
  double max_new_marker_error_;
  double max_track_error_;
  std::string reference_frame_;
  cv_bridge::CvImagePtr p_cv_bridge_image_;
  ar_track_alvar_msgs::AlvarMarkers alvar_markers_msg_;
  std::vector<MedianFilterPtr> median_filters_;
  int median_filter_size_;
};

MarkerDetectorNode::MarkerDetectorNode(std::vector<Bundle>& bundles)
  : bundles_(bundles)
  , node_handle_("~")
  , p_image_transport_(new image_transport::ImageTransport(node_handle_))
  , p_tf_listener_(new tf::TransformListener(node_handle_))
  , p_tf_broadcaster_(new tf::TransformBroadcaster())
{
  median_filters_.resize(bundles.size());
}

MarkerDetectorNode::~MarkerDetectorNode()
{}

void MarkerDetectorNode::run()
{
  double process_rate;
  std::string image_topic, camera_info_topic;

  node_handle_.param("marker_size", marker_size_, 5.0);
  node_handle_.param("max_new_marker_error", max_new_marker_error_, 0.08);
  node_handle_.param("max_track_error", max_track_error_, 0.2);
  node_handle_.param("image_topic", image_topic, std::string("/camera/rgb/image_raw"));
  node_handle_.param("camera_info_topic", camera_info_topic, std::string("/camera/rgb/camera_info"));
  node_handle_.param("reference_frame", reference_frame_, std::string("camera_link"));
  node_handle_.param("process_rate", process_rate, 50.0);
  node_handle_.param("median_filter_size", median_filter_size_, 0);

  if (marker_size_ <= 0.0)
  {
    ROS_ERROR_STREAM("Invalid marker size: " << marker_size_ << " should be > 0 cm");
    return;
  }

  marker_to_local_idxs_.clear();
  marker_poses_.clear();
  for (size_t b = 0; b < bundles_.size(); ++b)
  {
    const Bundle& bundle = bundles_[b];
    marker_to_local_idxs_[bundle.master_idx] = marker_poses_.size();
    marker_poses_.push_back(tf::Transform());

    for (size_t m = 0; m < bundle.marker_idxs.size(); ++m)
    {
      marker_to_local_idxs_[bundle.marker_idxs[m]] = marker_poses_.size();
      marker_poses_.push_back(tf::Transform());
    }

    if (median_filter_size_ > 0)
    {
      median_filters_[b] = MedianFilterPtr(new ar_track_alvar::MedianFilter(median_filter_size_));
    }
  }

  detected_markers_.assign(marker_poses_.size(), false);

  marker_detector_.SetMarkerSize(marker_size_);

  p_camera_ = CameraPtr(new Camera(node_handle_, camera_info_topic));

  ar_marker_publisher_ = node_handle_.advertise<ar_track_alvar_msgs::AlvarMarkers>("ar_pose_marker", 0);
  viz_marker_publisher_ = node_handle_.advertise<visualization_msgs::Marker>("visualization_marker", 0);

  image_subscriber_ = p_image_transport_->subscribe(image_topic, 1, &MarkerDetectorNode::imageCallback, this);

  ros::Rate rate(process_rate);

  while (ros::ok())
  {
    ros::spinOnce();
    rate.sleep();
  }
}

void MarkerDetectorNode::imageCallback(const sensor_msgs::ImageConstPtr& p_image_msg)
{
  if (p_camera_->getCamInfo_)
  {
    try
    {
      tf::StampedTransform cam_wrt_reference;
      try
      {
        p_tf_listener_->waitForTransform(reference_frame_, p_image_msg->header.frame_id, p_image_msg->header.stamp, ros::Duration(1.0));
        p_tf_listener_->lookupTransform(reference_frame_, p_image_msg->header.frame_id, p_image_msg->header.stamp, cam_wrt_reference);
      }
      catch (tf::TransformException excpetion)
      {
        ROS_ERROR_STREAM("Tf exception caught: " << excpetion.what());
        return;
      }

      //Convert the image
      cv_bridge::CvImagePtr p_cv_bridge_image = cv_bridge::toCvCopy(p_image_msg, sensor_msgs::image_encodings::BGR8);

      // GetMultiMarkersPoses expects an IplImage*, but as of ros groovy, cv_bridge gives
      // us a cv::Mat. I'm too lazy to change to cv::Mat throughout right now, so I
      // do this conversion here -jbinney
      IplImage ipl_image = p_cv_bridge_image->image;

      marker_detector_.Detect(&ipl_image, p_camera_.get(), true, false, max_new_marker_error_, max_track_error_, CVSEQ, true);

      detected_markers_.assign(marker_poses_.size(), false);
      for (size_t m = 0; m < marker_detector_.markers->size(); ++m)
      {
        int idx = (*(marker_detector_.markers))[m].GetId();

        if (idx >= 0)
        {
          tf::Transform pose;
          bool is_pose_valid = toRosPose((*(marker_detector_.markers))[m].pose, pose);
          if (is_pose_valid)
          {
            IdxToIdxMap::iterator iterator = marker_to_local_idxs_.find(idx);
            if (iterator != marker_to_local_idxs_.end())
            {
              int local_idx = iterator->second;
              marker_poses_[local_idx] = pose;
              detected_markers_[local_idx] = true;
            }

            visualization_msgs::Marker viz_marker_msg;
            makeVizMarkerMsg(VISIBLE_MARKER, idx, p_image_msg->header, pose, marker_size_, viz_marker_msg);

            viz_marker_publisher_.publish(viz_marker_msg);
          }
        }
      }

      alvar_markers_msg_.markers.clear();
      for (size_t b = 0; b < bundles_.size(); ++b)
      {
        int marker_seen = updateBundle(bundles_[b]);

        if (marker_seen > 0)
        {
          if (median_filter_size_ > 0)
          {
            Pose alvar_pose;
            toAlvarPose(bundles_[b].pose, alvar_pose);

            median_filters_[b]->addPose(alvar_pose);
            median_filters_[b]->getMedian(alvar_pose);

            toRosPose(alvar_pose, bundles_[b].pose);
          }

          int master_idx = bundles_[b].master_idx;
          tf::Transform bundle_pose =  bundles_[b].pose;

          std::string marker_frame;
          getMarkerFrame(master_idx, marker_frame);

          tf::StampedTransform marker_wrt_camera(bundle_pose, p_image_msg->header.stamp, p_image_msg->header.frame_id, marker_frame);
          p_tf_broadcaster_->sendTransform(marker_wrt_camera);

          tf::Transform marker_wrt_reference = cam_wrt_reference * bundle_pose;

          ar_track_alvar_msgs::AlvarMarker marker_msg;
          makeMarkerMsg(master_idx, reference_frame_, p_image_msg->header.stamp, marker_wrt_reference, marker_seen, marker_msg);
          alvar_markers_msg_.markers.push_back(marker_msg);

          visualization_msgs::Marker viz_marker_msg;
          makeVizMarkerMsg(MAIN_MARKER, master_idx, p_image_msg->header, bundle_pose, marker_size_, viz_marker_msg);

          viz_marker_publisher_.publish(viz_marker_msg);
        }
      }

      ar_marker_publisher_.publish(alvar_markers_msg_);
    }
    catch (cv_bridge::Exception &exception)
    {
      ROS_ERROR_STREAM("ar_track_alvar: cv_bridge exception: " << exception.what());
    }
  }
}

int MarkerDetectorNode::updateBundle(Bundle& bundle)
{
  int seen_marker = 0;

  geometry_msgs::Pose marker_wrt_camera;

  Eigen::Vector3d bundle_position(0.0, 0.0, 0.0);
  Eigen::Vector4d bundle_orientation(0.0, 0.0, 0.0, 0.0);

  int local_idx = marker_to_local_idxs_[bundle.master_idx];
  if (detected_markers_[local_idx])
  {
    tf::poseTFToMsg(marker_poses_[local_idx], marker_wrt_camera);

    Eigen::Vector3d position;
    tf::pointMsgToEigen(marker_wrt_camera.position, position);

    Eigen::Quaterniond orientation;
    tf::quaternionMsgToEigen(marker_wrt_camera.orientation, orientation);
    orientation.normalize();

    bundle_position += position;
    bundle_orientation += Eigen::Vector4d(orientation.w(), orientation.x(), orientation.y(), orientation.z());

    ++seen_marker;
  }

  for (size_t m = 0; m < bundle.marker_idxs.size(); ++m)
  {
    int local_idx = marker_to_local_idxs_[bundle.marker_idxs[m]];
    if (detected_markers_[local_idx])
    {
      tf::poseTFToMsg(marker_poses_[local_idx] * bundle.marker_pose_constraints[m], marker_wrt_camera);

      Eigen::Vector3d position;
      tf::pointMsgToEigen(marker_wrt_camera.position, position);

      Eigen::Quaterniond orientation;
      tf::quaternionMsgToEigen(marker_wrt_camera.orientation, orientation);
      orientation.normalize();

      bundle_position += position;
      bundle_orientation += Eigen::Vector4d(orientation.w(), orientation.x(), orientation.y(), orientation.z());

      ++seen_marker;
    }
  }

  if (seen_marker > 0)
  {
    double n = static_cast<double>(seen_marker);
    bundle_position = bundle_position / n;
    bundle_orientation = bundle_orientation / n;

    Eigen::Quaterniond orientation(bundle_orientation(0), bundle_orientation(1), bundle_orientation(2), bundle_orientation(3));
    orientation.normalize();

    geometry_msgs::Pose bundle_pose;
    tf::pointEigenToMsg(bundle_position, bundle_pose.position);
    tf::quaternionEigenToMsg(orientation, bundle_pose.orientation);

    tf::poseMsgToTF(bundle_pose, bundle.pose);
  }

  return seen_marker;
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "image_marker_detector");

  std::vector<std::string> bundle_config_list;
  for (int a = 1; a < argc; ++a)
  {
    bundle_config_list.push_back(std::string(argv[a]));
  }

  std::vector<Bundle> bundles;
  getBundleConfigurations(bundle_config_list, bundles);

  try
  {
    MarkerDetectorNode marker_detector_node(bundles);
    marker_detector_node.run();
  }
  catch (std::exception& exception)
  {
    ROS_ERROR_STREAM("Exception caught: " << exception.what());
  }

  return 0;
}
