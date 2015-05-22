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

#include <ar_track_alvar/ParamsConfig.h>

#include <ar_track_alvar_msgs/AlvarMarkers.h>

#include <tf/transform_listener.h>

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
  MarkerDetectorNode();

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

  void imageCallback(const sensor_msgs::ImageConstPtr& p_image_msg);

  MarkerDetector<MarkerData> marker_detector_;
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
};

MarkerDetectorNode::MarkerDetectorNode()
  : node_handle_("~")
  , p_image_transport_(new image_transport::ImageTransport(node_handle_))
  , p_tf_listener_(new tf::TransformListener(node_handle_))
  , p_tf_broadcaster_(new tf::TransformBroadcaster())
{}

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

  if (marker_size_ <= 0.0)
  {
    ROS_ERROR_STREAM("Invalid marker size: " << marker_size_ << " should be > 0 cm");
    return;
  }

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

      alvar_markers_msg_.markers.clear();

      for (size_t m = 0; m < marker_detector_.markers->size(); ++m)
      {
        int id = (*(marker_detector_.markers))[m].GetId();
        // bool is_valid = (*(marker_detector_.markers))[m].valid;
        bool is_valid = true;

        if ( (id >= 0) &&
             is_valid )
        {
          tf::Transform pose;
          bool is_pose_valid = toRosPose((*(marker_detector_.markers))[m].pose, pose);
          if (is_pose_valid)
          {
            std::string marker_frame;
            getMarkerFrame(id, marker_frame);

            tf::StampedTransform marker_wrt_camera(pose, p_image_msg->header.stamp, p_image_msg->header.frame_id, marker_frame);
            p_tf_broadcaster_->sendTransform(marker_wrt_camera);

            tf::Transform marker_wrt_reference = cam_wrt_reference * pose;

            ar_track_alvar_msgs::AlvarMarker marker_msg;
            makeMarkerMsg(id, reference_frame_, p_image_msg->header.stamp, marker_wrt_reference, 1, marker_msg);
            alvar_markers_msg_.markers.push_back(marker_msg);

            visualization_msgs::Marker viz_marker_msg;
            makeVizMarkerMsg(VISIBLE_MARKER, id, p_image_msg->header, pose, marker_size_, viz_marker_msg);

            viz_marker_publisher_.publish(viz_marker_msg);
          }
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

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "image_marker_detector");

  try
  {
    MarkerDetectorNode marker_detector_node;
    marker_detector_node.run();
  }
  catch (std::exception& exception)
  {
    ROS_ERROR_STREAM("Exception caught: " << exception.what());
  }

  return 0;
}
