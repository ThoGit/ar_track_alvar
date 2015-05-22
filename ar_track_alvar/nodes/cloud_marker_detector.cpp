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
#include "ar_track_alvar/CvTestbed.h"
#include "ar_track_alvar/MarkerDetector.h"
#include "ar_track_alvar/MultiMarkerBundle.h"
#include "ar_track_alvar/MultiMarkerInitializer.h"
#include "ar_track_alvar/Shared.h"
#include <cv_bridge/cv_bridge.h>
#include <ar_track_alvar_msgs/AlvarMarker.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include <tf/transform_listener.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/registration.h>

#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/image_encodings.h>
#include <ros/ros.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/filters/extract_indices.h>
#include <boost/lexical_cast.hpp>

#include <tf/tf.h>
#include <Eigen/Core>
#include <ar_track_alvar/filter/kinect_filtering.h>
#include <ar_track_alvar/filter/medianFilter.h>

#include "marker_detector_helper.h"

namespace gm=geometry_msgs;
namespace ata=ar_track_alvar;

typedef pcl::PointXYZRGB ARPoint;
typedef pcl::PointCloud<ARPoint> ARCloud;
typedef boost::shared_ptr<MultiMarkerBundle> MultiMarkerBundlePtr;

using namespace alvar;
using namespace std;
using boost::make_shared;

static const int EXPECTED_ARGUMENT_COUNT = 9;

struct MarkerDetectorParameters
{
  double marker_size;
  double max_new_marker_error;
  double max_track_error;
  std::string cam_image_topic;
  std::string cam_info_topic;
  std::string output_frame;
  int med_filt_size;
  int n_bundles;
  std::vector<MultiMarkerBundlePtr> multi_marker_bundles;
};

void getParametersFromArguments(const int argc, char *argv[], MarkerDetectorParameters& parameters)
{
  assert(argc >= EXPECTED_ARGUMENT_COUNT);

  // Get params from command line
  parameters.marker_size = static_cast<double>(atof(argv[1]));
  parameters.max_new_marker_error = static_cast<double>(atof(argv[2]));
  parameters.max_track_error = static_cast<double>(atof(argv[3]));
  parameters.cam_image_topic = std::string(argv[4]);
  parameters.cam_info_topic = std::string(argv[5]);
  parameters.output_frame = std::string(argv[6]);
  parameters.med_filt_size = atoi(argv[7]);
  parameters.n_bundles = argc - (EXPECTED_ARGUMENT_COUNT - 1);
  parameters.multi_marker_bundles.clear();

  for (int i = (EXPECTED_ARGUMENT_COUNT - 1); i < argc; ++i)
  {
    MultiMarker multi_marker_config;
    bool is_config_loaded = multi_marker_config.Load(argv[i] , FILE_FORMAT_XML);
    if (!is_config_loaded)
    {
      throw std::runtime_error(std::string("Unable to load xml bundle config: ") + std::string(argv[i]));
    }

    vector<int> indices = multi_marker_config.getIndices();

    MultiMarkerBundlePtr p_multi_marker_bundle(new MultiMarkerBundle(indices));
    p_multi_marker_bundle->Load(argv[i] , FILE_FORMAT_XML);

    parameters.multi_marker_bundles.push_back(p_multi_marker_bundle);
  }
}

/*
 * This class is used to detect AR marker whithin a PointCloud.
 */
class MarkerDetectorNode
{
public:
  /**
   * Constructor
   * @param parameters detector parameters
   */
  MarkerDetectorNode(const MarkerDetectorParameters& parameters);

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
  typedef boost::shared_ptr<ata::MedianFilter> MedianFilterPtr;

  void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& p_point_cloud);
  int makeMasterTransform(const CvPoint3D64f& p0, const CvPoint3D64f& p1, const CvPoint3D64f& p2, const CvPoint3D64f& p3, tf::Transform& transform);
  int setMasterCoordinates(MultiMarkerBundle& multi_marker_bundle);
  void getMultiMarkerPoses(IplImage* p_image, ARCloud& cloud);
  int planeFitPoseImprovement(int id, const ARCloud& corners_cloud, ARCloud::Ptr p_selected_points, const ARCloud& cloud, Pose& marker_pose);
  int inferCorners(const ARCloud& cloud, MultiMarkerBundle& multi_marker_bundle, ARCloud& bundle_corners);
  void broadcastMarkerTf(const std_msgs::Header& image_header, const std::string marker_frame, tf::Transform& pose);
  void draw3dPoints(ARCloud::Ptr p_cloud, const string& frame, int color, int id, double scale);
  void drawArrow(gm::Point& arrow_origin, const tf::Matrix3x3& tf_matrix, const string& frame, int color, int id);

  ros::NodeHandle node_handle_;
  CameraPtr p_camera_;
  TransformListenerPtr p_tf_listener_;
  TransformBroadcasterPtr p_tf_broadcaster_;
  ros::Publisher ar_marker_publisher_;
  ros::Publisher viz_marker_publisher_;
  ros::Publisher viz_marker_debug_publisher_;
  ros::Subscriber point_cloud_subscriber_;
  MarkerDetector<MarkerData> marker_detector_;
  MarkerDetectorParameters parameters_;
  std::vector<Pose> bundle_poses_;
  std::vector<int> master_ids_;
  std::vector<std::vector<int> > bundles_indices_;
  std::vector<int> seen_bundles_;
  std::vector<bool> visible_masters_;
  std::vector<MedianFilterPtr> median_filters_;
  ar_track_alvar_msgs::AlvarMarkers alvar_markers_msg_;
};

MarkerDetectorNode::MarkerDetectorNode(const MarkerDetectorParameters& parameters)
  : parameters_(parameters)
{
  marker_detector_.SetMarkerSize(parameters.marker_size);
  bundle_poses_.resize(parameters.n_bundles);
  master_ids_.resize(parameters.n_bundles);
  bundles_indices_.resize(parameters.n_bundles);
  median_filters_.resize(parameters.n_bundles);
  seen_bundles_.resize(parameters.n_bundles);
  visible_masters_.resize(parameters.n_bundles);

  for (int i = 0; i < parameters.n_bundles; ++i)
  {
    bundle_poses_[i].Reset();

    master_ids_[i] = (parameters.multi_marker_bundles[i])->getMasterId();

    bundles_indices_[i] = (parameters.multi_marker_bundles[i])->getIndices();

    median_filters_[i] = MedianFilterPtr(new ata::MedianFilter(parameters.med_filt_size));

    int result = setMasterCoordinates(*(parameters.multi_marker_bundles[i]));
    if (result != 0)
    {
      throw std::runtime_error("Failed to compute transform info between ROS and ArMarker.");
    }
  }
}

MarkerDetectorNode::~MarkerDetectorNode()
{}

void MarkerDetectorNode::run()
{
  double process_rate;

  node_handle_.param("process_rate", process_rate, 30.0);

  p_camera_ = CameraPtr(new Camera(node_handle_, parameters_.cam_info_topic));

  p_tf_listener_ = TransformListenerPtr(new tf::TransformListener(node_handle_));
  p_tf_broadcaster_ = TransformBroadcasterPtr(new tf::TransformBroadcaster());

  ar_marker_publisher_ = node_handle_.advertise<ar_track_alvar_msgs::AlvarMarkers>("ar_pose_marker", 0);
  viz_marker_publisher_ = node_handle_.advertise<visualization_msgs::Marker>("visualization_marker", 0);
  viz_marker_debug_publisher_ = node_handle_.advertise<visualization_msgs::Marker>("ARmarker_points", 0);

  // give tf a chance to catch up before the camera callback starts asking for transforms
  ros::Duration(1.0).sleep();
  ros::spinOnce();

  // subscribe to topics and set up callbacks
  ROS_INFO ("Subscribing to image topic");
  point_cloud_subscriber_ = node_handle_.subscribe(parameters_.cam_image_topic, 1, &MarkerDetectorNode::pointCloudCallback, this);

  ros::Rate rate(process_rate);

  while (ros::ok())
  {
    ros::spinOnce();
    rate.sleep();
  }
}

void MarkerDetectorNode::pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& p_point_cloud)
{
  //If we've already gotten the cam info, then go ahead
  if(p_camera_->getCamInfo_)
  {
    try
    {
      // Convert cloud to PCL
      pcl::PCLPointCloud2 pcl_cloud2;
      pcl_conversions::toPCL(*p_point_cloud, pcl_cloud2);

      ARCloud cloud;
      pcl::fromPCLPointCloud2(pcl_cloud2, cloud);

      // Get an OpenCV image from the cloud
      sensor_msgs::ImagePtr p_image_msg(new sensor_msgs::Image);
      pcl::toROSMsg(*p_point_cloud, *p_image_msg);
      p_image_msg->header = p_point_cloud->header;

      // Convert the image
      ROS_INFO_STREAM("cv_bridge::toCvCopy");
      cv_bridge::CvImagePtr p_cv_bridge_image = cv_bridge::toCvCopy(p_image_msg, sensor_msgs::image_encodings::BGR8);

      // Get the estimated pose of the main markers by using all the markers in each bundle

      // GetMultiMarkersPoses expects an IplImage*, but as of ros groovy, cv_bridge gives
      // us a cv::Mat. I'm too lazy to change to cv::Mat throughout right now, so I
      // do this conversion here -jbinney
      ROS_INFO_STREAM("getMultiMarkerPoses");
      IplImage ipl_image = p_cv_bridge_image->image;
      getMultiMarkerPoses(&ipl_image, cloud);

      // Get the transformation from the Camera to the output frame for this image capture
      tf::StampedTransform cam_wrt_output;
      try
      {
        p_tf_listener_->waitForTransform(parameters_.output_frame, p_point_cloud->header.frame_id, p_point_cloud->header.stamp, ros::Duration(1.0));
        p_tf_listener_->lookupTransform(parameters_.output_frame, p_point_cloud->header.frame_id, p_point_cloud->header.stamp, cam_wrt_output);
      }
      catch (tf::TransformException ex)
      {
        ROS_ERROR("%s",ex.what());
        return;
      }

      visualization_msgs::Marker viz_marker_msg;
      for (size_t m = 0 ; m < marker_detector_.markers->size(); ++m)
      {
        int id = (*(marker_detector_.markers))[m].GetId();
        bool is_valid = (*(marker_detector_.markers))[m].valid;

        if ( (id >= 0) &&
             is_valid )
        {
          tf::Transform pose;
          bool is_pose_valid = toRosPose((*(marker_detector_.markers))[m].pose, pose);
          if (is_pose_valid)
          {
            std::string marker_frame;
            getMarkerFrame(id, marker_frame);

            bool is_master = false;
            for (int j = 0; j < parameters_.n_bundles; j++)
            {
              if(id == master_ids_[j])
              {
                is_master = true;
              }
            }

            if (!is_master)
            {
              broadcastMarkerTf(p_image_msg->header, marker_frame, pose);
            }

            makeVizMarkerMsg(VISIBLE_MARKER, id, p_image_msg->header, pose, parameters_.marker_size, viz_marker_msg);

            viz_marker_publisher_.publish(viz_marker_msg);
          }
        }
      }

      // Draw the main markers, whether they are visible or not -- but only if at least 1 marker from their bundle is currently seen
      // Init and clear markers
      ar_track_alvar_msgs::AlvarMarker marker_msg;
      alvar_markers_msg_.markers.clear ();
      for (int b = 0; b < parameters_.n_bundles; ++b)
      {
        if (seen_bundles_[b] > 0)
        {
          int master_id = master_ids_[b];

          tf::Transform pose;
          bool is_pose_valid = toRosPose(bundle_poses_[b], pose);
          if (is_pose_valid)
          {
            std::string marker_frame;
            getMarkerFrame(master_id, marker_frame);

            broadcastMarkerTf(p_image_msg->header, marker_frame, pose);

            tf::Transform marker_wrt_reference = cam_wrt_output * pose;
            makeMarkerMsg(master_id, parameters_.output_frame, p_image_msg->header.stamp, marker_wrt_reference, seen_bundles_[b], marker_msg);
            alvar_markers_msg_.markers.push_back(marker_msg);

            makeVizMarkerMsg(MAIN_MARKER, master_id, p_image_msg->header, pose, parameters_.marker_size, viz_marker_msg);

            viz_marker_publisher_.publish(viz_marker_msg);
          }
        }
      }

      // Publish the marker messages
      ar_marker_publisher_.publish(alvar_markers_msg_);
    }
    catch (cv_bridge::Exception& exception)
    {
      ROS_ERROR("ar_track_alvar: cv_bridge exception: %s", exception.what());
    }
  }
}

int MarkerDetectorNode::setMasterCoordinates(MultiMarkerBundle& multi_marker_bundle)
{
  std::vector<tf::Vector3> rel_corner_coords;

  //Go through all the markers associated with this bundle
  for (size_t i = 0; i < multi_marker_bundle.marker_indices.size(); i++)
  {
    int mark_id = multi_marker_bundle.marker_indices[i];
    rel_corner_coords.clear();

    //Get the coords of the corners of the child marker in the master frame
    CvPoint3D64f mark_corners[4];
    for(int j = 0; j < 4; j++)
    {
      mark_corners[j] = multi_marker_bundle.pointcloud[multi_marker_bundle.pointcloud_index(mark_id, j)];
    }

    //Use them to find a transform from the master frame to the child frame
    tf::Transform transform;
    int result = makeMasterTransform(mark_corners[0], mark_corners[1], mark_corners[2], mark_corners[3], transform);
    if (result != 0)
    {
      return result;
    }

    //Finally, find the coords of the corners of the master in the child frame
    for(int j=0; j<4; j++){

      CvPoint3D64f corner_coord = multi_marker_bundle.pointcloud[multi_marker_bundle.pointcloud_index(multi_marker_bundle.master_id, j)];
      double px = corner_coord.x;
      double py = corner_coord.y;
      double pz = corner_coord.z;

      tf::Vector3 corner_vec (px, py, pz);
      tf::Vector3 ans = (transform.inverse()) * corner_vec;
      rel_corner_coords.push_back(ans);
    }

    multi_marker_bundle.rel_corners.push_back(rel_corner_coords);
  }

  return 0;
}

int MarkerDetectorNode::makeMasterTransform(const CvPoint3D64f& p0, const CvPoint3D64f& p1, const CvPoint3D64f& p2, const CvPoint3D64f& p3, tf::Transform& transform)
{
  const tf::Vector3 q0(p0.x, p0.y, p0.z);
  const tf::Vector3 q1(p1.x, p1.y, p1.z);
  const tf::Vector3 q2(p2.x, p2.y, p2.z);
  const tf::Vector3 q3(p3.x, p3.y, p3.z);

  // (inverse) matrix with the given properties
  const tf::Vector3 v = (q1-q0).normalized();
  const tf::Vector3 w = (q2-q1).normalized();
  const tf::Vector3 n = v.cross(w);
  tf::Matrix3x3 tf_matrix(v.x(), v.y(), v.z(), w.x(), w.y(), w.z(), n.x(), n.y(), n.z());
  tf_matrix = tf_matrix.inverse();

  //Translate to quaternion
  if (tf_matrix.determinant() <= 0)
  {
    return -1;
  }

  //Use Eigen for this part instead, because the ROS version of bullet appears to have a bug
  Eigen::Matrix3f matrix;
  for(int i = 0; i < 3; i++)
  {
    matrix(i, 0) = tf_matrix[i].x();
    matrix(i, 1) = tf_matrix[i].y();
    matrix(i, 2) = tf_matrix[i].z();
  }

  Eigen::Quaternion<float> quaternion(matrix);

  // Translate back to bullet
  tfScalar ex = quaternion.x();
  tfScalar ey = quaternion.y();
  tfScalar ez = quaternion.z();
  tfScalar ew = quaternion.w();

  tf::Quaternion tf_quaternion(ex,ey,ez,ew);
  tf_quaternion = tf_quaternion.normalized();

  double qx = (q0.x() + q1.x() + q2.x() + q3.x()) / 4.0;
  double qy = (q0.y() + q1.y() + q2.y() + q3.y()) / 4.0;
  double qz = (q0.z() + q1.z() + q2.z() + q3.z()) / 4.0;
  tf::Vector3 origin(qx,qy,qz);

  transform = tf::Transform(tf_quaternion, origin);  //transform from master to marker

  return 0;
}

// Updates the bundlePoses of the multi_marker_bundles by detecting markers and
// using all markers in a bundle to infer the master tag's position
void MarkerDetectorNode::getMultiMarkerPoses(IplImage* p_image, ARCloud& cloud)
{
  visible_masters_.assign(parameters_.n_bundles, false);
  seen_bundles_.assign(parameters_.n_bundles, 0);

  //Detect and track the markers
  bool detected = marker_detector_.Detect(p_image, p_camera_.get(), true, false, parameters_.max_new_marker_error, parameters_.max_track_error, CVSEQ, true);
  if (detected)
  {
    for (size_t i = 0; i < marker_detector_.markers->size(); i++)
    {
      vector<cv::Point, Eigen::aligned_allocator<cv::Point> > pixels;
      Marker* p_marker = &((*marker_detector_.markers)[i]);
      int id = p_marker->GetId();

      //Get the 3D points of the outer corners
      /*
      PointDouble corner0 = m->marker_corners_img[0];
      PointDouble corner1 = m->marker_corners_img[1];
      PointDouble corner2 = m->marker_corners_img[2];
      PointDouble corner3 = m->marker_corners_img[3];
      m->ros_corners_3D[0] = cloud(corner0.x, corner0.y);
      m->ros_corners_3D[1] = cloud(corner1.x, corner1.y);
      m->ros_corners_3D[2] = cloud(corner2.x, corner2.y);
      m->ros_corners_3D[3] = cloud(corner3.x, corner3.y);
      */

      //Get the 3D inner corner points - more stable than outer corners that can "fall off" object
      int resolution = p_marker->GetRes();
      int ori = p_marker->ros_orientation;

      PointDouble pt1, pt2, pt3, pt4;
      pt4 = p_marker->ros_marker_points_img[0];
      pt3 = p_marker->ros_marker_points_img[resolution - 1];
      pt1 = p_marker->ros_marker_points_img[(resolution * resolution) - resolution];
      pt2 = p_marker->ros_marker_points_img[(resolution * resolution) - 1];

      p_marker->ros_corners_3D[0] = cloud(pt1.x, pt1.y);
      p_marker->ros_corners_3D[1] = cloud(pt2.x, pt2.y);
      p_marker->ros_corners_3D[2] = cloud(pt3.x, pt3.y);
      p_marker->ros_corners_3D[3] = cloud(pt4.x, pt4.y);

      if (ori >= 0 && ori < 4)
      {
        if (ori != 0)
        {
          std::rotate(p_marker->ros_corners_3D.begin(), p_marker->ros_corners_3D.begin() + ori, p_marker->ros_corners_3D.end());
        }
      }
      else
      {
        ROS_ERROR("FindMarkerBundles: Bad Orientation: %i for ID: %i", ori, id);
      }

      //Check if we have spotted a master tag
      int master_index = -1;
      for (int j = 0; j < parameters_.n_bundles; j++)
      {
        if (id == master_ids_[j])
        {
          visible_masters_[j] = true;
          master_index = j;
        }
      }

      //Mark the bundle that marker belongs to as "seen"
      int bundle_index = -1;
      for (int j = 0; j < parameters_.n_bundles; j++)
      {
        for (int k = 0; k < bundles_indices_[j].size(); k++)
        {
          if (bundles_indices_[j][k] == id)
          {
            bundle_index = j;
            seen_bundles_[j] += 1;
            break;
          }
        }
      }

      //Get the 3D marker points
      BOOST_FOREACH (const PointDouble& p, p_marker->ros_marker_points_img)
        pixels.push_back(cv::Point(p.x, p.y));

      ARCloud::Ptr selected_points = ata::filterCloud(cloud, pixels);

      //Use the kinect data to find a plane and pose for the marker
      int result = planeFitPoseImprovement(i, p_marker->ros_corners_3D, selected_points, cloud, p_marker->pose);
      //If the plane fit fails...
      if(result < 0)
      {
        cout << "Plane fit failed" << endl;
        //Mark this tag as invalid
        p_marker->valid = false;
        //If this was a master tag, reset its visibility
        if (master_index >= 0)
        {
          visible_masters_[master_index] = false;
        }

        //decrement the number of markers seen in this bundle
        seen_bundles_[bundle_index] -= 1;
      }
      else
      {
        p_marker->valid = true;
      }
    } // for (..)

    //For each master tag, infer the 3D position of its corners from other visible tags
    //Then, do a plane fit to those new corners
    ARCloud inferred_corners;
    for (int i = 0; i < parameters_.n_bundles; i++)
    {
      if (seen_bundles_[i] > 0)
      {
        if( visible_masters_[i] &&
            seen_bundles_[i] == 1 )
        {
          for (size_t j = 0; j < marker_detector_.markers->size(); j++)
          {
            Marker *p_m = &((*marker_detector_.markers)[j]);
            if (p_m->GetId() == master_ids_[i])
            {
              bundle_poses_[i] = p_m->pose;
            }
          }
        }
        else
        {
          int result = inferCorners(cloud, *(parameters_.multi_marker_bundles[i]), inferred_corners);
          if (result >= 0)
          {
            ARCloud::Ptr inferred_cloud(new ARCloud(inferred_corners));
            planeFitPoseImprovement(i+5000, inferred_corners, inferred_cloud, cloud, bundle_poses_[i]);
          }
          else
          {
            cout << "Unable to infer corner" << endl;
          }
        }

        if (parameters_.med_filt_size > 0)
        {
          median_filters_[i]->addPose(bundle_poses_[i]);
          median_filters_[i]->getMedian(bundle_poses_[i]);
        }
      }
    }
  }
}

int MarkerDetectorNode::planeFitPoseImprovement(int id, const ARCloud& corners_cloud, ARCloud::Ptr p_selected_points, const ARCloud& cloud, Pose& marker_pose)
{
  if (corners_cloud.size() < 4)
  {
    return -1;
  }

  ata::PlaneFitResult plane_fit_result;
  ata::fitPlane(p_selected_points, plane_fit_result);
  if (plane_fit_result.inliers->size() <= 0)
  {
    return -1;
  }

  gm::PoseStamped pose_stamped;
  pose_stamped.header.stamp = pcl_conversions::fromPCL(cloud.header.stamp);
  pose_stamped.header.frame_id = cloud.header.frame_id;
  pose_stamped.pose.position = ata::centroid(*(plane_fit_result.inliers));

  draw3dPoints(p_selected_points, cloud.header.frame_id, 1, id, 0.005);

  //Get 2 points that point forward in marker x direction
  int i1,i2;
  if ( isnan(corners_cloud[0].x) || isnan(corners_cloud[0].y) || isnan(corners_cloud[0].z) ||
       isnan(corners_cloud[3].x) || isnan(corners_cloud[3].y) || isnan(corners_cloud[3].z) )
  {
    if ( isnan(corners_cloud[1].x) || isnan(corners_cloud[1].y) || isnan(corners_cloud[1].z) ||
         isnan(corners_cloud[2].x) || isnan(corners_cloud[2].y) || isnan(corners_cloud[2].z) )
    {
      return -1;
    }
    else
    {
      i1 = 1;
      i2 = 2;
    }
  }
  else
  {
    i1 = 0;
    i2 = 3;
  }

  //Get 2 points the point forward in marker y direction
  int i3,i4;
  if ( isnan(corners_cloud[0].x) || isnan(corners_cloud[0].y) || isnan(corners_cloud[0].z) ||
       isnan(corners_cloud[1].x) || isnan(corners_cloud[1].y) || isnan(corners_cloud[1].z) )
  {
    if ( isnan(corners_cloud[3].x) || isnan(corners_cloud[3].y) || isnan(corners_cloud[3].z) ||
         isnan(corners_cloud[2].x) || isnan(corners_cloud[2].y) || isnan(corners_cloud[2].z) )
    {
      return -1;
    }
    else
    {
      i3 = 2;
      i4 = 3;
    }
  }
  else
  {
    i3 = 1;
    i4 = 0;
  }

  ARCloud::Ptr orient_points(new ARCloud());
  orient_points->points.push_back(corners_cloud[i1]);
  draw3dPoints(orient_points, cloud.header.frame_id, 3, id+1000, 0.008);

  orient_points->clear();
  orient_points->points.push_back(corners_cloud[i2]);
  draw3dPoints(orient_points, cloud.header.frame_id, 2, id+2000, 0.008);

  int result;
  result = ata::extractOrientation(plane_fit_result.coeffs, corners_cloud[i1], corners_cloud[i2], corners_cloud[i3], corners_cloud[i4], pose_stamped.pose.orientation);
  if (result < 0) return -1;

  tf::Matrix3x3 mat;
  result = ata::extractFrame(plane_fit_result.coeffs, corners_cloud[i1], corners_cloud[i2], corners_cloud[i3], corners_cloud[i4], mat);
  if (result < 0) return -1;

  drawArrow(pose_stamped.pose.position, mat, cloud.header.frame_id, 1, id);

  marker_pose.translation[0] = pose_stamped.pose.position.x * 100.0;
  marker_pose.translation[1] = pose_stamped.pose.position.y * 100.0;
  marker_pose.translation[2] = pose_stamped.pose.position.z * 100.0;
  marker_pose.quaternion[1] = pose_stamped.pose.orientation.x;
  marker_pose.quaternion[2] = pose_stamped.pose.orientation.y;
  marker_pose.quaternion[3] = pose_stamped.pose.orientation.z;
  marker_pose.quaternion[0] = pose_stamped.pose.orientation.w;

  return 0;
}

// Infer the master tag corner positons from the other observed tags
// Also does some of the bookkeeping for tracking that MultiMarker::_GetPose does
int MarkerDetectorNode::inferCorners(const ARCloud& cloud, MultiMarkerBundle& multi_marker_bundle, ARCloud& bundle_corners)
{
  bundle_corners.resize(4);
  for (int i = 0; i < 4; i++)
  {
    bundle_corners[i].x = 0;
    bundle_corners[i].y = 0;
    bundle_corners[i].z = 0;
  }

  // Reset the marker_status to 1 for all markers in point_cloud for tracking purposes
  for (size_t i = 0; i < multi_marker_bundle.marker_status.size(); i++)
  {
    if (multi_marker_bundle.marker_status[i] > 0) multi_marker_bundle.marker_status[i] = 1;
  }

  int n_est = 0;

  // For every detected marker
  for (size_t i = 0; i < marker_detector_.markers->size(); i++)
  {
    const Marker* p_marker = &((*marker_detector_.markers)[i]);
    int id = p_marker->GetId();
    int index = multi_marker_bundle.get_id_index(id);
    int mast_id = multi_marker_bundle.master_id;
    if (index < 0) continue;

    // But only if we have corresponding points in the pointcloud
    if (multi_marker_bundle.marker_status[index] > 0 && p_marker->valid)
    {
      n_est++;

      std::string marker_frame = "ar_marker_";
      std::stringstream mark_out;
      mark_out << id;
      std::string id_string = mark_out.str();
      marker_frame += id_string;

      //Grab the precomputed corner coords and correct for the weird Alvar coord system
      for (int j = 0; j < 4; ++j)
      {
        tf::Vector3 corner_coord = multi_marker_bundle.rel_corners[index][j];

        gm::PointStamped point_wrt_alvar, point_wrt_camera;
        point_wrt_alvar.header.frame_id = marker_frame;
        point_wrt_alvar.point.x = corner_coord.y() / 100.0;
        point_wrt_alvar.point.y = -corner_coord.x() / 100.0;
        point_wrt_alvar.point.z = corner_coord.z() / 100.0;

        try
        {
          p_tf_listener_->waitForTransform(cloud.header.frame_id, marker_frame, ros::Time(0), ros::Duration(0.1));
          p_tf_listener_->transformPoint(cloud.header.frame_id, point_wrt_alvar, point_wrt_camera);
        }
        catch (tf::TransformException exception)
        {
          ROS_ERROR("ERROR inferred corners: %s", exception.what());
          return -1;
        }

        bundle_corners[j].x += point_wrt_camera.point.x;
        bundle_corners[j].y += point_wrt_camera.point.y;
        bundle_corners[j].z += point_wrt_camera.point.z;
      }

      multi_marker_bundle.marker_status[index] = 2; // Used for tracking
    }
  }

  // Divide to take the average of the summed estimates
  if (n_est > 0)
  {
    for(int i=0; i<4; i++)
    {
      bundle_corners[i].x /= n_est;
      bundle_corners[i].y /= n_est;
      bundle_corners[i].z /= n_est;
    }
  }

  return 0;
}

void MarkerDetectorNode::broadcastMarkerTf(const std_msgs::Header& image_header, const std::string marker_frame, tf::Transform& pose)
{
  tf::StampedTransform marker_wrt_camera(pose, image_header.stamp, image_header.frame_id, marker_frame);
  p_tf_broadcaster_->sendTransform(marker_wrt_camera);
}

// Debugging utility function
void MarkerDetectorNode::draw3dPoints(ARCloud::Ptr p_cloud, const string& frame, int color, int id, double scale)
{
  visualization_msgs::Marker viz_marker;
  viz_marker.header.frame_id = frame;
  viz_marker.header.stamp = ros::Time::now();
  viz_marker.id = id;
  viz_marker.ns = "3dpts";
  viz_marker.scale.x = scale;
  viz_marker.scale.y = scale;
  viz_marker.scale.z = scale;
  viz_marker.type = visualization_msgs::Marker::SPHERE_LIST;
  viz_marker.action = visualization_msgs::Marker::ADD;
  viz_marker.lifetime = ros::Duration(1.0);

  for(int i=0; i<p_cloud->points.size(); i++)
  {
    gm::Point point;
    point.x = p_cloud->points[i].x;
    point.y = p_cloud->points[i].y;
    point.z = p_cloud->points[i].z;
    viz_marker.points.push_back(point);
  }

  if(color==1)
  {
    viz_marker.color.r = 0.0f;
    viz_marker.color.g = 1.0f;
    viz_marker.color.b = 1.0f;
    viz_marker.color.a = 1.0;
  }
  if(color==2)
  {
    viz_marker.color.r = 1.0f;
    viz_marker.color.g = 0.0f;
    viz_marker.color.b = 1.0f;
    viz_marker.color.a = 1.0;
  }
  if(color==3)
  {
    viz_marker.color.r = 1.0f;
    viz_marker.color.g = 1.0f;
    viz_marker.color.b = 0.0f;
    viz_marker.color.a = 1.0;
  }

  viz_marker_debug_publisher_.publish (viz_marker);
}

void MarkerDetectorNode::drawArrow(gm::Point& arrow_origin, const tf::Matrix3x3& tf_matrix, const string& frame, int color, int id)
{
  visualization_msgs::Marker viz_marker_msg;
  viz_marker_msg.header.frame_id = frame;
  viz_marker_msg.header.stamp = ros::Time::now();
  viz_marker_msg.id = id;
  viz_marker_msg.ns = "arrow";
  viz_marker_msg.scale.x = 0.01;
  viz_marker_msg.scale.y = 0.01;
  viz_marker_msg.scale.z = 0.1;
  viz_marker_msg.type = visualization_msgs::Marker::ARROW;
  viz_marker_msg.action = visualization_msgs::Marker::ADD;
  viz_marker_msg.lifetime = ros::Duration(1.0);

  for (int i = 0; i < 3; i++)
  {
    gm::Point arrow_tip;
    arrow_tip.x = arrow_origin.x + tf_matrix.getRow(0)[i];
    arrow_tip.y = arrow_origin.y + tf_matrix.getRow(1)[i];
    arrow_tip.z = arrow_origin.z + tf_matrix.getRow(2)[i];

    viz_marker_msg.points.clear();
    viz_marker_msg.points.push_back(arrow_origin);
    viz_marker_msg.points.push_back(arrow_tip);
    viz_marker_msg.id += 10 * i;

    if(color == 1)
    {
      viz_marker_msg.color.r = 1.0f;
      viz_marker_msg.color.g = 0.0f;
      viz_marker_msg.color.b = 0.0f;
      viz_marker_msg.color.a = 1.0;
    }
    if(color == 2)
    {
      viz_marker_msg.color.r = 0.0f;
      viz_marker_msg.color.g = 1.0f;
      viz_marker_msg.color.b = 0.0f;
      viz_marker_msg.color.a = 1.0;
    }
    if(color == 3)
    {
      viz_marker_msg.color.r = 0.0f;
      viz_marker_msg.color.g = 0.0f;
      viz_marker_msg.color.b = 1.0f;
      viz_marker_msg.color.a = 1.0;
    }

    color += 1;

    viz_marker_debug_publisher_.publish(viz_marker_msg);
  }
}

int main(int argc, char *argv[])
{
  ros::init (argc, argv, "marker_detect");

  if (argc < EXPECTED_ARGUMENT_COUNT)
  {
    std::cout << std::endl;
    cout << "Not enough arguments provided." << endl;
    cout << "Usage: ./findMarkerBundles <marker size in cm> <max new marker error> <max track error> <cam image topic> <cam info topic> <output frame> <median filt size> <list of bundle XML files...>" << endl;
    std::cout << std::endl;
    return 0;
  }

  try
  {
    MarkerDetectorParameters parameters;
    getParametersFromArguments(argc, argv, parameters);

    MarkerDetectorNode marker_detector_manager(parameters);
    marker_detector_manager.run();
  }
  catch (std::exception& exception)
  {
    ROS_ERROR_STREAM(exception.what());
    ros::shutdown();
  }

  return 0;
}
