#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/point_types.h>
#include <pcl/range_image/range_image_spherical.h>

#include <opencv2/core/core.hpp>

#include <math.h>

#include <boost/thread/mutex.hpp>

#include "pointcloud_to_rangeimage/RangeImage.h"
#include "pointcloud_to_rangeimage/utils.h"

namespace
{
  typedef pcl::PointXYZ              PointType;
  typedef pcl::PointCloud<PointType> PointCloud;
  typedef PointCloud::ConstPtr       PointCloudConstPtr;

  typedef pcl::RangeImageSpherical      RangeImage;
  typedef boost::shared_ptr<RangeImage> RangeImagePtr;

  typedef pointcloud_to_rangeimage::RangeImageHeader   RangeImageHeader;
  typedef pointcloud_to_rangeimage::RangeImage         RangeImageMsg;
  typedef pointcloud_to_rangeimage::RangeImageConstPtr RangeImageMsgConstPtr;

  const Eigen::Affine3f I = Eigen::Affine3f::Identity();

  const ushort MAX_USHORT   = std::numeric_limits<ushort>::max();
  const float  MAX_USHORT_F = static_cast<float>(MAX_USHORT);
}

class PointCloudConverter
{
private:

  bool _newmsg;
  bool _laser_frame;
  bool _init;

  // RangeImage frame
  pcl::RangeImage::CoordinateFrame _frame;

  boost::mutex _mut;

  RangeImageMsgConstPtr _range_image_msg_ptr;
  cv_bridge::CvImagePtr _cv_image_ptr;

  PointCloud _pointcloud;

  RangeImagePtr _range_image_ptr;

  ros::NodeHandle _nh;

  ros::Publisher  _pub;
  ros::Subscriber _sub;

public:

  PointCloudConverter() :
    _newmsg(false),
    _laser_frame(true),
    _init(false),
    _range_image_ptr(new RangeImage),
    _nh("~")
  {
    _pub = _nh.advertise<sensor_msgs::PointCloud2>("pointcloud_out", 1);

    _sub = _nh.subscribe("image_in", 1, &PointCloudConverter::callback, this);

    _frame = (_laser_frame)? pcl::RangeImage::LASER_FRAME : pcl::RangeImage::CAMERA_FRAME;
  }

  ~PointCloudConverter()
  {

  }

  void createRangeImage()
  {
    _range_image_ptr->createEmpty(pcl::deg2rad(_range_image_msg_ptr->Specifics.ang_res_x),
                                  pcl::deg2rad(_range_image_msg_ptr->Specifics.ang_res_y),
                                  I, _frame,
                                  pcl::deg2rad(_range_image_msg_ptr->Specifics.max_ang_w),
                                  pcl::deg2rad(_range_image_msg_ptr->Specifics.max_ang_h));

    _range_image_ptr->setImageOffsets(_range_image_msg_ptr->Specifics.offset_x,
                                      _range_image_msg_ptr->Specifics.offset_y);
  }

  void callback(const RangeImageMsgConstPtr& msg)
  {
    if (msg == NULL) return;

    boost::mutex::scoped_lock lock(_mut);

    _range_image_msg_ptr = msg;
  }

  void convert()
  {
    // What the point if nobody cares ?
    if (_pub.getNumSubscribers() == 0)
      return;

    if (_newmsg || _range_image_msg_ptr == NULL)
      return;

    _pointcloud.clear();

    _mut.lock();

    createRangeImage();

    float factor = 1.0f / (_range_image_msg_ptr->Specifics.max_range -
                           _range_image_msg_ptr->Specifics.min_range);

    float inv_factor = 1.0f / factor;

    float offset = - _range_image_msg_ptr->Specifics.min_range;

    float offset_factor = offset * factor;

    try
    {
      _cv_image_ptr = cv_bridge::toCvCopy(_range_image_msg_ptr->Image,
                                          _range_image_msg_ptr->Image.encoding);

      pcl_conversions::toPCL(_range_image_msg_ptr->Image.header, _pointcloud.header);
    }
    catch (cv_bridge::Exception &e)
    {
      ROS_WARN_STREAM(e.what());
    }

    // CvCopy, we can unlock
    _mut.unlock();

    int cols = _cv_image_ptr->image.cols;
    int rows = _cv_image_ptr->image.rows;

    int top    = rows;
    int right  = -1;
    int bottom = -1;
    int left   = cols;

    if (_cv_image_ptr->encoding == "bgr8")
    {
      for (int i=0; i<cols; ++i)
      {
        for (int j=0; j<rows; ++j)
        {
          cv::Vec3b rgb = _cv_image_ptr->image.at<cv::Vec3b>(j, i);

          ushort range_short;

          getRangeFromFalseColor2(rgb[0], rgb[0], rgb[0], range_short);

          if (range_short == 0.) continue;

          // Rescale range
          float range = static_cast<float>(range_short) / MAX_USHORT_F;

          range = (range - offset_factor) * inv_factor;

          pcl::PointWithRange& p = _range_image_ptr->getPointNoCheck(i, j);

          p.range = range;

          top    = std::min(top,    j);
          right  = std::max(right,  i);
          bottom = std::max(bottom, j);
          left   = std::min(left,   i);
        }
      }
    }
    else if (_cv_image_ptr->encoding == "mono16")
    {
      for (int i=0; i<cols; ++i)
      {
        for (int j=0; j<rows; ++j)
        {
          ushort range_img = _cv_image_ptr->image.at<ushort>(j, i);

          // Discard unobserved points
          if (range_img == 0.) continue;

          // Rescale range
          float range = static_cast<float>(range_img) / MAX_USHORT_F;

          range = (range - offset_factor) * inv_factor;

          pcl::PointWithRange& p = _range_image_ptr->getPointNoCheck(i, j);

          p.range = range;

          top    = std::min(top,    j);
          right  = std::max(right,  i);
          bottom = std::max(bottom, j);
          left   = std::min(left,   i);
        }
      }
    }
    else
    {
      ROS_ERROR_THROTTLE(2, "Unknown image encoding!");
      _newmsg = false;
      return;
    }

    _range_image_ptr->cropImage(0, top, right, bottom, left);

    _range_image_ptr->recalculate3DPointPositions();

    unsigned int num_pts = _range_image_ptr->points.size();

    /** \todo there is probably a way to convert in pcl */
    for (int i=0; i<num_pts; ++i)
    {
      pcl::PointWithRange& pts = _range_image_ptr->points[i];

      // Discard unobserved points
      if (std::isinf(pts.range)) continue;

      PointType p(pts.x, pts.y, pts.z);

      _pointcloud.push_back(p);
    }

    _pub.publish(_pointcloud);

    _newmsg = false;
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "rangeimage_to_pointcloud");

  PointCloudConverter converter;

  ros::Rate rate(15);

  while (ros::ok())
  {
    converter.convert();

    ros::spinOnce();

    rate.sleep();
  }
}
