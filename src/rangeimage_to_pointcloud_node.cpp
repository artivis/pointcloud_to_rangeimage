#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <image_transport/image_transport.h>

#include <pcl/point_types.h>
#include <pcl/range_image/range_image_spherical.h>

#include <opencv2/core/core.hpp>

#include <math.h>

#include <boost/thread/mutex.hpp>

#include "pointcloud_to_rangeimage/utils.h"

namespace
{
  typedef pcl::PointXYZ              PointType;
  typedef pcl::PointCloud<PointType> PointCloud;

  typedef pcl::RangeImage          RI;
  typedef pcl::RangeImageSpherical RIS;


  typedef image_transport::ImageTransport It;
  typedef image_transport::Subscriber     Sub;
}

class PointCloudConverter
{
private:

  bool _newmsg;
  bool _laser_frame;
  bool _init;

  // RangeImage frame
  pcl::RangeImage::CoordinateFrame _frame;

  // RangeImage resolution
  float _ang_res_x;
  float _ang_res_y;

  // RangeImage angular FoV
  float _max_ang_w;
  float _max_ang_h;

  // Sensor min/max range
  float _min_range;
  float _max_range;

  boost::mutex _mut;

  cv_bridge::CvImagePtr _rangeImage;

  PointCloud _pointcloud;

  boost::shared_ptr<RIS> rangeImageSph_;

  ros::NodeHandle _nh;

  It              _it;
  ros::Publisher  _pub;
  Sub             _sub;

public:

  PointCloudConverter() :
    _newmsg(false),
    _laser_frame(true),
    _init(false),
    _ang_res_x(0.5),
    _ang_res_y(0.7),
    _max_ang_w(360.),
    _max_ang_h(360.),
    _min_range(0.5),
    _max_range(50),
    _nh("~"),
    _it(_nh)
  {
    rangeImageSph_ = boost::shared_ptr<RIS>(new RIS);

    _nh.param("laser_frame", _laser_frame, _laser_frame);

    double ang_res_x = static_cast<double>(_ang_res_x);
    double ang_res_y = static_cast<double>(_ang_res_y);
    double max_ang_w = static_cast<double>(_max_ang_w);
    double max_ang_h = static_cast<double>(_max_ang_h);
    double min_range = static_cast<double>(_min_range);
    double max_range = static_cast<double>(_max_range);

    _nh.param("ang_res_x", ang_res_x, ang_res_x);
    _nh.param("ang_res_y", ang_res_y, ang_res_y);
    _nh.param("max_ang_w", max_ang_w, max_ang_w);
    _nh.param("max_ang_h", max_ang_h, max_ang_h);
    _nh.param("min_range", min_range, min_range);
    _nh.param("max_range", max_range, max_range);

    _ang_res_x = static_cast<float>(ang_res_x);
    _ang_res_y = static_cast<float>(ang_res_y);
    _max_ang_w = static_cast<float>(max_ang_w);
    _max_ang_h = static_cast<float>(max_ang_h);
    _min_range = static_cast<float>(min_range);
    _max_range = static_cast<float>(max_range);

    _pub = _nh.advertise<sensor_msgs::PointCloud2>("pointcloud_out", 1);

    std::string transport = "raw";
    _nh.param("transport", transport, transport);

    if (transport != "raw" && transport != "compressedDepth")
    {
      ROS_WARN_STREAM("Transport " << transport
                      << ".\nThe only transports supported are :\n\t - raw\n\t - compressedDepth.\n"
                      << "Setting transport default 'raw'.");

      transport = "raw";
    }
    else
      ROS_INFO_STREAM("Transport " << transport);

    image_transport::TransportHints transportHint(transport);

    std::string image_in = "image_in";
    _nh.param("image_in", image_in, image_in);
    _sub = _it.subscribe(image_in, 1, &PointCloudConverter::callback, this, transportHint);

    _frame = (_laser_frame)? pcl::RangeImage::LASER_FRAME : pcl::RangeImage::CAMERA_FRAME;
  }

  ~PointCloudConverter()
  {

  }

  void init()
  {
    rangeImageSph_->createEmpty(pcl::deg2rad(_ang_res_x), pcl::deg2rad(_ang_res_y),
                                Eigen::Affine3f::Identity(), _frame,
                                pcl::deg2rad(_max_ang_w), pcl::deg2rad(_max_ang_h));
    _init = true;
  }

  void callback(const sensor_msgs::ImageConstPtr& msg)
  {
    if (msg == NULL) return;

    boost::mutex::scoped_lock lock(_mut);

    try
    {
      _rangeImage = cv_bridge::toCvCopy(msg, msg->encoding);
      pcl_conversions::toPCL(msg->header, _pointcloud.header);
      _newmsg = true;
    }
    catch (cv_bridge::Exception &e)
    {
      ROS_WARN_STREAM(e.what());
    }
  }

  void convert()
  {
    // What the point if nobody cares ?
    if (_pub.getNumSubscribers() <= 0)
      return;

    if (_rangeImage == NULL)
      return;

    if (!_newmsg)
      return;

    if (!_init)
      init();

    _pointcloud.clear();

    float factor = 1.0f / (_max_range - _min_range);
    float offset = -_min_range;

    _mut.lock();

    int cols = _rangeImage->image.cols;
    int rows = _rangeImage->image.rows;

    int top    = rows;
    int right  = -1;
    int bottom = -1;
    int left   = cols;

    if (_rangeImage->encoding == "bgr8")
    {
      for (int i=0; i<cols; ++i)
      {
        for (int j=0; j<rows; ++j)
        {
          uchar r = _rangeImage->image.at<cv::Vec3b>(j, i)[0];
          uchar g = _rangeImage->image.at<cv::Vec3b>(j, i)[1];
          uchar b = _rangeImage->image.at<cv::Vec3b>(j, i)[2];

          ushort range_short;

          getRangeFromFalseColor2(r, g, b, range_short);

          if (range_short == 0.) continue;

          // Rescale range
          float range = static_cast<float>(range_short) /
              static_cast<float>(std::numeric_limits<ushort>::max());

          range = (range - offset*factor) / factor;

          pcl::PointWithRange& p = rangeImageSph_->getPointNoCheck(i, j);

          p.range = range;

          top    = std::min(top,    j);
          right  = std::max(right,  i);
          bottom = std::max(bottom, j);
          left   = std::min(left,   i);
        }
      }
    }
    else if (_rangeImage->encoding == "mono16")
    {
      for (int i=0; i<cols; ++i)
      {
        for (int j=0; j<rows; ++j)
        {
          ushort range_img = _rangeImage->image.at<ushort>(j, i);

          // Discard unobserved points
          if (range_img == 0.) continue;

          // Rescale range
          float range = static_cast<float>(range_img) /
                        static_cast<float>(std::numeric_limits<ushort>::max());

          range = (range - offset*factor) / factor;

          pcl::PointWithRange& p = rangeImageSph_->getPointNoCheck(i, j);

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
      ROS_ERROR("Unknown image encoding!");
      _mut.unlock();
      return;
    }

    int offset_x = rangeImageSph_->getImageOffsetX();
    int offset_y = rangeImageSph_->getImageOffsetY();

    std::string off_x_res, off_y_res;
    if (_nh.searchParam("/pointcloud_to_rangeimage/range_image_offset_x", off_x_res))
      _nh.param(off_x_res, offset_x, offset_x);
    else
      ROS_WARN_ONCE("Couldn't find param 'range_image_offset_x'. Use Default value.");

    if (_nh.searchParam("/pointcloud_to_rangeimage/range_image_offset_y", off_y_res))
      _nh.param(off_y_res, offset_y, offset_y);
    else
      ROS_WARN_ONCE("Couldn't find param 'range_image_offset_y'. Use Default value.");

    rangeImageSph_->cropImage(0, top, right, bottom, left);

    rangeImageSph_->setImageOffsets(offset_x, offset_y);

    rangeImageSph_->recalculate3DPointPositions();

    for (int i=0; i<rangeImageSph_->points.size(); ++i)
    {
      pcl::PointWithRange& pts = rangeImageSph_->points[i];

      // Discard unobserved points
      if (std::isinf(pts.range))
        continue;

      PointType p(pts.x, pts.y, pts.z);

      _pointcloud.push_back(p);
    }

    _pub.publish(_pointcloud);

    _newmsg = false;

    _mut.unlock();
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
