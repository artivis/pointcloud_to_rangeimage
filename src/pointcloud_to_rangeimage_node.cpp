#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <image_transport/image_transport.h>

#include <dynamic_reconfigure/server.h>
#include <pointcloud_to_rangeimage/PointCloudToRangeImageReconfigureConfig.h>

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

  typedef pcl::RangeImage               RangeImage;
  typedef boost::shared_ptr<RangeImage> RangeImagePtr;

  typedef pointcloud_to_rangeimage::PointCloudToRangeImageReconfigureConfig Conf;
  typedef dynamic_reconfigure::Server<Conf>               RangeImageReconfServer;

  typedef image_transport::ImageTransport It;
  typedef image_transport::Publisher      Pub;

  typedef pointcloud_to_rangeimage::RangeImageHeader RangeImageHeader;
  typedef pointcloud_to_rangeimage::RangeImage       RangeImageMsg;

  const Eigen::Affine3f I = Eigen::Affine3f::Identity();
}

class RangeImageConverter
{
private:

  bool _newmsg;
  bool _rgb_range_img;
  bool _laser_frame;

  // RangeImage frame
  pcl::RangeImage::CoordinateFrame _frame;

  boost::mutex _mut;

  cv::Mat            _rangeImage;
  PointCloudConstPtr _pointcloud_ptr;

  RangeImagePtr _range_image_ptr;

  RangeImageHeader _header;

  ros::NodeHandle _nh;

  It              _it;
  Pub             _pub;
  ros::Subscriber _sub;

  boost::shared_ptr<RangeImageReconfServer> _drsv;

public:

  RangeImageConverter() :
    _newmsg(false),
    _rgb_range_img(false),
    _laser_frame(true),
    _range_image_ptr(new RangeImage),
    _nh("~"),
    _it(_nh)
  {
    _drsv.reset(new RangeImageReconfServer(ros::NodeHandle("pointcloud_to_rangeimage_dynreconf")));

    RangeImageReconfServer::CallbackType cb;
    cb = boost::bind(&RangeImageConverter::drcb, this, _1, _2);

    _drsv->setCallback(cb);

    double ang_res_x, ang_res_y,
           max_ang_w, max_ang_h,
           min_range, max_range;

    bool got_all_param = false;
    do
    {
      got_all_param = _nh.getParam("ang_res_x", ang_res_x) &&
                      _nh.getParam("ang_res_y", ang_res_y) &&
                      _nh.getParam("max_ang_w", max_ang_w) &&
                      _nh.getParam("max_ang_h", max_ang_h) &&
                      _nh.getParam("min_range", min_range) &&
                      _nh.getParam("max_range", max_range) &&
                      _nh.getParam("rgb_range_img", _rgb_range_img) &&
                      _nh.getParam("laser_frame", _laser_frame);

      ROS_INFO_THROTTLE(1, "Waiting for Range Image parameters to be uploaded to the param server.");

    } while (!got_all_param && ros::ok());

    _header.ang_res_x = static_cast<float>(ang_res_x);
    _header.ang_res_y = static_cast<float>(ang_res_y);

    _header.max_ang_w = static_cast<float>(max_ang_w);
    _header.max_ang_h = static_cast<float>(max_ang_h);

    _header.min_range = static_cast<float>(min_range);
    _header.max_range = static_cast<float>(max_range);

    _header.laser_frame = _laser_frame;

    _frame = (_laser_frame)? pcl::RangeImage::LASER_FRAME : pcl::RangeImage::CAMERA_FRAME;

    _pub = _it.advertise("image_out", 1);

    ros::NodeHandle nh;
    _sub = nh.subscribe<PointCloud>("point_cloud_in", 1, &RangeImageConverter::callback, this);
  }

  ~RangeImageConverter()
  {

  }

  void callback(const PointCloud::ConstPtr& msg)
  {
    if (msg == NULL) return;

    boost::mutex::scoped_lock(_mut);

    _pointcloud_ptr = msg;

    _newmsg = true;
  }

  void convert()
  {
    // What the point if nobody cares ?
    if (_pub.getNumSubscribers() <= 0)
      return;

    if (!_newmsg)
      return;

    boost::mutex::scoped_lock(_mut);

    _range_image_ptr->createFromPointCloud(*_pointcloud_ptr,
                                         pcl::deg2rad(_header.ang_res_x), pcl::deg2rad(_header.ang_res_y),
                                         pcl::deg2rad(_header.max_ang_w), pcl::deg2rad(_header.max_ang_h),
                                         I, _frame, 0.0, 0.0f, 0);

    _range_image_ptr->header.frame_id = _pointcloud_ptr->header.frame_id;
    _range_image_ptr->header.stamp    = _pointcloud_ptr->header.stamp;

    int cols = _range_image_ptr->width;
    int rows = _range_image_ptr->height;

    sensor_msgs::ImagePtr msg;

    float factor = 1.0f / (_header.max_range - _header.min_range);
    float offset = -_header.min_range;

    std::string encoding;

    if (_rgb_range_img)
    {
      encoding = "bgr8";
      _rangeImage = cv::Mat::zeros(rows, cols, cv_bridge::getCvType(encoding));

      unsigned char r,g,b;

      for (int i=0; i<cols; ++i)
        for (int j=0; j<rows; ++j)
        {
          pcl::PointWithRange p = _range_image_ptr->getPoint(i, j);

          float range = std::max(0.0f, std::min(1.0f, factor * (p.range + offset)));

          ushort range_short = static_cast<ushort>((range) * std::numeric_limits<ushort>::max());

          getFalseColorFromRange2(range_short, r, g, b);

          _rangeImage.at<cv::Vec3b>(j, i)[0] = r;
          _rangeImage.at<cv::Vec3b>(j, i)[1] = g;
          _rangeImage.at<cv::Vec3b>(j, i)[2] = b;
        }
    }
    else
    {
      encoding = "mono16";
      _rangeImage = cv::Mat::zeros(rows, cols, cv_bridge::getCvType(encoding));

      for (int i=0; i<cols; ++i)
      {
        for (int j=0; j<rows; ++j)
        {
          float r = _range_image_ptr->getPoint(i, j).range;

          float range = (!std::isinf(r))?
                std::max(0.0f, std::min(1.0f, factor * (r + offset))) :
                0.0;

          _rangeImage.at<ushort>(j, i) = static_cast<ushort>((range) * std::numeric_limits<ushort>::max());
        }
      }
    }

    msg = cv_bridge::CvImage(std_msgs::Header(), encoding, _rangeImage).toImageMsg();

    pcl_conversions::fromPCL(_range_image_ptr->header, msg->header);

    /** \todo : check if they actually change from _header. */
    float min_range, max_range;
    _range_image_ptr->getMinMaxRanges(min_range, max_range);

    RangeImageMsg range_image_msg;

    range_image_msg.Specifics.laser_frame = static_cast<int>(_frame);

    range_image_msg.Specifics = _header;

    range_image_msg.Specifics.min_range = min_range;
    range_image_msg.Specifics.max_range = max_range;

    range_image_msg.Specifics.ang_res_x = _range_image_ptr->getImageOffsetX();
    range_image_msg.Specifics.ang_res_x = _range_image_ptr->getImageOffsetY();

    range_image_msg.Image = *msg;

    _pub.publish(msg);

    // I need an image_transport_plugin to do that, crap!
    //_pub.publish(range_image_msg);

    _newmsg = false;
  }

private:

  void drcb(Conf &config, uint32_t level)
  {
    _header.ang_res_x = config.ang_res_x;
    _header.ang_res_y = config.ang_res_y;

    _header.max_ang_w = config.max_ang_w;
    _header.max_ang_h = config.max_ang_h;

    _header.min_range = config.min_range;
    _header.max_range = config.max_range;

    _header.laser_frame = config.laser_frame;

    _frame = (_laser_frame)? pcl::RangeImage::LASER_FRAME : pcl::RangeImage::CAMERA_FRAME;

   ROS_INFO_STREAM("RangeImageHeader is now set as:\n " << _header);

   if (_laser_frame)
     ROS_INFO_STREAM("Frame type : " << "LASER");
   else
     ROS_INFO_STREAM("Frame type : " << "CAMERA");
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pointcloud_to_rangeimage");

  RangeImageConverter converter;

  ros::Rate rate(15);

  while (ros::ok())
  {
    converter.convert();

    ros::spinOnce();

    rate.sleep();
  }
}
