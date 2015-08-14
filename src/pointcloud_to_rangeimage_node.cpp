#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include <dynamic_reconfigure/server.h>
#include <pointcloud_to_rangeimage/PointCloudToRangeImageReconfigureConfig.h>

#include <pcl/point_types.h>
#include <pcl/range_image/range_image.h>
#include <pcl/range_image/range_image_spherical.h>
#include <pcl/visualization/range_image_visualizer.h>

#include <opencv2/core/core.hpp>

#include <math.h>

#include "pointcloud_to_rangeimage/utils.h"

namespace
{
  typedef pcl::PointXYZ              PointType;
  typedef pcl::PointCloud<PointType> PointCloud;

  typedef pcl::RangeImage          RI;
  typedef pcl::RangeImageSpherical RIS;

  typedef pointcloud_to_rangeimage::PointCloudToRangeImageReconfigureConfig conf;
  typedef dynamic_reconfigure::Server<conf>               RangeImageReconfServer;
}

class RangeImageConverter
{
private:

  bool _visualize;
  bool _publish;
  bool _rgb_range_img;
  bool _laser_frame;

  pcl::RangeImage::CoordinateFrame _frame;

  float _ang_res_x;
  float _ang_res_y;
  float _max_ang_w;
  float _max_ang_h;

  float _min_range;
  float _max_range;

  cv::Mat _rangeImage;

  boost::shared_ptr<RIS> rangeImageSph_;

  ros::NodeHandle nh_;

  ros::Publisher  pub_;
  ros::Subscriber sub_;

  boost::shared_ptr<RangeImageReconfServer> drsv_;

public:

  RangeImageConverter() :
    _rgb_range_img(true),
    _laser_frame(true),
    _ang_res_x(0.5),
    _ang_res_y(0.5),
    _max_ang_w(360.),
    _max_ang_h(360.),
    _min_range(0.5),
    _max_range(50),
    nh_("~")
  {
    rangeImageSph_ = boost::shared_ptr<RIS>(new RIS);

    drsv_.reset(new RangeImageReconfServer(ros::NodeHandle("range_image_converter")));

    RangeImageReconfServer::CallbackType cb;
    cb = boost::bind(&RangeImageConverter::drcb, this, _1, _2);

    drsv_->setCallback(cb);

    nh_.param("rgb_range_img", _rgb_range_img, _rgb_range_img);
    nh_.param("laser_frame",   _laser_frame, _laser_frame);

    double ang_res_x = static_cast<double>(_ang_res_x);
    double ang_res_y = static_cast<double>(_ang_res_y);
    double max_ang_w = static_cast<double>(_max_ang_w);
    double max_ang_h = static_cast<double>(_max_ang_h);
    double min_range = static_cast<double>(_min_range);
    double max_range = static_cast<double>(_max_range);

    nh_.param("ang_res_x", ang_res_x, ang_res_x);
    nh_.param("ang_res_y", ang_res_y, ang_res_y);
    nh_.param("max_ang_w", max_ang_w, max_ang_w);
    nh_.param("max_ang_h", max_ang_h, max_ang_h);
    nh_.param("min_range", min_range, min_range);
    nh_.param("max_range", max_range, max_range);

    _ang_res_x = static_cast<float>(ang_res_x);
    _ang_res_y = static_cast<float>(ang_res_y);
    _max_ang_w = static_cast<float>(max_ang_w);
    _max_ang_h = static_cast<float>(max_ang_h);
    _min_range = static_cast<float>(min_range);
    _max_range = static_cast<float>(max_range);

    pub_ = nh_.advertise<sensor_msgs::Image>("image_out", 1);

    ros::NodeHandle nh;
    sub_ = nh.subscribe<PointCloud>("point_cloud_in", 1, &RangeImageConverter::callback, this);

    _frame = (_laser_frame)? pcl::RangeImage::LASER_FRAME : pcl::RangeImage::CAMERA_FRAME;
  }

  ~RangeImageConverter()
  {

  }

  void callback(const PointCloud::ConstPtr& msg)
  {
    if (msg == NULL) return;

    rangeImageSph_->createFromPointCloud(*msg, pcl::deg2rad(_ang_res_x), pcl::deg2rad(_ang_res_y),
                                         pcl::deg2rad(_max_ang_w), pcl::deg2rad(_max_ang_h),
                                         Eigen::Affine3f::Identity(), _frame, 0.0, 0.0f, 0);

    rangeImageSph_->header.frame_id = msg->header.frame_id;
    rangeImageSph_->header.stamp = msg->header.stamp;

    convert();
  }

  void convert()
  {
    if (pub_.getNumSubscribers() < 0) return;

    int cols = rangeImageSph_->width;
    int rows = rangeImageSph_->height;

    sensor_msgs::ImagePtr msg;

    float factor = 1.0f / (_max_range - _min_range);
    float offset = -_min_range;

    std::string encoding;

    if (_rgb_range_img)
    {
      encoding = "rgb8";
      _rangeImage = cv::Mat::zeros(rows, cols, cv_bridge::getCvType(encoding));

      unsigned char r,g,b;

      for (int i=0; i<cols; ++i)
        for (int j=0; j<rows; ++j)
        {
          pcl::PointWithRange p = rangeImageSph_->getPoint(i, j);

          float range = std::max(0.0f, std::min(1.0f, factor * (p.range + offset)));

          getColorFromRange(range, r, g, b);

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
        for (int j=0; j<rows; ++j)
        {
          float r = rangeImageSph_->getPoint(i, j).range;

          float range = (!std::isinf(r))?
                std::max(0.0f, std::min(1.0f, factor * (r + offset))) :
                0.0;

          _rangeImage.at<ushort>(j, i) = static_cast<ushort>((range) * std::numeric_limits<ushort>::max());
        }
    }

    msg = cv_bridge::CvImage(std_msgs::Header(), encoding, _rangeImage).toImageMsg();

    pcl_conversions::fromPCL(rangeImageSph_->header, msg->header);

    pub_.publish(msg);
  }

private:

  void drcb(conf &config, uint32_t level)
  {
    _ang_res_x = config.ang_res_x;
    _ang_res_y = config.ang_res_y;
    _max_ang_w = config.max_ang_w;
    _max_ang_h = config.max_ang_h;

   ROS_INFO_STREAM("ang_res_x " << _ang_res_x);
   ROS_INFO_STREAM("ang_res_y " << _ang_res_x);
   ROS_INFO_STREAM("max_ang_w " << _max_ang_w);
   ROS_INFO_STREAM("max_ang_h " << _max_ang_h);

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

  ros::spin();
}
