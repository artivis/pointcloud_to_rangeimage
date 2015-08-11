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

namespace
{
  typedef pcl::PointXYZ              PointType;
  typedef pcl::PointCloud<PointType> PointCloud;


  typedef pcl::RangeImage          RI;
  typedef pcl::RangeImageSpherical RIS;

  typedef pcl::visualization::RangeImageVisualizer visualizer;

  typedef pointcloud_to_rangeimage::PointCloudToRangeImageReconfigureConfig conf;
  typedef dynamic_reconfigure::Server<conf>               RangeImageReconfServer;
}

void getColorForFloat(float value, unsigned char& r, unsigned char& g, unsigned char& b)
{
  if (std::isinf(value))
  {
    if (value > 0.0f)
    {
      r = 150; g = 150; b = 200; // INFINITY
      return;
    }
    r = 150; g = 200; b = 150; // -INFINITY
    return;
  }
  if (!std::isfinite (value))
  {
    r = 200; g = 150; b = 150; // -INFINITY
    return;
  }
  r = g = b = 0;
  value *= 10;
  if (value <= 1.0)
  { // black -> purple
    b = static_cast<unsigned char> (static_cast<long int>(round((value*200))));
    r = static_cast<unsigned char> (static_cast<long int>(round((value*120))));
  }
  else if (value <= 2.0)
  { // purple -> blue
    b = static_cast<unsigned char> (200 + static_cast<long int>(round((value-1.0)*55)));
    r = static_cast<unsigned char> (120 - static_cast<long int>(round((value-1.0)*120)));
  }
  else if (value <= 3.0)
  { // blue -> turquoise
    b = static_cast<unsigned char> (255 - static_cast<long int>(round((value-2.0)*55)));
    g = static_cast<unsigned char> (static_cast<long int>(round((value-2.0)*200)));
  }
  else if (value <= 4.0)
  { // turquoise -> green
    b = static_cast<unsigned char> (200 - static_cast<long int>(round((value-3.0)*200)));
    g = static_cast<unsigned char> (200 + static_cast<long int>(round((value-3.0)*55)));
  }
  else if (value <= 5.0)
  { // green -> greyish green
    g = static_cast<unsigned char> (255 - static_cast<long int>(round((value-4.0)*100)));
    r = static_cast<unsigned char> (static_cast<long int>(round((value-4.0)*120)));
  }
  else if (value <= 6.0)
  { // greyish green -> red
    r = static_cast<unsigned char> (100 + static_cast<long int>(round((value-5.0)*155)));
    g = static_cast<unsigned char> (120 - static_cast<long int>(round((value-5.0)*120)));
    b = static_cast<unsigned char> (120 - static_cast<long int>(round((value-5.0)*120)));
  }
  else if (value <= 7.0)
  { // red -> yellow
    r = 255;
    g = static_cast<unsigned char> (static_cast<long int>(round((value-6.0)*255)));
  }
  else
  { // yellow -> white
    r = 255;
    g = 255;
    b = static_cast<unsigned char> (static_cast<long int>(round((value-7.0)*255.0/3.0)));
  }
}

class RangeImageConverter
{
private:

  bool _visualize;
  bool _publish;
  bool _laser_frame;

  pcl::RangeImage::CoordinateFrame _frame;

  float _ang_res_x;
  float _ang_res_y;
  float _max_ang_w;
  float _max_ang_h;

  cv::Mat _rangeImage;

  boost::shared_ptr<RIS> rangeImageSph_;

  boost::shared_ptr<visualizer> visualizer_;

  ros::NodeHandle nh_;
  ros::ServiceServer save_;

  ros::Publisher pub_;
  ros::Subscriber sub_;

  boost::shared_ptr<RangeImageReconfServer> drsv_;

public:

  RangeImageConverter() :
    _visualize(true),
    _publish(true),
    _laser_frame(true),
    _ang_res_x(0.1),
    _ang_res_y(0.1),
    _max_ang_w(180.),
    _max_ang_h(180.),
    nh_("~")
  {
    rangeImageSph_ = boost::shared_ptr<RIS>(new RIS);

    visualizer_ = boost::shared_ptr<visualizer>(new visualizer);

    drsv_.reset(new RangeImageReconfServer(ros::NodeHandle("range_image_converter")));

    RangeImageReconfServer::CallbackType cb;
    cb = boost::bind(&RangeImageConverter::drcb, this, _1, _2);

    drsv_->setCallback(cb);

    nh_.param("visualisation", _visualize, _visualize);
    nh_.param("publish", _publish, _publish);
    nh_.param("laser_frame", _laser_frame, _laser_frame);

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
    ros::Time start = ros::Time::now();

    Eigen::Affine3f eigen_sensor_pose;
    eigen_sensor_pose.setIdentity();

    rangeImageSph_->createFromPointCloud(*msg, pcl::deg2rad(_ang_res_x), pcl::deg2rad(_ang_res_y),
                                         pcl::deg2rad(_max_ang_w), pcl::deg2rad(_max_ang_h),
                                         eigen_sensor_pose, _frame, 0.0, 0.0f, 0);

    rangeImageSph_->setUnseenToMaxRange();

    if (_visualize)
      visualizer_->showRangeImage(*rangeImageSph_);

    if (_publish)
      convert();

    //ROS_INFO_STREAM("Process took : " << ros::Time::now() - start << " seconds.");
  }

  void convert()
  {
    int cols = rangeImageSph_->width;
    int rows = rangeImageSph_->height;

    float min_range;
    float max_range;
    rangeImageSph_->getMinMaxRanges(min_range, max_range);

    _rangeImage = cv::Mat::zeros(rows, cols, CV_8UC3);

    float factor = 1.0f / (max_range-min_range);
    float offset = -min_range;

    unsigned char r,g,b;

    for (int i=0; i<cols; ++i)
      for (int j=0; j<rows; ++j)
      {
        pcl::PointWithRange p = rangeImageSph_->getPoint(i, j);

        float range = std::max(0.0f, std::min(1.0f, factor * (p.range + offset)));

        getColorForFloat(range, r, g, b);

        _rangeImage.at<cv::Vec3b>(j, i)[0] = r;
        _rangeImage.at<cv::Vec3b>(j, i)[1] = g;
        _rangeImage.at<cv::Vec3b>(j, i)[2] = b;
      }

    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "rgb8", _rangeImage).toImageMsg();

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
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pointcloud_to_rangeimage");
  ros::NodeHandle nh;

  RangeImageConverter converter;

  ros::spin();
}
