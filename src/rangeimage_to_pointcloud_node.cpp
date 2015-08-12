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

class PointCloudConverter
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

  float _min_range;
  float _max_range;

  cv_bridge::CvImagePtr _rangeImage;

  PointCloud _pointcloud;

  boost::shared_ptr<RIS> rangeImageSph_;

  ros::NodeHandle nh_;
  ros::ServiceServer save_;

  ros::Publisher  pub_;
  ros::Subscriber sub_;

  Eigen::Affine3f       _eigen_transform;

  boost::shared_ptr<RangeImageReconfServer> drsv_;

public:

  PointCloudConverter() :
    _visualize(true),
    _publish(true),
    _laser_frame(true),
    _ang_res_x(0.1),
    _ang_res_y(0.1),
    _max_ang_w(180.),
    _max_ang_h(180.),
    _min_range(0.5),
    _max_range(50),
    nh_("~")
  {
    rangeImageSph_ = boost::shared_ptr<RIS>(new RIS);

    drsv_.reset(new RangeImageReconfServer(ros::NodeHandle("range_image_converter")));

    RangeImageReconfServer::CallbackType cb;
    cb = boost::bind(&PointCloudConverter::drcb, this, _1, _2);

    drsv_->setCallback(cb);

    nh_.param("laser_frame", _laser_frame, _laser_frame);

    pub_ = nh_.advertise<sensor_msgs::PointCloud2>("pointcloud_out", 1);

    ros::NodeHandle nh;
    sub_ = nh.subscribe<sensor_msgs::Image>("image_in", 1, &PointCloudConverter::callback, this);

    _frame = (_laser_frame)? pcl::RangeImage::LASER_FRAME : pcl::RangeImage::CAMERA_FRAME;
  }

  ~PointCloudConverter()
  {

  }

  void callback(const sensor_msgs::ImageConstPtr& msg)
  {
    rangeImageSph_->createEmpty(pcl::deg2rad(_ang_res_x), pcl::deg2rad(_ang_res_y),
                                Eigen::Affine3f::Identity(), _frame,
                                pcl::deg2rad(_max_ang_w), pcl::deg2rad(_max_ang_h));

    _pointcloud.clear();

    _rangeImage = cv_bridge::toCvCopy(msg, msg->encoding);

    pcl_conversions::toPCL(msg->header, _pointcloud.header);

    float factor = 1.0f / (_max_range - _min_range);
    float offset = -_min_range;

    int cols = _rangeImage->image.cols;
    int rows = _rangeImage->image.rows;

    if (msg->encoding == "bgr8")
    {
      for (int i=0; i<cols; ++i)
        for (int j=0; j<rows; ++j)
        {
          //float range = _rangeImage->image.at<cv::Vec3b>(j,i)[0];

          //rangeImageSph_->calculate3DPoint(i, j, range, pts);

          //PointType p(pts[0], pts[1], pts[2]);

          //_pointcloud.push_back(p);
        }
    }
    else if (msg->encoding == "mono16")
    {
      for (int i=0; i<cols; ++i)
        for (int j=0; j<rows; ++j)
        {
          ushort range_img = _rangeImage->image.at<ushort>(j, i);

          if (range_img == 0) continue;

          float range = static_cast<float>(range_img) /
                        static_cast<float>(std::numeric_limits<ushort>::max());

          range = (range - offset*factor) / factor;

          pcl::PointWithRange& p = rangeImageSph_->getPoint(i, j);
          p.range = range;
        }

      rangeImageSph_->recalculate3DPointPositions();

      for (int i=0; i<rangeImageSph_->points.size(); ++i)
      {
        pcl::PointWithRange pts = rangeImageSph_->points.at(i);
        PointType p(pts.x, pts.y, pts.z);

        _pointcloud.push_back(p);
      }
    }

    if (_publish)
      pub_.publish(_pointcloud);
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
  ros::init(argc, argv, "rangeimage_to_pointcloud");

  PointCloudConverter converter;

  ros::spin();
}
