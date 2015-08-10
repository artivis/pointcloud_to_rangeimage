#include <ros/ros.h>
#include <ros/package.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include <dynamic_reconfigure/server.h>
#include <pointcloud_to_rangeimage/PointCloudToRangeImageReconfigureConfig.h>

#include <pcl/point_types.h>
#include <pcl/range_image/range_image.h>
#include <pcl/range_image/range_image_spherical.h>
#include <pcl/visualization/range_image_visualizer.h>
//#include <pcl/io/pcd_io.h>

//#include <iterator>
//#include <boost/foreach.hpp>
//#include <iostream>

#include <std_srvs/Empty.h>

namespace
{
  typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
}

class RangeImageConverter
{
  typedef pcl::PointXYZ PointType;
  typedef pcl::RangeImageSpherical RIS;
  typedef pcl::visualization::RangeImageVisualizer visualizer;

  typedef pointcloud_to_rangeimage::PointCloudToRangeImageReconfigureConfig conf;
  typedef dynamic_reconfigure::Server<conf>            RangeImageReconfServer;

private:

  float _ang_res_x;
  float _ang_res_y;
  float _max_ang_w;
  float _max_ang_h;

  boost::shared_ptr<RIS> rangeImageSph_;

  boost::shared_ptr<visualizer> visualizer_;

  ros::NodeHandle nh_;
  ros::ServiceServer save_;

  boost::shared_ptr<RangeImageReconfServer> drsv_;

public:

  RangeImageConverter() :
    nh_("~"),
    _ang_res_x(0.1),
    _ang_res_y(0.1),
    _max_ang_w(180.),
    _max_ang_h(180.)
  {
    save_ = nh_.advertiseService("save", &RangeImageConverter::save, this);

    rangeImageSph_ = boost::shared_ptr<RIS>(new RIS);

    visualizer_ = boost::shared_ptr<visualizer>(new visualizer);

    drsv_.reset(new RangeImageReconfServer(ros::NodeHandle("range_image_converter")));

    RangeImageReconfServer::CallbackType cb;
    cb = boost::bind(&RangeImageConverter::drcb, this, _1, _2);

    drsv_->setCallback(cb);
  }

  ~RangeImageConverter()
  {

  }

  bool save(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
  {
//    std::string filename = ros::package::getPath("range_image_converter") + "/images";
    std::string filename = "/home/jeremiederay/Navigation/RangeImageSpherical/image.png";
    std::cout << "Saving to " << filename << std::endl;

    ros::Duration(2.0).sleep();

    return true;
  }

  void callback(const PointCloud::ConstPtr& msg)
  {
    ros::Time start = ros::Time::now();

    Eigen::Affine3f eigen_sensor_pose;
    eigen_sensor_pose.setIdentity();

    rangeImageSph_->createFromPointCloud(*msg, pcl::deg2rad(_ang_res_x), pcl::deg2rad(_ang_res_y),
                                         pcl::deg2rad(_max_ang_w), pcl::deg2rad(_max_ang_h),
                                         eigen_sensor_pose, pcl::RangeImage::LASER_FRAME, 0.0, 0.0f, 0);

    rangeImageSph_->setUnseenToMaxRange();

    visualizer_->showRangeImage(*rangeImageSph_);

    //ROS_INFO_STREAM("Process took : " << ros::Time::now() - start << " seconds.");
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

  ros::Subscriber sub = nh.subscribe<PointCloud>("point_cloud_in", 1, &RangeImageConverter::callback, &converter);

  ros::spin();
}
