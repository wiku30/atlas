#include <mutex>
#include <memory>
#include <iostream>

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_broadcaster.h>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <std_msgs/Time.h>
#include <nav_msgs/Odometry.h>
#include <nmea_msgs/Sentence.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/PointCloud2.h>
#include <geographic_msgs/GeoPointStamped.h>
#include <visualization_msgs/MarkerArray.h>



#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include <pcl/filters/voxel_grid.h>

#include <pclomp/ndt_omp.h>

#include <hdl_localization/pose_estimator.hpp>

#include <vector>

#include <geodesy/utm.h>
#include <geodesy/wgs84.h>

#include "conversion.h"
#include "atlas.h"
#include "kdtree.h"

using namespace std;


namespace hdl_localization {

class GlobalmapServerNodelet : public nodelet::Nodelet {
public:
  using PointT = pcl::PointXYZI;

  v3 now_gps;

  Eigen::Vector3d map_utm;

  Eigen::Vector3d xyz;

  geodesy::UTMPoint utm;

  geographic_msgs::GeoPointStampedPtr gps_msg;

  int map_num;

  atlas Atl;
  std::vector<kdt> trees;

  string map_dir;
  vector<string> map_names;

  zone now_zone;

  submap sub_map;
  
  bool single_map;

  GlobalmapServerNodelet() {
  }
  virtual ~GlobalmapServerNodelet() {
  }

  void onInit() override {
    nh = getNodeHandle();
    mt_nh = getMTNodeHandle();
    private_nh = getPrivateNodeHandle();

    gps_msg = geographic_msgs::GeoPointStampedPtr(new geographic_msgs::GeoPointStamped());

    initialize_params();

    // publish globalmap with "latched" publisher
    globalmap_pub = nh.advertise<sensor_msgs::PointCloud2>("/globalmap", 5, true);
    globalmap_pub.publish(globalmap);

    gps_sub = nh.subscribe("/ublox_gps_node/fix", 1024, &GlobalmapServerNodelet::gps_callback, this);
  }

private:



  void initialize_params() {
    // read globalmap from a pcd file
    std::string globalmap_pcd = private_nh.param<std::string>("globalmap_pcd", "");
    globalmap.reset(new pcl::PointCloud<PointT>());
    pcl::io::loadPCDFile(globalmap_pcd, *globalmap);
    globalmap->header.frame_id = "map";


    std::string atlas_dir = private_nh.param<std::string>("atlas_dir", "");

    // downsample globalmap
    double downsample_resolution = private_nh.param<double>("downsample_resolution", 0.1);
    boost::shared_ptr<pcl::VoxelGrid<PointT>> voxelgrid(new pcl::VoxelGrid<PointT>());
    voxelgrid->setLeafSize(downsample_resolution, downsample_resolution, downsample_resolution);
    voxelgrid->setInputCloud(globalmap);

    pcl::PointCloud<PointT>::Ptr filtered(new pcl::PointCloud<PointT>());
    voxelgrid->filter(*filtered);

    globalmap = filtered;

    std::string utm_path = private_nh.param<std::string>("globalmap_utm", "");
    std::cout << "globalmap UTM coordinate path:" << utm_path << std::endl;

    std::ifstream inf(utm_path);

    single_map = private_nh.param<bool>("single_map", "false");

    std::cout << "single map? " << single_map << std::endl;




    if (inf)
    {
        double x, y, z;
        inf >> x >> y >> z;
        map_utm = Eigen::Vector3d(x, y, z);

        std::cout << "globalmap UTM coordinate: " << std::fixed << std::setprecision(2) << x << " " << y << " " << z << std::endl;
    }
    else
    {
        map_utm = Eigen::Vector3d();
    }

    std::string atlas_trees_dir = private_nh.param<std::string>("atlas_trees_dir", "");
    std::string atlas_trees_files = private_nh.param<std::string>("atlas_trees_files", "");
    std::cout << "atlas dir: " << atlas_trees_dir << std::endl;
    std::cout << "atlas files in: " << atlas_trees_files << std::endl;

    std::ifstream inf2(atlas_trees_files);

    //std::string utm_path = private_nh.param<std::string>("globalmap_utm", "");

    int num;
    inf2 >> num;
    map_num = num;
    cout << atlas_trees_files << " ## " << num << endl;
    for (int i = 0; i < num; i++)
    {
        cout << "#";
        string pathh, filename;
        cout << "#";
        inf2 >> filename;
        pathh = atlas_trees_dir + filename;
        cout << pathh << ": ";

        

        ifstream inf3(pathh);
        kdt tmp;
        tmp.load(inf3);
        trees.push_back(tmp);
    }
    for (int i = 0; i < num; i++)
    {
        Atl.add(&trees[i]);
    }

    map_dir = private_nh.param<std::string>("atlas_map_dir", "");
    string map_conf_file = private_nh.param<std::string>("atlas_map_files", "");

    ifstream inf4(map_conf_file);

    cout << "map info file:" << map_conf_file << endl;

    for (int i = 0; i < num; i++)
    {
        string tmp, pt;
        inf4 >> tmp;
        //cout << tmp << endl;
        pt = map_dir + tmp;
        map_names.push_back(pt);
    }

    for (int i = 0; i < num; i++)
    {
        cout << "map received: " << map_names[i] << endl;
    }

  }

private:
  // ROS
  ros::NodeHandle nh;
  ros::NodeHandle mt_nh;
  ros::NodeHandle private_nh;

  ros::Subscriber gps_sub;
  ros::Publisher globalmap_pub;

  pcl::PointCloud<PointT>::Ptr globalmap;



  vector<pcl::PointCloud<PointT>::Ptr> clouds;

 

  void gps_callback(const sensor_msgs::NavSatFixConstPtr& navsat_msg) {

      struct timeval start1, end1;
      gettimeofday(&start1, NULL);

      now_gps = v3(navsat_msg->latitude, navsat_msg->longitude, navsat_msg->altitude);
      
      gps_msg->header = navsat_msg->header;
      gps_msg->position.latitude = navsat_msg->latitude;
      gps_msg->position.longitude = navsat_msg->longitude;
      gps_msg->position.altitude = navsat_msg->altitude;

      geodesy::fromMsg(gps_msg->position, utm);
      xyz = Eigen::Vector3d(utm.easting, utm.northing, utm.altitude) - map_utm ;

      vec2 qwq = gridmap(xyz.x(), xyz.y());


      int ifchanged;
      Atl.query(now_zone, xyz.x(), xyz.y(), 1);

      ifchanged = sub_map.update(now_zone);

      if (ifchanged)
      {
          cout << "#############ZONE CHANGED!\n";
          //std::string globalmap_pcd = private_nh.param<std::string>("globalmap_pcd", "");
          globalmap.reset(new pcl::PointCloud<PointT>());
          // find maps

          int nnn = 0;
          vector<pcl::PointCloud<PointT>::Ptr> nowmap_ptr;

          for (auto mp : sub_map.now_ids)
          {
              int idx = mp;
              cout << "############################## MAP " << nnn + 1 << ": " << idx << endl;
              
              
              nowmap_ptr.push_back(pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>()));
              pcl::io::loadPCDFile(map_names[idx], *nowmap_ptr[nnn]);

              

              cout << "********************** size:" << nnn + 1 << " " << nowmap_ptr[nnn]->size() << endl;

              pclomp::NormalDistributionsTransform<PointT, PointT>::Ptr ndt(new pclomp::NormalDistributionsTransform<PointT, PointT>());
              ndt->setTransformationEpsilon(0.01);
              ndt->setResolution(6);
              ndt->setNeighborhoodSearchMethod(pclomp::DIRECT1);

              Eigen::Matrix4f init_guess = Eigen::Matrix4f::Identity();
              if (nnn == 0)
              {
                  *globalmap += * nowmap_ptr[0];
                  ndt->setInputTarget(nowmap_ptr[0]);

                  if (single_map)
                  {
                      break;
                  }
              }
              else
              {
                  pcl::PointCloud<PointT>::Ptr aligned(new pcl::PointCloud<PointT>());
                  ndt->setInputSource(nowmap_ptr[1]);
                  ndt->setInputTarget(nowmap_ptr[0]);
                  ndt->align(*aligned, init_guess);
                  *globalmap += *aligned;
                  break;
              }

              

              nnn++;
          }
          cout << endl;
          globalmap_pub.publish(globalmap);

          pcl::io::savePCDFileASCII("/mnt/w/pcds/test_pcd.pcd", *globalmap);

      }

      gettimeofday(&end1, NULL);
      double timeUse = end1.tv_sec - start1.tv_sec + (end1.tv_usec - start1.tv_usec) / 1000000.0;

      cout << "Atlas overhead: " << std::setw(15) << timeUse * 1000 << " ms.\n\n ";
      
  }


};

}


PLUGINLIB_EXPORT_CLASS(hdl_localization::GlobalmapServerNodelet, nodelet::Nodelet)
