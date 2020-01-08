#ifndef POSE_ESTIMATOR_HPP
#define POSE_ESTIMATOR_HPP

const int RETRY_ANGLES = 4;
const int GPS_ENABLED = 1; //GPS relocalization
const int ENABLE_SEARCH = 1;
#define use_gps 1

#include <memory>
#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <pclomp/ndt_omp.h>
#include <pcl/filters/voxel_grid.h>

#include <hdl_localization/pose_system.hpp>
#include <kkl/alg/unscented_kalman_filter.hpp>

#include <geodesy/utm.h>
#include <geodesy/wgs84.h>

#include <geographic_msgs/GeoPointStamped.h>

#include <cmath>
#include <ctime>
#include <cstdlib>

#include<sys/time.h>

#include <iostream>
#include <iomanip>

#include <vector>

// Probability(%) of matching correctness computation, adaptive to computer performance
double examine_freq = 100;

namespace hdl_localization {

struct v3
{
  public:
  int flag;
  double x;
  double y;
  double z;
  v3(double xx,double yy,double zz)
  {
    x=xx;
    y=yy;
    z=zz;
    flag=1;
  }
  v3()
  {
    flag=0;
  }
  v3 diff(v3 i)
  {
	  if (flag && i.flag)
		  return v3(x - i.x, y - i.y, z - i.z);
	  else
		  return v3();
  }
  v3 add(v3 i)
  {
	  if (flag && i.flag)
		  return v3(x + i.x, y + i.y, z + i.z);
	  else
		  return v3();
  }
};



/**
 * @brief scan matching-based pose estimator
 */
class PoseEstimator {
public:
  using PointT = pcl::PointXYZI;
  
  //static double check_cooldown;

  /**
   * @brief constructor
   * @param registration        registration method
   * @param stamp               timestamp
   * @param pos                 initial position
   * @param quat                initial orientation
   * @param cool_time_duration  during "cool time", prediction is not performed
   */
  PoseEstimator(pcl::Registration<PointT, PointT>::Ptr& registration, const ros::Time& stamp, const Eigen::Vector3f& pos, const Eigen::Quaternionf& quat, double cool_time_duration = 1.0)
    : init_stamp(stamp),
      registration(registration),
      cool_time_duration(cool_time_duration)
  {
    process_noise = Eigen::MatrixXf::Identity(16, 16);
    process_noise.middleRows(0, 3) *= 1.0;
    process_noise.middleRows(3, 3) *= 1.0;
    process_noise.middleRows(6, 4) *= 0.5;
    process_noise.middleRows(10, 3) *= 1e-6;
    process_noise.middleRows(13, 3) *= 1e-6;

    Eigen::MatrixXf measurement_noise = Eigen::MatrixXf::Identity(7, 7);
    measurement_noise.middleRows(0, 3) *= 0.01;
    measurement_noise.middleRows(3, 4) *= 0.001;

    Eigen::VectorXf mean(16);
    mean.middleRows(0, 3) = pos;
    mean.middleRows(3, 3).setZero();
    mean.middleRows(6, 4) = Eigen::Vector4f(quat.w(), quat.x(), quat.y(), quat.z());
    mean.middleRows(10, 3).setZero();
    mean.middleRows(13, 3).setZero();

    Eigen::MatrixXf cov = Eigen::MatrixXf::Identity(16, 16) * 0.01;

    PoseSystem system;
    ukf.reset(new kkl::alg::UnscentedKalmanFilterX<float, PoseSystem>(system, 16, 6, 7, process_noise, measurement_noise, mean, cov));
  }

  /**
   * @brief predict
   * @param stamp    timestamp
   * @param acc      acceleration
   * @param gyro     angular velocity
   */
  void predict(const ros::Time& stamp, const Eigen::Vector3f& acc, const Eigen::Vector3f& gyro) {
    if((stamp - init_stamp).toSec() < cool_time_duration || prev_stamp.is_zero() || prev_stamp == stamp) {
      prev_stamp = stamp;
      return;
    }

    double dt = (stamp - prev_stamp).toSec();
    prev_stamp = stamp;

    ukf->setProcessNoiseCov(process_noise * dt);
    ukf->system.dt = dt;

    Eigen::VectorXf control(6);
    control.head<3>() = acc;
    control.tail<3>() = gyro;

    ukf->predict(control);
  }

  /**
   * @brief correct
   * @param cloud   input cloud
   * @return cloud aligned to the globalmap
   */



  pcl::PointCloud<PointT>::Ptr correct(const pcl::PointCloud<PointT>::ConstPtr& cloud, v3 gps, v3 map_utm, v3 vel) {
    Eigen::Matrix4f init_guess = Eigen::Matrix4f::Identity();
    init_guess.block<3, 3>(0, 0) = quat().toRotationMatrix();
    init_guess.block<3, 1>(0, 3) = pos();

    pcl::PointCloud<PointT>::Ptr aligned(new pcl::PointCloud<PointT>());
    registration->setInputSource(cloud);
    registration->align(*aligned, init_guess);

    Eigen::Matrix4f trans = registration->getFinalTransformation();
    
	Eigen::Vector3f p = trans.block<3, 1>(0, 3);
    Eigen::Quaternionf q(trans.block<3, 3>(0, 0)); 

	const double fitness_thres = 5;
	const double deviation_thres = 20;

	struct timeval start1, end1, start2, end2;

	// Computing fitness takes so much time (100~150 ms), so we only compute it occasionally.

	int retry_flag = 0;
	v3 utm_xyz;
	v3 rel;
	double quat_w, quat_z;

	double fitness;

	if (rand() % 100 < examine_freq)
	//std::cout << check_cooldown << std::endl;
	//if(check_cooldown >= 100)
	{
		//check_cooldown -= 100;
		gettimeofday(&start1, NULL);

		//	The smaller, the better. Successful tracking usually <1.
		//

		if(gps.flag)
		{

		  geodesy::UTMPoint utm;

		  geographic_msgs::GeoPointStampedPtr wgs(new geographic_msgs::GeoPointStamped());
		  wgs->position.latitude = gps.x;
		  wgs->position.longitude = gps.y;
		  wgs->position.altitude = gps.z;

		  geodesy::fromMsg(wgs->position,utm);

		  std::cout << "WGS84 Coordinates: " << std::fixed << std::setprecision(6) << gps.x <<" " <<gps.y << " " << gps.z << std::endl;

		  std::cout << "UTM Coordinates: " << std::fixed << std::setprecision(2) << utm.easting <<" " <<utm.northing << " " << utm.altitude << std::endl;

		  utm_xyz = v3(utm.easting, utm.northing, utm.altitude);
		  rel = utm_xyz.diff(map_utm);

		  std::cout << "Relative: " << std::fixed << std::setprecision(2) << rel.x << " " << rel.y << " " << rel.z << std::endl;

		  std::cout << "Velocity: " << std::fixed << std::setprecision(2) << vel.x << " " << vel.y << " " << vel.z << std::endl;
	      
		  double absv = sqrt(vel.x * vel.x + vel.y * vel.y);

		  int v_valid = (absv > 0.01);

		  if (v_valid)
		  {
			  double ori_angle = atan2(vel.y, vel.x);
			  double quat_angle = ori_angle / 2;
			  quat_w = cos(quat_angle);
			  quat_z = sin(quat_angle);

		  }

		  v3 dev(pos().x() - rel.x, pos().y() - rel.y, pos().z() - rel.z);
		  double devabs = sqrt(dev.x * dev.x + dev.y * dev.y);

		  
		  double d_bad = devabs / deviation_thres;

		  double f_bad;
		  


		  fitness = registration->getFitnessScore();

		  f_bad = fitness / fitness_thres;


		  if ((GPS_ENABLED) && (f_bad + d_bad > 1 && f_bad > 0.12 && d_bad > 0.2 && (v_valid || ENABLE_SEARCH))) //Retrying without orientation is still under development.
		  {
			  std::cout << "*** Bad! (Fitness " << std::fixed << std::setprecision(2) << fitness <<", Deviation " << devabs << ")";

			  init_guess = Eigen::Matrix4f::Identity();
			  init_guess.block<3, 1>(0, 3) = Eigen::Vector3f(rel.x, rel.y, rel.z);
			  //init_guess.block<3, 1>(0, 3) = Eigen::Vector3f(rel.x, rel.y, 0);
			  int search_flag = 0;

			  if (v_valid)
			  {
				  std::cout << "and GPS velocity available... Retrying with GPS orientation data." << std::endl;
				  init_guess.block<3, 3>(0, 0) = Eigen::Quaternionf(quat_w, 0, 0, quat_z).toRotationMatrix();
			  }
			  
			  else
			  {
				  search_flag = 1;
				  std::cout << "but GPS velocity not available... Retrying with different orientations." << std::endl;
				  double min_fitness = 1e+243;
				  int min_i = 0;
				  double w, z;
				  double dt = 3.141592654 / RETRY_ANGLES;
				  for (int i = 0; i < RETRY_ANGLES; i++)
				  {
					  w = cos(dt * i);
					  z = sin(dt * i);
					  init_guess.block<3, 3>(0, 0) = Eigen::Quaternionf(w, 0, 0, z).toRotationMatrix();

					  pcl::PointCloud<PointT>::Ptr aligned(new pcl::PointCloud<PointT>());
					  registration->setInputSource(cloud);
					  registration->align(*aligned, init_guess);

					  double try_fitness = registration->getFitnessScore();
					  if (try_fitness < min_fitness)
					  {
						  min_fitness = try_fitness;
						  min_i = i;
					  }
					  else
						  continue;

				  }
				  int i = min_i;
				  w = cos(dt * i);
				  z = sin(dt * i);
				  
				  init_guess.block<3, 3>(0, 0) = Eigen::Quaternionf(w, 0, 0, z).toRotationMatrix();
				  fitness = min_fitness;
			  }
			  //*/
			  

			  pcl::PointCloud<PointT>::Ptr aligned(new pcl::PointCloud<PointT>());
			  registration->setInputSource(cloud);
			  registration->align(*aligned, init_guess);

			  trans = registration->getFinalTransformation();

			  p = trans.block<3, 1>(0, 3);
			  q = Eigen::Quaternionf(trans.block<3, 3>(0, 0));

			  if (!search_flag)
			  {
				  fitness = registration->getFitnessScore();
			  }

			  std::cout << "Corrected fitness:" << std::fixed << std::setprecision(2) << fitness << std::endl;

		  }


		}

		//	Adaptive frequency control
		gettimeofday(&end1, NULL);

		double timeUse = end1.tv_sec - start1.tv_sec + (end1.tv_usec - start1.tv_usec) / 1000000.0;
		std::cout << std::fixed << std::setprecision(2);
		std::cout << "	pos     :" << std::setw(10) << p.x() << " " << std::setw(10) << p.y() << " " << std::setw(10) << p.z() << " " << std::endl;
		std::cout << std::fixed << std::setprecision(6);
		std::cout << "	quat    :" << std::setw(10) << q.w() << " " << std::setw(10) << q.x() << " " << std::setw(10) << q.y() << " " << std::setw(10) << q.z() << " " << std::endl;
		
		std::cout << std::endl;

		std::cout << "	quat_gps:" << std::setw(10) << quat_w << " " << std::setw(10) << 0.0 << " " << std::setw(10) << 0.0 << " " << std::setw(10) << quat_z << " " << std::endl;
		
		std::cout << std::fixed << std::setprecision(2) << std::endl;

		v3 dev(p.x() - rel.x, p.y() - rel.y, p.z() - rel.z);

		std::cout << "deviation:" << std::setw(10) << dev.x << " " << std::setw(10) << dev.y << " " << std::setw(10) << dev.z << " " << std::endl;
		std::cout << "fitness:" << fitness << "							 overhead: " << std::setw(15) << timeUse * 1000 << " ms, " << "checked " << examine_freq << "%." << std::endl;

		double overhead = timeUse * 1000 * examine_freq / 100;

		///////////////////////////////////////////RESET Kalman filter!

		process_noise = Eigen::MatrixXf::Identity(16, 16);
		process_noise.middleRows(0, 3) *= 1.0;
		process_noise.middleRows(3, 3) *= 1.0;
		process_noise.middleRows(6, 4) *= 0.5;
		process_noise.middleRows(10, 3) *= 1e-6;
		process_noise.middleRows(13, 3) *= 1e-6;

		Eigen::MatrixXf measurement_noise = Eigen::MatrixXf::Identity(7, 7);
		measurement_noise.middleRows(0, 3) *= 0.01;
		measurement_noise.middleRows(3, 4) *= 0.001;

		Eigen::VectorXf mean(16);
		mean.middleRows(0, 3) = p;
		mean.middleRows(3, 3).setZero();
		mean.middleRows(6, 4) = Eigen::Vector4f(q.w(), q.x(), q.y(), q.z());
		mean.middleRows(10, 3).setZero();
		mean.middleRows(13, 3).setZero();

		Eigen::MatrixXf cov = Eigen::MatrixXf::Identity(16, 16) * 0.01;

		PoseSystem system;
		ukf.reset(new kkl::alg::UnscentedKalmanFilterX<float, PoseSystem>(system, 16, 6, 7, process_noise, measurement_noise, mean, cov));
		///////////////////////////////////////////


		if (overhead > 15)
		{
			double adj = 35 / overhead;
			if (adj > 0.7) adj = 0.7;
			examine_freq *= adj;
			if (examine_freq < 7)
				examine_freq = 7;
		}
		else if (overhead < 10 && examine_freq < 20)
			examine_freq += 1.5;
		else if (overhead < 15 && examine_freq < 20)
			examine_freq += 0.4;

		//check_cooldown += 100.0 / examine_freq;

		std::cout << std::endl << "---------------------------------------" << std::endl;
	}



    if(quat().coeffs().dot(q.coeffs()) < 0.0f) {
      q.coeffs() *= -1.0f;
    }

    Eigen::VectorXf observation(7);
    observation.middleRows(0, 3) = p;
    observation.middleRows(3, 4) = Eigen::Vector4f(q.w(), q.x(), q.y(), q.z());

    ukf->correct(observation);
    return aligned;
  }

  pcl::PointCloud<PointT>::Ptr correct(const pcl::PointCloud<PointT>::ConstPtr& cloud){
	  return correct(cloud, v3(), v3(), v3());
  }

  /* getters */
  Eigen::Vector3f pos() const {
    return Eigen::Vector3f(ukf->mean[0], ukf->mean[1], ukf->mean[2]);
  }

  Eigen::Vector3f vel() const {
    return Eigen::Vector3f(ukf->mean[3], ukf->mean[4], ukf->mean[5]);
  }

  Eigen::Quaternionf quat() const {
    return Eigen::Quaternionf(ukf->mean[6], ukf->mean[7], ukf->mean[8], ukf->mean[9]).normalized();
  }

  Eigen::Matrix4f matrix() const {
    Eigen::Matrix4f m = Eigen::Matrix4f::Identity();
    m.block<3, 3>(0, 0) = quat().toRotationMatrix();
    m.block<3, 1>(0, 3) = pos();
    return m;
  }

private:
  ros::Time init_stamp;         // when the estimator was initialized
  ros::Time prev_stamp;         // when the estimator was updated last time
  double cool_time_duration;    //

  Eigen::MatrixXf process_noise;
  std::unique_ptr<kkl::alg::UnscentedKalmanFilterX<float, PoseSystem>> ukf;

  pcl::Registration<PointT, PointT>::Ptr registration;
};


}



#endif // POSE_ESTIMATOR_HPP
