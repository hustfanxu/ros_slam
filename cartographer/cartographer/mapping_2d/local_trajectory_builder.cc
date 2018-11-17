/*
 * Copyright 2016 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "cartographer/mapping_2d/local_trajectory_builder.h"

#include <limits>

#include "cartographer/common/make_unique.h"
#include "cartographer/sensor/laser.h"
#include "iostream"
#include <fstream>
#include "cartographer/mapping_2d/scan_matching/proto/fast_correlative_scan_matcher_options.pb.h"
#include "cartographer/mapping_2d/scan_matching/fast_correlative_scan_matcher.h"
#include "cartographer/mapping/sparse_pose_graph/proto/constraint_builder_options.pb.h"
#include <ctime>
#include <time.h>
#include "cartographer/transform/rigid_transform.h"
#include "cartographer/kalman_filter/gps_tracker.h"
#include "sys/time.h"
using namespace std;



bool pure_location=0;
bool read_map=0;
cartographer::transform::Rigid2d part_match_pose_estimate;
namespace cartographer {
namespace mapping_2d {

proto::LocalTrajectoryBuilderOptions CreateLocalTrajectoryBuilderOptions(
    common::LuaParameterDictionary* const parameter_dictionary) {
  proto::LocalTrajectoryBuilderOptions options;
  options.set_horizontal_laser_min_z(
      parameter_dictionary->GetDouble("horizontal_laser_min_z"));
  options.set_horizontal_laser_max_z(
      parameter_dictionary->GetDouble("horizontal_laser_max_z"));
  options.set_horizontal_laser_voxel_filter_size(
      parameter_dictionary->GetDouble("horizontal_laser_voxel_filter_size"));
  options.set_use_online_correlative_scan_matching(
      parameter_dictionary->GetBool("use_online_correlative_scan_matching"));
  *options.mutable_adaptive_voxel_filter_options() =
      sensor::CreateAdaptiveVoxelFilterOptions(
          parameter_dictionary->GetDictionary("adaptive_voxel_filter").get());
  *options.mutable_real_time_correlative_scan_matcher_options() =
      scan_matching::CreateRealTimeCorrelativeScanMatcherOptions(
          parameter_dictionary
              ->GetDictionary("real_time_correlative_scan_matcher")
              .get());
  *options.mutable_ceres_scan_matcher_options() =
      scan_matching::CreateCeresScanMatcherOptions(
          parameter_dictionary->GetDictionary("ceres_scan_matcher").get());
  *options.mutable_motion_filter_options() =
      mapping_3d::CreateMotionFilterOptions(
          parameter_dictionary->GetDictionary("motion_filter").get());
  *options.mutable_pose_tracker_options() =
      kalman_filter::CreatePoseTrackerOptions(
          parameter_dictionary->GetDictionary("pose_tracker").get());
  *options.mutable_submaps_options() = CreateSubmapsOptions(
      parameter_dictionary->GetDictionary("submaps").get());
  options.set_use_imu_data(parameter_dictionary->GetBool("use_imu_data"));
  return options;
}

LocalTrajectoryBuilder::LocalTrajectoryBuilder(
    const proto::LocalTrajectoryBuilderOptions& options)
    : options_(options),
      submaps_(options.submaps_options()),
      scan_matcher_pose_estimate_(transform::Rigid3d::Identity()),
      motion_filter_(options_.motion_filter_options()),
      real_time_correlative_scan_matcher_(
          options_.real_time_correlative_scan_matcher_options()),
      ceres_scan_matcher_(options_.ceres_scan_matcher_options()) ,
     full_map_grid(cartographer::mapping_2d:: MapLimits(0.05, Eigen::Vector2d(200, 200),cartographer::mapping_2d:: CellLimits(8000, 8000))),
      read_grid(1),
      draw_or_off(0)
      {}

LocalTrajectoryBuilder::~LocalTrajectoryBuilder() {}

const Submaps* LocalTrajectoryBuilder::submaps() const { return &submaps_; }

Submaps* LocalTrajectoryBuilder::submaps() { return &submaps_; }

kalman_filter::PoseTracker* LocalTrajectoryBuilder::pose_tracker() const {
  return pose_tracker_.get();
}

sensor::LaserFan LocalTrajectoryBuilder::BuildProjectedLaserFan(
    const transform::Rigid3f& tracking_to_tracking_2d,
    const sensor::LaserFan3D& laser_fan) const {
  const sensor::LaserFan projected_fan = sensor::ProjectCroppedLaserFan(
      sensor::TransformLaserFan3D(laser_fan, tracking_to_tracking_2d),
      Eigen::Vector3f(-std::numeric_limits<float>::infinity(),
                      -std::numeric_limits<float>::infinity(),
                      options_.horizontal_laser_min_z()),
      Eigen::Vector3f(std::numeric_limits<float>::infinity(),
                      std::numeric_limits<float>::infinity(),
                      options_.horizontal_laser_max_z()));
  return sensor::LaserFan{
      projected_fan.origin,
      sensor::VoxelFiltered(projected_fan.point_cloud,
                            options_.horizontal_laser_voxel_filter_size()),
      sensor::VoxelFiltered(projected_fan.missing_echo_point_cloud,
                            options_.horizontal_laser_voxel_filter_size())};
}
int count_frame_2=0,count_small_score=0;
double r_match_score=0;
bool in_location=0;
int follow_gps_location=0;
bool full_map_location=0;
int delay_for_send=0;
int  wait_for_full=0;
double last_score=0;
bool insert_or_not=0;
int real_time_match_area=0;
int first_insert=0;
void LocalTrajectoryBuilder::ScanMatch(
    common::Time time, const transform::Rigid3d& pose_prediction,
    const transform::Rigid3d& tracking_to_tracking_2d,
    const sensor::LaserFan& laser_fan_in_tracking_2d,
    transform::Rigid3d* pose_observation,
    kalman_filter::PoseCovariance* covariance_observation,bool offline) {

    const ProbabilityGrid& probability_grid =
    submaps_.Get(submaps_.matching_index())->probability_grid;

    transform::Rigid2d pose_prediction_2d =
      transform::Project2D(pose_prediction * tracking_to_tracking_2d.inverse());
  // The online correlative scan matcher will refine the initial estimate for
  // the Ceres scan matcher.
    transform::Rigid2d initial_ceres_pose = pose_prediction_2d;
    sensor::AdaptiveVoxelFilter adaptive_voxel_filter(
        options_.adaptive_voxel_filter_options());
    const sensor::PointCloud2D filtered_point_cloud_in_tracking_2d =
      adaptive_voxel_filter.Filter(laser_fan_in_tracking_2d.point_cloud);
    if (options_.use_online_correlative_scan_matching()) {


    r_match_score=real_time_correlative_scan_matcher_.Match(
        pose_prediction_2d, filtered_point_cloud_in_tracking_2d,
        probability_grid, &initial_ceres_pose,read_grid);
        if(read_grid==1)
        {
            cout<<"r_match_score   :   "<<r_match_score<<endl;
            cout<<"initial_ceres_pose : "<<initial_ceres_pose.translation()(0,0)<<" "<<initial_ceres_pose.translation()(1,0)<<endl;
        }
    read_grid=0;
  }

  transform::Rigid2d tracking_2d_to_map;
  kalman_filter::Pose2DCovariance covariance_observation_2d;
  ceres::Solver::Summary summary;
  ceres_scan_matcher_.Match(
      transform::Project2D(scan_matcher_pose_estimate_ *
                           tracking_to_tracking_2d.inverse()),
      initial_ceres_pose, filtered_point_cloud_in_tracking_2d, probability_grid,
      &tracking_2d_to_map, &covariance_observation_2d, &summary);

  CHECK(pose_tracker_ != nullptr);

  *pose_observation = transform::Embed3D(tracking_2d_to_map);
  // This covariance is used for non-yaw rotation and the fake height of 0.
  constexpr double kFakePositionCovariance = 1.;
  constexpr double kFakeOrientationCovariance = 1.;
  *covariance_observation =
      kalman_filter::Embed3D(covariance_observation_2d, kFakePositionCovariance,
                             kFakeOrientationCovariance);
  pose_tracker_->AddPoseObservation(
      time, (*pose_observation) * tracking_to_tracking_2d,
      *covariance_observation);
}

void LocalTrajectoryBuilder::ScanMatch_offline(
    common::Time time, const transform::Rigid3d& pose_prediction,
    const transform::Rigid3d& tracking_to_tracking_2d,
    const sensor::LaserFan& laser_fan_in_tracking_2d,
    transform::Rigid3d* pose_observation,
    kalman_filter::PoseCovariance* covariance_observation,
    const sensor::LaserFan3D& laser_fan,bool offline) {

    const ProbabilityGrid& probability_grid=full_map_grid ;

    count_frame_2=count_frame_2+1;
    transform::Rigid2d pose_prediction_2d =
        transform::Project2D(pose_prediction * tracking_to_tracking_2d.inverse());
  // The online correlative scan matcher will refine the initial estimate for
  // the Ceres scan matcher.
    if(!in_location||(count_small_score>5))
    {

          transform::Rigid2d insert_pose=gps_pose;
          int32 insert_pose_gps_flag=gps_flag;
          transform::Rigid2d relocation_pose=transform::Rigid2d::Translation(Eigen::Vector2d(insert_pose.translation()(0,0), insert_pose.translation()(1,0)));
          transform::Rigid2d relocation_angle=transform::Rigid2d::Rotation(insert_pose.rotation().angle());

          if(gps_flag==0)
            {
                pose_prediction_2d=transform::Rigid2d(relocation_pose.translation()/20,relocation_angle.rotation());
                count_small_score=0;
                insert_or_not=1;
                if(first_insert==0)
                    {
                        first_insert=10;
			last_pose_estimate_.my_pose_confidence=-10;
                    }
                else if(first_insert==10)
                {
                    first_insert=9;
                }
		else if(first_insert==9)
		{
		    first_insert=2;
		}
                LOG(INFO)<<"yes ,i use the relocation : "<<insert_pose.translation()(0,0)/20<<" "<<insert_pose.translation()(1,0)/20;
                follow_gps_location=follow_gps_location+1;
                if(follow_gps_location>15&&full_map_location==0)
                {

                    last_pose_estimate_.my_pose_confidence=-10;
                     wait_for_full=1;

                            if(last_pose_estimate_.low_confidence_sended==1)
                            {
                                transform::Rigid2d gps_position= transform::Rigid2d(relocation_pose.translation(),relocation_angle.rotation());
                                pose_prediction_2d=my_fast_scan_match(gps_position,laser_fan);
                                insert_or_not=1;
                                follow_gps_location=0;
                                wait_for_full=0;

                                full_map_location=1;
                                last_pose_estimate_.low_confidence_sended=0;
                            }
                }
            }
           else if(gps_flag==1)
            {
                transform::Rigid2d gps_position= transform::Rigid2d(relocation_pose.translation(),relocation_angle.rotation());
                pose_prediction_2d=my_fast_scan_match(gps_position,laser_fan);
            }

            in_location=1;
       }
    transform::Rigid2d initial_ceres_pose = pose_prediction_2d;
    sensor::AdaptiveVoxelFilter adaptive_voxel_filter(
        options_.adaptive_voxel_filter_options());
    const sensor::PointCloud2D filtered_point_cloud_in_tracking_2d =
      adaptive_voxel_filter.Filter(laser_fan_in_tracking_2d.point_cloud);
       struct timeval tv1;
    gettimeofday(&tv1,NULL);
	 if (options_.use_online_correlative_scan_matching()) {

	  //  cout<<"the position to follow location : "<<pose_prediction_2d.translation()(0,0)<<" "<<pose_prediction_2d.translation()(1,0)<<endl;

	if(first_insert==10){
    	r_match_score=real_time_correlative_scan_matcher_.Match(
        	pose_prediction_2d, filtered_point_cloud_in_tracking_2d,
       		 probability_grid, &initial_ceres_pose,1);
	}else{
        r_match_score=real_time_correlative_scan_matcher_.Match(
                pose_prediction_2d, filtered_point_cloud_in_tracking_2d,
                 probability_grid, &initial_ceres_pose,first_insert);
	}	


//	cout<<"follow location score : "<<r_match_score<<" wait_for_full:"<<wait_for_full<<" delay_for_send  : "<< delay_for_send<<endl;
           if(wait_for_full>0)
        {
       		 r_match_score=-10;
    	}
	 if(read_grid==1)
        {
            LOG(INFO)<<"r_match_score   :   "<<r_match_score;
            LOG(INFO)<<"initial_ceres_pose : "<<initial_ceres_pose.translation()(0,0)<<" "<<initial_ceres_pose.translation()(1,0);
        }
    if (count_frame_2==10)
     {
        LOG(INFO)<<"r_match_score   :   "<<r_match_score;
        count_frame_2=0;
     }
   read_grid=0;
     if(r_match_score<0.44)
     {
        // cout<<"enlarge the match map : "<<r_match_score<<endl;
         count_small_score++;
     }
     else{
        count_small_score=0;
        first_insert=0;
//	follow_gps_location=0;
     }
  }
  transform::Rigid2d tracking_2d_to_map;
  kalman_filter::Pose2DCovariance covariance_observation_2d;
  ceres::Solver::Summary summary;
  ceres_scan_matcher_.Match(
      transform::Project2D(scan_matcher_pose_estimate_ *
                           tracking_to_tracking_2d.inverse()),
      initial_ceres_pose, filtered_point_cloud_in_tracking_2d, probability_grid,
      &tracking_2d_to_map, &covariance_observation_2d, &summary);

  CHECK(pose_tracker_ != nullptr);
    struct timeval tv2;
  gettimeofday(&tv2,NULL);
  long int millisecond_1=tv1.tv_sec*1000000 + tv1.tv_usec;
  long int millisecond_2=tv2.tv_sec*1000000 + tv2.tv_usec;
 // cout<<"it takes : "<<millisecond_2-millisecond_1<<endl;
  if(last_score>0.6&&r_match_score<0.35)
   {
       LOG(INFO)<<"cccccccccccccccoooooooooooooooommmmmmmmmmmmppppppppp";
        cout<<"    cccccccccccccccoooooooooooooooommmmmmmm      pp"<<endl;
       cout<<"           cccccccccccccccoooooooo                ppo"<<endl;
       cout<<"               cccccccooooooo                     pp  "<<endl;
       cout<<"                   cccoo                          pp "<<endl;
       cout<<"               cccccccooooooo                     pp "<<endl;
       cout<<"           cccccccccccccccooooooooo               pp"<<endl;
       cout<<"    cccccccccccccccoooooooooooooooommmmmmmm       pp"<<endl;
       LOG(INFO)<<"cccccccccccccccoooooooooooooooommmmmmmmmmmmppppppppp";
        LOG(INFO)<<"the position to follow location : "<<pose_prediction_2d.translation()(0,0)<<" "<<pose_prediction_2d.translation()(1,0);
        LOG(INFO)<<"tracking_2d_to_map  :  "<<tracking_2d_to_map.translation()(0,0)<<" "<<tracking_2d_to_map.translation()(1,0);
  LOG(INFO)<<"follow location score : "<<r_match_score<<" wait_for_full:"<<wait_for_full<<" delay_for_send  : "<< delay_for_send;
   }
  last_score=r_match_score;
  *pose_observation = transform::Embed3D(tracking_2d_to_map);
  // This covariance is used for non-yaw rotation and the fake height of 0.
  constexpr double kFakePositionCovariance = 1.;
  constexpr double kFakeOrientationCovariance = 1.;
  *covariance_observation =
      kalman_filter::Embed3D(covariance_observation_2d, kFakePositionCovariance,
                             kFakeOrientationCovariance);
	  if(insert_or_not==1)
  	{
     		 pose_tracker_->InsertPoseEstimateMeanAndCovariance(
            		 time,*pose_observation, *covariance_observation) ;
     		 insert_or_not=0;
 	 }
 	 else if(insert_or_not==0){
        	pose_tracker_->AddPoseObservation(
     		 time, (*pose_observation) * tracking_to_tracking_2d, *covariance_observation);
 	     }
}
std::unique_ptr<LocalTrajectoryBuilder::InsertionResult>
LocalTrajectoryBuilder::AddHorizontalLaserFan(
    const common::Time time, const sensor::LaserFan3D& laser_fan) {

  bool draw_off_flag=1;
  draw_off_flag=get_draw_or_off();
  //cout<<"local horizontal laserrrrrrrrrrrrrrrrrrrr"<<endl;

  // Initialize pose tracker now if we do not ever use an IMU.
  if (!options_.use_imu_data()) {
    InitializePoseTracker(time);
  }
  if(full_map_location==1)
  {
      count_insert_laser++;
      if((count_insert_laser%40)==0)
	{
		LOG(INFO)<<"ddddddddddddooooooooooooooooo";
		cout<<"ddddddddddddooooooooooooooooo"<<endl;
		cout<<"ddddddddddddooooooooooooooooo"<<endl;
		cout<<"ddddddddddddooooooooooooooooo"<<endl;
		cout<<"ddddddddddddooooooooooooooooo"<<endl;

	}
      if(count_insert_laser>600)
	{
		count_insert_laser=0;
		full_map_location=0;
	}
  }

  if (pose_tracker_ == nullptr) {
    // Until we've initialized the UKF with our first IMU message, we cannot
    // compute the orientation of the laser scanner.
    LOG(INFO) << "PoseTracker not yet initialized.";
    return nullptr;
  }
  if(!draw_off_flag&&(!read_map))
  {
      read_grid_from_disk(laser_fan);
      read_map=1;
  }
  transform::Rigid3d pose_prediction;
  kalman_filter::PoseCovariance covariance_prediction;
  pose_tracker_->GetPoseEstimateMeanAndCovariance(time, &pose_prediction,
                                                  &covariance_prediction);

  // Computes the rotation without yaw, as defined by GetYaw().
  const transform::Rigid3d tracking_to_tracking_2d =
      transform::Rigid3d::Rotation(
          Eigen::Quaterniond(Eigen::AngleAxisd(
              -transform::GetYaw(pose_prediction), Eigen::Vector3d::UnitZ())) *
          pose_prediction.rotation());
  const sensor::LaserFan laser_fan_in_tracking_2d =
      BuildProjectedLaserFan(tracking_to_tracking_2d.cast<float>(), laser_fan);

  if (laser_fan_in_tracking_2d.point_cloud.empty()) {
    LOG(WARNING) << "Dropped empty horizontal laser point cloud.";
    return nullptr;
  }

  transform::Rigid3d pose_observation;
  kalman_filter::PoseCovariance covariance_observation;
   if(draw_off_flag)
 {
    ScanMatch(time, pose_prediction, tracking_to_tracking_2d,
            laser_fan_in_tracking_2d, &pose_observation,
            &covariance_observation,read_grid);
 }
 else
 {
     ScanMatch_offline(time, pose_prediction, tracking_to_tracking_2d,
            laser_fan_in_tracking_2d, &pose_observation,
            &covariance_observation,laser_fan,read_grid);
 }
  kalman_filter::PoseCovariance covariance_estimate;
  pose_tracker_->GetPoseEstimateMeanAndCovariance(
      time, &scan_matcher_pose_estimate_, &covariance_estimate);

  // Remove the untracked z-component which floats around 0 in the UKF.
  const auto translation = scan_matcher_pose_estimate_.translation();
  scan_matcher_pose_estimate_ = transform::Rigid3d(
      transform::Rigid3d::Vector(translation.x(), translation.y(), 0.),
      scan_matcher_pose_estimate_.rotation());

  const transform::Rigid3d tracking_2d_to_map =
      scan_matcher_pose_estimate_ * tracking_to_tracking_2d.inverse();
  last_pose_estimate_ = {
      time,
      {pose_prediction, covariance_prediction},
      {pose_observation, covariance_observation},
      {scan_matcher_pose_estimate_, covariance_estimate},
      scan_matcher_pose_estimate_,
      sensor::TransformPointCloud(
          sensor::ToPointCloud(laser_fan_in_tracking_2d.point_cloud),
          tracking_2d_to_map.cast<float>()),
          r_match_score,0};

  const transform::Rigid2d pose_estimate_2d =
      transform::Project2D(tracking_2d_to_map);
  if (motion_filter_.IsSimilar(time, transform::Embed3D(pose_estimate_2d))) {
    return nullptr;
  }
  const mapping::Submap* const matching_submap =
      submaps_.Get(submaps_.matching_index());
  std::vector<const mapping::Submap*> insertion_submaps;
  for (int insertion_index : submaps_.insertion_indices()) {
    insertion_submaps.push_back(submaps_.Get(insertion_index));
  }
 if(draw_off_flag)

 {
     submaps_.InsertLaserFan(TransformLaserFan(laser_fan_in_tracking_2d,
                                          pose_estimate_2d.cast<float>()));
 }
  return common::make_unique<InsertionResult>(InsertionResult{
      time, &submaps_, matching_submap, insertion_submaps,
      tracking_to_tracking_2d, tracking_2d_to_map, laser_fan_in_tracking_2d,
      pose_estimate_2d, covariance_estimate});
}



const mapping::GlobalTrajectoryBuilderInterface::PoseEstimate&
LocalTrajectoryBuilder::pose_estimate() const {
  return last_pose_estimate_;
}
void LocalTrajectoryBuilder::send_low_confidence()  {
   last_pose_estimate_.low_confidence_sended=1;
}
void LocalTrajectoryBuilder::AddImuData(
    const common::Time time, const Eigen::Vector3d& linear_acceleration,
    const Eigen::Vector3d& angular_velocity) {
  CHECK(options_.use_imu_data()) << "An unexpected IMU packet was added.";

  InitializePoseTracker(time);
  pose_tracker_->AddImuLinearAccelerationObservation(time, linear_acceleration);
  pose_tracker_->AddImuAngularVelocityObservation(time, angular_velocity);

  transform::Rigid3d pose_estimate;
  kalman_filter::PoseCovariance unused_covariance_estimate;
  pose_tracker_->GetPoseEstimateMeanAndCovariance(time, &pose_estimate,
                                                  &unused_covariance_estimate);

  // Log a warning if the backpack inclination exceeds 20 degrees. In these
  // cases, it's very likely that 2D SLAM will fail.
  const Eigen::Vector3d gravity_direction =
      Eigen::Quaterniond(pose_estimate.rotation()) * Eigen::Vector3d::UnitZ();
  const double inclination = std::acos(gravity_direction.z());
  constexpr double kMaxInclination = common::DegToRad(20.);
  LOG_IF_EVERY_N(WARNING, inclination > kMaxInclination, 1000)
      << "Max inclination exceeded: " << common::RadToDeg(inclination) << " > "
      << common::RadToDeg(kMaxInclination);
}

void LocalTrajectoryBuilder::AddOdometerPose(
    const common::Time time, const transform::Rigid3d& pose,
    const kalman_filter::PoseCovariance& covariance) {
  if (pose_tracker_ == nullptr) {
    // Until we've initialized the UKF with our first IMU message, we cannot
    // process odometry poses.
    LOG_EVERY_N(INFO, 100) << "PoseTracker not yet initialized.";
  } else {
    pose_tracker_->AddOdometerPoseObservation(time, pose, covariance);
    cout<<"test  odometer data"<<endl;
  }
}

void LocalTrajectoryBuilder::InitializePoseTracker(const common::Time time) {
  if (pose_tracker_ == nullptr) {
    pose_tracker_ = common::make_unique<kalman_filter::PoseTracker>(
        options_.pose_tracker_options(),
        kalman_filter::PoseTracker::ModelFunction::k2D, time);
  }
}
void LocalTrajectoryBuilder::read_grid_from_disk(const sensor::LaserFan3D& laser_fan)
{
        LOG(INFO)<<"start read the map ";

        ifstream infile;
        ifstream infile_2;
        int num_x=0,num_y=0,offset_x=0,offset_y=0;
        infile.open("/home/fan/ros_slam/catkin_ws/src/cartographer_ros/my_package/src/data/2018_08_03_10_43_31_4.txt");
        infile_2.open("/home/fan/ros_slam/catkin_ws/src/cartographer_ros/my_package/src/data/2018_08_03_10_43_31_2.txt");
        string s1,s2,s3,s4;
        getline(infile_2,s1);
        getline(infile_2,s2);
        getline(infile_2,s3);
        getline(infile_2,s4);
        num_x=std::stoi (s1,nullptr,10);
        num_y=std::stoi (s2,nullptr,10);
        offset_x=std::stoi (s3,nullptr,10);
        offset_y=std::stoi (s4,nullptr,10);
        full_map_grid.StartUpdate();
        cartographer::mapping_2d::CellLimits cell_limits_2(num_x-offset_x+1, num_y-offset_y+1);
	int start_point_x=int (num_y-offset_y+1)/2;
	int start_point_y=int (num_x-offset_x+1)/2;
        Eigen::Array2i offset_2(4000-start_point_y,4000-start_point_x);
     //    Eigen::Array2i offset_2( 3375,2876);
        LOG(INFO)<<"probability_grid_size  :  "<<num_x<<"  "<<num_y<<"  "<<offset_x<<"  "<<offset_y;
        for (const Eigen::Array2i& xy_index :
           cartographer::mapping_2d::XYIndexRangeIterator(cell_limits_2)) {
            string s;
            getline(infile,s);
            int data=std::stoi (s,nullptr,10);
            float data_2=data;
            float p=0.0;
            if (data!=-1)
            {
                p= (cartographer::mapping::kMaxProbability - cartographer::mapping::kMinProbability)* data_2/100.0+cartographer::mapping::kMinProbability;
                full_map_grid.SetProbability(xy_index+offset_2,p);
            }
        }
        infile.close();
        infile_2.close();
        LOG(INFO)<<"finish full_map_grid----------------->";

     if (pure_location)
        {
                LOG(INFO)<<"start pure location ---->>>>----->>>>>----->>>>>--->>>>>";
                cartographer::mapping_2d::scan_matching::proto::FastCorrelativeScanMatcherOptions fast_options ;
                fast_options.set_angular_search_window(4);
                fast_options.set_branch_and_bound_depth(2);
                fast_options.set_linear_search_window(2);
                LOG(INFO)<<"start match  : 1";

                std::time_t timep1;
                time (&timep1);
                char tmp1[64];
                strftime(tmp1, sizeof(tmp1), "%Y-%m-%d %H:%M:%S",localtime(&timep1) );
                //cout << "111 the time is "<<tmp1 << endl;

                cartographer::mapping_2d::ProbabilityGrid match_map_grid(cartographer::mapping_2d:: MapLimits(0.05,
                           Eigen::Vector2d(200, 200),cartographer::mapping_2d:: CellLimits(8000, 8000)));
                match_map_grid.StartUpdate();



                Eigen::Array2i offset_3(3600,3600);
                cartographer::mapping_2d::CellLimits cell_limits_3(800, 800);
                for (const Eigen::Array2i& xy_index :
                   cartographer::mapping_2d::XYIndexRangeIterator(cell_limits_3)) {
                    float pp=0.0;
                    pp=full_map_grid.GetProbability(offset_3+xy_index);

                    match_map_grid.SetProbability(xy_index+offset_3,pp);

                }
                //cout<<"start match  : 2"<<endl;



                cartographer::mapping_2d::scan_matching:: FastCorrelativeScanMatcher fast_correlative_scan_matcher_2(match_map_grid,
                                                             fast_options);
                cout<<"start match  : 3"<<endl;

                std::time_t timep2;
	        time (&timep2);
                char tmp2[64];
                strftime(tmp2, sizeof(tmp2), "%Y-%m-%d %H:%M:%S",localtime(&timep2) );
                cout << "222 the time is "<<tmp2 << endl;

                float score_2;
                const cartographer:: sensor::PointCloud2D point_cloud = cartographer::sensor::ProjectToPointCloud2D(laser_fan.returns);
                cout<<"start match  : 4"<<endl;
//                fast_correlative_scan_matcher_2.MatchPartSubmap(
 //                               point_cloud, 0.1, &score_2, &part_match_pose_estimate);
                cout<<"score_2   :  "<<score_2<<endl;
                cout<<"part_match_pose_estimate   :  "<<part_match_pose_estimate.translation().x()<<"   "<<part_match_pose_estimate.translation().y()<<"    "<<part_match_pose_estimate.rotation().angle()<<endl;
                pure_location=0;

                std::time_t timep3;
                time (&timep3);
                char tmp3[64];
                strftime(tmp3, sizeof(tmp3), "%Y-%m-%d %H:%M:%S",localtime(&timep3) );
                cout << "333 the time is "<<tmp3 << endl;

        }

}
cartographer::transform::Rigid2d LocalTrajectoryBuilder::my_fast_scan_match(const cartographer::transform::Rigid2d match_center,
                                                                             const sensor::LaserFan3D& laser_fan)
 {

                LOG(INFO)<<"start pure location ---->>>>----->>>>>----->>>>>--->>>>>";
                cartographer::mapping_2d::scan_matching::proto::FastCorrelativeScanMatcherOptions fast_options ;
                fast_options.set_angular_search_window(4);
                fast_options.set_branch_and_bound_depth(2);
                fast_options.set_linear_search_window(2);


                std::time_t timep1;
                time (&timep1);
                char tmp1[64];
                strftime(tmp1, sizeof(tmp1), "%Y-%m-%d %H:%M:%S",localtime(&timep1) );
                cartographer::mapping_2d::ProbabilityGrid match_map_grid(cartographer::mapping_2d:: MapLimits(0.05,
                           Eigen::Vector2d(200, 200),cartographer::mapping_2d:: CellLimits(8000, 8000)));
                match_map_grid.StartUpdate();

                int center_point_x=int (match_center.translation()(0,0));
                int center_point_y=int (match_center.translation()(1,0));

		 Eigen::Array2i offset_3(3600-center_point_y,3600-center_point_x);
                cartographer::mapping_2d::CellLimits cell_limits_3(800, 800);
                for (const Eigen::Array2i& xy_index :
                   cartographer::mapping_2d::XYIndexRangeIterator(cell_limits_3)) {
                    float pp=0.0;
                    pp=full_map_grid.GetProbability(offset_3+xy_index);

                    match_map_grid.SetProbability(xy_index+offset_3,pp);

                }

                cartographer::mapping_2d::scan_matching:: FastCorrelativeScanMatcher fast_correlative_scan_matcher_2(match_map_grid,
                                                             fast_options);
                std::time_t timep2;
                time (&timep2);
                char tmp2[64];
                strftime(tmp2, sizeof(tmp2), "%Y-%m-%d %H:%M:%S",localtime(&timep2) );

		float score_2=0;
                const cartographer:: sensor::PointCloud2D point_cloud = cartographer::sensor::ProjectToPointCloud2D(laser_fan.returns);
                for(int  full_map_match_times=0;full_map_match_times<3;full_map_match_times++)
		{

                	fast_correlative_scan_matcher_2.MatchPartSubmap(
                                point_cloud, 0.1, &score_2, &part_match_pose_estimate,match_center,full_map_match_times);
			LOG(INFO)<<"part map match : "<<full_map_match_times<<" score : "<<score_2; 
			LOG(INFO)<<"part_match_pose_estimate   :  "<<part_match_pose_estimate.translation().x()<<"   "<<part_match_pose_estimate.translation().y()<<"    "<<part_match_pose_estimate.rotation().angle();

			LOG(INFO)<<"|||||||||||||||||||||||||||||||||||";
			LOG(INFO)<<"                ||||              ";
			LOG(INFO)<<"                ||||              ";
			LOG(INFO)<<"                ||||";
			LOG(INFO)<<"       |||||||||||||||||||||";
                        LOG(INFO)<<"                ||||              ";
                        LOG(INFO)<<"                ||||              ";
                        LOG(INFO)<<"                ||||";
                        LOG(INFO)<<"|||||||||||||||||||||||||||||||||||";
			if(score_2>0.47)
                  	 {
                     		 break;
                 	 }
                }
                LOG(INFO)<<"part_match_pose_estimate   :  "<<part_match_pose_estimate.translation().x()<<"   "<<part_match_pose_estimate.translation().y()<<"    "<<part_match_pose_estimate.rotation().angle();
                pure_location=0;

		LOG(INFO)<<"the ffffffaaaaaaaaaaaaaassssssssst match  :  "<<score_2;
                std::time_t timep3;
                time (&timep3);
                char tmp3[64];
                strftime(tmp3, sizeof(tmp3), "%Y-%m-%d %H:%M:%S",localtime(&timep3) );
                return part_match_pose_estimate;

 }
void LocalTrajectoryBuilder::locate_record_pose(const transform::Rigid2d pose_estimate_2d)
{


}

void  LocalTrajectoryBuilder::  SetGpsPose(int64 timestamp,transform::Rigid2d my_gps_pose,
                   double my_confidence, int32 gps_flag_t)
{

    gps_timestamp=timestamp;

    gps_pose=transform::Rigid2d(my_gps_pose.translation(),my_gps_pose.rotation());

    gps_confidence=my_confidence;

    gps_flag=gps_flag_t;
//    cout<<"finish updating  00000 "<<gps_flag<<endl;

}
int32 LocalTrajectoryBuilder:: GetGpsFlag()
{
    return gps_flag;
}


}  // namespace mapping_2d
}  // mespace cartographer
