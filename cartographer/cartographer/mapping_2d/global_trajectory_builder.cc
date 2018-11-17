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

#include "cartographer/mapping_2d/global_trajectory_builder.h"
#include <iostream>
#include <fstream>
#include <sys/time.h>
#include <sys/types.h>
#include <cmath>
#include "cartographer/transform/rigid_transform.h"
#include "cartographer/transform/transform.h"
int count_gps=0;
ofstream mark_position_file;
bool mark_position_file_exist=0;
string mark_position_filename;
using namespace std;
using cartographer::transform::Rigid3d;
namespace cartographer {
namespace mapping_2d {

GlobalTrajectoryBuilder::GlobalTrajectoryBuilder(
    const proto::LocalTrajectoryBuilderOptions& options,
    SparsePoseGraph* sparse_pose_graph)
    : options_(options),
      sparse_pose_graph_(sparse_pose_graph),
      local_trajectory_builder_(options),
      ready_to_go(0),
      drawmap_offline(0) {}

GlobalTrajectoryBuilder::~GlobalTrajectoryBuilder() {}

const Submaps* GlobalTrajectoryBuilder::submaps() const {
  return local_trajectory_builder_.submaps();
}

Submaps* GlobalTrajectoryBuilder::submaps() {
  return local_trajectory_builder_.submaps();
}

kalman_filter::PoseTracker* GlobalTrajectoryBuilder::pose_tracker() const {
  return local_trajectory_builder_.pose_tracker();
}

void GlobalTrajectoryBuilder::AddHorizontalLaserFan(
    const common::Time time, const sensor::LaserFan3D& laser_fan) {
  bool move_or_not=0,draw_or_offline_flag=0;
 // recieve_commond(1,1,0,1);
  move_or_not=read_commond();
  draw_or_offline_flag=draw_or_offline();
//  move_or_not=1;
//   cout<<"global horizontal lasergggggggggggggggg1"<<" "<<move_or_not<<endl;
  if(move_or_not)
 {
//	cout<<"global horizontal lasergggggggggggggggg2"<<" "<<draw_or_offline_flag<<endl;
     std::unique_ptr<LocalTrajectoryBuilder::InsertionResult> insertion_result =
          local_trajectory_builder_.AddHorizontalLaserFan(time, laser_fan);
  //   cout<<"global horizontal laserggggggbbbbbbbbbbbbb"<<endl;
      if (insertion_result != nullptr && draw_or_offline_flag) {
        sparse_pose_graph_->AddScan(
            insertion_result->time, insertion_result->tracking_to_tracking_2d,
            insertion_result->laser_fan_in_tracking_2d,
            insertion_result->pose_estimate_2d,
            kalman_filter::Project2D(insertion_result->covariance_estimate),
            insertion_result->submaps, insertion_result->matching_submap,
            insertion_result->insertion_submaps);
      }
 }
}

void GlobalTrajectoryBuilder::AddImuData(
    const common::Time time, const Eigen::Vector3d& linear_acceleration,
    const Eigen::Vector3d& angular_velocity) {
  local_trajectory_builder_.AddImuData(time, linear_acceleration,
                                       angular_velocity);
}

void GlobalTrajectoryBuilder::AddOdometerPose(
    const common::Time time, const transform::Rigid3d& pose,
    const kalman_filter::PoseCovariance& covariance) {
  local_trajectory_builder_.AddOdometerPose(time, pose, covariance);
}
void GlobalTrajectoryBuilder::send_low_confidence()
{
    local_trajectory_builder_.send_low_confidence();
}
const mapping::GlobalTrajectoryBuilderInterface::PoseEstimate&
GlobalTrajectoryBuilder::pose_estimate() const {
  return local_trajectory_builder_.pose_estimate();
}

void GlobalTrajectoryBuilder::AddGpsCalibData(
    const common::Time time, const cartographer::transform::Rigid2d &gps_calib_data) {
  // Initialize pose tracker now if we do not ever use an IMU.
 // local_trajectory_builder_.pose_tracker()->GetGpsPose().gps_pose.translation()(0,0)=
  //                                gps_calib_data.translation()(0,0);

 LOG(INFO) << gps_calib_data.translation();
 // return common::make_unique<transform::Rigid3d>tracking_2d_to_map;
}

void GlobalTrajectoryBuilder::AddGpsData(
    int64 timestamp, const cartographer::transform::Rigid2d& gps_data,
    const cartographer::transform::Rigid2d& gps_data_covariance,int32 gps_flag ) {
  // Initialize pose tracker now if we do not ever use an IMU.
//	LOG(INFO) <<"GPS DATA COMES ----";
 local_trajectory_builder_.SetGpsPose(timestamp,gps_data,gps_data_covariance.translation()(0,0),gps_flag);
  count_gps++;
 // LOG(INFO) <<"gps data time >>>>>"<<timestamp<<endl;
  if(count_gps==90)
 {
    // LOG(INFO) <<"gps data time >>>>>"<<timestamp<<endl;
     LOG(INFO) <<"gps data is coming ----->>>>>"<< gps_data.translation()(0,0)<<"  "<<gps_data.translation()(1,0);
     count_gps=0;
 }
 // return common::make_unique<transform::Rigid3d>tracking_2d_to_map;
}


void GlobalTrajectoryBuilder::recieve_commond(int64 timestamp,bool commond,int32_t count_mark, bool draw_offline)
  {
      // LOG(INFO)<<"do you get there";
       ready_to_go=commond;
       drawmap_offline=draw_offline;
       local_trajectory_builder_.determine_draw_off(draw_offline);
    //   cout<<"global horizontal laserggaaaaaaaaaaaaaaaaaaaaaaaaa"<<endl;
       if(commond&&draw_offline&&!mark_position_file_exist)
       {
           time_t t = time(0);
           char ch[32];

           strftime(ch, sizeof(ch), "%Y_%m_%d_%H_%M_%S", localtime(&t));
           string v_name;
           v_name=ch;
           mark_position_filename="/home/fan/ros_slam/catkin_ws/src/cartographer_ros/my_package/src/data/mark_position_"+v_name+".txt";

           mark_position_file.open(mark_position_filename,ios::app);
           mark_position_file<<local_trajectory_builder_.get_laser_num()<<"\n";
	   mark_position_file<<count_mark<<"\n";
           mark_position_file.close();
	   mark_position_file_exist=1;
       }
       else if(commond&&draw_offline&&mark_position_file_exist)
       {
           mark_position_file.open(mark_position_filename,ios::app);
	   mark_position_file<<local_trajectory_builder_.get_laser_num()<<"\n";
           mark_position_file<<count_mark<<"\n";
           mark_position_file.close();
       }



  }

}  // namespace mapping_2d
}  // namespace cartographer
