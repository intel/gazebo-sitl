/*
// Copyright (c) 2016 Intel Corporation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
*/

#ifndef _GZSITL_PLUGIN_HH_
#define _GZSITL_PLUGIN_HH_

#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>

#include "mavserver.hh"

#define DEFAULT_PERM_TARGET_POSE_PUB_TOPIC_NAME "target_pose"
#define DEFAULT_SUBS_TARGET_POSE_SUB_TOPIC_NAME "coav/coav_target_pose"
#define DEFAULT_VEHICLE_POSE_PUB_TOPIC_NAME "vehicle_pose"

namespace gazebo
{

class GAZEBO_VISIBLE GZSitlPlugin : public ModelPlugin
{

  public:
    GZSitlPlugin();
    virtual ~GZSitlPlugin();

    void OnUpdate();
    virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

  private:
    // Simulation State
    enum sim_state {
        INIT,
        INIT_ON_GROUND,
        INIT_AIRBORNE,
        ACTIVE_AIRBORNE,
        ACTIVE_ROTATING,
        ACTIVE_ON_GROUND,
        ERROR
    };
    sim_state simstate = INIT;

    // Vehicle Status
    bool is_flying();
    bool is_ground_pos_locked();

    // Vehicle Coordinates
    mavlink_global_position_int_t init_global_pos = {0};
    common::SphericalCoordinates global_pos_coord_system;

    mavlink_global_position_int_t
    home_pos_to_global(mavlink_home_position_t home);
    math::Pose coord_gzlocal_to_mavlocal(math::Pose gzpose);
    void set_global_pos_coord_system(mavlink_global_position_int_t position);
    math::Pose calculate_pose(mavlink_attitude_t attitude,
                              mavlink_local_position_ned_t local_position);

    // Target and Target Override
    bool target_exists = false;
    physics::ModelPtr subs_target;
    std::string subs_target_name;
    std::mutex subs_target_pose_mtx;
    math::Pose subs_target_pose;
    std::chrono::time_point<std::chrono::system_clock>
        subs_target_pose_sub_recv_time =
            std::chrono::system_clock::from_time_t(0);

    bool is_target_overridden();
    math::Pose get_subs_target_pose();
    void on_subs_target_pose_recvd(ConstPosePtr &_msg);

    // Mavlink
    MavServer mavserver;

    // Gazebo Simulation
    physics::ModelPtr model;
    std::string perm_target_name;
    physics::ModelPtr perm_target;
    event::ConnectionPtr update_connection;

    // Gazebo Communication
    std::string perm_target_pub_topic_name = DEFAULT_PERM_TARGET_POSE_PUB_TOPIC_NAME;
    std::string subs_target_sub_topic_name = DEFAULT_SUBS_TARGET_POSE_SUB_TOPIC_NAME;
    std::string vehicle_pub_topic_name = DEFAULT_VEHICLE_POSE_PUB_TOPIC_NAME;
    transport::NodePtr node;
    transport::PublisherPtr perm_target_pose_pub;
    transport::PublisherPtr vehicle_pose_pub;
    transport::SubscriberPtr subs_target_pose_sub;

};
}

#endif
