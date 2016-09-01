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

#pragma once

#include <thread>
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>

#include "mavlink_vehicles.hh"

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
        ACTIVE_ON_GROUND,
        ERROR
    };
    sim_state simstate = INIT;

    // Vehicle Coordinates
    common::SphericalCoordinates global_pos_coord_system;

    math::Pose coord_gzlocal_to_mavlocal(math::Pose gzpose);
    void set_global_pos_coord_system(mavlink_vehicles::global_pos_int position);
    math::Pose calculate_pose(mavlink_vehicles::attitude attitude,
                              mavlink_vehicles::local_pos local_position);
    math::Vector3 gazebo_local_to_global(math::Pose p);

    // Target and Target Override
    bool perm_target_exists = false;
    bool subs_target_exists = false;
    gazebo::math::Pose subs_target_pose_prev = gazebo::math::Pose::Zero;
    gazebo::math::Pose perm_target_pose_prev = gazebo::math::Pose::Zero;
    std::mutex subs_target_pose_mtx;
    math::Pose subs_target_pose_from_topic;
    std::chrono::time_point<std::chrono::system_clock>
        subs_target_pose_sub_recv_time =
            std::chrono::system_clock::from_time_t(0);

    bool is_target_overridden();
    math::Pose get_subs_target_pose();
    void on_subs_target_pose_recvd(ConstPosePtr &_msg);

    // Mavlink
    std::shared_ptr<mavlink_vehicles::mav_vehicle> mav;
    int sock = 0;
    socklen_t fromlen = {0};
    struct sockaddr_in local_addr = {0};
    struct sockaddr_in remote_addr = {0};
    std::thread send_recv_thread;
    bool send_recv_thread_run = false;

    void send_recv();

    // Gazebo Simulation
    physics::ModelPtr model;
    std::string perm_target_name;
    std::string perm_target_vis_name;
    std::string subs_target_name;
    std::string subs_target_vis_name;
    physics::ModelPtr perm_target;
    physics::ModelPtr perm_target_vis;
    physics::ModelPtr subs_target;
    physics::ModelPtr subs_target_vis;
    event::ConnectionPtr update_connection;

    // Gazebo Communication
    std::string perm_target_pub_topic_name =
        DEFAULT_PERM_TARGET_POSE_PUB_TOPIC_NAME;
    std::string subs_target_sub_topic_name =
        DEFAULT_SUBS_TARGET_POSE_SUB_TOPIC_NAME;
    std::string vehicle_pub_topic_name = DEFAULT_VEHICLE_POSE_PUB_TOPIC_NAME;
    transport::NodePtr node;
    transport::PublisherPtr perm_target_pose_pub;
    transport::PublisherPtr vehicle_pose_pub;
    transport::SubscriberPtr subs_target_pose_sub;
};
}

