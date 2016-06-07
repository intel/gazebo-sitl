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

#include <thread>
#include <mavlink.h>
#include <sys/socket.h>
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>

class MavServer
{
  public:
    MavServer(short port);
    virtual ~MavServer();

    // Helpers
    void run();
    void stop();
    mavlink_mission_item_t
    pose_to_waypoint_relative_alt(double x, double y, double z, double yaw);

    // Vehicle Communication
    bool is_ready();
    int get_status();
    bool set_mode_guided();
    void queue_send_heartbeat_if_needed();
    void queue_send_data(const uint8_t *data, int data_len);
    void queue_send_cmd_long(mavlink_command_long_t mav_cmd);
    void queue_send_waypoint(mavlink_mission_item_t mav_waypoint);
    bool cmd_long_ack_recvd(int mav_cmd_id, int mav_result_expected);
    bool queue_send_cmd_long_until_ack(int cmd, float p1, float p2,
                                               float p3, float p4, float p5,
                                               float p6, float p7, int timeout);

    // State Variables
    bool is_new_attitude = false;
    bool is_new_heartbeat = false;
    bool is_new_command_ack = false;
    bool is_new_gps_raw_int = false;
    bool is_new_local_pos_ned = false;
    bool is_new_home_position = false;
    bool is_new_global_pos_int = false;
    mavlink_attitude_t get_svar_attitude();
    mavlink_heartbeat_t get_svar_heartbeat();
    mavlink_command_ack_t get_svar_command_ack();
    mavlink_gps_raw_int_t get_svar_gps_raw_int();
    mavlink_home_position_t get_svar_home_position();
    mavlink_local_position_ned_t get_svar_local_pos_ned();
    mavlink_global_position_int_t get_svar_global_pos_int();

  private:
    // Threading
    bool send_recv_thread_run = false;
    std::thread send_recv_thread;

    std::mutex svar_access_mtx;
    std::mutex data_to_send_access_mtx;
    std::mutex attitude_svar_access_mtx;
    std::mutex local_pos_ned_svar_access_mtx;

    // Vehicle Communication
    int sock = 0;
    socklen_t fromlen = 0;
    struct sockaddr_in local_addr;
    struct sockaddr_in remote_addr;

    int data_to_send_len = 0;
    enum { BUFFER_LEN = 2041 };
    uint8_t data_recv[BUFFER_LEN];
    uint8_t data_to_send[BUFFER_LEN];

    void send_recv();
    void handle_send();
    void handle_recv();
    void handle_message(const mavlink_message_t *msg);

    // State Variables
    mavlink_attitude_t attitude = {0};
    mavlink_heartbeat_t heartbeat = {0};
    mavlink_command_ack_t command_ack = {0};
    mavlink_gps_raw_int_t gps_raw_int = {0};
    mavlink_home_position_t home_position = {0};
    mavlink_local_position_ned_t local_pos_ned = {0};
    mavlink_global_position_int_t global_pos_int = {0};
};

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

    // Vehicle Status
    bool is_flying();
    bool is_ground_pos_locked();

    // Coordinates
    mavlink_global_position_int_t init_global_pos = {0};
    common::SphericalCoordinates global_pos_coord_system;

    mavlink_global_position_int_t
    home_pos_to_global(mavlink_home_position_t home);
    math::Pose coord_gzlocal_to_mavlocal(math::Pose gzpose);
    void set_global_pos_coord_system(mavlink_global_position_int_t position);
    void calculate_pose(math::Pose *pose, mavlink_attitude_t attitude,
                        mavlink_local_position_ned_t local_position);

    // Mavlink
    MavServer mavserver;

    // Gazebo
    physics::ModelPtr model;
    physics::ModelPtr target;
    event::ConnectionPtr update_connection;
};
}

#endif
