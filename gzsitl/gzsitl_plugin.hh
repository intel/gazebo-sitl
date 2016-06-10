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

#include "mavserver.hh"

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
    std::string target_name;
    event::ConnectionPtr update_connection;
};
}

#endif
