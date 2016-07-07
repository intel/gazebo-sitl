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

#include "defines.hh"
#include "gzsitl_plugin.hh"

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(GZSitlPlugin)

GZSitlPlugin::GZSitlPlugin()
    : global_pos_coord_system(common::SphericalCoordinates::EARTH_WGS84),
      mavserver(MAVPROXY_PORT)
{
    model = NULL;
}

GZSitlPlugin::~GZSitlPlugin()
{
    event::Events::DisconnectWorldUpdateBegin(update_connection);
}

void GZSitlPlugin::OnUpdate()
{
    mavserver.queue_send_heartbeat_if_needed();

    // Reset Physic States of the model
    model->ResetPhysicsStates();

    // Execute according to simulation state
    switch (simstate) {
    case INIT: {

        if (!mavserver.is_ready()) {
            return;
        }

        if (mavserver.get_status() == MAV_STATE_STANDBY) {
            simstate = INIT_ON_GROUND;
            print_debug_state("state: INIT_ON_GROUND\n");
        } else if (mavserver.get_status() == MAV_STATE_ACTIVE) {
            simstate = INIT_AIRBORNE;
            print_debug_state("state: INIT_AIRBORNE\n");
        }

        return;
    }

    case INIT_AIRBORNE: {
        // Check if home position has already been received
        if (mavserver.is_new_home_position) {
            mavlink_home_position_t home = mavserver.get_svar_home_position();
            mavlink_global_position_int_t home_pos = home_pos_to_global(home);
            set_global_pos_coord_system(home_pos);
            simstate = ACTIVE_AIRBORNE;
            print_debug_state("state: ACTIVE_AIRBORNE\n");
            return;
        }

        // Home position is critical. Request home position every
        // HOME_POSITION_REQUEST_INTERVAL_MS ms until it receiv
        mavserver.queue_send_cmd_long_until_ack(
            MAV_CMD_GET_HOME_POSITION, 0, 0, 0, 0, 0, 0, 0,
            HOME_POSITION_REQUEST_INTERVAL_MS);

        return;
    }

    case INIT_ON_GROUND: {
        // Get Status
        mavlink_heartbeat_t hb = mavserver.get_svar_heartbeat();
        bool is_guided = hb.base_mode & MAV_MODE_FLAG_GUIDED_ENABLED;
        bool is_armed = hb.base_mode & MAV_MODE_FLAG_SAFETY_ARMED;

        // Wait until initial global position is achieved
        if (!is_ground_pos_locked()) {
            return;
        }

        // Initial takeoff if AUTOTAKEOFF and if not already on air
        if (!TAKEOFF_AUTO) {
            simstate = ACTIVE_ON_GROUND;
            print_debug_state("state: ACTIVE_ON_GROUND\n");
            return;
        }

        if (!is_guided) {
            mavserver.set_mode_guided();
            return;
        }

        if (!is_armed) {
            mavserver.queue_send_cmd_long_until_ack(
                MAV_CMD_COMPONENT_ARM_DISARM, 1, 0, 0, 0, 0, 0, 0,
                TAKEOFF_REQUEST_INTERVAL_MS);
            return;
        }

        if (!is_flying()) {
            mavserver.queue_send_cmd_long_until_ack(
                MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, TAKEOFF_INIT_ALT_M,
                TAKEOFF_REQUEST_INTERVAL_MS);
            return;
        }

        print_debug_state("Takeoff Sucessfull.\n");
        simstate = ACTIVE_AIRBORNE;
        print_debug_state("state: ACTIVE_AIRBORNE\n");

        return;
    }

    case ACTIVE_ON_GROUND:
    case ACTIVE_AIRBORNE: {

        // Get Target Position
        math::Pose curr_pose;
        math::Vector3 curr_vel;
        math::Vector3 curr_ang_vel;
        static math::Pose tpose = math::Pose(math::Pose::Zero);
        math::Pose tpose_new = tpose;


        // Calculate pose according to new attitude and position
        if (mavserver.is_new_local_pos_ned || mavserver.is_new_attitude) {

            // Set New Drone Pose in Gazebo
            calculate_pose(&curr_pose, mavserver.get_svar_attitude(),
                           mavserver.get_svar_local_pos_ned());
            model->SetWorldPose(curr_pose);
        }

        // Make sure the target still exists
        target = model->GetWorld()->GetModel(target_name);
        if (!target) {
            return;
        }

        // Send Target if exists and if it has been moved
        tpose_new = target->GetWorldPose();
        if (is_flying() && tpose_new != tpose) {
            tpose = tpose_new;

            // Convert from Gazebo Local Coordinates to Mav Local NED
            // Coordinates
            math::Pose pose_mavlocal = coord_gzlocal_to_mavlocal(tpose_new);

            // Convert from Mav Local NED Coordinates to Global Coordinates
            math::Vector3 global_coord =
                global_pos_coord_system.SphericalFromLocal(
                    ignition::math::Vector3d(-pose_mavlocal.pos.y,
                                             -pose_mavlocal.pos.x,
                                             -pose_mavlocal.pos.z));

            // Convert from Global Coordinates to Global Coordinates with
            // Relative Alt
            global_coord.z = global_coord.z -
                             global_pos_coord_system.GetElevationReference();

            // Send target coordinates through mavlink
            mavserver.queue_send_waypoint(
                mavserver.pose_to_waypoint_relative_alt(
                    global_coord.x, global_coord.y, global_coord.z,
                    pose_mavlocal.rot.GetYaw()));
        }

        return;
    }

    case ERROR:
    default:
        break;
    }

    return;
}

void GZSitlPlugin::Load(physics::ModelPtr m, sdf::ElementPtr sdf)
{

    // Output the name of the model
    print_debug_state("The gzsitl plugin is attached to the model\n");

    // Store the model pointer for convenience
    model = m;

    // Also store the target pointer
    target_name = sdf->Get<std::string>("target_name");
    target = model->GetWorld()->GetModel(target_name);

    // Run MavServer thread
    mavserver.run();

    // Set initial simulation parameters
    printf("init\n");
    simstate = INIT;
    print_debug_state("state: INIT\n");

    // Listen to the update event
    update_connection = event::Events::ConnectWorldUpdateBegin(
        boost::bind(&GZSitlPlugin::OnUpdate, this));
}

bool GZSitlPlugin::is_flying()
{
    mavlink_heartbeat_t status = mavserver.get_svar_heartbeat();
    return status.system_status == MAV_STATE_ACTIVE;
}

bool GZSitlPlugin::is_ground_pos_locked()
{
    static int n = 0;

    // If vehicle is not airborne, receive Initial Position at least
    // INIT_POS_NUMSAMPLES times
    if (n < INIT_POS_NUMSAMPLES) {
        if (mavserver.is_new_global_pos_int && mavserver.is_ready()) {
            n++;
        }
        return false;
    }

    if (n == INIT_POS_NUMSAMPLES) {
        init_global_pos = mavserver.get_svar_global_pos_int();

        set_global_pos_coord_system(init_global_pos);

        n = INIT_POS_NUMSAMPLES + 1;
    }

    return true;
}

mavlink_global_position_int_t
GZSitlPlugin::home_pos_to_global(mavlink_home_position_t home)
{
    mavlink_global_position_int_t global_pos = {0};

    global_pos.lat = home.latitude;
    global_pos.lon = home.longitude;
    global_pos.alt = home.altitude;

    return global_pos;
}

math::Pose GZSitlPlugin::coord_gzlocal_to_mavlocal(math::Pose gzpose)
{
    return math::Pose(gzpose.pos.x, -gzpose.pos.y, -gzpose.pos.z,
                      gzpose.rot.GetRoll(), -gzpose.rot.GetPitch(),
                      -gzpose.rot.GetYaw());
}

void GZSitlPlugin::set_global_pos_coord_system(
    mavlink_global_position_int_t position)
{
    ignition::math::Angle ref_lat =
        ignition::math::Angle(((double)position.lat / 1E7) * (M_PI / 180.0));
    ignition::math::Angle ref_lon =
        ignition::math::Angle(((double)position.lon / 1E7) * (M_PI / 180.0));

    global_pos_coord_system.SetElevationReference(position.alt / 1000.0);
    global_pos_coord_system.SetLatitudeReference(ref_lat);
    global_pos_coord_system.SetLongitudeReference(ref_lon);
}

void GZSitlPlugin::calculate_pose(
    math::Pose *pose, mavlink_attitude_t attitude,
    mavlink_local_position_ned_t local_position)
{
    pose->Set(local_position.x,
              -local_position.y,
              -local_position.z,
              attitude.roll,
              -attitude.pitch,
              -attitude.yaw);
}
