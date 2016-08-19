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

#include <mutex>

#include "defines.hh"
#include "gzsitl_plugin.hh"


namespace defaults
{
const uint16_t GZSITL_PERM_TARG_POSE_PUB_FREQ_HZ = 50;
const uint16_t GZSITL_VEHICLE_POSE_PUB_FREQ_HZ = 50;
const uint16_t GZSITL_SUBS_TARG_POSE_SUB_MAX_RESPONSE_TIME = 1000;
const double GZSITL_LOOKAT_TARG_ANG_LIMIT = 30.0;
const double GZSITL_LOOKAT_ROT_SPEED_DEGPS = 90.0;
}

inline double rad2deg(double x)
{
    return (180.0 / M_PI) * x;
}

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
}

void GZSitlPlugin::OnUpdate()
{
    mavserver.queue_send_heartbeat_if_needed();

    // Reset Physic States of the model
    model->ResetPhysicsStates();

    // Get Status of the vehicle
    mavlink_heartbeat_t hb = mavserver.get_svar_heartbeat();
    bool is_guided = hb.base_mode & MAV_MODE_FLAG_GUIDED_ENABLED;
    bool is_armed = hb.base_mode & MAV_MODE_FLAG_SAFETY_ARMED;

    // Get vehicle pose and update gazebo vehicle model
    math::Pose curr_pose;
    if (mavserver.is_new_local_pos_ned || mavserver.is_new_attitude) {
        curr_pose = calculate_pose(mavserver.get_svar_attitude(),
                                   mavserver.get_svar_local_pos_ned());
        model->SetWorldPose(curr_pose);
    }

    // Publish current gazebo vehicle pose
    math::Pose vehicle_pose = model->GetWorldPose();
    this->vehicle_pose_pub->Publish(msgs::Convert(vehicle_pose.Ign()));

    // Get pointer to the permanent target if exists
    this->perm_target = model->GetWorld()->GetModel(perm_target_name);
    if (this->perm_target) {
        this->target_exists = true;
    } else {
        this->target_exists = false;
    }

    // Publish current permanent target pose if target exists
    math::Pose target_pose;
    if (this->target_exists) {
        target_pose = perm_target->GetWorldPose();
        this->perm_target_pose_pub->Publish(msgs::Convert(target_pose.Ign()));
    }

    // Check if target is being overridden by the substitute target
    if (is_target_overridden()) {
        target_pose = get_subs_target_pose();
        this->target_exists = true;
    }

    // Set substitute target position in Gazebo if exists
    this->subs_target = model->GetWorld()->GetModel(subs_target_name);
    if (this->subs_target) {
        this->subs_target->SetWorldPose(target_pose);
    }

    // Calculate target relative pose if exists
    math::Pose target_pose_rel;
    if (this->target_exists) {
        target_pose_rel = target_pose - vehicle_pose;
    }

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

    case ACTIVE_ON_GROUND: {

        // Wait for take off
        if (is_flying()) {
            simstate = ACTIVE_AIRBORNE;
            print_debug_state("state: ACTIVE_AIRBORNE\n");
        }

        return;
    }

    case ACTIVE_AIRBORNE: {

        // Calculate the target azimuthal angle of the target in relation to
        // the vehicle
        double targ_ang =
            rad2deg(atan2(target_pose_rel.pos.y, target_pose_rel.pos.x));

        // Check if the target is located within GZSITL_LOOKAT_TARG_ANG_LIMIT
        // degrees from the vehicle heading. If not, stop in the current
        // position and rotate towards target before moving forward.
        if (fabs(targ_ang) > defaults::GZSITL_LOOKAT_TARG_ANG_LIMIT) {

            // Set current target position equal to vehicle position
            target_pose = vehicle_pose;

            // Change state on next iteration
            simstate = ACTIVE_ROTATING;
            print_debug_state("state: ACTIVE_ROTATING\n");
        }

        // Store static target pose to avoid unnecessary repetition of requests
        static math::Pose target_pose_prev = math::Pose::Zero;

        // Send Target if exists and if it has been moved
        if (is_flying() && target_pose != target_pose_prev) {
            target_pose_prev = target_pose;

            // Convert from Gazebo Local Coordinates to Mav Local NED
            // Coordinates
            math::Pose pose_mavlocal = coord_gzlocal_to_mavlocal(target_pose);

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

    case ACTIVE_ROTATING: {

        // Calculate the target azimuthal angle of the target in relation to
        // the vehicle
        double targ_ang =
            rad2deg(atan2(target_pose_rel.pos.y, target_pose_rel.pos.x));

        // Change state if vehicle is already pointing at the target
        if (fabs(targ_ang) <= defaults::GZSITL_LOOKAT_TARG_ANG_LIMIT) {
            simstate = ACTIVE_AIRBORNE;
            print_debug_state("state: ACTIVE_AIRBORNE\n");
        }

        // Otherwise, continue to request the rotation
        mavserver.queue_send_cmd_long_until_ack(
            MAV_CMD_CONDITION_YAW, fabs(targ_ang),
            defaults::GZSITL_LOOKAT_ROT_SPEED_DEGPS, -copysign(1, targ_ang), 1,
            0, 0, 0, COND_YAW_REQUEST_INTERVAL_MS);

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

    // Get name for the permanent target
    perm_target_name = sdf->Get<std::string>("perm_target_name");
    perm_target = model->GetWorld()->GetModel(perm_target_name);

    // Get name for the substitute target
    subs_target_name = sdf->Get<std::string>("subs_target_name");

    // Get topic names
    if (sdf->HasElement("perm_target_topic_name")) {
        perm_target_pub_topic_name =
            sdf->Get<std::string>("perm_target_topic_name");
    }
    if (sdf->HasElement("subs_target_topic_name")) {
        subs_target_sub_topic_name =
            sdf->Get<std::string>("subs_target_topic_name");
    }
    if (sdf->HasElement("vehicle_topic_name")) {
        vehicle_pub_topic_name = sdf->Get<std::string>("vehicle_topic_name");
    }

    // Setup Publishers
    this->node = transport::NodePtr(new transport::Node());
    this->node->Init(this->model->GetWorld()->GetName());

    this->perm_target_pose_pub = this->node->Advertise<msgs::Pose>(
        "~/" + this->model->GetName() + "/" + perm_target_pub_topic_name, 1,
        defaults::GZSITL_PERM_TARG_POSE_PUB_FREQ_HZ);

    this->vehicle_pose_pub = this->node->Advertise<msgs::Pose>(
        "~/" + this->model->GetName() + "/" + vehicle_pub_topic_name, 1,
        defaults::GZSITL_VEHICLE_POSE_PUB_FREQ_HZ);

    // Setup Subscribers
    this->subs_target_pose_sub =
        node->Subscribe("~/" + std::string(subs_target_sub_topic_name),
                        &GZSitlPlugin::on_subs_target_pose_recvd, this);

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

math::Pose GZSitlPlugin::calculate_pose(mavlink_attitude_t attitude,
                             mavlink_local_position_ned_t local_position)
{
    // Extract pose from mavlink_attitude message converting from NED to ENU
    // coordinates.
    return math::Pose(local_position.x, -local_position.y, -local_position.z,
                     attitude.roll, -attitude.pitch, -attitude.yaw);
}

void GZSitlPlugin::on_subs_target_pose_recvd(ConstPosePtr &_msg)
{
    // Coav Target Pose has been received
    if (_msg->has_position() && _msg->has_orientation()) {
        subs_target_pose_sub_recv_time = std::chrono::system_clock::now();

        std::lock_guard<std::mutex> locker(subs_target_pose_mtx);
        subs_target_pose.Set(gazebo::msgs::ConvertIgn(_msg->position()),
                             gazebo::msgs::ConvertIgn(_msg->orientation()));
    }
}

math::Pose GZSitlPlugin::get_subs_target_pose()
{
    std::lock_guard<std::mutex> locker(subs_target_pose_mtx);
    return subs_target_pose;
}

bool GZSitlPlugin::is_target_overridden()
{
    using namespace std::chrono;

    time_point<system_clock> curr_time = system_clock::now();

    return duration_cast<milliseconds>(curr_time -
                                       subs_target_pose_sub_recv_time)
               .count() < defaults::GZSITL_SUBS_TARG_POSE_SUB_MAX_RESPONSE_TIME;
}
