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
#include <thread>

#include "defines.hh"
#include "gzsitl_plugin.hh"
#include "mavlink_vehicles.hh"


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
using namespace mavlink_vehicles;

GZ_REGISTER_MODEL_PLUGIN(GZSitlPlugin)

GZSitlPlugin::GZSitlPlugin()
    : global_pos_coord_system(common::SphericalCoordinates::EARTH_WGS84)
{
    // Socket Initialization
    this->sock = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock == -1) {
        perror("error opening socket");
        exit(EXIT_FAILURE);
    }

    memset(&local_addr, 0, sizeof(local_addr));
    local_addr.sin_family = AF_INET;
    local_addr.sin_addr.s_addr = INADDR_ANY;
    local_addr.sin_port = htons(MAVPROXY_PORT);

    if (bind(sock, (struct sockaddr *)&local_addr, sizeof(struct sockaddr)) ==
        -1) {
        perror("error bind failed");
        close(sock);
        exit(EXIT_FAILURE);
    }

    // Attempt to make it non blocking
    if (fcntl(sock, F_SETFL, O_NONBLOCK | FASYNC) < 0) {
        perror("error setting socket as nonblocking");
        close(sock);
        exit(EXIT_FAILURE);
    }

    // Instantiate mav_vehicle
    this->mav = std::make_shared<mav_vehicle>(sock);

    // Initialize mav_vehicle update thread
    this->send_recv_thread_run = true;
    this->send_recv_thread = std::thread(&GZSitlPlugin::send_recv, this);
    this->send_recv_thread.detach();
}

GZSitlPlugin::~GZSitlPlugin()
{
    this->send_recv_thread_run = false;
    this->send_recv_thread.join();
}

void GZSitlPlugin::send_recv()
{
    while (true) {
        this->mav->update();
    }
}


void GZSitlPlugin::OnUpdate()
{
    // Reset Physic States of the model
    this->model->ResetPhysicsStates();

    // Check if mavlink vehicle is initialized
    if(!this->mav->started()) {
        return;
    }

    // Get Status of the vehicle
    status status = this->mav->get_status();
    arm_status arm_stat = this->mav->get_arm_status();
    mode mod = this->mav->get_mode();

    // Get vehicle pose and update gazebo vehicle model
    math::Pose curr_pose;
    curr_pose = calculate_pose(this->mav->get_attitude(),
                               this->mav->get_local_position_ned());
    model->SetWorldPose(curr_pose);

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

        if (status == status::STANDBY) {
            simstate = INIT_ON_GROUND;
            print_debug_state("state: INIT_ON_GROUND\n");
        } else if (status == status::ACTIVE) {
            simstate = INIT_AIRBORNE;
            print_debug_state("state: INIT_AIRBORNE\n");
        }

        return;
    }

    case INIT_AIRBORNE: {

        // Set home position
        global_pos_int home_pos = this->mav->get_home_position_int();
        set_global_pos_coord_system(home_pos);
        simstate = ACTIVE_AIRBORNE;
        print_debug_state("state: ACTIVE_AIRBORNE\n");
        return;
    }

    case INIT_ON_GROUND: {

        // Set home position
        global_pos_int home_pos = this->mav->get_home_position_int();
        set_global_pos_coord_system(home_pos);

        // Initial takeoff if AUTOTAKEOFF and if not already on air
        if (!TAKEOFF_AUTO) {
            simstate = ACTIVE_ON_GROUND;
            print_debug_state("state: ACTIVE_ON_GROUND\n");
            return;
        }

        if (mod != mode::GUIDED) {
            this->mav->set_mode(mode::GUIDED);
            return;
        }

        if (arm_stat != arm_status::ARMED) {
            this->mav->arm_throttle();
            return;
        }

        if (status != status::ACTIVE) {
            this->mav->takeoff();
            return;
        }

        print_debug_state("Takeoff Sucessfull.\n");
        simstate = ACTIVE_AIRBORNE;
        print_debug_state("state: ACTIVE_AIRBORNE\n");

        return;
    }

    case ACTIVE_ON_GROUND: {

        // Wait for take off
        if (status == status::ACTIVE) {
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
        if (target_pose != target_pose_prev) {
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
            this->mav->goto_waypoint(global_coord.x, global_coord.y,
                                     global_coord.z);
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
        this->mav->rotate(targ_ang);
            // MAV_CMD_CONDITION_YAW, fabs(targ_ang),
            // GZSITL_LOOKAT_ROT_SPEED_DEGPS, -copysign(1, targ_ang), 1, 0, 0, 0,
            // 2000);
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

    // Set initial simulation parameters
    printf("init\n");
    simstate = INIT;
    print_debug_state("state: INIT\n");

    // Listen to the update event
    update_connection = event::Events::ConnectWorldUpdateBegin(
        boost::bind(&GZSitlPlugin::OnUpdate, this));
}

math::Pose GZSitlPlugin::coord_gzlocal_to_mavlocal(math::Pose gzpose)
{
    return math::Pose(gzpose.pos.x, -gzpose.pos.y, -gzpose.pos.z,
                      gzpose.rot.GetRoll(), -gzpose.rot.GetPitch(),
                      -gzpose.rot.GetYaw());
}

void GZSitlPlugin::set_global_pos_coord_system(
    global_pos_int position)
{
    ignition::math::Angle ref_lat =
        ignition::math::Angle(((double)position.lat / 1E7) * (M_PI / 180.0));
    ignition::math::Angle ref_lon =
        ignition::math::Angle(((double)position.lon / 1E7) * (M_PI / 180.0));

    global_pos_coord_system.SetElevationReference(position.alt / 1000.0);
    global_pos_coord_system.SetLatitudeReference(ref_lat);
    global_pos_coord_system.SetLongitudeReference(ref_lon);
}

math::Pose GZSitlPlugin::calculate_pose(attitude attitude,
                                        local_pos local_position)
{
    // Convert from NED (North, East, Down) to ENU (East, North Up)
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
