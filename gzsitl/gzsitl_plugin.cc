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
const double GZSITL_LOOKAT_TARG_ANG_LIMIT = 60.0;
const double GZSITL_LOOKAT_ROT_ANG_THRESH_DEG = 10.0;
const double GZSITL_LOOKAT_ROT_SPEED_DEGPS = 90.0;
const double GZSITL_MIN_ROT_DIST_M = 0.5;
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
    }

    // Attempt to make it non blocking
    if (fcntl(sock, F_SETFL, O_NONBLOCK | FASYNC) < 0) {
        perror("error setting socket as nonblocking");
        close(sock);
    }

    // Instantiate mav_vehicle
    this->mav = std::make_shared<mav_vehicle>(sock);

    // Initialize mav_vehicle update thread
    this->send_recv_thread_run = true;
    this->send_recv_thread = std::thread(&GZSitlPlugin::send_recv, this);
}

GZSitlPlugin::~GZSitlPlugin()
{
    std::cout << "Shutting down" << std::endl;
    this->send_recv_thread_run = false;
    this->send_recv_thread.join();
    close(this->sock);
    std::cout << "Shut down" << std::endl;
}

void GZSitlPlugin::send_recv()
{
    while (this->send_recv_thread_run) {
        this->mav->update();
    }
}

void GZSitlPlugin::OnUpdate()
{
    // Reset Physic States of the model
    this->model->ResetPhysicsStates();

    // Check if mavlink vehicle is initialized
    if (!this->mav->is_ready()) {
        return;
    }

    // Get Status of the vehicle
    status status = this->mav->get_status();
    arm_status arm_stat = this->mav->get_arm_status();
    mode mod = this->mav->get_mode();

    // Get vehicle pose and update gazebo vehicle model
    gazebo::math::Pose curr_pose;
    curr_pose = calculate_pose(this->mav->get_attitude(),
                               this->mav->get_local_position_ned());
    model->SetWorldPose(curr_pose);

    // Publish current gazebo vehicle pose
    gazebo::math::Pose vehicle_pose = model->GetWorldPose();
    this->vehicle_pose_pub->Publish(msgs::Convert(vehicle_pose.Ign()));

    // Get pointer to the permanent target control model if exists
    this->perm_target = model->GetWorld()->GetModel(perm_target_name);
    this->perm_target_exists = (bool)this->perm_target;

    // Retrieve and publish current permanent target pose if target exists
    if (this->perm_target_exists) {
        this->perm_target_pose = this->perm_target->GetWorldPose();
        this->perm_target_pose_pub->Publish(
            msgs::Convert(this->perm_target_pose.Ign()));
    }

    // Get pointer to the subs target control model if exists
    this->subs_target = model->GetWorld()->GetModel(subs_target_name);
    this->subs_target_exists = (bool)this->subs_target;

    // Retrieve current substitute target pose if target exists
    if (this->subs_target_exists) {
        this->subs_target_pose = this->subs_target->GetWorldPose();
    } else if(is_target_overridden()) {
        this->subs_target_pose = this->get_subs_target_pose();

    }

    // Update permanent target visualization according to the vehicle
    mavlink_vehicles::local_pos perm_targ_pos =
        mavlink_vehicles::math::global_to_local_ned(
            this->mav->get_mission_waypoint(),
            this->mav->get_home_position_int());
    if (this->perm_target_vis =
            model->GetWorld()->GetModel(perm_target_vis_name)) {
        this->perm_target_vis->SetWorldPose(gazebo::math::Pose(
            perm_targ_pos.x, perm_targ_pos.y, -perm_targ_pos.z, 0, 0, 0));
    }

    // Update substitute target visualization according to vehicle
    mavlink_vehicles::local_pos subs_targ_pos =
        mavlink_vehicles::math::global_to_local_ned(
            this->mav->get_detour_waypoint(),
            this->mav->get_home_position_int());
    if (this->subs_target_vis =
            model->GetWorld()->GetModel(subs_target_vis_name)) {
        this->subs_target_vis->SetWorldPose(gazebo::math::Pose(
            subs_targ_pos.x, subs_targ_pos.y, -subs_targ_pos.z, 0, 0, 0));
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

        break;
    }

    case INIT_AIRBORNE: {

        // Set home position
        global_pos_int home_pos = this->mav->get_home_position_int();
        set_global_pos_coord_system(home_pos);
        simstate = ACTIVE_AIRBORNE;
        print_debug_state("state: ACTIVE_AIRBORNE\n");

        break;
    }

    case INIT_ON_GROUND: {

        // Set home position
        global_pos_int home_pos = this->mav->get_home_position_int();
        set_global_pos_coord_system(home_pos);

        // Initial takeoff if AUTOTAKEOFF and if not already on air
        if (!TAKEOFF_AUTO) {
            simstate = ACTIVE_ON_GROUND;
            print_debug_state("state: ACTIVE_ON_GROUND\n");
            break;
        }

        if (mod != mode::GUIDED) {
            this->mav->set_mode(mode::GUIDED);
            break;
        }

        if (arm_stat != arm_status::ARMED) {
            this->mav->arm_throttle();
            break;
        }

        if (status != status::ACTIVE) {
            this->mav->takeoff();
            break;
        }

        print_debug_state("Takeoff Sucessfull.\n");
        simstate = ACTIVE_AIRBORNE;
        print_debug_state("state: ACTIVE_AIRBORNE\n");

        break;
    }

    case ACTIVE_ON_GROUND: {

        // Wait for take off
        if (status == status::ACTIVE) {
            simstate = ACTIVE_AIRBORNE;
            print_debug_state("state: ACTIVE_AIRBORNE\n");
        }

        break;
    }

    case ACTIVE_AIRBORNE: {

        // Send the permanent target to the vehicle
        if (this->perm_target_pose != this->perm_target_pose_prev) {
            gazebo::math::Vector3 global_coord =
                gazebo_local_to_global(this->perm_target_pose);

            this->mav->send_mission_waypoint(global_coord.x, global_coord.y,
                                             global_coord.z);
            this->perm_target_pose_prev = this->perm_target_pose;
        }

        // Send the substitute target to the vehicle
        if (this->subs_target_pose != this->subs_target_pose_prev) {
            gazebo::math::Vector3 global_coord =
                gazebo_local_to_global(this->subs_target_pose);
            this->mav->send_detour_waypoint(global_coord.x, global_coord.y,
                                            global_coord.z, false);
            this->subs_target_pose_prev = this->subs_target_pose;
        }

        break;
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
    perm_target_name = sdf->Get<std::string>("perm_target_ctrl_name");
    perm_target_vis_name = sdf->Get<std::string>("perm_target_vis_name");

    // Get name for the substitute target
    subs_target_name = sdf->Get<std::string>("subs_target_ctrl_name");
    subs_target_vis_name = sdf->Get<std::string>("subs_target_vis_name");

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

gazebo::math::Pose GZSitlPlugin::coord_gzlocal_to_mavlocal(gazebo::math::Pose gzpose)
{
    return gazebo::math::Pose(gzpose.pos.y, gzpose.pos.x, -gzpose.pos.z,
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

gazebo::math::Pose GZSitlPlugin::calculate_pose(attitude attitude,
                                                local_pos local_position)
{
    // Convert from NED (North, East, Down) to ENU (East, North Up)
    return gazebo::math::Pose(local_position.y, local_position.x,
                              -local_position.z, attitude.pitch, attitude.roll,
                              -attitude.yaw);
}

void GZSitlPlugin::on_subs_target_pose_recvd(ConstPosePtr &_msg)
{
    // Coav Target Pose has been received
    if (_msg->has_position() && _msg->has_orientation()) {
        this->subs_target_pose_sub_recv_time = std::chrono::system_clock::now();

        std::lock_guard<std::mutex> locker(subs_target_pose_mtx);
        this->subs_target_pose_from_topic.Set(
            gazebo::msgs::ConvertIgn(_msg->position()),
            gazebo::msgs::ConvertIgn(_msg->orientation()));
    }
}

gazebo::math::Pose GZSitlPlugin::get_subs_target_pose()
{
    std::lock_guard<std::mutex> locker(subs_target_pose_mtx);
    return subs_target_pose_from_topic;
}

bool GZSitlPlugin::is_target_overridden()
{
    using namespace std::chrono;

    time_point<system_clock> curr_time = system_clock::now();

    return duration_cast<milliseconds>(curr_time -
                                       subs_target_pose_sub_recv_time)
               .count() < defaults::GZSITL_SUBS_TARG_POSE_SUB_MAX_RESPONSE_TIME;
}

gazebo::math::Vector3 GZSitlPlugin::gazebo_local_to_global(gazebo::math::Pose p)
{

    // Convert from Gazebo Local Coordinates to Mav Local NED
    // Coordinates
    gazebo::math::Pose pose_mavlocal = coord_gzlocal_to_mavlocal(p);

    // Convert from Mav Local NED Coordinates to Global Coordinates
    gazebo::math::Vector3 global_coord =
        global_pos_coord_system.SphericalFromLocal(ignition::math::Vector3d(
            -pose_mavlocal.pos.y, -pose_mavlocal.pos.x, -pose_mavlocal.pos.z));

    // Convert from Global Coordinates to Global Coordinates with
    // Relative Alt
    global_coord.z -= global_pos_coord_system.GetElevationReference();

    return global_coord;
}

