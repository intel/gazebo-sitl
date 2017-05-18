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
const uint16_t GZSITL_MISSION_TARGET_POSE_PUB_FREQ_HZ = 50;
const uint16_t GZSITL_VEHICLE_POSE_PUB_FREQ_HZ = 50;
}

inline double rad2deg(double x)
{
    return (180.0 / M_PI) * x;
}

using namespace gazebo;
using namespace mavlink_vehicles;

GZ_REGISTER_MODEL_PLUGIN(GZSitlPlugin)

GZSitlPlugin::GZSitlPlugin()
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

    // Get vehicle pose and update gazebo vehicle model
    Pose3d vehicle_pose;
    vehicle_pose = calculate_pose(this->mav->get_attitude(),
                               this->mav->get_local_position_ned());
    model->SetWorldPose(vehicle_pose);

    // Publish current gazebo vehicle pose
    this->vehicle_pose_pub->Publish(msgs::Convert(vehicle_pose));

    // Get pointer to the mission target model if exists
    this->mission_target = model->GetWorld()->ModelByName(mission_target_name);
    this->mission_target_exists = (bool)this->mission_target;

    // Retrieve and publish current mission target pose if it exists
    if (this->mission_target_exists) {
        this->mission_target_pose = this->mission_target->WorldPose();
        this->mission_target_pose_pub->Publish(
            msgs::Convert(this->mission_target_pose));
    }

    // Execute according to simulation state
    switch (simstate) {
    case INIT: {

        if (status == mavlink_vehicles::status::STANDBY) {
            simstate = INIT_ON_GROUND;
            print_debug_state("state: INIT_ON_GROUND\n");
        } else if (status == mavlink_vehicles::status::ACTIVE) {
            simstate = INIT_AIRBORNE;
            print_debug_state("state: INIT_AIRBORNE\n");
        }

        break;
    }

    case INIT_AIRBORNE: {

        // Set home position
        this->home_position = this->mav->get_home_position_int();
        simstate = ACTIVE_AIRBORNE;
        print_debug_state("state: ACTIVE_AIRBORNE\n");

        break;
    }

    case INIT_ON_GROUND: {

        // Set home position
        this->home_position = this->mav->get_home_position_int();

        // Initial takeoff if AUTOTAKEOFF and if not already on air
        if (!TAKEOFF_AUTO) {
            simstate = ACTIVE_ON_GROUND;
            print_debug_state("state: ACTIVE_ON_GROUND\n");
            break;
        }

        if (status != mavlink_vehicles::status::ACTIVE) {
            this->mav->takeoff();
            break;
        }

        // Wait until the vehicle is at least 1m off the ground
        if (vehicle_pose.Pos()[2] < 1.0) {
            break;
        }

        print_debug_state("Takeoff Sucessfull.\n");
        simstate = ACTIVE_AIRBORNE;
        print_debug_state("state: ACTIVE_AIRBORNE\n");

        break;
    }

    case ACTIVE_ON_GROUND: {

        // Wait for take off
        if (status == mavlink_vehicles::status::ACTIVE) {
            simstate = ACTIVE_AIRBORNE;
            print_debug_state("state: ACTIVE_AIRBORNE\n");
        }

        break;
    }

    case ACTIVE_AIRBORNE: {

        // Send the mission target to the vehicle
        if (this->mission_target_pose != this->mission_target_pose_prev) {
            mavlink_vehicles::global_pos_int global_coord =
                mavlink_vehicles::math::local_ned_to_global(
                    mavlink_vehicles::local_pos(this->mission_target_pose.Pos()[1],
                                                this->mission_target_pose.Pos()[0],
                                                -this->mission_target_pose.Pos()[2]),
                    this->home_position);
            this->mav->send_mission_waypoint(global_coord, true);
            this->mission_target_pose_prev = this->mission_target_pose;
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

    // Get name for the mission target
    mission_target_name = sdf->Get<std::string>("mission_target_name");

    // Get topic names
    if (sdf->HasElement("mission_target_topic_name")) {
        mission_target_pub_topic_name =
            sdf->Get<std::string>("mission_target_topic_name");
    }
    if (sdf->HasElement("vehicle_topic_name")) {
        vehicle_pub_topic_name = sdf->Get<std::string>("vehicle_topic_name");
    }

    // Setup Publishers
    this->node = transport::NodePtr(new transport::Node());
    this->node->Init(this->model->GetWorld()->Name());

    this->mission_target_pose_pub = this->node->Advertise<msgs::Pose>(
        "~/" + this->model->GetName() + "/" + mission_target_pub_topic_name, 1,
        defaults::GZSITL_MISSION_TARGET_POSE_PUB_FREQ_HZ);

    this->vehicle_pose_pub = this->node->Advertise<msgs::Pose>(
        "~/" + this->model->GetName() + "/" + vehicle_pub_topic_name, 1,
        defaults::GZSITL_VEHICLE_POSE_PUB_FREQ_HZ);

    // Set initial simulation parameters
    printf("init\n");
    simstate = INIT;
    print_debug_state("state: INIT\n");

    // Listen to the update event
    update_connection = event::Events::ConnectWorldUpdateBegin(
        boost::bind(&GZSitlPlugin::OnUpdate, this));
}

Pose3d GZSitlPlugin::calculate_pose(attitude attitude, local_pos local_position)
{
    // Convert from NED (North, East, Down) to ENU (East, North Up)
    return Pose3d(local_position.y, local_position.x,
                              -local_position.z, attitude.pitch, attitude.roll,
                              -attitude.yaw);
}

