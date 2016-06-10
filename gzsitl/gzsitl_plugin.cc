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

#include <fcntl.h>
#include <arpa/inet.h>
#include "gzsitl_plugin.hh"

// TODO: Set as .sdf plugin parameters
#define DEBUG_STATE true
#define DEBUG_MAVLINK false
#define MAVPROXY_IP "127.0.0.1"
#define MAVPROXY_PORT 14556
#define LOCAL_PORT 14550
#define DEFAULT_TARGET_SYSTEM_ID 1    // Default Copter system ID
#define DEFAULT_TARGET_COMPONENT_ID 1 // Default Copter component ID
#define DEFAULT_SYSTEM_ID 22          // This system ID
#define DEFAULT_COMPONENT_ID 0        // This component ID
#define HEARTBEAT_SEND_INTERVAL_MS 1000
#define INIT_POS_NUMSAMPLES 3
#define TAKEOFF_AUTO true
#define TAKEOFF_INIT_ALT_M 0.5
#define HOME_POSITION_REQUEST_INTERVAL_MS 3000
#define MODE_SET_REQUEST_INTERVAL_MS 3000
#define TAKEOFF_REQUEST_INTERVAL_MS 3000

#if DEBUG_MAVLINK
#define print_debug_mav(...) printf(__VA_ARGS__)
#else
#define print_debug_mav(...) ;
#endif

#if DEBUG_STATE
#define print_debug_state(...) printf(__VA_ARGS__)
#else
#define print_debug_state(...) ;
#endif

MavServer::MavServer(short port)
{

    // Socket Initialization
    sock = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if(sock == -1) {
        perror("error opening socket");
        exit(EXIT_FAILURE);
    }

    memset(&local_addr, 0, sizeof(local_addr));
    local_addr.sin_family = AF_INET;
    local_addr.sin_addr.s_addr = INADDR_ANY;
    local_addr.sin_port = htons(port);

    if (bind(sock, (struct sockaddr *)&local_addr, sizeof(struct sockaddr)) == -1) {
        perror("error bind failed");
        close(sock);
        exit(EXIT_FAILURE);
    }

    /* Attempt to make it non blocking */
    if (fcntl(sock, F_SETFL, O_NONBLOCK | FASYNC) < 0) {
        perror("error setting socket as nonblocking");
        close(sock);
        exit(EXIT_FAILURE);
    }

    remote_addr.sin_family = AF_INET;
    remote_addr.sin_addr.s_addr = inet_addr(MAVPROXY_IP);
    remote_addr.sin_port = htons(LOCAL_PORT);
}

MavServer::~MavServer()
{
    stop(); 
    close(sock);
}

void MavServer::run()
{
    send_recv_thread_run = true;
    send_recv_thread = std::thread(&MavServer::send_recv, this);
    send_recv_thread.detach();
}

void MavServer::stop()
{
    send_recv_thread_run = false;
}

mavlink_mission_item_t
MavServer::pose_to_waypoint_relative_alt(double x, double y, double z, double yaw)
{
    mavlink_mission_item_t mav_waypoint;

    mav_waypoint.param1 = 0;        // Hold time in decimal seconds
    mav_waypoint.param2 = 0.01;     // Acceptance radius in meters
    mav_waypoint.param3 = 0;        // Radius in meters to pass through wp
    mav_waypoint.param4 = yaw;      // Desired yaw angle
    mav_waypoint.x = x;
    mav_waypoint.y = y;
    mav_waypoint.z = z;
    mav_waypoint.seq = 0;
    mav_waypoint.command = MAV_CMD_NAV_WAYPOINT;
    mav_waypoint.target_system = DEFAULT_TARGET_SYSTEM_ID;
    mav_waypoint.target_component = DEFAULT_TARGET_COMPONENT_ID;

    // Arducopter supports only MAV_FRAME_GLOBAL_RELATIVE_ALT.
    mav_waypoint.frame = MAV_FRAME_GLOBAL_RELATIVE_ALT;
    mav_waypoint.current = 2;       // Must be set as 2 for GUIDED waypoint
    mav_waypoint.autocontinue = 0;

    return mav_waypoint;
}

bool MavServer::is_ready()
{
    // Check if GPS is locked and if system status is OK
    mavlink_heartbeat_t status = get_svar_heartbeat();
    mavlink_gps_raw_int_t gps = get_svar_gps_raw_int();

    return gps.fix_type >= 2 && (status.system_status == MAV_STATE_STANDBY ||
                                 status.system_status == MAV_STATE_ACTIVE);
}

int MavServer::get_status()
{
    mavlink_heartbeat_t status = get_svar_heartbeat();
    return status.system_status;
}

bool MavServer::set_mode_guided()
{
    using namespace std::chrono;

    mavlink_set_mode_t mav_cmd_set_mode;
    mavlink_message_t mav_msg;
    static uint8_t mav_data_buffer[BUFFER_LEN];

    static time_point<system_clock> send_time = system_clock::from_time_t(0);

    time_point<system_clock> curr_time = system_clock::now();

    // Send command to change mode to Guided
    if (duration_cast<milliseconds>(curr_time - send_time).count() >
        MODE_SET_REQUEST_INTERVAL_MS) {
        mav_cmd_set_mode.target_system = DEFAULT_TARGET_COMPONENT_ID;

        // Arducopter does not use the standard MAV_MODE_FLAG. It uses
        // a custom mode instead. GUIDED mode is defined as 4.
        mav_cmd_set_mode.base_mode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
        mav_cmd_set_mode.custom_mode = 4; // GUIDED == 4

        mavlink_msg_set_mode_encode(DEFAULT_SYSTEM_ID,
                                    DEFAULT_COMPONENT_ID, &mav_msg,
                                    &mav_cmd_set_mode);

        int n = mavlink_msg_to_send_buffer(mav_data_buffer, &mav_msg);
        queue_send_data(mav_data_buffer, n);

        send_time = curr_time;

        print_debug_state("Changing to GUIDED mode...\n");
    }

    // Check if mode has changed and ACK has been received
    if (cmd_long_ack_recvd(MAVLINK_MSG_ID_SET_MODE,
                                   MAV_RESULT_ACCEPTED)) {
        return true;
    }

    return false;
}

void MavServer::queue_send_heartbeat_if_needed()
{

    using namespace std::chrono;
    time_point<system_clock> curr_time = system_clock::now();

    static time_point<system_clock> last_hb_sendtime = curr_time;

    int time_elapsed =
        duration_cast<milliseconds>(curr_time - last_hb_sendtime).count();

    if (time_elapsed <= HEARTBEAT_SEND_INTERVAL_MS) {
        return;
    }

    mavlink_message_t mav_msg;
    mavlink_heartbeat_t mav_heartbeat;
    static uint8_t mav_data_buffer[BUFFER_LEN];

    mav_heartbeat.type = MAV_TYPE_GCS;
    mav_heartbeat.autopilot = MAV_AUTOPILOT_INVALID;
    mav_heartbeat.base_mode = 0;
    mav_heartbeat.custom_mode = 0;
    mav_heartbeat.system_status = MAV_STATE_ACTIVE;

    mavlink_msg_heartbeat_encode(DEFAULT_SYSTEM_ID, DEFAULT_COMPONENT_ID,
                                 &mav_msg, &mav_heartbeat);

    int n = mavlink_msg_to_send_buffer(mav_data_buffer, &mav_msg);
    queue_send_data(mav_data_buffer, n);

    last_hb_sendtime = curr_time;
}

void MavServer::queue_send_data(const uint8_t *data, int data_len)
{

    data_to_send_access_mtx.lock();

    int safe_data_len =
        std::min((int)data_len, (int)(BUFFER_LEN - data_to_send_len));

    if (safe_data_len >= data_len) {
        memcpy(&data_to_send[data_to_send_len], data,
               safe_data_len * sizeof(*data));
        data_to_send_len += safe_data_len;
    }

    data_to_send_access_mtx.unlock();
}

void MavServer::queue_send_cmd_long(mavlink_command_long_t cmd)
{
    mavlink_message_t mav_msg;
    static uint8_t mav_data_buffer[BUFFER_LEN];

    mavlink_msg_command_long_encode(
        DEFAULT_SYSTEM_ID, DEFAULT_COMPONENT_ID, &mav_msg, &cmd);

    int n = mavlink_msg_to_send_buffer(mav_data_buffer, &mav_msg);
    queue_send_data(mav_data_buffer, n);
}

void MavServer::queue_send_waypoint(mavlink_mission_item_t mav_waypoint)
{
    mavlink_message_t mav_msg;
    static uint8_t mav_data_buffer[BUFFER_LEN];

    mav_waypoint.current = 2; // Set as a Guided waypoint

    mavlink_msg_mission_item_encode(DEFAULT_SYSTEM_ID,
                                    DEFAULT_COMPONENT_ID, &mav_msg,
                                    &mav_waypoint);

    int n = mavlink_msg_to_send_buffer(mav_data_buffer, &mav_msg);
    queue_send_data(mav_data_buffer, n);
}

bool MavServer::cmd_long_ack_recvd(int mav_cmd_id,
                                           int mav_result_expected)
{
    if (!is_new_command_ack) {
        return false;
    }

    mavlink_command_ack_t cmd_ack = get_svar_command_ack();

    return (cmd_ack.command == mav_cmd_id) &&
           (cmd_ack.result == mav_result_expected);
}

bool MavServer::queue_send_cmd_long_until_ack(int cmd, float p1, float p2,
                                                float p3, float p4, float p5,
                                                float p6, float p7, int timeout)
{
    using namespace std::chrono;

    mavlink_command_long_t mav_cmd;

    static std::unordered_map<int, time_point<system_clock>> send_time = {};

    time_point<system_clock> curr_time = system_clock::now();

    if (!send_time.count(cmd) ||
        duration_cast<milliseconds>(curr_time - send_time[cmd]).count() >
            timeout) {

        mav_cmd.target_system = DEFAULT_TARGET_SYSTEM_ID;
        mav_cmd.target_component = DEFAULT_TARGET_COMPONENT_ID;
        mav_cmd.command = cmd;
        mav_cmd.confirmation = 0;
        mav_cmd.param1 = p1;
        mav_cmd.param2 = p2;
        mav_cmd.param3 = p3;
        mav_cmd.param4 = p4;
        mav_cmd.param5 = p5;
        mav_cmd.param6 = p6;
        mav_cmd.param7 = p7;

        queue_send_cmd_long(mav_cmd);
        send_time[cmd] = curr_time;

        switch (cmd) {
        case MAV_CMD_COMPONENT_ARM_DISARM:
            print_debug_state("Arming Throttle...\n");
            break;
        case MAV_CMD_NAV_TAKEOFF:
            print_debug_state("Requesting Takeoff...\n");
            break;
        case MAV_CMD_GET_HOME_POSITION:
            print_debug_state("Requesting Home Position...\n");
            break;
        default:
            break;
        }
    }

    if (cmd_long_ack_recvd(cmd, MAV_RESULT_ACCEPTED)) {
        return true;
    }

    return false;
}

mavlink_attitude_t MavServer::get_svar_attitude()
{
    attitude_svar_access_mtx.lock();
    mavlink_attitude_t attitude_copy = attitude;
    is_new_attitude = false;
    attitude_svar_access_mtx.unlock();
    return attitude_copy;
}

mavlink_heartbeat_t MavServer::get_svar_heartbeat()
{
    svar_access_mtx.lock();
    mavlink_heartbeat_t heartbeat_copy = heartbeat;
    is_new_heartbeat = false;
    svar_access_mtx.unlock();
    return heartbeat_copy;
}

mavlink_command_ack_t MavServer::get_svar_command_ack()
{
    svar_access_mtx.lock();
    mavlink_command_ack_t command_ack_copy = command_ack;
    is_new_command_ack = false;
    svar_access_mtx.unlock();
    return command_ack_copy;
}

mavlink_gps_raw_int_t MavServer::get_svar_gps_raw_int()
{
    svar_access_mtx.lock();
    mavlink_gps_raw_int_t gps_raw_int_copy = gps_raw_int;
    is_new_gps_raw_int = false;
    svar_access_mtx.unlock();
    return gps_raw_int_copy;
}

mavlink_home_position_t MavServer::get_svar_home_position()
{
    svar_access_mtx.lock();
    mavlink_home_position_t home_position_copy = home_position;
    is_new_home_position = false;
    svar_access_mtx.unlock();
    return home_position_copy;
}

mavlink_local_position_ned_t MavServer::get_svar_local_pos_ned()
{

    local_pos_ned_svar_access_mtx.lock();
    mavlink_local_position_ned_t local_pos_int_copy = local_pos_ned;
    is_new_local_pos_ned = false;
    local_pos_ned_svar_access_mtx.unlock();
    return local_pos_int_copy;
}

mavlink_global_position_int_t MavServer::get_svar_global_pos_int()
{

    svar_access_mtx.lock();
    mavlink_global_position_int_t global_pos_int_copy = global_pos_int;
    is_new_global_pos_int = false;
    svar_access_mtx.unlock();
    return global_pos_int_copy;
}

void MavServer::send_recv()
{
    while (send_recv_thread_run) {
        handle_recv();
        handle_send();
    }
}

void MavServer::handle_send()
{
    data_to_send_access_mtx.lock();

    if (data_to_send_len > 0) {
        sendto(sock, (void *)data_to_send, data_to_send_len, 0,
               (struct sockaddr *)&remote_addr, sizeof(struct sockaddr_in));
        data_to_send_len = 0;
    }

    data_to_send_access_mtx.unlock();
}

void MavServer::handle_recv()
{
    mavlink_message_t msg;
    mavlink_status_t status;

    memset(data_recv, 0, BUFFER_LEN);
    ssize_t bytes_recvd = recvfrom(sock, (void *)data_recv, BUFFER_LEN, 0,
                                   (struct sockaddr *)&remote_addr, &fromlen);

    if(bytes_recvd <= 0) {
        return;
    }

    print_debug_mav("Bytes Received: %d\nDatagram: ", (int)bytes_recvd);

    for (unsigned int i = 0; i < bytes_recvd; ++i) {
        print_debug_mav("%02x ", (unsigned char)data_recv[i]);

        if (mavlink_parse_char(MAVLINK_COMM_0, data_recv[i], &msg, &status)) {

            // Do not handle unexpected mavlink messages
            if (msg.sysid != DEFAULT_TARGET_SYSTEM_ID ||
                msg.compid != DEFAULT_TARGET_COMPONENT_ID) {
                continue;
            }

            print_debug_mav("\nReceived packet: CHK: %d, MGC: %d, SYS: %d, "
                            "COMP: %d, LEN: %d, MSG ID: %d, SEQ: %d\n",
                            msg.checksum, msg.magic, msg.sysid, msg.compid,
                            msg.len, msg.msgid, msg.seq);

            handle_message(&msg);
        }
    }

    print_debug_mav("\n");
}

void MavServer::handle_message(const mavlink_message_t *msg)
{

#if DEBUG_MAVLINK
    static uint32_t prev_time_att = 0;
    static uint32_t prev_time_local_pos = 0;
#endif

    // High Priority Messages - with individual locks
    switch (msg->msgid) {
    case MAVLINK_MSG_ID_LOCAL_POSITION_NED:
        local_pos_ned_svar_access_mtx.lock();
        mavlink_msg_local_position_ned_decode(msg, &local_pos_ned);
        is_new_local_pos_ned = true;
        print_debug_mav("locpos_msg_time = %d\n",
                        local_pos_ned.time_boot_ms - prev_time_local_pos);
        local_pos_ned_svar_access_mtx.unlock();
        return;
    case MAVLINK_MSG_ID_ATTITUDE:
        static int attnum = 1;
        attnum++;
        attitude_svar_access_mtx.lock();
        mavlink_msg_attitude_decode(msg, &attitude);
        is_new_attitude = true;
        print_debug_mav("att_msg_time = %d\n",
                        attitude.time_boot_ms - prev_time_att);
        attitude_svar_access_mtx.unlock();
        return;
    }

    // Low Priority Messages - with global lock
    svar_access_mtx.lock();
    switch (msg->msgid) {
    case MAVLINK_MSG_ID_HOME_POSITION:
        mavlink_msg_home_position_decode(msg, &home_position);
        is_new_home_position = true;
        break;
    case MAVLINK_MSG_ID_COMMAND_ACK:
        mavlink_msg_command_ack_decode(msg, &command_ack);
        is_new_command_ack = true;
        break;
    case MAVLINK_MSG_ID_GPS_RAW_INT:
        mavlink_msg_gps_raw_int_decode(msg, &gps_raw_int);
        is_new_gps_raw_int = true;
        break;
    case MAVLINK_MSG_ID_HEARTBEAT:
        mavlink_msg_heartbeat_decode(msg, &heartbeat);
        is_new_heartbeat = true;
        break;
    case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
        mavlink_msg_global_position_int_decode(msg, &global_pos_int);
        is_new_global_pos_int = true;
        break;
    }
    svar_access_mtx.unlock();
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

        // Make sure the target still exists
        target = model->GetWorld()->GetModel(target_name);

        if (target) {
            tpose_new = target->GetWorldPose();
        }

        // Send Target if exists
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

        // Calculate pose according to new attitude and position
        if (mavserver.is_new_local_pos_ned || mavserver.is_new_attitude) {

            // Set New Drone Pose in Gazebo
            calculate_pose(&curr_pose, mavserver.get_svar_attitude(),
                           mavserver.get_svar_local_pos_ned());
            model->SetWorldPose(curr_pose);
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
    mavlink_global_position_int_t global_pos;

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
