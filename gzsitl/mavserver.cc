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

#include "mavserver.hh"

#include <fcntl.h>
#include <unistd.h>
#include <unordered_map>

#include "defines.hh"

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

        print_debug_mav("Changing to GUIDED mode...\n");
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
    std::lock_guard<std::mutex> locker(data_to_send_access_mtx);

    int safe_data_len =
        std::min((int)data_len, (int)(BUFFER_LEN - data_to_send_len));

    if (safe_data_len < data_len)
        return;

    memcpy(&data_to_send[data_to_send_len], data,
        safe_data_len * sizeof(*data));
    data_to_send_len += safe_data_len;
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
            print_debug_mav("Arming Throttle...\n");
            break;
        case MAV_CMD_NAV_TAKEOFF:
            print_debug_mav("Requesting Takeoff...\n");
            break;
        case MAV_CMD_GET_HOME_POSITION:
            print_debug_mav("Requesting Home Position...\n");
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
    std::lock_guard<std::mutex> locker(attitude_svar_access_mtx);
    is_new_attitude = false;
    return attitude;
}

mavlink_heartbeat_t MavServer::get_svar_heartbeat()
{
    std::lock_guard<std::mutex> locker(svar_access_mtx);
    is_new_heartbeat = false;
    return heartbeat;
}

mavlink_command_ack_t MavServer::get_svar_command_ack()
{
    std::lock_guard<std::mutex> locker(svar_access_mtx);
    is_new_command_ack = false;
    return command_ack;
}

mavlink_gps_raw_int_t MavServer::get_svar_gps_raw_int()
{
    std::lock_guard<std::mutex> locker(svar_access_mtx);
    is_new_gps_raw_int = false;
    return gps_raw_int;
}

mavlink_home_position_t MavServer::get_svar_home_position()
{
    std::lock_guard<std::mutex> locker(svar_access_mtx);
    is_new_home_position = false;
    return home_position;
}

mavlink_local_position_ned_t MavServer::get_svar_local_pos_ned()
{
    std::lock_guard<std::mutex> locker(local_pos_ned_svar_access_mtx);
    is_new_local_pos_ned = false;
    return local_pos_ned;
}

mavlink_global_position_int_t MavServer::get_svar_global_pos_int()
{
    std::lock_guard<std::mutex> locker(svar_access_mtx);
    is_new_global_pos_int = false;
    return global_pos_int;
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
    std::lock_guard<std::mutex> locker(data_to_send_access_mtx);

    if (data_to_send_len <= 0)
        return;

    sendto(sock, (void *)data_to_send, data_to_send_len, 0,
        (struct sockaddr *)&remote_addr, sizeof(struct sockaddr_in));
    data_to_send_len = 0;
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
    case MAVLINK_MSG_ID_LOCAL_POSITION_NED: {
        std::lock_guard<std::mutex> locker(local_pos_ned_svar_access_mtx);
        mavlink_msg_local_position_ned_decode(msg, &local_pos_ned);
        is_new_local_pos_ned = true;
        print_debug_mav("locpos_msg_time = %d\n",
                        local_pos_ned.time_boot_ms - prev_time_local_pos);
        return;
    }
    case MAVLINK_MSG_ID_ATTITUDE: {
        static int attnum = 1;
        attnum++;
        std::lock_guard<std::mutex> locker_svar(attitude_svar_access_mtx);
        mavlink_msg_attitude_decode(msg, &attitude);
        is_new_attitude = true;
        print_debug_mav("att_msg_time = %d\n",
                        attitude.time_boot_ms - prev_time_att);
        return;
    }
    }

    // Low Priority Messages - with global lock
    std::lock_guard<std::mutex> locker(svar_access_mtx);
    switch (msg->msgid) {
    case MAVLINK_MSG_ID_HOME_POSITION: {
        mavlink_msg_home_position_decode(msg, &home_position);
        is_new_home_position = true;
        break;
    }
    case MAVLINK_MSG_ID_COMMAND_ACK: {
        mavlink_msg_command_ack_decode(msg, &command_ack);
        is_new_command_ack = true;
        break;
    }
    case MAVLINK_MSG_ID_GPS_RAW_INT: {
        mavlink_msg_gps_raw_int_decode(msg, &gps_raw_int);
        is_new_gps_raw_int = true;
        break;
    }
    case MAVLINK_MSG_ID_HEARTBEAT: {
        mavlink_msg_heartbeat_decode(msg, &heartbeat);
        is_new_heartbeat = true;
        break;
    }
    case MAVLINK_MSG_ID_GLOBAL_POSITION_INT: {
        mavlink_msg_global_position_int_decode(msg, &global_pos_int);
        is_new_global_pos_int = true;
        break;
    }
    }
}
