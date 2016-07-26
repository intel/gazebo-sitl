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

// TODO: Set as .sdf plugin parameters
#define GZSITL_TARGET_MODEL_NAME "gzsitl_target"
#define DEBUG_STATE (true)
#define DEBUG_MAVLINK (false)
#define MAVPROXY_PORT (14556)
#define DEFAULT_TARGET_SYSTEM_ID (1)    // Default Copter system ID
#define DEFAULT_TARGET_COMPONENT_ID (1) // Default Copter component ID
#define DEFAULT_SYSTEM_ID (22)          // This system ID
#define DEFAULT_COMPONENT_ID (0)        // This component ID
#define HEARTBEAT_SEND_INTERVAL_MS (1000)
#define INIT_POS_NUMSAMPLES (3)
#define TAKEOFF_AUTO (true)
#define TAKEOFF_INIT_ALT_M (0.5)
#define COND_YAW_REQUEST_INTERVAL_MS (2000)
#define HOME_POSITION_REQUEST_INTERVAL_MS (3000)
#define MODE_SET_REQUEST_INTERVAL_MS (3000)
#define TAKEOFF_REQUEST_INTERVAL_MS (3000)

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

