#ifndef __DEFINITIONS_HH__
#define __DEFINITIONS_HH__

// TODO: Set as .sdf plugin parameters
#define GZSITL_TARGET_MODEL_NAME "gzsitl_target"
#define DEBUG_STATE (true)
#define DEBUG_MAVLINK (false)
#define MAVPROXY_IP "127.0.0.1"
#define MAVPROXY_PORT (14556)
#define LOCAL_PORT (14550)
#define DEFAULT_TARGET_SYSTEM_ID (1)    // Default Copter system ID
#define DEFAULT_TARGET_COMPONENT_ID (1) // Default Copter component ID
#define DEFAULT_SYSTEM_ID (22)          // This system ID
#define DEFAULT_COMPONENT_ID (0)        // This component ID
#define HEARTBEAT_SEND_INTERVAL_MS (1000)
#define INIT_POS_NUMSAMPLES (3)
#define TAKEOFF_AUTO (true)
#define TAKEOFF_INIT_ALT_M (0.5)
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

#endif
