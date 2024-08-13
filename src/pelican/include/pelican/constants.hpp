namespace constants {
    // Timings
    static constexpr unsigned int DELAY_MILLIS = 100;
    static constexpr unsigned int ACK_WAIT_MILLIS = 100;
    static constexpr unsigned int P2P_PERIOD_MILLIS = 200;
    static constexpr unsigned int OFFBOARD_PERIOD_MILLIS = 20;
    static constexpr unsigned int FORMATION_PERIOD_MILLIS = 200;
    static constexpr unsigned int RENDEZVOUS_PERIOD_MILLIS = 500;
    static constexpr unsigned int POS_SHARING_PERIOD_MILLIS = 100;
    static constexpr unsigned int HB_REPETITION_PERIOD_MILLIS = 100;
    static constexpr unsigned int ACK_TIME_SECS = 1;
    static constexpr unsigned int ROLLCALL_TIME_SECS = 1;
    static constexpr unsigned int PRECHECKS_TIME_SECS = 10;
    static constexpr unsigned int MAX_SEARCH_TIME_SECS = 10;
    static constexpr unsigned int COMPENSATION_GAP_SECS = 1;
    static constexpr unsigned int MAX_WAITING_TIME_SECS = 2;
    static constexpr unsigned int SEARCH_LEADER_STEP_SECS = 1;
    static constexpr unsigned int SEARCH_SERVER_STEP_SECS = 1;
    static constexpr unsigned int SERVICE_FUTURE_WAIT_SECS = 1;
    static constexpr unsigned int FORMATION_WAIT_TIME_SECS = 3;
    static constexpr unsigned int RENDEZVOUS_WAIT_TIME_SECS = 2;

    // Conversions
    static constexpr double MILLIS_TO_SECS_CONVERSION = 1000.0;
    static constexpr unsigned int NANO_TO_MILLI_CONVERSION = 1000000;

    // Others
    // Standard heartbeat period is 100ms, so this would keep a log of 10s
    static constexpr double STEP_SIZE = 0.5;                 // [m]
    static constexpr double EXTRACTION_HEIGHT = 2.0;         // [m]
    static constexpr double SETPOINT_REACHED_DISTANCE = 0.4; // [m]
    static constexpr unsigned int MAX_HB_NUM = 100;
    static constexpr unsigned int QOS_HISTORY_AMOUNT = 5;
    static constexpr unsigned int MAX_GET_ACK_RETRIES = 10;
    static constexpr unsigned int MAX_BROADCAST_RETRIES = 10;
    static constexpr unsigned int OFFBOARD_SETPOINT_LIMIT = 10;
    static constexpr unsigned int MAX_SEND_COMMAND_RETRIES = 10;

    // Buffers
    static constexpr unsigned int STATUS_BUFFER_SIZE = 10;
    static constexpr unsigned int GLOBALPOS_BUFFER_SIZE = 10;
    static constexpr unsigned int NED_ODOMETRY_BUFFER_SIZE = 10;
    static constexpr unsigned int ENU_ODOMETRY_BUFFER_SIZE = 10;

    // PX4-related
    static constexpr double CONFIRM_SET_HOME = 1.0;
    static constexpr double PX4_LOITER_SUB_MODE = 3.0;
    static constexpr double PX4_CUSTOM_MAIN_MODE = 4.0;
    static constexpr unsigned int PX4_OFFBOARD_MODE = 6;
    static constexpr unsigned int MAVLINK_ENABLE_CUSTOM_MODE = 1;

    // Rendezvous-related
    static constexpr double REND_COLL_WEIGHT = 1.0;
    static constexpr double TARGET_LOC_WEIGHT = 0.2;
    static constexpr double AVOIDANCE_DISTANCE = 1.0;  // [m]
    static constexpr double REND_COLL_THRESHOLD = 1.0; // [m]

    // Formation-related
    static constexpr double FORM_DIST_WEIGHT = 1.0;
    static constexpr double FORM_COLL_WEIGHT = 1.0;
    static constexpr double FORM_CLOSING_VEL = 0.2;    // [m/s]
    static constexpr double FORM_COLL_THRESHOLD = 0.5; // [m]
    static constexpr unsigned int FORM_TARGET_COUNT = 10;
} // namespace constants
