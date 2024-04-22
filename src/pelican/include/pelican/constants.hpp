namespace constants {
    static constexpr unsigned int DELAY_MILLIS = 10;
    static constexpr unsigned int ROLLCALL_TIME_SECS = 1;
    static constexpr unsigned int ACK_TIME_SECS = 1;
    static constexpr unsigned int PRECHECKS_TIME_SECS = 10;
    static constexpr unsigned int HB_REPETITION_PERIOD_MILLIS = 100;
    static constexpr unsigned int ACK_WAIT_MILLIS = 100;
    static constexpr unsigned int OFFBOARD_PERIOD_MILLIS = 20;
    static constexpr unsigned int RENDEZVOUS_WAIT_TIME_SECS = 2;
    static constexpr unsigned int SEARCH_LEADER_STEP_SECS = 1;
    static constexpr unsigned int MAX_SEARCH_TIME_SECS = 10;
    static constexpr unsigned int SERVICE_FUTURE_WAIT_SECS = 1;
    static constexpr unsigned int COMPENSATION_GAP_SECS = 1;
    static constexpr unsigned int POS_SHARING_PERIOD_MILLIS = 100;
    static constexpr unsigned int RENDEZVOUS_CONSENSUS_PERIOD_MILLIS = 500;

    // Standard heartbeat period is 100ms, so this would keep a log of 10s
    static constexpr unsigned int MAX_HB_NUM = 100;
    static constexpr unsigned int MAX_BROADCAST_RETRIES = 10;
    static constexpr unsigned int MAX_GET_ACK_RETRIES = 10;
    static constexpr unsigned int MAX_SEND_COMMAND_RETRIES = 10;
    static constexpr unsigned int OFFBOARD_SETPOINT_LIMIT = 10;
    static constexpr unsigned int QOS_HISTORY_AMOUNT = 5;

    static constexpr unsigned int GLOBALPOS_BUFFER_SIZE = 10;
    static constexpr unsigned int NED_ODOMETRY_BUFFER_SIZE = 10;
    static constexpr unsigned int ENU_ODOMETRY_BUFFER_SIZE = 10;
    static constexpr unsigned int STATUS_BUFFER_SIZE = 10;

    static constexpr unsigned int NANO_TO_MILLI_ORDER_CONVERSION = 1000000;

    static constexpr unsigned int MAVLINK_ENABLE_CUSTOM_MODE = 1;
    static constexpr unsigned int PX4_OFFBOARD_MODE = 6;
    static constexpr double CONFIRM_SET_HOME = 1.0;

    // Up to and including C++17, pi is not a constant introduced into the language.
    // It is present in C++20, but switching standard just for a constant seems a bit
    // of an overkill. Have it hardcoded.
    static constexpr double PI = 3.14159265358979323846;

    static constexpr double ALPHA_REND = 0.2;
    static constexpr double BETA_REND = 1.0;
    static constexpr double AVOIDANCE_DISTANCE = 1.0;
} // namespace constants
