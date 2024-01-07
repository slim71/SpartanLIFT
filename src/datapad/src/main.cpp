#include "datapad.hpp"
#include "types.hpp"

int main(int argc, char* argv[]) {
    std::cout << "Starting Datapad node..." << std::endl;
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    // Initialization
    rclcpp::init(argc, argv);

    // Instantiation
    rclcpp::Node::SharedPtr node = std::make_shared<Datapad>();
    rclcpp::executors::MultiThreadedExecutor executor;

    // Set the instance pointer to the shared pointer of the main node
    Datapad::setInstance(node);
    // Register the signal handler for SIGINT (CTRL+C)
    auto ret = std::signal(SIGINT, Datapad::signalHandler);
    if (ret == SIG_ERR) {
        std::cout << "Error while setting signal handler!" << std::endl;
        return -1;
    }

    executor.add_node(node);

    try {
        executor.spin();
    } catch (std::exception& e) {
        std::cout << "rclcpp shutting down..." << std::endl;
        rclcpp::shutdown();
        return 0;
    }
}
