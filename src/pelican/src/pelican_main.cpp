#include "pelican.hpp"

int main(int argc, char* argv[]) {
    std::cout << "Starting PelicanUnit node..." << std::endl;
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    // Initialization
    rclcpp::init(argc, argv);

    // Instantiation
    rclcpp::Node::SharedPtr node = std::make_shared<PelicanUnit>();
    rclcpp::executors::MultiThreadedExecutor executor;

    // Set the instance pointer to the shared pointer of the main node
    PelicanUnit::setInstance(node);
    // Register the signal handler for SIGINT (CTRL+C)
    signal(SIGINT, PelicanUnit::signalHandler);

    executor.add_node(node);

    try {
        executor.spin();
    } catch (std::exception& e) {
        std::cout << "rclcpp shutting down..." << std::endl;
        rclcpp::shutdown();
        return 0;
    }
}
