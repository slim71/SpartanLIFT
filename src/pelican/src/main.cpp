#include "PelicanModule/pelican.hpp"

int main(int argc, char* argv[]) {
    std::cout << "Starting Pelican node..." << std::endl;
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    // Initialization
    rclcpp::init(argc, argv);

    // Instantiation
    rclcpp::Node::SharedPtr node = std::make_shared<Pelican>();
    rclcpp::executors::MultiThreadedExecutor executor;

    // Set the instance pointer to the shared pointer of the main node
    Pelican::setInstance(node);
    // Register the signal handler for SIGINT (CTRL+C)
    std::signal(SIGINT, Pelican::signalHandler);

    executor.add_node(node);

    try {
        executor.spin();
    } catch (std::exception& e) {
        std::cout << "rclcpp shutting down..." << std::endl;
        rclcpp::shutdown();
        return 0;
    }
}
