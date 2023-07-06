#include "pelican.hpp"

int main(int argc, char *argv[]) {
	std::cout << "Starting pelican_unit listener node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    // Initialization
	rclcpp::init(argc, argv);

	try {
		// Instantiation
		rclcpp::Node::SharedPtr node = std::make_shared<PelicanUnit>();
		rclcpp::executors::MultiThreadedExecutor executor;
		
		executor.add_node(node);

		// TODO: try-catch needed for the spin() call too?
		executor.spin();

	} catch (...) {
		std::cout << "rclcpp shutting down..." << std::endl;
		rclcpp::shutdown();
		return 0;
	}

}