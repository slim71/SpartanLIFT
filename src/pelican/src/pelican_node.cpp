#include "pelican.hpp"

int main(int argc, char *argv[]) {
	std::cout << "Starting pelican_unit listener node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    // Initialization
	rclcpp::init(argc, argv);

    // Instantiation
	rclcpp::spin(std::make_shared<PelicanUnit>());

	rclcpp::shutdown();
	return 0;
}