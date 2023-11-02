#include "PelicanModule/pelican.hpp"
#include "pugixml.hpp"

// Initialize the static instance pointer to a weak pointer
std::weak_ptr<Pelican> Pelican::instance_;

/************************** Ctors/Dctors ***************************/
Pelican::Pelican() : Node("Pelican"), logger_(), hb_core_(this), el_core_(this), tac_core_(this) {
    // Declare parameters
    declare_parameter("model", ""); // default to ""
    declare_parameter("id", 0);     // default to 0

    // Get parameters values and store them
    get_parameter("model", this->model_);
    get_parameter("id", this->id_);

    // Setting up the Reentrant group
    this->reentrant_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    this->reentrant_opt_.callback_group = this->reentrant_group_;

    this->logger_.initSetup(std::make_shared<rclcpp::Logger>(this->get_logger()), this->getID());
    this->hb_core_.initSetup(&(this->logger_));
    this->el_core_.initSetup(&(this->logger_));
    this->tac_core_.initSetup(&(this->logger_));

    this->parseModel();

    // Log parameters values
    this->sendLogInfo("Loaded model {} | Agent mass: {}", this->getModel(), this->getMass());

    this->ready_ = true;
    this->sendLogInfo("Node ready!");

    this->becomeFollower();
}

Pelican::~Pelican() {
    this->sendLogDebug("Destructor for agent {}", this->getID());

    // Reset shared pointers
    this->instance_.reset();
}

/********************** Core functionalities ***********************/
void Pelican::parseModel() {
    this->sendLogDebug("Trying to load model {}", this->getModel());
    pugi::xml_document doc;
    pugi::xml_parse_result result = doc.load_file(this->getModel().c_str());

    if (result) {
        pugi::xml_node start = doc.child("sdf").child("model");

        for (pugi::xml_node link = start.first_child(); link; link = link.next_sibling()) {
            if (strcmp(link.attribute("name").value(), "base_link") == 0) {
                this->setMass(link.child("inertial").child("mass").text().as_double());
            }
        }
    } else {
        this->sendLogError(
            "Model file {} could not be loaded! Error description: {}", this->getModel(),
            result.description()
        );
        // Abort everything
        throw std::runtime_error("Agent model could not be parsed!");
    }
}

void Pelican::signalHandler(int signum) {
    // Stop the thread gracefully
    std::shared_ptr<Pelican> node = getInstance();
    if (node) {
        node->hb_core_.stopHeartbeat();
        node->el_core_.stopBallotThread();
        node->tac_core_.stopData();
    }

    rclcpp::shutdown();
}
