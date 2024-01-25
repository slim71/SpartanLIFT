#include "PelicanModule/pelican.hpp"

// Initialize the static instance pointer to a weak pointer
std::weak_ptr<Pelican> Pelican::instance_;

/************************** Ctors/Dctors ***************************/
Pelican::Pelican()
    : Node("Pelican"),
      logger_(),
      hb_core_(this),
      el_core_(this),
      tac_core_(this),
      unsc_core_(this) {
    // Declare parameters
    declare_parameter("model", ""); // default to ""
    declare_parameter("id", 0);     // default to 0

    // Get parameters values and store them
    get_parameter("model", this->model_);
    get_parameter("id", this->id_);

    // Setting up the Reentrant group
    this->reentrant_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    this->reentrant_opt_.callback_group = this->reentrant_group_;

    // Own setup
    this->sub_to_discovery_ = this->create_subscription<comms::msg::Status>(
        this->discovery_topic_, this->data_qos_,
        std::bind(&Pelican::storeAttendance, this, std::placeholders::_1), this->reentrant_opt_
    );
    this->sub_to_dispatch_ = this->create_subscription<comms::msg::Command>(
        this->dispatch_topic_, this->qos_,
        std::bind(&Pelican::handleCommandReception, this, std::placeholders::_1),
        this->reentrant_opt_
    );
    this->pub_to_discovery_ =
        this->create_publisher<comms::msg::Status>(this->discovery_topic_, this->data_qos_);
    this->pub_to_dispatch_ =
        this->create_publisher<comms::msg::Command>(this->dispatch_topic_, this->qos_);

    // Other modules' setup
    this->logger_.initSetup(std::make_shared<rclcpp::Logger>(this->get_logger()), this->getID());
    this->hb_core_.initSetup(&(this->logger_));
    this->el_core_.initSetup(&(this->logger_));
    this->tac_core_.initSetup(&(this->logger_));
    this->unsc_core_.initSetup(&(this->logger_));

    // Parse SDF model
    this->parseModel();

    // Log parameters values
    this->sendLogInfo("Loaded model {} | Agent mass: {}", this->getModel(), this->getMass());

    // Wait and notify presence to other agents
    std::this_thread::sleep_for(std::chrono::seconds(1));
    this->rollCall();

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
    // Only handle SIGINT
    if (signum == SIGINT) {
        // Stop the thread gracefully
        std::shared_ptr<Pelican> node = getInstance();
        if (node) {
            node->hb_core_.stopService();
            node->el_core_.stopService();
            node->tac_core_.stopService();
            node->unsc_core_.stopService();
        }

        rclcpp::shutdown();
    }
}

void Pelican::rollCall() {
    comms::msg::Status msg;
    msg.agent_id = this->getID();
    msg.status = true;

    this->sendLogInfo("Notifying presence to the fleet");
    this->pub_to_discovery_->publish(msg);
}

void Pelican::storeAttendance(comms::msg::Status::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(this->discovery_mutex_);

    // CHECK: do not consider this node here, then count it already in the fleet?
    if (std::find_if_not(
            this->discovery_vector_.begin(), this->discovery_vector_.end(),
            [msg](const comms::msg::Status& obj) {
                return msg->agent_id != obj.agent_id;
            }
        ) == this->discovery_vector_.end()) {
        this->sendLogDebug("Storing discovery msg with id {}", msg->agent_id);
        this->discovery_vector_.push_back(*msg);
    } else {
        this->sendLogDebug("Agent {} already discovered", msg->agent_id);
    }
}
