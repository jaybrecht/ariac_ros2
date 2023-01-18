// Copyright 2018 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// Gazebo
#include <gazebo/physics/World.hh>
#include <gazebo_ros/node.hpp>

// Messages
#include <ariac_plugins/task_manager_plugin.hpp>
#include <ariac_msgs/msg/trial.hpp>
#include <ariac_msgs/msg/order.hpp>
#include <ariac_msgs/msg/order_condition.hpp>
#include <ariac_msgs/msg/challenge.hpp>
#include <ariac_msgs/msg/sensors.hpp>
#include <ariac_msgs/msg/kitting_task.hpp>
#include <ariac_msgs/msg/assembly_task.hpp>
#include <ariac_msgs/msg/robots.hpp>
#include <ariac_msgs/msg/competition_state.hpp>
// ROS
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executor.hpp>
#include <controller_manager_msgs/srv/switch_controller.hpp>

// C++
#include <memory>
// ARIAC
#include <ariac_plugins/ariac_common.hpp>

// #include <ariac_msgs/msg/kitting_task.hpp>
// #include <ariac_msgs/msg/assembly_task.hpp>
// #include <ariac_msgs/msg/part.hpp>
// #include <std_msgs/msg/string.hpp>
// #include <ariac_plugins/ariac_common.hpp>

namespace ariac_plugins
{
    /// Class to hold private data members (PIMPL pattern)
    class TaskManagerPluginPrivate
    {
    public:
        //============== C++ =================
        /*!< A mutex to protect the current state. */
        std::mutex lock_;
        /*!< Pointer to the current state. */
        unsigned int current_state_ = ariac_msgs::msg::CompetitionState::UNSTARTED;

        /*!< Time limit for the current trial. */
        double time_limit_{-1};
        /*!< Name of the trial file. */
        std::string trial_name_;

        //============== GAZEBO =================
        /*!< Connection to world update event. Callback is called while this is alive. */
        gazebo::event::ConnectionPtr update_connection_;
        /*!< ROS node. */
        gazebo_ros::Node::SharedPtr ros_node_{nullptr};
        /*!< Pointer to the world. */
        gazebo::physics::WorldPtr world_;
        /*!< Pointer to the sdf tag. */
        sdf::ElementPtr sdf_;
        /*!< Time since the start competition service is called. */
        gazebo::common::Time start_competition_time_;
        gazebo::common::Time last_sim_time_;
        gazebo::common::Time last_on_update_time_;

        //============== ROS =================
        /*!< Time when this plugin is loaded. */
        rclcpp::Time current_sim_time_;

        //============== SUBSCRIBERS =================
        /*!< Subscriber to topic: "trial_config"*/
        rclcpp::Subscription<ariac_msgs::msg::Trial>::SharedPtr trial_config_sub_;

        //============== SERVICES =================
        /*!< Service that allows the user to start the competition. */
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_competition_srv_{nullptr};
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr end_competition_srv_{nullptr};
        /*!< Client to start/stop robot controllers. */
        rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr switch_controller_srv_client_;
        /*!< Client to stop the competition. */
        rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr end_competition_srv_client_;

        //============== PUBLISHERS =================
        /*!< Publisher to the topic /ariac/orders */
        rclcpp::Publisher<ariac_msgs::msg::Order>::SharedPtr order_pub_;
        /*!< Publisher to the topic /ariac/sensor_health */
        rclcpp::Publisher<ariac_msgs::msg::Sensors>::SharedPtr sensor_health_pub_;
        /*!< Publisher to the topic /ariac/competition_state */
        rclcpp::Publisher<ariac_msgs::msg::CompetitionState>::SharedPtr competition_state_pub_;
        /*!< Publisher to the topic /ariac/robot_health */
        rclcpp::Publisher<ariac_msgs::msg::Robots>::SharedPtr robot_health_pub_;

        //============== LISTS OF ORDERS AND CHALLENGES =================
        /*!< List of orders that are announced based on time. */
        std::vector<std::shared_ptr<ariac_common::OrderTemporal>> time_based_orders_;
        /*!< List of orders that are announced based on part placement. */
        std::vector<std::shared_ptr<ariac_common::OrderOnPartPlacement>> on_part_placement_orders_;
        /*!< List of orders that are announced based on submission. */
        std::vector<std::shared_ptr<ariac_common::OrderOnSubmission>> on_order_submission_orders_;
    };
    //==============================================================================
    TaskManagerPlugin::TaskManagerPlugin()
        : impl_(std::make_unique<TaskManagerPluginPrivate>())
    {
    }
    //==============================================================================
    TaskManagerPlugin::~TaskManagerPlugin()
    {
        impl_->ros_node_.reset();
    }
    //==============================================================================
    // void TaskManagerPlugin::ManageOrders(std::vector<ariac_msgs::msg::Order> _orders)
    // {
    // for (auto order : _orders)
    // {
    //     ariac_common::Order new_order;
    //     new_order.order_id_ = order.id;
    //     new_order.order_type_ = order.type;
    //     new_order.high_priority_ = order.priority;
    //     new_order.announced_condition_ = order.announcement_condition;
    //     new_order.announced_value_ = order.announcement_value;

    //     // KittingTask

    //     // AssemblyTask

    //     impl_->time_based_orders_.push_back(new_order);
    // }
    // }
    //==============================================================================
    void
    TaskManagerPlugin::Load(gazebo::physics::WorldPtr _world, sdf::ElementPtr _sdf)
    {

        GZ_ASSERT(_world, "TaskManagerPlugin world pointer is NULL");
        GZ_ASSERT(_sdf, "TaskManagerPlugin sdf pointer is NULL");

        // // Create a GazeboRos node instead of a common ROS node.
        // // Pass it SDF parameters so common options like namespace and remapping
        // // can be handled.
        impl_->ros_node_ = gazebo_ros::Node::Get(_sdf);
        impl_->world_ = _world;
        impl_->sdf_ = _sdf;

        RCLCPP_INFO(impl_->ros_node_->get_logger(), "Starting ARIAC 2023");

        // Get QoS profiles
        const gazebo_ros::QoS &qos = impl_->ros_node_->get_qos();

        // Create a connection so the OnUpdate function is called at every simulation
        // iteration. Remove this call, the connection and the callback if not needed.
        impl_->update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
            std::bind(&TaskManagerPlugin::OnUpdate, this));

        impl_->trial_config_sub_ = impl_->ros_node_->create_subscription<ariac_msgs::msg::Trial>(
            "/ariac/trial_config", qos.get_subscription_qos("/ariac/trial_config", rclcpp::QoS(1)),
            std::bind(&TaskManagerPlugin::OnTrialCallback, this, std::placeholders::_1));

        impl_->sensor_health_pub_ = impl_->ros_node_->create_publisher<ariac_msgs::msg::Sensors>("/ariac/sensor_health", 10);

        // Initialize service clients
        impl_->switch_controller_srv_client_ = impl_->ros_node_->create_client<controller_manager_msgs::srv::SwitchController>("/controller_manager/switch_controller");
        impl_->end_competition_srv_client_ = impl_->ros_node_->create_client<std_srvs::srv::Trigger>("/ariac/end_competition");
        // impl_->current_sim_time_ = impl_->ros_node_->get_clock()->now();
        // impl_->current_sim_time_ = impl_->world_->SimTime();
    }

    //==============================================================================
    // void TaskManagerPlugin::AnnounceOrder(std::shared_ptr<ariac_common::Order> _order)
    // {
    //     auto message = ariac_msgs::msg::Order();
    //     message.id = _order->GetId();
    //     message.type = _order->GetType();
    //     message.priority = _order->GetPriority();
    // }

    //==============================================================================
    // void TaskManagerPlugin::ProcessOrdersToAnnounce(double _current_sim_time)
    // {
    //     // ProcessTemporalOrders(_current_sim_time);
    //     // ProcessDuringKittingOrders(_current_sim_time);
    //     // ProcessDuringAssemblyOrders(_current_sim_time);
    //     // ProcessAfterKittingOrders(_current_sim_time);
    //     // ProcessAfterAssemblyOrders(_current_sim_time);
    // }

    //==============================================================================
    void TaskManagerPlugin::OnUpdate()
    {
        std::lock_guard<std::mutex> lock(impl_->lock_);
        auto current_sim_time = impl_->world_->SimTime();

        // Delay advertising the competition start service to avoid a crash.
        // Sometimes if the competition is started before the world is fully loaded, it causes a crash.
        if (!impl_->start_competition_srv_ && current_sim_time.Double() >= 5.0)
        {
            // create the start competition service
            impl_->start_competition_srv_ = impl_->ros_node_->create_service<std_srvs::srv::Trigger>("/ariac/start_competition",
                                                                                                     std::bind(&TaskManagerPlugin::StartCompetitionServiceCallback,
                                                                                                               this,
                                                                                                               std::placeholders::_1,
                                                                                                               std::placeholders::_2));

            impl_->end_competition_srv_ = impl_->ros_node_->create_service<std_srvs::srv::Trigger>("/ariac/end_competition",
                                                                                                    std::bind(&TaskManagerPlugin::EndCompetitionServiceCallback,
                                                                                                              this,
                                                                                                              std::placeholders::_1,
                                                                                                              std::placeholders::_2));

            RCLCPP_INFO_STREAM(impl_->ros_node_->get_logger(), "/ariac/start_competition service created");
            // RCLCPP_INFO_STREAM(impl_->ros_node_->get_logger(), "updated time: " << updated_sim_time.seconds());
        }

        if ((current_sim_time - impl_->last_sim_time_).Double() >= 1.0)
        {
            impl_->last_sim_time_ = current_sim_time;
        }

        auto elapsed_time = (current_sim_time - impl_->last_on_update_time_).Double();
        if (impl_->time_limit_ >= 0 && impl_->current_state_ == ariac_msgs::msg::CompetitionState::STARTED &&
            (current_sim_time - impl_->start_competition_time_) > impl_->time_limit_)
        {
            impl_->current_state_ = ariac_msgs::msg::CompetitionState::ENDED;

            // Call the end competition service
            auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
            auto result_future = impl_->end_competition_srv_client_->async_send_request(request);
            if (rclcpp::spin_until_future_complete(impl_->ros_node_, result_future) !=
                rclcpp::FutureReturnCode::SUCCESS)
            {
                RCLCPP_ERROR(impl_->ros_node_->get_logger(), "End competition service call failed");
            }
        }

        // current state is set to "ready" in start competition service callback
        if (impl_->current_state_ == ariac_msgs::msg::CompetitionState::UNSTARTED)
        {
            impl_->start_competition_time_ = current_sim_time;
            impl_->current_state_ = ariac_msgs::msg::CompetitionState::STARTED;
        }

        if (impl_->current_state_ == ariac_msgs::msg::CompetitionState::STARTED)
        {
            // ProcessOrdersToAnnounce((current_sim_time - impl_->start_competition_time_).Double());
        }

        impl_->last_on_update_time_ = current_sim_time;
    }

    //==============================================================================
    // std::shared_ptr<ariac_common::KittingTask> static BuildKittingTask(const ariac_msgs::msg::KittingTask &_kitting_task)
    // {
    //     // auto agv_number = _kitting_task.agv_number;
    //     // auto tray_id = _kitting_task.tray_id;
    //     // auto destination = ariac_common::ConvertDestinationToString(_kitting_task.destination, agv_number);
    //     // std::vector<ariac_common::KittingPart> kitting_parts;

    //     // for (auto product : _kitting_task.products)
    //     // {
    //     //     auto quadrant = product.quadrant;
    //     //     auto part_type = ariac_common::ConvertPartTypeToString(product.part.type);
    //     //     auto part_color = ariac_common::ConvertPartColorToString(product.part.color);
    //     //     auto part = ariac_common::Part(part_type, part_color);
    //     //     kitting_parts.emplace_back(ariac_common::KittingPart(quadrant, part));
    //     // }
    //     // return std::make_shared<ariac_common::KittingTask>(agv_number, tray_id, destination, kitting_parts);
    // }

    //==============================================================================
    void TaskManagerPlugin::OnTrialCallback(const ariac_msgs::msg::Trial::SharedPtr _msg)
    {
        std::lock_guard<std::mutex> scoped_lock(impl_->lock_);
        // RCLCPP_FATAL_STREAM(impl_->ros_node_->get_logger(), "------Time limit: " << _msg->time_limit);
        // RCLCPP_FATAL_STREAM(impl_->ros_node_->get_logger(), "------Trial name: " << _msg->trial_name);
        impl_->time_limit_ = _msg->time_limit;
        impl_->trial_name_ = _msg->trial_name;

        // Store orders to be processed later
        std::vector<std::shared_ptr<ariac_msgs::msg::OrderCondition>> order_conditions;
        for (auto order_condition : _msg->order_conditions)
        {
            order_conditions.push_back(std::make_shared<ariac_msgs::msg::OrderCondition>(order_condition));
        }
        StoreOrders(order_conditions);
        RCLCPP_INFO_STREAM(impl_->ros_node_->get_logger(), "Number of temporal orders: " << impl_->time_based_orders_.size());
        RCLCPP_INFO_STREAM(impl_->ros_node_->get_logger(), "Number of part place orders: " << impl_->on_part_placement_orders_.size());
        RCLCPP_INFO_STREAM(impl_->ros_node_->get_logger(), "Number of submission orders: " << impl_->on_order_submission_orders_.size());

        // Store challenges to be processed later
        if (_msg->challenges.size() > 0)
        {
            std::vector<std::shared_ptr<ariac_msgs::msg::Challenge>> challenges;
            for (auto challenge : _msg->challenges)
            {
                challenges.push_back(std::make_shared<ariac_msgs::msg::Challenge>(challenge));
            }
        }
    }

    //==============================================================================
    std::shared_ptr<ariac_common::KittingTask> TaskManagerPlugin::BuildKittingTask(const ariac_msgs::msg::KittingTask &_kitting_task)
    {
        auto agv_number = _kitting_task.agv_number;
        auto tray_id = _kitting_task.tray_id;
        auto destination = _kitting_task.destination;
        std::vector<ariac_common::KittingPart> kitting_parts;

        for (auto product : _kitting_task.parts)
        {
            auto quadrant = product.quadrant;
            auto part_type = product.part.type;
            auto part_color = product.part.color;
            auto part = ariac_common::Part(part_type, part_color);
            kitting_parts.emplace_back(ariac_common::KittingPart(quadrant, part));
        }
        return std::make_shared<ariac_common::KittingTask>(agv_number, tray_id, destination, kitting_parts);
    }

    //==============================================================================
    std::shared_ptr<ariac_common::AssemblyTask> TaskManagerPlugin::BuildAssemblyTask(const ariac_msgs::msg::AssemblyTask &_assembly_task)
    {
        std::vector<unsigned int> agv_numbers = {};
        for (auto agv_number : _assembly_task.agv_numbers)
        {
            agv_numbers.push_back(agv_number);
        }
        auto station = _assembly_task.station;
        std::vector<ariac_common::AssemblyPart> assembly_parts;

        for (auto product : _assembly_task.parts)
        {
            auto part_type = product.part.type;
            auto part_color = product.part.color;
            auto part = ariac_common::Part(part_type, part_color);

            ignition::math::Pose3d assembled_pose;
            assembled_pose.Pos().X() = product.assembled_pose.pose.position.x;
            assembled_pose.Pos().Y() = product.assembled_pose.pose.position.y;
            assembled_pose.Pos().Z() = product.assembled_pose.pose.position.z;
            assembled_pose.Rot().X() = product.assembled_pose.pose.orientation.x;
            assembled_pose.Rot().Y() = product.assembled_pose.pose.orientation.y;
            assembled_pose.Rot().Z() = product.assembled_pose.pose.orientation.z;
            assembled_pose.Rot().W() = product.assembled_pose.pose.orientation.w;

            ignition::math::Vector3<double> part_direction;
            part_direction.X() = product.install_direction.x;
            part_direction.Y() = product.install_direction.y;
            part_direction.Z() = product.install_direction.z;

            assembly_parts.emplace_back(ariac_common::AssemblyPart(part, assembled_pose, part_direction));
        }
        return std::make_shared<ariac_common::AssemblyTask>(agv_numbers, station, assembly_parts);
    }

    //==============================================================================
    std::shared_ptr<ariac_common::CombinedTask> TaskManagerPlugin::BuildCombinedTask(const ariac_msgs::msg::CombinedTask &_combined_task)
    {
        auto station = _combined_task.station;
        std::vector<ariac_common::AssemblyPart> assembly_parts;

        for (auto product : _combined_task.parts)
        {
            auto part_type = product.part.type;
            auto part_color = product.part.color;
            auto part = ariac_common::Part(part_type, part_color);

            ignition::math::Pose3d assembled_pose;
            assembled_pose.Pos().X() = product.assembled_pose.pose.position.x;
            assembled_pose.Pos().Y() = product.assembled_pose.pose.position.y;
            assembled_pose.Pos().Z() = product.assembled_pose.pose.position.z;
            assembled_pose.Rot().X() = product.assembled_pose.pose.orientation.x;
            assembled_pose.Rot().Y() = product.assembled_pose.pose.orientation.y;
            assembled_pose.Rot().Z() = product.assembled_pose.pose.orientation.z;
            assembled_pose.Rot().W() = product.assembled_pose.pose.orientation.w;

            ignition::math::Vector3<double> part_direction;
            part_direction.X() = product.install_direction.x;
            part_direction.Y() = product.install_direction.y;
            part_direction.Z() = product.install_direction.z;

            assembly_parts.emplace_back(ariac_common::AssemblyPart(part, assembled_pose, part_direction));
        }
        return std::make_shared<ariac_common::CombinedTask>(station, assembly_parts);
    }

    //==============================================================================
    void TaskManagerPlugin::StoreOrders(const std::vector<std::shared_ptr<ariac_msgs::msg::OrderCondition>> &orders)
    {
        for (auto order : orders)
        {
            auto order_id = order->id;
            auto order_type = order->type;
            auto order_priority = order->priority;

            // Pointers to the tasks
            std::shared_ptr<ariac_common::KittingTask> kitting_task = nullptr;
            std::shared_ptr<ariac_common::AssemblyTask> assembly_task = nullptr;
            std::shared_ptr<ariac_common::CombinedTask> combined_task = nullptr;

            if (order_type == ariac_common::OrderType::KITTING)
            {
                // RCLCPP_ERROR_STREAM(impl_->ros_node_->get_logger(), "Kitting task: " << order_id);
                kitting_task = BuildKittingTask(order->kitting_task);
            }
            else if (order_type == ariac_common::OrderType::ASSEMBLY)
            {
                // RCLCPP_ERROR_STREAM(impl_->ros_node_->get_logger(), "Assembly task: " << order_id);
                assembly_task = BuildAssemblyTask(order->assembly_task);
            }
            else if (order_type == ariac_common::OrderType::COMBINED)
            {
                // RCLCPP_ERROR_STREAM(impl_->ros_node_->get_logger(), "Combined task: " << order_id);
                combined_task = BuildCombinedTask(order->combined_task);
            }
            else
            {
                RCLCPP_ERROR_STREAM(impl_->ros_node_->get_logger(), "Unknown order type: " << int(order_type));
            }

            // std::vector<std::shared_ptr<ariac_common::OrderOnSubmission>> on_order_submission_orders_;

            // Get the condition
            if (order->condition.type == ariac_common::ConditionType::TIME)
            {
                auto announcement_time = order->condition.time_condition.seconds;
                auto order_instance = std::make_shared<ariac_common::OrderTemporal>(order_id, order_type, order_priority, impl_->time_limit_, announcement_time);
                // Set the task
                if (!kitting_task)
                    order_instance->SetKittingTask(kitting_task);
                else if (!assembly_task)
                    order_instance->SetAssemblyTask(assembly_task);
                else if (!combined_task)
                    order_instance->SetCombinedTask(combined_task);

                // Add to the list of time based orders
                impl_->time_based_orders_.push_back(order_instance);
            }
            else if (order->condition.type == ariac_common::ConditionType::PART_PLACE)
            {
                auto agv = order->condition.part_place_condition.agv;
                auto part = std::make_shared<ariac_common::Part>(order->condition.part_place_condition.part.type, order->condition.part_place_condition.part.color);
                auto order_instance = std::make_shared<ariac_common::OrderOnPartPlacement>(order_id, order_type, order_priority, impl_->time_limit_, agv, part);
                // Set the task
                if (!kitting_task)
                    order_instance->SetKittingTask(kitting_task);
                else if (!assembly_task)
                    order_instance->SetAssemblyTask(assembly_task);
                else if (!combined_task)
                    order_instance->SetCombinedTask(combined_task);

                // Add to the list of part placement orders
                impl_->on_part_placement_orders_.push_back(order_instance);
            }
            else if (order->condition.type == ariac_common::ConditionType::SUBMISSION)
            {
                auto submitted_order_id = order->condition.submission_condition.order_id;
                auto order_instance = std::make_shared<ariac_common::OrderOnSubmission>(order_id, order_type, order_priority, impl_->time_limit_, submitted_order_id);
                // Set the task
                if (!kitting_task)
                    order_instance->SetKittingTask(kitting_task);
                else if (!assembly_task)
                    order_instance->SetAssemblyTask(assembly_task);
                else if (!combined_task)
                    order_instance->SetCombinedTask(combined_task);
                
                // Add to the list of submission orders
                impl_->on_order_submission_orders_.push_back(order_instance);
            }
            else
            {
                RCLCPP_ERROR_STREAM(impl_->ros_node_->get_logger(), "Unknown condition type: " << int(order->condition.type));
            }
        }
    }

    //==============================================================================
    void TaskManagerPlugin::StoreChallenges(const std::vector<ariac_msgs::msg::Challenge::SharedPtr> &challenges)
    {
    }

    //==============================================================================
    void TaskManagerPlugin::ActivateAllSensors()
    {
        // publish on /ariac/sensor_health topic
        auto sensor_message = ariac_msgs::msg::Sensors();
        sensor_message.break_beam = true;
        sensor_message.proximity = true;
        sensor_message.laser_profiler = true;
        sensor_message.lidar = true;
        sensor_message.camera = true;
        sensor_message.logical_camera = true;
        impl_->sensor_health_pub_->publish(sensor_message);
        RCLCPP_INFO_STREAM(impl_->ros_node_->get_logger(), "Activated all sensors");
    }

    // ==================================== //
    // Services
    // ==================================== //
    bool TaskManagerPlugin::StartCompetitionServiceCallback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        std::lock_guard<std::mutex> lock(impl_->lock_);

        // gzdbg << "\n";
        // gzdbg << "StartCompetitionServiceCallback\n";

        (void)request;

        if (impl_->current_state_ == ariac_msgs::msg::CompetitionState::UNSTARTED)
        {
            impl_->current_state_ = ariac_msgs::msg::CompetitionState::STARTED;
            response->success = true;
            response->message = "Competition started successfully!";

            // Activate all sensors
            ActivateAllSensors();

            return true;
        }
        response->success = false;
        response->message = "ERROR: Cannot start competition if current state is not 'init'";
        return true;
    }
    //==============================================================================
    bool TaskManagerPlugin::EndCompetitionServiceCallback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        std::lock_guard<std::mutex> lock(this->impl_->lock_);
        // gzdbg << "\n";
        // gzdbg << "EndCompetitionServiceCallback\n";

        (void)request;

        this->impl_->current_state_ = ariac_msgs::msg::CompetitionState::ENDED;
        response->success = true;
        response->message = "Competition ended successfully!";
        return true;
    }

    // Register this plugin with the simulator
    GZ_REGISTER_WORLD_PLUGIN(TaskManagerPlugin)
} // namespace ariac_plugins