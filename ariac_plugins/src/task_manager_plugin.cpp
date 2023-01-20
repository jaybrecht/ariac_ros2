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
#include <gazebo/transport/Subscriber.hh>
#include <gazebo/transport/Node.hh>
#include <gazebo/sensors/LogicalCameraSensor.hh>

// Messages
#include <ariac_plugins/task_manager_plugin.hpp>
#include <ariac_msgs/msg/trial.hpp>
#include <ariac_msgs/msg/order.hpp>
#include <ariac_msgs/msg/order_condition.hpp>
#include <ariac_msgs/msg/challenge.hpp>
#include <ariac_msgs/msg/sensors.hpp>
#include <ariac_msgs/msg/condition.hpp>
#include <ariac_msgs/msg/kitting_task.hpp>
#include <ariac_msgs/msg/kitting_part.hpp>
#include <ariac_msgs/msg/assembly_task.hpp>
#include <ariac_msgs/msg/assembly_part.hpp>
#include <ariac_msgs/msg/combined_task.hpp>
#include <ariac_msgs/msg/part.hpp>
#include <ariac_msgs/msg/parts.hpp>
#include <ariac_msgs/msg/robots.hpp>
#include <ariac_msgs/msg/competition_state.hpp>
#include <ariac_msgs/srv/submit_order.hpp>
// ROS
#include <rclcpp/rclcpp.hpp>
#include <controller_manager_msgs/srv/switch_controller.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

// C++
#include <memory>
// ARIAC
#include <ariac_plugins/ariac_common.hpp>

namespace ariac_plugins
{
    /// Class to hold private data members (PIMPL pattern)
    class TaskManagerPluginPrivate
    {
    public:
        gazebo::transport::NodePtr gznode_;

        //============== C++ =================
        /*!< A mutex to protect the current state. */
        std::mutex lock_;
        /*!< Pointer to the current state. */
        unsigned int current_state_{ariac_msgs::msg::CompetitionState::IDLE};

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

        //============== FLAGS =================
        bool competition_time_set_{false};
        /*< Counter to keep track of the total number of orders in the current trial. */
        int total_orders_{-1};

        //============== ROS =================
        /*!< Time when this plugin is loaded. */
        rclcpp::Time current_sim_time_;
        /*!< Elapsed time since the competition started. */
        double elapsed_time_;

        //============== SUBSCRIBERS =================
        /*!< Subscriber to topic: "trial_config"*/
        rclcpp::Subscription<ariac_msgs::msg::Trial>::SharedPtr trial_config_sub_;
        // rclcpp::Subscription<ariac_msgs::msg::Parts>::SharedPtr agv1_tray_sub_;
        // rclcpp::Subscription<ariac_msgs::msg::Parts>::SharedPtr agv2_tray_sub_;
        // rclcpp::Subscription<ariac_msgs::msg::Parts>::SharedPtr agv3_tray_sub_;
        // rclcpp::Subscription<ariac_msgs::msg::Parts>::SharedPtr agv4_tray_sub_;

        gazebo::transport::SubscriberPtr agv1_tray_sub_;
        gazebo::transport::SubscriberPtr agv2_tray_sub_;
        gazebo::transport::SubscriberPtr agv3_tray_sub_;
        gazebo::transport::SubscriberPtr agv4_tray_sub_;

        gazebo::transport::SubscriberPtr station1_sub_;
        gazebo::transport::SubscriberPtr station2_sub_;
        gazebo::transport::SubscriberPtr station3_sub_;
        gazebo::transport::SubscriberPtr station4_sub_;

        //============== SERVICES =================
        /*!< Service that allows the user to start the competition. */
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_competition_srv_{nullptr};
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr end_competition_srv_{nullptr};
        rclcpp::Service<ariac_msgs::srv::SubmitOrder>::SharedPtr submit_order_srv_{nullptr};
        /*!< Client to start/stop robot controllers. */
        // rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr switch_controller_srv_client_;
        /*!< Client to stop the competition. */
        rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr end_competition_srv_client_;

        //============== PUBLISHERS =================

        bool break_beam_sensor_health_;
        bool proximity_sensor_health_;
        bool laser_profiler_sensor_health_;
        bool lidar_sensor_health_;
        bool camera_sensor_health_;
        bool logical_camera_sensor_health_;

        /*!< Publisher to the topic /ariac/orders */
        rclcpp::Publisher<ariac_msgs::msg::Order>::SharedPtr order_pub_;
        /*!< Publisher to the topic /ariac/sensor_health */
        rclcpp::Publisher<ariac_msgs::msg::Sensors>::SharedPtr sensor_health_pub_;
        /*!< Publisher to the topic /ariac/competition_state */
        rclcpp::Publisher<ariac_msgs::msg::CompetitionState>::SharedPtr competition_state_pub_;
        /*!< Publisher to the topic /ariac/robot_health */
        rclcpp::Publisher<ariac_msgs::msg::Robots>::SharedPtr robot_health_pub_;

        //============== LISTS FOR ORDERS AND CHALLENGES =================

        std::vector<std::shared_ptr<ariac_common::SensorBlackout>> in_progress_sensor_blackouts_;
        /*!< List of orders that are announced based on time. */
        std::vector<std::shared_ptr<ariac_common::OrderTemporal>> time_based_orders_;
        /*!< List of orders that are announced based on part placement. */
        std::vector<std::shared_ptr<ariac_common::OrderOnPartPlacement>> on_part_placement_orders_;
        /*!< List of orders that are announced based on submission. */
        std::vector<std::shared_ptr<ariac_common::OrderOnSubmission>> on_order_submission_orders_;
        /*!< List of all orders in the trial. */
        std::vector<std::shared_ptr<ariac_common::Order>> all_orders_;

        /*!< List of sensor blackout challenges that are announced based on time. */
        std::vector<std::shared_ptr<ariac_common::SensorBlackoutTemporal>> time_based_sensor_blackouts_;
        /*!< List of sensor blackout challenges that are announced based on part placement. */
        std::vector<std::shared_ptr<ariac_common::SensorBlackoutOnPartPlacement>> on_part_placement_sensor_blackouts_;
        /*!< List of sensor blackout challenges that are announced based on submission. */
        std::vector<std::shared_ptr<ariac_common::SensorBlackoutOnSubmission>> on_submission_sensor_blackouts_;

        /*!< List of sensor blackout challenges that are announced based on time. */
        std::vector<std::shared_ptr<ariac_common::RobotMalfunctionTemporal>> time_based_robot_malfunctions_;
        /*!< List of sensor blackout challenges that are announced based on part placement. */
        std::vector<std::shared_ptr<ariac_common::RobotMalfunctionOnPartPlacement>> on_part_placement_robot_malfunctions_;
        /*!< List of sensor blackout challenges that are announced based on submission. */
        std::vector<std::shared_ptr<ariac_common::RobotMalfunctionOnSubmission>> on_submission_robot_malfunctions_;

        /*!< List of parts on AGV1. */
        std::vector<ariac_common::Part> agv1_tray_contents_;
        /*!< List of parts on AGV2. */
        std::vector<ariac_common::Part> agv2_tray_contents_;
        /*!< List of parts on AGV3. */
        std::vector<ariac_common::Part> agv3_tray_contents_;
        /*!< List of parts on AGV4. */
        std::vector<ariac_common::Part> agv4_tray_contents_;
        /*!< List of orders. that have already been submitted*/
        std::vector<std::string> submitted_orders_;
        /*!< List of orders. that have already been announced*/
        std::vector<std::string> announced_orders_;
        /*!< List of trial orders. */
        std::vector<std::string> trial_orders_;
        /*!< Kit tray id for agv1. */
        int agv1_kit_tray_id_;

        void AGV1TraySensorCallback(ConstLogicalCameraImagePtr &_msg);
        void AGV2TraySensorCallback(ConstLogicalCameraImagePtr &_msg);
        void AGV3TraySensorCallback(ConstLogicalCameraImagePtr &_msg);
        void AGV4TraySensorCallback(ConstLogicalCameraImagePtr &_msg);

        // tray id + list of parts
        void ParseTrayModels(unsigned int agv_number);
        //  void ParseAssemblyModels();
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

        impl_->gznode_ = gazebo::transport::NodePtr(new gazebo::transport::Node());
        impl_->gznode_->Init(impl_->world_->Name());

        std::string tray1_topic = "/gazebo/world/agv_tray_sensor_1/sensor_base/agv_tray_sensor/models";
        std::string tray2_topic = "/gazebo/world/agv_tray_sensor_2/sensor_base/agv_tray_sensor/models";
        std::string tray3_topic = "/gazebo/world/agv_tray_sensor_3/sensor_base/agv_tray_sensor/models";
        std::string tray4_topic = "/gazebo/world/agv_tray_sensor_4/sensor_base/agv_tray_sensor/models";

        impl_->agv1_tray_sub_ = impl_->gznode_->Subscribe(tray1_topic, &TaskManagerPluginPrivate::AGV1TraySensorCallback, impl_.get());
        impl_->agv2_tray_sub_ = impl_->gznode_->Subscribe(tray2_topic, &TaskManagerPluginPrivate::AGV2TraySensorCallback, impl_.get());
        impl_->agv3_tray_sub_ = impl_->gznode_->Subscribe(tray3_topic, &TaskManagerPluginPrivate::AGV3TraySensorCallback, impl_.get());
        impl_->agv4_tray_sub_ = impl_->gznode_->Subscribe(tray4_topic, &TaskManagerPluginPrivate::AGV4TraySensorCallback, impl_.get());

        std::string station1_topic = "/gazebo/world/assembly_station_sensor_1/sensor_base/assembly_station_sensor/models";
        std::string station2_topic = "/gazebo/world/assembly_station_sensor_2/sensor_base/assembly_station_sensor/models";
        std::string station3_topic = "/gazebo/world/assembly_station_sensor_3/sensor_base/assembly_station_sensor/models";
        std::string station4_topic = "/gazebo/world/assembly_station_sensor_4/sensor_base/assembly_station_sensor/models";

        // impl_->station1_sub_ = impl_->gznode_->Subscribe(station1_topic, &TaskManagerPluginPrivate::Station1SensorCallback, impl_.get());
        // impl_->station2_sub_ = impl_->gznode_->Subscribe(station2_topic, &TaskManagerPluginPrivate::Station2SensorCallback, impl_.get());
        // impl_->station3_sub_ = impl_->gznode_->Subscribe(station3_topic, &TaskManagerPluginPrivate::Station3SensorCallback, impl_.get());
        // impl_->station4_sub_ = impl_->gznode_->Subscribe(station4_topic, &TaskManagerPluginPrivate::Station4SensorCallback, impl_.get());

        RCLCPP_INFO(impl_->ros_node_->get_logger(), "Starting ARIAC 2023");

        // Get QoS profiles
        const gazebo_ros::QoS &qos = impl_->ros_node_->get_qos();

        // Create a connection so the OnUpdate function is called at every simulation
        // iteration. Remove this call, the connection and the callback if not needed.
        impl_->update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
            std::bind(&TaskManagerPlugin::OnUpdate, this));

        //============== SUBSCRIBERS =================
        impl_->trial_config_sub_ = impl_->ros_node_->create_subscription<ariac_msgs::msg::Trial>(
            "/ariac/trial_config", qos.get_subscription_qos("/ariac/trial_config", rclcpp::QoS(1)),
            std::bind(&TaskManagerPlugin::OnTrialCallback, this, std::placeholders::_1));

        // impl_->agv1_tray_sub_ = impl_->ros_node_->create_subscription<ariac_msgs::msg::Parts>(
        //     "/ariac/agv1_tray_contents", qos.get_subscription_qos("/ariac/agv1_tray_contents", rclcpp::QoS(1)),
        //     std::bind(&TaskManagerPlugin::OnAGV1TrayContentsCallback, this, std::placeholders::_1));

        // impl_->agv2_tray_sub_ = impl_->ros_node_->create_subscription<ariac_msgs::msg::Parts>(
        //     "/ariac/agv2_tray_contents", qos.get_subscription_qos("/ariac/agv2_tray_contents", rclcpp::QoS(1)),
        //     std::bind(&TaskManagerPlugin::OnAGV2TrayContentsCallback, this, std::placeholders::_1));

        // impl_->agv3_tray_sub_ = impl_->ros_node_->create_subscription<ariac_msgs::msg::Parts>(
        //     "/ariac/agv3_tray_contents", qos.get_subscription_qos("/ariac/agv3_tray_contents", rclcpp::QoS(1)),
        //     std::bind(&TaskManagerPlugin::OnAGV3TrayContentsCallback, this, std::placeholders::_1));

        // impl_->agv4_tray_sub_ = impl_->ros_node_->create_subscription<ariac_msgs::msg::Parts>(
        //     "/ariac/agv4_tray_contents", qos.get_subscription_qos("/ariac/agv4_tray_contents", rclcpp::QoS(1)),
        //     std::bind(&TaskManagerPlugin::OnAGV4TrayContentsCallback, this, std::placeholders::_1));

        //============== PUBLISHERS =================
        impl_->sensor_health_pub_ = impl_->ros_node_->create_publisher<ariac_msgs::msg::Sensors>("/ariac/sensor_health", 10);
        impl_->robot_health_pub_ = impl_->ros_node_->create_publisher<ariac_msgs::msg::Robots>("/ariac/robot_health", 10);
        impl_->competition_state_pub_ = impl_->ros_node_->create_publisher<ariac_msgs::msg::CompetitionState>("/ariac/competition_state", 10);
        impl_->order_pub_ = impl_->ros_node_->create_publisher<ariac_msgs::msg::Order>("/ariac/orders", 1);
        //============== SERVICES =================
        impl_->end_competition_srv_client_ = impl_->ros_node_->create_client<std_srvs::srv::Trigger>("/ariac/end_competition");

        impl_->break_beam_sensor_health_ = true;
        impl_->proximity_sensor_health_ = true;
        impl_->laser_profiler_sensor_health_ = true;
        impl_->lidar_sensor_health_ = true;
        impl_->camera_sensor_health_ = true;
        impl_->logical_camera_sensor_health_ = true;

        impl_->elapsed_time_ = 0.0;
    }

    void TaskManagerPluginPrivate::AGV1TraySensorCallback(ConstLogicalCameraImagePtr &_msg)
    {
        // RCLCPP_INFO_STREAM(ros_node_->get_logger(), _msg->);
    }

    void TaskManagerPluginPrivate::AGV2TraySensorCallback(ConstLogicalCameraImagePtr &_msg)
    {
    }

    void TaskManagerPluginPrivate::AGV3TraySensorCallback(ConstLogicalCameraImagePtr &_msg)
    {
    }

    void TaskManagerPluginPrivate::AGV4TraySensorCallback(ConstLogicalCameraImagePtr &_msg)
    {
    }

    // void TaskManagerPluginPrivate::Station1SensorCallback(ConstContactsPtr &_msg)
    // {
    // }

    // void TaskManagerPluginPrivate::Station2SensorCallback(ConstContactsPtr &_msg)
    // {
    // }

    // void TaskManagerPluginPrivate::Station3SensorCallback(ConstContactsPtr &_msg)
    // {
    // }

    // void TaskManagerPluginPrivate::Station4SensorCallback(ConstContactsPtr &_msg)
    // {
    // }

    //==============================================================================
    const ariac_msgs::msg::KittingTask
    TaskManagerPlugin::BuildKittingTaskMsg(std::shared_ptr<ariac_common::KittingTask> _task)
    {
        auto message = ariac_msgs::msg::KittingTask();
        message.agv_number = _task->GetAgvNumber();
        message.tray_id = _task->GetTrayId();
        message.destination = _task->GetDestination();

        for (auto &part : _task->GetProducts())
        {
            auto part_msg = ariac_msgs::msg::KittingPart();
            part_msg.part.color = part.GetPart().GetColor();
            part_msg.part.type = part.GetPart().GetType();
            part_msg.quadrant = part.GetQuadrant();
            message.parts.push_back(part_msg);
        }
        return message;
    }

    //==============================================================================
    const ariac_msgs::msg::AssemblyTask TaskManagerPlugin::BuildAssemblyTaskMsg(std::shared_ptr<ariac_common::AssemblyTask> _task)
    {
        auto message = ariac_msgs::msg::AssemblyTask();
        // station
        message.station = _task->GetStation();
        // agvs
        for (auto &agv : _task->GetAgvNumbers())
        {
            message.agv_numbers.push_back(agv);
        }

        // products
        for (auto &part : _task->GetProducts())
        {
            auto part_msg = ariac_msgs::msg::AssemblyPart();
            part_msg.part.color = part.GetPart().GetColor();
            part_msg.part.type = part.GetPart().GetType();

            auto assembly_pose_msg = geometry_msgs::msg::PoseStamped();
            assembly_pose_msg.pose.position.x = part.GetPartPose().Pos().X();
            assembly_pose_msg.pose.position.y = part.GetPartPose().Pos().Y();
            assembly_pose_msg.pose.position.z = part.GetPartPose().Pos().Z();
            assembly_pose_msg.pose.orientation.x = part.GetPartPose().Rot().X();
            assembly_pose_msg.pose.orientation.y = part.GetPartPose().Rot().Y();
            assembly_pose_msg.pose.orientation.z = part.GetPartPose().Rot().Z();
            assembly_pose_msg.pose.orientation.w = part.GetPartPose().Rot().W();
            part_msg.assembled_pose = assembly_pose_msg;

            auto install_direction_msg = geometry_msgs::msg::Vector3();
            install_direction_msg.x = part.GetPartDirection().X();
            install_direction_msg.y = part.GetPartDirection().Y();
            install_direction_msg.z = part.GetPartDirection().Z();
            part_msg.install_direction = install_direction_msg;

            message.parts.push_back(part_msg);
        }

        return message;
    }

    //==============================================================================
    const ariac_msgs::msg::CombinedTask TaskManagerPlugin::BuildCombinedTaskMsg(std::shared_ptr<ariac_common::CombinedTask> _task)
    {
        auto message = ariac_msgs::msg::CombinedTask();
        // station
        message.station = _task->GetStation();

        // products
        for (auto &part : _task->GetProducts())
        {
            auto part_msg = ariac_msgs::msg::AssemblyPart();
            part_msg.part.color = part.GetPart().GetColor();
            part_msg.part.type = part.GetPart().GetType();

            auto assembly_pose_msg = geometry_msgs::msg::PoseStamped();
            assembly_pose_msg.pose.position.x = part.GetPartPose().Pos().X();
            assembly_pose_msg.pose.position.y = part.GetPartPose().Pos().Y();
            assembly_pose_msg.pose.position.z = part.GetPartPose().Pos().Z();
            assembly_pose_msg.pose.orientation.x = part.GetPartPose().Rot().X();
            assembly_pose_msg.pose.orientation.y = part.GetPartPose().Rot().Y();
            assembly_pose_msg.pose.orientation.z = part.GetPartPose().Rot().Z();
            assembly_pose_msg.pose.orientation.w = part.GetPartPose().Rot().W();
            part_msg.assembled_pose = assembly_pose_msg;

            auto install_direction_msg = geometry_msgs::msg::Vector3();
            install_direction_msg.x = part.GetPartDirection().X();
            install_direction_msg.y = part.GetPartDirection().Y();
            install_direction_msg.z = part.GetPartDirection().Z();
            part_msg.install_direction = install_direction_msg;

            message.parts.push_back(part_msg);
        }

        return message;
    }

    //==============================================================================
    const ariac_msgs::msg::Order
    TaskManagerPlugin::BuildOrderMsg(std::shared_ptr<ariac_common::Order> _order)
    {
        auto order_message = ariac_msgs::msg::Order();
        order_message.id = _order->GetId();
        order_message.type = _order->GetType();
        order_message.priority = _order->IsPriority();
        if (_order->GetKittingTask())
        {
            order_message.kitting_task = BuildKittingTaskMsg(_order->GetKittingTask());
        }
        else if (_order->GetAssemblyTask())
        {
            // RCLCPP_INFO_STREAM_ONCE(impl_->ros_node_->get_logger(), "GetAssemblyTask");
            order_message.assembly_task = BuildAssemblyTaskMsg(_order->GetAssemblyTask());
        }
        else if (_order->GetCombinedTask())
        {
            // RCLCPP_INFO_STREAM_ONCE(impl_->ros_node_->get_logger(), "GetCombinedTask");
            order_message.combined_task = BuildCombinedTaskMsg(_order->GetCombinedTask());
        }
        return order_message;
    }

    //==============================================================================
    void TaskManagerPlugin::ProcessTemporalOrders()
    {
        for (const auto &order : impl_->time_based_orders_)
        {
            if (impl_->elapsed_time_ >= order->GetAnnouncementTime() && !order->IsAnnounced())
            {
                auto order_message = BuildOrderMsg(order);
                RCLCPP_INFO_STREAM(impl_->ros_node_->get_logger(), "Publishing order: " << order_message.id);
                impl_->order_pub_->publish(order_message);
                impl_->announced_orders_.push_back(order_message.id);
                order->SetAnnouncedTime(impl_->elapsed_time_);
                order->SetIsAnnounced();
                impl_->total_orders_--;
            }
        }
    }

    //==============================================================================
    void TaskManagerPlugin::ProcessOnPartPlacementOrders()
    {
        // for (const auto &order : impl_->on_part_placement_orders_)
        // {
        //     auto agv = order->GetAgv();
        //     auto part = order->GetPart();

        //     bool found{false};

        //     switch (agv)
        //     {
        //     case 1:
        //         if (std::find(impl_->agv1_tray_contents_.begin(), impl_->agv1_tray_contents_.end(), part) != impl_->agv1_tray_contents_.end())
        //         {
        //             found = true;
        //         }
        //     case 2:
        //         if (std::find(impl_->agv2_tray_contents_.begin(), impl_->agv2_tray_contents_.end(), part) != impl_->agv2_tray_contents_.end())
        //         {
        //             found = true;
        //         }
        //     case 3:
        //         if (std::find(impl_->agv3_tray_contents_.begin(), impl_->agv3_tray_contents_.end(), part) != impl_->agv3_tray_contents_.end())
        //         {
        //             found = true;
        //         }
        //     case 4:
        //         if (std::find(impl_->agv4_tray_contents_.begin(), impl_->agv4_tray_contents_.end(), part) != impl_->agv4_tray_contents_.end())
        //         {
        //             found = true;
        //         }
        //         break;

        //     default:
        //         break;
        //     }

        //     if (found && !order->IsAnnounced())
        //     {
        //         auto order_message = BuildOrderMsg(order);
        //         RCLCPP_INFO_STREAM(impl_->ros_node_->get_logger(), "Publishing order: " << order_message.id);
        //         impl_->order_pub_->publish(order_message);
        //         // impl_->submitted_orders_.push_back(order_message.id);
        //         order->SetIsAnnounced();
        //         impl_->total_orders_--;
        //     }
        // }
    }

    //==============================================================================
    void TaskManagerPlugin::ProcessOnSubmissionOrders()
    {
        for (const auto &order_ins : impl_->on_order_submission_orders_)
        {
            // Get the id of the order which will trigger the annoucement of order_ins
            auto trigger_order = order_ins->GetOrderId();

            // parse the list of submitted orders to see if the trigger order has been submitted
            if (std::find(impl_->submitted_orders_.begin(), impl_->submitted_orders_.end(), trigger_order) != impl_->submitted_orders_.end())
            {
                if (!order_ins->IsAnnounced())
                {
                    auto order_message = BuildOrderMsg(order_ins);
                    RCLCPP_INFO_STREAM(impl_->ros_node_->get_logger(), "Publishing order: " << order_message.id);
                    impl_->order_pub_->publish(order_message);
                    order_ins->SetAnnouncedTime(impl_->elapsed_time_);
                    order_ins->SetIsAnnounced();
                    impl_->total_orders_--;
                }
            }
        }
    }

    //==============================================================================
    void TaskManagerPlugin::ProcessOrdersToAnnounce()
    {
        if (!impl_->time_based_orders_.empty())
        {
            ProcessTemporalOrders();
        }
        // TODO: Implement on part placement and on submission orders
        if (!impl_->on_part_placement_orders_.empty())
        {
            ProcessOnPartPlacementOrders();
        }
        if (!impl_->on_order_submission_orders_.empty())
        {
            ProcessOnSubmissionOrders();
        }
    }

    void TaskManagerPlugin::ProcessInProgressSensorBlackouts()
    {
        for (const auto &sensor_blackout : impl_->in_progress_sensor_blackouts_)
        {
            if (sensor_blackout->IsStarted() && !sensor_blackout->IsCompleted())
            {
                auto duration = sensor_blackout->GetDuration();
                auto start_time = sensor_blackout->GetStartTime();
                // RCLCPP_INFO_STREAM(impl_->ros_node_->get_logger(), "Sensor blackout challenge in progress");
                RCLCPP_INFO_STREAM(impl_->ros_node_->get_logger(), start_time);
                RCLCPP_INFO_STREAM(impl_->ros_node_->get_logger(), duration);

                if (impl_->elapsed_time_ >= start_time + duration)
                {
                    RCLCPP_INFO_STREAM(impl_->ros_node_->get_logger(), "Stopping sensor blackout challenge");
                    sensor_blackout->SetCompleted();
                    impl_->break_beam_sensor_health_ = true;
                    impl_->proximity_sensor_health_ = true;
                    impl_->laser_profiler_sensor_health_ = true;
                    impl_->lidar_sensor_health_ = true;
                    impl_->camera_sensor_health_ = true;
                    impl_->logical_camera_sensor_health_ = true;
                    // remove the sensor blackout from the list
                    impl_->in_progress_sensor_blackouts_.erase(std::remove(impl_->in_progress_sensor_blackouts_.begin(),
                                                                           impl_->in_progress_sensor_blackouts_.end(), sensor_blackout),
                                                               impl_->in_progress_sensor_blackouts_.end());
                }

                
            }
        }
    }

    void TaskManagerPlugin::ProcessTemporalSensorBlackouts()
    {
        // RCLCPP_INFO_STREAM_ONCE(impl_->ros_node_->get_logger(), "Size temporal blackouts: " << impl_->time_based_sensor_blackouts_.size());
        for (const auto &sensor_blackout : impl_->time_based_sensor_blackouts_)
        {
            
            // RCLCPP_INFO_STREAM(impl_->ros_node_->get_logger(), "Announcement time: " << sensor_blackout->GetAnnouncementTime());
            if (impl_->elapsed_time_ >= sensor_blackout->GetAnnouncementTime() && !sensor_blackout->IsStarted())
            {
                // RCLCPP_INFO_STREAM(impl_->ros_node_->get_logger(), "Elapsed time: " << impl_->elapsed_time_);
                // RCLCPP_INFO_STREAM(impl_->ros_node_->get_logger(), "Starting sensor blackout challenge");
                auto duration = sensor_blackout->GetDuration();
                auto sensors_to_disable = sensor_blackout->GetSensorsToDisable();

                for (const auto &sensor : sensors_to_disable)
                {
                    if (sensor == "break_beam")
                        impl_->break_beam_sensor_health_ = false;
                    if (sensor == "proximity")
                        impl_->proximity_sensor_health_ = false;
                    if (sensor == "laser_profiler")
                        impl_->laser_profiler_sensor_health_ = false;
                    if (sensor == "lidar")
                        impl_->lidar_sensor_health_ = false;
                    if (sensor == "camera")
                        impl_->camera_sensor_health_ = false;
                    if (sensor == "logical_camera")
                        impl_->logical_camera_sensor_health_ = false;
                }
                sensor_blackout->SetStartTime(impl_->elapsed_time_);
                sensor_blackout->SetStarted();
                impl_->in_progress_sensor_blackouts_.push_back(sensor_blackout);
            }
        }
    }
    //==============================================================================
    void TaskManagerPlugin::ProcessChallengesToAnnounce()
    {
        if (!impl_->time_based_sensor_blackouts_.empty())
        {
            ProcessTemporalSensorBlackouts();
        }
        if (!impl_->on_part_placement_sensor_blackouts_.empty())
        {
        }
        if (!impl_->on_submission_sensor_blackouts_.empty())
        {
        }
        if (!impl_->time_based_robot_malfunctions_.empty())
        {
        }
        if (!impl_->on_part_placement_robot_malfunctions_.empty())
        {
        }
        if (!impl_->on_submission_robot_malfunctions_.empty())
        {
        }
    }

    void TaskManagerPlugin::PublishCompetitionState(unsigned int _state)
    {
        auto state_message = ariac_msgs::msg::CompetitionState();
        state_message.competition_state = _state;
        impl_->competition_state_pub_->publish(state_message);
    }

    //==============================================================================
    void TaskManagerPlugin::OnUpdate()
    {
        std::lock_guard<std::mutex> lock(impl_->lock_);
        auto current_sim_time = impl_->world_->SimTime();

        if (impl_->total_orders_ == 0 && impl_->current_state_ == ariac_msgs::msg::CompetitionState::STARTED)
        {
            RCLCPP_INFO_STREAM_ONCE(impl_->ros_node_->get_logger(), "All orders have been announced.");
            impl_->current_state_ = ariac_msgs::msg::CompetitionState::ORDER_ANNOUNCEMENTS_DONE;
        }

        // publish the competition state
        PublishCompetitionState(impl_->current_state_);

        // Delay advertising the competition start service to avoid a crash.
        // Sometimes if the competition is started before the world is fully loaded, it causes a crash.
        if (!impl_->start_competition_srv_ && current_sim_time.Double() >= 5.0)
        {
            // Create the start competition service
            // Now competitors can call this service to start the competition
            impl_->start_competition_srv_ = impl_->ros_node_->create_service<std_srvs::srv::Trigger>("/ariac/start_competition",
                                                                                                     std::bind(&TaskManagerPlugin::StartCompetitionServiceCallback,
                                                                                                               this,
                                                                                                               std::placeholders::_1,
                                                                                                               std::placeholders::_2));

            RCLCPP_INFO_STREAM(impl_->ros_node_->get_logger(), "You can now start the competition!");
            impl_->current_state_ = ariac_msgs::msg::CompetitionState::READY;

            // Create the end competition service
            // Now competitors can call this service to end the competition
            impl_->end_competition_srv_ = impl_->ros_node_->create_service<std_srvs::srv::Trigger>("/ariac/end_competition",
                                                                                                   std::bind(&TaskManagerPlugin::EndCompetitionServiceCallback,
                                                                                                             this,
                                                                                                             std::placeholders::_1,
                                                                                                             std::placeholders::_2));

            impl_->submit_order_srv_ = impl_->ros_node_->create_service<ariac_msgs::srv::SubmitOrder>("/ariac/submit_order",
                                                                                                      std::bind(&TaskManagerPlugin::SubmitOrderServiceCallback,
                                                                                                                this,
                                                                                                                std::placeholders::_1,
                                                                                                                std::placeholders::_2));
        }

        if ((current_sim_time - impl_->last_sim_time_).Double() >= 1.0)
        {
            impl_->last_sim_time_ = current_sim_time;
        }

        

        // End the competition if:
        // The time limit was set to a positive value
        // The elapsed time since the start of the competition is greater than the time limit
        // The current state is ORDER_ANNOUNCEMENTS_DONE
        if (impl_->time_limit_ >= 0 &&
            (current_sim_time - impl_->start_competition_time_).Double() > impl_->time_limit_ && impl_->current_state_ == ariac_msgs::msg::CompetitionState::ORDER_ANNOUNCEMENTS_DONE)
        {
            RCLCPP_INFO_STREAM(impl_->ros_node_->get_logger(), "Time limit reached. Ending competition.");
            this->impl_->current_state_ = ariac_msgs::msg::CompetitionState::ENDED;
        }

        // current state was set to STARTED in start competition service callback
        if (impl_->current_state_ == ariac_msgs::msg::CompetitionState::STARTED && !impl_->competition_time_set_)
        {
            impl_->start_competition_time_ = current_sim_time;
            impl_->competition_time_set_ = true;
        }

        // Elapsed time since the competition started
        if (impl_->current_state_ == ariac_msgs::msg::CompetitionState::STARTED)
            impl_->elapsed_time_ = (current_sim_time - impl_->start_competition_time_).Double();
        else
            impl_->elapsed_time_ = 0.0;

        // If the competition has started, do the main work
        if (impl_->current_state_ == ariac_msgs::msg::CompetitionState::STARTED)
        {
            ProcessOrdersToAnnounce();
            ProcessChallengesToAnnounce();
            ProcessInProgressSensorBlackouts();
            UpdateSensorsHealth();
            // ProcessInProgressRobotMalfunctions();
            // UpdateRobotsHealth();
        }

        impl_->last_on_update_time_ = current_sim_time;
    }

    //==============================================================================
    void TaskManagerPlugin::OnAGV1TrayContentsCallback(const ariac_msgs::msg::Parts::SharedPtr _msg)
    {
        std::lock_guard<std::mutex> scoped_lock(impl_->lock_);
        if (_msg->parts.size() == 0)
        {
            impl_->agv1_tray_contents_.clear();
            return;
        }

        for (auto &part : _msg->parts)
        {
            impl_->agv1_tray_contents_.emplace_back(ariac_common::Part(part.color, part.type));
        }
    }

    //==============================================================================
    void TaskManagerPlugin::OnAGV2TrayContentsCallback(const ariac_msgs::msg::Parts::SharedPtr _msg)
    {
        std::lock_guard<std::mutex> scoped_lock(impl_->lock_);
        if (_msg->parts.size() == 0)
        {
            impl_->agv2_tray_contents_.clear();
            return;
        }
        for (auto &part : _msg->parts)
        {
            impl_->agv2_tray_contents_.emplace_back(ariac_common::Part(part.color, part.type));
        }
    }

    //==============================================================================
    void TaskManagerPlugin::OnAGV3TrayContentsCallback(const ariac_msgs::msg::Parts::SharedPtr _msg)
    {
        std::lock_guard<std::mutex> scoped_lock(impl_->lock_);
        if (_msg->parts.size() == 0)
        {
            impl_->agv3_tray_contents_.clear();
            return;
        }
        for (auto &part : _msg->parts)
        {
            impl_->agv3_tray_contents_.emplace_back(ariac_common::Part(part.color, part.type));
        }
    }

    //==============================================================================
    void TaskManagerPlugin::OnAGV4TrayContentsCallback(const ariac_msgs::msg::Parts::SharedPtr _msg)
    {
        std::lock_guard<std::mutex> scoped_lock(impl_->lock_);
        if (_msg->parts.size() == 0)
        {
            impl_->agv4_tray_contents_.clear();
            return;
        }
        for (auto &part : _msg->parts)
        {
            impl_->agv4_tray_contents_.emplace_back(ariac_common::Part(part.color, part.type));
        }
    }

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
        // RCLCPP_INFO_STREAM(impl_->ros_node_->get_logger(), "Number of orders in the trial: " << impl_->all_orders_.size());

        // This attribute is used to keep track of the total number of orders
        impl_->total_orders_ = impl_->time_based_orders_.size() + impl_->on_part_placement_orders_.size() + impl_->on_order_submission_orders_.size();
        // RCLCPP_INFO_STREAM(impl_->ros_node_->get_logger(), "Number of temporal orders: " << impl_->time_based_orders_.size());
        // RCLCPP_INFO_STREAM(impl_->ros_node_->get_logger(), "Number of part place orders: " << impl_->on_part_placement_orders_.size());
        // RCLCPP_INFO_STREAM(impl_->ros_node_->get_logger(), "Number of submission orders: " << impl_->on_order_submission_orders_.size());

        // Store challenges to be processed later
        if (_msg->challenges.size() > 0)
        {
            std::vector<std::shared_ptr<ariac_msgs::msg::Challenge>> challenges;
            for (auto challenge : _msg->challenges)
            {
                challenges.push_back(std::make_shared<ariac_msgs::msg::Challenge>(challenge));
            }
            if (challenges.size() > 0)
            {
                StoreChallenges(challenges);
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
            auto part = ariac_common::Part(part_color, part_type);
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
            auto part = ariac_common::Part(part_color, part_type);

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
            auto part = ariac_common::Part(part_color, part_type);

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
            // Keep track of order ids
            impl_->trial_orders_.push_back(order_id);

            auto order_type = order->type;
            auto order_priority = order->priority;

            // Pointers to the tasks
            std::shared_ptr<ariac_common::KittingTask> kitting_task = nullptr;
            std::shared_ptr<ariac_common::AssemblyTask> assembly_task = nullptr;
            std::shared_ptr<ariac_common::CombinedTask> combined_task = nullptr;

            if (order_type == ariac_msgs::msg::Order::KITTING)
            {
                // RCLCPP_ERROR_STREAM(impl_->ros_node_->get_logger(), "Kitting task: " << order_id);
                kitting_task = BuildKittingTask(order->kitting_task);
            }
            else if (order_type == ariac_msgs::msg::Order::ASSEMBLY)
            {
                // RCLCPP_ERROR_STREAM(impl_->ros_node_->get_logger(), "Assembly task: " << order_id);
                assembly_task = BuildAssemblyTask(order->assembly_task);
            }
            else if (order_type == ariac_msgs::msg::Order::COMBINED)
            {
                // RCLCPP_ERROR_STREAM(impl_->ros_node_->get_logger(), "Combined task: " << order_id);
                combined_task = BuildCombinedTask(order->combined_task);
            }
            else
            {
                RCLCPP_ERROR_STREAM(impl_->ros_node_->get_logger(), "Unknown order type: " << int(order_type));
            }

            // Get the condition
            if (order->condition.type == ariac_msgs::msg::Condition::TIME)
            {
                // RCLCPP_ERROR_STREAM(impl_->ros_node_->get_logger(), "Order ID: " << order_id);
                auto announcement_time = order->condition.time_condition.seconds;
                // RCLCPP_ERROR_STREAM(impl_->ros_node_->get_logger(), "Time based order: " << order_id << " announcement time: " << announcement_time);
                auto order_instance = std::make_shared<ariac_common::OrderTemporal>(order_id, order_type, order_priority, impl_->time_limit_, announcement_time);
                // Set the task
                if (kitting_task)
                    order_instance->SetKittingTask(kitting_task);
                else if (assembly_task)
                    order_instance->SetAssemblyTask(assembly_task);
                else if (combined_task)
                    order_instance->SetCombinedTask(combined_task);

                // Add to the list of time based orders
                impl_->time_based_orders_.push_back(order_instance);
                impl_->all_orders_.push_back(order_instance);
            }
            else if (order->condition.type == ariac_msgs::msg::Condition::PART_PLACE)
            {
                auto agv = order->condition.part_place_condition.agv;
                auto part = std::make_shared<ariac_common::Part>(order->condition.part_place_condition.part.type, order->condition.part_place_condition.part.color);
                auto order_instance = std::make_shared<ariac_common::OrderOnPartPlacement>(order_id, order_type, order_priority, impl_->time_limit_, agv, part);
                // Set the task
                if (kitting_task)
                    order_instance->SetKittingTask(kitting_task);
                else if (assembly_task)
                    order_instance->SetAssemblyTask(assembly_task);
                else if (combined_task)
                    order_instance->SetCombinedTask(combined_task);

                // Add to the list of part placement orders
                impl_->on_part_placement_orders_.push_back(order_instance);
                impl_->all_orders_.push_back(order_instance);
            }
            else if (order->condition.type == ariac_msgs::msg::Condition::SUBMISSION)
            {
                auto submitted_order_id = order->condition.submission_condition.order_id;
                auto order_instance = std::make_shared<ariac_common::OrderOnSubmission>(order_id, order_type, order_priority, impl_->time_limit_, submitted_order_id);
                // Set the task
                if (kitting_task)
                    order_instance->SetKittingTask(kitting_task);
                else if (assembly_task)
                    order_instance->SetAssemblyTask(assembly_task);
                else if (combined_task)
                    order_instance->SetCombinedTask(combined_task);

                // Add to the list of submission orders
                impl_->on_order_submission_orders_.push_back(order_instance);
                impl_->all_orders_.push_back(order_instance);
            }
            else
            {
                RCLCPP_ERROR_STREAM(impl_->ros_node_->get_logger(), "Unknown condition type: " << int(order->condition.type));
            }
        }
    }

    //==============================================================================
    //==============================================================================
    void TaskManagerPlugin::BuildRobotMalfunctionChallenge(const ariac_msgs::msg::RobotMalfunctionChallenge &_challenge)
    {
        std::shared_ptr<ariac_common::RobotMalfunction> robot_malfunction{nullptr};
        auto duration = _challenge.duration;
        // Get the condition
        auto condition = _challenge.condition;
        // Get the robots to disable
        auto robots_to_disable = _challenge.robots_to_disable;

        // Vector with only the robots to disable
        std::vector<std::string> robots_to_disable_vect;

        // Check which robots to disable
        if (robots_to_disable.ceiling_robot)
        {
            robots_to_disable_vect.push_back("ceiling_robot");
        }
        if (robots_to_disable.floor_robot)
        {
            robots_to_disable_vect.push_back("floor_robot");
        }

        // Check the announcement condition for the current challenge
        if (condition.type == ariac_msgs::msg::Condition::TIME)
        {
            auto announcement_time = condition.time_condition.seconds;
            auto robot_malfunction = std::make_shared<ariac_common::RobotMalfunctionTemporal>(duration, robots_to_disable_vect, announcement_time);
            impl_->time_based_robot_malfunctions_.push_back(robot_malfunction);
        }
        else if (condition.type == ariac_msgs::msg::Condition::PART_PLACE)
        {
            auto agv = condition.part_place_condition.agv;
            auto part = std::make_shared<ariac_common::Part>(condition.part_place_condition.part.color, condition.part_place_condition.part.type);
            auto robot_malfunction = std::make_shared<ariac_common::RobotMalfunctionOnPartPlacement>(duration, robots_to_disable_vect, part, agv);
            impl_->on_part_placement_robot_malfunctions_.push_back(robot_malfunction);
        }
        else if (condition.type == ariac_msgs::msg::Condition::SUBMISSION)
        {
            auto submitted_order_id = condition.submission_condition.order_id;
            auto robot_malfunction = std::make_shared<ariac_common::RobotMalfunctionOnSubmission>(duration, robots_to_disable_vect, submitted_order_id);
            impl_->on_submission_robot_malfunctions_.push_back(robot_malfunction);
        }
    }

    //==============================================================================
    void TaskManagerPlugin::BuildSensorBlackoutChallenge(const ariac_msgs::msg::SensorBlackoutChallenge &_challenge)
    {

        // Get the duration
        auto duration = _challenge.duration;
        // Get the condition
        auto condition = _challenge.condition;
        // Get the sensors to disable
        auto sensors_to_disable = _challenge.sensors_to_disable;

        // Vector with only the sensors to disable
        std::vector<std::string> sensors_to_disable_vect;

        if (sensors_to_disable.break_beam)
        {
            sensors_to_disable_vect.push_back("break_beam");
        }
        if (sensors_to_disable.proximity)
        {
            sensors_to_disable_vect.push_back("proximity");
        }
        if (sensors_to_disable.laser_profiler)
        {
            sensors_to_disable_vect.push_back("laser_profiler");
        }
        if (sensors_to_disable.lidar)
        {
            sensors_to_disable_vect.push_back("lidar");
        }
        if (sensors_to_disable.camera)
        {
            sensors_to_disable_vect.push_back("camera");
        }
        if (sensors_to_disable.logical_camera)
        {
            sensors_to_disable_vect.push_back("logical_camera");
        }

        // Check condition
        if (condition.type == ariac_msgs::msg::Condition::TIME)
        {
            auto announcement_time = condition.time_condition.seconds;
            auto sensor_blackout = std::make_shared<ariac_common::SensorBlackoutTemporal>(duration, sensors_to_disable_vect, announcement_time);
            impl_->time_based_sensor_blackouts_.push_back(sensor_blackout);
        }
        else if (condition.type == ariac_msgs::msg::Condition::PART_PLACE)
        {
            auto agv = condition.part_place_condition.agv;
            auto part = std::make_shared<ariac_common::Part>(condition.part_place_condition.part.color, condition.part_place_condition.part.type);
            auto sensor_blackout = std::make_shared<ariac_common::SensorBlackoutOnPartPlacement>(duration, sensors_to_disable_vect, part, agv);
            impl_->on_part_placement_sensor_blackouts_.push_back(sensor_blackout);
        }
        else if (condition.type == ariac_msgs::msg::Condition::SUBMISSION)
        {
            auto submitted_order_id = condition.submission_condition.order_id;
            auto sensor_blackout = std::make_shared<ariac_common::SensorBlackoutOnSubmission>(duration, sensors_to_disable_vect, submitted_order_id);
            impl_->on_submission_sensor_blackouts_.push_back(sensor_blackout);
        }
    }

    //==============================================================================
    void
    TaskManagerPlugin::StoreChallenges(const std::vector<ariac_msgs::msg::Challenge::SharedPtr> &challenges)
    {
        for (auto challenge : challenges)
        {
            auto challenge_type = challenge->type;

            // std::shared_ptr<ariac_common::KittingTask> faulty_part_challenge = nullptr;
            // std::shared_ptr<ariac_common::AssemblyTask> dropped_part_challenge = nullptr;
            std::shared_ptr<ariac_common::SensorBlackout> sensor_blackout_challenge = nullptr;
            std::shared_ptr<ariac_common::RobotMalfunction> robot_malfunction_challenge = nullptr;
            // std::shared_ptr<ariac_common::CombinedTask> human_challenge = nullptr;

            if (challenge_type == ariac_msgs::msg::Challenge::SENSOR_BLACKOUT)
            {
                BuildSensorBlackoutChallenge(challenge->sensor_blackout_challenge);
            }
            else if (challenge_type == ariac_msgs::msg::Challenge::ROBOT_MALFUNCTION)
            {
                BuildRobotMalfunctionChallenge(challenge->robot_malfunction_challenge);
            }
            else
            {
                RCLCPP_ERROR_STREAM(impl_->ros_node_->get_logger(), "Unknown challenge type: " << int(challenge_type));
            }
        }
    }

    //==============================================================================
    void TaskManagerPlugin::UpdateSensorsHealth()
    {
        // publish on /ariac/sensor_health topic
        auto sensor_message = ariac_msgs::msg::Sensors();
        sensor_message.break_beam = impl_->break_beam_sensor_health_;
        sensor_message.proximity = impl_->proximity_sensor_health_;
        sensor_message.laser_profiler = impl_->laser_profiler_sensor_health_;
        sensor_message.lidar = impl_->lidar_sensor_health_;
        sensor_message.camera = impl_->camera_sensor_health_;
        sensor_message.logical_camera = impl_->logical_camera_sensor_health_;

        impl_->sensor_health_pub_->publish(sensor_message);
    }

    //==============================================================================
    void TaskManagerPlugin::ActivateAllSensors()
    {
        // publish on /ariac/sensor_health topic
        auto sensor_message = ariac_msgs::msg::Sensors();
        sensor_message.break_beam = impl_->break_beam_sensor_health_;
        sensor_message.proximity = impl_->proximity_sensor_health_;
        sensor_message.laser_profiler = impl_->laser_profiler_sensor_health_;
        sensor_message.lidar = impl_->lidar_sensor_health_;
        sensor_message.camera = impl_->camera_sensor_health_;
        sensor_message.logical_camera = impl_->logical_camera_sensor_health_;
        impl_->sensor_health_pub_->publish(sensor_message);
        RCLCPP_INFO_STREAM(impl_->ros_node_->get_logger(), "Activated all sensors");
    }

    //==============================================================================
    void TaskManagerPlugin::DeactivateAllSensors()
    {
        // publish on /ariac/sensor_health topic
        auto sensor_message = ariac_msgs::msg::Sensors();
        sensor_message.break_beam = false;
        sensor_message.proximity = false;
        sensor_message.laser_profiler = false;
        sensor_message.lidar = false;
        sensor_message.camera = false;
        sensor_message.logical_camera = false;
        impl_->sensor_health_pub_->publish(sensor_message);
        RCLCPP_INFO_STREAM(impl_->ros_node_->get_logger(), "Deactivated all sensors");
    }

    //==============================================================================
    int TaskManagerPlugin::StartAllRobots()
    {
        // publish on /ariac/robot_health topic
        auto robot_health_message = ariac_msgs::msg::Robots();
        robot_health_message.ceiling_robot = true;
        robot_health_message.floor_robot = true;
        impl_->robot_health_pub_->publish(robot_health_message);
        RCLCPP_INFO_STREAM(impl_->ros_node_->get_logger(), "Started all robots");
        return 1;
    }

    // ==================================== //
    // Services
    // ==================================== //

    //==============================================================================
    bool TaskManagerPlugin::SubmitOrderServiceCallback(
        const std::shared_ptr<ariac_msgs::srv::SubmitOrder::Request> request,
        std::shared_ptr<ariac_msgs::srv::SubmitOrder::Response> response)
    {
        std::lock_guard<std::mutex> lock(impl_->lock_);

        auto submitted_order_id = request->order_id;

        // If the order id is part of the trial orders, then it is submitted
        if (std::find(impl_->trial_orders_.begin(), impl_->trial_orders_.end(), submitted_order_id) != impl_->trial_orders_.end())
        {
            // We need to set the submission time to be able to calculate the score
            for (auto &order : impl_->all_orders_)
            {
                if (order->GetId() == submitted_order_id)
                {
                    order->SetIsSubmitted();
                    order->SetSubmittedTime(impl_->elapsed_time_);
                }
            }
            // Push the order id to the list of submitted orders
            impl_->submitted_orders_.push_back(submitted_order_id);
            response->success = true;
            response->message = "Order submitted successfully";
            impl_->trial_orders_.erase(std::remove(impl_->trial_orders_.begin(), impl_->trial_orders_.end(), submitted_order_id), impl_->trial_orders_.end());
        }
        else
        {
            response->success = false;
            response->message = "Order is not part of the trial or has already been submitted";
        }
        return true;
    }

    //==============================================================================
    bool TaskManagerPlugin::StartCompetitionServiceCallback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        std::lock_guard<std::mutex> lock(impl_->lock_);

        // gzdbg << "\n";
        // gzdbg << "StartCompetitionServiceCallback\n";

        (void)request;

        if (impl_->current_state_ == ariac_msgs::msg::CompetitionState::READY)
        {
            impl_->current_state_ = ariac_msgs::msg::CompetitionState::STARTED;
            // publish the competition state
            // impl_->competition_state_pub_->publish(impl_->current_state_);
            response->success = true;
            response->message = "Competition started successfully!";

            // Activate all sensors
            // ActivateAllSensors();
            // Start all robot controllers
            StartAllRobots();

            return true;
        }
        response->success = false;
        response->message = "ERROR: Cannot start competition if current state is not READY";
        return true;
    }

    //==============================================================================
    bool TaskManagerPlugin::EndCompetitionServiceCallback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        std::lock_guard<std::mutex> lock(this->impl_->lock_);

        (void)request;

        this->impl_->current_state_ = ariac_msgs::msg::CompetitionState::ENDED;

        response->success = true;
        response->message = "Competition ended successfully!";
        return true;
    }

    // Register this plugin with the simulator
    GZ_REGISTER_WORLD_PLUGIN(TaskManagerPlugin)
} // namespace ariac_plugins