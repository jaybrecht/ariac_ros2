#ifndef ARIAC_PLUGINS__TASK_MANAGER_PLUGIN_HPP_
#define ARIAC_PLUGINS__TASK_MANAGER_PLUGIN_HPP_

// Gazebo
#include <gazebo/common/Plugin.hh>
// C++
#include <memory>
#include <vector>
// Services
#include <std_srvs/srv/trigger.hpp>
// Messages
#include <ariac_msgs/msg/order.hpp>
#include <ariac_msgs/msg/order_condition.hpp>
#include <ariac_msgs/msg/trial.hpp>
#include <ariac_msgs/msg/challenge.hpp>
#include <ariac_msgs/msg/parts.hpp>
#include <ariac_msgs/msg/kitting_task.hpp>
#include <ariac_msgs/msg/competition_state.hpp>
#include <ariac_msgs/srv/submit_order.hpp>
// ARIAC
#include <ariac_plugins/ariac_common.hpp>

namespace ariac_plugins
{
    // Forward declaration of private data class.
    class TaskManagerPluginPrivate;

    /// Example ROS-powered Gazebo plugin with some useful boilerplate.
    /// \details This is a `WorldPlugin`, but it could be any supported Gazebo plugin type, such as
    /// System, Visual, GUI, World, Sensor, etc.
    class TaskManagerPlugin : public gazebo::WorldPlugin
    {
    public:
        /// Constructor
        TaskManagerPlugin();

        /// Destructor
        virtual ~TaskManagerPlugin();

        /**
         * @brief Publish the competition state
         *
         * @param _state State of the competition
         */
        void PublishCompetitionState(unsigned int _state);
        /**
         * @brief Activate all sensor types
         */
        void ActivateAllSensors();
        /**
         * @brief Deactivate all sensor types
         */
        void DeactivateAllSensors();
        /**
         * @brief Deactivate some sensor types
         * @param _sensor_types List of sensor types to deactivate
         */
        void DeactivateSensors(std::vector<std::string> _sensor_types);
        /**
         * @brief Activate some sensor types
         * @param _sensor_types List of sensor types to activate
         */
        void ActivateSensors(std::vector<std::string> _sensor_types);
        /**
         * @brief Start the controllers for all robots
         */
        int StartAllRobots();
        /**
         * @brief Stop the controllers for all robots
         */
        void StopAllRobots();
        /**
         * @brief Start the controllers for the floor robot
         */
        void StartFloorRobot();
        /**
         * @brief Start the controllers for the ceiling robot
         */
        void StartCeilingRobot();
        /**
         * @brief Stop the controllers for the ceiling robot
         */
        void StopFloorRobot();
        /**
         * @brief Stop the controllers for the floor robot
         */
        void StopCeilingRobot();
        /**
         * @brief Build a kitting task from ROS message
         *
         * @param _kitting_task Kitting task ROS message
         * @return std::shared_ptr<ariac_common::KittingTask> Pointer to the kitting task
         */
        std::shared_ptr<ariac_common::KittingTask> BuildKittingTask(const ariac_msgs::msg::KittingTask &_kitting_task);
        /**
         * @brief Build an assembly task from ROS message
         *
         * @param _assembly_task Assembly task ROS message
         * @return std::shared_ptr<ariac_common::AssemblyTask> Pointer to the assembly task
         */
        std::shared_ptr<ariac_common::AssemblyTask> BuildAssemblyTask(const ariac_msgs::msg::AssemblyTask &_assembly_task);
        /**
         * @brief Build a combined task from ROS message
         *
         * @param _combined_task Combined task ROS message
         * @return std::shared_ptr<ariac_common::CombinedTask> Pointer to the combined task
         */
        std::shared_ptr<ariac_common::CombinedTask> BuildCombinedTask(const ariac_msgs::msg::CombinedTask &_combined_task);

        /**
         * @brief Store the orders
         * @param orders Orders retrieved from the topic /ariac/task_manager/trial_config
         */
        void StoreOrders(const std::vector<ariac_msgs::msg::OrderCondition::SharedPtr> &orders);

        /**
         * @brief Store the challenges
         * @param challenges Challenges retrieved from the topic /ariac/task_manager/trial_config
         */
        void StoreChallenges(const std::vector<ariac_msgs::msg::Challenge::SharedPtr> &challenges);

        /**
         * @brief Callback for the start competition service
         * @param request
         * @param response
         * @return true
         * @return false
         */
        bool StartCompetitionServiceCallback(
            const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
            std::shared_ptr<std_srvs::srv::Trigger::Response> response);

        bool SubmitOrderServiceCallback(
            const std::shared_ptr<ariac_msgs::srv::SubmitOrder::Request> request,
            std::shared_ptr<ariac_msgs::srv::SubmitOrder::Response> response);

        /**
         * @brief
         * @param request
         * @param response
         * @return true
         * @return false
         */
        bool EndCompetitionServiceCallback(
            const std::shared_ptr<std_srvs::srv::Trigger::Request> _request,
            std::shared_ptr<std_srvs::srv::Trigger::Response> _response);

        /**
         * @brief Build an Order ROS message from an ariac_common::Order
         *
         * @param _order Order to build the ROS message from
         * @return const ariac_msgs::msg::Order& ROS message
         */
        const ariac_msgs::msg::Order BuildOrderMsg(std::shared_ptr<ariac_common::Order> _order);
        /**
         * @brief Build ariac_common::SensorBlackoutChallenge from ROS message and store them in a list.
         *
         * @param _challenge  SensorBlackoutChallenge ROS message
         */
        void BuildSensorBlackoutChallenge(const ariac_msgs::msg::SensorBlackoutChallenge &_challenge);
        /**
         * @brief Build ariac_common::RobotMalfunctionChallenge from ROS message and store them in a list.
         *
         * @param _challenge  RobotMalfunctionChallenge ROS message
         */
        void BuildRobotMalfunctionChallenge(const ariac_msgs::msg::RobotMalfunctionChallenge &_challenge);

        /**
         * @brief Build a KittinTask ROS message from ariac_common::KittingTask
         *
         * @param _task Pointer to ariac_common::KittingTask
         */
        const ariac_msgs::msg::KittingTask BuildKittingTaskMsg(std::shared_ptr<ariac_common::KittingTask> _task);
        /**
         * @brief Build a AssemblyTask ROS message from ariac_common::AssemblyTask
         *
         * @param _task Pointer to ariac_common::AssemblyTask
         */
        const ariac_msgs::msg::AssemblyTask BuildAssemblyTaskMsg(std::shared_ptr<ariac_common::AssemblyTask> _task);
        /**
         * @brief Build a CombinedTask ROS message from ariac_common::CombinedTask
         *
         * @param _task Pointer to ariac_common::CombinedTask
         */
        const ariac_msgs::msg::CombinedTask BuildCombinedTaskMsg(std::shared_ptr<ariac_common::CombinedTask> _task);
        /**
         * @brief Manage orders to be announced
         *
         * @param _current_sim_time Current simulation time
         */
        void ProcessOrdersToAnnounce();
        /**
         * @brief Process the challenges to be announced
         *
         * @param _elapsed_time Elapsed time since the start of the competition
         */
        void ProcessChallengesToAnnounce();
        /**
         * @brief Manage time-based orders to be submitted
         *
         * @param _current_sim_time Current simulation time
         */
        void ProcessTemporalOrders();
        /**
         * @brief Manage on part placement orders to be submitted
         *
         * @param _current_sim_time Current simulation time
         */
        void ProcessOnPartPlacementOrders();
        /**
         * @brief Manage on submission orders to be submitted
         *
         * @param _current_sim_time Current simulation time
         */
        void ProcessOnSubmissionOrders();

        void ProcessTemporalSensorBlackouts();

        void UpdateSensorsHealth();
        void ProcessInProgressSensorBlackouts();
        /**
         * @brief Callback function for the topic 'trial_config'
         *
         * @param _msg Shared pointer to the message
         */
        void
        OnTrialCallback(const ariac_msgs::msg::Trial::SharedPtr _msg);

        /**
         * @brief Callback function for the topic '/ariac/agv1_tray_contents'
         *
         * @param _msg Shared pointer to the message
         */
        void OnAGV1TrayContentsCallback(const ariac_msgs::msg::Parts::SharedPtr _msg);
        /**
         * @brief Callback function for the topic '/ariac/agv2_tray_contents'
         *
         * @param _msg Shared pointer to the message
         */
        void OnAGV2TrayContentsCallback(const ariac_msgs::msg::Parts::SharedPtr _msg);
        /**
         * @brief Callback function for the topic '/ariac/agv3_tray_contents'
         *
         * @param _msg Shared pointer to the message
         */
        void OnAGV3TrayContentsCallback(const ariac_msgs::msg::Parts::SharedPtr _msg);
        /**
         * @brief Callback function for the topic '/ariac/agv4_tray_contents'
         *
         * @param _msg Shared pointer to the message
         */
        void OnAGV4TrayContentsCallback(const ariac_msgs::msg::Parts::SharedPtr _msg);

        /**
         * @brief Build and publish an order message
         *
         * @param _order Order to be published
         */
        // void AnnounceOrder(std::shared_ptr<ariac_common::Order> _order);

        /// \brief Load the plugin.
        virtual void
        Load(gazebo::physics::WorldPtr _world, sdf::ElementPtr _sdf) override;

    protected:
        /// Optional callback to be called at every simulation iteration.
        virtual void OnUpdate();

    private:
        /// Recommended PIMPL pattern. This variable should hold all private
        /// data members.
        std::unique_ptr<TaskManagerPluginPrivate> impl_;
    };
} // namespace ariac_plugins

#endif // ARIAC_PLUGINS__TASK_MANAGER_PLUGIN_HPP_