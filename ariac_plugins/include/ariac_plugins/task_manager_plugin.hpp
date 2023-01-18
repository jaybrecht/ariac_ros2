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
#include <ariac_msgs/msg/kitting_task.hpp>
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

        void ActivateAllSensors();
        void StartConveyor();
        std::shared_ptr<ariac_common::KittingTask> BuildKittingTask(const ariac_msgs::msg::KittingTask& _kitting_task);
        std::shared_ptr<ariac_common::AssemblyTask> BuildAssemblyTask(const ariac_msgs::msg::AssemblyTask &_assembly_task);
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
        void StoreChallenges(const std::vector<ariac_msgs::msg::Challenge::SharedPtr>& challenges);

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
        // /**
        //  * @brief Process sensor blackout challenges
        //  *
        //  */
        // void ProcessSensorBlackoutChallenge();
        // /**
        //  * @brief Process robot breakdown challenges
        //  *
        //  */
        // void ProcessRobotBreakdownChallenge();
        // /**
        //  * @brief Check which order(s) should be announced
        //  *
        //  * @param _current_sim_time Current simulation time
        //  */
        // void ProcessOrdersToAnnounce(double _current_sim_time);
        // /**
        //  * @brief Announce temporal orders
        //  *
        //  * @param _current_sim_time Current simulation time
        //  */
        // void ProcessTemporalOrders(double _current_sim_time);
        // /**
        //  * @brief Announce orders during kitting
        //  *
        //  * @param _current_sim_time Current simulation time
        //  */
        // void ProcessDuringKittingOrders(double _current_sim_time);
        // /**
        //  * @brief Announce orders during assembly
        //  *
        //  * @param _current_sim_time Current simulation time
        //  */
        // void ProcessDuringAssemblyOrders(double _current_sim_time);
        // /**
        //  * @brief Announce orders after kitting
        //  *
        //  * @param _current_sim_time Current simulation time
        //  */
        // void ProcessAfterKittingOrders(double _current_sim_time);
        // /**
        //  * @brief Announce orders after assembly
        //  *
        //  * @param _current_sim_time Current simulation time
        //  */
        // void ProcessAfterAssemblyOrders(double _current_sim_time);
        /**
         * @brief Callback function for the topic 'trial_config'
         *
         * @param _msg Shared pointer to the message
         */
        void OnTrialCallback(const ariac_msgs::msg::Trial::SharedPtr _msg);
       
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