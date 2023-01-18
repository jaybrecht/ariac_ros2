#ifndef ARIAC_PLUGINS__ARIAC_COMMON_HPP_
#define ARIAC_PLUGINS__ARIAC_COMMON_HPP_

// C++
#include <ostream>
#include <map>
#include <string>
#include <vector>
#include <memory>
// ROS
#include <geometry_msgs/msg/pose.hpp>
// Gazebo
#include <gazebo/gazebo.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>
// Messages
#include <ariac_msgs/msg/order.hpp>
#include <ariac_msgs/msg/condition.hpp>

namespace ariac_common
{

    // forward declarations
    class Order;
    class OrderTemporal;
    class Part;

    //==============================================================================
    std::string static ConvertPartTypeToString(unsigned int part_type)
    {
        switch (part_type)
        {
        case 10:
            return "battery";
            break;
        case 11:
            return "pump";
            break;
        case 12:
            return "sensor";
            break;
        case 13:
            return "regulator";
            break;
        default:
            return "unknown";
            break;
        }
    }

    //==============================================================================
    std::string static ConvertAssemblyStationToString(unsigned int station_id)
    {
        switch (station_id)
        {
        case 1:
            return "as1";
            break;
        case 2:
            return "as2";
            break;
        case 3:
            return "as3";
            break;
        case 4:
            return "as4";
            break;
        default:
            return "unknown";
            break;
        }
    }

    //==============================================================================
    std::string static ConvertDestinationToString(unsigned int destination, unsigned int agv_id)
    {
        switch (destination)
        {
        case 0:
            return "kitting";
            break;
        case 1:
            if (agv_id == 1 || agv_id == 2)
                return "as1";
            else if (agv_id == 3 || agv_id == 4)
                return "as3";

            else
                return "unknown";

            break;
        case 2:
            if (agv_id == 1 || agv_id == 2)
                return "as2";
            else if (agv_id == 3 || agv_id == 4)
                return "as4";
            else
                return "unknown";
            break;
        case 3:
            return "warehouse";
            break;
        default:
            return "unknown";
            break;
        }
    }

    //==============================================================================
    std::string static ConvertPartColorToString(unsigned int part_color)
    {
        switch (part_color)
        {
        case 1:
            return "red";
            break;
        case 2:
            return "green";
            break;
        case 3:
            return "blue";
            break;
        case 4:
            return "orange";
            break;
        case 5:
            return "purple";
            break;
        default:
            return "unknown";
            break;
        }
    }

    //==============================================================================
    /**
     * @brief Helper function to convert an order type to a string
     *
     * @param order_type Integer representing the order type
     * @return std::string Type of order as a string
     */
    std::string static ConvertOrderTypeToString(unsigned int order_type)
    {
        switch (order_type)
        {
        case 1:
            return "kitting";
            break;
        case 2:
            return "assembly";
            break;
        case 3:
            return "combined";
            break;
        default:
            return "unknown";
            break;
        }
    }

    //==============================================================================
    enum agv_destination
    {
        KITTING = 0,
        ASSEMBLY_FRONT = 1,
        ASSEMBLY_BACK = 2,
        WAREHOUSE = 3
    };

    struct OrderType
    {
        const static unsigned int KITTING{ariac_msgs::msg::Order::KITTING};
        const static unsigned int ASSEMBLY{ariac_msgs::msg::Order::ASSEMBLY};
        const static unsigned int COMBINED{ariac_msgs::msg::Order::COMBINED};
    };

    struct ConditionType
    {
        const static unsigned int TIME{ariac_msgs::msg::Condition::TIME};
        const static unsigned int PART_PLACE{ariac_msgs::msg::Condition::PART_PLACE};
        const static unsigned int SUBMISSION{ariac_msgs::msg::Condition::SUBMISSION};
    };

    //==============================================================================
    class Part
    {
    public:
        Part(unsigned int _color, unsigned int _type) : color_(_color), type_(_type) {}

    private:
        unsigned int color_;
        unsigned int type_;
    };

    //==============================================================================
    class KittingPart
    {
    public:
        KittingPart(unsigned _quadrant, const Part &_part) : quadrant_(_quadrant), part_(_part) {}

    private:
        unsigned int quadrant_;
        Part part_;
    };

    //==============================================================================
    class AssemblyPart
    {
    public:
        AssemblyPart(const Part &_part,
                     const ignition::math::Pose3d &_part_pose,
                     const ignition::math::Vector3<double> &_part_direction) : part_(_part),
                                                                               part_pose_(_part_pose),
                                                                               part_direction_(_part_direction) {}

    private:
        Part part_;
        ignition::math::Pose3d part_pose_;
        ignition::math::Vector3<double> part_direction_;
    };

    //==============================================================================
    class AssemblyTask
    {
    public:
        AssemblyTask(std::vector<unsigned int> _agv_numbers,
                     unsigned int _station,
                     const std::vector<AssemblyPart> &_products) : agv_numbers_(_agv_numbers),
                                                                   station_(_station),
                                                                   products_(_products) {}

    private:
        std::vector<unsigned int> agv_numbers_;
        unsigned int station_;
        std::vector<AssemblyPart> products_;
    };

    //==============================================================================
    /**
     * @brief Class to represent the fields for a kitting task
     */
    class KittingTask
    {
    public:
        KittingTask(unsigned int _agv_number,
                    unsigned int _tray_id,
                    unsigned int _destination,
                    const std::vector<KittingPart> &_products) : agv_number_(_agv_number),
                                                                 tray_id_(_tray_id),
                                                                 destination_(_destination),
                                                                 products_(_products) {}

    private:
        unsigned int agv_number_;
        unsigned int tray_id_;
        unsigned int destination_;
        std::vector<KittingPart> products_;
    };

    //==============================================================================
    /**
     * @brief Class to represent the fields for a combined task
     */
    class CombinedTask
    {
    public:
        CombinedTask(unsigned int _station,
                     const std::vector<AssemblyPart> &_products) : station_(_station),
                                                                   products_(_products) {}

    private:
        unsigned int station_;
        std::vector<AssemblyPart> products_;
    };

    //==============================================================================
    /**
     * @brief Base class for the sensor blackout challenges.
     *
     */
    class SensorBlackout
    {
    public:
        SensorBlackout(double _duration,
                       const std::vector<std::string> &_sensors_to_disable) : duration_(_duration),
                                                                              sensors_to_disable_(_sensors_to_disable) {}

    protected:
        //! Duration of the challenge
        double duration_;
        //! List of sensors to disable
        std::vector<std::string> sensors_to_disable_;
    };

    //==============================================================================
    /**
     * @brief Derived class for the sensor blackout challenge.
     *
     * This challenge is triggered at a specific simulation time.
     */
    class SensorBlackoutTemporal : public SensorBlackout
    {
    public:
        /**
         * @brief Construct a new SensorBlackoutTemporal object
         *
         * @param _duration Duration of the blackout
         * @param _time Time to trigger the challenge
         * @param _sensors_to_disable List of sensors to disable
         */
        SensorBlackoutTemporal(double _duration,
                               const std::vector<std::string> &_sensors_to_disable,
                               double _time) : SensorBlackout(_duration, _sensors_to_disable),
                                               time_(_time) {}

    private:
        //! Simulation time at which the challenge is triggered
        double time_;
    };

    //==============================================================================
    /**
     * @brief Class to represent the fields for the sensor blackout challenge
     *
     * This challenge is triggered during kitting
     */
    class SensorBlackoutKittingAction : public SensorBlackout
    {
    public:
        /**
         * @brief Construct a new SensorBlackoutKittingAction object
         *
         * @param _duration Duration of the blackout
         * @param _part_type Type of part to trigger the challenge
         * @param _part_color Color of part to trigger the challenge
         * @param _agv AGV on which the part is placed
         * @param _sensors_to_disable List of sensors to disable
         */

        SensorBlackoutKittingAction(double _duration,
                                    const std::vector<std::string> &_sensors_to_disable,
                                    std::string _part_type,
                                    std::string _part_color,
                                    unsigned int _agv) : SensorBlackout(_duration, _sensors_to_disable),
                                                         part_type_(_part_type),
                                                         part_color_(_part_color),
                                                         agv_(_agv) {}

    private:
        //! Type of part to trigger the challenge
        std::string part_type_;
        //! Color of part to trigger the challenge
        std::string part_color_;
        //! AGV on which the part is placed
        unsigned int agv_;
    }; // class SensorBlackoutKittingAction

    //==============================================================================
    /**
     * @brief Class to represent the fields for the sensor blackout challenge
     *
     * This challenge is triggered when a kit is submitted
     */
    class SensorBlackoutKittingSubmission : public SensorBlackout
    {
    public:
        /**
         * @brief Construct a new SensorBlackoutKittingSubmission object
         *
         * @param _duration Duration of the blackout
         * @param _agv AGV on which the part is placed
         * @param _destination Destination of the kit
         * @param _sensors_to_disable List of sensors to disable
         */

        SensorBlackoutKittingSubmission(double _duration,
                                        const std::vector<std::string> &_sensors_to_disable,
                                        unsigned int _agv,
                                        std::string _destination) : SensorBlackout(_duration, _sensors_to_disable),
                                                                    agv_(_agv),
                                                                    destination_(_destination) {}

    private:
        //! AGV on which the part is placed
        unsigned int agv_;
        //! Destination of the kit
        std::string destination_;
    }; // class SensorBlackoutKittingAction

    //==============================================================================
    /**
     * @brief Class to represent the fields for the sensor blackout challenge
     *
     * This challenge is triggered during assembly
     */
    class SensorBlackoutAssemblyAction : public SensorBlackout
    {
    public:
        /**
         * @brief Construct a new SensorBlackoutAssemblyActionn object
         *
         * @param _duration Duration of the blackout
         * @param _part_type Type of part to trigger the challenge
         * @param _part_color Color of part to trigger the challenge
         * @param _station Assembly station where the part is placed
         * @param _sensors_to_disable List of sensors to disable
         */

        SensorBlackoutAssemblyAction(double _duration,
                                     const std::vector<std::string> &_sensors_to_disable,
                                     std::string _part_type,
                                     std::string _part_color,
                                     std::string _station) : SensorBlackout(_duration, _sensors_to_disable), part_type_(_part_type),
                                                             part_color_(_part_color),
                                                             station_(_station)
        {
        }

    private:
        //! Type of part to trigger the blackout
        std::string part_type_;
        //! Color of part to trigger the blackout
        std::string part_color_;
        //! Assembly station where the part is placed
        std::string station_;
    }; // class SensorBlackoutAssemblyAction

    //==============================================================================
    /**
     * @brief Class to represent the fields for the sensor blackout challenge
     *
     * This challenge is triggered when an assembly is submitted
     */
    class SensorBlackoutAssemblySubmission : public SensorBlackout
    {
    public:
        /**
         * @brief Construct a new SensorBlackoutAssemblySubmission object
         *
         * @param _duration Duration of the blackout
         * @param _station Assembly station where the assembly is completed
         * @param _sensors_to_disable List of sensors to disable
         */

        SensorBlackoutAssemblySubmission(double _duration,
                                         std::vector<std::string> _sensors_to_disable,
                                         std::string _station) : SensorBlackout(_duration, _sensors_to_disable),
                                                                 station_(_station)
        {
        }

    private:
        //! Assembly station where the assembly is completed
        std::string station_;
    }; // class SensorBlackoutAssemblySubmission

    //==============================================================================
    /**
     * @brief Class to represent the fields for the robot malfunction challenge
     */
    class RobotMalfunction
    {
    public:
        /**
         * @brief Construct a new Robot Malfunction object
         *
         * @param _duration Time during which the robot(s) is (are) disabled
         * @param _robot_to_disable Robot(s) to disable
         * @param _part_type Type of part to trigger the challenge
         * @param _part_color Color of part to trigger the challenge
         * @param _agv AGV on which the part is placed
         */
        RobotMalfunction(double _duration,
                         std::string _part_type,
                         std::string _part_color,
                         unsigned int _agv,
                         std::vector<std::string> _robot_to_disable) : duration_(_duration),
                                                                       part_type_(_part_type),
                                                                       part_color_(_part_color),
                                                                       agv_(_agv),
                                                                       robot_to_disable_(_robot_to_disable) {}

    private:
        //! Time during which the robot(s) is(are) disabled
        double duration_;
        //! Type of part to trigger the challenge
        std::string part_type_;
        //! Color of part to trigger the challenge
        std::string part_color_;
        //! AGV on which the part is placed
        unsigned int agv_;
        //! List of robot(s) to disable
        std::vector<std::string> robot_to_disable_;
    }; // class RobotMalfunction

    //==============================================================================
    class Order
    {
    public:
        /**
         * @brief Construct a new Order object
         *
         * @param _order_id Unique id of the order
         * @param _order_type  Type of the order.
         * @param _high_priority Priority of the order (true or false)
         * @param _trial_time_limit  Time limit for the trial
         */
        Order(std::string _id,
              unsigned int _type,
              bool _priority,
              double _trial_time_limit) : announced_(false),
                                          id_(_id),
                                          type_(_type),
                                          priority_(_priority),
                                          trial_time_limit_(_trial_time_limit) {}

        /**
         * @brief Get the Id of the order
         *
         * @return std::string Id of the order
         */
        std::string GetId() const
        {
            return id_;
        }
        /**
         * @brief Get the type of the order
         *
         * @return std::string Type of the order
         */
        unsigned int GetType() const
        {
            return type_;
        }

        bool GetPriority() const
        {
            return priority_;
        }

        /**
         * @brief Get the priority of the order
         *
         * @return true High priority
         * @return false Normal priority
         */
        bool IsPriority() const
        {
            return priority_;
        }

        /**
         * @brief Get a shared pointer to a kitting task
         *
         * @return std::shared_ptr<KittingTask> Pointer to a kitting task
         */
        std::shared_ptr<KittingTask> GetKittingTask() const
        {
            return kitting_task_;
        }

        /**
         * @brief Get a shared pointer to an assembly task
         *
         * @return std::shared_ptr<AssemblyTask> Assembly task
         */
        std::shared_ptr<AssemblyTask> GetAssemblyTask() const
        {
            return assembly_task_;
        }

        /**
         * @brief Set the Kitting Task object for the order
         *
         * @param _kitting_task  Shared pointer to the kitting task for the order
         */
        virtual void SetKittingTask(std::shared_ptr<KittingTask> _kitting_task)
        {
            kitting_task_ = _kitting_task;
        }
        /**
         * @brief Set the Assembly Task object for the order
         *
         * @param _kitting_task  Shared pointer to the assembly task for the order
         */
        virtual void SetAssemblyTask(std::shared_ptr<AssemblyTask> _assembly_task)
        {
            assembly_task_ = _assembly_task;
        }

        /**
         * @brief Check whether or not the order has been announced
         *
         * @return true Order has already been announced
         * @return false Order has not been announced yet
         */
        virtual bool IsAnnounced() const
        {
            return announced_;
        }

        /**
         * @brief Set the order as announced
         */
        virtual void SetIsAnnounced()
        {
            announced_ = true;
        }

    protected:
        /**
         * @brief Whether or not this order has already been announced
         */
        bool announced_;
        /**
         * @brief id of the order
         */
        std::string id_;
        /**
         * @brief Type of the order. The possibilities are:
         * -0: kitting
         * -1: assembly
         * -2: combined
         */
        unsigned int type_;
        /**
         * @brief priority of the order
         * true: high-priority order
         * false: regular order
         */
        bool priority_;
        /**
         * @brief Time limit to perform the trial to which this order belongs
         */
        double trial_time_limit_;
        /**
         * @brief Time at which the order is submitted since it was announced
         */
        double submitted_time_;

        /**
         * @brief A pointer to the next order for the current order.
         *
         */
        std::shared_ptr<Order> next_order_ = nullptr;
        /**
         * @brief A pointer to the kitting task for this order.
         *
         */
        std::shared_ptr<KittingTask> kitting_task_ = nullptr;
        /**
         * @brief A pointer to the assembly task for this order.
         *
         */
        std::shared_ptr<AssemblyTask> assembly_task_ = nullptr;
    };
    //-- end class Order

    //==============================================================================
    /**
     * @brief Class to represent the fields for a time-based order
     */
    class OrderTemporal : public Order
    {
    public:
        /**
         * @brief Construct a new OrderTemporal object
         *
         * @param _id Unique id of the order
         * @param _type Type of the order
         * @param _priority Priority of the order (true or false)
         * @param _trial_time_limit Time limit for the trial
         * @param _announcement_time Time at which the order should be announced
         */
        OrderTemporal(std::string _id,
                      unsigned int _type,
                      bool _priority,
                      double _trial_time_limit,
                      double _announcement_time) : Order(_id, _type, _priority, _trial_time_limit),
                                                   announcement_time_(_announcement_time) {}

        /**
         * @brief Get the announcement time for the order
         *
         * @return double Time at which the order should be announced
         */
        double GetAnnouncementTime() const
        {
            return announcement_time_;
        }

    private:
        //! Time at which the order should be announced
        double announcement_time_;
    }; // class OrderTemporal

    //==============================================================================
    /**
     * @brief Class to represent the fields for an order announced during kitting
     */
    class OrderOnPartPlacement : public Order
    {
    public:
        /**
         * @brief Construct a new OrderDuringKitting object
         *
         * @param _id Unique id of the order
         * @param _type Type of the order
         * @param _priority Priority of the order (true or false)
         * @param _trial_time_limit Time limit for the trial
         * @param _agv Announcement: AGV on which the part is placed
         * @param _quadrant Announcement: Quadrant of the AGV on which the part is placed
         */
        OrderOnPartPlacement(std::string _id,
                             unsigned int _type,
                             bool _priority,
                             double _trial_time_limit,
                             unsigned int _agv,
                             unsigned int _quadrant) : Order(_id, _type, _priority, _trial_time_limit),
                                                       agv_(_agv), quadrant_(_quadrant) {}

    private:
        //! AGV on which the part is placed
        unsigned int agv_;
        //! Quadrant of the AGV on which the part is placed
        unsigned int quadrant_;
    }; // class OrderOnPartPlacement

    //==============================================================================
    /**
     * @brief Class to represent the fields for an order announced after kitting
     *
     */
    class OrderOnSubmission : public Order
    {
    public:
        /**
         * @brief Construct a new OrderAfterKitting object
         *
         * @param _id Unique id of the order
         * @param _type Type of the order
         * @param _priority Priority of the order (true or false)
         * @param _trial_time_limit Time limit for the trial
         * @param _order_id Id of the order submitted by the competitor
         */
        OrderOnSubmission(std::string _id,
                          unsigned int _type,
                          bool _priority,
                          double _trial_time_limit,
                          std::string _order_id) : Order(_id, _type, _priority, _trial_time_limit),
                                                   order_id_(_order_id) {}

    private:
        //! Id of the order submitted by the competitor
        std::string order_id_;
    }; // class OrderOnSubmission

    //==============================================================================
    /**
     * @brief Class to represent the fields for an order announced during assembly
     *
     */
    class OrderDuringAssembly : public Order
    {
    public:
        /**
         * @brief Construct a new OrderDuringAssembly object
         *
         * @param _id  Unique id of the order
         * @param _type  Type of the order
         * @param _priority  Priority of the order (true or false)
         * @param _trial_time_limit Time limit for the trial
         * @param _order_id Id of an order competitors started working on
         * @param _part_type Type of the part used in assembly
         * @param _part_color Color of the part used in assembly
         */
        OrderDuringAssembly(std::string _id,
                            unsigned int _type,
                            bool _priority,
                            double _trial_time_limit,
                            std::string _order_id,
                            std::string _part_type,
                            std::string _part_color) : Order(_id, _type, _priority, _trial_time_limit),
                                                       order_id_(_order_id),
                                                       part_type_(_part_type),
                                                       part_color_(_part_color) {}

    private:
        //! Id of an order competitors started working on
        std::string order_id_;
        //! Type of the part used in assembly
        std::string part_type_;
        //! Color of the part used in assembly
        std::string part_color_;
    }; // class OrderDuringAssembly

    //==============================================================================
    /**
     * @brief Class to represent the fields for an order announced after assembly
     *
     */
    class OrderAfterAssembly : public Order
    {
    public:
        /**
         * @brief Construct a new OrderAfterAssembly object
         *
         * @param _id Unique id of the order
         * @param _type Type of the order
         * @param _priority Priority of the order (true or false)
         * @param _trial_time_limit Time limit for the trial
         * @param _order_id Id of an order competitors submitted
         */
        OrderAfterAssembly(std::string _id,
                           unsigned int _type,
                           bool _priority,
                           double _trial_time_limit,
                           std::string _order_id) : Order(_id, _type, _priority, _trial_time_limit),
                                                    order_id_(_order_id) {}

    private:
        //! Id of an order competitors submitted
        std::string order_id_;
    }; // class OrderAfterAssembly

} // namespace ariac_common

#endif // ARIAC_PLUGINS__ARIAC_COMMON_HPP_