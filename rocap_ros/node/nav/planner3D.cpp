#include <chrono>
#include <future>
#include <memory>
#include <mutex>
#include <string>



#include "octomap/octomap.h"
// #include "octomap_msgs/octomap_msgs/srv/get_octomap.h"
#include "octomap_msgs/srv/get_octomap.hpp"
#include "octomap_msgs/conversions.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "nav_msgs/msg/path.hpp"
#include "AStarPlanner.h"


class Planner : public rclcpp::Node
{
  public:
    Planner() : Node("Planner3D") {


        publisher_ = this->create_publisher<nav_msgs::msg::Path>("/plan", 10);

        srvCallBackGroup = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        getOctomapClient = this->create_client<octomap_msgs::srv::GetOctomap>("/octomap_binary",
                                                                                rmw_qos_profile_services_default,
                                                                                srvCallBackGroup);

        timerCallbackGroup = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

        updateMapTimer = this->create_wall_timer(std::chrono::seconds(5),
                                            std::bind(&Planner::update_map_callback, this),
                                            timerCallbackGroup);


        plannerCallbackGroup = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
                                                                        "/goal",
                                                                        10,
                                                                        std::bind(&Planner::find_path_callback,
                                                                                  this, std::placeholders::_1));
    }

private:
    void update_map_callback() {
        auto request = std::make_shared<octomap_msgs::srv::GetOctomap::Request>();

        while (!getOctomapClient->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
            }
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
        }

        auto result = getOctomapClient->async_send_request(request);

        if (result.wait_for(std::chrono::seconds(5)) == std::future_status::ready) {
            if (result.future.valid()) {
                octomap_msgs::msg::Octomap map =  result.get()->map;
                mapMutex.lock();
                octreePtr = std::shared_ptr<octomap::AbstractOcTree>(octomap_msgs::binaryMsgToMap(map));
                mapMutex.unlock();
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "new map request succed");

            } else {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "new map request failed");
            }

        } else {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "map request timeout");
        }
    }

    void find_path_callback(const geometry_msgs::msg::PoseStamped::SharedPtr goal_msg) {
        geometry_msgs::msg::Pose start_msg;
        Coord start = Coord(start_msg.position.x, start_msg.position.y, start_msg.position.z+1.0);
        Coord goal = Coord(goal_msg->pose.position.x, goal_msg->pose.position.y, goal_msg->pose.position.z+2.0);

        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "begin find path start(%f,%f,%f) goal(%f,%f,%f)",
                                                                                    start.getX(),
                                                                                    start.getY(),
                                                                                    start.getZ(),
                                                                                    goal.getX(),
                                                                                    goal.getY(),
                                                                                    goal.getZ());

        bool cancel_flag;

        mapMutex.lock();
        AStartPlanner planner(octreePtr);
        mapMutex.unlock();
        auto path_future = planner.find_path_async(start, goal, &cancel_flag, 16);

        if (path_future.wait_for(std::chrono::seconds(5)) == std::future_status::ready && path_future.valid()) {
                nav_msgs::msg::Path path = path_future.get();
                if (path.poses.size() > 0) {
                    publisher_->publish(path);
                } else {
                    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "cannot find path to target");
                }
                return;
        }
        cancel_flag = true;
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "find path request timeout");
        return;
    }

    // callback
    rclcpp::TimerBase::SharedPtr updateMapTimer;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr publisher_;
    std::shared_ptr<rclcpp::Client<octomap_msgs::srv::GetOctomap>> getOctomapClient;

    // callback group
    rclcpp::CallbackGroup::SharedPtr timerCallbackGroup;
    rclcpp::CallbackGroup::SharedPtr srvCallBackGroup;
    rclcpp::CallbackGroup::SharedPtr plannerCallbackGroup;

    std::shared_ptr<octomap::AbstractOcTree> octreePtr;
    std::mutex mapMutex;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);

    std::shared_ptr<rclcpp::Node> node1 = std::make_shared<Planner>();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node1);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}