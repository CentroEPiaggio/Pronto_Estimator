#include <rclcpp/rclcpp.hpp> 
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <gazebo_msgs/msg/link_states.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <builtin_interfaces/msg/time.hpp>

using std::placeholders::_1;

class linkStatesRepublisher : public rclcpp::Node
{
    public: 
        linkStatesRepublisher()
        : Node("link_states_republisher")
        {
            pose_pub = this->create_publisher<geometry_msgs::msg::PoseStamped>("pose_ground_truth", 10);
            twist_pub = this->create_publisher<geometry_msgs::msg::TwistStamped>("twist_ground_truth", 10);
            sub = this->create_subscription<gazebo_msgs::msg::LinkStates>(
                "gazebo/link_states", 10, std::bind(& linkStatesRepublisher::callback, this, _1));
        }

    private:
        void callback(const gazebo_msgs::msg::LinkStates::SharedPtr msg)
        {
            geometry_msgs::msg::PoseStamped pose;
            geometry_msgs::msg::TwistStamped twist;
            // std::string base_link = "anymal_c::base";
            std::string base_link = "solo12::base_link";

            auto index = std::find(msg->name.begin(), msg->name.end(), base_link);

            if(index != msg->name.end()){
                int i = index - msg->name.begin();
                time = this->get_clock()->now();
                pose.header.set__stamp(time);
                pose.header.set__frame_id("base_link");

                // set pose and publish
                pose.pose.set__orientation(msg->pose[i].orientation);
                pose.pose.set__position(msg->pose[i].position);
                pose_pub->publish(pose);

                twist.header.set__stamp(time);
                twist.header.set__frame_id("base_link");

                // set twist and publish
                twist.twist.set__linear(msg->twist[i].linear);
                twist.twist.set__angular(msg->twist[i].angular);
                twist_pub->publish(twist);

            }
            else{
                RCLCPP_ERROR(this->get_logger(), "%s not found", base_link.c_str());
            }
        }

        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub;
        rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub;
        rclcpp::Subscription<gazebo_msgs::msg::LinkStates>::SharedPtr sub;
        builtin_interfaces::msg::Time time;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<linkStatesRepublisher>());
    rclcpp::shutdown();
    return 0;
}