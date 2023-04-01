// chatgpt generated code 3.08.23

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("my_publisher");

    auto publisher = node->create_publisher<std_msgs::msg::Float32MultiArray>("my_topic", 10);

    std_msgs::msg::Float32MultiArray message;
    message.data = {1.23, 4.56};

    rclcpp::Rate loop_rate(10);

    while (rclcpp::ok()) {
        publisher->publish(message);
        rclcpp::spin_some(node);
        loop_rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}
