// chatgpt generated code 3.08.23

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

void callback(const std_msgs::msg::Float32MultiArray::SharedPtr message) {
    RCLCPP_INFO(rclcpp::get_logger("my_subscriber"), "Received array: [%f, %f]", message->data[0], message->data[1]);
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("my_subscriber");

    auto subscription = node->create_subscription<std_msgs::msg::Float32MultiArray>("my_topic", 10, callback);

    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
