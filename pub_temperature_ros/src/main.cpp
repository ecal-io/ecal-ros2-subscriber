#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/temperature.hpp>

using namespace std::chrono_literals;

class TemperaturePublisher : public rclcpp::Node
{
public:
  TemperaturePublisher() : Node("pub_temperature_ros")
  {
    pub_ = this->create_publisher<sensor_msgs::msg::Temperature>("temperature", 10);
    msg_ = sensor_msgs::msg::Temperature();

    FillMessage(msg_);

    auto timer_callback =
      [this]() -> void {
        RCLCPP_INFO(this->get_logger(), "Publishing Temperature");
        this->pub_->publish(this->msg_);
      };

    auto timer_ = this->create_wall_timer(100ms, timer_callback);
  }

  static void FillMessage(sensor_msgs::msg::Temperature& msg)
  {
    // header
    msg.header.stamp.sec     = 1;
    msg.header.stamp.nanosec = 2;
    msg.header.frame_id      = "id";

    // temperature
    msg.temperature = 42.0;

    // variance
    msg.variance = 1.234;
  }

private:
  rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr  pub_;
  sensor_msgs::msg::Temperature                                msg_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TemperaturePublisher>());
  rclcpp::shutdown();
  return 0;
}
