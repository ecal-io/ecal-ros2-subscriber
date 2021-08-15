#ifdef _MSC_VER
#pragma warning(push)
#pragma warning(disable : 4456)
#endif
#include <rclcpp/rclcpp.hpp>
#ifdef _MSC_VER
#pragma warning(pop)
#endif

#include <ecal/ecal.h>
#include <ecal/msg/protobuf/publisher.h>

#include <functional>
#include <memory>

#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/temperature.hpp>

#ifdef _MSC_VER
#pragma warning(push)
#pragma warning(disable : 4251)
#endif
#include "sensor_msgs/navsatfix.pb.h"
#include "sensor_msgs/temperature.pb.h"
#ifdef _MSC_VER
#pragma warning(pop)
#endif

class GatewayNode : public rclcpp::Node
{
public:
  GatewayNode() : Node("ros2ecal")
  {
    // pub <pb::NavSatFix> / sub <sensor_msgs::msg::NavSatFix>
    pub_nav_ = eCAL::protobuf::CPublisher<pb::sensor_msgs::NavSatFix>("navsatfix_ecal");
    sub_nav_ = this->create_subscription<sensor_msgs::msg::NavSatFix>("navsatfix", 10, std::bind(&GatewayNode::navsatfix_cb, this, std::placeholders::_1));

    // pub <pb::Temperature> /sub <sensor_msgs::msg::Temperature>
    pub_tmp_ = eCAL::protobuf::CPublisher<pb::sensor_msgs::Temperature>("temperature_ecal");
    sub_tmp_ = this->create_subscription<sensor_msgs::msg::Temperature>("temperature", 10, std::bind(&GatewayNode::temperature_cb, this, std::placeholders::_1));
  }

private:
  void navsatfix_cb(const sensor_msgs::msg::NavSatFix::SharedPtr msg) //-V801
  {
    // clear proto message
    msg_nav_.Clear();

    // header
    auto header = msg_nav_.mutable_header();
    header->mutable_stamp()->set_sec(msg->header.stamp.sec);
    header->mutable_stamp()->set_nanosec(msg->header.stamp.nanosec);
    header->set_frame_id(msg->header.frame_id);

    // status
    switch (msg->status.status)
    {
    case sensor_msgs::msg::NavSatStatus::STATUS_FIX:       // unaugmented fix
      msg_nav_.mutable_status()->set_status(pb::sensor_msgs::NavSatStatus::STATUS_FIX);
      break;
    case sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX:    // unable to fix position
      msg_nav_.mutable_status()->set_status(pb::sensor_msgs::NavSatStatus::STATUS_NO_FIX);
      break;
    case sensor_msgs::msg::NavSatStatus::STATUS_SBAS_FIX:  // with satellite-based augmentation
      msg_nav_.mutable_status()->set_status(pb::sensor_msgs::NavSatStatus::STATUS_SBAS_FIX);
      break;
    case sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX:  // with ground-based augmentation
      msg_nav_.mutable_status()->set_status(pb::sensor_msgs::NavSatStatus::STATUS_GBAS_FIX);
      break;
    }

    // latitude
    msg_nav_.set_latitude(msg->latitude);

    // longitude
    msg_nav_.set_longitude(msg->longitude);

    // altitude
    msg_nav_.set_altitude(msg->altitude);

    // position_covariance[]
    for (auto it : msg->position_covariance)
    {
      msg_nav_.add_position_covariance(it);
    }

    // send it to eCAL
    pub_nav_.Send(msg_nav_);
  }

  void temperature_cb(const sensor_msgs::msg::Temperature::SharedPtr msg) //-V801
  {
    // clear proto message
    msg_tmp_.Clear();

    // header
    auto header = msg_nav_.mutable_header();
    header->mutable_stamp()->set_sec(msg->header.stamp.sec);
    header->mutable_stamp()->set_nanosec(msg->header.stamp.nanosec);
    header->set_frame_id(msg->header.frame_id);

    // temperature
    msg_tmp_.set_temperature(msg->temperature);

    // variance
    msg_tmp_.set_variance(msg->variance);

    // send it to eCAL
    pub_tmp_.Send(msg_tmp_);
  }

  // pub <pb::NavSatFix> / sub <sensor_msgs::msg::NavSatFix>
  eCAL::protobuf::CPublisher<pb::sensor_msgs::NavSatFix>          pub_nav_;
  pb::sensor_msgs::NavSatFix                                      msg_nav_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr    sub_nav_;

  // pub <pb::Temperature> / sub <sensor_msgs::msg::Temperature>
  eCAL::protobuf::CPublisher<pb::sensor_msgs::Temperature>        pub_tmp_;
  pb::sensor_msgs::Temperature                                    msg_tmp_;
  rclcpp::Subscription<sensor_msgs::msg::Temperature>::SharedPtr  sub_tmp_;
};


int main(int argc, char* argv[])
{
  // initialize eCAL and set process state
  eCAL::Initialize(argc, argv, "ros2ecal");
  eCAL::Process::SetState(proc_sev_healthy, proc_sev_level1, "I'm fine ..");

  // initialize ROS node
  rclcpp::init(argc, argv);

  // start ROS node
  auto node = std::make_shared<GatewayNode>();
  rclcpp::spin(node);

  // shutdown ROS node
  rclcpp::shutdown();

  // finalize eCAL
  eCAL::Finalize();

  return 0;
}
