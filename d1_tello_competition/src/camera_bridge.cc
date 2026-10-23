// RGB cameras use this small Gazebo Transport -> ROS 2 adapter.
#include <ignition/transport/Node.hh>
#include <ignition/msgs/image.pb.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <functional>
#include <map>

int main(int argc,char **argv){
  rclcpp::init(argc,argv);
  auto node=std::make_shared<rclcpp::Node>("competition_cameras");
  ignition::transport::Node transport;
  std::map<std::string,rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr> publishers;
  for(std::string role:{"overview","dog","uav","fpv","uav_front","entry"}){
    const std::string topic="/competition/"+role+"/image";
    auto publisher=node->create_publisher<sensor_msgs::msg::Image>(topic,rclcpp::SensorDataQoS());
    publishers[role]=publisher;
    std::function<void(const ignition::msgs::Image&)> callback=[publisher,role](const ignition::msgs::Image &m){
      if(m.pixel_format_type()!=ignition::msgs::RGB_INT8)return;
      sensor_msgs::msg::Image image;
      image.header.stamp.sec=m.header().stamp().sec();
      image.header.stamp.nanosec=m.header().stamp().nsec();
      image.header.frame_id=role+"_camera_optical";
      image.width=m.width();image.height=m.height();
      image.encoding="rgb8";image.is_bigendian=false;image.step=m.step();
      image.data.assign(m.data().begin(),m.data().end());
      publisher->publish(image);
    };
    if(!transport.Subscribe(topic,callback))return 1;
  }
  rclcpp::spin(node);
  for(const auto &item:publishers)transport.Unsubscribe("/competition/"+item.first+"/image");
  rclcpp::shutdown();
  return 0;
}
