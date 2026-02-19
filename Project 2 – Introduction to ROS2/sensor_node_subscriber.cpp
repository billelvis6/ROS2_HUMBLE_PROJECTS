
    #include <functional>
    #include <memory>
    #include <sstream>

    #include "rclcpp/rclcpp.hpp"
    #include "std_msgs/msg/string.hpp"

    using std::placeholders::_1;

    class SensorSubscriber : public rclcpp::Node
    {
    public:
      SensorSubscriber()
      : Node("sensor_node_subscriber")
      {
        subscription_ = this->create_subscription<std_msgs::msg::String>(
          "sensor_data", 10, std::bind(&SensorSubscriber::sensor_callback, this, _1));
      }

    private:
      void sensor_callback(const std_msgs::msg::String & msg) const
      {
        // std::string message = "Data received: " + msg.data + ": " + is_correct_value(msg.data);
        // RCLCPP_INFO(this->get_logger(), "%s", message.c_str());
        RCLCPP_INFO(this->get_logger(), "Data received: %s : %s", msg.data.c_str(), (is_correct_value(msg.data)).c_str());
      }
      rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;

      std::string is_correct_value(const std::string &txt) const{
        std::stringstream ss(txt);
        std::string buffer;
        int T, H, P;

        ss >> buffer >> buffer >> T >> buffer  
          >> buffer >> buffer >> H >> buffer 
          >> buffer >> buffer >> P >> buffer;
        
        return (15 <= T && T <= 35 && 30 <= H && H <= 70 && 950 <= P && P <= 1050)? "correct values":"incorrect values";
    }
    };


    int main(int argc, char * argv[])
    {
      rclcpp::init(argc, argv);
      rclcpp::spin(std::make_shared<SensorSubscriber>());
      rclcpp::shutdown();
      return 0;
    }
            