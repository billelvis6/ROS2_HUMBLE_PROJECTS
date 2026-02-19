
    #include <chrono>
    #include <functional>
    #include <memory>
    #include <string>
    #include <random>

    #include "rclcpp/rclcpp.hpp"
    #include "std_msgs/msg/string.hpp"

    using namespace std::chrono_literals;


    class SensorPublisher : public rclcpp::Node
    {
    private:
      rclcpp::TimerBase::SharedPtr timer_;
      rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
      int temp, pression, hum;
      // float temp, pression, hum;
      void timer_callback()
      {
        temp = nbr_generator(15, 35);
        hum = nbr_generator(30, 70);
        pression = nbr_generator(950, 1050);
        auto message = std_msgs::msg::String();
        message.data = "T = " + std::to_string(temp) + "°C, H = " + std::to_string(hum) + "%, P = " + std::to_string(pression) + "hPa";
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
        publisher_->publish(message);
      }

      int nbr_generator(int min_value, int max_value){
        static std::random_device rd;
        static std::mt19937 gen(rd());
        std::uniform_int_distribution<int> distrib(min_value, max_value);

        return distrib(gen);
    }


    public:
      SensorPublisher()
      : Node("sensor_node_publisher")
      {
        publisher_ = this->create_publisher<std_msgs::msg::String>("sensor_data", 10);
        timer_ = this->create_wall_timer(
          500ms, std::bind(&SensorPublisher::timer_callback, this));
      }
    };

    int main(int argc, char * argv[])
    {
      rclcpp::init(argc, argv);
      rclcpp::spin(std::make_shared<SensorPublisher>());
      rclcpp::shutdown();
      return 0;
    }
            