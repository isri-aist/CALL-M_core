#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>  // Include termios header
#include <iostream>
#include <string>
#include <chrono>
#include <cmath>

class SonarPublisher : public rclcpp::Node
{
public:
    SonarPublisher(): Node("sonar_range_node")
    {
        initialize_params();
        refresh_params();

        RCLCPP_INFO(this->get_logger(), "\nStart sonar node for %s \nwith topic: %s",device_name.c_str(),topic_out.c_str());

        pub_ = this->create_publisher<sensor_msgs::msg::Range>(topic_out, 10);

        fd_ = open(device_name.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);  // Open the device for reading and writing

        if (fd_ < 0)
        {
            RCLCPP_ERROR(this->get_logger(), "Error opening sonar %s: %s", device_name.c_str(),strerror(errno));
            rclcpp::shutdown();
            return;
        }

        // Set the baud rate
        if (!set_interface_attribs(fd_, B57600))
        {
            RCLCPP_ERROR(this->get_logger(), "Error setting baud rate for sonar %s",device_name.c_str());
            rclcpp::shutdown();
            return;
        }

        RCLCPP_INFO(this->get_logger(), "\nConnected to sonar %s",device_name.c_str());

        timer_ = this->create_wall_timer(std::chrono::milliseconds(25), std::bind(&SonarPublisher::timer_callback, this));
    }

private:
    bool set_interface_attribs(int fd, int speed)
    {
        struct termios tty;
        if (tcgetattr(fd, &tty) != 0) {
            RCLCPP_ERROR(this->get_logger(), "Error from tcgetattr: %s", strerror(errno));
            return false;
        }

        cfsetospeed(&tty, speed);
        cfsetispeed(&tty, speed);

        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
        tty.c_iflag &= ~IGNBRK;         // disable break processing
        tty.c_lflag = 0;                // no signaling chars, no echo,
                                        // no canonical processing
        tty.c_oflag = 0;                // no remapping, no delays
        tty.c_cc[VMIN]  = 0;            // read doesn't block
        tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

        tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

        tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
                                        // enable reading
        tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
        tty.c_cflag |= 0;
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CRTSCTS;

        if (tcsetattr(fd, TCSANOW, &tty) != 0) {
            RCLCPP_ERROR(this->get_logger(), "Error from tcsetattr: %s", strerror(errno));
            return false;
        }
        return true;
    }

    void timer_callback()
    {
        char rxBuf[80];
        float range;
        // Maxbotix output is R1234, range in mm
        int i = 0;
        while (true) {
            int bytes_read = read(fd_, &rxBuf[i], 1);
            if (bytes_read < 0) {
                if (errno == EAGAIN || errno == EWOULDBLOCK) {
                    // No data available, break to avoid blocking
                    //RCLCPP_ERROR(this->get_logger(), "No data received from sonar %s",device_name.c_str());
                    break;
                } else {
                    // Handle other errors
                    RCLCPP_ERROR(this->get_logger(), "Error reading from sonar %s",device_name.c_str());
                    return;
                }
            } else if (bytes_read == 0) {
                // End of file
                //RCLCPP_ERROR(this->get_logger(), "End of file, sonar %s",device_name.c_str());
                break;
            } else {
                if (rxBuf[i] == '\r') {
                    rxBuf[i] = '\0';
                    break;
                }
                i++;
            }
        }

        range = strtof(&rxBuf[1], nullptr) / 1000; // output in meters

        if (i != 0) {
            sensor_msgs::msg::Range rangeMsg;
            rangeMsg.range = range;
            rangeMsg.header.stamp = this->get_clock()->now();
            //RCLCPP_INFO(this->get_logger(), "Sonar msg: %s range: %f", rxBuf, range);
            pub_->publish(rangeMsg);
        }

        //else no data are read because the timer might be too fast, so we just don't publish anything

    }

    void initialize_params(){
      this->declare_parameter("device_name","/dev/ttyUSB0"); 
      this->declare_parameter("topic_out","sonar0/range"); 
    }

    void refresh_params(){
        this->device_name=get_parameter("device_name").as_string();
        this->topic_out=get_parameter("topic_out").as_string();
    }

    std::string topic_out;
    std::string device_name;
    rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    int fd_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SonarPublisher>());
    rclcpp::shutdown();
    return 0;
}
