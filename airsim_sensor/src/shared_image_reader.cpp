#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <sensor_msgs/msg/image.hpp>
#include <cstring>  // For memcpy

class ImagePublisher : public rclcpp::Node
{
private:
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    key_t key;
    int shmid;
    uint8_t* shm_data;
    int data_sz = 512 * 512 * 3;  // RGB data size
    
    sensor_msgs::msg::Image img_msg;

public:
    ImagePublisher()
        : Node("ue_image_publisher")
    {
        publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/ue_image_data", 100);

        // Create shared memory key
        key = ftok("/home/justin/Documents/Unreal Projects/TestUE5/Source/data.conf", 1);
        shmid = shmget(key, data_sz + sizeof(unsigned long), 0666 | IPC_CREAT);
        
        if (shmid == -1)
        {
            RCLCPP_ERROR(this->get_logger(), "ERROR CREATING SHARED MEMORY SEGMENT FOR IMAGE DATA");
        }
        else
        {
            shm_data = (uint8_t*)shmat(shmid, NULL, 0);
            if (shm_data == (void*)-1)
            {
                RCLCPP_ERROR(this->get_logger(), "FAILED TO ATTACH SHARED MEMORY");
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "SUCCESSFULLY CREATED SHARED MEMORY SEGMENT FOR IMAGE DATA");
            }
        }

        // Preallocate image data memory
        img_msg.width = 512;
        img_msg.height = 512;
        img_msg.encoding = "bgr8";
        img_msg.is_bigendian = 0;
        img_msg.step = 512 * 3;  // RGB format
        img_msg.data.resize(data_sz);  // Preallocate once
    }

    void run()
    {
        static unsigned long msg_id = 0;

        if (shmid != -1 && shm_data != (void*)-1)
        {
            // Check for new data
            unsigned long* q = (unsigned long*)shm_data;
            unsigned long id = *q;
            
            if (id > msg_id)
            {
                msg_id = id;

                // Use memcpy to quickly copy image data
                memcpy(img_msg.data.data(), shm_data + sizeof(unsigned long), data_sz);

                img_msg.header.stamp = this->now();
                publisher_->publish(img_msg);

                RCLCPP_INFO(this->get_logger(), "Published image with msg_id: %lu", msg_id);
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "Waiting for more data");
            }
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "No image data available");
        }
    }

    void detach_shm()
    {
        shmdt(shm_data);
        RCLCPP_INFO(this->get_logger(), "Shared memory detached.");
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto img_publisher = std::make_shared<ImagePublisher>();

    rclcpp::WallRate loop_rate(1000);  // Run the loop at 1000Hz
    while (rclcpp::ok())
    {
        img_publisher->run();
        loop_rate.sleep();
    }

    img_publisher->detach_shm();
    rclcpp::shutdown();
    return 0;
}
