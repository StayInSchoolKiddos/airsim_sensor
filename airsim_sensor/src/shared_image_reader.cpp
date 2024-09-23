#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <sensor_msgs/msg/image.hpp>

class ImagePublisher : public rclcpp::Node
{

private:
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    key_t key;
    int shmid;
    uint8_t* shm_data;
    struct ImgData
    {
        unsigned long msg_id = 0;
        uint8_t data[512 * 512 * 3];
    }ImgDataMsg;
    sensor_msgs::msg::Image img_msg;

public:
    ImagePublisher()
        : Node("ue_image_publisher")
    {
        publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/ue_image_data", 10);
        key = ftok("/workspaces/data.conf", 1);
        shmid = shmget(key, sizeof(ImgDataMsg), 0666|IPC_CREAT);
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

        // Initialize image message metadata
        img_msg.width = 512;
        img_msg.height = 512;
        std::string encoding = "rgb8";
        img_msg.encoding = encoding;
        img_msg.is_bigendian = 0;
        img_msg.step = 512 * 3; // width * number of channels
        img_msg.data.resize(512 * 512 * 3); // width * height * number of channels
    }

    void run()
    {
        static unsigned long msg_id = 0;
        if (shmid != -1 && shm_data != (void*)-1)
        {
            unsigned long* q = (unsigned long*)shm_data;
            unsigned long id = *q;
            if (id > msg_id)
            {
                msg_id = id;
                q++;
                uint8_t* d = (uint8_t*)q;
                for (int i = 0; i < img_msg.data.size(); i++)
                {
                    img_msg.data[i] = *d;
                    d++;
                }
                img_msg.header.stamp = this->now();
                publisher_->publish(img_msg);
                RCLCPP_INFO(this->get_logger(), "Published image with msg_id: %lu", msg_id);
            }
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
    std::shared_ptr<ImagePublisher> img_publisher = std::make_shared<ImagePublisher>();
    while(rclcpp::ok())
    {
        img_publisher->run();
    }
    img_publisher->detach_shm();
    rclcpp::shutdown();
    return 0;
}
