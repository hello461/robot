#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "vector"
#include "cstring"

using namespace std::chrono_literals;

// Custom node class
class CloudAccumulator : public rclcpp::Node
{
public:
    CloudAccumulator() : Node("cloud_accumulator")
    {   
        RCLCPP_INFO(this->get_logger(), "Cloud accumulator node started.");
        // Configure QoS to ensure data transmission. You need to confirm which QoS slam-toolbox is using.
        rclcpp::QoS qos = rclcpp::SensorDataQoS();
        qos.reliable();

        // Create a subscriber and subscribe to the `/utlidar/cloud_deskewed` topic for the robot dog.
        sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/utlidar/cloud_deskewed",
            rclcpp::SensorDataQoS(),
            std::bind(&CloudAccumulator::cloud_callback, this, std::placeholders::_1)
        );

        // Create a publisher and publish the robot dog's topic `/utlidar/cloud_accumulated`.
        pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/utlidar/cloud_accumulated",
            qos
        );

        // Create a timer to publish data periodically.
        timer_ = this->create_wall_timer(
            25ms,
            std::bind(&CloudAccumulator::timer_callback, this)
        );
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::vector<sensor_msgs::msg::PointCloud2::ConstSharedPtr> clouds_; // Create a container to store the point cloud (note that the point cloud data is read-only and cannot be modified).
    sensor_msgs::msg::PointCloud2 accumulated_cloud_;                   // Create cumulative point cloud data

    const size_t max_clouds_ = 5;  // Stores up to 25 frames of point cloud data
    const float min_height_ = 0.2f; // Minimum height threshold
    const float max_height_ = 0.8f; // Maximum height threshold

    // Callback function (note that the parameter uses the ConstSharedPtr type to prevent point cloud information from being modified)
    void cloud_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud_msg)
    {
        clouds_.push_back(cloud_msg);

        // If the number of point clouds exceeds max_clouds_, delete the oldest point cloud data.
        if(clouds_.size() > max_clouds_)
        {
            clouds_.erase(clouds_.begin());
        }

        // If no point cloud data is available, return [the value].
        if(clouds_.empty())
        {
            return;
        }

        // Merge all point clouds
        auto merged_cloud = merged_clouds();

        // High-level filtration
        auto filtered_cloud = filter_cloud(*merged_cloud);

        // Update the accumulated point cloud
        accumulated_cloud_ = filtered_cloud;
        accumulated_cloud_.header.frame_id = "odom"; // Set coordinate system
    }

    // Function to merge multiple point clouds
    std::shared_ptr<sensor_msgs::msg::PointCloud2> merged_clouds()
    {   
        // std::shared_ptr<sensor_msgs::msg::PointCloud2>
        auto merged_cloud = std::make_shared<sensor_msgs::msg::PointCloud2>();

        *merged_cloud = *clouds_[0]; // Initialize the merged point cloud using the first point cloud.  // *merged_cloud = *clouds_.front();

        // Traverse the remaining point cloud and merge them one by one into the base point cloud.
        for (size_t i = 1; i < clouds_.size(); i++)
        {
            // 1. Accumulate total points: The number of points in the merged point cloud = the number of points already existing + the number of points in the current point cloud.
            merged_cloud->width += clouds_[i]->width;

            // 2. Accumulate the length of each row in bytes: The length of the merged point cloud in bytes = the length of the existing point cloud in bytes + the length of the current point cloud in bytes.
            merged_cloud->row_step += clouds_[i]->row_step;

            // 3. Concatenate the original data: Append the current point cloud's data array to the end of the point cloud's data array.
            merged_cloud->data.insert(merged_cloud->data.end(),
                    clouds_[i]->data.begin(), 
                    clouds_[i]->data.end()
                );
        }
        return merged_cloud;
    }

    // High-level filtration
    sensor_msgs::msg::PointCloud2 filter_cloud(const sensor_msgs::msg::PointCloud2 &cloud)
    {
        // sensor_msgs::msg::PointCloud2
        sensor_msgs::msg::PointCloud2 filtered_cloud;
        // Copy header information and attributes
        filtered_cloud.header = cloud.header;
        filtered_cloud.height = 1;              // The height of an unordered point cloud is 1.
        filtered_cloud.fields = cloud.fields;   // Field definitions (offsets and types of fields such as x/y/z, which are completely reused)
        filtered_cloud.is_bigendian = cloud.is_bigendian;
        filtered_cloud.point_step = cloud.point_step;
        filtered_cloud.row_step = 0;
        filtered_cloud.is_dense = false; // Whether to include invalid points (false = allows invalid points, more general).

        // Iterate through all the points. Filter out points with suitable height.
        for (size_t i = 0; i < cloud.width * cloud.height; i++)
        {
            // Extraction height
            float z;
            memcpy(&z, &cloud.data[i * cloud.point_step + cloud.fields[2].offset], sizeof(float));

            // Only keep points whose height is in the range of min_height_ to max_height_.
            if(z >= min_height_ && z <= max_height_)
            {
                // Copy the data of the entire point to the filtered point cloud.
                filtered_cloud.data.insert(filtered_cloud.data.end(),              // Insert at the end of the filtered data
                                           &cloud.data[i * cloud.point_step],      // The starting address of the current byte block
                                           &cloud.data[(i + 1) * cloud.point_step] // The "end address of the byte block" at the current point (the starting address of the next point)
                );
                filtered_cloud.row_step += cloud.point_step;
                filtered_cloud.width++;
            }
        }
        return filtered_cloud;
    }

    void timer_callback()
    {
        if(!accumulated_cloud_.data.empty())
        {
            // accumulated_cloud_.header.stamp = this->now();
            accumulated_cloud_.header.stamp = this->get_clock()->now();
            pub_->publish(accumulated_cloud_);
        }
    }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CloudAccumulator>());
    rclcpp::shutdown();
    return 0;
}