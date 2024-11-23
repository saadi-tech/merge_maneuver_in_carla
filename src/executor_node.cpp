#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "rosbag2_cpp/writer.hpp"
#include "rosbag2_storage/storage_options.hpp"
#include <chrono>
#include <string>

class EgoVehicleControlNode : public rclcpp::Node
{
public:
    EgoVehicleControlNode()
    : Node("ego_vehicle_control_node"),
      start_time_(this->now())
    {   

        // Declare the parameter with a default value
        this->declare_parameter<std::string>("dump_path", "/home/saad/pro_ws/src/merge_simulation/data/latest_data_dump");
        std::string dump_path;
        this->get_parameter("dump_path", dump_path);


        writer_ = std::make_unique<rosbag2_cpp::Writer>();

        // Set up the storage options for MCAP
        rosbag2_storage::StorageOptions storage_options;
        storage_options.uri = dump_path;       // Directory name for the MCAP file
        storage_options.storage_id = "mcap";    // Specify the MCAP storage backend
        // Open the writer with storage options
        writer_->open(storage_options, rosbag2_cpp::ConverterOptions());

        // Create publishers for the Ego and Actor vehicle poses
        ego_speed_publisher_ = this->create_publisher<geometry_msgs::msg::Pose>("/carla/ego_vehicle/control/set_transform", 10);
        actor_speed_publisher_ = this->create_publisher<geometry_msgs::msg::Pose>("/carla/actor_1/control/set_transform", 10);

        // Initialize time step and speeds
        delta_t_ms_ = 100; // to ensure update at 10 Hz
        max_sim_time_ = 20; // seconds (Max simulation time. After this time, the node will stop)
        ego_speed_ = 3.0;  // Linear speed in m/s
        actor_speed_ = 4.0;


        merge_time_ = 3.0; //               seconds - Merge maneuver starts at t = 3 seconds
        merge_speed_ = 0.6; // m/s          lateral speed of merging maneuver
        merge_distance_ = 3.0; // meters    distance to merge (Distance between lanes)
        merge_duration_ = merge_distance_ / merge_speed_; // seconds

        // Set initial position of EGO (30, 5, 0)
        ego_pos_ = {30.0, 5.0, 0.0};  // Initial x, y, z position
        ego_vel_ = {ego_speed_, 0.0, 0.0};  // Linear velocity in x, y, z
        
        // Set initial position of Actor (30, 8, 0)
        actor_pos_ = {30.0, 8.0, 0.0};  // Initial x, y, z position
        actor_vel_ = {actor_speed_, 0.0, 0.0};  // Linear velocity in x, y, z

        // Subscribers
        imu_subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/carla/ego_vehicle/imu", 10, std::bind(&EgoVehicleControlNode::imu_callback, this, std::placeholders::_1)
        );
        
        lidar_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/carla/ego_vehicle/lidar", 10, std::bind(&EgoVehicleControlNode::lidar_callback, this, std::placeholders::_1)
        );

        gnss_subscriber_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
            "/carla/ego_vehicle/gnss", 10, std::bind(&EgoVehicleControlNode::gnss_callback, this, std::placeholders::_1)
        );

        image_subscriber_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/carla/ego_vehicle/rgb_front/image", 10, std::bind(&EgoVehicleControlNode::image_callback, this, std::placeholders::_1)
        );

        // Timer to call the callback function at 10 Hz
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(delta_t_ms_), std::bind(&EgoVehicleControlNode::publish_transform, this));

        RCLCPP_INFO(this->get_logger(), "Ego vehicle control node has started");
    }

private:
    void publish_transform()
    {   
        rclcpp::Time current_time = this->now();
        rclcpp::Duration duration = current_time - start_time_;

        // Calculate the elapsed time
        float elapsed_time = duration.seconds();

        // Check if elapsed time is greater than max_sim_time
        if (elapsed_time > max_sim_time_)
        {
            RCLCPP_INFO(this->get_logger(), "**** Simulation completed!!! Shutting down the node****");
            rclcpp::shutdown();
            return;
        }

        // Implement the merge maneuver at t = 3 seconds
        if (elapsed_time > merge_time_ && elapsed_time < merge_time_ + merge_duration_)
        {   
            actor_vel_[1] = -merge_speed_; // Move the actor vehicle to the right
        }
        else
        {
            actor_vel_[1] = 0.0; //else keep straight movement ONLY
        }


        // Set actor-speed = ego-speed after completion of merge maneuver
        if (elapsed_time >= merge_time_ + merge_duration_)
        {
            actor_vel_[0] = ego_vel_[0];
        }


        // Update the position of the EGO and Actor vehicles
        for (int i = 0; i < 3; ++i)
        {
            ego_pos_[i] += ego_vel_[i] * delta_t_ms_ / 1000.0;
            actor_pos_[i] += actor_vel_[i] * delta_t_ms_ / 1000.0;
        }

        // Create and publish the EGO vehicle pose
        geometry_msgs::msg::Pose ego_pose;
        ego_pose.position.x = ego_pos_[0];
        ego_pose.position.y = ego_pos_[1];
        ego_pose.position.z = ego_pos_[2];
        ego_pose.orientation.x = 0.0;
        ego_pose.orientation.y = 0.0;
        ego_pose.orientation.z = 0.0;
        ego_pose.orientation.w = 1.0; // No rotation (unit quaternion)

        ego_speed_publisher_->publish(ego_pose);


        // Create and publish the Actor vehicle pose
        geometry_msgs::msg::Pose actor_pose;
        actor_pose.position.x = actor_pos_[0];
        actor_pose.position.y = actor_pos_[1];
        actor_pose.position.z = actor_pos_[2];
        actor_pose.orientation.x = 0.0;
        actor_pose.orientation.y = 0.0;
        actor_pose.orientation.z = 0.0;
        actor_pose.orientation.w = 1.0; // No rotation (unit quaternion)

        actor_speed_publisher_->publish(actor_pose);
    }

    // Member variables
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr ego_speed_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr actor_speed_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Time start_time_;
    int delta_t_ms_;
    int max_sim_time_;
    double ego_speed_;
    double actor_speed_;
    double merge_time_;
    double merge_speed_;
    double merge_distance_;
    double merge_duration_;

    std::vector<double> ego_pos_;
    std::vector<double> ego_vel_;
    std::vector<double> actor_pos_;
    std::vector<double> actor_vel_;


    // Callbacks
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received IMU data: orientation=(%f, %f, %f, %f), angular_velocity=(%f, %f, %f), linear_acceleration=(%f, %f, %f)",
                    msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w,
                    msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z,
                    msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);

        // Serialize the incoming message and get the current timestamp.
        auto serialized_msg = std::make_shared<rclcpp::SerializedMessage>();
        rclcpp::Serialization<sensor_msgs::msg::Imu> serializer;
        serializer.serialize_message(msg.get(), serialized_msg.get());

        rclcpp::Time time_stamp = this->now();

        // Write the serialized message to the bag file.
        writer_->write(serialized_msg, "/carla/ego_vehicle/imu", "sensor_msgs/msg/Imu", time_stamp);
    }

    void lidar_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received LIDAR data: width=%d, height=%d, point step=%d",
                    msg->width, msg->height, msg->point_step);

        // Serialize the incoming message and get the current timestamp.
        auto serialized_msg = std::make_shared<rclcpp::SerializedMessage>();
        rclcpp::Serialization<sensor_msgs::msg::PointCloud2> serializer;
        serializer.serialize_message(msg.get(), serialized_msg.get());

        rclcpp::Time time_stamp = this->now();

        // Write the serialized message to the bag file.
        writer_->write(serialized_msg, "/carla/ego_vehicle/lidar", "sensor_msgs/msg/PointCloud2", time_stamp);
    }

    void gnss_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received GNSS data: latitude=%f, longitude=%f, altitude=%f",
                    msg->latitude, msg->longitude, msg->altitude);

        // Serialize the incoming message and get the current timestamp.
        auto serialized_msg = std::make_shared<rclcpp::SerializedMessage>();
        rclcpp::Serialization<sensor_msgs::msg::NavSatFix> serializer;
        serializer.serialize_message(msg.get(), serialized_msg.get());

        rclcpp::Time time_stamp = this->now();

        // Write the serialized message to the bag file.
        writer_->write(serialized_msg, "/carla/ego_vehicle/gnss", "sensor_msgs/msg/NavSatFix", time_stamp);
    }

    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received image data: width=%d, height=%d, encoding=%s",
                    msg->width, msg->height, msg->encoding.c_str());

        // Serialize the incoming message and get the current timestamp.
        auto serialized_msg = std::make_shared<rclcpp::SerializedMessage>();
        rclcpp::Serialization<sensor_msgs::msg::Image> serializer;
        serializer.serialize_message(msg.get(), serialized_msg.get());

        rclcpp::Time time_stamp = this->now();

        // Write the serialized message to the bag file.
        writer_->write(serialized_msg, "/carla/ego_vehicle/rgb_front/image", "sensor_msgs/msg/Image", time_stamp);
    }

    // Subscribers
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gnss_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscriber_;

    std::unique_ptr<rosbag2_cpp::Writer> writer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    // Create the node
    rclcpp::spin(std::make_shared<EgoVehicleControlNode>());

    rclcpp::shutdown();
    return 0;
}
