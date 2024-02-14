#include "rclcpp/rclcpp.hpp"
#include <string>
#include <vector>
#include <utility>
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include <limits>
#include <memory>
using std::placeholders::_1;
using namespace std;

class ReactiveFollowGap : public rclcpp::Node {
// Implement Reactive Follow Gap on the car
// This is just a template, you are free to implement your own node!

public:
    ReactiveFollowGap() : Node("reactive_node")
    {
        // scan sub
        scan_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(lidarscan_topic, 10, std::bind(&ReactiveFollowGap::lidar_callback, this, _1));

        // drive pub
        drive_publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(drive_topic, 10);
    }

private:
    string lidarscan_topic = "/scan";
    string drive_topic = "/drive";
    double rejection_thresh = 5.0;
    double filter_window = 5.0;
    int bubble_radius = 10;
    double velocity = 0.5;
    double disparity_thresh = 0.4;
    double disparity_window = 10.0;
    double gap_thresh = 0.7;

    /// ROS subscribers and publishers
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;

    pair<int,float*> preprocess_lidar(const float* ranges, const int ranges_size)
    {
        // Preprocess the LiDAR scan array. Expert implementation includes:
        // 1.Setting each value to the mean over some window
        // 2.Rejecting high values (eg. > 3m)
        // const int ranges_size = sizeof(ranges) / sizeof(ranges[0]);

        float closest_val = numeric_limits<float>::max();
        int closest_idx;

        float* smoothed_ranges = new float[ranges_size];

        for (int i=0; i<ranges_size; i++){
            RCLCPP_INFO(this->get_logger(), "range: % f", ranges[i]);
            // set large values to rejection thresh
            if(ranges[i] > rejection_thresh){
                smoothed_ranges[i] = rejection_thresh;
            }

            // handle nan values -- set to 0
            else if (isnan(ranges[i])){
                smoothed_ranges[i] = 0.0;
            }

            // moving average filter to smooth ranges
            else{
                double sum = 0;
                double total = 0;

                for (int j=i-filter_window; j<=i+filter_window; j++){
                    // check inf, nan, and oob
                    if(!isnan(ranges[j]) && j>=0 && j<ranges_size){
                        if (ranges[j] > rejection_thresh){
                            sum += rejection_thresh;
                            total += 1;
                        }
                        else{
                            sum += ranges[j];
                            total += 1;
                        }
                    }
                }
                smoothed_ranges[i] = sum / total;
                RCLCPP_INFO(this->get_logger(), "smoothed_range: % f", smoothed_ranges[i]);

                // keep track of closest point
                if (smoothed_ranges[i] < closest_val){
                    closest_val = smoothed_ranges[i];
                    closest_idx = i;
                }
            }
        }

        // RCLCPP_INFO(this->get_logger(), "closest_idx: %i", closest_idx);
        return make_pair(closest_idx, smoothed_ranges);
    }


    void find_max_gap(float* ranges, int* indices, const int ranges_size)
    {
        // Return the start index & end index of the max gap in free_space_ranges
        int start = indices[0];
        int end = indices[1];
        int max_gap_size = 0;

        while(end < ranges_size){
            if (ranges[end]==0 || end==ranges_size-1){
                if (end-start > max_gap_size){
                    max_gap_size = end-start;
                    indices[0] = start;
                    indices[1] = end-1;
                }
                end++;
                start = end;
            }
            end++;
        }
    }

    void find_deep_gap(float* ranges, int* indices, const int ranges_size)
    {
        // Return the start index & end index of the max gap in free_space_ranges
        int start = indices[0];
        int end = indices[1];
        int max_gap_size = 0;

        while(end < ranges_size){
            if (ranges[end]>gap_thresh || end==ranges_size-1){
                if (end-start > max_gap_size){
                    max_gap_size = end-start;
                    indices[0] = start;
                    indices[1] = end-1;
                }
                end++;
                start = end;
            }
            end++;
        }
    }


    int find_best_point(float* ranges, int* indices)
    {   
        // Start_i & end_i are start and end indicies of max-gap range, respectively
        // Return index of best point in ranges
	    // Naive: Choose the furthest point within ranges and go there
        double best_point = 0;
        int best_idx = 0;

        for (int i=indices[0]; i<indices[1]; i++){
            if (ranges[i] > best_point){
                best_point = ranges[i];
                best_idx = i;
            }
        }
        return best_idx;
    }

    void disparity_extender(float* ranges, const int ranges_size){

        auto prev = ranges[0];
        int i = 1;

        // iterate through negative angles
        while(i<ranges_size && ranges[i]<0){
            // check for disparity (low to high jump)
            if(ranges[i]-prev > disparity_thresh){
                // extend the smaller value forwards by disparity window
                for (int j=i; j<i+disparity_window; j++){
                    ranges[j] = prev;
                }
                i += disparity_window;
            }
            else{
                prev = ranges[i];
                i+=1;
            }
        }

        // iterate through positive angles
        while(i<ranges_size){
            // check for disparity (high to low jump)
            if(prev-ranges[i] > disparity_thresh){
                // extend the smaller value backwards by disparity window
                for (int j=i-1; j>i-disparity_window; j--){
                    ranges[j] = ranges[i];
                }
            }
            prev = ranges[i]; 
            i+=1;
        }
    }


    void lidar_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg) 
    {
        // Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message

        // preprocess lidar range data 
        auto preprocess = preprocess_lidar(scan_msg->ranges.data(), scan_msg->ranges.size());

        // Find closest point to LiDAR
        auto closest_idx = preprocess.first;
        auto ranges = preprocess.second;
        disparity_extender(ranges, scan_msg->ranges.size());

        // Eliminate all points inside 'bubble' (set them to zero)
        for (int i = max(closest_idx-bubble_radius, 0); i < min(closest_idx+bubble_radius, static_cast<int>(scan_msg->ranges.size())); ++i) {
            ranges[i] = 0;
        }

        // Find max length gap
        int indices[] = {0,1};
        find_max_gap(ranges, indices, scan_msg->ranges.size());
        // find_deep_gap(ranges, indices, scan_msg->ranges.size());
        RCLCPP_INFO(this->get_logger(), "max gap size: %d", indices[1]-indices[0]);

        // Find the best point in the gap
        int best_point = find_best_point(ranges, indices); // for furthest point
        // int best_point = (indices[0]+indices[1])/2; // for midpoint
        RCLCPP_INFO(this->get_logger(), "best point index: %d", best_point);

        // Publish Drive message
        double steering_angle = scan_msg->angle_min + best_point*scan_msg->angle_increment;
        auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
        drive_msg.drive.speed = velocity;
        drive_msg.drive.steering_angle = steering_angle;
        RCLCPP_INFO(this->get_logger(), "steering angle: %f", steering_angle);
        drive_publisher_->publish(drive_msg);

        delete[] ranges;
        ranges = nullptr;
    }
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ReactiveFollowGap>());
    rclcpp::shutdown();
    return 0;
}