#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <std_msgs/msg/header.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "adaptive_A_star/astar_planner.hpp"
#include "adaptive_A_star/types.hpp"
#include <optional>
#include <memory>

namespace adaptive_a_star {

class AdaptiveAStarNode : public rclcpp::Node {
public:
    AdaptiveAStarNode() : Node("adaptive_a_star_node") {
        // パラメータ設定
        this->declare_parameter("wall_clearance_distance", 0.3);
        this->declare_parameter("global_frame", "map");
        this->declare_parameter("base_frame", "base_footprint");
        
        // TF2関連初期化
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        
        // パブリッシャー初期化
        path_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("planned_path", 10);
        path_viz_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("astar_path_viz", 10);
        
        // サブスクライバー初期化
        auto map_qos = rclcpp::QoS(1).reliable().transient_local();
        map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "map", map_qos, std::bind(&AdaptiveAStarNode::map_callback, this, std::placeholders::_1));
            
        // clicked_pointを受け取ってゴールとして設定
        clicked_point_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
            "clicked_point", 10, std::bind(&AdaptiveAStarNode::clicked_point_callback, this, std::placeholders::_1));
        
        RCLCPP_INFO(this->get_logger(), "Adaptive A* Node initialized");
    }

private:
    void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        try {
            occupancy_grid_ = *msg;
            
            // A*経路計画器を初期化
            tvvf_vo_c::Position origin(msg->info.origin.position.x, msg->info.origin.position.y);
            double wall_clearance = this->get_parameter("wall_clearance_distance").as_double();
            
            path_planner_ = std::make_unique<tvvf_vo_c::AStarPathPlanner>(
                *msg, msg->info.resolution, origin, wall_clearance);
            
            RCLCPP_INFO(this->get_logger(), "Map received: %dx%d, resolution: %.3f m/cell",
                       msg->info.width, msg->info.height, msg->info.resolution);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Map callback error: %s", e.what());
        }
    }
    
    void clicked_point_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
        try {
            goal_position_ = tvvf_vo_c::Position(msg->point.x, msg->point.y);
            RCLCPP_INFO(this->get_logger(), "Goal set from clicked_point: (%.2f, %.2f)", 
                       goal_position_->x, goal_position_->y);
            
            // ロボットの現在位置をTFから取得
            auto robot_pos = get_robot_position();
            if (robot_pos.has_value()) {
                start_position_ = robot_pos.value();
                RCLCPP_INFO(this->get_logger(), "Start position from TF: (%.2f, %.2f)", 
                           start_position_->x, start_position_->y);
            } else {
                // TF取得に失敗した場合は原点を使用
                start_position_ = tvvf_vo_c::Position(0.0, 0.0);
                RCLCPP_WARN(this->get_logger(), "Failed to get robot position from TF, using origin: (0.0, 0.0)");
            }
            
            plan_path();
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Clicked point callback error: %s", e.what());
        }
    }
    
    void plan_path() {
        if (!path_planner_ || !start_position_.has_value() || !goal_position_.has_value()) {
            RCLCPP_WARN(this->get_logger(), "Path planning: missing requirements");
            return;
        }
        
        try {
            auto start_time = std::chrono::high_resolution_clock::now();
            auto planned_path = path_planner_->plan_path(start_position_.value(), goal_position_.value());
            auto end_time = std::chrono::high_resolution_clock::now();
            
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
            
            if (planned_path.has_value()) {
                current_path_ = planned_path.value();
                RCLCPP_INFO(this->get_logger(), "A* Path Planning - Points: %zu, Time: %ld ms",
                           current_path_.size(), duration.count());
                           
                publish_path();
                publish_path_visualization();
            } else {
                RCLCPP_WARN(this->get_logger(), "A* Path planning failed after %ld ms", duration.count());
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Path planning error: %s", e.what());
        }
    }
    
    void publish_path() {
        if (current_path_.empty()) {
            return;
        }
        
        auto path_msg = geometry_msgs::msg::PoseArray();
        path_msg.header.frame_id = this->get_parameter("global_frame").as_string();
        path_msg.header.stamp = this->get_clock()->now();
        
        for (const auto& path_point : current_path_.points) {
            geometry_msgs::msg::Pose pose;
            pose.position.x = path_point.position.x;
            pose.position.y = path_point.position.y;
            pose.position.z = 0.0;
            pose.orientation.w = 1.0;
            path_msg.poses.push_back(pose);
        }
        
        path_pub_->publish(path_msg);
    }
    
    void publish_path_visualization() {
        if (current_path_.empty()) {
            return;
        }
        
        auto marker_array = visualization_msgs::msg::MarkerArray();
        std::string global_frame = this->get_parameter("global_frame").as_string();
        
        // 線分マーカー（経路全体）
        auto line_marker = visualization_msgs::msg::Marker();
        line_marker.header.frame_id = global_frame;
        line_marker.header.stamp = this->get_clock()->now();
        line_marker.id = 0;
        line_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        line_marker.action = visualization_msgs::msg::Marker::ADD;
        
        for (const auto& path_point : current_path_.points) {
            geometry_msgs::msg::Point point;
            point.x = path_point.position.x;
            point.y = path_point.position.y;
            point.z = 0.05;
            line_marker.points.push_back(point);
        }
        
        line_marker.scale.x = 0.03;
        line_marker.color.r = 0.0;
        line_marker.color.g = 0.0;
        line_marker.color.b = 1.0;
        line_marker.color.a = 0.8;
        line_marker.lifetime = rclcpp::Duration::from_seconds(0);
        
        marker_array.markers.push_back(line_marker);
        path_viz_pub_->publish(marker_array);
    }
    
    std::optional<tvvf_vo_c::Position> get_robot_position() {
        try {
            std::string base_frame = this->get_parameter("base_frame").as_string();
            std::string global_frame = this->get_parameter("global_frame").as_string();
            
            // TF取得
            geometry_msgs::msg::TransformStamped transform_stamped;
            transform_stamped = tf_buffer_->lookupTransform(
                global_frame, base_frame, tf2::TimePointZero, tf2::durationFromSec(0.1));
            
            // 位置の取得
            tvvf_vo_c::Position position(transform_stamped.transform.translation.x,
                                        transform_stamped.transform.translation.y);
            
            return position;
        } catch (const tf2::TransformException& ex) {
            RCLCPP_DEBUG(this->get_logger(), "TF lookup failed: %s", ex.what());
            return std::nullopt;
        }
    }
    
    // メンバ変数
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr path_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr path_viz_pub_;
    
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr clicked_point_sub_;
    
    nav_msgs::msg::OccupancyGrid occupancy_grid_;
    std::unique_ptr<tvvf_vo_c::AStarPathPlanner> path_planner_;
    
    // TF2関連
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    
    std::optional<tvvf_vo_c::Position> start_position_;
    std::optional<tvvf_vo_c::Position> goal_position_;
    tvvf_vo_c::Path current_path_;
};

} // namespace adaptive_a_star

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<adaptive_a_star::AdaptiveAStarNode>());
    rclcpp::shutdown();
    return 0;
}