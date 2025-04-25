#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <nav_msgs/Odometry.h>
#include <vector>
#include <tf2/LinearMath/Quaternion.h>
#include <cmath>  // 添加数学计算头文件

// 添加全局变量用于指标统计
struct NavigationMetrics {
    ros::Time total_start_time;
    ros::Time segment_start_time;
    double total_distance = 0.0;
    geometry_msgs::Point last_position;
    bool first_odom = true;
} metrics;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

struct NavGoal {
    float x;        // X coordinate in map frame
    float y;        // Y coordinate in map frame
    float yaw;      // Orientation in radians (0 = facing east)
};

// 添加里程计回调函数
void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    if (metrics.first_odom) {
        metrics.last_position = msg->pose.pose.position;
        metrics.first_odom = false;
    } else {
        // 计算两点间距离
        double dx = msg->pose.pose.position.x - metrics.last_position.x;
        double dy = msg->pose.pose.position.y - metrics.last_position.y;
        double distance = std::hypot(dx, dy);
        
        metrics.total_distance += distance;
        metrics.last_position = msg->pose.pose.position;
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "multi_goal_navigator");
    MoveBaseClient ac("move_base", true);

    // 初始化订阅者
    ros::NodeHandle nh;
    ros::Subscriber odom_sub = nh.subscribe("/odom", 10, odomCallback);

    // 记录总开始时间
    metrics.total_start_time = ros::Time::now();

    // Wait for action server
    if (!ac.waitForServer(ros::Duration(5.0))) {
        ROS_ERROR("Could not connect to move_base server");
        return 1;
    }

    // Define goals: {x, y, yaw_radians}
    std::vector<NavGoal> goals = {
        {0.8, 5.0, 0.0},     // Point 1: East
        {3.5, 8.0, M_PI/2},  // Point 2: North (修正航向)
        {7.5, 5.0, M_PI},    // Point 3: West 
        {3.5, 2.0, -M_PI/2}  // Point 4: South
    };

    for (const auto& goal_spec : goals) {
        // 记录分段开始时间
        metrics.segment_start_time = ros::Time::now();
        
        move_base_msgs::MoveBaseGoal goal;
        
        // Set position
        goal.target_pose.header.frame_id = "map";
        goal.target_pose.header.stamp = ros::Time::now();
        goal.target_pose.pose.position.x = goal_spec.x;
        goal.target_pose.pose.position.y = goal_spec.y;

        // Convert yaw to quaternion
        tf2::Quaternion q;
        q.setRPY(0, 0, goal_spec.yaw);  // Roll, Pitch, Yaw
        goal.target_pose.pose.orientation.x = q.x();
        goal.target_pose.pose.orientation.y = q.y();
        goal.target_pose.pose.orientation.z = q.z();
        goal.target_pose.pose.orientation.w = q.w();

        ROS_INFO("Sending goal: (%.1f, %.1f) @ %.2f rad", 
                goal_spec.x, goal_spec.y, goal_spec.yaw);
        ac.sendGoal(goal);

        // 改进等待循环以处理回调
        bool success = false;
        ros::Time wait_start = ros::Time::now();
        while ((ros::Time::now() - wait_start) < ros::Duration(40.0)) {
            if (ac.getState().isDone()) {
                success = true;
                break;
            }
            ros::spinOnce();  // 关键：处理回调更新位置
            ros::Duration(0.05).sleep();
        }

        if (!success) {
            ROS_ERROR("Timeout reaching goal (%.1f, %.1f)", goal_spec.x, goal_spec.y);
            return 1;
        }

        if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
            // 计算分段耗时
            ros::Duration segment_time = ros::Time::now() - metrics.segment_start_time;
            ROS_INFO("Reached goal! Segment time: %.1fs, Current total distance: %.2fm", 
                    segment_time.toSec(), metrics.total_distance);
            ros::Duration(2.0).sleep();
        } else {
            ROS_ERROR("Failed to reach goal (%.1f, %.1f)", goal_spec.x, goal_spec.y);
            return 1;
        }
    }

    // 最终统计输出
    ros::Duration total_time = ros::Time::now() - metrics.total_start_time;
    ROS_INFO("\n====== Final Statistics ======");
    ROS_INFO("Total navigation time: %.1f seconds", total_time.toSec());
    ROS_INFO("Total path length: %.2f meters", metrics.total_distance);
    ROS_INFO("Average speed: %.2f m/s", metrics.total_distance / total_time.toSec());

    return 0;
}