#include "geometry_msgs/msg/pose.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <CSE180_FinalProject/navigation.hpp>
#include <cmath>
#include <iostream>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <vector>

class RobotController
{
public:
  RobotController(int argc, char **argv)
  {
    rclcpp::init(argc, argv);
    node_ = std::make_shared<rclcpp::Node>("robot_controller");
    navigator_ = std::make_shared<Navigator>(true, false); 

    // declare tf2 listener
    auto clock = node_->get_clock();
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(clock);
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    laser_scan_subscription_ = node_->create_subscription<sensor_msgs::msg::LaserScan>("/scan", 10, std::bind(&RobotController::laserScanCallback, this, std::placeholders::_1));

    run();
  }

  ~RobotController()
  {
    rclcpp::shutdown(); 
  }

private:
  struct Wall
  {
    float x1, y1, x2, y2;

    float distanceToPoint(float x, float y) const
    {
      
    }
  };

  // calculate the position of the contact 
  std::pair<float, float> calculateContactPoint(float range, size_t index)
  {
    float angle = laser_scan_.angle_min + index * laser_scan_.angle_increment;
    float x = range * std::cos(angle);
    float y = range * std::sin(angle);
    return std::make_pair(x, y);
  }

  // set pillar point
  bool isPointInKnownPillar(float x, float y)
  {
    std::vector<std::pair<float, float>> known_pillar_centers = {
        {-1.1, -1.1},
        {-1.1, 0},
        {-1.1, 1.1},
        {0, -1.1},
        {0, 0},
        {0, 1.1},
        {1.1, -1.1},
        {1.1, 0},
        {1.1, 1.1},
    };
    //set wall points for compute against htreshold 
    std::vector<Wall> walls = {
        {2.275f, 0.0f, 2.57f, 0.53f},
        {2.57f, 0.53f, 1.75f, 2.0f},
        {1.75f, 2.0f, 1.3625f, 2.0f},
        {1.3625f, 2.0f, 1.05f, 2.5375f},
        {1.05f, 2.5375f, -1.05f, 2.5375f},
        {-1.05f, 2.5375f, -1.3625f, 2.0f},
        {-1.3625f, 2.0f, -1.75f, 2.0f},
        {-1.75f, 2.0f, -2.9f, 0.0f},
        {-2.9f, 0.0f, -1.75f, -2.0f},
        {-1.75f, -2.0f, -1.3625f, -2.0f},
        {-1.3625f, -2.0f, -1.05f, -2.5375f},
        {-1.05f, -2.5375f, 1.05f, -2.5375f},
        {1.05f, -2.5375f, 1.3625f, -2.0f},
        {1.3625f, -2.0f, 1.75f, -2.0f},
        {1.75f, -2.0f, 2.57f, -0.53f},
        {2.57f, -0.53f, 2.275f, 0.0f},
    };

    float pillar_radius = 0.08;
    float threshold = 0.15;

    for (const auto &center : known_pillar_centers)
    {
      float distance = std::hypot(x - center.first, y - center.second);
      if (distance <= (pillar_radius + threshold))
      {
        return true;
      }
    }

    return false;
  }

  bool isPointInWall(float x, float y)
  {
    float wall_threshold = 0.05;

    for (const auto &wall : walls)
    {
      float denominator = std::sqrt((wall.y2 - wall.y1) * (wall.y2 - wall.y1) + (wall.x2 - wall.x1) * (wall.x2 - wall.x1));
      float numerator = std::abs((wall.y2 - wall.y1) * x - (wall.x2 - wall.x1) * y + wall.x2 * wall.y1 - wall.y2 * wall.x1);

      float distance = numerator / denominator;

      if (distance <= wall_threshold)
      {
        return true;
      }
    }

    return false;
  }

  void laserScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    laser_scan_ = *msg;
    std::vector<std::pair<float, float>> unknown_points;

    for (size_t i = 0; i < laser_scan_.ranges.size(); ++i)
    {
      float range = laser_scan_.ranges[i];
      if (range == std::numeric_limits<float>::infinity()) // ignore inf values
      {
        continue;
      }
      auto contact_point = calculateContactPoint(range, i);

      if (!isPointInKnownPillar(contact_point.first, contact_point.second) &&
          !isPointInWall(contact_point.first, contact_point.second))
      {
        unknown_points.push_back(contact_point);
      }
    }

    // compute clustering on the unknown points
    float cluster_distance_threshold = 0.2;
    std::vector<std::vector<std::pair<float, float>>> clusters;
    for (const auto &point : unknown_points)
    {
      bool found_cluster = false;
      for (auto &cluster : clusters)
      {
        for (const auto &cluster_point : cluster)
        {
          float dx = point.first - cluster_point.first;
          float dy = point.second - cluster_point.second;
          float distance = std::sqrt(dx * dx + dy * dy);

          if (distance < cluster_distance_threshold)
          {
            cluster.push_back(point);
            found_cluster = true;
            break;
          }
        }
        if (found_cluster)
        {
          break;
        }
      }
      if (!found_cluster)
      {
        clusters.push_back({point});
      }
    }

    // print the clusters
    for (const auto &cluster : clusters)
    {
      std::cout << "Cluster:\n";
      for (const auto &point : cluster)
      {
        std::cout << "(" << point.first << ", " << point.second << ")\n";
      }
    }
  }

  geometry_msgs::msg::PoseStamped getCurrentPose()
  {
    geometry_msgs::msg::TransformStamped transform_stamped;
    geometry_msgs::msg::PoseStamped current_pose;

    try
    {
      transform_stamped = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
      current_pose.header = transform_stamped.header;
      current_pose.pose.position.x = transform_stamped.transform.translation.x;
      current_pose.pose.position.y = transform_stamped.transform.translation.y;
      current_pose.pose.orientation = transform_stamped.transform.rotation;
    }
    catch (tf2::TransformException &ex)
    {
      RCLCPP_ERROR(node_->get_logger(), "Transform error: %s", ex.what());
    }

    return current_pose;
  }

  void run()
  {
    // initialize the pose of the robot
    geometry_msgs::msg::Pose::SharedPtr init =
        std::make_shared<geometry_msgs::msg::Pose>();
    init->position.x = -2;
    init->position.y = -0.5;
    init->orientation.w = 1;
    navigator_->SetInitialPose(init);

    //wait
    navigator_->WaitUntilNav2Active();

    std::vector<float> x = {-1.1f, 0.0f, 1.1f, 1.5f, 1.5,
                            1.5f, 1.1f, 0.0f, -1.1f, -2,
                            -1.1f, 0.0f, 1.1f, -1.5f, 1.5f,
                            1.1f, 0.0f, -1.1f, -1.1f, 0.0f,
                            1.1f},
                       y = {0.55f, 0.55f, 0.55f, 0.55f, 0.0f,
                            -0.55f, -0.55f, -0.55f, -0.55f, -0.5f,
                            -1.5f, -1.5f, -1.5f, -1.5f, 1.5f,
                            1.5f, 1.5f, 1.5f, 0.55f, 0.55f,
                            0.55f},
                       w = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
                            1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
                            1};

    navigator_->Spin();
    //threshold 
    const float clear_distance_threshold = 1.0; 

    while (!navigator_->IsTaskComplete())
    {
    } // busy waiting for task to be completed
    geometry_msgs::msg::Pose::SharedPtr goal_pos =
        std::make_shared<geometry_msgs::msg::Pose>();

    for (size_t i = 0; i < x.size(); i++)
    {
      goal_pos->position.x = x[i];
      goal_pos->position.y = y[i];
      goal_pos->orientation.w = w[i];
      // move to new pose
      navigator_->GoToPose(goal_pos);
      while (!navigator_->IsTaskComplete())
      {
      }

      // Print current position
      auto current_pose = getCurrentPose();
      std::cout << "Current Position: (" << current_pose.pose.position.x << ", "
                << current_pose.pose.position.y << ")\n";

      // spin 360 degrees after reaching waypoint
      float angle_degrees = 180.0;
      float angle_radians = angle_degrees * (M_PI / 180.0);
      navigator_->Spin(angle_radians);
      while (!navigator_->IsTaskComplete())
      {
        rclcpp::spin_some(node_); 
        // compare the laser scan
        for (size_t j = 0; j < laser_scan_.ranges.size(); ++j)
        {
          float range = laser_scan_.ranges[j];

          // check if the range reading exceeds the threshold
          if (range > clear_distance_threshold)
          {
            // clculate non-obstacle 
            float angle =
                laser_scan_.angle_min + j * laser_scan_.angle_increment;
            float x_robot = range * std::cos(angle);
            float y_robot = range * std::sin(angle);

            // print current position
            auto current_pose = getCurrentPose();

            // apply quaterion
            tf2::Quaternion q(current_pose.pose.orientation.x,
                              current_pose.pose.orientation.y,
                              current_pose.pose.orientation.z,
                              current_pose.pose.orientation.w);

            tf2Scalar yaw, pitch, roll;
            tf2::Matrix3x3(q).getEulerYPR(yaw, pitch, roll);

            // Transform the "non-obstacle" position to the global "map" frame
            float x_map = current_pose.pose.position.x +
                          (x_robot * std::cos(yaw) - y_robot * std::sin(yaw));
            float y_map = current_pose.pose.position.y +
                          (x_robot * std::sin(yaw) + y_robot * std::cos(yaw));

            std::cout << "Non-obstacle found at map coordinates (" << x_map
                      << ", " << y_map << ")" << std::endl;
          }

          std::cout << "Range at " << j << ": " << laser_scan_.ranges[j]
                    << std::endl;
        }
      }

      // print final position
      auto final_pose = getCurrentPose();
      std::cout << "Final Position: (" << final_pose.pose.position.x << ", "
                << final_pose.pose.position.y << ")\n";

      navigator_->Backup();
      while (!navigator_->IsTaskComplete())
      {
      }
    }
  }

  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<Navigator> navigator_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_subscription_;
  sensor_msgs::msg::LaserScan laser_scan_;
  std::vector<Wall> walls;
};

int main(int argc, char **argv)
{
  RobotController robotController(argc, argv);
  return 0;
}