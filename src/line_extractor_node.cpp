#include <geometry_msgs/Point.h>
#include <math.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/Marker.h>
#include "line_extractor/line_extractor.h"

using namespace sensor_msgs;

ros::Publisher *pub_;
void laserCallback(const sensor_msgs::LaserScanConstPtr& scan);
void lineExtractor(const std::vector<Point> data, std::vector<Line>& real_line);

void laserCallback(const sensor_msgs::LaserScanConstPtr& scan) {
  size_t range_size = scan->ranges.size();
  double theta(0);
  Point p;
  std::vector<Point> point_cloud;
  for (size_t i = 0; i < range_size; i++) {
    if (scan->ranges[i] < scan->range_min || scan->ranges[i] > scan->range_max)
      continue;

    theta = scan->angle_increment * i - scan->angle_min;
    p.x = scan->ranges[i] * cos(theta);
    p.y = scan->ranges[i] * sin(theta);
    point_cloud.push_back(p);
  }

  std::vector<Line> line;
  // kernel function
  lineExtractor(point_cloud, line);

  using namespace visualization_msgs;

  visualization_msgs::Marker l_marker;
  l_marker.action = Marker::ADD;
  l_marker.color.a = 1.0f;
  l_marker.color.r = 1.0f;
  l_marker.header.frame_id = "base_scan";
  l_marker.header.stamp = ros::Time::now();
  l_marker.id = 1;
  l_marker.ns = "l_marker";
  l_marker.scale.x = 0.1;
  l_marker.scale.y = 0.1;
  l_marker.scale.z = 0.1;
  l_marker.type = Marker::LINE_LIST;

  for (Line& l : line) {
    geometry_msgs::Point p;
    p.x = point_cloud.at(l.start_index).x;
    p.y = point_cloud.at(l.start_index).y;
    l_marker.points.push_back(p);

    p.x = point_cloud.at(l.end_index).x;
    p.y = point_cloud.at(l.end_index).y;
    l_marker.points.push_back(p);
  }

  pub_->publish(l_marker);
}

void publishLine(geometry_msgs::Point p, geometry_msgs::Point q)
{
  using namespace visualization_msgs;
  visualization_msgs::Marker l_marker;
  l_marker.action = Marker::ADD;
  l_marker.color.a = 1.0f;
  l_marker.color.r = 1.0f;
  l_marker.header.frame_id = "base_scan";
  l_marker.header.stamp = ros::Time::now();
  l_marker.id = 1;
  l_marker.ns = "l_marker";
  l_marker.scale.x = 0.05;
  l_marker.scale.y = 0.05;
  l_marker.scale.z = 0.05;
  l_marker.type = Marker::LINE_LIST;
  l_marker.points.push_back(p);
  l_marker.points.push_back(q);
  pub_->publish(l_marker);
}

void publishMarker(geometry_msgs::Point p)
{
  using namespace visualization_msgs;
  visualization_msgs::Marker l_marker;
  l_marker.action = Marker::ADD;
  l_marker.color.a = 1.0f;
  l_marker.color.r = 1.0f;
  l_marker.header.frame_id = "base_scan";
  l_marker.header.stamp = ros::Time::now();
  l_marker.id = 1;
  l_marker.ns = "l_marker";
  l_marker.scale.x = 0.05;
  l_marker.scale.y = 0.05;
  l_marker.scale.z = 0.05;
  l_marker.type = Marker::CUBE;
  l_marker.pose.position.x = p.x;
  l_marker.pose.position.y = p.y;
  l_marker.pose.orientation.w = 1.0f;
  pub_->publish(l_marker);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "line_extractor_node");
  ros::NodeHandle nh;
  pub_ = new ros::Publisher(nh.advertise<visualization_msgs::Marker>("/line", 1));
  ros::Subscriber laser_sub =
      nh.subscribe<sensor_msgs::LaserScan>("/scan", 1, laserCallback);
  ros::spin();
  return 0;
}
