// Copyright 2024 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "autoware/path_generator/utils.hpp"

#include "autoware/trajectory/interpolator/linear.hpp"
#include "autoware/trajectory/utils/closest.hpp"
#include "autoware/trajectory/utils/crop.hpp"
#include "autoware/trajectory/utils/find_intervals.hpp"

#include <autoware/motion_utils/constants.hpp>
#include <autoware/motion_utils/resample/resample.hpp>
#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware/trajectory/forward.hpp>
#include <autoware/trajectory/path_point_with_lane_id.hpp>
#include <autoware/trajectory/utils/pretty_build.hpp>
#include <autoware_lanelet2_extension/utility/message_conversion.hpp>
#include <autoware_lanelet2_extension/utility/utilities.hpp>
#include <autoware_utils/geometry/geometry.hpp>
#include <autoware_utils/math/unit_conversion.hpp>

#include <autoware_internal_planning_msgs/msg/path_point_with_lane_id.hpp>

#include <lanelet2_core/Forward.h>
#include <lanelet2_core/geometry/Lanelet.h>

#include <algorithm>
#include <limits>
#include <optional>
#include <set>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace autoware::path_generator
{
namespace utils
{
namespace
{
const std::unordered_map<std::string, uint8_t> turn_signal_command_map = {
  {"left", TurnIndicatorsCommand::ENABLE_LEFT},
  {"right", TurnIndicatorsCommand::ENABLE_RIGHT},
  {"straight", TurnIndicatorsCommand::DISABLE}};

template <typename T>
bool exists(const std::vector<T> & vec, const T & item)
{
  return std::find(vec.begin(), vec.end(), item) != vec.end();
}

template <typename const_iterator>
std::vector<geometry_msgs::msg::Point> to_geometry_msgs_points(
  const const_iterator begin, const const_iterator end)
{
  std::vector<geometry_msgs::msg::Point> geometry_msgs_points{};
  geometry_msgs_points.reserve(std::distance(begin, end));
  std::transform(begin, end, std::back_inserter(geometry_msgs_points), [](const auto & point) {
    return lanelet::utils::conversion::toGeomMsgPt(point);
  });
  return geometry_msgs_points;
}
}  // namespace

std::optional<lanelet::ConstLanelet> get_previous_lanelet_within_route(
  const lanelet::ConstLanelet & lanelet, const PlannerData & planner_data)
{
  if (exists(planner_data.start_lanelets, lanelet)) {
    return std::nullopt;
  }

  const auto prev_lanelets = planner_data.routing_graph_ptr->previous(lanelet);
  if (prev_lanelets.empty()) {
    return std::nullopt;
  }

  const auto prev_lanelet_itr = std::find_if(
    prev_lanelets.cbegin(), prev_lanelets.cend(),
    [&](const lanelet::ConstLanelet & l) { return exists(planner_data.route_lanelets, l); });
  if (prev_lanelet_itr == prev_lanelets.cend()) {
    return std::nullopt;
  }
  return *prev_lanelet_itr;
}

std::optional<lanelet::ConstLanelet> get_next_lanelet_within_route(
  const lanelet::ConstLanelet & lanelet, const PlannerData & planner_data)
{
  if (planner_data.preferred_lanelets.empty()) {
    return std::nullopt;
  }

  if (exists(planner_data.goal_lanelets, lanelet)) {
    return std::nullopt;
  }

  const auto next_lanelets = planner_data.routing_graph_ptr->following(lanelet);
  if (
    next_lanelets.empty() ||
    next_lanelets.front().id() == planner_data.preferred_lanelets.front().id()) {
    return std::nullopt;
  }

  const auto next_lanelet_itr = std::find_if(
    next_lanelets.cbegin(), next_lanelets.cend(),
    [&](const lanelet::ConstLanelet & l) { return exists(planner_data.route_lanelets, l); });
  if (next_lanelet_itr == next_lanelets.cend()) {
    return std::nullopt;
  }
  return *next_lanelet_itr;
}

std::optional<PathRange<std::vector<geometry_msgs::msg::Point>>> get_path_bounds(
  const std::vector<PathPointWithLaneId> & path_points,
  const lanelet::LaneletSequence & lanelet_sequence)
{
  if (lanelet_sequence.empty()) {
    return std::nullopt;
  }

  const auto start_arc_lengths = get_arc_length_on_bounds(lanelet_sequence, path_points.front());
  const auto end_arc_lengths = get_arc_length_on_bounds(lanelet_sequence, path_points.back());

  if (!start_arc_lengths || !end_arc_lengths) {
    RCLCPP_ERROR(
      rclcpp::get_logger("path_generator").get_child("utils").get_child(__func__),
      "Failed to set path bounds");
    return std::nullopt;
  }

  return PathRange<std::vector<geometry_msgs::msg::Point>>{
    crop_line_string(
      lanelet_sequence.leftBound().basicLineString(), start_arc_lengths->left,
      end_arc_lengths->left),
    crop_line_string(
      lanelet_sequence.rightBound().basicLineString(), start_arc_lengths->right,
      end_arc_lengths->right)};
}

std::vector<geometry_msgs::msg::Point> crop_line_string(
  const lanelet::BasicLineString3d & line_string, const double s_start, const double s_end)
{
  const auto geom_msgs_points = to_geometry_msgs_points(line_string.begin(), line_string.end());

  if (s_start < 0.) {
    RCLCPP_WARN(
      rclcpp::get_logger("path_generator").get_child("utils").get_child(__func__),
      "Start of crop range is negative, returning input as is");
    return geom_msgs_points;
  }

  if (s_start > s_end) {
    RCLCPP_WARN(
      rclcpp::get_logger("path_generator").get_child("utils").get_child(__func__),
      "Start of crop range is larger than end, returning input as is");
    return geom_msgs_points;
  }

  auto trajectory =
    autoware::experimental::trajectory::Trajectory<geometry_msgs::msg::Point>::Builder()
      .set_xy_interpolator<autoware::experimental::trajectory::interpolator::Linear>()
      .build(geom_msgs_points);
  if (!trajectory) {
    return {};
  }

  trajectory->crop(s_start, s_end - s_start);
  return trajectory->restore();
}

std::optional<PathRange<double>> get_arc_length_on_bounds(
  const lanelet::LaneletSequence & lanelet_sequence, const PathPointWithLaneId & path_point)
{
  auto s_left = 0.;
  auto s_right = 0.;

  for (const auto & lanelet : lanelet_sequence) {
    const auto left_bound_length = lanelet::geometry::length(lanelet.leftBound2d());
    const auto right_bound_length = lanelet::geometry::length(lanelet.rightBound2d());

    if (
      std::find(path_point.lane_ids.begin(), path_point.lane_ids.end(), lanelet.id()) ==
      path_point.lane_ids.end()) {
      s_left += left_bound_length;
      s_right += right_bound_length;
      continue;
    }

    const auto lanelet_point =
      lanelet::utils::conversion::toLaneletPoint(path_point.point.pose.position).basicPoint2d();
    s_left += lanelet::geometry::toArcCoordinates(lanelet.leftBound2d(), lanelet_point).length;
    s_right += lanelet::geometry::toArcCoordinates(lanelet.rightBound2d(), lanelet_point).length;

    return PathRange<double>{s_left, s_right};
  }

  // The path point is outside lanelet_sequence
  return std::nullopt;
}

std::optional<experimental::trajectory::Trajectory<PathPointWithLaneId>>
connect_path_to_goal_inside_lanelets(
  const experimental::trajectory::Trajectory<PathPointWithLaneId> & path,
  const lanelet::ConstLanelets & lanelets, const geometry_msgs::msg::Pose & goal_pose,
  const lanelet::Id goal_lane_id, const double connection_section_length,
  const double pre_goal_offset)
{
  for (auto m = connection_section_length; m > 0.0; m -= 0.1) {
    auto path_to_goal = connect_path_to_goal(path, goal_pose, goal_lane_id, m, pre_goal_offset);
    if (!is_path_inside_lanelets(path_to_goal, lanelets)) {
      continue;
    }
    path_to_goal.align_orientation_with_trajectory_direction();
    return path_to_goal;
  }
  return std::nullopt;
}

experimental::trajectory::Trajectory<PathPointWithLaneId> connect_path_to_goal(
  const experimental::trajectory::Trajectory<PathPointWithLaneId> & path,
  const geometry_msgs::msg::Pose & goal_pose, const lanelet::Id goal_lane_id,
  const double connection_section_length, const double pre_goal_offset)
{
  auto has_goal_lane_id = [&](const PathPointWithLaneId & point) {
    const auto & lane_ids = point.lane_ids;
    return std::find(lane_ids.begin(), lane_ids.end(), goal_lane_id) != lane_ids.end();
  };

  const auto s_goal =
    autoware::experimental::trajectory::closest_with_constraint(path, goal_pose, has_goal_lane_id);

  if (!s_goal) {
    RCLCPP_WARN(
      rclcpp::get_logger("path_generator").get_child("utils").get_child(__func__),
      "Failed to find closest point to goal, returning input as is");
    return path;
  }

  auto goal = path.compute(*s_goal);
  goal.point.pose = goal_pose;
  goal.point.longitudinal_velocity_mps = 0.0;

  const auto pre_goal_pose =
    autoware_utils_geometry::calc_offset_pose(goal_pose, -pre_goal_offset, 0.0, 0.0);
  auto pre_goal = path.compute(autoware::experimental::trajectory::closest(path, pre_goal_pose));
  pre_goal.point.pose = pre_goal_pose;

  std::vector<PathPointWithLaneId> path_points_to_goal;

  if (*s_goal <= connection_section_length) {
    // If distance from start to goal is smaller than connection_section_length and start is
    // farther from goal than pre-goal, we just connect start, pre-goal, and goal.
    path_points_to_goal = {path.compute(0)};
  } else {
    const auto cropped_path =
      autoware::experimental::trajectory::crop(path, 0, *s_goal - connection_section_length);
    path_points_to_goal = cropped_path.restore(2);
  }
  if (*s_goal > pre_goal_offset) {
    path_points_to_goal.push_back(pre_goal);
  }
  path_points_to_goal.push_back(goal);

  if (const auto output = autoware::experimental::trajectory::pretty_build(path_points_to_goal)) {
    return *output;
  }

  return path;
}

bool is_pose_inside_lanelets(
  const geometry_msgs::msg::Pose & pose, const lanelet::ConstLanelets & lanelets)
{
  return std::any_of(lanelets.begin(), lanelets.end(), [&](const lanelet::ConstLanelet & l) {
    return lanelet::utils::isInLanelet(pose, l);
  });
}

bool is_path_inside_lanelets(
  const experimental::trajectory::Trajectory<PathPointWithLaneId> & path,
  const lanelet::ConstLanelets & lanelets)
{
  for (double s = 0.0; s < path.length(); s += 0.1) {
    const auto point = path.compute(s);
    if (!is_pose_inside_lanelets(point.point.pose, lanelets)) {
      return false;
    }
  }
  return true;
}

TurnIndicatorsCommand get_turn_signal(
  const PathWithLaneId & path, const PlannerData & planner_data,
  const geometry_msgs::msg::Pose & current_pose, const double current_vel,
  const double search_distance, const double search_time, const double angle_threshold_deg,
  const double base_link_to_front)
{
  TurnIndicatorsCommand turn_signal;
  turn_signal.command = TurnIndicatorsCommand::NO_COMMAND;

  const lanelet::BasicPoint2d current_point{current_pose.position.x, current_pose.position.y};
  const auto base_search_distance = search_distance + current_vel * search_time;

  std::vector<lanelet::Id> searched_lanelet_ids = {};
  std::optional<double> arc_length_from_vehicle_front_to_lanelet_start = std::nullopt;

  auto calc_arc_length =
    [&](const lanelet::ConstLanelet & lanelet, const lanelet::BasicPoint2d & point) -> double {
    return lanelet::geometry::toArcCoordinates(lanelet.centerline2d(), point).length;
  };

  for (const auto & point : path.points) {
    for (const auto & lane_id : point.lane_ids) {
      if (exists(searched_lanelet_ids, lane_id)) {
        continue;
      }
      searched_lanelet_ids.push_back(lane_id);

      const auto lanelet = planner_data.lanelet_map_ptr->laneletLayer.get(lane_id);
      if (!get_next_lanelet_within_route(lanelet, planner_data)) {
        continue;
      }

      if (
        !arc_length_from_vehicle_front_to_lanelet_start &&
        !lanelet::geometry::inside(lanelet, current_point)) {
        continue;
      }

      if (lanelet.hasAttribute("turn_direction")) {
        turn_signal.command =
          turn_signal_command_map.at(lanelet.attribute("turn_direction").value());

        if (arc_length_from_vehicle_front_to_lanelet_start) {  // ego is in front of lanelet
          if (
            *arc_length_from_vehicle_front_to_lanelet_start >
            lanelet.attributeOr("turn_signal_distance", base_search_distance)) {
            turn_signal.command = TurnIndicatorsCommand::NO_COMMAND;
          }
          return turn_signal;
        }

        // ego is inside lanelet
        const auto required_end_point_opt =
          get_turn_signal_required_end_point(lanelet, angle_threshold_deg);
        if (!required_end_point_opt) continue;
        if (
          calc_arc_length(lanelet, current_point) <=
          calc_arc_length(lanelet, required_end_point_opt.value())) {
          return turn_signal;
        }
      }

      const auto lanelet_length = lanelet::utils::getLaneletLength2d(lanelet);
      if (arc_length_from_vehicle_front_to_lanelet_start) {
        *arc_length_from_vehicle_front_to_lanelet_start += lanelet_length;
      } else {
        arc_length_from_vehicle_front_to_lanelet_start =
          lanelet_length - calc_arc_length(lanelet, current_point) - base_link_to_front;
      }
      break;
    }
  }

  return turn_signal;
}

std::optional<lanelet::ConstPoint2d> get_turn_signal_required_end_point(
  const lanelet::ConstLanelet & lanelet, const double angle_threshold_deg)
{
  std::vector<geometry_msgs::msg::Pose> centerline_poses(lanelet.centerline().size());
  std::transform(
    lanelet.centerline().begin(), lanelet.centerline().end(), centerline_poses.begin(),
    [](const auto & point) {
      geometry_msgs::msg::Pose pose{};
      pose.position = lanelet::utils::conversion::toGeomMsgPt(point);
      return pose;
    });

  // NOTE: Trajectory does not support less than 4 points, so resample if less than 4.
  //       This implementation should be replaced in the future
  if (centerline_poses.size() < 4) {
    const auto lanelet_length = autoware::motion_utils::calcArcLength(centerline_poses);
    const auto resampling_interval = lanelet_length / 4.0;
    std::vector<double> resampled_arclength;
    for (double s = 0.0; s < lanelet_length; s += resampling_interval) {
      resampled_arclength.push_back(s);
    }
    if (lanelet_length - resampled_arclength.back() < autoware::motion_utils::overlap_threshold) {
      resampled_arclength.back() = lanelet_length;
    } else {
      resampled_arclength.push_back(lanelet_length);
    }
    centerline_poses =
      autoware::motion_utils::resamplePoseVector(centerline_poses, resampled_arclength);
    if (centerline_poses.size() < 4) return std::nullopt;
  }

  auto centerline =
    autoware::experimental::trajectory::Trajectory<geometry_msgs::msg::Pose>::Builder{}.build(
      centerline_poses);
  if (!centerline) return std::nullopt;
  centerline->align_orientation_with_trajectory_direction();

  const auto terminal_yaw = tf2::getYaw(centerline->compute(centerline->length()).orientation);
  const auto intervals = autoware::experimental::trajectory::find_intervals(
    centerline.value(),
    [terminal_yaw, angle_threshold_deg](const geometry_msgs::msg::Pose & point) {
      const auto yaw = tf2::getYaw(point.orientation);
      return std::fabs(autoware_utils::normalize_radian(yaw - terminal_yaw)) <
             autoware_utils::deg2rad(angle_threshold_deg);
    });
  if (intervals.empty()) return std::nullopt;

  return lanelet::utils::conversion::toLaneletPoint(
    centerline->compute(intervals.front().start).position);
}
}  // namespace utils
}  // namespace autoware::path_generator
