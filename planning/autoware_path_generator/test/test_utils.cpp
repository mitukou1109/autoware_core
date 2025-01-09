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

#include "test_utils.hpp"

#include <autoware/universe_utils/geometry/geometry.hpp>

#include <gtest/gtest.h>

namespace autoware::path_generator
{
TEST_F(PathGeneratorUtilsTest, getPreviousLaneletWithinRoute)
{
  {  // Normal case
    const auto lanelet = planner_data_.lanelet_map_ptr->laneletLayer.get(4785);

    const auto prev_lanelet = utils::get_previous_lanelet_within_route(lanelet, planner_data_);

    ASSERT_TRUE(prev_lanelet.has_value());
    ASSERT_EQ(prev_lanelet->id(), 4780);
  }

  {  // The given lanelet is at the start of the route
    const auto lanelet = planner_data_.lanelet_map_ptr->laneletLayer.get(4424);

    const auto prev_lanelet = utils::get_previous_lanelet_within_route(lanelet, planner_data_);

    ASSERT_FALSE(prev_lanelet.has_value());
  }
}

TEST_F(PathGeneratorUtilsTest, getNextLaneletWithinRoute)
{
  {  // Normal case
    const auto lanelet = planner_data_.lanelet_map_ptr->laneletLayer.get(4785);

    const auto next_lanelet = utils::get_next_lanelet_within_route(lanelet, planner_data_);

    ASSERT_TRUE(next_lanelet.has_value());
    ASSERT_EQ(next_lanelet->id(), 4790);
  }

  {  // The given lanelet is at the end of the route
    const auto lanelet = planner_data_.lanelet_map_ptr->laneletLayer.get(4795);

    const auto next_lanelet = utils::get_next_lanelet_within_route(lanelet, planner_data_);

    ASSERT_FALSE(next_lanelet.has_value());
  }
}

TEST_F(PathGeneratorUtilsTest, getLaneletsWithinRouteUpTo)
{
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }

  {  // Normal case
    const auto lanelet = planner_data_.lanelet_map_ptr->laneletLayer.get(4785);
    const auto distance = 30.0;

    const auto prev_lanelets =
      utils::get_lanelets_within_route_up_to(lanelet, planner_data_, distance);

    ASSERT_TRUE(prev_lanelets.has_value());
    ASSERT_EQ(prev_lanelets->size(), 2);
    ASSERT_EQ(prev_lanelets->at(0).id(), 4424);
    ASSERT_EQ(prev_lanelets->at(1).id(), 4780);
  }

  {  // The given distance exceeds the route section
    const auto lanelet = planner_data_.lanelet_map_ptr->laneletLayer.get(4785);
    const auto distance = 80.0;

    const auto prev_lanelets =
      utils::get_lanelets_within_route_up_to(lanelet, planner_data_, distance);

    ASSERT_TRUE(prev_lanelets.has_value());
    ASSERT_EQ(prev_lanelets->size(), 2);
    ASSERT_EQ(prev_lanelets->at(0).id(), 4424);
    ASSERT_EQ(prev_lanelets->at(1).id(), 4780);
  }

  {  // The given distance is negative
    const auto lanelet = planner_data_.lanelet_map_ptr->laneletLayer.get(4785);
    const auto distance = -30.0;

    const auto prev_lanelets =
      utils::get_lanelets_within_route_up_to(lanelet, planner_data_, distance);

    ASSERT_TRUE(prev_lanelets.has_value());
    ASSERT_TRUE(prev_lanelets->empty());
  }

  {  // The given lanelet is not within the route
    const auto lanelet = planner_data_.lanelet_map_ptr->laneletLayer.get(4775);
    const auto distance = 30.0;

    const auto prev_lanelets =
      utils::get_lanelets_within_route_up_to(lanelet, planner_data_, distance);

    ASSERT_FALSE(prev_lanelets.has_value());
  }
}

TEST_F(PathGeneratorUtilsTest, getLaneletsWithinRouteAfter)
{
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }

  {  // Normal case
    const auto lanelet = planner_data_.lanelet_map_ptr->laneletLayer.get(4785);
    const auto distance = 30.0;

    const auto next_lanelets =
      utils::get_lanelets_within_route_after(lanelet, planner_data_, distance);

    ASSERT_TRUE(next_lanelets.has_value());
    ASSERT_EQ(next_lanelets->size(), 2);
    ASSERT_EQ(next_lanelets->at(0).id(), 4790);
    ASSERT_EQ(next_lanelets->at(1).id(), 4795);
  }

  {  // The given distance exceeds the route section
    const auto lanelet = planner_data_.lanelet_map_ptr->laneletLayer.get(4785);
    const auto distance = 80.0;

    const auto next_lanelets =
      utils::get_lanelets_within_route_after(lanelet, planner_data_, distance);

    ASSERT_TRUE(next_lanelets.has_value());
    ASSERT_EQ(next_lanelets->size(), 2);
    ASSERT_EQ(next_lanelets->at(0).id(), 4790);
    ASSERT_EQ(next_lanelets->at(1).id(), 4795);
  }

  {  // The given distance is negative
    const auto lanelet = planner_data_.lanelet_map_ptr->laneletLayer.get(4785);
    const auto distance = -30.0;

    const auto next_lanelets =
      utils::get_lanelets_within_route_after(lanelet, planner_data_, distance);

    ASSERT_TRUE(next_lanelets.has_value());
    ASSERT_TRUE(next_lanelets->empty());
  }

  {  // The given lanelet is not within the route
    const auto lanelet = planner_data_.lanelet_map_ptr->laneletLayer.get(4775);
    const auto distance = 30.0;

    const auto next_lanelets =
      utils::get_lanelets_within_route_after(lanelet, planner_data_, distance);

    ASSERT_FALSE(next_lanelets.has_value());
  }
}

TEST_F(PathGeneratorUtilsTest, getLaneletsWithinRoute)
{
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }

  {  // Normal case
    const auto pose = autoware::test_utils::createPose(55.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    const auto lanelet = get_lanelet_closest_to_pose(pose);
    const auto backward_distance = 40.0;
    const auto forward_distance = 40.0;

    const auto lanelets = utils::get_lanelets_within_route(
      lanelet, planner_data_, pose, backward_distance, forward_distance);

    ASSERT_TRUE(lanelets.has_value());
    ASSERT_EQ(lanelets->size(), 4);
    ASSERT_EQ(lanelets->at(0).id(), 4424);
    ASSERT_EQ(lanelets->at(1).id(), 4780);
    ASSERT_EQ(lanelets->at(2).id(), 4785);
    ASSERT_EQ(lanelets->at(3).id(), 4790);
  }

  {  // The given backward distance is too small to search for the previous lanelets
    const auto pose = autoware::test_utils::createPose(55.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    const auto lanelet = get_lanelet_closest_to_pose(pose);
    const auto backward_distance = 5.0;
    const auto forward_distance = 40.0;

    const auto lanelets = utils::get_lanelets_within_route(
      lanelet, planner_data_, pose, backward_distance, forward_distance);

    ASSERT_TRUE(lanelets.has_value());
    ASSERT_EQ(lanelets->size(), 2);
    ASSERT_EQ(lanelets->at(0).id(), 4785);
    ASSERT_EQ(lanelets->at(1).id(), 4790);
  }

  {  // The given forward distance is too small to search for the next lanelets
    const auto pose = autoware::test_utils::createPose(55.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    const auto lanelet = get_lanelet_closest_to_pose(pose);
    const auto backward_distance = 40.0;
    const auto forward_distance = 20.0;

    const auto lanelets = utils::get_lanelets_within_route(
      lanelet, planner_data_, pose, backward_distance, forward_distance);

    ASSERT_TRUE(lanelets.has_value());
    ASSERT_EQ(lanelets->size(), 3);
    ASSERT_EQ(lanelets->at(0).id(), 4424);
    ASSERT_EQ(lanelets->at(1).id(), 4780);
    ASSERT_EQ(lanelets->at(2).id(), 4785);
  }

  {  // The given lanelet is not within the route
    const auto pose = autoware::test_utils::createPose(-5.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    const auto lanelet = planner_data_.lanelet_map_ptr->laneletLayer.get(4775);
    const auto backward_distance = 40.0;
    const auto forward_distance = 40.0;

    const auto lanelets = utils::get_lanelets_within_route(
      lanelet, planner_data_, pose, backward_distance, forward_distance);

    ASSERT_FALSE(lanelets.has_value());
  }
}
}  // namespace autoware::path_generator
