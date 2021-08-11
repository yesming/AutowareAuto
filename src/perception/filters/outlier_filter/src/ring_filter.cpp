// Copyright 2021 Tier IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "outlier_filter/ring_filter.hpp"

namespace autoware
{
namespace perception
{
namespace filters
{
namespace outlier_filter
{
namespace ring_filter
{

RingFilter::RingFilter(
  common::types::float64_t distance_ratio,
  common::types::float64_t object_length_threshold, int num_points_threshold)
{

}

void RingFilter::filter(
  const pcl::PointCloud<common::types::PointXYZIF> & input,
  pcl::PointCloud<common::types::PointXYZIF> & output)
{

}

}  // namesapce ring_filter
}  // namespace outlier_filter
}  // namespace filters
}  // namespace perception
}  // namespace autoware
