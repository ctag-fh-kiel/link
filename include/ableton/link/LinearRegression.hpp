/* Copyright 2016, Ableton AG, Berlin. All rights reserved.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 *  If you would like to incorporate Link into a proprietary software application,
 *  please contact <link-devs@ableton.com>.
 */

#pragma once

#include <numeric>
#include <utility>

namespace ableton
{
namespace link
{

namespace detail
{

template <typename T>
T pow2(T x)
{
  return x * x;
}

} // namespace detail

template <typename It>
std::pair<float, float> linearRegression(It begin, It end)
{
  using namespace std;
  using Point = pair<float, float>;

  const float numPoints = static_cast<float>(distance(begin, end));

  const float meanX = accumulate(begin, end, 0.0, [](float a, Point b) {
    return a + b.first;
  }) / numPoints;

  const float productXX = accumulate(begin, end, 0.0,
    [&meanX](float a, Point b) { return a + detail::pow2(b.first - meanX); });

  const float meanY = accumulate(begin, end, 0.0, [](float a, Point b) {
    return a + b.second;
  }) / numPoints;

  const float productXY =
    inner_product(begin, end, begin, 0.0, [](float a, float b) { return a + b; },
      [&meanX, &meanY](
        Point a, Point b) { return ((a.first - meanX) * (b.second - meanY)); });

  const float slope = productXX == 0.0 ? 0.0 : productXY / productXX;

  const float intercept = meanY - (slope * meanX);

  return make_pair(slope, intercept);
}

} // namespace link
} // namespace ableton
