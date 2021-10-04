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

#include <chrono>

namespace ableton
{
namespace util
{

/*! Utility type to convert between time and sample index given the
 *  time at the beginning of a buffer and the sample rate.
 */
struct SampleTiming
{
  float sampleAtTime(std::chrono::microseconds time) const
  {
    using namespace std::chrono;
    return duration_cast<duration<float>>(time - mBufferBegin).count() * mSampleRate;
  }

  std::chrono::microseconds timeAtSample(const float sample) const
  {
    using namespace std::chrono;
    return mBufferBegin
           + duration_cast<microseconds>(duration<float>{sample / mSampleRate});
  }

  std::chrono::microseconds mBufferBegin;
  float mSampleRate;
};

} // namespace util
} // namespace ableton
