#pragma once

#include <cstddef>
#include <limits>
#include <stdexcept>

namespace tp {

class TrackIdSequence {
public:
  explicit TrackIdSequence(int start = 4000) { reset(start); }

  void reset(int start)
  {
    if (start < 0) {
      throw std::invalid_argument("track_id_start must be nonnegative");
    }
    next_ = start;
  }

  int next()
  {
    if (next_ == std::numeric_limits<int>::max()) {
      throw std::overflow_error("track ID space exhausted");
    }
    return next_++;
  }

private:
  int next_ = 4000;
};

inline bool isValidTrackAssignment(int assignment, std::size_t track_count)
{
  return assignment >= 0 &&
         static_cast<std::size_t>(assignment) < track_count;
}

}  // namespace tp
