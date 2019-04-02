/* author: Daniel Hert */

#include <mrs_lib/MedianFilter.h>

/* constructor //{ */

MedianFilter::MedianFilter(int buffer_size, double max_valid_value, double min_valid_value, double max_difference) {

  is_filled = false;

  // maximum value that is considered valid
  this->max_valid_value = max_valid_value;

  // minimum value that is considered valid
  this->min_valid_value = min_valid_value;

  // if new value is more different from the filter median than this value, it is classified as invalid
  this->max_difference = max_difference;

  this->buffer_size = buffer_size;

  buffer.resize(buffer_size);
  next = 0;
  ROS_INFO("[MedianFilter]: initialized, buffer size: %d", buffer_size);
}

//}

/* isValid() //{ */

bool MedianFilter::isValid(double input) {

  double value = input;

  // add new value to the filter buffer
  buffer[next] = value;
  next++;

  if (next == buffer_size) {
    next = 0;
    is_filled = true;
  }

  // check if new value is valid
  if (input > max_valid_value || input < min_valid_value) {
    return false;
  }

  int    current_value  = 0;
  int    best_med_value = std::numeric_limits<int>::max();
  int    zero_counter   = 0;
  double median         = 0;
  bool   valid          = true;

  // calculate the median from the filter buffer
  for (int i = 0; i < buffer_size; i++) {
    if (buffer[i] == 0) {
      zero_counter++;
    }
  }

  // if there is more zeroes in the buffer than half of the buffer size, the median value is automatically zero
  if (zero_counter < buffer_size / 2) {

    for (int i = 0; i < buffer_size; i++) {

      current_value = 0;

      if (buffer[i] == 0) {
        continue;
      }

      for (int j = 0; j < buffer_size; j++) {

        if (i == j) {
          continue;
        }

        if (buffer[i] > buffer[j]) {
          current_value++;
        } else if (buffer[i] < buffer[j]) {
          current_value--;
        }
      }

      if (abs(current_value) < best_med_value) {
        best_med_value = abs(current_value);
        median         = buffer[i];
      }
    }
  } else {
    median = 0;
  }

  // if a new value is not close to the median, it is discarded
  if (fabs(value - median) > max_difference) {
    valid = false;
  } else {
    valid = true;
  }

  // check whether the output is finite
  if (std::isfinite(value)) {
    return valid;
  } else {

    ROS_WARN_THROTTLE(1.0, "[MedianFilter]: received value is not a finite number!");
    return false;
  }
}

bool MedianFilter::isFilled() {
  return is_filled;
}

//}

