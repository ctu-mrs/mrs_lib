#ifndef MEDIAN_FILTER
#define MEDIAN_FILTER

/* author: Daniel Hert */

#include <vector>
#include <ros/ros.h>

class MedianFilter {

public:
  MedianFilter(int buffer_size, double max_valid_value, double min_valid_value, double max_difference);
  bool isValid(double input);

private:
  std::vector<double> buffer;
  int                 buffer_size;
  int                 next;
  double              max_valid_value;
  double              min_valid_value;
  double              max_difference;
};

#endif
