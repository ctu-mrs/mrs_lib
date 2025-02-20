#include <mrs_lib/median_filter.h>

namespace mrs_lib
{
/* constructor overloads //{ */

MedianFilter::MedianFilter(const size_t buffer_length, const double min_value, const double max_value, const double max_diff)
    : m_median(std::nullopt), m_min_valid(min_value), m_max_valid(max_value), m_max_diff(max_diff) {
  m_buffer.set_capacity(buffer_length);
  m_buffer_sorted.reserve(buffer_length);
}

MedianFilter::MedianFilter() : m_median(std::nullopt), m_min_valid(0.0), m_max_valid(0.0), m_max_diff(0.0) {
  m_buffer.set_capacity(0);
}

MedianFilter::MedianFilter(const MedianFilter& other) {
  *this = other;
}

MedianFilter::MedianFilter(MedianFilter&& other) {
  *this = other;
}

//}

/* operator=() method and overloads //{ */
MedianFilter& MedianFilter::operator=(const MedianFilter& other) {
  std::scoped_lock lck(other.m_mtx, m_mtx);

  m_buffer        = other.m_buffer;
  m_buffer_sorted = other.m_buffer_sorted;
  m_median        = other.m_median;

  // parameters specified by the user
  m_min_valid = other.m_min_valid;
  m_max_valid = other.m_max_valid;
  m_max_diff  = other.m_max_diff;

  return *this;
}

MedianFilter& MedianFilter::operator=(MedianFilter&& other) {
  std::scoped_lock lck(other.m_mtx, m_mtx);

  m_buffer        = std::move(other.m_buffer);
  m_buffer_sorted = std::move(other.m_buffer_sorted);
  m_median        = std::move(other.m_median);

  // parameters specified by the user
  m_min_valid = other.m_min_valid;
  m_max_valid = other.m_max_valid;
  m_max_diff  = other.m_max_diff;

  return *this;
}
//}

/* add() method //{ */
void MedianFilter::add(const double value) {
  std::scoped_lock lck(m_mtx);
  // add the value to the buffer
  m_buffer.push_back(value);
  // reset the cached median value
  m_median = std::nullopt;
}
//}

/* check() method //{ */
bool MedianFilter::check(const double value) {
  std::scoped_lock lck(m_mtx);
  // check if all constraints are met
  const double diff = m_buffer.empty() ? 0.0 : std::abs(median() - value);
  return value > m_min_valid && value < m_max_valid && diff < m_max_diff;
}
//}

/* addCheck() method //{ */
bool MedianFilter::addCheck(const double value) {
  std::scoped_lock lck(m_mtx);
  add(value);
  return check(value);
}
//}

/* clear() method //{ */
void MedianFilter::clear() {
  std::scoped_lock lck(m_mtx);
  m_median = std::nullopt;
  m_buffer.clear();
}
//}

/* full() method //{ */
bool MedianFilter::full() const {
  std::scoped_lock lck(m_mtx);
  return m_buffer.full();
}
//}

/* median() method //{ */
double MedianFilter::median() const {
  std::scoped_lock lck(m_mtx);
  // if the value was already calculated, just return it
  if (m_median.has_value())
    return m_median.value();

  // check if there are even any numbers to calculate the median from
  if (m_buffer.empty()) {
    m_median = std::numeric_limits<double>::quiet_NaN();
    return m_median.value();
  }

  // remove any elements from buffer_sorted
  m_buffer_sorted.clear();
  // copy all elements from the input buffer to buffer_sorted
  m_buffer_sorted.insert(std::end(m_buffer_sorted), std::begin(m_buffer), std::end(m_buffer));
  // check for the special case of the median when there is an even number of numbers in the set
  const bool even_set = m_buffer_sorted.size() % 2 == 0;

  // if it's an even set, we'll need one more element sorted than for an odd set of numbers
  const size_t median_pos = even_set ? std::ceil(m_buffer_sorted.size() / 2.0) : std::floor(m_buffer_sorted.size() / 2.0);
  // actually sort the elements in buffer_sorted up to the n-th element
  std::nth_element(std::begin(m_buffer_sorted), std::begin(m_buffer_sorted) + median_pos, std::end(m_buffer_sorted));

  // special case for a median of an even set of numbers
  if (even_set)
    m_median = (m_buffer_sorted.at(median_pos) + m_buffer_sorted.at(median_pos - 1)) / 2.0;
  // the "normal" case with an odd set
  else
    m_median = m_buffer_sorted.at(median_pos);
  // return the now-cached value
  return m_median.value();
}
//}

/* initialized() method //{ */
bool MedianFilter::initialized() const {
  std::scoped_lock lck(m_mtx);
  return m_buffer.size() > 0;
}
//}

/* setBufferLength() method //{ */
void MedianFilter::setBufferLength(const size_t buffer_length) {
  std::scoped_lock lck(m_mtx);
  // the median may change if the some values are discarded
  if (buffer_length < m_buffer.size())
    m_median = std::nullopt;

  m_buffer.set_capacity(buffer_length);
  m_buffer_sorted.reserve(buffer_length);
}
//}

/* setMinValue() method //{ */
void MedianFilter::setMinValue(const double min_value) {
  std::scoped_lock lck(m_mtx);
  m_min_valid = min_value;
}
//}

/* setMaxValue() method //{ */
void MedianFilter::setMaxValue(const double max_value) {
  std::scoped_lock lck(m_mtx);
  m_max_valid = max_value;
}
//}

/* setMaxDifference() method //{ */
void MedianFilter::setMaxDifference(const double max_diff) {
  std::scoped_lock lck(m_mtx);
  m_max_diff = max_diff;
}
//}

}  // namespace mrs_lib
