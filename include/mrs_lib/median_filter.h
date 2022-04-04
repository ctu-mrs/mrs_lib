#ifndef MEDIAN_FILTER
#define MEDIAN_FILTER

/**  \file
     \brief Defines the MedianFilter class.
     \author Daniel Hert
     \author Matouš Vrba - matous.vrba@fel.cvut.cz
 */

#include <boost/circular_buffer.hpp>
#include <mutex>
#include <cmath>
#include <vector>
#include <optional>

namespace mrs_lib
{
  /**
   * \brief Implementation of a median filter with a fixed-length buffer.
   *
   */
  class MedianFilter
  {
    public:
      /*!
       * \brief The main constructor.
       *
       * \param buffer_length   the number of last values to be kept in the buffer.
       * \param min_value       values below this threshold will be discarded (won't be added to the buffer).
       * \param max_value       values above this threshold will be discarded.
       * \param max_diff        values that differ from the current mean by more than this threshold will be discarded.
       */
      MedianFilter(const size_t buffer_length, const double min_value = -std::numeric_limits<double>::infinity(), const double max_value = std::numeric_limits<double>::infinity(), const double max_diff = std::numeric_limits<double>::infinity());

      /*!
       * \brief A convenience empty constructor that will construct an invalid filter.
       *
       * \warning This constructor will construct an unusable filter with a zero-length buffer.
       * To actually initialize this object, use the main constructor.
       * You can use the initialized() method to check whether the object is valid.
       *
       */
      MedianFilter();

      /*!
       * \brief A convenience copy constructor.
       *
       * This constructor copies all data from the object that is being assigned from in a thread-safe manner.
       *
       * \param  other  the object to assign from.
       *
       */
      MedianFilter(const MedianFilter& other);

      /*!
       * \brief A convenience move constructor.
       *
       * This constructor moves all data from the object that is being assigned from in a thread-safe manner, invalidating it.
       *
       * \param  other  the object to assign from. It will be invalid after this method returns.
       *
       */
      MedianFilter(MedianFilter&& other);

      /**
       * \brief A convenience copy assignment operator.
       *
       * This operator copies all data from the object that is being assigned from in a thread-safe manner.
       *
       * \param  other  the object to assign from.
       * \return        a reference to the object being assigned to.
       */
      MedianFilter& operator=(const MedianFilter& other);

      /**
       * \brief A convenience move assignment operator.
       *
       * This operator moves all data from the object that is being assigned from in a thread-safe manner, invalidating it.
       *
       * \param  other  the object to assign from. It will be invalid after this method returns.
       * \return        a reference to the object being assigned to.
       */
      MedianFilter& operator=(MedianFilter&& other);

      /*!
       * \brief Add a new value to the buffer.
       *
       * \note The median value will not be updated until the median() method is called (lazy evaluation).
       *
       * \param value   the new value to be added to the buffer.
       */
      void add(const double value);

      /*!
       * \brief Check whether a value complies with the constraints.
       *
       * The value is compliant if it's above the \p min_value, below the \p max_value
       * and its (absolute) difference from the current mean is below \p max_diff.
       *
       * \param value   the value to be checked.
       * \return        true if the value is compliant, false otherwise.
       */
      bool check(const double value);

      /*!
       * \brief Add a new value to the buffer and check if it complies with the constraints.
       *
       * The value is compliant if it's above the \p min_value, below the \p max_value
       * and its (absolute) difference from the current mean is below \p max_diff.
       *
       * \note The median value will not be updated until the median() method is called (lazy evaluation).
       *
       * \param value   the new value to be added to the buffer and checked.
       * \return        true if the value is compliant, false otherwise.
       */
      bool addCheck(const double value);

      /*!
       * \brief Clear the buffer of all values.
       * 
       * Doesn't change the buffer's length set in the constructor or any other parameters, only clears all stored values.
       */
      void clear();

      /*!
       * \brief Check whether the buffer is filled with values.
       *
       * If true, adding a new value will remove the oldest value in the buffer.
       *
       * \return        true if the buffer contains \p buffer_length values.
       */
      bool full() const;

      /*!
       * \brief Obtain the median.
       *
       * If an up-to-date median value is available, it's not recalculated.
       * Otherwise, the new median value is calculated and then returned (lazy evaluation).
       *
       * \return        the current median value (returns \p nan if the input buffer is empty).
       */
      double median() const;

      /*!
       * \brief Check whether the filter was initialized with a valid buffer length.
       *
       * \return true if the buffer length is larger than zero.
       */
      bool initialized() const;

      /*!
       * \brief Set a new size of the buffer.
       *
       * \note The median value may change.
       *
       * \param buffer_length   the new size of the buffer.
       */
      void setBufferLength(const size_t buffer_length);

      /*!
       * \brief Set a new minimal threshold for new values.
       *
       * \note The current buffer is not changed - the change only applies to new values.
       *
       * \param min_value   the new minimal value of new buffer elements.
       */
      void setMinValue(const double min_value);

      /*!
       * \brief Set a new maximal threshold for new values.
       *
       * \note The current buffer is not changed - the change only applies to new values.
       *
       * \param max_value   the new maximal value of new buffer elements.
       */
      void setMaxValue(const double max_value);

      /*!
       * \brief Set a new maximal difference from median for new values.
       *
       * \note The current buffer is not changed - the change only applies to new values.
       *
       * \param max_diff   the new maximal difference of new buffer elements from the current median.
       */
      void setMaxDifference(const double max_diff);

    private:
      // for thread-safety
      mutable std::recursive_mutex m_mtx;
      // the input buffer
      boost::circular_buffer<double> m_buffer;
      // a helper buffer for sorting the input buffer
      mutable std::vector<double> m_buffer_sorted;
      // the last median value for lazy evaluation
      mutable std::optional<double> m_median;

      // parameters specified by the user
      double m_min_valid;
      double m_max_valid;
      double m_max_diff;
  };

} // namespace mrs_lib

#endif
