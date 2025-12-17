// clang: TomasFormat
#ifndef MRS_LIB_LINE_OPERATIONS_H
#define MRS_LIB_LINE_OPERATIONS_H

#include <eigen3/Eigen/Eigen>

namespace mrs_lib
{

  namespace safety_zone
  {

    struct Intersection
    {
      Eigen::RowVector2d point;
      bool parallel;
      bool intersect;

      explicit Intersection(bool intersect, bool parallel = false, Eigen::RowVector2d point = Eigen::RowVector2d{});
    };

    Intersection sectionIntersect(Eigen::RowVector2d start1, Eigen::RowVector2d end1, Eigen::RowVector2d start2, Eigen::RowVector2d end2);

  } // namespace safety_zone

} // namespace mrs_lib

#endif
