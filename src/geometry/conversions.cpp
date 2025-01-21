// clang: MatousFormat
#include <mrs_lib/geometry/conversions.h>

namespace mrs_lib
{
  namespace geometry
  {

    /* conversions from/to Eigen //{ */

    geometry_msgs::msg::Point fromEigen(const Eigen::Vector3d& what)
    {

      geometry_msgs::msg::Point pt;

      pt.x = what.x();
      pt.y = what.y();
      pt.z = what.z();

      return pt;
    }

    geometry_msgs::msg::Vector3 fromEigenVec(const Eigen::Vector3d& what)
    {

      geometry_msgs::msg::Vector3 pt;

      pt.x = what.x();
      pt.y = what.y();
      pt.z = what.z();

      return pt;
    }

    Eigen::Vector3d toEigen(const geometry_msgs::msg::Point& what)
    {
      return {what.x, what.y, what.z};
    }

    Eigen::Vector3d toEigen(const geometry_msgs::msg::Vector3& what)
    {
      return {what.x, what.y, what.z};
    }

    Eigen::Matrix<double, 6, 6> toEigenMatrix(const std::array<double, 36>& what)
    {

      Eigen::Matrix<double, 6, 6> ret;

      for (int r = 0; r < 6; r++)
        for (int c = 0; c < 6; c++)
          ret(r, c) = what.at(6 * r + c);

      return ret;
    }

    geometry_msgs::msg::Quaternion fromEigen(const Eigen::Quaterniond& what)
    {

      geometry_msgs::msg::Quaternion q;

      q.x = what.x();
      q.y = what.y();
      q.z = what.z();
      q.w = what.w();

      return q;
    }

    Eigen::Quaterniond toEigen(const geometry_msgs::msg::Quaternion& what)
    {

      // better to do this manually than through the constructor to avoid ambiguities (e.g. position of x and w)
      Eigen::Quaterniond q;

      q.x() = what.x;
      q.y() = what.y;
      q.z() = what.z;
      q.w() = what.w;

      return q;
    }

    //}

  }  // namespace geometry
}  // namespace mrs_lib
