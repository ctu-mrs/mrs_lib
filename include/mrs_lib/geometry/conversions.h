#ifndef GEOMETRY_CONVERSIONS_H
#define GEOMETRY_CONVERSIONS_H

namespace mrs_lib
{
  namespace geometry
  {

    geometry_msgs::Point fromEigen(const Eigen::Vector3d& what)
    {
      geometry_msgs::Point pt;
      pt.x = what.x();
      pt.y = what.y();
      pt.z = what.z();
      return pt;
    }

    Eigen::Vector3d toEigen(const geometry_msgs::Point& what)
    {
      return {what.x, what.y, what.z};
    }

    geometry_msgs::Quaternion fromEigen(const Eigen::Quaterniond& what)
    {
      geometry_msgs::Quaternion q;
      q.x = what.x();
      q.y = what.y();
      q.z = what.z();
      q.w = what.w();
      return q;
    }

    Eigen::Quaterniond toEigen(const geometry_msgs::Quaternion& what)
    {
      // better to do this manually than through the constructor to avoid ambiguities (e.g. position of x and w)
      Eigen::Quaterniond q;
      q.x() = what.x;
      q.y() = what.y;
      q.z() = what.z;
      q.w() = what.w;
      return q;
    }

  }
}

#endif // GEOMETRY_CONVERSIONS_H
