#include "mrs_lib/icp.h"
#include <iostream>

void fixSymmetry(e::Vector3d &input){
  input.z() = fmod(input.z(),M_PI_2);
  if (fabs(input.z())>(M_PI_2/2.0)){
    if (input.z()>0)
      input.z() -= M_PI_2;
    else
      input.z() += M_PI_2;
  }
}

int main(int argc, char** argv){
  e::Vector3d gradient;
  std::vector<e::Vector3d> square_points;

  square_points.push_back(e::Vector3d(0.0,0.1,1.0));
  square_points.push_back(e::Vector3d(0.9,-0.1,1.2));
  square_points.push_back(e::Vector3d(1.1,1.0,0.9));
  square_points.push_back(e::Vector3d(-0.1,0.9,1.0));
  auto square_result = icp::ICPSolver::matchSquare(square_points, 1.0, gradient);
  fixSymmetry(square_result);
  std::cout << "aligned square result: " << square_result.transpose() << std::endl;

  square_points.clear();
  square_points.push_back(e::Vector3d(0.5,-0.2,1.0));
  square_points.push_back(e::Vector3d(1.2,0.5,1.0));
  square_points.push_back(e::Vector3d(0.5,1.2,1.0));
  square_points.push_back(e::Vector3d(-0.2,0.5,1.0));
  square_result.z() = fmod(square_result.z(),M_PI_2);
  square_result = icp::ICPSolver::matchSquare(square_points, 1.0, gradient);
  fixSymmetry(square_result);
  std::cout << "diagonal square result: " << square_result.transpose() << std::endl;

  std::vector<e::Vector3d> l_points;
  l_points.push_back(e::Vector3d(100.0,0.0,1.0));
  l_points.push_back(e::Vector3d(100.0,1.0,1.0));
  l_points.push_back(e::Vector3d(102.0,0.0,1.0));
  std::vector<std::pair<e::Vector2d,e::Vector2d>> l_shape;
  std::pair<e::Vector2d,e::Vector2d> one_side;
  one_side.first = e::Vector2d(0.0, 0.0);
  one_side.second = e::Vector2d(-1.0, 0.0);
  l_shape.push_back(one_side);
  one_side.first = e::Vector2d(0.0, 0.0);
  one_side.second = e::Vector2d(0.0, 2.0);
  l_shape.push_back(one_side);
  auto l_result = icp::ICPSolver::matchShape(l_points,l_shape,gradient);
  std::cout << "l shape result: " << l_result.transpose() << " gradient: " << gradient.transpose() << std::endl;
  
  l_points.clear();
  l_points.push_back(e::Vector3d(100.0,0.0,1.0));
  l_points.push_back(e::Vector3d(99.826,0.985,1.0));
  l_points.push_back(e::Vector3d(101.970,0.347,1.0));
  l_result = icp::ICPSolver::matchShape(l_points,l_shape,gradient);
  std::cout << "rotated l shape result: " << l_result.transpose() << " gradient: " << gradient.transpose() << std::endl;
  
  l_points.clear();
  l_points.push_back(e::Vector3d(100.1,0.1,1.0));
  l_points.push_back(e::Vector3d(100.0,0.9,1.0));
  l_points.push_back(e::Vector3d(101.9,0.0,1.0));
  l_result = icp::ICPSolver::matchShape(l_points,l_shape,gradient);
  std::cout << "deformed l shape result: " << l_result.transpose() << " gradient: " << gradient.transpose() << std::endl;

  return 0;
  }
