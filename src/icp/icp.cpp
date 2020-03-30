#include "mrs_lib/icp.h"

e::Vector3d icp::ICPSolver::matchSquare(std::vector<e::Vector3d> input, double side, e::Vector3d &gradient){
  std::vector<std::pair<e::Vector2d,e::Vector2d>> square = makeSquare(side);
  std::vector<e::Vector2d> input_2d;
  e::Vector2d p_2d;
  for (auto p : input){
    p_2d.x() = p.x();
    p_2d.y() = p.y();
    input_2d.push_back(p_2d);
  }
  e::Vector2d center = flatAverage(input_2d);
  /* double yaw = optimizeRotation(input_2d,square, center); */
  double yaw = optimizeFull(input_2d,square, center, gradient);
  /* ROS_INFO_STREAM("e " << yaw); */

  /* center = optimizeTranslation(square); *///TODO
  return e::Vector3d(center.x(), center.y(), yaw);
}
//}

/* matchTriangle() //{ */
e::Vector3d icp::ICPSolver::matchTriangle(std::vector<e::Vector3d> input, double side, e::Vector3d &gradient){
  std::vector<std::pair<e::Vector2d,e::Vector2d>> triangle = makeTriangle(side);
  std::vector<e::Vector2d> input_2d;
  e::Vector2d p_2d;
  for (auto p : input){
    p_2d.x() = p.x();
    p_2d.y() = p.y();
    input_2d.push_back(p_2d);
  }
  e::Vector2d center = flatAverage(input_2d);
  /* double yaw = optimizeRotation(input_2d,square, center); */
  double yaw = optimizeFull(input_2d,triangle, center, gradient);
  /* ROS_INFO_STREAM("e " << yaw); */

  /* center = optimizeTranslation(square); *///TODO
  return e::Vector3d(center.x(), center.y(), yaw);
}
//}

/* matchLine() //{ */
e::Vector3d icp::ICPSolver::matchLine(std::vector<e::Vector3d> input, double side, e::Vector3d &gradient){
  std::vector<std::pair<e::Vector2d,e::Vector2d>> triangle = makeLine(side);
  std::vector<e::Vector2d> input_2d;
  e::Vector2d p_2d;
  for (auto p : input){
    p_2d.x() = p.x();
    p_2d.y() = p.y();
    input_2d.push_back(p_2d);
  }
  e::Vector2d center = flatAverage(input_2d);
  /* double yaw = optimizeRotation(input_2d,square, center); */
  double yaw = optimizeFull(input_2d,triangle, center, gradient);
  /* ROS_INFO_STREAM("e " << yaw); */

  /* center = optimizeTranslation(square); *///TODO
  return e::Vector3d(center.x(), center.y(), yaw);
}

std::vector<std::pair<e::Vector2d,e::Vector2d>> icp::ICPSolver::makeSquare(double side){
  double half = side/2.0;
  std::vector<std::pair<e::Vector2d,e::Vector2d>> output;
  std::pair<e::Vector2d,e::Vector2d> one_side;
  one_side.first = e::Vector2d(half, half);
  one_side.second = e::Vector2d(half, -half);
  output.push_back(one_side);
  one_side.first = e::Vector2d(half, -half);
  one_side.second = e::Vector2d(-half, -half);
  output.push_back(one_side);
  one_side.first = e::Vector2d(-half, -half);
  one_side.second = e::Vector2d(-half, half);
  output.push_back(one_side);
  one_side.first = e::Vector2d(-half, half);
  one_side.second = e::Vector2d(half, half);
  output.push_back(one_side);
  return output;
}
std::vector<std::pair<e::Vector2d,e::Vector2d>> icp::ICPSolver::makeSquareChamfered(double side){
  double half = side/2.0;
  double end = 0.25;
  std::vector<std::pair<e::Vector2d,e::Vector2d>> output;
  std::pair<e::Vector2d,e::Vector2d> one_side;
  one_side.first = e::Vector2d(half, end);
  one_side.second = e::Vector2d(half, -end);
  output.push_back(one_side);
  one_side.first = e::Vector2d(end, -half);
  one_side.second = e::Vector2d(-end, -half);
  output.push_back(one_side);
  one_side.first = e::Vector2d(-half, -end);
  one_side.second = e::Vector2d(-half, end);
  output.push_back(one_side);
  one_side.first = e::Vector2d(-end, half);
  one_side.second = e::Vector2d(end, half);
  output.push_back(one_side);

  one_side.first = e::Vector2d(end, half);
  one_side.second = e::Vector2d(half, end);
  output.push_back(one_side);
  one_side.first = e::Vector2d(half, -end);
  one_side.second = e::Vector2d(end, -half);
  output.push_back(one_side);
  one_side.first = e::Vector2d(-end, -half);
  one_side.second = e::Vector2d(-half, -end);
  output.push_back(one_side);
  one_side.first = e::Vector2d(-half, end);
  one_side.second = e::Vector2d(-end, half);
  output.push_back(one_side);
  return output;
}
std::vector<std::pair<e::Vector2d,e::Vector2d>> icp::ICPSolver::makeTriangle(double side){
  double half = side/2.0;
  double top = 0.329423;
  std::vector<std::pair<e::Vector2d,e::Vector2d>> output;
  std::pair<e::Vector2d,e::Vector2d> one_side;
  one_side.first = e::Vector2d(-half, -half);
  one_side.second = e::Vector2d(top, 0);
  output.push_back(one_side);
  one_side.first = e::Vector2d(-half, half);
  one_side.second = e::Vector2d(top, 0);
  output.push_back(one_side);
  one_side.first = e::Vector2d(-half, -half);
  one_side.second = e::Vector2d(-half, half);
  output.push_back(one_side);
  return output;
}
std::vector<std::pair<e::Vector2d,e::Vector2d>> icp::ICPSolver::makeLine(double side){
  double half = side/2.0;
  double top = 0.329423;
  std::vector<std::pair<e::Vector2d,e::Vector2d>> output;
  std::pair<e::Vector2d,e::Vector2d> one_side;
  one_side.first = e::Vector2d(0, -half);
  one_side.second = e::Vector2d(0, half);
  output.push_back(one_side);
  return output;
}

e::Vector2d icp::ICPSolver::flatAverage(std::vector<e::Vector2d> input){
  e::Vector2d sum(0,0);
  for (auto p : input){
    sum +=p;
  }
  sum /= (double)(input.size());
  return sum;
}

e::Vector3d icp::ICPSolver::fullAverage(std::vector<e::Vector3d> input){
  e::Vector3d sum(0,0,0);
  for (auto p : input){
    sum +=p;
  }
  sum /= (double)(input.size());
  return sum;
}

double icp::ICPSolver::optimizeRotation(std::vector<e::Vector2d> input, std::vector<std::pair<e::Vector2d,e::Vector2d>> shape, e::Vector2d center){
  std::vector<std::pair<e::Vector2d,e::Vector2d>> shape_shifted = translateShape(shape, center);
  double yaw_step = 5.0*(M_PI/180.0);
  double error_total = totalError(shape_shifted, input);
  std::vector<std::pair<e::Vector2d,e::Vector2d>> shape_left, shape_right;
  double gradient = std::numeric_limits<double>::max();
  double yaw_curr = 0;
  shape_shifted = initializeYaw(input, shape_shifted, 15*(M_PI/180.0), center, yaw_curr);
  double threshold = (int)(input.size())*0.001;
  int iters = 0;
  while ((error_total > threshold) && (fabs(gradient) > 0.0001) && (iters < 50)){
    shape_left = rotateShape(shape_shifted, center, yaw_step);
    shape_right = rotateShape(shape_shifted, center, -yaw_step);
    double left_error = totalError(shape_left, input);
    double right_error = totalError(shape_right, input);
    gradient = (left_error-right_error)/(2*yaw_step)/((double)(input.size()));
    double yaw_diff = -gradient;
    yaw_curr = yaw_curr+yaw_diff;
    shape_shifted = rotateShape(shape_shifted, center, yaw_diff);
    error_total = totalError(shape_shifted, input);
    yaw_step = fabs(yaw_diff/2);
    iters++;
    ROS_INFO_STREAM("[FireLoclaize]: icpish: yaw: "<< yaw_curr << " error: " << error_total << " gradient: "<< gradient << " from " << left_error << " and " << right_error);
  }
  ROS_INFO_STREAM("[FireLoclaize]: icpish final: yaw: "<< yaw_curr << " error: " << error_total << " gradient: "<< gradient);
  return yaw_curr;
}

double icp::ICPSolver::optimizeFull(std::vector<e::Vector2d> input, std::vector<std::pair<e::Vector2d,e::Vector2d>> shape, e::Vector2d &center, e::Vector3d &gradient){
  std::vector<std::pair<e::Vector2d,e::Vector2d>> shape_shifted = translateShape(shape, center);
  double yaw_step = 5.0*(M_PI/180.0);
  double x_step = 0.1;
  double y_step = 0.1;
  double error_total = totalError(shape_shifted, input);
  std::vector<std::pair<e::Vector2d,e::Vector2d>> shape_ccv, shape_cv;
  std::vector<std::pair<e::Vector2d,e::Vector2d>> shape_left, shape_right;
  std::vector<std::pair<e::Vector2d,e::Vector2d>> shape_front, shape_back;
  double ccv_error, cv_error;
  double left_error, right_error;
  double front_error, back_error;
  gradient = e::Vector3d (
      std::numeric_limits<double>::max(),
      std::numeric_limits<double>::max(),
      std::numeric_limits<double>::max()
      );
  double yaw_curr = 0;
  shape_shifted = initializeYaw(input, shape_shifted, 15*(M_PI/180.0), center, yaw_curr);
  double threshold = (int)(input.size())*0.001;
  int iters = 0;
  while ((error_total > threshold) && ((gradient.norm()) > 0.0001) && (iters < 50)){
    shape_ccv   = rotateShape(shape_shifted, center, yaw_step);
    shape_cv    = rotateShape(shape_shifted, center, -yaw_step);
    shape_left  = translateShape(shape_shifted, e::Vector2d(0,y_step));
    shape_right = translateShape(shape_shifted, e::Vector2d(0,-y_step));
    shape_front = translateShape(shape_shifted, e::Vector2d(x_step,0));
    shape_back  = translateShape(shape_shifted, e::Vector2d(-x_step,0));
    ccv_error = totalError(shape_ccv, input);
    cv_error = totalError(shape_cv, input);
    left_error = totalError(shape_left, input);
    right_error = totalError(shape_right, input);
    front_error = totalError(shape_front, input);
    back_error = totalError(shape_back, input);
    gradient.z() = ((ccv_error-cv_error)/(2*yaw_step))/((double)(input.size()));
    gradient.y() = ((left_error-right_error)/(2*y_step))/((double)(input.size()));
    gradient.x() = ((front_error-back_error)/(2*x_step))/((double)(input.size()));
    double yaw_diff, y_diff, x_diff;
    if (gradient.z() > 1E-9)
      yaw_diff = -gradient.z();
    else
      yaw_diff = 0;
    if (gradient.y() > 1E-9)
      y_diff = -gradient.y();
    else
      y_diff = 0;
    if (gradient.x() > 1E-9)
      x_diff = -gradient.x();
    else
      x_diff = 0;
    yaw_curr = yaw_curr+yaw_diff;
    center = center+e::Vector2d(x_diff,y_diff);
    shape_shifted = rotateShape(translateShape(shape_shifted, e::Vector2d(x_diff, y_diff)), center, yaw_diff);
    error_total = totalError(shape_shifted, input);
    yaw_step = fabs(yaw_diff/2);
    iters++;
    /* ROS_INFO_STREAM("[FireLoclaize]: icpish: yaw: "<< yaw_curr << " center: " <<  center.transpose() << " error: " << error_total << " gradient: "<< gradient.transpose()); */
  }
  //final gradient check
  double mean_error = error_total/((double)(input.size()));
  yaw_step = 5.0*(M_PI/180.0);
  x_step = 0.10;
  y_step = 0.10;
  shape_ccv   = rotateShape(shape_shifted, center, yaw_step);
  shape_cv    = rotateShape(shape_shifted, center, -yaw_step);
  shape_left  = translateShape(shape_shifted, e::Vector2d(0,y_step));
  shape_right = translateShape(shape_shifted, e::Vector2d(0,-y_step));
  shape_front = translateShape(shape_shifted, e::Vector2d(x_step,0));
  shape_back  = translateShape(shape_shifted, e::Vector2d(-x_step,0));
  gradient.z() = ((((abs(ccv_error)+abs(cv_error))/2))/(yaw_step))/((double)(input.size()));
  gradient.y() = ((((abs(left_error)+abs(right_error))/2)-mean_error)/(y_step))/((double)(input.size()));
  gradient.x() = ((((abs(front_error)+abs(back_error))/2)-mean_error)/(x_step))/((double)(input.size()));
  /* ROS_INFO_STREAM("[FireLoclaize]: icpish: yaw: "<< yaw_curr << " center: " <<  center.transpose() << " error: " << error_total << " gradient: "<< gradient.transpose()); */
  /* ROS_INFO_STREAM("e " << yaw); */
  return yaw_curr;
}

double icp::ICPSolver::totalError(std::vector<std::pair<e::Vector2d,e::Vector2d>> ref_shape, std::vector<e::Vector2d> input){
  double error = 0;
  std::vector<double> side_errors;
  for (int i=0; i<(int)(input.size()); i++){
    double best_side_error = std::numeric_limits<double>::max();
    int best_side = -1;
    for (int j=0; j<(int)(ref_shape.size()); j++){
      double curr_side_error = distSquarePointSegment(input[i], ref_shape[j]);
      side_errors.push_back(curr_side_error);
      /* ROS_INFO_STREAM("curr_side_error: " << curr_side_error); */
      if (curr_side_error < best_side_error){
        best_side_error = curr_side_error;
        best_side = j;
      }

    }
    /* if (best_side >= 0) */
    /*   ROS_INFO_STREAM("bse: " << best_side_error << " P: " << input[i].transpose() << " s: " << ref_shape[best_side].first.transpose() << " : " << ref_shape[best_side].second.transpose()); */
    /* else */
    /*   ROS_INFO_STREAM("bse: not found!"); */
    error += best_side_error;
  }
  if (error > 1E100){
    ROS_INFO_STREAM("TOTAL ERROR: " << error);
    ROS_INFO_STREAM("side errors: " << side_errors.size());
    for (auto s : side_errors)
      ROS_INFO_STREAM("s: " << s);
    ROS_INFO_STREAM("input: " << input.size());
    for (auto i : input)
      ROS_INFO_STREAM("i: " << i);
    ROS_INFO_STREAM("ref_shape size: " << ref_shape.size(););
  }
  return error;
}

double icp::ICPSolver::distSquarePointSegment(e::Vector2d point, std::pair<e::Vector2d,e::Vector2d> segment){
  // Return minimum distance between line segment vw and point p
  float l2 = (segment.first - segment.second).squaredNorm();  // i.e. |w-v|^2 -  avoid a sqrt
  if (l2 == 0.0) return (segment.first - point).squaredNorm();   // v == w case
  // Consider the line extending the segment, parameterized as v + t (w - v).
  // We find projection of point p onto the line. 
  // It falls where t = [(p-v) . (w-v)] / |w-v|^2
  // We clamp t from [0,1] to handle points outside the segment vw.
  float t = ((point - segment.first).dot(segment.second - segment.first)) / l2;
  if (std::isnan(t))
    ROS_INFO_STREAM("t: " << t<< " : " << point.transpose() << " : "<< segment.first.transpose()<< " : " << segment.second.transpose() << " : " << l2);
  if ( t < 0 ){
    /* if (((point - segment.first).squaredNorm() > 10000) || std::isnan((point - segment.first).squaredNorm())) */
    /*   ROS_INFO_STREAM("a: " << segment.first.transpose() << ":" << point.transpose()); */
    return (point - segment.first).squaredNorm();
  } else if ( t > 1 ){
    /* if (((point - segment.second).squaredNorm() > 10000) || (std::isnan((point - segment.second).squaredNorm()))) */
    /*   ROS_INFO_STREAM("b: " << segment.second.transpose() << ":" << point.transpose()); */
    return (point - segment.second).squaredNorm();
  } else {
    e::Vector2d projection = segment.first + t * (segment.second - segment.first);  // Projection falls on the segment
    if (((point - projection).squaredNorm() > 10000) || (std::isnan((point - projection).squaredNorm())))
      ROS_INFO_STREAM("c: " << projection.transpose() << ":" << point.transpose());
    return (point - projection).squaredNorm();
  }
}

std::vector<std::pair<e::Vector2d,e::Vector2d>> icp::ICPSolver::copyShape(std::vector<std::pair<e::Vector2d,e::Vector2d>> shape){
  std::vector<std::pair<e::Vector2d,e::Vector2d>> output;
  std::pair<e::Vector2d,e::Vector2d> one_side;
  for (auto side : shape){
    one_side.first = side.first;
    one_side.second = side.second;
    output.push_back(one_side);
  }
  return output;
}

std::vector<std::pair<e::Vector2d,e::Vector2d>> icp::ICPSolver::initializeYaw(std::vector<e::Vector2d> input, std::vector<std::pair<e::Vector2d,e::Vector2d>> ref_shape, double yaw_step, e::Vector2d center, double& yaw_best){
  int step_count = ceil((M_PI*2)/yaw_step) + 1;
  double yaw_curr = yaw_best;
  double error_best = std::numeric_limits<double>::max();
  double error_curr;
  std::vector<std::pair<e::Vector2d,e::Vector2d>> shape_best = copyShape(ref_shape);
  std::vector<std::pair<e::Vector2d,e::Vector2d>> shape_curr = copyShape(ref_shape);
  for (int i=0; i<step_count; i++){
    shape_curr = rotateShape(shape_curr, center, yaw_step);
    error_curr = totalError(shape_curr, input);
    yaw_curr +=yaw_step;
    if (error_curr < error_best){
      shape_best = copyShape(shape_curr);
      error_best = error_curr;
      yaw_best = yaw_curr;
    }
  }
  return shape_best;
}

std::vector<std::pair<e::Vector2d,e::Vector2d>> icp::ICPSolver::translateShape(std::vector<std::pair<e::Vector2d,e::Vector2d>> shape, e::Vector2d pos){
  std::vector<std::pair<e::Vector2d,e::Vector2d>> output;
  std::pair<e::Vector2d,e::Vector2d> one_side;
  for (auto side : shape){
    one_side.first = side.first+pos;
    one_side.second = side.second+pos;
    output.push_back(one_side);
  }
  return output;
}
std::vector<std::pair<e::Vector2d,e::Vector2d>> icp::ICPSolver::rotateShape(std::vector<std::pair<e::Vector2d,e::Vector2d>> shape, e::Vector2d center, double angle){
  std::vector<std::pair<e::Vector2d,e::Vector2d>> output;
  std::pair<e::Vector2d,e::Vector2d> one_side;
  e::Rotation2D R(-angle); //yaw is ccw
  for (auto side : shape){
    one_side.first = R*(side.first-center)+center;
    one_side.second = R*(side.second-center)+center;
    output.push_back(one_side);
  }
  return output;
}
