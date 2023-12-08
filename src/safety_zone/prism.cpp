#include "mrs_lib/safety_zone/prism.h"

#include <boost/geometry.hpp>

namespace bg = boost::geometry;

namespace mrs_lib 
{
  Prism::Prism(Polygon& polygon, double max_z, double min_z) {
    std::string msg;
    bool is_valid;
    is_valid = bg::is_valid(polygon, msg);

    if(msg == "Geometry has wrong orientation") {
      bg::reverse(polygon);
      is_valid = bg::is_valid(polygon, msg); 
    }

    if(!is_valid){
      throw std::invalid_argument("The polygon is invalid: " + msg);
    }

    polygon_ = polygon;
    if(max_z < min_z){
      max_z_ = min_z;
      min_z_ = max_z;
    }else{
      max_z_ = max_z;
      min_z_ = min_z;
    }
  }

  void Prism::subscribe(Subscriber* entity) {
    subscribers.emplace(entity);
  }

  void Prism::unsubscribe(Subscriber* entity) {
    subscribers.erase(entity);
  }

  void Prism::notifySubscribers() {
    for(auto s : subscribers){
      s->update();
    }
  }

  void Prism::setMaxZ(double value){
    max_z_ = value < min_z_ ? min_z_ : value;
    notifySubscribers();
  }

  void Prism::setMinZ(double value){
    min_z_ = value > max_z_ ? max_z_: value;
    notifySubscribers();
  }

  bool Prism::setVertex(Point2d vertex, unsigned int index) {
    auto &outer_ring = polygon_.outer();
    if(index >= outer_ring.size() - 1) { // -1 because the last is the same as the first
      return false;
    }

    Point2d tmp = outer_ring[index];
    outer_ring[index] = vertex;
    if(index == 0) {
      outer_ring.back() = vertex;
    }

    if(!bg::is_valid(polygon_)){
      outer_ring[index] = tmp;
      if(index == 0){
        outer_ring.back() = tmp;
      }
      return false;
    }

    notifySubscribers();
    return true;
  }

  // TODO: test this
  bool Prism::setVertices(std::vector<Point2d>& vertices, std::vector<unsigned int> indices){
    if(vertices.size() != indices.size()){
      throw std::invalid_argument("Number of vertices and indices must be equal");
    }
    
    Polygon backup = polygon_;
    auto &outer_ring = polygon_.outer();
    bool success = true;
    for(int i=0; i<vertices.size(); i++){
      Point2d vertex = vertices[i];
      unsigned int index = indices[i];
      if(index >= outer_ring.size() - 1) { // -1 because the last is the same as the first
        success = false;
        break;
      }

      outer_ring[index] = vertex;
      if(index == 0) {
        outer_ring.back() = vertex;
      }
    }

    if(!bg::is_valid(polygon_)){
      success = false;
    }

    if(success){
      notifySubscribers();
    }else{
      polygon_ = backup;
    }
    return success;
  }

  void Prism::addVertexCounterclockwise(unsigned int index) {
    auto &outer_ring = polygon_.outer();
    if(index >= outer_ring.size() - 1) { // -1 because the last is the same as the first
      throw std::invalid_argument("Index is out of bounds");
    }

    unsigned int prev_index = index == 0 ? outer_ring.size() - 2 : index - 1;
    double x1 = bg::get<0>(outer_ring[prev_index]);
    double y1 = bg::get<1>(outer_ring[prev_index]);
    double x2 = bg::get<0>(outer_ring[index]);
    double y2 = bg::get<1>(outer_ring[index]);

    Point2d new_point;
    bg::set<0>(new_point, (x1 + x2) / 2);
    bg::set<1>(new_point, (y1 + y2) / 2);

    outer_ring.insert(outer_ring.begin() + prev_index + 1, new_point);
    notifySubscribers();
  }

  void Prism::addVertexClockwise(unsigned int index) {
    auto &outer_ring = polygon_.outer();
    if(index >= outer_ring.size() - 1) { // -1 because the last is the same as the first
      throw std::invalid_argument("Index is out of bounds");
    }

    double x1 = bg::get<0>(outer_ring[index]);
    double y1 = bg::get<1>(outer_ring[index]);
    double x2 = bg::get<0>(outer_ring[index + 1]);
    double y2 = bg::get<1>(outer_ring[index + 1]);

    Point2d new_point;
    bg::set<0>(new_point, (x1 + x2) / 2);
    bg::set<1>(new_point, (y1 + y2) / 2); 

    outer_ring.insert(outer_ring.begin() + index + 1, new_point);
    notifySubscribers();
  }

  void Prism::deleteVertex(unsigned int index){
    auto &outer_ring = polygon_.outer();
    if(index >= outer_ring.size() - 1) { // -1 because the last is the same as the first
      throw std::invalid_argument("Index is out of bounds");
    }
    outer_ring.erase(outer_ring.begin() + index);
    if(index == 0){
      outer_ring.back() = outer_ring[0];
    }
    notifySubscribers();
  }

  void Prism::move(Point3d adjustment) {
    bool do_notify = false;

    double dz = adjustment.get<2>();
    if(dz != 0){
      do_notify = true;
      max_z_ += dz;
      min_z_ += dz;
    }

    double dx = adjustment.get<0>();
    double dy = adjustment.get<1>();
    Point2d adjustment2d = Point2d{dx, dy};
    if(dx != 0 || dy != 0){
      do_notify = true;
      auto& outer_ring = polygon_.outer();
      for(int i=0; i<outer_ring.size(); i++){
        bg::add_point(outer_ring[i], adjustment2d);
      }
    }  

    if(do_notify){
      notifySubscribers();
    }
  }

  void Prism::rotate(double alpha){
    if(alpha == 0.0){
      return;
    }

    Point2d cur_center = getCenter();
    auto& outer_ring = polygon_.outer();
    for(int i=0; i<outer_ring.size(); i++){
      double x1 = outer_ring[i].get<0>();
      double y1 = outer_ring[i].get<1>();
      double x2 = (x1-cur_center.get<0>())*cos(alpha) - (y1-cur_center.get<1>())*sin(alpha) + cur_center.get<0>();
      double y2 = (x1-cur_center.get<0>())*sin(alpha) + (y1-cur_center.get<1>())*cos(alpha) + cur_center.get<1>();
      outer_ring[i].set<0>(x2);
      outer_ring[i].set<1>(y2);
    }

    notifySubscribers();
  }

  bool Prism::isPointIn(Point3d point) {
    Point2d point2d;
    bg::set<0>(point2d, bg::get<0>(point));
    bg::set<1>(point2d, bg::get<1>(point));
    double z = bg::get<2>(point);

    bool result = bg::within(point2d, polygon_) && min_z_ < z && z < max_z_;

    return result;
  }

  bool Prism::isPointIn(double x, double y, double z) {
    Point3d point;
    bg::set<0>(point, x);
    bg::set<1>(point, y);
    bg::set<2>(point, z);

    return isPointIn(point);
  }

  bool Prism::isPointIn(Point2d point) {
    return bg::within(point, polygon_);
  }

  bool Prism::isPointIn(double x, double y) {
    Point2d point;
    bg::set<0>(point, x);
    bg::set<1>(point, y);

    return bg::within(point, polygon_);
  }

} // namespace mrs_lib