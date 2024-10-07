#include "mrs_lib/safety_zone/safety_zone.h"
#include "mrs_lib/safety_zone/vertex_control.h"
#include "mrs_lib/safety_zone/bounds_control.h"
#include "mrs_lib/safety_zone/int_edges_visualization.h"
#include "mrs_lib/safety_zone/static_edges_visualization.h"
#include "mrs_lib/safety_zone/center_control.h"
#include "mrs_lib/safety_zone/yaml_export_visitor.h"
#include <ros/ros.h>

#include <iostream>
#include <sstream>

namespace bg = boost::geometry;
using namespace mrs_lib;

void basic_validation(){
    std::cout<< std::endl << "Test: basic validation" << std::endl;
    Point2d point1 = Point2d{0, 0};
    Point2d point2 = Point2d{0, 10};
    Point2d point3 = Point2d{10, 10};
    Point2d point4 = Point2d{10, 0};
    std::vector<Point2d> poly = std::vector<Point2d>{point1, point2, point3, point4};

    Prism outer_boarder = Prism(poly, 0.0, 15);

    SafetyZone safety_zone = SafetyZone(outer_boarder);

    bool passed = true;
    if(!safety_zone.isPointValid(Point2d{1, 1})){
        passed = false;
        std::cout<< "[basic_validation]: ERROR: Point (1, 1) must be valid!\n";
    }

    if(safety_zone.isPointValid(Point2d{-1, -1})){
        passed = false;
        std::cout<< "[basic_validation]: ERROR: Point (-1, -1) must be invalid!\n";
    }

    if(passed){
        std::cout << "[basic_validation]: Test passed\n";
    }
}

void vertex_add_counter_clockwise(){
    Point2d point1 = Point2d{0, 0};
    Point2d point2 = Point2d{0, 10};
    Point2d point3 = Point2d{10, 10};
    Point2d point4 = Point2d{10, 0};
    Point2d point5 = Point2d{0, 0};
    std::vector<Point2d> poly = std::vector<Point2d>{point1, point2, point3, point4};

    Prism outer_boarder = Prism(poly, 0.0, 15);
    outer_boarder.addVertexCounterclockwise(1);

    SafetyZone safety_zone = SafetyZone(outer_boarder);

    Prism* new_outer_boarder = safety_zone.getBorder();
    Polygon new_poly = new_outer_boarder->getPolygon();

    bool passed = true;

    passed = passed && (new_poly.outer()[0].get<0>() == point1.get<0>());
    passed = passed && (new_poly.outer()[1].get<0>() == 0);
    passed = passed && (new_poly.outer()[2].get<0>() == point2.get<0>());
    passed = passed && (new_poly.outer()[3].get<0>() == point3.get<0>());
    passed = passed && (new_poly.outer()[4].get<0>() == point4.get<0>());
    passed = passed && (new_poly.outer()[5].get<0>() == point5.get<0>());

    passed = passed && (new_poly.outer()[0].get<1>() == point1.get<1>());
    passed = passed && (new_poly.outer()[1].get<1>() == 5);
    passed = passed && (new_poly.outer()[2].get<1>() == point2.get<1>());
    passed = passed && (new_poly.outer()[3].get<1>() == point3.get<1>());
    passed = passed && (new_poly.outer()[4].get<1>() == point4.get<1>());
    passed = passed && (new_poly.outer()[5].get<1>() == point5.get<1>());

    if(passed){
        std::cout<< "[prism_changed_in_advance_counter_clockwise]: Test passed\n";
    } else{
        std::cout << "[prism_changed_in_advance_counter_clockwise]\n";
        std::cout << "Actual:   " << bg::dsv(new_poly) << std::endl;
        std::cout << "Expected: " << "(((0, 0), (0, 5), (0, 10), (10, 10), (10, 0), (0, 0)))" << std::endl;
        std::cout<< "[prism_changed_in_advance_counter_clockwise]: Test did not pass\n";
    }
}

void vertex_add_clockwise() {
    Point2d point1 = Point2d{0, 0};
    Point2d point2 = Point2d{0, 10};
    Point2d point3 = Point2d{10, 10};
    Point2d point4 = Point2d{10, 0};
    Point2d point5 = Point2d{0, 0};
    std::vector<Point2d> poly = std::vector<Point2d>{point1, point2, point3, point4};

    Prism outer_boarder = Prism(poly, 0.0, 15);
    outer_boarder.addVertexClockwise(1);

    SafetyZone safety_zone = SafetyZone(outer_boarder);

    Prism* new_outer_boarder = safety_zone.getBorder();
    Polygon new_poly = new_outer_boarder->getPolygon();

    bool passed = true;

    passed = passed && (new_poly.outer()[0].get<0>() == point1.get<0>());
    passed = passed && (new_poly.outer()[1].get<0>() == point2.get<0>());
    passed = passed && (new_poly.outer()[2].get<0>() == 5);
    passed = passed && (new_poly.outer()[3].get<0>() == point3.get<0>());
    passed = passed && (new_poly.outer()[4].get<0>() == point4.get<0>());
    passed = passed && (new_poly.outer()[5].get<0>() == point5.get<0>());

    passed = passed && (new_poly.outer()[0].get<1>() == point1.get<1>());
    passed = passed && (new_poly.outer()[1].get<1>() == point2.get<1>());
    passed = passed && (new_poly.outer()[2].get<1>() == 10);
    passed = passed && (new_poly.outer()[3].get<1>() == point3.get<1>());
    passed = passed && (new_poly.outer()[4].get<1>() == point4.get<1>());
    passed = passed && (new_poly.outer()[5].get<1>() == point5.get<1>());

    if(passed){
        std::cout<< "[prism_changed_in_advance_clockwise]: Test passed\n";
    } else{
        std::cout << "[prism_changed_in_advance_clockwise]\n";
        std::cout << "Actual:   " << bg::dsv(new_poly) << std::endl;
        std::cout << "Expected: " << "(((0, 0), (0, 10), (5, 10), (10, 10), (10, 0), (0, 0)))" << std::endl;
        std::cout<< "[prism_changed_in_advance_clockwise]: Test did not pass\n";
    }
}

void vertex_add_counter_clockwise_0() {
    Point2d point1 = Point2d{0, 0};
    Point2d point2 = Point2d{0, 10};
    Point2d point3 = Point2d{10, 10};
    Point2d point4 = Point2d{10, 0};
    Point2d point5 = Point2d{0, 0};
    std::vector<Point2d> poly = std::vector<Point2d>{point1, point2, point3, point4};

    Prism outer_boarder = Prism(poly, 0.0, 15);
    outer_boarder.addVertexCounterclockwise(0);

    SafetyZone safety_zone = SafetyZone(outer_boarder);

    Prism* new_outer_boarder = safety_zone.getBorder();
    Polygon new_poly = new_outer_boarder->getPolygon();

    bool passed = true;

    passed = passed && (new_poly.outer()[0].get<0>() == point1.get<0>());
    passed = passed && (new_poly.outer()[1].get<0>() == point2.get<0>());
    passed = passed && (new_poly.outer()[2].get<0>() == point3.get<0>());
    passed = passed && (new_poly.outer()[3].get<0>() == point4.get<0>());
    passed = passed && (new_poly.outer()[4].get<0>() == 5);
    passed = passed && (new_poly.outer()[5].get<0>() == point5.get<0>());

    passed = passed && (new_poly.outer()[0].get<1>() == point1.get<1>());
    passed = passed && (new_poly.outer()[1].get<1>() == point2.get<1>());
    passed = passed && (new_poly.outer()[2].get<1>() == point3.get<1>());
    passed = passed && (new_poly.outer()[3].get<1>() == point4.get<1>());
    passed = passed && (new_poly.outer()[4].get<1>() == 0);
    passed = passed && (new_poly.outer()[5].get<1>() == point5.get<1>());

    if(passed){
        std::cout<< "[prism_changed_in_advance_clockwise_0]: Test passed\n";
    } else{
        std::cout << "[prism_changed_in_advance_clockwise_0]\n";
        std::cout << "Actual:   " << bg::dsv(new_poly) << std::endl;
        std::cout << "Expected: " << "(((0, 0), (0, 10), (10, 10), (10, 0), (5, 0), (0, 0)))" << std::endl;
        std::cout<< "[prism_changed_in_advance_clockwise_0]: Test did not pass\n";
    }
}

void point_valid_after_vertex_change() {
    Point2d point1 = Point2d{0, 0};
    Point2d point2 = Point2d{0, 10};
    Point2d point3 = Point2d{10, 10};
    Point2d point4 = Point2d{10, 0};
    std::vector<Point2d> poly = std::vector<Point2d>{point1, point2, point3, point4};

    Prism outer_boarder = Prism(poly, 0.0, 15);

    SafetyZone safety_zone(outer_boarder);

    Prism* new_outer_boarder = safety_zone.getBorder();

    if(new_outer_boarder->setVertex(Point2d{0, 20}, 1)) {
        if(safety_zone.isPointValid(Point2d{1, 11})){
            std::cout << "[point_valid_after_vertex_change]: Test passed\n";
        }else {
            std::cout << "[point_valid_after_vertex_change]: Tesst did not pass \nThe point (1, 11) must be valid\n";
        }
    } else {
        std::cout << "[point_valid_after_vertex_change]: the vertex was not set\n";
    }
}

void invalid_vertex_assignment(){
    Point2d point1 = Point2d{0, 0};
    Point2d point2 = Point2d{0, 10};
    Point2d point3 = Point2d{10, 10};
    Point2d point4 = Point2d{10, 0};
    Point2d point5 = Point2d{0, 0};
    std::vector<Point2d> poly = std::vector<Point2d>{point1, point2, point3, point4};

    Prism outer_boarder = Prism(poly, 0.0, 15);

    SafetyZone safety_zone(outer_boarder);

    Prism* new_outer_boarder = safety_zone.getBorder();

    if(new_outer_boarder->setVertex(Point2d{12, 7}, 1)) {
        std::cout<<"[invalid_vertex_assignment]: Test did not pass\n";
        return;
    }

    Polygon new_poly = new_outer_boarder->getPolygon();

    bool passed = true;

    passed = passed && (new_poly.outer()[0].get<0>() == point1.get<0>());
    passed = passed && (new_poly.outer()[1].get<0>() == point2.get<0>());
    passed = passed && (new_poly.outer()[2].get<0>() == point3.get<0>());
    passed = passed && (new_poly.outer()[3].get<0>() == point4.get<0>());
    passed = passed && (new_poly.outer()[4].get<0>() == point5.get<0>());

    passed = passed && (new_poly.outer()[0].get<1>() == point1.get<1>());
    passed = passed && (new_poly.outer()[1].get<1>() == point2.get<1>());
    passed = passed && (new_poly.outer()[2].get<1>() == point3.get<1>());
    passed = passed && (new_poly.outer()[3].get<1>() == point4.get<1>());
    passed = passed && (new_poly.outer()[4].get<1>() == point5.get<1>());

    if(passed){
        std::cout<< "[invalid_vertex_assignment]: Test passed\n";
    } else{
        std::cout << "[invalid_vertex_assignment]\n";
        std::cout << "Actual:   " << bg::dsv(new_poly) << std::endl;
        std::cout << "Expected: " << "(((0, 0), (0, 10), (10, 10), (10, 0), (0, 0)))" << std::endl;
        std::cout<< "[invalid_vertex_assignment]: Test did not pass\n";
    }
}

void show_markers(ros::NodeHandle nh) {
    Point2d point1 = Point2d{0, 0};
    Point2d point2 = Point2d{0, 10};
    Point2d point3 = Point2d{10, 10};
    Point2d point4 = Point2d{10, 0};
    std::vector<Point2d> poly = std::vector<Point2d>{point1, point2, point3, point4};

    Prism outer_boarder = Prism(poly, 0.0, 15);

    StaticEdgesVisualization static_edges_vis = StaticEdgesVisualization(&outer_boarder, "map", nh, 2);
    IntEdgesVisualization edges_vis = IntEdgesVisualization(&outer_boarder, "map", nh);
    VertexControl vertex_control = VertexControl(&outer_boarder, "map", nh);
    BoundsControl bounds_control = BoundsControl(&outer_boarder, "map", nh);
    CenterControl center_control = CenterControl(&outer_boarder, "map", nh);

    Prism outer_boarder2 = Prism(poly, 16.0, 18.0);
    StaticEdgesVisualization edges_vis2 = StaticEdgesVisualization(&outer_boarder2, "map", nh, 2);
    IntEdgesVisualization edges_vis_int2 = IntEdgesVisualization(&outer_boarder2, "map", nh);
    VertexControl vertex_control2 = VertexControl(&outer_boarder2, "map", nh);
    BoundsControl bounds_control2 = BoundsControl(&outer_boarder2, "map", nh);
    CenterControl center_control2 = CenterControl(&outer_boarder2, "map", nh);
    
    ros::spin();
}

void load_params(ros::NodeHandle nh){

    XmlRpc::XmlRpcValue value;
    bool success = nh.getParam("safety_area/obstacles", value);

    std::cout << (success ? "success" : "not success =(" )<< std::endl;

    for(int i=0; i<value.size(); i++){
        std::cout << value[i].begin()->first << std::endl;
    }
}

void show_safety_zone(ros::NodeHandle nh) {
    Point2d point1 = Point2d{0, 0};
    Point2d point2 = Point2d{0, 10};
    Point2d point3 = Point2d{10, 10};
    Point2d point4 = Point2d{10, 0};
    std::vector<Point2d> poly = std::vector<Point2d>{point1, point2, point3, point4};

    Prism outer_boarder = Prism(poly, 0.0, 15);
    Prism obstacle = Prism(poly, 16.0, 18.0);

    std::vector<std::unique_ptr<Prism>> obstacles{&obstacle, &obstacle};
    SafetyZone safety_zone = SafetyZone(outer_boarder, obstacles);

    StaticEdgesVisualization static_edges_vis = StaticEdgesVisualization(&safety_zone, "map", nh, 2);
    IntEdgesVisualization edges_vis = IntEdgesVisualization(&safety_zone, "map", nh);
    VertexControl vertex_control = VertexControl(&safety_zone, "map", nh);
    BoundsControl bounds_control = BoundsControl(&safety_zone, "map", nh);
    CenterControl center_control = CenterControl(&safety_zone, "map", nh);

    int obstacle_id = safety_zone.getObstaclesBegin()->first;
    // std::cout << obstacle_id << std::endl;
    StaticEdgesVisualization edges_vis2 = StaticEdgesVisualization(&safety_zone, obstacle_id, "map", nh, 2);
    IntEdgesVisualization edges_vis_int2 = IntEdgesVisualization(&safety_zone, obstacle_id, "map", nh);
    VertexControl vertex_control2 = VertexControl(&safety_zone, obstacle_id, "map", nh);
    BoundsControl bounds_control2 = BoundsControl(&safety_zone, obstacle_id, "map", nh);
    CenterControl center_control2 = CenterControl(&safety_zone, obstacle_id, "map", nh);

    YamlExportVisitor visitor = YamlExportVisitor("", "map", "map", "map", "LATLON", 0.0, 0.0);
    safety_zone.accept(visitor);
    std::cout << visitor.getResult() << std::endl;
    
    ros::spin();
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "Safety_zone_test");
    ros::NodeHandle nh;

    basic_validation();
    vertex_add_counter_clockwise();
    vertex_add_clockwise();
    vertex_add_counter_clockwise_0();
    point_valid_after_vertex_change();
    invalid_vertex_assignment();

    // load_params(nh);

    // show_markers(nh);
    show_safety_zone(nh);
    return 0;
}
