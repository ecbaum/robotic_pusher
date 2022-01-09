#include "robotic_pusher/calibrateColor.h"
#include "robotic_pusher/getColor.h"
#include "robotic_pusher/getWeightType.h"
#include "ros/ros.h"
#include <iostream>
#include <rosprolog/rosprolog_client/PrologClient.h>
#include <string>

using namespace std;

/********
This node takes the color of color_extractor_node.py, gets the corresponding
weight class and returns the weight class and color to the get_action_node.cpp
********/

// Explore possible queries
// rdf_has(S, rdfs:subPropertyOf, cube:'color').
// rdf_has(S, rdfs:domain, cube:'goldCube').
// rdf_has(cube:'gold', rdfs:domain, Object)
// rdf_has(WeightClass, rdfs:domain, cube:'goldCube')..
// owl_subclass_of(A, cube:'cube').
// owl_subclass_of(cube:'gold', A).

string color;
bool b_full_color_search = true; // Color detection works better when true
bool first = true;

bool get_weight_type(robotic_pusher::getWeightType::Request &req,
                     robotic_pusher::getWeightType::Response &res) {
    
    // Client to get the color from get_color_node
    ros::NodeHandle n;

    if (b_full_color_search){
    ros::ServiceClient calibrateclient =
    n.serviceClient<robotic_pusher::getColor>("robotic_pusher/full_color_search");
    robotic_pusher::getColor srv0;
    srv0.request.get_color = true;
    if (calibrateclient.call(srv0)) {
        ROS_INFO("Full color search gives color %s", srv0.response.object_color.c_str());
        color = srv0.response.object_color;
    } else {
        ROS_ERROR("Failed to do full color search");
        return 1;
    }
    }
    else{
        // Do calibration only once
        if(first){
            ros::ServiceClient calibrateclient =
            n.serviceClient<robotic_pusher::calibrateColor>("robotic_pusher/calibrate_color");
            robotic_pusher::calibrateColor srv1;
            srv1.request.calibrate = true;
            if (calibrateclient.call(srv1)) {
                ROS_INFO("Camera calibration successful");
            } else {
                ROS_ERROR("Failed to calibrate color");
                return 1;
            }
            first = false;
        }

        // Wait task to be finished
        ros::Duration(1).sleep();
        
        ros::ServiceClient colorclient =
        n.serviceClient<robotic_pusher::getColor>("robotic_pusher/get_color");
        robotic_pusher::getColor srv2;
        srv2.request.get_color = true;
        if (colorclient.call(srv2)) {
            ROS_INFO("Color I got is %s", srv2.response.object_color.c_str());
            color = srv2.response.object_color;
        } else {
            ROS_ERROR("Failed to get color of Object");
            return 1;
        }
    }
    
    PrologClient pl = PrologClient("/rosprolog", true);
    string object;
    string weightClass;

    // Example Query: ?- owl_subclass_of(cube_ontology:'gold', A).
    PrologQuery classbdgs =
        pl.query("owl_subclass_of(cube_ontology:'" + color + "', WeightClass)");
    PrologQuery::iterator it = classbdgs.begin();
    it++;
    it++;
    PrologBindings bdg = *it;
    weightClass = bdg["WeightClass"].toString();

    // Remove the syntax in front of the output
    string substring = "http://www.semanticweb.org/janmorlock/ontologies/2021/11/untitled-ontology-11#";
    std::size_t pos = weightClass.find(substring); // Find the starting index of substring in the string, else it returns std::string::npos
    if(pos !=std::string::npos)
        weightClass.erase(pos,substring.length());

    cout << "WeightClass (heavy, medium or light) = " << weightClass << endl;
    res.weight_type = weightClass;
    res.object_color = color;

    return true;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "get_weight_node");

  ros::NodeHandle n;

  // Service that returns the weight class of the object corresponding to the
  // color.
  ROS_INFO("Ready to get the weight class.");
  ros::ServiceServer service =
      n.advertiseService("robotic_pusher/weight_type", get_weight_type);
  ros::spin();

  return 0;
}