#include "ros/ros.h"
#include <string>
#include <iostream>
#include "robotic_pusher/getColor.h"
#include "robotic_pusher/getWeightType.h"
#include <rosprolog/rosprolog_client/PrologClient.h>

using namespace std;

/* 
This node takes the color of color_extractor_node.py, creates an instance in the ontology
and gives the weight class and color to the get_action_node.cpp
*/ 

// Explore possible queries
    // rdf_has(S, rdfs:subPropertyOf, cube:'color').
    // rdf_has(S, rdfs:domain, cube:'goldCube').
    // rdf_has(cube:'gold', rdfs:domain, Object)
    // rdf_has(WeightClass, rdfs:domain, cube:'goldCube')..
    // owl_subclass_of(A, cube:'cube').
    // owl_subclass_of(cube:'gold', A).

string color;

bool get_weight_type(robotic_pusher::getWeightType::Request &req, robotic_pusher::getWeightType::Response &res)
{
    PrologClient pl = PrologClient("/rosprolog",true);
    string object;
    string weightClass;

    // Example Query: ?- owl_subclass_of(cube:'gold', A).
    PrologQuery classbdgs = pl.query("owl_subclass_of(cube:'"+color+"', WeightClass)");
    PrologQuery::iterator it=classbdgs.begin();
    PrologBindings bdg = *it;
    weightClass = bdg["WeightClass"].toString();
    cout << "WeightClass (heavy, medium or light) = "<< weightClass << endl;
    res.weight_type = weightClass;
    res.object_color = color;

    return true;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "get_weight_node");

    // Client to get the color from get_color_node
    ros::NodeHandle n;
    ros::ServiceClient colorclient = n.serviceClient<robotic_pusher::getColor>("color_service");
    robotic_pusher::getColor srv;
    srv.request.get_color = true;
    if (colorclient.call(srv))
    {
        ROS_INFO("Color I got is %s", srv.response.object_color.c_str());
        color = srv.response.object_color;
    }
    else
    {
        ROS_ERROR("Failed to get drink id");
        return 1;
    }

    // Service that returns the weight class of the object corresponding to the color.
    ROS_INFO("Ready to get the weight class.");
    ros::ServiceServer service=n.advertiseService("weight_type_service",get_weight_type);
    ros::spin();

    return 0;
}