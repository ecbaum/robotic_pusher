#include "ros/ros.h"
#include <string>
#include <iostream>
#include "robotic_pusher/getColor.h"
#include "robotic_pusher/getWeightType.h"
#include <rosprolog/rosprolog_client/PrologClient.h>

using namespace std;

// Explore possible queries
    // rdf_has(S, rdfs:subPropertyOf, cube:'color').
    // rdf_has(S, rdfs:domain, cube:'goldCube').
    // owl_subclass_of(A, cube:'cube').

string color;

bool get_weight_type(robotic_pusher::getWeightType::Request &req, robotic_pusher::getWeightType::Response &res)
{
    PrologClient pl = PrologClient("/rosprolog",true);
    string object;
    string weightClass;

    // Example Query: ?- rdf_has(cube:'gold', rdfs:domain, Object).
    PrologQuery classbdgs = pl.query("rdf_has(cube:'"+color+"', rdfs:domain, Object)");
    PrologQuery::iterator it=classbdgs.begin();
    PrologBindings bdg = *it;
    object = bdg["Object"].toString();
    cout << "Object = "<< object << endl;

    // Example Query: ?- rdf_has(WeightClass, rdfs:domain, cube:'goldCube').
    PrologQuery weightbdgs = pl.query("rdf_has(WeightClass, rdfs:domain, cube:'"+object+"')");
    it=weightbdgs.begin();
    it++; // Skip first entry as this is the color property
    bdg = *it;
    weightClass = bdg["WeightClass"].toString();
    cout << "Weight type (heavy, medium or light) = "<< weightClass << endl;
    res.weight_type = weightClass;

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
        ROS_INFO("Color I got is "+srv.response.object_color);
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