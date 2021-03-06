:- register_ros_package(knowrob_maps).
:- register_ros_package(knowrob_actions).
:- register_ros_package(knowrob_common).


:- consult('myComputable_utils.pl').


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% parse OWL files, register name spaces
:- owl_parser:owl_parse('package://robotic_pusher/owl/cube_ontology.owl').
:- rdf_db:rdf_register_ns(cube_ontology, 'http://www.semanticweb.org/janmorlock/ontologies/2021/11/untitled-ontology-11#', [keep(true)]).