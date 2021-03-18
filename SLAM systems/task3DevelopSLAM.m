clear all;
% This script can be used to develop your SLAM system. It runs a "little
% map" which should be a lot faster to work with

% Configure to disable unneed sensors
parameters = minislam.event_generators.simulation.Parameters();
parameters.enableGPS = false; %%%for this question there is no gps

% Set up the simulator and the output
simulator = minislam.event_generators.simulation.Simulator(parameters);

% Create and run the different localization systems
g2oSLAMSystem = minislam.slam.g2o.G2OSLAMSystem();
%performKeyFrame_state = 2; 
%FirstEdgeFlag = true; 
%g2oSLAMSystem.setKeyFrameFlags(performKeyFrame_state,FirstEdgeFlag); %%%Need to move this to 4;
results = minislam.mainLoop(simulator, g2oSLAMSystem);

% You will need to add your analysis code here
graphInfo = g2oSLAMSystem.getGraphInformation();

%%%Need to do this as well,This includes the number of landmarks initialized, 
%%%the number of vehicle poses stored, and the average number of observations
%%%made by a robot at each timestep, and the average number of observations received by a
%%%landmark.

% Here's how to print out the number of edges and vertices
g2oGraph = g2oSLAMSystem.optimizer();

numVertices = length(g2oGraph.vertices());
numEdges = length(g2oGraph.edges());
