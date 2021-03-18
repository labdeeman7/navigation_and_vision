% This script can be used to compare your SLAM system

% Configure to disable other sensor types
parameters = minislam.event_generators.simulation.Parameters(); 


% Magic tuning for the no-prediction case
parameters.laserDetectionRange = 20; 

% By setting true / false you can enable different combinations of sensors
parameters.enableGPS = true; 

parameters.enableLaser = true; 
parameters.gpsMeasurementPeriod = 1;  

% Set up the simulator and the output
simulator = minislam.event_generators.simulation.Simulator(parameters, 'task3');

% Create and run the different localization systems
g2oSLAMSystem = minislam.slam.g2o.G2OSLAMSystem();
QUEST_4_1 = 0;
QUEST_4_2 = 1;
QUEST_4_3 = 2;
performKeyFrame_state = QUEST_4_1; 
KeepFirstEdgeFlag = false; 
g2oSLAMSystem.setKeyFrameFlags(performKeyFrame_state,KeepFirstEdgeFlag);

results = minislam.mainLoop(simulator, g2oSLAMSystem);


% Plot the error curves
minislam.graphics.FigureManager.getFigure('Errors');
clf
plot(results{1}.vehicleStateHistory'-results{1}.vehicleStateHistory')

% Plot covariance
minislam.graphics.FigureManager.getFigure('Vehicle Covariances');
clf
plot(results{1}.vehicleCovarianceHistory')

% Plot errors
minislam.graphics.FigureManager.getFigure('Errors');
clf
plot(results{1}.vehicleStateHistory'-results{1}.vehicleTrueStateHistory')
