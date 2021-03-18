clear all;
% This script runs the odometry 
%%%What does simon mean by run the odometry?

% Configure to disable other sensor types
parameters = minislam.event_generators.simulation.Parameters();
%%%so parameters is a class that gives public access to properties, he can
%%%change some values this way, I best steal this method.
parameters.enableGPS = false;
parameters.enableLaser = false;
%%%For Question 1, no GPS or lasers

% If you set this parameter to false, the simulator generates measurements
% with no noise in them. You might find this useful for debugging.
parameters.perturbWithNoise = false; 
%%% ok I am gonna try that and see, - observation: need to figure out what
%%% The graphs mean before I can tinker with that

% Set up the simulator and the output
simulator = minislam.event_generators.simulation.Simulator(parameters);
%%% There are two ellipses in simulator, what are they? how is the
%%% simulator output accessed?

% Create and run the different localization systems
kalmanFilterSLAMSystem = minislam.slam.kalman.KalmanFilterSLAMSystem();
g2oSLAMSystem = minislam.slam.g2o.G2OSLAMSystem();
results = minislam.mainLoop(simulator, {kalmanFilterSLAMSystem, g2oSLAMSystem});
%%% Simulator generates the events, as well as an output.
%%%results{1} refers to kalmann while results{2} refers to g2o.

% Plot optimisation times
minislam.graphics.FigureManager.getFigure('Optimization times');
clf
plot(results{1}.optimizationTimes)
hold on
plot(results{2}.optimizationTimes,'+')
legend({'Kalmann','g2o'}) %%% My code

% Plot the error curves
minislam.graphics.FigureManager.getFigure('Errors');
clf
plot(results{1}.vehicleStateHistory'-results{1}.vehicleStateHistory')
%%% why is he subtracting the same thing from itself?

% Plot covariance
minislam.graphics.FigureManager.getFigure('Vehicle Covariances');
clf
plot(results{1}.vehicleCovarianceHistory')
hold on
plot(results{2}.vehicleCovarianceHistory', '--')
%%%hold on a minute, we are plotting three different things here, cov for
%%%x_K, y_k and sigma_k, the same applies for state as well
legend({'Kalmann x', 'Kalmann y', 'Kalmann sigma', 'g2o x', 'g2o y' 'g2o sigma' })

% Plot errors
minislam.graphics.FigureManager.getFigure('Errors');
clf
plot(results{1}.vehicleStateHistory'-results{1}.vehicleTrueStateHistory')
hold on
plot(results{2}.vehicleStateHistory'-results{2}.vehicleTrueStateHistory','--')
legend({'Kalmann x', 'Kalmann y', 'Kalmann sigma', 'g2o x', 'g2o y' 'g2o sigma' })

%%%I am assuming the plots are majorly correct, and the covariance ellipse
%%%are the covariance for g2o red and blue is for kalmann, obviously g2o is
%%%better
