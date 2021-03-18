function results = mainLoop(eventGenerator, localizationSystems)

% This function runs the main loop for the MiniSLAM system. It sets up the
% event generator and the localization systems. It sets up the graphics. It
% runs the main loop and collects results.

% Set up the graphics for output
graphicalOutput = minislam.graphics.GraphicalOutput(); %%% some form of graphics maybe
graphicalOutput.initialize(eventGenerator, localizationSystems); 
%%% THe simulator output is the graphical output, it is initialized with
%%% eventgenerator and localization systems, which means it usually shows
%%% two sets of result, one for kalmann, one for g2o, I am guessing the
%%% covariances, blue and red circle, should be g20 vs kalmann covariance?

% Helper to make passing arguments easier
if (iscell(localizationSystems) == false)
    localizationSystems = {localizationSystems};
end

% Get the number of localization systems
numLocalizationSystems = length(localizationSystems); %%%numLocalizationSystems usually 2

% Allocate the results structure
results = cell(numLocalizationSystems, 1); 
%%% Results  store the answer, it is a cell 2x1
%%% Note: results are the output of mainloop
%%% Results has 6 fields: 
%%% optimizationTimes, for plotting how long optimization took if you optimize every step
%%% vehicleTrueStateTime, The time for the true state I suspec
%%% vehicleTrueStateHistory, all the true states for the motion
%%% vehicleStateTime, predicted state time, not sure how this works
%%% vehicleStateHistory, all predicted states
%%% vehicleCovarianceHistory, all predicted covariances
for l = 1 : numLocalizationSystems
    results{l} = minislam.Results();
end

% Start the event generator
eventGenerator.start(); %%% Remember, it is either one of 4 events been generated
%%% Each of the event contains stuff

storeCount = 0;

while (eventGenerator.keepRunning() == true)
    
    % Get the step number
    storeCount = eventGenerator.stepCount() + 1;
    
    % Print out
    if (rem(storeCount, 100) == 0)
        disp(num2str(storeCount)) %%% this is the display that occurs when we run
    end
    
    % Get the events and generate the events
    events = eventGenerator.events();
    %%% I still cant find the place where events are gotten in event
    %%% generator, it seems to be an abstract class implemented elsewhere
    
    % Log the ground truth
    groundTruthState = eventGenerator.groundTruth(false);
    
    for l = 1 : numLocalizationSystems
        localizationSystem = localizationSystems{l};    
        localizationSystem.processEvents(events); %%% process events
        runOptimizer = localizationSystem.recommendOptimization();
        %%%runOptimizer is a true or false boolean which optimizes the
        %%%graph after ever time step
        if (runOptimizer == true)
            tic
            localizationSystem.optimize(); 
            %%%For question 1, no .optimize() going on for kalmann whihch
            %%%is the only one run_optimizer is set to true, so why is
            %%%there a graph? Maybe that is the reason it is varying,
            %%%depends on systems
            results{l}.optimizationTimes(storeCount) = toc;
        else
            results{l}.optimizationTimes(storeCount) = NaN;
        end
        
        % Store ground truth in each results structure
        results{l}.vehicleTrueStateTime(storeCount) = eventGenerator.time();
        results{l}.vehicleTrueStateHistory(:, storeCount) = groundTruthState.xTrue;
        %%% storing the groundTruth for each localizationsystems
    end
    
    % Draw the graphics. Note this draws once every third frame which is very smooth but
    % can be slow. The graphics only update when "true" is passed in.
    if (runOptimizer == true)%if (((stepNumber - lastUpdateStepNumber) > 10) && (runOptimizer == true))
        graphicalOutput.update();
        %%% I am not entirely sure how this works, for Q1, when I thinker
        %%% with it, I get errors, about cameras or marginals
    %    lastUpdateStepNumber = stepNumber;
    end
    
    eventGenerator.step();
end

% Run the optimizer
for l = 1 : numLocalizationSystems
    localizationSystems{l}.optimize(20);
    [T, X, P] = localizationSystems{l}.robotEstimateHistory();
    results{l}.vehicleStateTime = T;
    results{l}.vehicleStateHistory = X;
    results{l}.vehicleCovarianceHistory = P;
end

graphicalOutput.update();


end