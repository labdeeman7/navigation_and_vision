% This class implements an event-based estimation system for building up a
% minimal, ideal SLAM system. The system is event-based and responds to a
% sequence of events which are time stamped and served in order.

classdef VehicleSLAMSystem < handle %%%handle is a default super class, allows for events and more
        
    properties(Access = protected)
        
        % Step number - how many times have we iterated?
        stepNumber;
        
        % The last time an event was processed.
        currentTime;
        
        % The vehicle control inputs. These are wheel speed and steer
        % angle. These are assumed to be held constant until the next
        % control input event comes along.
        u; %%% control input
        uCov; %%% I think it is process noise
                
        % Flag to show if debugging is enabled
        debug = true;
        
        initialized;
        
        % Temporary!
        %parameters;
        
    end
       
    methods(Access = public)
        
        % Create the localization system and start it up.
        function this = VehicleSLAMSystem() %%%This is the initialization
            
            % Set the start time to 0.
            this.currentTime = 0;
            
            % Set the current step number to zero
            this.stepNumber = 0;
            
            % Default odometry. This will stay the same until new
            % measurements are provided.
            this.u = zeros(3, 0); %%%I have no idea why this is 3,0 it gives an empty matrix 
                                  %%%this.u control inputs, x_k, y_k=0 and sigma_k (angle). 
            this.uCov = zeros(3, 3);  %%% Cant exactly remember what uCov is right now          
            
            this.initialized = false;  %%% Take note of initialized, when we do the _init_ we set
                                       %%% Initialized to false. 
        end
        
        % Process a cell array which is a sequence of events. Each event is
        % processed in the order it appears in the array.
        function processEvents(this, eventQueue) %%%There is an eventQueue
            
            % Increment the step number
            this.stepNumber = this.stepNumber + 1;  %%%step number is protected property
           
            events = eventQueue.events(); %%%check the eventQueue class for more information
            
            % Get the next event
            for k = 1 : length(events)                
                event = events{k};  %%% I need to check events

                % First check it's of the right class.
                assert(isa(event, 'minislam.event_types.Event'));%%%There are 4 types of events
                %%%GPSObservation, initialCondition, Landmarkobservation, VehicleOdometryEvent   
                
                % Now do all the actual work
                this.processEvent(event); %%%note this is different from processEvents
            end
        end
        
        function processEvent(this, event) %%%to process a single event

            % If initialized, predict if the timestep length is
            % sufficiently long
            if (this.initialized == true)
                dT = event.time() - this.currentTime; %%% Find difference between time

                % Nothing to do if it's really close to the last time or we
                % have no odometry
                if (abs(dT) < 1e-3)
                    this.handleNoPrediction(); %%% This is implemented in the child classes,
                    %%% handleNoPrediction occurs when there is not enough
                    %%% time difference between newtime and old time
                else
                    this.handlePredictToTime(event.time(), dT);  %%%If there is predict
                    %%%Remeber this can be done either via kalmann or via
                    %%%g20 method
                end
            end

            this.currentTime = event.time; %%% Update time after predicitng

            % Handle the event on the basis of its type. Note that if we
            % are not initialized yet, we can only handle the
            % odometry and initialization events.
            %%% very important note ontop, initialization, only odometry
            %%% and initial conditions allowed
            switch(event.type)
                
                case minislam.event_types.Event.INITIAL_CONDITION
                    assert(this.initialized == false)
                    this.handleInitialConditionEvent(event);
                    this.initialized = true;

                case minislam.event_types.Event.VEHICLE_ODOMETRY
                    this.handleVehicleOdometryEvent(event);
                     %%% Note that this is done in vehicle slam itself.
                     %%% if there is new odometry, it changes u and uCov
                     %%% Remeber that we assume they u and uCov constant
                     %%% until new values arrive
                    
                case minislam.event_types.Event.GPS
                    if (this.initialized == true)
                        this.handleGPSObservationEvent(event); 
                       
                    end

                case minislam.event_types.Event.LANDMARK
                    if (this.initialized == true)
                        this.handleLandmarkObservationEvent(event);
                    end
                    
                otherwise
                    error('Unknown observation type %d', event.type)     
            end            
        end
    end
        
    methods(Access = protected)
        
        % Handle a new odometry event. We simply change the vehicle
        % odometry to the new value.
        function handleVehicleOdometryEvent(this, event)
            this.u = event.data;
            this.uCov = event.covariance;
        end
    end
    
    methods(Access = public, Abstract)
        
        % Get the mean and covariance of the estimate at the current time.
        % This is used for output purposes.
        [x, P] = robotEstimate(this);
        
        % Get the mean and covariance history of the robot across the whole
        % run. This is used for analysis purposes.
        [T, X, PX] = robotEstimateHistory(this);
        
        % Get the current landmarks estimates.
        [x, P, landmarkIds] = landmarkEstimates(this);
        
        % Recommend if an optimization is required
        recommendation = recommendOptimization(this);
        
        % Optimize
        optimize(this, maximumNumberOfOptimizationSteps);
        
    end
    
    methods(Access = protected, Abstract)
       
        % Handle a GPS measurement
        handleInitialConditionEvent(this, event);
                
        % Handle not predicting
        handleNoPrediction(this);
        
        % Handle everything needed to predict to the current state.
        handlePredictToTime(this, time);
 
        % Handle a GPS measurement
        handleGPSObservationEvent(this, event);
            
        % Handle a set of laser measurements
        handleLandmarkObservationEvent(this, event);
    end
end
