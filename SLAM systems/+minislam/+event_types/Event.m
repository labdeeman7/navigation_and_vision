% This class stores an event from the scnario generator. The known
% even types are: %%% So scenario generator generates the events
% %%% events contain specification and data, like messages in ROS
%
% InitialConditionEvent:
% This is a one-off event sent at the start. The data is
% the initial position and orientation of the vehicle. This is known
% perfectly and so the covariance is a zero matrix. %%% Known perfectly
% %%% Initial covariance is zero, they are talking about process covariance
%
% VehicleOdometryEvent: 
% This specifies the drive speed and angular speed of the robot. The
% covariance specifies Q_k. %%% data - drive speed, angular speed,
% %%% covariance specifies Q_k, this is for u and ucov I think or.  
% %%% u and ucov may be meant to handle all values which I doubt
%
% GPSObservationEvent:
% This specifies a GPS measurement of vehicle (x,y). The covariance
% is R^GPS_k. %%% data - x,y,R(cov)_k. i think R_k is constant
%
% LandmarkObservationEvent:
% This specifies the range, attitude and elevation measurements of the
% landmarks. The event also contains the IDs of the detected landmarks.
% The observations are in an 3xn array, where n is the number of landmarks.
%%% I do not understand it perfectly, I do know that the 3 values needs to be
%%% turned to range and angle(beta) via calculations somehow 

classdef Event < handle

    % Enumeration of observation types
    properties(Access = public, Constant = true)
        INITIAL_CONDITION = 0;
        VEHICLE_ODOMETRY = 1;
        GPS = 2;
        LANDMARK = 3;
    end
    
    properties(Access = public)
        %%%All these properties can probably be assesed via event.time etc
        % The time of the event.
        time; 
        
        % The type of the event. Must be one of the enums listed above.
        type;
        
        % The event data
        data;
        
        % The noise on the event data
        covariance;
    end
    
    methods(Access = public)
        function this = Event(time, type, data, covariance)
            
            % Copy over the common values
            this.time = time;
            this.type = type;
            this.data = data;
            
            % If the noise is a vector, assume that it encodes a diagonal covariance
            % matrix. Therfore, reshape into a matrix.
            if ((size(covariance, 1) == 1) || (size(covariance, 2) == 1))
                covariance = diag(covariance);
            end            
            this.covariance = covariance;
        end
    end
    
end
