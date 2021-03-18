classdef Results < handle
    
    % This structure stores potentially useful information.
    
    properties(Access = public)
    
        % For each time step, time required to run the optimization. Is NaN
        % when no optimization was run.
        optimizationTimes; %%% so this is what is plotted, the time taken
        
        % For each time step, the ground truth of the vehicle
        vehicleTrueStateTime;  %%% Time is been predicted ooo, trouble
        vehicleTrueStateHistory;
        
        % For each time step, the predicted pose
        vehicleStateTime; %%%Maybe the time at each timeStep    
        vehicleStateHistory; %%%All the state history
        vehicleCovarianceHistory; %%% All the covariance history
        
    end
    
end