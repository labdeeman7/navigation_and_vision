classdef KalmanFilterSLAMSystem < minislam.slam.VehicleSLAMSystem
    
    % This is a very minimal Kalman filter; its purpose is to let me do
    % some debugging and comparison. It is not fully featured.
    
    properties(Access = protected)
        
        % Kalman filter mean
        xEst; %%% I am guessing this is the current estimate of state(x_k, y_k, sigma_k)
        PEst; %%% I am guessing this is the current estimate of covariance
        
        % Kalman filter covariance
        xPred; %%% guessing this is the predicted state by kalmann filter
        PPred; %%% guessing this is the predicted covariance by kalmann filter
        
        % Store of the mean and covariance values for the vehicle
        timeStore; %%%stores time step  
        xEstStore; %%%stores current state estimate
        PEstStore; %%%stores diagonal of the covariance of process noise
    end
    
    methods(Access = public)
        
        function this = KalmanFilterSLAMSystem()
            this = this@minislam.slam.VehicleSLAMSystem();
            this.xEstStore = NaN(3, 1);
            this.PEstStore = NaN(3, 1);
            this.xEst = NaN(3, 1);
            this.PEst = NaN(3, 3);
        end
        
        function [x,P] = robotEstimate(this)
            x = this.xEst(1:3);
            P = this.PEst(1:3, 1:3);
        end
        
        function [T, X, PX] = robotEstimateHistory(this)
            T = this.timeStore;
            X = this.xEstStore;
            PX = this.PEstStore;
        end
        
        function [x, P, landmarkIds] = landmarkEstimates(this)
            landmarkIds = [];
            x = NaN(2, 0);
            P = NaN(2, 2, 0);
        end
        
        function recommendation = recommendOptimization(this)
            recommendation = true; %%% How will you optimize? this is a kalmann filter
        end
        
        function processEvents(this, events)
            % Handle the events
            processEvents@minislam.slam.VehicleSLAMSystem(this, events); 
            %%% perform processEvents in VehicleSLAMSystem and then store
            %%% in timeStore, Xest store and PEst store
            % Store the estimate for the future
            this.timeStore(:, this.stepNumber) = this.currentTime;
            this.xEstStore(:, this.stepNumber) = this.xEst(1:3);
            this.PEstStore(:, this.stepNumber) = diag(this.PEst(1:3, 1:3));
        end
        
        
        function optimize(~, ~)
            % Nothing to do
        end
    end
       
    methods(Access = protected)
                    
        function handleInitialConditionEvent(this, event)
            this.xEst = event.data;
            this.PEst = event.covariance;
            this.xPred = this.xEst;
            this.PPred = this.PEst;
        end
        
        function handleNoPrediction(this)
            this.xPred = this.xEst;
            this.PPred = this.PEst;
        end
        
        function handlePredictToTime(this, time, dT)

            % You will need to write the code to implement the process
            % model which:
            % 1. Computes this.xPred
            % 2. Computes the Jacobian
            % 3. Computes the process noise 
            
            %%% time is event.time 
            %%% My code
            %process noise
            Q = this.uCov;
            odometry=this.u; 
            
            prev_X = this.xEst;

            c = cos(prev_X(3));
            s = sin(prev_X(3));
            
            M = [c, -s, 0;...
                s, c, 0;...
                0, 0, 1];
            
            %v = sqrtm(Q) * randn(3, 1);
            predictedX = this.xEst;
            predictedX = predictedX + dT * M * (odometry); 
            %%% For prediction, error is zero, I think
            predictedX(3) = g2o.stuff.normalize_theta(predictedX(3));
            
            this.xPred = predictedX;
            this.xEst = this.xPred;
%             dx = predictedX(1) - prev_X(1);
%             dy = predictedX(2) - prev_X(2);
%             
%             %Jacobians
%             J_x = dT * [1, 0, -dx*s - dy*c;...
%                     0, 1, dx*c - dy*s;...
%                     0,0,1];
%             % J_i = dT*M;
            
            dx = odometry(1);
            %Jacobians
            J_x =  [1, 0, -dT*dx*s;...
                    0, 1, dT*dx*c;...
                    0,0,1];
            
            this.PPred = J_x * this.PEst * J_x' + Q; %  %J_i * Q * J_i'
            this.PEst = this.PPred;
            %%% End of my code
            
%             error('handlePredictToTime: implement');
        end
        
        function handleGPSObservationEvent(this, event)
            
            % You will need to write the code to implement the fusing the
            % platform position estimate with a GPS measurement
            ndim_state = length(this.xPred);
            z = event.data;
            R = event.covariance;
            J_H = [1,0,0;...
                 0,1,0];
            
            innovation = z-this.xPred(1:2);
            S = J_H * this.PPred * J_H' + R; %%% Intermediate variable
            K = this.PPred * J_H' * inv(S);
            this.xPred = this.xPred + K*innovation;
            this.PPred = (eye(ndim_state)- K*J_H) * this.PPred; 


%             C = this.PPred * J_H';
%             S = J_H * C + R;
%             W = C/S;
%             predicted_z = J_H * this.xPred;
%             this.xPred = this.xPred + W * (z - predicted_z);
%             this.PPred = this.PPred - (W * S * W');

            this.xEst = this.xPred;
            this.PEst = this.PPred;
            
            
%             error('handleGPSObservationEvent: implement');
        end
        
        function handleLandmarkObservationEvent(this, event)
            error('Not supported; this should only be used to implement odometry and GPS')
        end
 
    end
end