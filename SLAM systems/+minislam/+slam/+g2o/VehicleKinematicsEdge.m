% This class uses a slightly simpler model for the vehicle kinematics used
% in the lectures. This is the more standard built in type for estimate.
%
% The model assumes that the vehicle speed is specified in the vehicle
% frame and is then projected into the world frame. Specifically,
%
% M = [cos(theta) -sin(theta) 0; sin(theta) cos(theta) 0;0 0 1];
%
% The process model has the form:
%
% x = x + M * [vx;vy;theta]
%
% where vx, vy and vtheta are the velocities.
%
% The error model 
% eTheta = 

classdef VehicleKinematicsEdge < g2o.core.BaseBinaryEdge
    
    properties(Access = protected)
        % The length of the time step
        dT;
    end
    
    methods(Access = public)
        function this = VehicleKinematicsEdge(dT)
            this = this@g2o.core.BaseBinaryEdge(3);            
            this.dT = dT;
        end
       
        function initialize(this)
            
            %this.dT = this.edgeVertices{2}.time() - this.edgeVertices{1}.time();
            
            assert(this.dT > 0);
                        
            priorX = this.edgeVertices{1}.x;

            c = cos(priorX(3));
            s = sin(priorX(3));
            
            M = this.dT * [c -s 0;
                s c 0;
                0 0 1];
            
            % Compute the posterior assming no noise
            this.edgeVertices{2}.x = this.edgeVertices{1}.x + M * this.z;

            % Wrap the heading to -pi to pi
            this.edgeVertices{2}.x(3) = g2o.stuff.normalize_theta(this.edgeVertices{2}.x(3));

        end
        
        function computeError(this)
            
             % Rotation matrix from prior state
            priorX = this.edgeVertices{1}.x;

            c = cos(priorX(3));
            s = sin(priorX(3));
            
            Mi = 1/this.dT * [c s 0;
                            -s c 0;
                            0 0 1];
            
            % Compute the error.
%             this.errorZ = Mi * (this.edgeVertices{2}.x ...
%                 - priorX) - this.z;
%             this.errorZ(3) = g2o.stuff.normalize_theta(this.errorZ(3));
        
            dx = this.edgeVertices{2}.x - priorX;
            dx(3) = g2o.stuff.normalize_theta(dx(3));
            
            this.errorZ = Mi * (dx) - this.z;
            
            % Wrap the heading error to -pi to pi
            

            % Compute the error and put it in this.errorZ
            % error('Implement');
        end
        
        % Compute the Jacobians
        function linearizeOplus(this)
            priorX = this.edgeVertices{1}.x;
            c = cos(priorX(3));
            s = sin(priorX(3));
            dx = this.edgeVertices{2}.x - priorX;
            Mi = 1/this.dT * [c s 0;
                -s c 0;
                0 0 1];
            this.J{2} = Mi;
            this.J{1}(1, 1) = - c/this.dT ;
            this.J{1}(1, 2) = - s/this.dT;
            this.J{1}(1, 3) = (-dx(1) * s + dx(2) * c)/this.dT;
            this.J{1}(2, 1) = s/this.dT;
            this.J{1}(2, 2) = - c/this.dT;
            this.J{1}(2, 3) = (-dx(1) * c - dx(2) * s)/this.dT;
            this.J{1}(3, 3) = -1/this.dT;
            
            
            % error('Implement');
            
        end
    end    
end