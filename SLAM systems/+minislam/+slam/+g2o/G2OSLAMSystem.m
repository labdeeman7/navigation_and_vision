% This class implements an event-based estimation system using g2o and
% the barebones for building up a minimal, ideal SLAM system. The system is
% event-based and responds to a sequence of events which are time stamped
% and served in order. To implement your SLAM system, you will need to
% implement various methods which mostly involve working with the graph.
% These methods are initially stubbed out and will generate exceptions if
% you try to call them.

classdef G2OSLAMSystem < minislam.slam.VehicleSLAMSystem
    
    properties(Access = protected)
                
        % Flag to run the detailed graph validation checks
        validateGraphOnInitialization; %%% a parameter that can be passed to 
        %%% graph.initializeOptimization() true or false, checks if graph
        %%% has been constructed properly
        
        % The graph used for performing estimation.
        graph; %%% I am guessing this is the graph we use for estimation
        
        % The optimization algorithm
        optimizationAlgorithm; %%% The newton method or others
        
        % The most recently created vehicle vertex.
        currentVehicleVertex;  
        %%% as it says, check how it is used. I am guessing it is going to
        %%% be a baseBinaryVertex
        
        % The set of all vertices associated with the vehicle state over
        % time.
        vehicleVertices; %%%likely all the vehicle vertice for the SLAM problem 
        vehicleVertexId;  %%% I am guessing the Id of that particular vertex
        
        % The landmark vertices. Confusingly enough, "Map" here refers to
        % the data structure which is used to store the landmarks. (It
        % allows random access of landmarkID to landmark object.)
        landmarksMap; %%%Map is literally a dictionary or hash-table, ID to object. Cool
        
        %%% if 0, do not perform keyFrame,
        %%% if 1, perform 4.2, 
        %%% if 2, perform 4.3
        performKeyFrame_state;
        
        %%%%true, keep first edge
        %%%%false, delete first edge
        keepFirstEdgeFlag;
        
        
    end
        
    methods(Access = public)
        
        % Create the localization system and start it up.
        function this = G2OSLAMSystem()
            
            % Call the base class constructor
            this = this@minislam.slam.VehicleSLAMSystem(); 
            %%% I guess this is how inheritance is done in Matlab
            %%% Remeber, that vehicleSLAMSystem majorly handles events this
            %%% is the init as well too
            
            % Create the graph and the optimization algorithm
            this.graph = g2o.core.SparseOptimizer();
            algorithm = g2o.core.LevenbergMarquardtOptimizationAlgorithm();
            this.graph.setAlgorithm(algorithm);
            
            % Do detailed checking by default
            this.validateGraphOnInitialization = true; %%% Seems important, no idea
            
            % Preallocate; this is a lower bound on size
            this.vehicleVertices = cell(1, 10000);  %%%Maximum amount of vertex seem to be 10,000
            %%% Why he did this I do not know
            
            % No vehicle vertices initally set
            this.vehicleVertexId = 0; %%None set yet
            
            % Allocate the landmark map
            this.landmarksMap = containers.Map('KeyType', 'int64', 'ValueType', 'any');
            %%% A way to configure the dictionary for the landmarks, keys
            %%% only int64 allowed, valueType, can be whatever
            
            this.performKeyFrame_state = 0;
            %%%0 = do not perform keyframe
            %%%1 = perform  4.1 sparse keyframe
            
            this.keepFirstEdgeFlag = true;
            
        end
        
        % Get the underlying g2o graph
        function graph = optimizer(this)
            graph = this.graph; %%% An interface of some sorts for accesing the graph
            %%% Most likely for output purposes as graph is protected.
        end
        
        % Set the flag to enable / disable validation. This spots errors in
        % graph construction, but repeated calls can slow things down by
        % 20% or more.
        function setValidateGraph(this, validateGraphOnInitialization)
            this.validateGraphOnInitialization = validateGraphOnInitialization;
            %%% I am guessing this is also for outside control, not
            %%% outputting, others fxn can control validation from here
        end
        
        function validateGraphOnInitialization = validateGraph(this)
            validateGraphOnInitialization = this.validateGraphOnInitialization;
            %%% Another getter for the value of
            %%% validateGrapphOnInitialization i gues
        end
        
        % Recommend if an optimization is a good idea. Based on an event,
        % some activities (e.g., such as loop closing) can have a very big
        % impact on the estimates. Therefore, this method returns if the
        % localization algorithm thinks optimizing is a good idea. Here we
        % always return false because this gives the fastest results
        % because you just optimize once, right and the very end
        function recommendation = recommendOptimization(this)
            recommendation = false;
            %%% After every step, do you want to optimize?
            % This is how to do it after every 500 steps
            %recommendation = rem(this.stepNumber, 100) == 0;
        end
        
        
        %%%%%
        
        function graphInformation = getGraphInformation(this)
            graphInformation.numVertices = length(this.graph.vertices());
            graphInformation.numEdges = length(this.graph.edges());
            graphInformation.amountOfLandmarks = size(this.landmarksMap,1);
            graphInformation.amountOfvehicleVertices = this.vehicleVertexId;
            
            allAmountOfLandmarks = zeros(1,this.vehicleVertexId);
            for l = 1:this.vehicleVertexId
                vehicleVertex = this.vehicleVertices{l};
                EdgesConnectedToVertex = vehicleVertex.edges();
                amountOfLandmarks = 0;
                for i = 1:length(EdgesConnectedToVertex)
                    edge = EdgesConnectedToVertex{i};
                    if class(edge) == "minislam.slam.g2o.LandmarkRangeBearingEdge"
                        amountOfLandmarks = amountOfLandmarks + 1;
                    end  
                end
                allAmountOfLandmarks(l) = amountOfLandmarks;
            end
            
            graphInformation.amountLandmarkSeenPerVertex = allAmountOfLandmarks;
            graphInformation.meanAmountLandmarkSeenPerVertex = mean(allAmountOfLandmarks);
            
            
            
            allAmountOfVehicleVertexConnected = zeros(1,graphInformation.amountOfLandmarks);
            for l = 1:graphInformation.amountOfLandmarks
                LandmarkVertex = this.vehicleVertices{l};
                EdgesConnectedToLandmarkVertex = LandmarkVertex.edges();
                AmountOfVehicleVertexConnected = length(EdgesConnectedToLandmarkVertex);
                allAmountOfVehicleVertexConnected(l) = AmountOfVehicleVertexConnected;
            end
            
            graphInformation.amountVehicleVertexPerLandmark = allAmountOfVehicleVertexConnected;
            graphInformation.meanAmountVehicleVertexPerLandmark = mean(allAmountOfVehicleVertexConnected);
        end
        
        %%%%%
        
        
        function setKeyFrameFlags(this, performKeyFrame_state, FirstEdgeFlag)
            this.performKeyFrame_state = performKeyFrame_state;
            this.keepFirstEdgeFlag = FirstEdgeFlag;
        end
        
        % This method runs the graph optimiser and the outputs of the
        % optimisation are used as the initial conditions for the next
        % round of the graph. This makes it possible to iterative compute
        % solutions as more information becomes available over time.
        function optimize(this, maximumNumberOfOptimizationSteps)
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            %%%my code
            KEYFRAME4_2 = 1;
            SPARSEKEYFRAMESLAM4_3 = 2;
            
            if (this.performKeyFrame_state == KEYFRAME4_2)
                this.keyFrameSLAM4_2(this.keepFirstEdgeFlag);
            elseif (this.performKeyFrame_state == SPARSEKEYFRAMESLAM4_3)
                this.sparseKeyFrameSLAM4_3();
            end
            
            
            this.graph.initializeOptimization(this.validateGraphOnInitialization);
            if (nargin > 1)
                this.graph.optimize(maximumNumberOfOptimizationSteps); 
                %%% remember there was an error about amount of times
                %%% optimization is done, simon corrected it saying 10
                %%% times may not be enough, you need to check
            else
                this.graph.optimize();
                %%% cool, amount of iterations here, use
                %%% default(10) if extra param not given
            end
        end
                
        % Return the current mean and covariance estimate of the robot.
        % This is only valid after optimization has been called.
        function [x, P] = robotEstimate(this)
            [xS, PS] = this.graph.computeMarginals(this.currentVehicleVertex);
            %%% still a bit confused, I thought we dont compute marginals
            %%% for slam, maybe this is that MAP solution he said
            x=full(xS); %%% Turns a sparse matrix to a full one
            P=full(PS); 
            %%% Note that only valid after optimization called
        end
        
        function [T, X, P] = robotEstimateHistory(this)
            %%% I am guessing this does some form export, it takes the time
            %%% the state and the covariance so they can plot later
            % Extract the graph
            [xS, PS] = this.graph.computeMarginals();
            
            % Create the output array
            X = zeros(3, this.vehicleVertexId);
            P = zeros(3, this.vehicleVertexId);
            T = zeros(1, this.vehicleVertexId);
            
            % Copy the outputs over
            for v = 1 : this.vehicleVertexId
                idx = this.vehicleVertices{v}.hessianIndex();
                
                T(v) = this.vehicleVertices{v}.time();
               
                % Copy the estimate into the array. If the vertices is
                % fixed (conditioned), its estimate is okay. The covariance
                % is not explicitly defined, but has a value of zero.
                % Therefore we fill this manually.
                if (isempty(idx) == true)
                    X(:, v) = this.vehicleVertices{v}.estimate();
                    P(:, v) = zeros(3, 1);
                else
                    X(:, v) = full(xS(idx));
                    P(:, v) = full(diag(PS(idx, idx)));
                end
            end
        end
        
        % Return the means and covariances of the landmark estimates. These
        % are only valid after optimization has been called.
        function [x, P, landmarkIds] = landmarkEstimates(this) 
            
            landmarkVertices = values(this.landmarksMap);
            
            numberOfLandmarks = length(landmarkVertices);
            
            landmarkIds = NaN(1, numberOfLandmarks);
            x = NaN(2, numberOfLandmarks);
            P = NaN(2, 2, numberOfLandmarks);
            
            [xS, PS] = this.graph.computeMarginals();
            
            for l = 1 : numberOfLandmarks
                landmarkIds(l) = landmarkVertices{l}.landmarkId();
                idx = landmarkVertices{l}.hessianIndex();
                x(:, l) = full(xS(idx));
                if (isempty(idx == true))
                    P(:, :, l) = zeros(3, 3);
                else
                    P(:, :, l) = full(PS(idx, idx));
                end
            end
        end
    end
        
    % These are the methods you will need to overload
    methods(Access = protected)
                        
        % Declare bodies all of these methods. This makes it possible to
        % instantiate this object.
         
        function handleInitialConditionEvent(this, event)
            
            % Add the first vertex and the initial condition edge
            this.currentVehicleVertex = minislam.slam.g2o.VehicleStateVertex(this.currentTime);
            this.currentVehicleVertex.setEstimate(event.data);
            this.currentVehicleVertex.setFixed(true); %%% first value is setFixed ya. 
            %%% setFixed
            this.graph.addVertex(this.currentVehicleVertex);
            this.vehicleVertexId = 1;
            this.vehicleVertices{this.vehicleVertexId} = this.currentVehicleVertex;
       end
       
        function handleNoPrediction(~)
            % Nothing to do
        end
        
        function handlePredictToTime(this, time, dT)
            
            % Create the next vehicle vertex and add it to the graph
            this.currentVehicleVertex = minislam.slam.g2o.VehicleStateVertex(time);
            this.graph.addVertex(this.currentVehicleVertex);
            
            % Handle the prediction
            
            %%% My code 
            k = this.vehicleVertexId; %% previous vertex in graph
            Q = this.uCov; 

            omegaQ = inv(Q); %%%why are we taking this inverse as information matrix?  
            odometry=this.u;
            
            processModelEdge = minislam.slam.g2o.VehicleKinematicsEdge(dT);
            processModelEdge.setVertex(1, this.vehicleVertices{k});
            processModelEdge.setVertex(2, this.currentVehicleVertex);
            processModelEdge.setMeasurement(odometry);
            processModelEdge.setInformation(omegaQ); %%%why though, y?
            this.graph.addEdge(processModelEdge);

            priorX = this.vehicleVertices{k}.estimate();
            c = cos(priorX(3));
            s = sin(priorX(3));

            M = [c, -s, 0;...
                s, c, 0;...
                0, 0, 1];

            predictedX = priorX;
            predictedX = predictedX + dT * M * (odometry);
            predictedX(3) = g2o.stuff.normalize_theta(predictedX(3));        
            this.currentVehicleVertex.setEstimate(predictedX);
             
            %%%end code
            
        
%             error('handlePredictToTime: implement me');
            
            % Bump the indices
            this.vehicleVertexId = this.vehicleVertexId + 1;
            this.vehicleVertices{this.vehicleVertexId} = this.currentVehicleVertex;
        end
        
        function handleGPSObservationEvent(this, event)
       
            % Handle a GPS measurement
            
            %%%My code
            k = this.vehicleVertexId;
            
            e = minislam.slam.g2o.GPSMeasurementEdge();
    
            % Link it so that it connects to the vertex we want to estimate
            e.setVertex(1, this.vehicleVertices{k});

            % Set the measurement value and the measurement covariance
            omegaR = inv(event.covariance); %%% why though, why are we taking inverses?
            e.setMeasurement(event.data);
            e.setInformation(omegaR); %(event.covariance);

            % Add the edge to the graph
            this.graph.addEdge(e);
%             error('handleGPSObservationEvent: implement me');
        end
        
        function handleLandmarkObservationEvent(this, event)
            % Iterate over all the landmark measurements
            for l = 1 : length(event.landmarkIds)
                
                % Get the landmark vertex associated with this measurement.
                % If necessary, a new landmark vertex is created and added
                % to the graph.
                [landmarkVertex, newVertexCreated] = this.createOrGetLandmark(event.landmarkIds(l));
                z = event.data(:, l);
            
                
                %%%My code
                if (newVertexCreated == true)
%                     X_k = this.currentVehicleVertex.estimate();
%                     estimated_landmark_state =  X_k(1:2) + (z(1) * randn(2,1));
                  
                    landmarkVertex.setEstimate([0;0]);
                    this.graph.addVertex(landmarkVertex);
                end
                
                %%%develop edges between the currentVehicleVertex and the
                %%%landmark been seen at that point
                
                e = minislam.slam.g2o.LandmarkRangeBearingEdge();
    
                % Link it so that it connects to the vertex we want to estimate
                %%% 1 is the vehicle vertex, 2 is the landmark vertex
                e.setVertex(1, this.currentVehicleVertex);
                e.setVertex(2, landmarkVertex);

                % Set the measurement value and the measurement covariance
                e.setMeasurement(z);
                e.setInformation(pinv(event.covariance));
                this.graph.addEdge(e);                
            end
                
        end
   end
    
    methods(Access = protected)
        
        % This method returns a landmark associated with landmarkId. If a
        % landmark exists already, it is returned. If it does not exist, a
        % vertex is created and is added to the graph.
        function [landmarkVertex, newVertexCreated] = createOrGetLandmark(this, landmarkId)
            %%%landmarkVertex is the landmark returned
            %%%newVertexCreated is a flag telling if new or not
            %%%remember the landmarksmap is a dictionary with id as keys.
            % If the landmark exists already, return it
            if (isKey(this.landmarksMap, landmarkId) == true)
                landmarkVertex = this.landmarksMap(landmarkId);
                newVertexCreated = false;
                return
            end
            
            %%% Handle the condition that occurs when you create a new
            %%% vertex
            %%%My code
            landmarkVertex = minislam.slam.g2o.LandmarkStateVertex(landmarkId);
            this.landmarksMap(landmarkId) = landmarkVertex;
            newVertexCreated = true;
%             error('createOrGetLandmark: implement me');
            
        end
        
        %%%Question 4.2
        function keyFrameSLAM4_2(this, keepFirstEdge)
            disp(['TOTAL VERTICES: ', length(this.graph.vertices())]);
            disp(['TOTAL EDGES: ', length(this.graph.edges())]);   
            
            edgeArray = this.graph.edges();
            numberOfEdges = length(edgeArray);
            
            VehicleEdgeCount = 0;
            LandmarkEdgeCount = 0;
            gpsEdgeCount = 0;
            TotalEdgeCount = 0;

            for l = 1:numberOfEdges
                edge = edgeArray{l};
                TotalEdgeCount = TotalEdgeCount + 1;
                if class(edge) == "minislam.slam.g2o.VehicleKinematicsEdge"
                    VehicleEdgeCount = VehicleEdgeCount + 1;
                elseif class(edge) == "minislam.slam.g2o.LandmarkRangeBearingEdge"
                    LandmarkEdgeCount = LandmarkEdgeCount + 1;
                elseif class(edge) == "minislam.slam.g2o.GPSMeasurementEdge"
                    gpsEdgeCount = gpsEdgeCount + 1;
                end 
            end
            
            disp(['Vehicle Edge Count: ', num2str(VehicleEdgeCount)]);
            disp(['Landmark Edge Count: ', num2str(LandmarkEdgeCount)]);
            disp(['gpsEdgeCount: ', num2str(gpsEdgeCount)]);
            disp(['TOTAL EDGE COUNT: ', num2str(TotalEdgeCount)]);
            disp(['ACTUAL EDGE COUNT: ', num2str(length(this.graph.edges()))]);   
            
            
            VehicleEdgeCount = 0;
            for l = 1:numberOfEdges
                edge = edgeArray{l};
                if class(edge) == "minislam.slam.g2o.VehicleKinematicsEdge"
                    VehicleEdgeCount = VehicleEdgeCount + 1;
                    if (VehicleEdgeCount == 1 && keepFirstEdge == true)
                        disp("First Edge kept");
                    else
                        this.graph.removeEdge(edge);
                        disp(["#", num2str(VehicleEdgeCount), " Vehicle Edge Removed"]);
                    end
                end
            end
            
            disp("Total Vehicle Edge Count:");
            disp(VehicleEdgeCount);            
        end
        
        
        %%%% check this function
        %%%Question 4.3
        function sparseKeyFrameSLAM4_3(this)
            %%%method, I would be deleting the vehicleVertex based on the
            %%%heuristic that seeing less than 3 landmarks is not useful.
            %%%so I would be removing all the edges related to the vertex.
            %%% ensure to keep first vehicleVertice though
            
            disp('number of vertices and edges')
            disp(length(this.graph.vertices()));
            disp(length(this.graph.edges()));
            
            numberOfDeletedVertices = 0; 
            numberOfDeletedEdges = 0;
            
            for l = 100 : this.vehicleVertexId
                vehicleVertex = this.vehicleVertices{l};
                edgesConnectedToVehicleVertex = vehicleVertex.edges();
                numberOfEdges = length(edgesConnectedToVehicleVertex);
                
                numberOfLandmarkEdgesOnVertex = 0;
                
                for edge_index = 1:numberOfEdges
                    edge = edgesConnectedToVehicleVertex{edge_index};
                    if class(edge) == "minislam.slam.g2o.LandmarkRangeBearingEdge"
                        numberOfLandmarkEdgesOnVertex = numberOfLandmarkEdgesOnVertex + 1;      
                    end
                end
                
%                 disp("number of landmarks")
%                 disp(numberOfLandmarkEdgesOnVertex)
%                 disp(numberOfLandmarkEdgesOnVertex)
                
                if (numberOfLandmarkEdgesOnVertex < 14)
                    numberOfDeletedVertices = numberOfDeletedVertices + 1;
%                     disp("vertice deleted")
                    %%%get the edge array to be deleted
                    edgesConnectedToVehicleVertex = vehicleVertex.edges();
                    
                    %%%delete the edge array
                    numberOfEdges = length(edgesConnectedToVehicleVertex);
                    for edge_index = 1:numberOfEdges
                        this.graph.removeEdge(edgesConnectedToVehicleVertex{edge_index});
                        numberOfDeletedEdges = numberOfDeletedEdges+1;
                    end
                end
                
                
%                 if (l == 100)
%                     error("This is America");
%                 end
                
                
            end   
            
            disp("number of remaining vertices and edges")
            disp(length(this.graph.vertices()));
            disp(length(this.graph.edges()));
        end
                
    end
end
