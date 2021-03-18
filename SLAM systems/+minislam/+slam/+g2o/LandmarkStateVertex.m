%%% The object state vertex:
%%% x(1) - x_i 
%%% x(2) - y_i
%%% The values of x_i and y_i are not given
%%% since neither is an angle, we would not reimplement the oplus method. 
%%% A lot of the things I have been assuming are correct, I just needed to
%%% check the g2o library.


classdef LandmarkStateVertex < g2o.core.BaseVertex
    
    properties(Access = protected)
        Id;
    end
    
    methods(Access = public)
        function this = LandmarkStateVertex(landmarkId)
            this=this@g2o.core.BaseVertex(2); %%% Means there are two states to be set in measurement
            this.Id = landmarkId;
        end
        
        function Id = landmarkId(this)
            Id = this.Id;
        end
    end
end