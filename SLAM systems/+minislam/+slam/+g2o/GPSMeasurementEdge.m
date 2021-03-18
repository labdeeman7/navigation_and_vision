classdef GPSMeasurementEdge < g2o.core.BaseUnaryEdge
   
    methods(Access = public)
    
        function this = GPSMeasurementEdge()
            this = this@g2o.core.BaseUnaryEdge(2);
        end
        
        function computeError(this)
            x = this.edgeVertices{1}.estimate();
            this.errorZ = x(1:2) - this.z;
            %%%A bit confused about this equation. Shouldnt error be
            %%%this.z-x(1:2) instead? and shouldnt j have -1's 
        end
        
        function linearizeOplus(this)
            this.J{1} = ...
                [1 0 0;
                0 1 0];
        end        
    end
end