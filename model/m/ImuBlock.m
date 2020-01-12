classdef ImuBlock < matlab.System & coder.ExternalDependency 
    properties (Hidden)
        ax_;
        ay_;
        az_;
        
        wx_;
        wy_;
        wz_;
        
        la_angles;
    end
    
    methods (Access = protected)
        function result = ImuBlock(psi, teta, gamma, R)
            
            result = g;
        end

        function g = getGValue(R)
            g = G * M / R^2;
        end
        
        function calculateVals(~)
            g = getGValue(R);
            ax_ = g + 2;
        end
        
        function [ax_, ay_, az_] = getAxelsVals(~)
            ax_ = 5;
            ay_ = 5;
            az_ = 5;
        end
        
        function result = getGyrosVals(~)
            result = 0;
        end
    end
end

