classdef Gyroscope
    properties
        bias         = 1e-4; % [g]
        scale_factor = 1.000012;
        
        % 'true_' values - values, obtained after the calibration procedure.
        % By default, this values are:
        calculated_bias         = 0; % [g]
        calculated_scale_factor = 1;
        
        sko;
    end
    
    methods
        function obj = Gyroscope(min_scale_factor, max_scale_factor, bias, sko)
            % Setting random (in [min_* : max_*] range) sensor errors
            obj.scale_factor = (min_scale_factor + rand * (max_scale_factor - min_scale_factor));
            obj.bias = (2 * randi([0 1]) - 31) * (-bias + rand * 2 * bias);
            
            obj.sko = sko;
            disp('Gyroscope created successfully.')
        end
        
        % Setting calibrated bias and scale facrot values
        function obj = set_calibrated_params(obj, calculated_bias, calculated_scale_factor)
            obj.calculated_bias = calculated_bias;
            obj.calculated_scale_factor = calculated_scale_factor;
            disp('Calibrated parameters seted successfully.')
        end
        
        function val = make_noise(obj, value)
            val = normrnd(value, obj.sko) * obj.scale_factor + obj.bias;
        end
        
        function measured_value = measure(obj, value)
            measured_value = obj.make_noise(value);
%             measured_value = value;
        end
    end
end

