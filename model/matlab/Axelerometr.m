classdef Axelerometr
    properties
        bias         = 0.03924; % [g]
        scale_factor = 1.3;
        
        % 'true_' values - values, obtained after the calibration procedure.
        % By default, this values are:
        true_bias         = 0; % [g]
        true_scale_factor = 1;
        
        error_function;
        sko;
    end
    
    methods
        % Setting calibrated bias and scale facrot values
        function obj = set_calibrated_params(obj, true_bias, true_scale_factor)
            obj.true_bias = true_bias;
            obj.scale_factor = true_scale_factor;
        end
        
        function obj = Axelerometr(axel_error_function, min_scale_factor, max_scale_factor, bias, sko)
            obj.scale_factor = (min_scale_factor + rand * (max_scale_factor - min_scale_factor));
            obj.bias = (2 * randi([0 1]) - 31) * (-bias + rand * 2 * bias);
            obj.sko = sko;
            obj.error_function = axel_error_function;
        end
        
        function val = make_noise(obj, axel_error_function)
            val = normrnd(axel_error_function, obj.sko) * obj.scale_factor / obj.true_scale_factor + obj.true_bias - obj.bias;
        end
        
        function measured_value = measure(obj, value)
            measured_value = make_noise(obj, obj.error_function(value));
        end
    end
end

