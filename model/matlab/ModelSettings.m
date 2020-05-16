classdef ModelSettings    
    properties
        % Model settings
        % ----------------------------------------------------------------------------------------
        gc  =  9.8153;        % Acceleration of gravity [m/s^2]
        g   = [0, 9.8153, 0]; % Array of 'g' for model  [{m/s^2, m/s^2, m/s^2}]
        Rad = 6378245;        % Earth radius            [m]
        h   = 200;            % Height above sea level  [m]
        M   = 5.9726e24;      % Earth mass              [kg]
        U   = 7.29e-5;        % Earth angular velocity  [rad/s]
        
        % Simulation settings
        % ----------------------------------------------------------------------------------------
        dt                     = 1/400;                 % Alghoritm sampling period         [1/Hz]
        simulation_time        = 3600;                  % Alghoritm simulation time         [sec]
        simulation_iterations  = simulation_time*1/dt;  % Alghoritm simulation itterations  [n]
        
        % Calibration settings
        % ----------------------------------------------------------------------------------------
        calibration_time       = 5*60;                  % Alghoritm calibartion time        [sec] 
        calibration_iterations = calibration_time*1/dt; % Alghoritm calibration itterations [n]
        axels_errors_file_path = 'logs/axels_errors.txt';
        gyros_errors_file_path = 'logs/gyros_errors.txt';

        % Programm settings
        % ----------------------------------------------------------------------------------------

        
        % Programm flags
        % ----------------------------------------------------------------------------------------
        debug_mode_flag      = true;
        print_solutions_flag = true;
        subplot_print_flag   = true;
        save_solutions_flag  = true;
        
        % Calibration flags
        % ----------------------------------------------------------------------------------------        
        log_calibration_parameters_flag = true;
    end
    
    methods
        function obj = ModelSettings(sim_time, sample_rate)
            obj.simulation_time = sim_time;
            obj.dt = 1 / sample_rate;
            
            obj.simulation_iterations = obj.simulation_time * sample_rate;
        end
        
        % Programm flag setters
        % ------------------------------------------------------------------
        function obj = set_debug_mode_flag(obj, debug_mode)
            obj.debug_mode_flag = debug_mode;
        end
        
        function obj = set_print_solutions_flag(obj, print_solutions)
            obj.print_solutions_flag = print_solutions;
        end
        
        function obj = set_subplot_print_flag(obj, subplot_print)
            obj.subplot_print_flag = subplot_print;
        end
        
        function obj = set_save_solutions_flag(obj, save_solutions)
            obj.save_solutions_flag = save_solutions;
        end
    end
end