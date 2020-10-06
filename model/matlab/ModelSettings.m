classdef ModelSettings    
    properties
        % Model settings
        % ----------------------------------------------------------------------------------------
        gc    =  9.8153;        % Acceleration of gravity       [m/s^2]
        g     = [0, 9.8153, 0]; % Array of 'g' for model        [{m/s^2, m/s^2, m/s^2}]
        Rad   = 6371200;        % Earth radius                  [m]
        h_min = 400e3;          % Min height above sea level    [m]
        h_max = 428e3;          % Max height above sea level    [m]
        v_min = 7649.7222;      % Min satellite linear velocity [m/sec]
        v_max = 7666.6667;      % Max satellite linear velocity [m/sec]
        M     = 5.9726e24;      % Earth mass                    [kg]
        G     = 6.67430e-11     % Gravitational constant        [m^3/(kg * s^2)]
        U     = 7.29e-5;        % Earth angular velocity        [rad/s]
        phi   = 51.7275 / 57.3; % NSSK start latitude           [rad]        
        la    = 36.7559 / 57.3; % NSSK start longtitude         [rad]
        gn;                     % Array of g in SSK             [m/sec^2]
        gb;                     % Array of g in NSSK            [m/sec^2]        
        
        % Initial conditions
        % ----------------------------------------------------------------------------------------
        Ax_n_0; Ay_n_0; Az_n_0;
        Vx_n_0; Vy_n_0; Vz_n_0;
        Wx_n_0; Wy_n_0; Wz_n_0;
        Fx_n_0; Fy_n_0; Fz_n_0;

        Ax_b_0; Ay_b_0; Az_b_0;
        Vx_b_0; Vy_b_0; Vz_b_0;
        Wx_b_0; Wy_b_0; Wz_b_0;
        Fx_b_0; Fy_b_0; Fz_b_0;
        
        % Simulation settings
        % ----------------------------------------------------------------------------------------
        dt;                    % Alghoritm sampling period                  [1/Hz]
        simulation_time;       % Alghoritm simulation time                  [sec]
        sample_rate;           % Alghoritm sample rate                      [Hz]         
        simulation_iterations; % Alghoritm simulation itterations           [n]
        revolution_time;       % Spaceship revolution around the Eatth time [sec]
        revolution_iterations; % Algoritm simulation itterations            [n]
        
        % Calibration settings
        % ----------------------------------------------------------------------------------------
        calibration_time = 2 * 60; % Alghoritm calibartion time        [sec] 
        calibration_iterations;    % Alghoritm calibration itterations [n]
        axels_calibration_results_file_name = 'axels_errors.txt';
        gyros_calibration_results_file_name = 'gyros_errors.txt';
        algorithm_results_errors_file_name  = 'errors.txt';

        % Sensors settings
        % ----------------------------------------------------------------------------------------
        axel_min_scale_factor = 1.2; 
        axel_max_scale_factor = 1.4;
        axel_bias             = 0.03924;      % Accelerometer bias [g]
        axel_sko              = 3.3e-4 * 9.8; % Accelerometer sko  [g]
        
        gyro_min_scale_factor = 1.2; 
        gyro_max_scale_factor = 1.4;
        gyro_bias             = 0.03924;            % Gyro bias [rad/sec]
        gyro_sko              = 0.03 / 57.3 / 3600; % Gyro sko  [rad/sec]

        % Programm settings
        % ----------------------------------------------------------------------------------------
        font_size     = 15; % Font size of axis names on solutions plots
        font_size_sub = 10; % Font size of others names on solutions plots
        
        % Programm flags
        % ----------------------------------------------------------------------------------------
        debug_mode_flag                 = true; % Print debug messeges in console, print debug graphics
        display_solutions_flag          = true; % Show solution graphics on screen
        subplot_print_flag              = true; % Print all solutions on one plot 
        log_algorithm_solutions_flag    = true; % Store work resualt of alghorithm in file, save pictures
        plot_trajectory_simulation_flag = true; % Print trajectory simulation params on screen
        
        % Calibration flags
        % ----------------------------------------------------------------------------------------
        calibration_mode_flag        = true; % Enable sensors calibration
        log_calibration_results_flag = true; % Store calibration resualt of alghorithm in file
        
        % Other variables
        % ----------------------------------------------------------------------------------------  
        E;                                   % Single matrix 4x4
        solution_folder_name;                % Folder in which will be stored alghorithm solution errors file
        calibration_folder_name;             % Folder in which will be stored calibration files
        gyros_calibration_results_file_path; % Path to file which will be stored gyros calibration resualts
        axels_calibration_results_file_path; % Path to file which will be stored axels calibration resualts
        algorithm_results_errors_file_path;  % Path to file which will be stored algorithm results errors
        path_to_telemetry_file;              % Path to file which contain real ISS teelemetry
        path_to_linear_vel_file;             % Path to file which contain real ISS linear velocity
        
        % Progress bar parameters
        % ----------------------------------------------------------------------------------------         
        progress_bar_counter = 2;
        progress_bar_width   = 30;
        progress_line_size   = 0;
        progress_bar;
    end
    
    methods
        function obj = ModelSettings(sim_time, sample_rate)
            disp('Creating settings object ...')
            
            % Time settings
            obj.simulation_time = sim_time;
            obj.revolution_time = 93 * 60; % ISS one revolution around the Earth time [s]
            
            obj.sample_rate = sample_rate;
            obj.dt = 1 / sample_rate;
            
            obj.simulation_iterations  = obj.simulation_time  * obj.sample_rate;
            obj.revolution_iterations  = obj.revolution_time  * obj.sample_rate;
            obj.calibration_iterations = obj.calibration_time * obj.sample_rate;

            
            % Files settings
            obj.solution_folder_name = join(['./solutions/', datestr(datetime('now'), "yyyy-mm-dd_HH-MM-SS")]);
            obj.gyros_calibration_results_file_path = join([obj.solution_folder_name, '/', obj.gyros_calibration_results_file_name]);
            obj.axels_calibration_results_file_path = join([obj.solution_folder_name, '/', obj.axels_calibration_results_file_name]);
            obj.algorithm_results_errors_file_path = join([obj.solution_folder_name, '/', obj.algorithm_results_errors_file_name]);

            obj.path_to_telemetry_file = './telemetry/russia_one_rev_km.txt';
            obj.path_to_linear_vel_file = './telemetry/V.txt';
            
            mkdir(obj.solution_folder_name);
            
            % Navigtion settings in NSSK
            obj.Ax_n_0 = 0;
            obj.Ay_n_0 = 0;
            obj.Az_n_0 = 0;

            obj.Vx_n_0 = 0;
            obj.Vy_n_0 = obj.v_min;
            obj.Vz_n_0 = 0;

            obj.Wx_n_0 = 0;
            obj.Wy_n_0 = 0;
            obj.Wz_n_0 = 0;

            obj.Fx_n_0 = 0;
            obj.Fy_n_0 = 0;
            obj.Fz_n_0 = 0;

            % Navigtion settings in SSK
            obj.Ax_b_0 = 0;
            obj.Ay_b_0 = 0;
            obj.Az_b_0 = obj.M * obj.G / ((obj.Rad + obj.h_min) ^ 2);

            obj.Vx_b_0 = 0;
            obj.Vy_b_0 = obj.v_min;
            obj.Vz_b_0 = 0;

            obj.Wx_b_0 = 2 * pi / obj.revolution_time;
            obj.Wy_b_0 = 0;
            obj.Wz_b_0 = 0;

            obj.Fx_b_0 = 0;
            obj.Fy_b_0 = 0;
            obj.Fz_b_0 = 0;
            
            % Progress bar settings
            obj.progress_bar = '[';
            for i = 2:obj.progress_bar_width + 1
                obj.progress_bar = append(obj.progress_bar, '-');
            end
            obj.progress_bar = append(obj.progress_bar, ']');
            disp('Settings object creating successfully.')
            
            % Other settings
            obj.E = eye(4);
        end
        
        % Programm flag setters
        % ------------------------------------------------------------------
        function obj = set_debug_mode_flag(obj, debug_mode)
            obj.debug_mode_flag = debug_mode;
            fprintf('Debug mode:            %d.\n', obj.debug_mode_flag);
        end
        
        function obj = set_display_solutions_flag(obj, display_solutions)
            obj.display_solutions_flag = display_solutions;
            fprintf('Display solution mode: %d.\n', obj.display_solutions_flag);
        end
        
        function obj = set_subplot_print_flag(obj, subplot_print)
            obj.subplot_print_flag = subplot_print;
            fprintf('Solution subplot mode: %d.\n', obj.subplot_print_flag);
        end
        
        function obj = set_log_algorithm_solutions_flag(obj, save_solutions)
            obj.log_algorithm_solutions_flag = save_solutions;
            fprintf('Solution log mode:     %d.\n', obj.log_algorithm_solutions_flag);
        end
        
        function obj = set_plot_trajectoey_simulation_flag(obj, plot_trajectory)
            obj.plot_trajectory_simulation_flag = plot_trajectory;
            fprintf('Plot trajectory mode:  %d.\n', obj.plot_trajectory_simulation_flag);
        end
    end
end