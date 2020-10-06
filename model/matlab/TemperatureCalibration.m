classdef TemperatureCalibration
    properties
        sample_rate;             % Elements sample rate   [hz]
        calibration_time;        % Simulation time        [min] 
        calibration_interations; % Simulation iterrations [n]
        dt;
        
        T;           % Current temperature [deg]
        T_max = 60;  % Maximum temperature [deg]
        T_min = -40; % Minimum temperature [deg]
        dT;          % Delta temperature   [deg/s]
        
        w_b; w_f; w_s; w_sko; % Errors parameters
        
        t_k;  % Temperature factor [rad/s]
        dt_k; % Temperature increment factor [rad/s]
        t_k_calculated;  % Calculated temperature factor [rad/s]
        dt_k_calculated; % Calculated temperature increment factor [rad/s]
        
        gla;  % Generated linear axeleration [g]
        gav;  % Generated angular velocity (~10 [deg/min]) [rad/s]
        
        % measurement arrays
        positive_rotation_measurement;
        negative_rotation_measurement;
        positive_rotation_temp_measurement;
        negative_rotation_temp_measurement;
        calibrated_measurement;
        
        % Flags
        sf_and_bias;            % take into account residual factors
        plot_grapsh;            % plot all graphs
        log_to_file;            % log calibration solution to file
        sf_and_bias_level = 2;  % residual sf and bias "size" [1,3]
    end
    
    methods
        function obj = TemperatureCalibration()
            obj.sf_and_bias = false;
            obj.plot_grapsh = true;
            obj.log_to_file = true;
            
            fprintf("Residual factors: %d\n", obj.sf_and_bias);
            fprintf("Plotting graphs:  %d\n", obj.plot_grapsh);
            fprintf("Solution logging: %d\n", obj.log_to_file);
            
            obj = obj.initialize_arrays();
            obj = obj.set_initial_conditions();
            
            obj = obj.run();
        end
        
        function obj = initialize_arrays(obj)
            obj.positive_rotation_measurement = zeros(1, obj.calibration_interations);
            obj.negative_rotation_measurement = zeros(1, obj.calibration_interations);
            obj.positive_rotation_temp_measurement = zeros(1, obj.calibration_interations);
            obj.negative_rotation_temp_measurement = zeros(1, obj.calibration_interations);
        end
        
        function obj = set_initial_conditions(obj)
            obj.w_f = [1.000001, 1.000005, 1.000010];
            obj.w_b = [-2.9000e-04, -1.3400e-04, 3.4700e-04];
            obj.t_k  = @(t) (2e-5 * t - 2e-4) / 3600 / 57.3;
            obj.dt_k = @(dt) 5e-1 * dt / 3600 / 57.3;
            obj.w_sko = 2e-10;
            
            obj.sample_rate = 20;
            obj.dt = 1 / obj.sample_rate;
            obj.calibration_time = 12; 
            obj.calibration_interations = obj.calibration_time * 60 * obj.sample_rate;
            
            obj.gla = zeros(1, obj.calibration_interations) + 1;
            obj.gav = zeros(1, obj.calibration_interations) + (10 / 57.3 / 60);
            
            obj.T  = zeros(1, obj.calibration_interations);
            obj.dT = zeros(1, obj.calibration_interations);
            obj.T(1) = 0;
            obj.dT(1) = 0;
        end
        
        function plt(obj, array, ttl, xlbl, ylbl)
            if obj.plot_grapsh
                figure()
                for i = 1:length(array)
                    t = 1:length(array{i});
                    plot(t, array{i});
                    hold on;
                end
                hold off;
                xlabel(xlbl, 'FontSize', 18);
                ylabel(ylbl, 'FontSize', 18);
                title(ttl, 'FontSize', 20);
                grid on;
%                 saveas(gcf, join(['/media/d/112/kursovaya/model/matlab/nir/', ttl, '.jpg']));     
            end
        end
        
        function [ta, dta] = create_linear_temperature_array(obj, temp_array, time_array)
            if time_array(end) > obj.calibration_time
                disp('Invalid time_array: index out of range');
                ta = 0;
                dta = 0;
            elseif length(temp_array) ~= length(time_array)
                disp('Arrays must be same length')
                ta = 0;
                dta = 0;
            else 
                iterations_count = time_array(end) * 60 * obj.sample_rate;
                ta  = zeros(1, iterations_count);
                dta = zeros(1, iterations_count);
                
                ta(1) = temp_array(1);
                dta(1) = 0;
                
                c = 1; % counter
                for i = 2:iterations_count
                    ta(i)  = ta(i-1) + (temp_array(c + 1) - temp_array(c)) / ...
                        ((time_array(c+1) - time_array(c)) * 60 * obj.sample_rate);
                    dta(i) = ta(i) - ta(i-1);
                    if i == time_array(c+1) * 60 * obj.sample_rate
                        c = c + 1;
                    end
                end
                
                disp('Array created successfully')
            end
        end
        
        function [ta, dta] = create_nolinear_temperature_array(obj)
            cnv = 60 * obj.sample_rate;
            ta  = zeros(1, obj.calibration_interations);
            dta = zeros(1, obj.calibration_interations);
            
            ta(1) = 0;
            dta(1) = 0;
            
            for i = 1:obj.calibration_interations
                if i < 2 * cnv
                    ta(i) = -10 * (i/cnv - 2) ^ 2 + 40;
                    dta(i) = 0.03333 - 1.3888e-5 * i;
                elseif i < 5 * cnv
                    ta(i) = 40;
                    dta(i) = 0;
                elseif i < 10 * cnv
                    ta(i) = 5.555e-7 * (i - 10*20*60) ^ 2 + 20;
                    dta(i) = 1.111e-6 * i - 13.333e-3;
                else
                    ta(i) = 20;
                    dta(i) = 0;        
                end
            end
        end
        
        function val = gyro_mesuare(obj, val_raw)
            if obj.sf_and_bias
                val = normrnd(val_raw, obj.w_sko) * ...
                    obj.w_f(obj.sf_and_bias_level) + obj.w_b(obj.sf_and_bias_level);
            else
                val = normrnd(val_raw, obj.w_sko);
            end
        end
        
        function val = t_gyro_mesuare(obj, val_raw, T, delta_T)
            val = val_raw + (obj.t_k(T) + obj.dt_k(delta_T)) * obj.dt;
        end
        
        function val = t_compensated_gyro_mesuare(obj, val_raw, T, delta_T)
            val = val_raw + (obj.t_k(T) + obj.dt_k(delta_T) - obj.t_k_calculated(T) - obj.dt_k_calculated(delta_T)) * obj.dt;
        end
        
        function obj = log_solution_to_file(obj, file_name)
            if obj.log_to_file
                f = fopen(file_name, 'w+');
                
                fprintf(f, "Mean calibration diff:\n");
                fprintf(f, "|%9f - %9f| = %9f, [rad/s]\n", ...
                    mean(obj.positive_rotation_measurement), ...
                    mean(obj.calibrated_measurement), ...
                    mean(obj.positive_rotation_measurement) - mean(obj.calibrated_measurement));
                
                tk_real  = obj.t_k(40) - obj.t_k(-60)
                dtk_real = obj.dt_k(40) - obj.dt_k(-60)
                
                tk_calc  = obj.t_k_calculated(40) - obj.t_k_calculated(-60)
                dtk_calc = obj.dt_k_calculated(40) - obj.dt_k_calculated(-60)
                
                fprintf(f, "\nReal temperature drift factors:\n");
                fprintf(f, "Temperature:       %9f, [rad/s]\n", tk_real); 
                fprintf(f, "Delta temperature: %9f, [rad/s]\n", dtk_real); 
                
                fprintf(f, "\nCalculated temperature drift factors:\n");
                fprintf(f, "Temperature:       %9f, [rad/s]\n", tk_calc); 
                fprintf(f, "Delta temperature: %9f, [rad/s]\n", dtk_calc); 
                
                fprintf(f, "\nCalculation errors:\n");
                fprintf(f, "Temperature:       |%9f - %9f| = %9f, [rad/s]\n", ...
                    tk_real, tk_calc, abs(tk_real - tk_calc)); 
                fprintf(f, "Delta temperature: |%9f - %9f| = %9f, [rad/s]\n", ...
                    dtk_real, dtk_calc, abs(dtk_real - dtk_calc)); 
                
                abs(tk_real - tk_calc)
                abs(dtk_real - dtk_calc)
                
                fclose(f);
            end
        end
        
        function obj = run(obj)
            % Set values
            [obj.T, obj.dT] = obj.create_nolinear_temperature_array();
            
            obj.plt({obj.T}, 'Temperature', 'iterrations, n', 'temperature, deg');
            obj.plt({obj.dT}, 'Delta temperature', 'iterrations, n', 'delta temperature, deg');
            
            % Set measurmnet in normal conditions and measurements in thermal chamber
            for i = 1:obj.calibration_interations
                % Simulation table rotation clockwise and counterclockwise
                % in normal conditions
                obj.positive_rotation_measurement(i) = obj.gyro_mesuare(obj.gav(i));
                obj.negative_rotation_measurement(i) = obj.gyro_mesuare( - obj.gav(i));
                
                % Simulation table rotation clockwise and counterclockwise
                % in thermal chamber
                obj.positive_rotation_temp_measurement(i) = obj.t_gyro_mesuare(obj.positive_rotation_measurement(i), ...
                    obj.T(i), obj.dT(i));
                obj.negative_rotation_temp_measurement(i) = obj.t_gyro_mesuare(obj.negative_rotation_measurement(i), ...
                    obj.T(i), obj.dT(i));
            end
            
            obj.plt({obj.positive_rotation_measurement}, 'Non-thermal chamber mesuarments', 'Atterations, n', 'Angular speed, rad/s');
            
            obj.plt({obj.positive_rotation_measurement, obj.positive_rotation_temp_measurement}, ...
                'Normal and thermal chamber', 'iterrations, n', 'Angular speedt, rad/s')
            
            % Calculation t_k procedure
            t_40 = 2*20*60:5*20*60;
            t_20 = 10*20*60:12*20*60;
            tk_on_40 = (mean(obj.positive_rotation_temp_measurement(t_40)) + mean(obj.negative_rotation_temp_measurement(t_40))) * obj.sample_rate / 2;
            tk_on_20 = (mean(obj.positive_rotation_temp_measurement(t_20)) + mean(obj.negative_rotation_temp_measurement(t_20))) * obj.sample_rate / 2;
            
            % Former t_k_calculated relation
            A = tk_on_40 - tk_on_20
            B = 20 - 40
            C = 40*tk_on_20 - 20*tk_on_40
            
            obj.t_k_calculated = @(t) (-C - A * t) / B;
            
            mean_y1 =   (mean(obj.positive_rotation_measurement(1:5) + mean(obj.negative_rotation_measurement(1:5)))) * obj.sample_rate / 2;
            mean_y1_t = (mean(obj.positive_rotation_temp_measurement(1:5) + mean(obj.negative_rotation_temp_measurement(1:5)))) * obj.sample_rate / 2;
            
            mean_y2 =   (mean(obj.positive_rotation_measurement(114*obj.sample_rate:119*obj.sample_rate) ...
                + mean(obj.negative_rotation_measurement(114*obj.sample_rate:119*obj.sample_rate)))) * obj.sample_rate / 2;
            mean_y2_t = (mean(obj.positive_rotation_temp_measurement(114*obj.sample_rate:119*obj.sample_rate) + ...
                mean(obj.negative_rotation_temp_measurement(114*obj.sample_rate:119*obj.sample_rate)))) * obj.sample_rate / 2;
            
            
            % Calculation dt_k procedure
            x1 = obj.dT(1);
            x2 = obj.dT(119*obj.sample_rate);
            y1 = mean_y1_t - mean_y1;
            y2 = mean_y2_t - mean_y2;
            
            % Former dt_k_calculated relation
            A = y2 - y1
            B = x1 - x2
            C = x2*y1 - x1*y2
            
            obj.dt_k_calculated = @(dt) (-C - A * dt) / B;
            
            if obj.plot_grapsh
                figure()
                plot([-40, 60],[obj.t_k(-40), obj.t_k(60)]);
                hold on;
                plot([-40, 60],[obj.t_k_calculated(-40), obj.t_k_calculated(60)]);
                hold off;
                xlabel('temperature, deg');
                ylabel('Wdr, rad/s')
                grid on;
                title("T lambda's diff");
                saveas(gcf, join(['/media/d/112/kursovaya/model/matlab/nir/', 'T_diff', '.jpg']));
                
                figure()
                plot([-40, 60],[obj.dt_k(-40), obj.dt_k(60)]);
                hold on;
                plot([-40, 60],[obj.dt_k_calculated(-40), obj.dt_k_calculated(60)]);
                hold off;
                xlabel('temperature, deg');
                ylabel('Wdr, rad/s')
                grid on;
                title("dT lambda's diff");
                saveas(gcf, join(['/media/d/112/kursovaya/model/matlab/nir/', 'dT_diff', '.jpg']));
            end
            
            % Form tempearture calibrated measurements array
            for i = 1:obj.calibration_interations
                obj.calibrated_measurement(i) = obj.t_compensated_gyro_mesuare(obj.gyro_mesuare(obj.gav(i)), ...
                    obj.T(i), obj.dT(i));
            end
            
            obj.plt({obj.positive_rotation_measurement, obj.positive_rotation_temp_measurement, obj.calibrated_measurement}, ...
                'Compare normal and calibrated measurements', 'iterrations, n', 'Angular speed, rad/s')

            obj.plt({obj.positive_rotation_measurement - obj.calibrated_measurement}, ...
                'Calibration error', 'iterrations, n', 'oangular drift, rad/s')
            
            obj.log_solution_to_file('/media/d/112/kursovaya/model/matlab/calibration_results.txt');
        end
    end
end