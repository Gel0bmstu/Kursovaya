classdef Bins    
    properties
        % Sensors
        % ---------------------------------------------------------------------------------------------------------------        
        ax; ay; az; % Accelerometers
        wx; wy; wz; % Gyroscopes
        
        % Sensors errors
        % ---------------------------------------------------------------------------------------------------------------
        ax_error_function = @(value, psi, teta, gamma) value * sin(teta / 57.3);
        ay_error_function = @(value, psi, teta, gamma) value * cos(teta / 57.3) * cos(gamma / 57.3);
        az_error_function = @(value, psi, teta, gamma) value * cos(teta / 57.3) * sin(gamma / 57.3);
        
        Ux_err = @(psi, teta, gamma) U * cos(phi) * cos(psi / 57.3) + U * sin(phi) * sin(teta / 57.3);
        Uy_err = @(psi, teta, gamma) U * cos(phi) * (-cos(gamma / 57.3) * cos(psi / 57.3) * sin(teta / 57.3) ...
            + sin(gamma / 57.3) * sin(psi / 57.3)) + U * sin(phi) * cos(gamma / 57.3) * cos(teta / 57.3);
        Uz_err = @(psi, teta, gamma) U * cos(phi) * (sin(gamma / 57.3) * cos(psi / 57.3) * sin(teta / 57.3) ...
            + sin(gamma / 57.3) * sin(psi / 57.3)) + U * sin(phi) * sin(gamma / 57.3) * cos(teta / 57.3);

        axis_alligment_error = 1e-5 % Error of SSK & NSSK axis aligment [rad]
        
        % Initial exhibition error
        psi_err   = 1.5 / 57.3; % Exhibition error by X [ãðàä]
        teta_err  = 1.0 / 57.3; % Exhibition error by Z [ãðàä]
        gamma_err = 2.0 / 57.3; % Exhibition error by Y [ãðàä]
        
        % Solutions
        % ---------------------------------------------------------------------------------------------------------------   
        Ax_n; Ay_n; Az_n; % Object accelerations in NSSK
        Wx_n; Wy_n; Wz_n; % Object angular velocity in NSSK
        Vx_n; Vy_n; Vz_n; % Object linear velocity in NSSK
        Fx_n; Fy_n; Fz_n; % Object angular vels incriments in NSSK
        
        Ax_b; Ay_b; Az_b; % Object accelerations in SSK
        Wx_b; Wy_b; Wz_b; % Object angular velocity in SSK
        Vx_b; Vy_b; Vz_b; % Object linear velocity in SSK
        Fx_b; Fy_b; Fz_b; % Object angular vels incriments in SSK
        
        psi, teta, gamma, h; % Object angles in NSSK
        
        % Files
        % ---------------------------------------------------------------------------------------------------------------   
        axels_calibration_results_file; % File which will be stored gyros calibration resualts
        gyros_calibration_results_file; % File which will be stored axels calibration resualts
        algorithm_results_error_file;   % File which will be stored algorithm results errors
        
        % Rotation quaternions
        % ----------------------------------------------------------------------------------------------------------------
        L; % Quaternion characterizing the transition from SSK to NSSK
        G; % Quaternion ...
        
        % Angular velocity
        % ----------------------------------------------------------------------------------------------------------------
        
        % Linear velocity
        % ----------------------------------------------------------------------------------------------------------------
        
        % Other variables
        % ----------------------------------------------------------------------------------------------------------------
        settings; % Variable that contains setings of program
       
        % Containers for plots
        plot_tile;
        plot_parameter;
        plot_x_label;
        plot_y_label;
        plot_solution_path;
    end
    
    methods
        function obj = Bins(settings)
            % Apply BINS settings
            obj.settings = settings;
            
            % Initialize quaternions
            obj.L = [1, 0, 0, 0];
            obj.G = [1, - settings.U * cos(settings.phi) * settings.dt, - settings.U * sin(settings.phi) * settings.dt, 0];
            
            % Sensors initialization
            obj.ax = Accelerometer(settings.axel_min_scale_factor, settings.axel_max_scale_factor, settings.axel_bias, settings.axel_sko);
            obj.ay = Accelerometer(settings.axel_min_scale_factor, settings.axel_max_scale_factor, settings.axel_bias, settings.axel_sko);
            obj.az = Accelerometer(settings.axel_min_scale_factor, settings.axel_max_scale_factor, settings.axel_bias, settings.axel_sko);

            obj.wx = Gyroscope(settings.axel_min_scale_factor, settings.axel_max_scale_factor, settings.axel_bias, settings.axel_sko);
            obj.wy = Gyroscope(settings.axel_min_scale_factor, settings.axel_max_scale_factor, settings.axel_bias, settings.axel_sko);
            obj.wz = Gyroscope(settings.axel_min_scale_factor, settings.axel_max_scale_factor, settings.axel_bias, settings.axel_sko);
            
            % Fill solution arrays with zeros
            obj.Ax_n = zeros(1, settings.simulation_time);
            obj.Ay_n = zeros(1, settings.simulation_time);
            obj.Az_n = zeros(1, settings.simulation_time);
            
            obj.Ax_n(1) = 0;
            obj.Ay_n(1) = 0;
            obj.Az_n(1) = 0;
            
            obj.Vx_n = zeros(1, settings.simulation_time);
            obj.Vy_n = zeros(1, settings.simulation_time);
            obj.Vz_n = zeros(1, settings.simulation_time);

        end
        
        function obj = calibration(obj)
            % Setting calibration angles for sensors
            axels_calibrating_angenls = [  
                % X
                0, 90, 0;  %1
                0, 270, 0; %2

                % Y
                90, 0, 0;  %3
                270, 0, 0; %4

                % Z
                0, 0, 0;   %5
                0, 180, 0; %6
                ];

            gyros_calibrating_angenls = [  
                % N and Up
                0, 0, 0;    %1
                0, 180, 0;  %2

                % E
                90, 0, 0;   %3
                90, 0, 180; %4
                ];
            
            % Set accelerometer readings on turntable
            gx_1 = - obj.settings.gc * cos(axels_calibrating_angenls(1, 1) / 57.3) * sin(axels_calibrating_angenls(1, 2) / 57.3);
            gy_1 = obj.settings.gc * sin(axels_calibrating_angenls(3, 1) / 57.3);
            gz_1 = obj.settings.gc * cos(axels_calibrating_angenls(5, 1) / 57.3) * cos(axels_calibrating_angenls(5, 2) / 57.3);
            
            gx_2 = - obj.settings.gc * cos(axels_calibrating_angenls(2, 1) / 57.3) * sin(axels_calibrating_angenls(2, 2) / 57.3);
            gy_2 = obj.settings.gc * sin(axels_calibrating_angenls(4, 1) / 57.3);
            gz_2 = obj.settings.gc * cos(axels_calibrating_angenls(6, 1) / 57.3) * cos(axels_calibrating_angenls(6, 2) / 57.3);
            
            % Create array 
            % Fill in the array with zeros
            cAx_1 = zeros(1, obj.settings.calibration_iterations);
            cAx_2 = zeros(1, obj.settings.calibration_iterations);
            cAy_1 = zeros(1, obj.settings.calibration_iterations);
            cAy_2 = zeros(1, obj.settings.calibration_iterations);
            cAz_1 = zeros(1, obj.settings.calibration_iterations);
            cAz_2 = zeros(1, obj.settings.calibration_iterations);

            for i=1:obj.settings.calibration_iterations
                cAx_1(i) = obj.ax.measure(gx_1);
                cAx_2(i) = obj.ax.measure(gx_2);
                
                cAy_1(i) = obj.ay.measure(gy_1);
                cAy_2(i) = obj.ay.measure(gy_2);

                cAz_1(i) = obj.az.measure(gz_1);
                cAz_2(i) = obj.az.measure(gz_2);
                
        %         cWx_1(i) = normrnd(Ux(gyros_calibrating_angenls(1, 1), gyros_calibrating_angenls(1, 2), gyros_calibrating_angenls(1, 3)), wSigma) * Wz_true_scale_f + Wz_true_bias;
        %         cWx_2(i) = normrnd(Ux(gyros_calibrating_angenls(2, 1), gyros_calibrating_angenls(2, 2), gyros_calibrating_angenls(2, 3)), wSigma) * Wz_true_scale_f + Wz_true_bias;
        %         
        %         cWy_1(i) = normrnd(Uy(gyros_calibrating_angenls(1, 1), gyros_calibrating_angenls(1, 2), gyros_calibrating_angenls(1, 3)), wSigma) * Wz_true_scale_f + Wz_true_bias;
        %         cWy_2(i) = normrnd(Uy(gyros_calibrating_angenls(2, 1), gyros_calibrating_angenls(2, 2), gyros_calibrating_angenls(2, 3)), wSigma) * Wz_true_scale_f + Wz_true_bias;
        %         
        %         cWz_1(i) = normrnd(Uz(gyros_calibrating_angenls(3, 1), gyros_calibrating_angenls(3, 2), gyros_calibrating_angenls(3, 3)), wSigma) * Wz_true_scale_f + Wz_true_bias;
        %         cWz_2(i) = normrnd(Uz(gyros_calibrating_angenls(4, 1), gyros_calibrating_angenls(4, 2), gyros_calibrating_angenls(4, 3)), wSigma) * Wz_true_scale_f + Wz_true_bias;
            end
            
            % Calculate mean g for all axels
            meanAx1 = mean(cAx_1);
            meanAx2 = mean(cAx_2);

            meanAy1 = mean(cAy_1);
            meanAy2 = mean(cAy_2);

            meanAz1 = mean(cAz_1);
            meanAz2 = mean(cAz_2);

            % Calculate mean g for all axels
            Ax_z_bias = (meanAx1 + meanAx2) / 2;
            Ay_z_bias = (meanAy1 + meanAy2) / 2;
            Az_z_bias = (meanAz1 + meanAz2) / 2;

            Ax_scale_f = (meanAx2 - meanAx1) / 2 / obj.settings.gc;
            Ay_scale_f = (meanAy1 - meanAy2) / 2 / obj.settings.gc;
            Az_scale_f = (meanAz1 - meanAz2) / 2 / obj.settings.gc;
            
            obj.ax = obj.ax.set_calibrated_params(Ax_z_bias, Ax_scale_f);
            obj.ay = obj.ay.set_calibrated_params(Ay_z_bias, Ay_scale_f);
            obj.az = obj.az.set_calibrated_params(Az_z_bias, Az_scale_f);
            
            if (obj.settings.log_calibration_results_flag)
                obj.axels_calibration_results_file = fopen(obj.settings.axels_calibration_results_file_path, 'w+');
                obj.gyros_calibration_results_file = fopen(obj.settings.gyros_calibration_results_file_path, 'w+');
                
                fprintf(obj.axels_calibration_results_file, "%s\n", "Real axels biases:");
                fprintf(obj.axels_calibration_results_file, "Ax: %6f\n",   obj.ax.bias);
                fprintf(obj.axels_calibration_results_file, "Ay: %6f\n",   obj.ay.bias);
                fprintf(obj.axels_calibration_results_file, "Az: %6f\n\n", obj.az.bias);

                fprintf(obj.axels_calibration_results_file, "%s\n", "Real axels scale factor:");
                fprintf(obj.axels_calibration_results_file, "Ax: %6f\n",   obj.ax.scale_factor);
                fprintf(obj.axels_calibration_results_file, "Ay: %6f\n",   obj.ay.scale_factor);
                fprintf(obj.axels_calibration_results_file, "Az: %6f\n\n", obj.az.scale_factor);
                
                fprintf(obj.axels_calibration_results_file, "%s\n", "Calculated axels biases:");
                fprintf(obj.axels_calibration_results_file, "Ax: %6f\n",   obj.ax.calculated_bias);
                fprintf(obj.axels_calibration_results_file, "Ay: %6f\n",   obj.ay.calculated_bias);
                fprintf(obj.axels_calibration_results_file, "Az: %6f\n\n", obj.az.calculated_bias);

                fprintf(obj.axels_calibration_results_file, "%s\n", "Calculated axels scale factor:");
                fprintf(obj.axels_calibration_results_file, "Ax: %6f\n",   obj.ax.calculated_scale_factor);
                fprintf(obj.axels_calibration_results_file, "Ay: %6f\n",   obj.ay.calculated_scale_factor);
                fprintf(obj.axels_calibration_results_file, "Az: %6f\n\n", obj.az.calculated_scale_factor);

                fprintf(obj.axels_calibration_results_file, "%s\n", "--------------------------------------------------");

                fprintf(obj.axels_calibration_results_file, "%s\n", "Bias calibration error:");
                fprintf(obj.axels_calibration_results_file, "Ax: |%6f - %6f| = %6f\n", ...
                    obj.ax.bias, obj.ax.calculated_bias, abs(obj.ax.bias - obj.ax.calculated_bias));
                fprintf(obj.axels_calibration_results_file, "Ay: |%6f - %6f| = %6f\n", ...
                    obj.ay.bias, obj.ay.calculated_bias, abs(obj.ay.bias - obj.ay.calculated_bias));
                fprintf(obj.axels_calibration_results_file, "Az: |%6f - %6f| = %6f\n\n", ...
                    obj.az.bias, obj.az.calculated_bias, abs(obj.az.bias - obj.az.calculated_bias));

                fprintf(obj.axels_calibration_results_file, "%s\n", "Scale factor calibration error:");
                fprintf(obj.axels_calibration_results_file, "Ax: |%6f - %6f| = %6f\n", ...
                    obj.ax.scale_factor, obj.ax.calculated_scale_factor, abs(obj.ax.scale_factor - obj.ax.calculated_scale_factor));
                fprintf(obj.axels_calibration_results_file, "Ay: |%6f - %6f| = %6f\n", ...
                    obj.ay.scale_factor, obj.ay.calculated_scale_factor, abs(obj.ay.scale_factor - obj.ay.calculated_scale_factor));
                fprintf(obj.axels_calibration_results_file, "Az: |%6f - %6f| = %6f\n\n", ...
                    obj.az.scale_factor, obj.az.calculated_scale_factor, abs(obj.az.scale_factor - obj.az.calculated_scale_factor));
                
                fclose(obj.axels_calibration_results_file);
                fclose(obj.gyros_calibration_results_file);
            end
            
        %     % Íàõîäèì ñðåäíåå çíà÷åíèå ñêîðîñòåé Çåìëè U
        %     meanWx1 = mean(cWx_1);
        %     meanWx2 = mean(cWx_2);
        %     
        %     meanWy1 = mean(cWy_1);
        %     meanWy2 = mean(cWy_2);
        %     
        %     meanWz1 = mean(cWz_1);
        %     meanWz2 = mean(cWz_2);
        %     
        %     % Íàõîäèì ñìåùåíèå íóëÿ èìàñøòàáíûé êîýôôèöèåíò ãèðîñêîïîâ
        %       Wx_z_bias = (meanWx1 + meanWx2) / 2;
        %     Wy_z_bias = (meanWy1 + meanWy2) / 2;
        %     Wz_z_bias = (meanWz1 + meanWz2) / 2;
        %     
        %     Wx_scale_f = (meanWx2 - meanWx1) / 2 / Ux(0, 0, 0);
        %     Wy_scale_f = (meanWy1 - meanWy2) / 2 / Uy(0, 0, 0);
        %     Wz_scale_f = (meanWz1 - meanWz2) / 2 / Uz(0, 0, 0);
        %     
        %     fprintf(obj.gyros_calibration_results_file, "%s\n", "Âû÷èñëåííîå ñìåùåíèå íóëÿ ãèðîñêîïîâ:");
        %     fprintf(obj.gyros_calibration_results_file, "Ax: %6f\n", Ax_z_bias);
        %     fprintf(obj.gyros_calibration_results_file, "Ay: %6f\n", Ay_z_bias);
        %     fprintf(obj.gyros_calibration_results_file, "Az: %6f\n\n", Az_z_bias);
        % 
        %     fprintf(obj.gyros_calibration_results_file, "%s\n", "Âû÷åñëåííûå ìàñøòàáíûå êîýôôèöèåíòû ãèðîñêîïîâ:");
        %     fprintf(obj.gyros_calibration_results_file, "Ax: %6f\n", Ax_scale_f);
        %     fprintf(obj.gyros_calibration_results_file, "Ay: %6f\n", Ay_scale_f);
        %     fprintf(obj.gyros_calibration_results_file, "Az: %6f\n\n", Az_scale_f);
        %     
        %     fprintf(obj.gyros_calibration_results_file, "%s\n", "--------------------------------------------------");
        %     
        %     fprintf(obj.gyros_calibration_results_file, "%s\n", "Îøèáêà êàëèáðîâêè ñìåùåíèÿ íóëÿ ãèðîñêîïîâ:");
        %     fprintf(obj.gyros_calibration_results_file, "Ax: |%6f - %6f| = %6f\n", Ax_true_bias, Ax_z_bias, abs(Ax_true_bias - Ax_z_bias));
        %     fprintf(obj.gyros_calibration_results_file, "Ay: |%6f - %6f| = %6f\n", Ay_true_bias, Ay_z_bias, abs(Ax_true_bias - Ay_z_bias));
        %     fprintf(obj.gyros_calibration_results_file, "Ay: |%6f - %6f| = %6f\n\n", Az_true_bias, Az_z_bias, abs(Ax_true_bias - Az_z_bias));
        %     
        %     fprintf(obj.gyros_calibration_results_file, "%s\n", "Îøèáêà êàëèáðîâêè ìàñøòàáíîãî êîýôôèöèåíòà àêñåëåðîìåòðîâ:");
        %     fprintf(obj.gyros_calibration_results_file, "Ax: |%6f - %6f| = %6f\n", Ax_true_scale_f, Ax_scale_f, abs(Ax_true_scale_f - Ax_scale_f));
        %     fprintf(obj.gyros_calibration_results_file, "Ay: |%6f - %6f| = %6f\n", Ay_true_scale_f, Ay_scale_f, abs(Ay_true_scale_f - Ay_scale_f));
        %     fprintf(obj.gyros_calibration_results_file, "Ay: |%6f - %6f| = %6f\n\n", Az_true_scale_f, Az_scale_f, abs(Az_true_scale_f - Az_scale_f));
        %     
        end
        
        function obj = run(obj)
            % Sensors clibtarion step
            if (obj.settings.calibration_mode_flag)
                obj.calibration();
            end
            
            % Solutions arrays initialization step
            obj.initialize_solutions_arrays_();
            
            % Main cycle
            for i=2:obj.settings.simulation_iterations
                
            end
            
            % Log solution results & errors in file
            if (obj.settings.log_algorithm_solutions_flag)
                obj.log_solution_to_file_();
            end
            
            % Print graphs
            if (obj.settings.display_solutions_flag)
                obj.set_plot_parameters_();
                
                if (obj.settings.subplot_print_flag)
                    % Print solution on one subplot
                    obj.print_solutions_sub_plot_();
                else
                    % Print solution on several plots
                    obj.print_solution_single_plot_();
                end
            end
        end
    end
    
    methods (Access = 'private')
        function obj = initialize_solutions_arrays_(obj)
            obj.Ax_n = zeros(1, obj.settings.simulation_time);
            obj.Ay_n = zeros(1, obj.settings.simulation_time);
            obj.Az_n = zeros(1, obj.settings.simulation_time);
            
            obj.Ax_n(1) = 0;
            obj.Ay_n(1) = 0;
            obj.Az_n(1) = 0;

            obj.Vx_n = zeros(1, obj.settings.simulation_time);
            obj.Vy_n = zeros(1, obj.settings.simulation_time);
            obj.Vz_n = zeros(1, obj.settings.simulation_time);
            
            obj.Vx_n(1) = 0;
            obj.Vy_n(1) = 0;
            obj.Vz_n(1) = 0;
            
            obj.Wx_n = zeros(1, obj.settings.simulation_time);
            obj.Wy_n = zeros(1, obj.settings.simulation_time);
            obj.Wz_n = zeros(1, obj.settings.simulation_time);
            
            obj.Wx_n(1) = 0;
            obj.Wy_n(1) = 0;
            obj.Wz_n(1) = 0;
            
            obj.Fx_n = zeros(1, obj.settings.simulation_time);
            obj.Fy_n = zeros(1, obj.settings.simulation_time);
            obj.Fz_n = zeros(1, obj.settings.simulation_time);

            obj.Fx_n(1) = 0;
            obj.Fy_n(1) = 0;
            obj.Fz_n(1) = 0;
        end
        function mtr = make_4x4_skew_matrinx_from_vector_(v)
            mtr = [ 0, -v(1), -v(2),  -v(3);
                  v(1),  0,   v(3),  -v(2);
                  v(2), -v(3),   0,   v(1);
                  v(3),  v(2), v(1),  0];
        end
        function obj = log_solution_to_file_(obj)
            fe = fopen(obj.settings.algorithm_results_errors_file_path, 'w+');

            fprintf(fe, "%s\n", "Âåêòîð g, äëÿ ïðîâåðêè óõîäà ïî ñêîðîñòÿì:");
            fprintf(fe, 'Ax - gx: %6f\nAy - gy: %6f\nAz - gz: %6f\n\n', An(1) - gn(1), An(2) - gn(2), An(3) - gn(3));

            fprintf(fe, 'gx * %f = %6f\ngy * %f = %6f\ngz * %f = %6f\n\n', simulation_time * dt, (An(1) - gn(1)) * simulation_time * dt, simulation_time * dt, (An(2) - gn(2)) * simulation_time * dt, simulation_time * dt, (An(3) - gn(3)) * simulation_time * dt);

            fprintf(fe, "%s\n", "Îøèáêà àëãîðèòìà ÁÈÍÑ ïî ñêîðîñòÿì:");
            fprintf(fe, "Vx: |%6f - %6f| = %6f [m/^2]\n", 0, Vx_n(simulation_time), abs(0 - Vx_n(simulation_time)));
            fprintf(fe, "Vy: |%6f - %6f| = %6f [ì/ñ^2]\n", 0, Vy_n(simulation_time), abs(0 - Vy_n(simulation_time)));
            fprintf(fe, "Vz: |%6f - %6f| = %6f [ì/ñ^2]\n\n", 0, Vz_n(simulation_time), abs(0 - Vz_n(simulation_time)));

            fprintf(fe, "%s\n", "Îøèáêà àëãîðèòìà ÁÈÍÑ ïî óãëàì:");
            fprintf(fe, "psi:   |%6f - %6f| = %6f [ãðàä]\n", U * sin(phi) * simulation_time * dt * 57.3, psi(simulation_time) * 57.3, abs(U * sin(phi) * simulation_time * dt * 57.3 - psi(simulation_time) * 57.3));
            fprintf(fe, "teta:  |%6f - %6f| = %6f [ãðàä]\n", 0, teta(simulation_time) * 57.3, abs(0 - teta(simulation_time) * 57.3));
            fprintf(fe, "gamma: |%6f - %6f| = %6f [ãðàä]\n\n", U * cos(phi) * simulation_time * dt * 57.3, gamma(simulation_time) * 57.3, abs(U * cos(phi) * simulation_time * dt * 57.3 - gamma(simulation_time) * 57.3));

        end
        function obj = set_plot_parameters_(obj)
            obj.plot_parameter = [obj.Vx_n,       obj.Vy_n,       obj.Vz_n,       obj.psi,      obj.teta,      obj.gamma,      obj.h];
            obj.plot_tile      = ['Vx_n',         'Vy_n',         'Vz_n',         'Psi',        'Teta',        'Gamma',        'H'];
            obj.plot_x_label   = ['t, [s]',       't, [s]',       't, [s]',       't, [s]',     't, [s]',      't, [s]',       't, [s]'];
            obj.plot_y_label   = ['Vx_n, [ms/s]', 'Vy_n, [ms/s]', 'Vz_n, [ms/s]', 'Psi, [rad]', 'Teta, [rad]', 'Gamma, [rad]', 'H, [m]'];
            obj.plot_solution_path = [];
            for i = 1:length(obj.plot_tile)
                obj.plot_solution_path = [obj.plot_solution_path, join(['/', obj.plot_tile, '.jpg'])];
            end
        end
        function obj = print_solution_single_plot_(obj)
            for i = 1:length(obj.plot_parameter)
                figure(1);
                plot(obj.settings.simulation_time, obj.plot_parameter(i));
                set(gca, 'fontsize', obj.settings.font_size);
                set(gcf, 'Position', get(0, 'Screensize'));
                title(obj.plot_tile(i));
                xlabel(obj.plot_x_label(i));
                ylabel(obj.plot_y_label(i));
                grid on;
                saveas(gcf, join([solution_folder_name, obj.plot_solution_path]));                
            end
        end
        function obj = print_solutions_sub_plot_(obj)
            total_plots = length(obj.plot_parameter);
            delimeters = [];
            for i = 1:total_plots
                if (mod(total_plots, i) == 0)
                    delimeters = [delimeters, i];
                end
            end
            
            columns_count = delimeters(1 + round(length(delimeters) / 2));
            rows_count = total_plots / columns_count;
            
            for i = 1:total_plots
                subplot(rows_count, columns_count, i);
                plot(obj.settings.simulation_time, obj.plot_parameter(i));
                set(gca, 'fontsize', obj.settings.font_size);
                title(obj.plot_tile(i));
                xlabel(obj.plot_x_label(i));
                ylabel(obj.plot_y_label(i));
                grid on;
            end
        end
        function obj = generate_trajectory_(obj)
            
        end
        function obj = calculate_ort_and_nav_params_(obj, i)
            gn = quatrotate(L, g);

            % Get accelerations in SSK frame
            obj.Ax_b = obj.ax.measure(4);
            obj.Ay_b = obj.ay.measure(4);
            obj.Az_b = obj.az.measure(4);

            Ab = [obj.Ax_b; obj.Ay_b; obj.Az_b];

            Fx_b = normrnd(Ux_err(psi_err, teta_err, gamma_err) * dt, wSigma);
            Fy_b = normrnd(Uy_err(psi_err, teta_err, gamma_err) * dt, wSigma);
            Fz_b = normrnd(Uz_err(psi_err, teta_err, gamma_err) * dt, wSigma);

            Fb = [Fx_b; Fy_b; Fz_b];

            % Gyros compensation
            % ----

            % Axels compensation
            % ----

            % Recalculate G quat
            A = make4SkewMatrix([U * cos(phi) * dt; U * sin(phi) * dt; 0]);
            dG = E + 0.5 * A + 0.25 * A^2;
            G = (dG * G')';

            % Recalculate main quat
            A = make4SkewMatrix(Fb);
            dL = E + 0.5 * A + 0.25 * A^2;
            L = (dL * L')';

            % Project IMU info to NSSK
            An = quatrotate(L, Ab');
            Fn = quatrotate(L, Fb');

            M = quat2rotm(L);

            m0 = sqrt(M(3,1)^2 + M(3,3)^2);

            gamma(i)  = gamma(1) + atan2(M(3,2), m0);
            teta(i)   = teta(1)  + atan2(M(1,2),M(2,2));
            psi(i)    = psi(1)   - atan2(M(3,1),M(3,3));

            Vx_n(i) = Vx_n(i-1) + (An(1) - gn(1)) * dt;
            Vy_n(i) = Vy_n(i-1) + (An(2) - gn(2)) * dt;
            Vz_n(i) = Vz_n(i-1) + (An(3) - gn(3)) * dt;

            h(i) = h(i-1) + Vy_n(i) * dt;
        end
    end
end

