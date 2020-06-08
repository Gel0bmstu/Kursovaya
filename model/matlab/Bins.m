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
        An; Ax_n; Ay_n; Az_n; % Object accelerations in NSSK
        Wn; Wx_n; Wy_n; Wz_n; % Object angular velocity in NSSK
        Vn; Vx_n; Vy_n; Vz_n; % Object linear velocity in NSSK
        Fn; Fx_n; Fy_n; Fz_n; % Object angular vels incriments in NSSK
        Sn; Sx_n; Sy_n; Sz_n; % Object coordinates in NSSK 
        
        Ab; Ax_b; Ay_b; Az_b; % Object accelerations in SSK
        Wb; Wx_b; Wy_b; Wz_b; % Object angular velocity in SSK
        Vb; Vx_b; Vy_b; Vz_b; % Object linear velocity in SSK
        Fb; Fx_b; Fy_b; Fz_b; % Object angular vels incriments in SSK
        
        Sg; Sx_g; Sy_g; Sz_g; % Object coordinates in GSK
        Sr; Sx_r; Sy_r; Sz_r; % Real ISS coordinates in GSK
        
        psi; teta; gamma; h; % Object angles in NSSK
        phi; la;             % Object trajectory in GSK

        
        % Files
        % ---------------------------------------------------------------------------------------------------------------   
        axels_calibration_results_file; % File which will be stored gyros calibration resualts
        gyros_calibration_results_file; % File which will be stored axels calibration resualts
        algorithm_results_error_file;   % File which will be stored algorithm results errors
        
        % Rotation quaternions
        % ----------------------------------------------------------------------------------------------------------------
        L; % Quaternion characterizing the transition from SSK to NSSK
        G; % Quaternion between NSSK and GSK
        
        % Generated trajectory parameters
        % ----------------------------------------------------------------------------------------------------------------    
        k;            % Turnover speed/radius factor
                
        R_generated;  % Generated distance from the 
                      % center of the earth to the spacecraft [m]
        fi_generated; % Generated latitude                    [rad]
        la_generated; % Generated longitude                   [rad]
        
        Vx_generated; Vy_generated; Vz_generated; % Generated liner velocity in SSK
        Wx_generated; Wy_generated; Wz_generated; % Generated angular velocity in SSK
        Ax_generated; Ay_generated; Az_generated; % Generated spaceship acceleration in SSK
        
        % Real trajectory parameters
        % ----------------------------------------------------------------------------------------------------------------        
        R_real;  % Real distance from the
                 % center of the earth to the spacecraft [m]
        fi_real; % Real latitude                         [rad]
        la_real; % Real longitude                        [rad]
        t_real;  % UTC time at each point of trajectory  [date]
        
        Vx_real; Vy_real; Vz_real; % Real liner velocity in SSK
        Wx_real; Wy_real; Wz_real; % Real angular velocity in SSK
        Ax_real; Ay_real; Az_real; % Real spaceship acceleration in SSK
        
        % Angular velocity
        % ----------------------------------------------------------------------------------------------------------------
        
        % Linear velocity
        % ----------------------------------------------------------------------------------------------------------------
        
        % Other variables
        % ----------------------------------------------------------------------------------------------------------------
        settings; % Variable that contains setings of program
        c                    = 1; % Counter for main loop
        gpv;
        
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
            
            obj = obj.initialize_model_arrays_();
            
            obj.phi(1) = obj.settings.phi;
            obj.la(1)  = obj.settings.la;
            
            obj.psi   = 36.76 / 57.3;
            obj.teta  = - 38.45 / 57.3;
            obj.gamma = - 1.36  / 57.3;
            
            P = [cos(obj.psi/2), 0, sin(obj.psi/2), 0];
            Q = [cos(obj.teta/2), 0, 0, sin(obj.teta/2)];
            R = [cos(obj.gamma/2), sin(obj.gamma/2), 0, 0];
            
            % Initialize quaternions
            obj.L = [1, 0, 0, 0];
            obj.G = [cos(obj.psi/2) * cos(obj.teta/2) * cos(obj.gamma/2), sin(obj.gamma/2), sin(obj.psi/2), sin(obj.teta/2)];
%             obj.G = quatmultiply(P, quatmultiply(Q, R));
%             obj.G = [1, - settings.U * cos(settings.phi) * settings.dt, - settings.U * sin(settings.phi) * settings.dt, 0];
            
            % Sensors initialization
            obj.ax = Accelerometer(settings.axel_min_scale_factor, settings.axel_max_scale_factor, settings.axel_bias, settings.axel_sko);
            obj.ay = Accelerometer(settings.axel_min_scale_factor, settings.axel_max_scale_factor, settings.axel_bias, settings.axel_sko);
            obj.az = Accelerometer(settings.axel_min_scale_factor, settings.axel_max_scale_factor, settings.axel_bias, settings.axel_sko);

            obj.wx = Gyroscope(settings.axel_min_scale_factor, settings.axel_max_scale_factor, settings.axel_bias, settings.axel_sko);
            obj.wy = Gyroscope(settings.axel_min_scale_factor, settings.axel_max_scale_factor, settings.axel_bias, settings.axel_sko);
            obj.wz = Gyroscope(settings.axel_min_scale_factor, settings.axel_max_scale_factor, settings.axel_bias, settings.axel_sko);
            
            % Set real trajectory params
            obj.k = 2 * pi / settings.revolution_time;
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
                obj = obj.log_accels_calibration_results_to_file_();
                obj = obj.log_gyros_calibration_results_to_file_();
                
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
                obj = obj.calibration();
            end
            
            obj = obj.get_real_telemetry_();
            
            % Ideal telemetry generation step
            obj = obj.generate_telemetry_();
            
            % Main cycle
            obj.Sy_n(1) = obj.R_generated(1);
            for i=2:obj.settings.simulation_iterations
                obj = obj.calculate_ort_and_nav_params_(i);
            end
            
            disp('Calculations is over successfully!')
            
            obj = obj.print_calculated_trajectory_();
            obj = obj.print_trajectory_on_map_();
            
            % Log solution results & errors in file
            if (obj.settings.log_algorithm_solutions_flag)
%                 obj = obj.log_solution_to_file_();
            end
            
            % Print graphs
            if (obj.settings.display_solutions_flag)
                obj = obj.set_plot_parameters_();
                
                if (obj.settings.subplot_print_flag)
                    % Print solution on one subplot
                    obj = obj.print_solutions_sub_plot_();
                else
                    % Print solution on several plots
                    obj = obj.print_solution_single_plot_();
                end
            end
        end
    end
    
    methods (Access = 'private')
        % Solution (main) function
        function obj = calculate_ort_and_nav_params_(obj, i)
            % Get current g array in SSK and NSSK
            gn = [0, obj.get_current_g_(obj.R_generated(i)), 0];

            % Get accelerations in SSK frame
            obj.Ax_b(i) = obj.ax.measure(obj.Ax_generated(i));
            obj.Ay_b(i) = obj.ay.measure(obj.Ay_generated(i));
            obj.Az_b(i) = obj.az.measure(obj.Az_generated(i));

            Ab = [obj.Ax_b(i); obj.Ay_b(i); obj.Az_b(i)];

            obj.Fx_b(i) = obj.wx.measure(obj.Wx_generated(i));        
            obj.Fy_b(i) = obj.wx.measure(obj.Wy_generated(i));
            obj.Fz_b(i) = obj.wx.measure(obj.Wz_generated(i));

            Fb = [obj.Fx_b(i); obj.Fy_b(i); obj.Fz_b(i)];

            obj.Vx_b(i) = obj.Vx_generated(i);
            obj.Vy_b(i) = obj.Vy_generated(i);
            obj.Vz_b(i) = obj.Vz_generated(i);

            Vb = [obj.Vx_b(i), obj.Vy_b(i), obj.Vz_b(i)];

            % Gyros compensation
            % ----

            % Axels compensation
            % ----

            % Recalculate G quat
%             A = obj.make_4x4_skew_matrinx_from_vector_([... 
%                 obj.settings.U * cos(obj.settings.phi) * obj.settings.dt, ...
%                 obj.settings.U * sin(obj.settings.phi) * obj.settings.dt, ...
%                 0]);
            A = obj.make_4x4_skew_matrinx_from_vector_([ ...
                obj.settings.U * sin(obj.teta) * cos(obj.psi)   * obj.settings.dt, ...
                obj.settings.U * cos(obj.teta) * cos(obj.gamma) * obj.settings.dt, ...
                obj.settings.U * sin(obj.psi)  * sin(obj.gamma) * obj.settings.dt]);
            dG = obj.settings.E + 0.5 * A + 0.25 * A^2;
            obj.G = (dG * obj.G')';

            % Recalculate main quat
            A = obj.make_4x4_skew_matrinx_from_vector_(Fb);
            dL = obj.settings.E + 0.5 * A + 0.25 * A^2;
            obj.L = (dL * obj.L')';

            % Project IMU info to NSSK
            An = quatrotate(obj.L, Ab');
            Fn = quatrotate(obj.L, Fb');
            Vn = quatrotate(obj.L, Vb);

            M = quat2rotm(obj.L);

            m0 = sqrt(M(3,1)^2 + M(3,3)^2);

%             gamma(i)  = gamma(1) + atan2(M(3,2), m0);
%             teta(i)   = teta(1)  + atan2(M(1,2), M(2,2));
%             psi(i)    = psi(1)   - atan2(M(3,1), M(3,3));
% 
%             obj.Vx_n(i) = obj.Vx_n(i-1) + (An(1) - gn(1)) * obj.settings.dt;
%             obj.Vy_n(i) = obj.Vy_n(i-1) + (An(2) - gn(2)) * obj.settings.dt;
%             obj.Vz_n(i) = obj.Vz_n(i-1) + (An(3) - gn(3)) * obj.settings.dt;
            
            obj.Sx_n(i) = obj.Sx_n(i - 1) + Vn(1) * obj.settings.dt;
            obj.Sy_n(i) = obj.Sy_n(i - 1) + Vn(2) * obj.settings.dt;
            obj.Sz_n(i) = obj.Sz_n(i - 1) + Vn(3) * obj.settings.dt;
            
            if (mod(i, obj.settings.sample_rate * 60) == 0) && (i ~= obj.settings.sample_rate * 60)
                c = ceil(i / obj.settings.sample_rate / 60);
                Sn = [obj.Sx_n(i), obj.Sy_n(i), obj.Sz_n(i)];

                Sg = quatrotate(obj.G, Sn);

%                 lla = eci2lla([Sg(1), Sg(3), Sg(2)], obj.t_real(c, :), 'IAU-2000/2006');
% 
%                 obj.Sx_g(c) = lla(1);
%                 obj.Sy_g(c) = lla(3);
%                 obj.Sz_g(c) = lla(2);

                obj.Sx_g(c) = Sg(1);
                obj.Sy_g(c) = Sg(2);
                obj.Sz_g(c) = Sg(3);
                
                lla = eci2lla([Sg(1), Sg(3), Sg(2) - obj.settings.Rad], obj.t_real(c, :), 'IAU-2000/2006');

                obj.phi(c) = lla(1);
                obj.la(c)  = lla(2);
                obj.h(c)   = lla(3);
            end
            
            obj = obj.print_progress_bar_(i);
        end
        
        % Model functions
        function obj = initialize_model_arrays_(obj)
            % GSK solutions initialization
            obj.phi = zeros(1, obj.settings.revolution_time / obj.settings.sample_rate / 60);
            obj.la  = zeros(1, obj.settings.revolution_time / obj.settings.sample_rate / 60);
            obj.h   = zeros(1, obj.settings.revolution_time / obj.settings.sample_rate / 60);
            
            % NSSK params initialization
            obj.Ax_n = zeros(1, obj.settings.simulation_iterations);
            obj.Ay_n = zeros(1, obj.settings.simulation_iterations);
            obj.Az_n = zeros(1, obj.settings.simulation_iterations);
            
            obj.Ax_n(1) = obj.settings.Ax_n_0;
            obj.Ay_n(1) = obj.settings.Ay_n_0;
            obj.Az_n(1) = obj.settings.Az_n_0;

            obj.Vx_n = zeros(1, obj.settings.simulation_iterations);
            obj.Vy_n = zeros(1, obj.settings.simulation_iterations);
            obj.Vz_n = zeros(1, obj.settings.simulation_iterations);
            
            obj.Vx_n(1) = obj.settings.Vx_n_0;
            obj.Vy_n(1) = obj.settings.Vy_n_0;
            obj.Vz_n(1) = obj.settings.Vz_n_0;
            
            obj.Wx_n = zeros(1, obj.settings.simulation_iterations);
            obj.Wy_n = zeros(1, obj.settings.simulation_iterations);
            obj.Wz_n = zeros(1, obj.settings.simulation_iterations);
            
            obj.Wx_n(1) = obj.settings.Wx_n_0;
            obj.Wy_n(1) = obj.settings.Wy_n_0;
            obj.Wz_n(1) = obj.settings.Wz_n_0;
            
            obj.Fx_n = zeros(1, obj.settings.simulation_iterations);
            obj.Fy_n = zeros(1, obj.settings.simulation_iterations);
            obj.Fz_n = zeros(1, obj.settings.simulation_iterations);

            obj.Fx_n(1) = obj.settings.Fx_n_0;
            obj.Fy_n(1) = obj.settings.Fy_n_0;
            obj.Fz_n(1) = obj.settings.Fz_n_0;

            obj.Sx_n = zeros(1, obj.settings.simulation_iterations);
            obj.Sy_n = zeros(1, obj.settings.simulation_iterations);
            obj.Sz_n = zeros(1, obj.settings.simulation_iterations);

            obj.Sx_n(1) = 0;
            obj.Sy_n(1) = obj.settings.Rad + obj.settings.h_min + (obj.settings.h_max - obj.settings.h_min) / 2;
            obj.Sz_n(1) = 0; 

            obj.Sx_r = zeros(1, obj.settings.revolution_time / obj.settings.sample_rate / 60);
            obj.Sy_r = zeros(1, obj.settings.revolution_time / obj.settings.sample_rate / 60);
            obj.Sz_r = zeros(1, obj.settings.revolution_time / obj.settings.sample_rate / 60);
            
            obj.Sx_g = zeros(1, obj.settings.revolution_time / obj.settings.sample_rate / 60);
            obj.Sy_g = zeros(1, obj.settings.revolution_time / obj.settings.sample_rate / 60);
            obj.Sz_g = zeros(1, obj.settings.revolution_time / obj.settings.sample_rate / 60);

            eci = lla2eci([51.55, 36.76, 0],[2020, 05, 24, 18, 03, 00]);
            
            obj.Sx_g(1) = eci(1);
            obj.Sy_g(1) = obj.settings.Rad + obj.settings.h_min + (obj.settings.h_max - obj.settings.h_min) / 2;
            obj.Sz_g(1) = eci(2); 
            
            % SSK dynamic params initialization
            obj.Ax_b = zeros(1, obj.settings.simulation_iterations);
            obj.Ay_b = zeros(1, obj.settings.simulation_iterations);
            obj.Az_b = zeros(1, obj.settings.simulation_iterations);
            
            obj.Ax_b(1) = obj.settings.Ax_b_0;
            obj.Ay_b(1) = obj.settings.Ay_b_0;
            obj.Az_b(1) = obj.settings.Az_b_0;

            obj.Vx_b = zeros(1, obj.settings.simulation_iterations);
            obj.Vy_b = zeros(1, obj.settings.simulation_iterations);
            obj.Vz_b = zeros(1, obj.settings.simulation_iterations);
            
            obj.Vx_b(1) = obj.settings.Vx_b_0;
            obj.Vy_b(1) = obj.settings.Vy_b_0;
            obj.Vz_b(1) = obj.settings.Vz_b_0;
            
            obj.Wx_b = zeros(1, obj.settings.simulation_iterations);
            obj.Wy_b = zeros(1, obj.settings.simulation_iterations);
            obj.Wz_b = zeros(1, obj.settings.simulation_iterations);
            
            obj.Wx_b(1) = obj.settings.Wx_b_0;
            obj.Wy_b(1) = obj.settings.Wy_b_0;
            obj.Wz_b(1) = obj.settings.Wz_b_0;
            
            obj.Fx_b = zeros(1, obj.settings.simulation_iterations);
            obj.Fy_b = zeros(1, obj.settings.simulation_iterations);
            obj.Fz_b = zeros(1, obj.settings.simulation_iterations);

            obj.Fx_b(1) = obj.settings.Fx_b_0;
            obj.Fy_b(1) = obj.settings.Fy_b_0;
            obj.Fz_b(1) = obj.settings.Fz_b_0;

            % Fill generated dynamic parameters arrays with zeros
            obj.R_generated  = zeros(1, obj.settings.simulation_iterations);
            obj.fi_generated = zeros(1, obj.settings.simulation_iterations);
            obj.la_generated = zeros(1, obj.settings.simulation_iterations);

            obj.Ax_generated = zeros(1, obj.settings.simulation_iterations);
            obj.Ay_generated = zeros(1, obj.settings.simulation_iterations);
            obj.Az_generated = zeros(1, obj.settings.simulation_iterations);
            
            obj.Vx_generated = zeros(1, obj.settings.simulation_iterations);
            obj.Vy_generated = zeros(1, obj.settings.simulation_iterations);
            obj.Vz_generated = zeros(1, obj.settings.simulation_iterations);
            
            obj.Wx_generated = zeros(1, obj.settings.simulation_iterations);
            obj.Wy_generated = zeros(1, obj.settings.simulation_iterations);
            obj.Wz_generated = zeros(1, obj.settings.simulation_iterations);
            
            % Fill real dynamic parameters arrays with zeros
            obj.R_real  = zeros(1, obj.settings.revolution_time / obj.settings.sample_rate / 60);
            obj.fi_real = zeros(1, obj.settings.revolution_time / obj.settings.sample_rate / 60);
            obj.la_real = zeros(1, obj.settings.revolution_time / obj.settings.sample_rate / 60);
            obj.t_real  = zeros(obj.settings.revolution_time / obj.settings.sample_rate / 60, 6);
            
            obj.Ax_real = zeros(1, obj.settings.simulation_iterations);
            obj.Ay_real = zeros(1, obj.settings.simulation_iterations);
            obj.Az_real = zeros(1, obj.settings.simulation_iterations);
            
            obj.Vx_real = zeros(1, obj.settings.simulation_iterations);
            obj.Vy_real = zeros(1, obj.settings.simulation_iterations);
            obj.Vz_real = zeros(1, obj.settings.simulation_iterations);
            
            obj.Wx_real = zeros(1, obj.settings.simulation_iterations);
            obj.Wy_real = zeros(1, obj.settings.simulation_iterations);
            obj.Wz_real = zeros(1, obj.settings.simulation_iterations);
        end
        function mtr = make_4x4_skew_matrinx_from_vector_(obj, v)
            mtr = [ 0, -v(1), -v(2),  -v(3);
                  v(1),  0,   v(3),  -v(2);
                  v(2), -v(3),   0,   v(1);
                  v(3),  v(2), v(1),  0];
        end
        function mtr = make_3x3_skew_matrinx_from_vector_(obj, v)
            mtr = [0 -v(3) v(2);
                   v(3) 0 -v(1);
                  -v(2) v(3) 0];
        end
        function g   = get_current_g_(obj, R)
            g = obj.settings.M * obj.settings.G / (R ^ 2);
        end
        
        % File log functions
        function obj = log_solution_to_file_(obj)
%             fe = fopen(obj.settings.algorithm_results_errors_file_path, 'w+');

%             fprintf(fe, "%s\n", "Âåêòîð g, äëÿ ïðîâåðêè óõîäà ïî ñêîðîñòÿì:");
%             fprintf(fe, 'Ax - gx: %6f\nAy - gy: %6f\nAz - gz: %6f\n\n', obj.An(1) - obj.settings.gn(1), obj.An(2) - obj.settings.gn(2), obj.An(3) - obj.settings.gn(3));

%             fprintf(fe, 'gx * %f = %6f\ngy * %f = %6f\ngz * %f = %6f\n\n', obj.settings.simulation_iterations, (obj.An(1) - obj.settings.gn(1)) * simulation_time * dt, ... 
%                 obj.settings.simulation_iterations, (obj.An(2) - obj.settings.gn(2)) * obj.settings.simulation_iterations, obj.settings.simulation_iterations, (obj.An(3) - obj.settings.gn(3)) * obj.settings.simulation_iterations);

%             fprintf(fe, "%s\n", "Îøèáêà àëãîðèòìà ÁÈÍÑ ïî ñêîðîñòÿì:");
%             fprintf(fe, "Vx: |%6f - %6f| = %6f [m/^2]\n", 0, obj.Vx_n(obj.settings.simulation_time), abs(0 - obj.Vx_n(obj.settings.simulation_time)));
%             fprintf(fe, "Vy: |%6f - %6f| = %6f [ì/ñ^2]\n", 0, obj.Vy_n(obj.settings.simulation_time), abs(0 - obj.Vy_n(obj.settings.simulation_time)));
%             fprintf(fe, "Vz: |%6f - %6f| = %6f [ì/ñ^2]\n\n", 0, obj.Vz_n(obj.settings.simulation_time), abs(0 - obj.Vz_n(obj.settings.simulation_time)));

%             fprintf(fe, "%s\n", "Îøèáêà àëãîðèòìà ÁÈÍÑ ïî óãëàì:");
%             fprintf(fe, "psi:   |%6f - %6f| = %6f [ãðàä]\n", U * sin(obj.phi) * obj.settings.simulation_time * dt * 57.3, obj.psi(obj.settings.simulation_time) * 57.3, abs(obj.settings.U * sin(obj.phi) * obj.settings.simulation_time * dt * 57.3 - obj.psi(obj.settings.simulation_time) * 57.3));
%             fprintf(fe, "teta:  |%6f - %6f| = %6f [ãðàä]\n", 0, obj.teta(obj.settings.simulation_time) * 57.3, abs(0 - obj.teta(obj.settings.simulation_time) * 57.3));
%             fprintf(fe, "gamma: |%6f - %6f| = %6f [ãðàä]\n\n", U * cos(obj.phi) * obj.settings.simulation_time * dt * 57.3, obj.gamma(obj.settings.simulation_time) * 57.3, abs(obj.settings.U * cos(obj.phi) * obj.settings.simulation_time * dt * 57.3 - obj.gamma(obj.settings.simulation_time) * 57.3));
            fprintf('Solution errors were written to the file: %s\n', obj.settings.algorithm_results_errors_file_path)
        end
        function obj = log_accels_calibration_results_to_file_(obj)
            obj.axels_calibration_results_file = fopen(obj.settings.axels_calibration_results_file_path, 'w+');

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
        end
        function obj = log_gyros_calibration_results_to_file_(obj)
            obj.gyros_calibration_results_file = fopen(obj.settings.gyros_calibration_results_file_path, 'w+');
        end
        
        % Plot functions
        function obj = set_plot_parameters_(obj)
            obj.plot_parameter = {obj.Ax_b,       obj.Ay_b,       obj.Az_b,       };%obj.psi,      obj.teta,      obj.gamma,      obj.h};
            obj.plot_tile      = {'Vx_n',         'Vy_n',         'Vz_n',         'Psi',        'Teta',        'Gamma',        'H'};
            obj.plot_x_label   = {'t, [s]',       't, [s]',       't, [s]',       't, [s]',     't, [s]',      't, [s]',       't, [s]'};
            obj.plot_y_label   = {'Vx_n, [ms/s]', 'Vy_n, [ms/s]', 'Vz_n, [ms/s]', 'Psi, [rad]', 'Teta, [rad]', 'Gamma, [rad]', 'H, [m]'};
            obj.plot_solution_path = {};
            for i = 1:length(obj.plot_parameter)
                obj.plot_solution_path{i} = join(['/', obj.plot_tile{i}, '.jpg']);
            end
        end
        function obj = print_solution_single_plot_(obj)
            for i = 1:length(obj.plot_parameter)
                figure();
                plot(1:obj.settings.simulation_time, obj.plot_parameter{i}(obj.settings.sample_rate:obj.settings.sample_rate:end));
                set(gca, 'fontsize', obj.settings.font_size);
                set(gcf, 'Position', get(0, 'Screensize'));
                title(obj.plot_tile{i});
                xlabel(obj.plot_x_label{i});
                ylabel(obj.plot_y_label{i});
                grid on;
                saveas(gcf, join([obj.settings.solution_folder_name, obj.plot_solution_path{i}]));                
            end
        end
        function obj = print_solutions_sub_plot_(obj)
            disp('Plotting solution in single plot ...');
            % Calculate rows and cols count of subplot
            total_plots = length(obj.plot_parameter);
            delimeters = [];
            for i = 1:total_plots
                if (mod(total_plots, i) == 0)
                    delimeters = [delimeters, i];
                end
            end
            
            columns_count = delimeters(1 + round(length(delimeters) / 2));
            rows_count = total_plots / columns_count;
            
            % Plot graphs
            figure();
            for i = 1:total_plots
                subplot(rows_count, columns_count, i);
                plot(1:obj.settings.simulation_time, obj.plot_parameter{i}(obj.settings.sample_rate:obj.settings.sample_rate:end));
                set(gca, 'fontsize', obj.settings.font_size);
                title(obj.plot_tile(i));
                xlabel(obj.plot_x_label(i));
                ylabel(obj.plot_y_label(i));
                grid on;
            end
        end
        function obj = print_calculated_trajectory_(obj)
            figure();
            hold on;
            plot3(obj.Sx_n, obj.Sy_n, obj.Sz_n);
            grid on;
            hold on;
            plot3(obj.Sx_g, obj.Sz_g, obj.Sy_g);
            grid on;
            hold on;
            plot3(obj.Sx_r, obj.Sz_r, obj.Sy_r);
            grid on;
            xlabel('X');
            ylabel('Y');
            zlabel('Z');
            legend(...
                'Object trajectory in NSSK', ...
                'Object trajectory in GSK', ...
                'ISS trajectory in GSK')
        end
        function obj = print_trajectory_on_map_(obj)
            figure();
            geoplot(obj.gpv.Latitude,obj.gpv.Longitude, '+','LineWidth',2)
            hold on;
            geoplot(obj.phi, obj.la, '+r','LineWidth',2)
        end
        
        % Telemetry funtions
        function obj = generate_telemetry_(obj)
            disp('Generating ideal (real) telemetry ...')
            h_med = (obj.settings.h_max - obj.settings.h_min) / 2;
            v_med = (obj.settings.v_max - obj.settings.v_min) / 2;
            
            % Spacecraft angular velocity of rotation about his 'Z' axis [rad/sec]
            wx = 2 * pi / obj.settings.revolution_iterations; 
            
            for i = 1:obj.settings.simulation_iterations
                obj.R_generated(i)  = obj.settings.Rad + obj.settings.h_min + h_med + h_med * sin(i * obj.k);

                obj.Vz_generated(i) = obj.settings.v_min + v_med - v_med * sin(i * obj.k);
                obj.Ay_generated(i) = obj.get_current_g_(obj.R_generated(i));
                obj.Wx_generated(i) = - wx;
            end
            disp('Telemetry generated successfully.')
            
            if (obj.settings.plot_trajectoey_simulation_flag)
                figure(1);
                subplot(1, 2, 1);
                plot(1:obj.settings.simulation_iterations, obj.R_generated);
                title('Distance from the center of the Earth to the spacecraft')
                xlabel('time, [sec]');
                ylabel('R, [m]');
                grid on;
                subplot(1, 2, 2);
                plot(1:obj.settings.simulation_iterations, obj.Vz_generated);
                title('Linear speed of the spacecraft')
                xlabel('time, [sec]');
                ylabel('Linear vel (X axis), [m/sec]');
                grid on;
            end
        end
        % Open real telemetry    
        function obj = get_real_telemetry_(obj)
            disp('Try to get ISS telemetry ...');
            telemetry_file = fopen(obj.settings.path_to_telemetry_file);
            
            i = 1;
            while ~feof(telemetry_file)
                string = fgetl(telemetry_file);
                line = split(string);
                a = split(line{1}, '/');
                b = split(line{2}, ':');
                obj.t_real(i, :) = str2num(join(["20" + a{1}, a{2:end}, b{:}], " "));
                obj.R_real(i)  = str2double(line{8}) * obj.settings.Rad;
                obj.fi_real(i) = str2double(line{6});
                obj.la_real(i) = str2double(line{7});
                
                eci = lla2eci([obj.fi_real(i), obj.la_real(i), obj.R_real(i) - obj.settings.Rad], obj.t_real(i, :));
                
                obj.Sx_r(i) = eci(1);
                obj.Sy_r(i) = eci(2);
                obj.Sz_r(i) = eci(3);
                
                i = i + 1;
            end
            obj.gpv = geopoint();
            obj.gpv = append(obj.gpv, obj.fi_real, obj.la_real);
            disp('ISS telemetry getted successfully.');
        end
        
        % Other functions
        function obj = print_progress_bar_(obj, i)
            if (mod(i, floor(obj.settings.simulation_iterations / obj.settings.progress_bar_width)) == 0)
                fprintf(repmat('\b', 1, obj.settings.progress_line_size));
                obj.settings.progress_bar(obj.settings.progress_bar_counter) = '#';
                obj.settings.progress_line_size = fprintf('%s\n%d of %d iterations is over (%.2d %%).\n', ...
                    obj.settings.progress_bar, i, obj.settings.simulation_iterations, floor((obj.settings.progress_bar_counter - 1 ) * 100 / obj.settings.progress_bar_width));
                obj.settings.progress_bar_counter = obj.settings.progress_bar_counter + 1;
            end
        end
    end
end

