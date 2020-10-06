% clear all;

settings = ModelSettings(97*60, 2);
disp('Algorithm setting:');
settings = settings.set_debug_mode_flag(1);                  % Poka ne nujen :(
settings = settings.set_display_solutions_flag(0);           % Plot solution on screen
settings = settings.set_subplot_print_flag(0);               % Plot solution as one subplot on screen, if flag = true
                                                             % else Plot solution as several graphs (Vx(t), R(t), La(t), etc ...)
settings = settings.set_log_algorithm_solutions_flag(0);     % Save solution errors in file
settings = settings.set_plot_trajectoey_simulation_flag(1);  % Plot trajectory simulation prams (Vx, Ay, Wz) on screen
disp('Algorithm configured successfully.');

obj = Bins(settings);
obj = obj.run();