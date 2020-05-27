total_plots = 8;
delimeters = [];
for i = 1:total_plots
    if (mod(total_plots, i) == 0)
        delimeters = [delimeters, i];
    end
end

delimeters
columns_count = delimeters(1 + round(length(delimeters) / 2))
rows_count = total_plots / columns_count