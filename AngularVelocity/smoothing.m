function smoothing
    % import csv table to smooth
    file_name_in = '11.17_speedpt1.csv';
    data = readtable(file_name_in);
    
    [s_l, ] = size(data(:,1));
    speed_len = s_l(1);
    
    times = table2array(data(:,1)).';
    speeds = table2array(data(:,2)).';
    
    file_name_out = '11.17_speedpt1_smooth.csv';
    
    %   Laplace smoothing
    num_smoothing = 7; %try for 10 or less
    for n=1:num_smoothing
       for i=2:speed_len-1
           speeds(i) = 0.5*(speeds(i-1)+speeds(i+1));
       end
    end
    
    % Graphing
    posfigure = figure;
    axes1 = axes('Parent',posfigure,'YGrid','on','XGrid','on','FontSize',14);
    box(axes1,'on');
    hold(axes1,'on');
    ylabel('Speed (rpm)');
    xlabel('Time (sec)');
    title('Rotational Velocity of Button'); 
    plot(times,speeds);
    
    %save last values
    csvwrite(file_name_out,[times',speeds']);
end