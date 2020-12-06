function frame_analysis
    %  Finds rpm of button as a fn of time

    %% Preprocessing
    % FPS data and converting video to a list of greyscale images
    
    clear all;
    close all;
    fps         = 1000;       % camera frame rate (frames/sec)
    dt          = 1/fps;       % Time between successive frames 

    csvfilename = '11.17_speedpt2.csv';

    % Read the video and convert the video frames to grayscale images 
    % adjust video name here
    video = VideoReader('11.17.20_edit.mpeg');
    
    % converts video to greyscale
    % k is number of frames in video
    k = 0;
    while hasFrame(video)
        k = k + 1;
        image_list(:,:,k) = rgb2gray(readFrame(video));
    end  
    
    % Vector of time values 
    time = 0:dt:(k-1)/fps;  
    %initial empty vector of theta values, which will be populated upon
    %first analysis of all frames
    button_thetas = zeros(1, length(time)); 
    
    %initial settings
    start_time = 0;
    end_time = time(length(time));
 
    %flag that turns true when user satisfied
    analysis_complete = false;
    %if alternate method of calculating angular velocity is active
    alt_method = false;
    
    %% Analysis
    while ~analysis_complete
        % 1) get angle of maximum intensity for each frame
        % 2) derive angular velocity from angular position
        %how we calculate angle depends on method
        if ~alt_method
            button_thetas = get_angle(image_list, button_thetas, start_time, end_time, dt);
        else
            button_thetas = get_angle_alt(image_list, button_thetas, start_time, end_time, dt);
        end
        button_angvels = calc_angvel(button_thetas, dt);
    
        %% Graphing
        posfigure = figure;
        axes1 = axes('Parent',posfigure,'YGrid','on','XGrid','on','FontSize',14);
        box(axes1,'on');
        hold(axes1,'on');
        ylabel('Speed (rpm)');
        xlabel('Time (sec)');
        title('Rotational Velocity of Button'); 
        plot(time,button_angvels);
        
        %% Recalculate Prompt
        % Asks user if they would like to recalculate a certain time range
        prompt = ['Would you like to recalculate a portion of the graph? Type "Yes" or "No"',... 
            '\nType "Yes*" to recaulcate with alternate method',...
            '\nEnter:'];
        retry = input(prompt, 's'); 

        % if they want to recalculate from time range, get new time range
        % from them
        if strcmp(retry, 'Yes')
            alt_method = false;
            
            prompt = 'Enter start time:';
            start_time = input(prompt); 
            
            prompt = 'Enter end time:';
            end_time = input(prompt); 
        elseif strcmp(retry, 'Yes*')
            alt_method = true;
            
            prompt = 'Enter start time:';
            start_time = input(prompt); 
            
            prompt = 'Enter end time:';
            end_time = input(prompt); 
        else
            % if not, turn flag true
            analysis_complete = true;
        end
        
        close all;
        % Save the position-v-time data to a csv file
        csvwrite(csvfilename,[time',button_angvels', button_thetas']);
    end 
    
end

%% Given image list, returns list of theta values of black dot
function theta_list = get_angle(image_list, theta_list, start_time, end_time, dt)
    %pre-condition: image_list and theta_list are array of same dimensions
    
    % 0) Calculate the frames which we start and end
    start_frame = round(start_time/dt) + 1;
    end_frame = round(end_time/dt) + 1;

    % 1) Selecting circle coordinates
    [x, y, rad] = select_circle(image_list(:,:,start_frame));
    
    % 2) generate an array of points based on those circle coordinates
    [xpts, ypts] = generate_pts(x,y,rad);
    
    %plot the points that we're measuring on top of first frame
    figure(1);
    clf;
    imshow(image_list(:,:,start_frame));
    hold on;
    plot(xpts,ypts, 'r+', 'MarkerSize', 3, 'LineWidth', 1);
    
    % 3) For each frame, find angle with most black
    % get the subset of image list corresponding to appropriate time
    % interval
    image_clip = image_list(:,:,start_frame:end_frame);
    adjusted_thetas = find_angle(xpts, ypts, image_clip);
    
    % 4) replace old theta values for new theta values return
    theta_list(start_frame:end_frame) = adjusted_thetas;
end

%% Gets circle sub-image by clicking on two points
function [x, y, rad] = select_circle(image)
    %  Function to ask the user to select a sub-image

    %  image:  Image to be displayed to the user
    %   x,y, rad:  center and radius of circle
    figure(1);
    clf;
    imshow(image);
    hold on;

    title('Select center of circle, then a point on edge of circle.');
    fprintf('\n')
    [xc_raw,yc_raw] = ginput(2);
    
    %round coordinates
    xc = round(xc_raw);
    yc = round(yc_raw);
    
    x = xc(1);
    y = yc(1);
    rad = sqrt(abs((xc(2)-xc(1))).^2 + abs((yc(2)-yc(1))).^2);
    
    %close figure
    close;
end

%% Creates 2d array of points corresponding to parts of circle
function [xpts, ypts] = generate_pts(x, y, rad)
    % x,y are center of circle, rad is radius
    
    num_theta = 720;
    width_arr = [0.96, 0.98, 1, 1.02, 1.04]; %how much on either side or circle to go
    
    thetas = linspace(0,2*pi*(num_theta-1)/num_theta, num_theta)';
    for circle_num = 1:length(width_arr)
        xpts(:,circle_num) = round(x+rad*cos(thetas)*width_arr(circle_num));
        ypts(:,circle_num) = round(y-rad*sin(thetas)*width_arr(circle_num)); 
    end
end

%% For each frame, gets theta of black dot
function thetas = find_angle(xpts, ypts, image_list)
    %gets number of frames in video
    num_frames = size(image_list, 3);
    thetas = zeros(1, num_frames);
    
    %matches angles to number of anglers in xpts
    theta_vals = linspace(0, 2*pi*(length(xpts)-1)/length(xpts), length(xpts));
    
    for frame = 1:num_frames
        % go through each point, extracting brightness from that point
        theta_brightness = zeros(1, length(xpts));
        for theta = 1:length(xpts)
            for circle = 1:size(xpts,2)
                col = xpts(theta, circle);
                row = ypts(theta, circle);
                theta_brightness(1, theta) = theta_brightness(1, theta) + image_list(row, col, frame);
            end
        end
        
        %make theta average of itself and 7 surrounding values 
        %on both sides
        %note to self: maybe this portion extract out to separate function
        theta_temp = zeros(1, length(xpts));
        for i = 1:length(theta_brightness)
            angs = [mod(i-8,length(theta_brightness)), mod(i-7,length(theta_brightness)), ...
                mod(i-6,length(theta_brightness)), mod(i-5,length(theta_brightness)), ...
                mod(i-4,length(theta_brightness)), mod(i-3,length(theta_brightness)), ...
                mod(i-2,length(theta_brightness)), mod(i-1,length(theta_brightness)), ...
                mod(i,length(theta_brightness)), mod(i+1,length(theta_brightness)), ...
                mod(i+2,length(theta_brightness)), mod(i+3,length(theta_brightness)), ...
                mod(i+4,length(theta_brightness)), mod(i+5,length(theta_brightness)), ...
                mod(i+6,length(theta_brightness))]+1;
            
            for ang_index = 1:length(angs)
                theta_temp(i) = theta_temp(i) + theta_brightness(angs(ang_index));
            end
        end
        theta_brightness = theta_temp;
        
        %find index of lowest brightness angle, then match it with theta for
        %that frame
        [ma, index_of_least_bright_theta] = max(theta_brightness); %CHANGED TO MAX 
        thetas(1, frame) = theta_vals(index_of_least_bright_theta);
    end
end

%% Given image list, returns list of theta values of black dot
% Alternate method!
function theta_list = get_angle_alt(image_list, theta_list, start_time, end_time, dt)
    %pre-condition: image_list and theta_list are array of same dimensions
    
    % 0) Calculate the frames which we start and end
    start_frame = round(start_time/dt) + 1;
    end_frame = round(end_time/dt) + 1;
    
    % 1) Ask user to calibrate a light and dark color using first frame
    [light, dark] = calibrate_lightdark(image_list(:,:,start_frame));
    
    % 2) Ask user to select rectangle to analyze
    [x1, y1, x2, y2] = select_rect(image_list(:,:,start_frame));
    
    % 3) Show rectangle that we will analyze
    figure(1);
    clf;
    imshow(image_list(:,:,start_frame));
    hold on;
    
    plot([x1,x2,x2,x1,x1],[y1,y1,y2,y2,y1],'g-','linew',2);
    % 3b) Ask user what angle this rectangle corresponds with (ie pi/2)
    angle_prompt = 'Enter start angle this rectangle corresponds with:';
    ref_angle = input(angle_prompt); 
    
    close;
    % 3c) Ask the direction in which button rotating for that cycle
    dir_prompt = 'Type "Yes" if rotation clockwise, "No" otherwise:';
    clockwise_dir = strcmp(input(dir_prompt, 's'), 'Yes'); 
    
    % 4) Go through each frame of clip and determine if dot present
    image_clip = image_list(:,:,start_frame:end_frame);
    dot_present_list = dot_present(x1, y1, x2, y2, image_clip, 180);%%hardcoded value for now
    
    % 5) Convert list of booleans if dot present to list of angles
    % first, get start angle one frame before and after time frame
    start_angle = theta_list(start_frame-1);
    end_angle = theta_list(end_frame+1);
    adjusted_thetas = calc_angle(ref_angle, dot_present_list, clockwise_dir, start_angle, end_angle);
    
    % 6) replace old angles with new ones and return
    theta_list(start_frame:end_frame) = adjusted_thetas;
end

%% Calibrates the dark and light pixel value of image
function [light, dark] = calibrate_lightdark(image)
%{
    %image corner coords are x1,y1 and x2,y2
    figure(1);
    clf;
    imshow(image);
    hold on;

    title('Select a light pixel, then a dark pixel');
    fprintf('\n')
    [xc_raw,yc_raw] = ginput(2);
    
    %round coordinates
    xc = round(xc_raw);
    yc = round(yc_raw);
    
    light = image(xc(1), yc(1)); %COULD BE ISSUE HERE WITH DIMS
    dark = image(xc(2), yc(2));
    
    %close figure
    close;
%}
    %hardcoded for now
    light = 230;
    dark = 50;
end

%% Returns list of angles corresponding to booleeans of whether dot is present in rectangle
function angle_list = calc_angle(ref_angle, dot_present_list, clockwise_dir, start_angle, end_angle)
    % function assumes never 1 full rotation between frames, that
    % consectuive 1s means dot has not left frame
    angle_list = zeros(1, length(dot_present_list));
    
    % 1) deals with first few frames, when we go from "start_angle" to
    % ref_angle in some # frames
    % if clockwise, ref_angle must be higher than start angle
    % if ccw, ref_angle must be lower than start angle
    first_1 = find(dot_present_list, 1);
    if first_1 == 1
        %this is case where dot present in first frame, set first angle to
        %ref_angle
        first_angs = ref_angle;
    else
        if(clockwise_dir)
            if(ref_angle > start_angle)
                first_angs = mod(start_angle+2*pi:(ref_angle-(start_angle+2*pi))/first_1:ref_angle, 2*pi);
            else
                first_angs = mod(start_angle:(ref_angle-start_angle)/first_1:ref_angle, 2*pi);
            end
            first_angs = first_angs(2:length(first_angs));
        else
            if(ref_angle < start_angle)
                first_angs = mod(start_angle:((ref_angle+2*pi)-start_angle)/first_1:(ref_angle+2*pi), 2*pi);
            else
                first_angs = mod(start_angle:(ref_angle-start_angle)/first_1:ref_angle, 2*pi);
            end
            first_angs = first_angs(2:length(first_angs));
        end
    end
    angle_list(1:length(first_angs)) = first_angs;
    
    % 2) second set of frames, when we are revolving a lot
    counter = 1;
    zero_switch = false;
    for i = first_1 + 1:length(dot_present_list)
        %go through array, go until there is a 0, flip switch
        if(zero_switch && dot_present_list(i) == 1) 
            % have reached start of next rotation
            % go back and calc angles in angle_list
            if(clockwise_dir)
                angs = mod(ref_angle+2*pi:-2*pi/counter:ref_angle, 2*pi);
                angs = angs(2:length(angs));
            else
                angs = mod(ref_angle:2*pi/counter:ref_angle+2*pi, 2*pi);
                angs = angs(2:length(angs));
            end
            angle_list(i-counter+1:i) = angs;
            
            %reset counter & switch
            zero_switch = false;
            counter = 1;
        elseif(~zero_switch && dot_present_list(i) == 0)
            zero_switch = 1;
            counter = counter + 1;
        else
            counter = counter + 1;
        end
    end
    
    % 3) last set of frames, when we go from ref angle to end angle in
    % some # of frames
    if(clockwise_dir)
        if(ref_angle < end_angle)
            ref_angle = ref_angle + 2*pi;
        end
        last_angs = mod(ref_angle:(end_angle-ref_angle)/counter:end_angle, 2*pi);
        last_angs = last_angs(2:length(last_angs)-1);
    else
        if(ref_angle > end_angle)
            end_angle = end_angle + 2*pi;
        end
        last_angs = mod(ref_angle:(end_angle-ref_angle)/counter:end_angle, 2*pi);
        last_angs = last_angs(2:length(last_angs)-1);
    end
    angle_list(length(angle_list)-counter+2:length(angle_list)) = last_angs;
    
end

%% Gets rect of image by clicking on two corner points
function [x1, y1, x2, y2] = select_rect(image)
    %image corner coords are x1,y1 and x2,y2
    figure(1);
    clf;
    imshow(image);
    hold on;

    title('Select top left, then bottom right corner of the rectangle of interest.');
    fprintf('\n')
    [xc_raw,yc_raw] = ginput(2);
    
    %round coordinates
    xc = round(xc_raw);
    yc = round(yc_raw);
    
    x1 = xc(1);
    y1 = yc(1);
    x2 = xc(2);
    y2 = yc(2);
    
    %close figure
    close;
end

%%Given list of images, return list of booleans corresponding to if dot is
%%present in that rectangle bounded by x1, y1, x2, y2
function dot_bools = dot_present(x1, y1, x2, y2, images, dark_thresh)
    [M,N,n_frames] = size(images);
    dot_bools = zeros(1, n_frames);
    
    for f=1:n_frames 
        %get the current frame
        cur_frame = images(:,:,f);
        
        %accumulator vars
        total_val = 0;
        num_pix = 0;
        
        for col=x1:x2
            for row=y1:y2
                %need this unit value for conversion from 8 bit val
                total_val = total_val + uint32(cur_frame(row,col)); %might run into issue here
                num_pix = num_pix + 1;
            end
        end
        
        avg_val = total_val/num_pix;
        
        % Converting to booleans 
        dot_present = avg_val >= dark_thresh; %CHANGED FOR DRILL
        dot_bools(f) = dot_present;
    end
end

%% Given list of thetas (in radians) and dt between frames, returns angular velocity in rpm
function angvels = calc_angvel(thetas, dt)
    % Assumptions: button never rotates more than a half-rotation (pi rads)
    % in 2 successive frames
    
    num_frames = length(thetas);
    
    %first frame speed is auto set to 0
    angvels(1) = 0;
    
    for frame = 2:num_frames
        delta_theta = thetas(frame) - thetas(frame - 1);
        if(delta_theta < -pi)
            delta_theta = delta_theta + 2*pi;
        elseif (delta_theta > pi)
            delta_theta = delta_theta - 2*pi;
        end
        angvels(frame) = delta_theta / dt; 
    end
    
    %converts angular velocity array from rad/sec to rpm
    angvels = angvels * 60 / (2*pi);
end