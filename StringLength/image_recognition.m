function image_recognition
%  Track width of handle as function of time
%
    clear all;
    close all;

    csvfilename = 'string_length1.csv';

%   Read the video and convert the video frames to grayscale images 
    video = VideoReader('stringLengthEdit_11.17.20.mpeg');
    
    fps         = ceil(video.framerate);       % camera frame rate (frames/sec)
    dt          = 1/fps;       % Time between successive frames
    button_width = 0.335*2.54; % width of button, in cm
    
    %converts video to greyscale
    %k is exact number of frames in video
    k = 0;
    while hasFrame(video)
        k = k + 1;
        %new line
        image_list(:,:,k) = rgb2gray(readFrame(video));
    end
    time = 0:dt:(k-1)/fps;  % Vector of time values  
    
%% Analysis
    %Ask the user to identify the width of button
    button_pix = calibrate_camera(image_list(:,:,1), 'the button.');
    pixpercm =  button_pix/button_width;
    
    string_length = tracker(image_list);
    %corrects units and makes everything relative to first frame
    string_length = string_length/pixpercm;

    posfigure = figure;
    axes1 = axes('Parent',posfigure,'YGrid','on','XGrid','on','FontSize',14);
    box(axes1,'on');
    hold(axes1,'on');
    ylabel('String Length (cm)');
    xlabel('Time (sec)');
    title('String Length vs. time'); 
    plot(time,string_length); 
    % Save the position-v-time data to a csv file
    csvwrite(csvfilename,[time',string_length']);
end

function width = calibrate_camera(refImage, item)
%   Function to determine # pixels corresponding to the item
%   User must select the scale marks

    figure;
    imshow(refImage);
    hold on;
    title(strcat('Click on left and right edge of', item))
    disp(strcat('Click on left and right edge of', item))
    [xtick_space,ytick_space] = ginput(2);
    width   = sqrt( (xtick_space(2)-xtick_space(1)).^2 + (ytick_space(2)-ytick_space(1)).^2 );
    close all;
end

function string_length = tracker(image_list)
%   Function which returns list of the number of consectuive columns with a
%   white "string" in it

    %iterate through each frame
    for frame = 1:size(image_list, 3)
        
        %white threshold - sensitivity
        white_threshold = 140;
        
        %number of whte pixels in that column must be over a certain number
        pix_threshold = 4;
        %number of white must be less than another number
        pix_max = 150;
        
        %max in a row white
        col_count_max = 0;
        %number of columns in a row that have detected white
        col_count = 0;
        
        for col = 1:size(image_list, 2)
            %in each column, count how many white pixels there are
            white_pix_count = 0;
            
            for row = 1:size(image_list, 1)
                if(image_list(row, col, frame) > white_threshold)
                    white_pix_count = white_pix_count + 1;
                end
            end
            
            if(white_pix_count >= pix_threshold ... 
                && white_pix_count <= pix_max)
                col_count = col_count + 1;
                
                %if new record for most white rows in a row, set as max
                if(col_count >= col_count_max)
                    col_count_max = col_count;
                end
            else
                col_count = 0;
            end
        end
        
        string_length(frame) = col_count_max;
    end
end