%% main_with_detection.m
% WIG3006 – Vehicle detection + lines (integrated with Leen's detectVehicles)

clear; clc; close all;

%% 1. User configuration


videoFile = "data/Road traffic video.mp4";

%% line position
countLineY  = 260;   
speedLineY1 = 230;   
speedLineY2 = 290;

%% 2. Create video reader

if ~isfile(videoFile)
    error('Video file not found: %s', videoFile);
end

v   = VideoReader(videoFile);
fps = v.FrameRate;

fprintf('Loaded video: %s\n', videoFile);
fprintf('Resolution: %d x %d | FPS: %.2f | Duration: %.2f s\n', ...
    v.Width, v.Height, fps, v.Duration);

%% 3. Preview loop: video reading + display + detection

hFig = figure('Name', 'Traffic Video Preview – Detection Integrated');
frameIdx = 0;

while hasFrame(v)
    frame = readFrame(v);
    frameIdx = frameIdx + 1;

    % ---- STEP 1: 
    % detectVehicles(frame)
    [bboxes, centroids] = detectVehicles(frame);

    % ---- STEP 2: 
    imshow(frame, 'Border', 'tight');
    hold on;

    if ~isempty(bboxes)
       
        for i = 1:size(bboxes, 1)
            rectangle('Position', bboxes(i,:), ...
                'EdgeColor', 'g', 'LineWidth', 2);
            plot(centroids(i,1), centroids(i,2), 'r+', ...
                'MarkerSize', 6, 'LineWidth', 1);
        end
    end

    % ---- STEP 3: 
    [h, w, ~] = size(frame); %#ok<ASGLU>

    % counting line (yellow)
    line([1, w], [countLineY, countLineY], ...
        'Color', 'y', 'LineWidth', 2);

    % speed lines (cyan, dashed)
    line([1, w], [speedLineY1, speedLineY1], ...
        'Color', 'c', 'LineWidth', 1, 'LineStyle', '--');
    line([1, w], [speedLineY2, speedLineY2], ...
        'Color', 'c', 'LineWidth', 1, 'LineStyle', '--');

    % ---- STEP 4: 
    tSec = frameIdx / fps;
    title(sprintf('Frame %d | t = %.2f s', frameIdx, tSec), ...
        'FontSize', 12, 'Color', 'w');

    hold off;
    drawnow;

    
    if ~ishandle(hFig)
        break;
    end
end

disp('Preview finished.');
