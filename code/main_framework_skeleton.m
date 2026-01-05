%% main_framework_skeleton.m
% WIG3006 Digital Video Processing – Group Project
% Vision-based vehicle counting and speed estimation framework (skeleton)
%
% Current scope (Meilin):
%   - Read traffic video
%   - Display frames
%   - Overlay virtual counting & speed-estimation lines
%
% Later (Leen / Zhang / group):
%   - Vehicle detection (classical / DL)
%   - Tracking & virtual-line counting
%   - Basic speed estimation between two lines
%
% This script matches the research aim:
%   "a relatively simple MATLAB-based vision framework for vehicle
%    counting and speed estimation from ordinary CCTV traffic videos."

clear; clc; close all;

%% 1. User configuration (can be adjusted per video)

% Traffic video path (ordinary CCTV-style video)
videoFile = "Road traffic video.mp4";   % TODO: change to your file name

% Virtual lines (in pixel coordinates, along image width)
% One counting line + two speed lines
countLineY  = 300;   % counting line (vehicle crosses → count +1)
speedLineY1 = 260;   % speed line 1 (enter)
speedLineY2 = 350;   % speed line 2 (exit)

% NOTE:
% After you run once, adjust these Y positions so that
% all three lines lie across the main traffic lanes.

%% 2. Video reader and basic info

if ~isfile(videoFile)
    error('Video file not found: %s', videoFile);
end

v   = VideoReader(videoFile);
fps = v.FrameRate;

fprintf('Loaded video: %s\n', videoFile);
fprintf('Resolution: %d x %d | FPS: %.2f | Duration: %.2f s\n', ...
    v.Width, v.Height, fps, v.Duration);

%% 3. Preview loop: video reading + display + virtual lines (no detection yet)

hFig = figure('Name', 'Traffic Video Preview – Skeleton');

frameIdx = 0;

while hasFrame(v)
    frame = readFrame(v);
    frameIdx = frameIdx + 1;

    % Show raw frame
    imshow(frame, 'Border', 'tight');
    hold on;

    % Get width for drawing horizontal lines
    [h, w, ~] = size(frame); %#ok<ASGLU>

    % --- Virtual counting line (yellow, solid) ---
    line([1, w], [countLineY, countLineY], ...
        'Color', 'y', 'LineWidth', 2);  % future: vehicle crosses → count

    % --- Virtual speed lines (cyan, dashed) ---
    line([1, w], [speedLineY1, speedLineY1], ...
        'Color', 'c', 'LineWidth', 1, 'LineStyle', '--');  % future: t_enter
    line([1, w], [speedLineY2, speedLineY2], ...
        'Color', 'c', 'LineWidth', 1, 'LineStyle', '--');  % future: t_exit

    % Title: frame index + time stamp
    tSec = frameIdx / fps;
    title(sprintf('Frame %d | t = %.2f s', frameIdx, tSec), ...
        'FontSize', 12, 'Color', 'w');

    hold off;
    drawnow;

    % Allow user to close window to stop preview
    if ~ishandle(hFig)
        break;
    end
end

disp('Preview finished.');

%% 4. Next steps (for group, not implemented here)
% - Step 1 (Leen): foreground detection module
%       -> function detectVehicles(frame) returning bboxes / centroids
% - Step 2 (Leen + Meilin): tracking + line-crossing counting
% - Step 3 (Leen + Meilin): time stamps at speedLineY1/Y2 -> speed (km/h)
% - Step 4 (Zhang): run on multiple videos, save counts & speeds for evaluation
