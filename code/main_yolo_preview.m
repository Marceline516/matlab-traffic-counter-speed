%% main_yolo_preview.m
% Preview traffic video with YOLO detections and three reference lines.

clear; clc; close all;

%% 1. Video configuration

videoFile = "data/video4.mp4";  % change this if your path/name is different

if ~isfile(videoFile)
    error('Video file not found: %s', videoFile);
end

v   = VideoReader(videoFile);
fps = v.FrameRate;

fprintf('Loaded video: %s\n', videoFile);
fprintf('Resolution: %d x %d | FPS: %.2f | Duration: %.2f s\n', ...
    v.Width, v.Height, fps, v.Duration);

%% 2. Initialize YOLO detector

detector = initYoloVehicleDetector();   % the function you created before

%% 3. Line positions (in pixels, for 640 x 360 video)

% You can adjust these values later if you want to move the lines.
countLineY  = 260;   % counting line
speedLineY1 = 230;   % speed line 1 (upper)
speedLineY2 = 290;   % speed line 2 (lower)

%% 4. Main loop: read frames, detect, draw boxes and lines

hFig = figure('Name', 'YOLO Traffic Preview');
frameIdx = 0;

while hasFrame(v)
    frame = readFrame(v);
    frameIdx = frameIdx + 1;

    % ---- YOLO detection on current frame ----
    [bboxes, scores, labels] = detect(detector, frame, 'Threshold', 0.43);

    % Keep only vehicle classes
    isVehicle = labels == "car" | labels == "bus" | labels == "truck" | labels == "motorbike";
    bboxes    = bboxes(isVehicle, :);
    scores    = scores(isVehicle);
    labelsV   = labels(isVehicle);

    % ---- Draw lines ----
    [H, W, ~] = size(frame);
    lines = [...
        1, countLineY,  W, countLineY;  ...
        1, speedLineY1, W, speedLineY1; ...
        1, speedLineY2, W, speedLineY2];

    lineColors = {'yellow', 'cyan', 'cyan'};

    frameOut = insertShape(frame, 'Line', lines, ...
        'Color', lineColors, 'LineWidth', 2);

    % ---- Draw bounding boxes and labels ----
    if ~isempty(bboxes)
        ann = strings(numel(scores), 1);
        for i = 1:numel(scores)
            ann(i) = sprintf('%s (%.2f)', labelsV(i), scores(i));
        end

        frameOut = insertObjectAnnotation(frameOut, 'rectangle', ...
            bboxes, cellstr(ann), ...
            'Color', 'green', 'LineWidth', 2, 'FontSize', 10);
    end

    % ---- Show frame ----
    imshow(frameOut, 'Border', 'tight');
    tSec = frameIdx / fps;
    title(sprintf('YOLO preview | frame %d | t = %.2f s', frameIdx, tSec), ...
        'Color', 'w', 'FontSize', 12);

    drawnow;

    if ~ishandle(hFig)
        break;  % window closed by user
    end
end

disp('YOLO preview finished.');
