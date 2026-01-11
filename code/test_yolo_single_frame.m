%% test_yolo_single_frame.m
% Quick test: run YOLO detection on a single frame from the traffic video.

clear; clc; close all;

%% 1. Load one frame from the video

videoFile = "data/video1.mp4";  % <-- change if your path/name is different

if ~isfile(videoFile)
    error('Video file not found: %s', videoFile);
end

v = VideoReader(videoFile);

% Read a frame somewhere in the middle of the video
targetTime = min(5.0, v.Duration / 2);   % seconds
v.CurrentTime = targetTime;
frame = readFrame(v);

figure;
imshow(frame);
title('Original frame for YOLO test');

%% 2. Initialize YOLO detector

detector = initYoloVehicleDetector();  % this may take some time on first run

%% 3. Run detection

% detect returns bounding boxes, scores and labels
[bboxes, scores, labels] = detect(detector, frame, 'Threshold', 0.3);

% Keep only vehicles: car, bus, truck, maybe motorcycle
isVehicle = labels == "car" | labels == "bus" | labels == "truck" | labels == "motorbike";
bboxes  = bboxes(isVehicle, :);
scores  = scores(isVehicle);
labelsV = labels(isVehicle);

%% 4. Visualize results

if isempty(bboxes)
    warning('No vehicles detected in this frame.');
    outFrame = frame;
else
    % Create annotation strings: class + score
    annotations = strings(numel(scores), 1);
    for i = 1:numel(scores)
        annotations(i) = sprintf('%s (%.2f)', labelsV(i), scores(i));
    end

    outFrame = insertObjectAnnotation(frame, 'rectangle', bboxes, cellstr(annotations), ...
        'Color', 'green', 'LineWidth', 2, 'FontSize', 10);
end

figure;
imshow(outFrame);
title('YOLO detection result on single frame');
