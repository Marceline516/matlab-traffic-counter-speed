%% main_final_interactive_fixed_spike.m
% WIG3006 - FINAL SUBMISSION CODE
%
% OVERVIEW:
% This script detects vehicles using YOLO, tracks them with a Kalman Filter,
% and estimates speed using a perspective transform (homography).
%
% KEY FEATURES:
% 1. Interactive Calibration: User defines the road geometry at runtime.
% 2. Robust Filtering: Only tracks vehicles strictly inside the drawn zone.
% 3. Noise Reduction: Ignores unrealistic speeds (>220 km/h).

clear; clc; close all;

%% 1. Configuration
% ---------------------------------------------------------
videoFile = "Road traffic video.mp4"; 
if ~isfile(videoFile)
    if isfile("data/video4.mp4"), videoFile = "data/video4.mp4";
    else, error('Video file not found.'); end
end

v = VideoReader(videoFile);
fps = v.FrameRate;
width = v.Width;
height = v.Height;
fprintf('Loaded: %s | %dx%d | %.2f FPS\n', videoFile, width, height, fps);

% --- REAL WORLD CONSTANTS ---
% These numbers define the "Scale" of the world.
% If the speed is too high/low, adjust REAL_LENGTH.
REAL_WIDTH  = 32;   % Approx width of the road in meters
REAL_LENGTH = 140;  % Approx length of the road segment in meters

% --- YELLOW LINE POSITION ---
% The pixel Y-coordinate (vertical) where the count happens.
% 0.60 means the line is 60% down from the top of the screen.
countLineY = round(height * 0.60); 

%% 2. Detector Initialization
% ---------------------------------------------------------
try
    detector = initYoloVehicleDetector();
    fprintf('Detector: Custom YOLO\n');
catch
    detector = vehicleDetectorACF('car');
    fprintf('Detector: Standard ACF (Fallback)\n');
end

%% 3. INTERACTIVE CALIBRATION (Run on every start)
% ---------------------------------------------------------
% This section pauses the code to let you draw the "World".
frame1 = readFrame(v);
hCalibFig = figure('Name', 'CALIBRATION STEP');
imshow(frame1);

% Instructions for the user (Black text for visibility)
title('STEP 1: Draw the Road Zone (Click 4 corners)', 'Color', 'r', 'FontSize', 14);
text(10, 30, 'Order: Top-Left -> Top-Right -> Bot-Right -> Bot-Left, Right click when done adding points', ...
    'Color', 'black', 'FontSize', 12, 'FontWeight', 'bold', 'BackgroundColor', 'white');

% User draws the polygon (The Blue Box)
hPoly = drawpolygon('Color', 'cyan', 'LineWidth', 2);
title('STEP 2: Adjust corners if needed, then double-click inside to Finish.', 'Color', 'g');
wait(hPoly); % Code pauses here until you double-click

zonePoints = hPoly.Position; 
close(hCalibFig); 

if size(zonePoints, 1) ~= 4
    error('Calibration Failed: You must click exactly 4 points!');
end

% Create the Math Transform (Pixels -> Meters)
% This matrix 'tform' allows us to convert screen pixels to real-world meters.
worldPoints = [0, 0; REAL_WIDTH, 0; REAL_WIDTH, REAL_LENGTH; 0, REAL_LENGTH];
tform = fitgeotrans(zonePoints, worldPoints, 'projective');
fprintf('Calibration Saved. Starting Tracker...\n');

%% 4. Tuning Parameters (CRITICAL SECTION)
% ---------------------------------------------------------
% 1. MIN BOX AREA:
%    - Filters out small noise (birds, distant specks).
%    - Decrease (e.g., 50) to detect cars further away at the horizon.
%    - Increase (e.g., 300) to only track cars when they are close.
minBoxArea = 150; 

% 2. COST OF NON-ASSIGNMENT:
%    - Controls how "sticky" the tracker is.
%    - High (e.g., 100+): Keeps the ID even if the detection is far away.
%    - Low (e.g., 20): Creates a new ID easily.
costOfNonAssignment = 80; 

% 3. MAX MATCHING DISTANCE:
%    - How far (in pixels) a car can move in 1 frame and still be tracked.
%    - Increase this if fast cars are losing their IDs.
maxMatchingDistance = 50; 

%% 5. Tracks Initialization
% ---------------------------------------------------------
% The 'tracks' struct stores the history of every car.
tracks = struct('id', {}, 'bbox', {}, 'kalmanFilter', {}, 'age', {}, ...
    'totalVisibleCount', {}, 'consecutiveInvisibleCount', {}, ...
    'centroid', {}, 'prevCentroid', {}, ...
    'worldTrace', {}, 'speedKmh', {}, 'counted', {});

nextTrackID = 1;
totalCount = 0;
savedSpeeds = []; 

%% 6. Main Loop
% ---------------------------------------------------------
v.CurrentTime = 0; 
hFig = figure('Name','Final Speed Tracker'); 
setappdata(hFig,'isPaused',false);
uicontrol('Parent',hFig,'Style','togglebutton','String','Pause', ...
    'Position',[20 20 80 30],'Callback',@(s,~) setappdata(hFig,'isPaused',get(s,'Value')));

while hasFrame(v) && ishandle(hFig)
    frame = readFrame(v); 
    
    % --- A. Detect ---
    try
        % Threshold 0.35: Only accepts detections with >35% confidence.
        [bboxes, scores, labels] = detect(detector, frame, 'Threshold', 0.35);
    catch
        [bboxes, scores] = detect(detector, frame);
    end
    
    detBboxes = []; detCentroids = [];
    
    if ~isempty(bboxes)
        % Calculate "Wheel Point" (Bottom Center of box)
        centroids = [bboxes(:,1) + bboxes(:,3)/2, bboxes(:,2) + bboxes(:,4)];
        
        % Filter 1: Is it inside your Blue Box?
        inZone = inpolygon(centroids(:,1), centroids(:,2), zonePoints(:,1), zonePoints(:,2));
        
        % Filter 2: Is it big enough?
        validSize = (bboxes(:,3) .* bboxes(:,4)) > minBoxArea;
        
        % Strict Filter: Must be both IN ZONE and VALID SIZE
        keep = inZone & validSize;
        detBboxes = bboxes(keep, :);
        detCentroids = centroids(keep, :);
    end
    
    nDets = size(detBboxes, 1);

    % --- B. Predict (Kalman) ---
    nTracks = numel(tracks);
    predictedCentroids = zeros(nTracks,2);
    for i = 1:nTracks
        pred = predict(tracks(i).kalmanFilter); 
        predCentroid = reshape(pred(1:2), 1, 2); 
        predictedCentroids(i,:) = predCentroid;
        tracks(i).prevCentroid = tracks(i).centroid;
        tracks(i).centroid = predCentroid;
    end
    
    % --- C. Association (Hungarian Algorithm) ---
    if nTracks==0 || nDets==0
        assignments = zeros(0,2);
        unassignedTracks = (1:nTracks)';
        unassignedDetections = (1:nDets)';
    else
        cost = zeros(nTracks, nDets);
        for i=1:nTracks
            for j=1:nDets
                % Calculate distance between Prediction and Actual Detection
                cost(i,j) = norm(predictedCentroids(i,:) - detCentroids(j,:));
                if cost(i,j) > maxMatchingDistance, cost(i,j) = Inf; end
            end
        end
        [assignments, unassignedTracks, unassignedDetections] = assignDetectionsToTracks(cost, costOfNonAssignment);
    end
    
    % --- D. Updates ---
    for k = 1:size(assignments,1)
        tr = assignments(k,1); detIdx = assignments(k,2);
        measurement = detCentroids(detIdx,:);
        
        correct(tracks(tr).kalmanFilter, measurement);
        tracks(tr).bbox = detBboxes(detIdx,:);
        tracks(tr).age = tracks(tr).age + 1;
        tracks(tr).totalVisibleCount = tracks(tr).totalVisibleCount + 1;
        tracks(tr).consecutiveInvisibleCount = 0;
        tracks(tr).centroid = measurement; 
        
        % --- SPEED MATH (Continuous Displacement) ---
        % 1. Map pixel (x,y) to meters (x,y)
        currWorld = transformPointsForward(tform, measurement);
        tracks(tr).worldTrace = [tracks(tr).worldTrace; currWorld];
        
        % Buffer: Keep only last 1 second of history
        if size(tracks(tr).worldTrace, 1) > fps, tracks(tr).worldTrace(1,:) = []; end
        
        if size(tracks(tr).worldTrace, 1) > 2
            % Calculate how many meters moved since last frame
            diffs = abs(diff(tracks(tr).worldTrace));
            
            % Use MEDIAN to filter out jittery noise
            dx = median(diffs(:,1)); dy = median(diffs(:,2));
            instSpeed = hypot(dx, dy) * fps * 3.6; % Convert m/s to km/h
            
            % SPIKE FIX: If speed > 220 km/h, ignore it (likely an error)
            if instSpeed < 220
                if isnan(tracks(tr).speedKmh)
                    tracks(tr).speedKmh = instSpeed;
                else
                    % Smooth display: 80% old value, 20% new value
                    tracks(tr).speedKmh = 0.8 * tracks(tr).speedKmh + 0.2 * instSpeed;
                end
                
                % Save statistics
                if instSpeed > 5
                     savedSpeeds(end+1) = instSpeed; 
                     if length(savedSpeeds) > 5000, savedSpeeds(1) = []; end
                end
            end
        end
        
        % --- LINE CROSSING ---
        yPrev = tracks(tr).prevCentroid(2);
        yNow  = tracks(tr).centroid(2);
        
        if ~tracks(tr).counted && yPrev < countLineY && yNow >= countLineY
            totalCount = totalCount + 1;
            tracks(tr).counted = true;
        end
    end
    
    % Maintenance (Delete lost tracks, create new ones)
    for i = 1:numel(unassignedTracks)
        idx = unassignedTracks(i);
        tracks(idx).age = tracks(idx).age + 1;
        tracks(idx).consecutiveInvisibleCount = tracks(idx).consecutiveInvisibleCount + 1;
    end
    
    for i = 1:numel(unassignedDetections)
        d = unassignedDetections(i);
        meas = detCentroids(d,:);
        kf = configureKalmanFilter('ConstantVelocity', meas, [5, 5], [50, 50], 25);
        tracks(end+1) = struct('id', nextTrackID, 'bbox', detBboxes(d,:), ...
            'kalmanFilter', kf, 'age', 1, 'totalVisibleCount', 1, 'consecutiveInvisibleCount', 0, ...
            'centroid', meas, 'prevCentroid', meas, 'worldTrace', [], 'speedKmh', NaN, 'counted', false);
        nextTrackID = nextTrackID + 1;
    end
    
    if ~isempty(tracks)
        invisible = [tracks.consecutiveInvisibleCount] > 5;
        tracks(invisible) = [];
    end

    % --- F. Visualization ---
    frameOut = frame;
    
    polyFlat = reshape(zonePoints', 1, []);
    frameOut = insertShape(frameOut, 'Polygon', polyFlat, 'Color', 'cyan', 'LineWidth', 1);
    frameOut = insertShape(frameOut, 'Line', [0, countLineY, width, countLineY], 'Color', 'yellow', 'LineWidth', 2);
    
    for t = 1:numel(tracks)
        if tracks(t).totalVisibleCount > 3
            
            % --- COLOR LOGIC (Green -> Purple -> Red) ---
            if ~isnan(tracks(t).speedKmh) && tracks(t).speedKmh > 0
                col = 'red'; lbl = sprintf('%.0f km/h', tracks(t).speedKmh);
            elseif tracks(t).counted
                col = 'magenta'; lbl = sprintf('ID %d', tracks(t).id);
            else
                col = 'green'; lbl = sprintf('ID %d', tracks(t).id);
            end
            
            frameOut = insertObjectAnnotation(frameOut, "rectangle", tracks(t).bbox, lbl, "Color", col, "LineWidth", 2);
        end
    end
    
    frameOut = insertText(frameOut, [10 10], sprintf('Total: %d', totalCount), "FontSize", 18, "BoxOpacity", 0.6);
    imshow(frameOut); drawnow;
    while ishandle(hFig) && getappdata(hFig,'isPaused'); pause(0.1); end
end

fprintf('\nFinished. Total Count: %d\n', totalCount);
if ~isempty(savedSpeeds)
    fprintf('Avg Speed: %.2f km/h | Max: %.2f km/h\n', mean(savedSpeeds), max(savedSpeeds));
else
    fprintf('No speeds recorded.\n');
end