%% main_final_sticky_fix.m
% WIG3006 - Speed Estimation fix
%
% DESCRIPTION:
% This script detects vehicles, tracks them using a Kalman Filter + Hungarian Algorithm,
% and estimates their speed based on a calibrated zone.
%
% KEY FEATURES:
% 1. "Sticky" Tracking: High assignment cost prevents ID swapping in traffic jams.
% 2. Bottom-Center Tracking: Tracks the tires (not the roof) for accuracy.
% 3. Horizon Logic: Automatically removes tracks that fly into the sky.

clear; clc; close all;

%% 1. Configuration
videoFile = "Road traffic video.mp4"; %%change depending on your video file
if ~isfile(videoFile)
    if isfile("data/video4.mp4"), videoFile = "data/video4.mp4";
    else, error('Video file not found.'); end
end

v = VideoReader(videoFile);
fps = v.FrameRate;
width = v.Width;
height = v.Height;
fprintf('Loaded: %s | %dx%d | %.2f FPS\n', videoFile, width, height, fps);

%% 2. Detector Initialization
try
    detector = initYoloVehicleDetector();
catch
    detector = vehicleDetectorACF('car');
end

%% 3. Tuning Parameters (ADJUST THESE TO FIX ANY ISSUES )

% --- Line Positions (Percent of Screen Height) ---
% If cars are missed, move these lines up or down.
countLineY  = round(height * 0.70);   % Yellow Counting Line
speedLineY1 = round(height * 0.60);   % Top Cyan Line (Start Timer)
speedLineY2 = round(height * 0.80);   % Bottom Cyan Line (End Timer)

% --- Region of Interest (ROI) ---
% Controls where we look for cars. 
% - Increase Min (e.g., 150) to cut off left-side grass noise.
% - Decrease Min (e.g., 50) to see cars earlier.
roiXMin = 100; 
roiXMax = 540;

% --- Detection Sensitivity ---
% - Lower (0.20): Detects more cars, but also sees ghosts/shadows.
% - Higher (0.50): Only detects clear cars, might miss some.
yoloThreshold = 0.30;

% --- Tracking Stability ---
minTrackAge = 5; % Wait 5 frames before showing a new ID (Prevents flicker).

% --- PACKED TRAFFIC TUNING (CRITICAL) ---
% 1. Min Box Area: 
%    - Increase (e.g., 400) to ignore tiny cars in the distance.
%    - Decrease (e.g., 100) to track cars continuously from the horizon.
minBoxArea = 300;          

% 2. Cost of Non-Assignment (Stickiness):
%    - Increase (e.g., 200): "Super Sticky". Forces code to keep the old ID. Good for jams.
%    - Decrease (e.g., 20): "Loose". Cars get new IDs easily. Good for sparse traffic.
costOfNonAssignment = 150; 

% 3. Max Matching Distance:
%    - Increase (e.g., 150): Catches fast cars that jump far between frames.
%    - Decrease (e.g., 30): Strict matching. Only links if car barely moves.
maxMatchingDistance = 100; 

%% 4. Calibration
calibrationFile = "calibration_Clean.mat"; 
if ~isfile(calibrationFile)
    if isfile("calibration_Highway.mat"), calibrationFile = "calibration_Highway.mat"; end
end

if isfile(calibrationFile)
    load(calibrationFile);
    pt1 = [width/2, speedLineY1];
    pt2 = [width/2, speedLineY2];
    [wx1, wy1] = transformPointsForward(tform, pt1(1), pt1(2));
    [wx2, wy2] = transformPointsForward(tform, pt2(1), pt2(2));
    dist_m = hypot(wx2-wx1, wy2-wy1);
    fprintf('Trap distance: %.2f meters\n', dist_m);
else
    fprintf('WARNING: No calibration found. Speeds will be wrong.\n');
    dist_m = 20; 
end

%% 5. Tracks Initialization
tracks = struct('id', {}, 'bbox', {}, 'kalmanFilter', {}, 'age', {}, ...
    'totalVisibleCount', {}, 'consecutiveInvisibleCount', {}, ...
    'tLine1', {}, 'tLine2', {}, 'speedKmh', {}, 'counted', {}, ...
    'centroid', {}, 'prevCentroid', {});
nextTrackID = 1;
totalCount = 0;
savedSpeeds = [];

%% 6. Main Loop
hFig = figure('Name','Sticky Kalman Tracker'); 
setappdata(hFig,'isPaused',false);
uicontrol('Parent',hFig,'Style','togglebutton','String','Pause', ...
    'Position',[20 20 80 30],'Callback',@(s,~) setappdata(hFig,'isPaused',get(s,'Value')));

frameIdx = 0;
% Function to calculate exact sub-frame crossing time
crossTime = @(yPrev, yNow, yLine, fPrev, fNow, fps) ...
    ((fPrev - 1) + max(0, min(1, (yLine - yPrev) / (yNow - yPrev)))) / fps;

while hasFrame(v) && ishandle(hFig)
    frame = readFrame(v); frameIdx = frameIdx + 1;
    tNow = frameIdx / fps;

    % --- A. Detect ---
    try
        [bboxes, scores, labels] = detect(detector, frame, 'Threshold', yoloThreshold);
    catch
        [bboxes, scores] = detect(detector, frame);
        labels = repmat("car", size(bboxes,1), 1);
    end

    % Filter Detections
    detBboxes = [];
    if ~isempty(bboxes)
        isVehicle = labels == "car" | labels == "bus" | labels == "truck";
        bboxes = bboxes(isVehicle, :);
        keep = false(size(bboxes,1),1);
        for i=1:size(bboxes,1)
            cx = bboxes(i,1) + bboxes(i,3)/2;
            area = bboxes(i,3)*bboxes(i,4);
            % Filter out noise using Area and ROI
            if area > minBoxArea && cx > roiXMin && cx < roiXMax
                keep(i) = true;
            end
        end
        detBboxes = bboxes(keep,:);
    end
    nDets = size(detBboxes,1);

    % Calculate Bottom-Center (Tires)
    % This is more accurate for speed than using the center of the box.
    detCentroids = zeros(nDets,2);
    for j=1:nDets
        detCentroids(j,:) = [detBboxes(j,1)+detBboxes(j,3)/2, detBboxes(j,2)+detBboxes(j,4)]; 
    end

    % --- B. Predict (Kalman Filter) ---
    nTracks = numel(tracks);
    predictedCentroids = zeros(nTracks,2);
    for i = 1:nTracks
        pred = predict(tracks(i).kalmanFilter); 
        predCentroid = reshape(pred(1:2), 1, 2); % Fix for vertcat crash
        
        predictedCentroids(i,:) = predCentroid;
        tracks(i).prevCentroid = tracks(i).centroid;
        tracks(i).centroid = predCentroid;
    end

    % --- C. Association (Hungarian Algorithm) ---
    if nTracks==0
        assignments = zeros(0,2);
        unassignedTracks = [];
        unassignedDetections = (1:nDets)';
    elseif nDets==0
        assignments = zeros(0,2);
        unassignedTracks = (1:nTracks)';
        unassignedDetections = [];
    else
        % Cost Matrix: Distance between Prediction and Measurement
        cost = zeros(nTracks, nDets);
        for i=1:nTracks
            for j=1:nDets
                cost(i,j) = norm(predictedCentroids(i,:) - detCentroids(j,:));
                if cost(i,j) > maxMatchingDistance, cost(i,j) = Inf; end
            end
        end
        [assignments, unassignedTracks, unassignedDetections] = assignDetectionsToTracks(cost, costOfNonAssignment);
    end

    % --- D. Updates ---
    % 1. Assigned Tracks (Update with new data)
    for k = 1:size(assignments,1)
        tr = assignments(k,1); detIdx = assignments(k,2);
        measurement = detCentroids(detIdx,:);
        
        correct(tracks(tr).kalmanFilter, measurement);
        
        tracks(tr).bbox = detBboxes(detIdx,:);
        tracks(tr).age = tracks(tr).age + 1;
        tracks(tr).totalVisibleCount = tracks(tr).totalVisibleCount + 1;
        tracks(tr).consecutiveInvisibleCount = 0;
        tracks(tr).centroid = reshape(measurement, 1, 2); 
    end

    % 2. Unassigned Tracks (Invisible / Coasting)
    for i = 1:numel(unassignedTracks)
        idx = unassignedTracks(i);
        tracks(idx).age = tracks(idx).age + 1;
        tracks(idx).consecutiveInvisibleCount = tracks(idx).consecutiveInvisibleCount + 1;
    end

    % 3. Delete Lost Tracks
    if ~isempty(tracks)
        invisibleForTooLong = [tracks.consecutiveInvisibleCount] >= 15;
        centrs = vertcat(tracks.centroid); 
        if ~isempty(centrs)
            % Kill tracks that go out of bounds
            outOfBounds = centrs(:,2) > height | centrs(:,2) < 0 | centrs(:,1) > width | centrs(:,1) < 0;
        else
            outOfBounds = false(size(invisibleForTooLong));
        end
        tracks(invisibleForTooLong | outOfBounds') = [];
    end

    % 4. Create New Tracks
    for i = 1:numel(unassignedDetections)
        d = unassignedDetections(i);
        bbox = detBboxes(d,:);
        meas = detCentroids(d,:); 

        % Configure Kalman: Constant Velocity
        kf = configureKalmanFilter('ConstantVelocity', meas, [50, 25], [25, 10], 25);

        newTrack = struct('id', nextTrackID, 'bbox', bbox, 'kalmanFilter', kf, 'age', 1, ...
            'totalVisibleCount', 1, 'consecutiveInvisibleCount', 0, ...
            'tLine1', NaN, 'tLine2', NaN, 'speedKmh', NaN, 'counted', false, ...
            'centroid', meas, 'prevCentroid', meas);
        tracks(end+1) = newTrack;
        nextTrackID = nextTrackID + 1;
    end

    % --- E. Logic ---
    for t = 1:numel(tracks)
        if tracks(t).totalVisibleCount < minTrackAge, continue; end
        yPrev = tracks(t).prevCentroid(2);
        yNow  = tracks(t).centroid(2);

        % Count (Yellow Line)
        if ~tracks(t).counted
            if (yPrev < countLineY && yNow >= countLineY) || (yPrev > countLineY && yNow <= countLineY)
                totalCount = totalCount + 1;
                tracks(t).counted = true;
            end
        end

        % Speed (Cyan Lines)
        if yNow > yPrev % Moving Down
            if isnan(tracks(t).tLine1) && yPrev <= speedLineY1 && yNow > speedLineY1
                tracks(t).tLine1 = crossTime(yPrev, yNow, speedLineY1, frameIdx-1, frameIdx, fps);
            end
            if isnan(tracks(t).tLine2) && yPrev <= speedLineY2 && yNow > speedLineY2
                tracks(t).tLine2 = crossTime(yPrev, yNow, speedLineY2, frameIdx-1, frameIdx, fps);
            end
        elseif yNow < yPrev % Moving Up
            if isnan(tracks(t).tLine2) && yPrev >= speedLineY2 && yNow < speedLineY2
                tracks(t).tLine2 = crossTime(yPrev, yNow, speedLineY2, frameIdx-1, frameIdx, fps);
            end
            if isnan(tracks(t).tLine1) && yPrev >= speedLineY1 && yNow < speedLineY1
                tracks(t).tLine1 = crossTime(yPrev, yNow, speedLineY1, frameIdx-1, frameIdx, fps);
            end
        end

        % Calculate Speed
        if ~isnan(tracks(t).tLine1) && ~isnan(tracks(t).tLine2) && isnan(tracks(t).speedKmh)
            dt = abs(tracks(t).tLine2 - tracks(t).tLine1);
            if dt > 0.05
                speed_mps = dist_m / dt;
                speedKmh = speed_mps * 3.6;
                % Filter insane speeds (e.g. 300 km/h)
                if speedKmh > 5 && speedKmh < 300
                    tracks(t).speedKmh = speedKmh;
                    savedSpeeds(end+1) = speedKmh; 
                end
            end
        end
    end

    % --- F. Visualization ---
    frameOut = frame;
    lines = [roiXMin, countLineY, roiXMax, countLineY; ...
             roiXMin, speedLineY1, roiXMax, speedLineY1; ...
             roiXMin, speedLineY2, roiXMax, speedLineY2];
    frameOut = insertShape(frameOut, 'Line', lines, 'Color', {'yellow','cyan','cyan'}, 'LineWidth', 2);

    for t = 1:numel(tracks)
        if tracks(t).totalVisibleCount > minTrackAge
            % Color Logic: Red (Speed), Magenta (Counted), Green (Tracking)
            if ~isnan(tracks(t).speedKmh)
                col = 'red'; lbl = sprintf('ID %d | %.0f', tracks(t).id, tracks(t).speedKmh);
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