%% main_step2_tracking_counting.m
% WIG3006 – Step 2: Detection, Tracking, and Counting 
%Notice: some of the numbers may need to be changed for accuracy such as the
%parameters and other such numbers 

clear; clc; close all;

%% 1. User Configuration
% ---------------------------------------------------------
videoFile = "Road traffic video.mp4"; 

% Virtual Line Positions (Y-coordinates)
countLineY  = 260;    % Yellow Line: Cars are counted when crossing this
speedLineY1 = 230;    % Cyan Line 1: Speed measurement start (Future Step)
speedLineY2 = 290;    % Cyan Line 2: Speed measurement end (Future Step)

% Tracking Parameters (Tuned for Accuracy)
costOfNonAssignment = 10;   % Max distance a car can move between frames (Lower = Stricter)
invisibleForTooLong = 5;    % Frames to wait before deleting a lost car (Lower = Less ghosts)
ageThreshold        = 5;    % Min frames a car must exist before being drawn (Reduces noise)

%% 2. Setup Video Reader
% ---------------------------------------------------------
if ~isfile(videoFile)
    if isfile("data/" + videoFile)
        videoFile = "data/" + videoFile;
    else
        error('Video file not found: %s', videoFile);
    end
end

v   = VideoReader(videoFile);
fps = v.FrameRate;
width = v.Width; 
fprintf('Loaded video: %s | Resolution: %dx%d\n', videoFile, v.Width, v.Height);

%% 3. Initialize Variables
% ---------------------------------------------------------
vehicleCount = 0; % Total cars counted

% The "Memory" Structure: Stores data for every car currently tracked
tracks = struct(...
    'id', {}, ...                   % Unique ID for the car
    'bbox', {}, ...                 % Bounding Box [x, y, w, h]
    'kalmanFilter', {}, ...         % Kalman Filter object for prediction
    'age', {}, ...                  % How many frames it has been alive
    'totalVisibleCount', {}, ...    % How many frames it was actually detected
    'consecutiveInvisibleCount', {}, ... % How long since we last saw it
    'isCounted', {});               % Boolean: Has this car been counted yet?

nextId = 1; % Counter for assigning new IDs

%% 4. Main Processing Loop
% ---------------------------------------------------------
hFig = figure('Name', 'Traffic Counter - Final Step 2');
frameIdx = 0;

while hasFrame(v)
    frame = readFrame(v);
    frameIdx = frameIdx + 1;
    
    
    % MODULE A: DETECTION (Step 1 Integration)
    
    % Calls the external function to find cars in the current frame
    [bboxes, centroids] = detectVehicles(frame);
    
   
    % MODULE B: TRACKING & COUNTING (Step 2 Logic)
    
    
    % --- B1. Predict New Locations ---
    % Use Kalman Filter to guess where existing cars moved to
    for i = 1:length(tracks)
        bbox = tracks(i).bbox;
        predictedCentroid = predict(tracks(i).kalmanFilter);
        
        % Shift box center to predicted position
        predictedCentroid = int32(predictedCentroid) - bbox(3:4) / 2;
        tracks(i).bbox = [predictedCentroid, bbox(3:4)];
    end
    
    % --- B2. Assign Detections to Tracks ---
    % Match the predicted boxes (tracks) to the actual found boxes (detections)
    nTracks = length(tracks);
    nDetections = size(centroids, 1);
    cost = zeros(nTracks, nDetections);
    
    % Calculate distance cost matrix
    for i = 1:nTracks
        cost(i, :) = distance(tracks(i).kalmanFilter, centroids);
    end
    
    % Solve assignment problem (Hungarian Algorithm)
    [assignments, unassignedTracks, unassignedDetections] = ...
        assignDetectionsToTracks(cost, costOfNonAssignment);
    
    % --- B3. Update Assigned Tracks ---
    for i = 1:size(assignments, 1)
        trackIdx = assignments(i, 1);
        detectionIdx = assignments(i, 2);
        
        centroid = centroids(detectionIdx, :);
        bbox = bboxes(detectionIdx, :);
        
        % Correct the Kalman Filter with the new real measurement
        correct(tracks(trackIdx).kalmanFilter, centroid);
        
        % Save Previous Y Position (Vital for crossing logic)
        prevY = tracks(trackIdx).bbox(2) + tracks(trackIdx).bbox(4)/2;
        
        % Update Track with new data
        tracks(trackIdx).bbox = bbox;
        tracks(trackIdx).age = tracks(trackIdx).age + 1;
        tracks(trackIdx).totalVisibleCount = tracks(trackIdx).totalVisibleCount + 1;
        tracks(trackIdx).consecutiveInvisibleCount = 0;
        
        % --- COUNTING LOGIC START ---
        currY = bbox(2) + bbox(4)/2; % Current Center Y
        
        if ~tracks(trackIdx).isCounted
            
            % Check crossing direction based on movement history
            % Case 1: Moving DOWN (Previous <= Line AND Current > Line)
            crossedDown = (prevY <= countLineY) && (currY > countLineY);
            
            % Case 2: Moving UP (Previous >= Line AND Current < Line)
            crossedUp   = (prevY >= countLineY) && (currY < countLineY);
            
            if crossedDown || crossedUp
                vehicleCount = vehicleCount + 1;
                tracks(trackIdx).isCounted = true; % Mark as counted
                fprintf('Car ID %d passed! Total: %d\n', tracks(trackIdx).id, vehicleCount);
            end
        end
        % --- COUNTING LOGIC END ---
    end
    
    % --- B4. Handle Lost Tracks ---
    % Mark tracks as invisible if we didn't find a match this frame
    for i = 1:length(unassignedTracks)
        ind = unassignedTracks(i);
        tracks(ind).age = tracks(ind).age + 1;
        tracks(ind).consecutiveInvisibleCount = tracks(ind).consecutiveInvisibleCount + 1;
    end
    
    % Delete tracks that have been lost for too long
    if ~isempty(tracks)
        lostInds = [tracks.consecutiveInvisibleCount] >= invisibleForTooLong;
        tracks(lostInds) = [];
    end
    
    % --- B5. Create New Tracks ---
    % Create new IDs for detections that didn't match any existing car
    for i = 1:length(unassignedDetections)
        detectionIdx = unassignedDetections(i);
        centroid = centroids(detectionIdx, :);
        bbox = bboxes(detectionIdx, :);
        
        % Filter out giant boxes (Noise/Sky/Trees)
        if bbox(3) < (width * 0.6) 
            kalmanFilter = configureKalmanFilter('ConstantVelocity', ...
                centroid, [200, 50], [100, 25], 100);
            
            newTrack = struct(...
                'id', nextId, ...
                'bbox', bbox, ...
                'kalmanFilter', kalmanFilter, ...
                'age', 1, ...
                'totalVisibleCount', 1, ...
                'consecutiveInvisibleCount', 0, ...
                'isCounted', false);
            
            tracks(end + 1) = newTrack; %#ok<AGROW>
            nextId = nextId + 1;
        end
    end
    
    
    % MODULE C: VISUALIZATION
   
    imshow(frame, 'Border', 'tight');
    hold on;
    
    [h, w, ~] = size(frame);
    
    % Draw Virtual Lines
    line([1, w], [countLineY, countLineY], 'Color', 'y', 'LineWidth', 2);
    line([1, w], [speedLineY1, speedLineY1], 'Color', 'c', 'LineWidth', 1, 'LineStyle', '--');
    line([1, w], [speedLineY2, speedLineY2], 'Color', 'c', 'LineWidth', 1, 'LineStyle', '--');
    
    % Draw Car Boxes and IDs
    if ~isempty(tracks)
        for i = 1:length(tracks)
            % Only draw if track is stable (age > threshold)
            if tracks(i).totalVisibleCount > ageThreshold && tracks(i).consecutiveInvisibleCount == 0
                
                % Color Logic: Green = Approaching, Magenta = Counted
                boxColor = 'g';
                if tracks(i).isCounted
                    boxColor = 'm'; 
                end
                
                rectangle('Position', tracks(i).bbox, 'EdgeColor', boxColor, 'LineWidth', 2);
                
                % ID Label
                text(tracks(i).bbox(1), tracks(i).bbox(2) - 10, ...
                    sprintf('ID: %d', tracks(i).id), ...
                    'Color', 'y', 'FontSize', 12, 'FontWeight', 'bold');
            end
        end
    end
    
    % Top Info Bar
    tSec = frameIdx / fps;
    infoStr = sprintf('Frame: %d | Time: %.2fs | COUNT: %d', frameIdx, tSec, vehicleCount);
    text(10, 30, infoStr, 'FontSize', 14, 'Color', 'white', 'BackgroundColor', 'black');
    
    hold off;
    drawnow;
    
    if ~ishandle(hFig)
        break;
    end
end

disp('Simulation finished.');