%% main_yolo_count_speed.m
% YOLO-based vehicle detection + simple tracking + counting + speed estimation
% with perspective calibration (fitgeotrans + transformPointsForward)

clear; clc; close all;

%% 1. Video configuration

videoFile = "data/video4.mp4";  % <- change if your path/name is different

if ~isfile(videoFile)
    error('Video file not found: %s', videoFile);
end

v   = VideoReader(videoFile);
fps = v.FrameRate;

fprintf('Loaded video: %s\n', videoFile);
fprintf('Resolution: %d x %d | FPS: %.2f | Duration: %.2f s\n', ...
    v.Width, v.Height, fps, v.Duration);

%% 2. Initialize YOLO detector (must exist in path)
% You already created this in previous steps.
detector = initYoloVehicleDetector();
   
%% 3. Line positions (in pixels, tune for your video)

countLineY  = 460;   % counting line (yellow)
speedLineY1 = 550;   % upper speed line (cyan)
speedLineY2 = 650;   % lower speed line (cyan)

%% 4. Perspective calibration (projective transform)
% This uses four road-plane corners and maps them to a rectangle in metres.

calibrationFile    = "calibration_T.mat";
forceRecalibrate   = false;    % set true if you want to re-click the 4 points

if forceRecalibrate || ~isfile(calibrationFile)
    fprintf('\n=== Perspective calibration ===\n');
    fprintf('A figure will open. Click 4 road-plane corners in CLOCKWISE order:\n');
    fprintf('Top-left, top-right, bottom-right, bottom-left of the road patch.\n\n');

    % Use the first frame for calibration
    firstFrame = readFrame(v);
    figure; imshow(firstFrame);
    title('Click 4 road-plane corners (clockwise: TL, TR, BR, BL)', ...
        'Color','y','FontSize',12);

    [x, y] = ginput(4);
    close(gcf);

    imagePoints = [x, y];

    % Approximate real-world rectangle (metres)
    % Adjust these values according to your scene (road width & length).
    roadWidth_m  = 15;   % e.g. 3 lanes * 3.5–3.7 m
    roadLength_m = 40;   % length along driving direction

    worldPoints = [ ...
        0,            0; ...
        roadWidth_m,  0; ...
        roadWidth_m,  roadLength_m; ...
        0,            roadLength_m];

    tform = fitgeotrans(imagePoints, worldPoints, "projective");

    save(calibrationFile, "tform", "roadWidth_m", "roadLength_m", ...
        "imagePoints", "worldPoints");

    % Re-open the VideoReader from the beginning
    v = VideoReader(videoFile);
else
    load(calibrationFile, "tform", "roadWidth_m", "roadLength_m", ...
        "imagePoints", "worldPoints");
    % Reset video to start
    v = VideoReader(videoFile);
end

% Use perspective transform to compute real-world distance between speed lines
xCenter = v.Width / 2;
pt1_img = [xCenter, speedLineY1];
pt2_img = [xCenter, speedLineY2];

[pt1_worldX, pt1_worldY] = transformPointsForward(tform, pt1_img(1), pt1_img(2));
[pt2_worldX, pt2_worldY] = transformPointsForward(tform, pt2_img(1), pt2_img(2));

distanceBetweenLines_m = hypot(pt2_worldX - pt1_worldX, pt2_worldY - pt1_worldY);

fprintf('Estimated real-world distance between speed lines: %.2f m\n', ...
    distanceBetweenLines_m);

%% 5. Tracking data structures and counting constraints

tracks = struct( ...
    "id",           {}, ...
    "centroid",     {}, ...
    "prevCentroid", {}, ...
    "bbox",         {}, ...
    "lastFrame",    {}, ...
    "tLine1",       {}, ...
    "tLine2",       {}, ...
    "speedKmh",     {}, ...
    "counted",      {}, ...
    "age",          {});     % age in frames

nextTrackID   = 1;
maxMatchDist  = 120;      % keep this for now
totalCount    = 0;

% Counting constraints (tune for your lanes)
roiXMin       = 350;      % include a bit more of left lanes
roiXMax       = 1570;     % include a bit more of right lanes
minBBoxHeight = 30;       % accept slightly smaller cars
minTrackAge   = 3;        % track only needs 3 frames before counting


%% 6. Main loop

hFig     = figure('Name', 'YOLO Count + Speed (Perspective)');
frameIdx = 0;

% --- Create a pause toggle button on the figure ---
setappdata(hFig, 'isPaused', false);

uicontrol('Parent', hFig, ...
    'Style',  'togglebutton', ...
    'String', 'Pause', ...
    'Units',  'pixels', ...
    'Position', [20 20 80 30], ...   % (x,y,width,height) you can adjust
    'Callback', @(src, ~) setappdata(hFig, 'isPaused', get(src, 'Value')));


while hasFrame(v)
    frame = readFrame(v);
    frameIdx = frameIdx + 1;
    tSec     = frameIdx / fps;

    % ---- 6.1 YOLO detection on current frame ----
    [bboxes, scores, labels] = detect(detector, frame, 'Threshold', 0.4);

    % Keep only vehicle classes
    isVehicle = labels == "car" | labels == "bus" | ...
                labels == "truck" | labels == "motorbike";
    bboxes    = bboxes(isVehicle, :);
    scores    = scores(isVehicle);
    labelsV   = labels(isVehicle);

    % Compute centroids of detections
    numDet = size(bboxes, 1);
    centroids = zeros(numDet, 2);
    for i = 1:numDet
        cx = bboxes(i,1) + bboxes(i,3)/2;
        cy = bboxes(i,2) + bboxes(i,4)/2;
        centroids(i,:) = [cx, cy];
    end

    % ---- 6.2 Simple nearest-neighbour tracking ----
    numTracks = numel(tracks);
    assignedTrack = zeros(numDet, 1);   % 0 means new track

    % Match detections to existing tracks (greedy nearest-neighbour)
    for d = 1:numDet
        bestTrack = 0;
        bestDist  = inf;
        for t = 1:numTracks
            dxy  = centroids(d,:) - tracks(t).centroid;
            dist = norm(dxy);
            if dist < bestDist && dist < maxMatchDist
                bestDist  = dist;
                bestTrack = t;
            end
        end
        if bestTrack > 0
            % update existing track
            tracks(bestTrack).prevCentroid = tracks(bestTrack).centroid;
            tracks(bestTrack).centroid     = centroids(d,:);
            tracks(bestTrack).bbox         = bboxes(d,:);
            tracks(bestTrack).lastFrame    = frameIdx;
            tracks(bestTrack).age          = tracks(bestTrack).age + 1;
            assignedTrack(d)               = bestTrack;
        end
    end

    % Create new tracks for unmatched detections
    for d = 1:numDet
        if assignedTrack(d) == 0
            newTrack.id           = nextTrackID;
            newTrack.centroid     = centroids(d,:);
            newTrack.prevCentroid = centroids(d,:);  % same for first frame
            newTrack.bbox         = bboxes(d,:);
            newTrack.lastFrame    = frameIdx;
            newTrack.tLine1       = NaN;
            newTrack.tLine2       = NaN;
            newTrack.speedKmh     = NaN;
            newTrack.counted      = false;
            newTrack.age          = 1;

            tracks(end+1) = newTrack; %#ok<AGROW>
            assignedTrack(d) = numel(tracks);
            nextTrackID = nextTrackID + 1;
        end
    end

    % ---- 6.3 Update counting and speed for each track ----
    for t = 1:numel(tracks)
        cPrev = tracks(t).prevCentroid;
        cNow  = tracks(t).centroid;
        bbox  = tracks(t).bbox;

        if isempty(bbox) || any(isnan(cPrev)) || any(isnan(cNow))
            continue;
        end

        yPrev = cPrev(2);
        yNow  = cNow(2);

        % 6.3.1 Counting: top -> bottom across countLineY, with constraints

if ~tracks(t).counted
    yPrev = cPrev(2);
    yNow  = cNow(2);

   
    crossedDown  = (yPrev < countLineY && yNow >= countLineY);
    crossedUp    = (yPrev > countLineY && yNow <= countLineY);

    if crossedDown || crossedUp
        totalCount = totalCount + 1;
        tracks(t).counted = true;
    end
end


        % 6.3.2 Speed line 1 time stamp
        if isnan(tracks(t).tLine1) && yPrev < speedLineY1 && yNow >= speedLineY1
            tracks(t).tLine1 = tracks(t).lastFrame / fps;
        end

        % 6.3.3 Speed line 2 time stamp
        if isnan(tracks(t).tLine2) && yPrev < speedLineY2 && yNow >= speedLineY2
            tracks(t).tLine2 = tracks(t).lastFrame / fps;
        end

        % 6.3.4 Compute speed once both times are available
        if ~isnan(tracks(t).tLine1) && ~isnan(tracks(t).tLine2) && ...
                isnan(tracks(t).speedKmh)

            dt = tracks(t).tLine2 - tracks(t).tLine1;
            if dt > 0
                speed_mps          = distanceBetweenLines_m / dt;
                tracks(t).speedKmh = speed_mps * 3.6;
            end
        end
    end

    % ---- 6.4 Visualisation ----
    frameOut = frame;

    % Draw lines
    [H, W, ~] = size(frameOut);
    lines = [ ...
        1, countLineY,  W, countLineY;  ...
        1, speedLineY1, W, speedLineY1; ...
        1, speedLineY2, W, speedLineY2];

    lineColors = {'yellow', 'cyan', 'cyan'};
    frameOut = insertShape(frameOut, 'Line', lines, ...
        "Color", lineColors, "LineWidth", 2);

    % Draw bounding boxes + ID + speed (red if speed available)
    for t = 1:numel(tracks)

        % Only draw tracks updated in this frame
        if tracks(t).lastFrame ~= frameIdx
            continue;
        end

        bbox = tracks(t).bbox;
        if isempty(bbox)
            continue;
        end

        hasSpeed = ~isnan(tracks(t).speedKmh);
        labelStr = sprintf('ID %d', tracks(t).id);
        if hasSpeed
            labelStr = sprintf('%s | %.1f km/h', labelStr, tracks(t).speedKmh);
        end

        if hasSpeed
            boxColor = "red";
        else
            boxColor = "green";
        end

        frameOut = insertObjectAnnotation(frameOut, "rectangle", bbox, labelStr, ...
            "Color", boxColor, "LineWidth", 2, "FontSize", 10);
    end

    % Overlay total count and time
    infoText = sprintf('Total count: %d', totalCount);
    frameOut = insertText(frameOut, [10 10], infoText, ...
        "FontSize", 18, "BoxOpacity", 0.6, "TextColor", "white");

    timeText = sprintf('t = %.2f s', tSec);
    frameOut = insertText(frameOut, [10 40], timeText, ...
        "FontSize", 14, "BoxOpacity", 0.6, "TextColor", "yellow");

    % Show frame
   imshow(frameOut, 'Border', 'tight');
title(sprintf('YOLO Count + Speed | frame %d', frameIdx), ...
    "Color", "w", "FontSize", 11);
drawnow;

% --- Pause loop while the toggle button is ON ---
while ishandle(hFig) && getappdata(hFig, 'isPaused')
    pause(0.05);   % small delay to avoid busy-waiting
end

if ~ishandle(hFig)
    break;
end


if ~ishandle(hFig)
    break;
end

end

%% 7. Summary statistics

fprintf('\nFinished. Total counted vehicles: %d\n', totalCount);

allSpeeds = [tracks.speedKmh];
allSpeeds = allSpeeds(~isnan(allSpeeds));

if ~isempty(allSpeeds)
    fprintf('Number of vehicles with valid speed: %d\n', numel(allSpeeds));
    fprintf('Speed statistics (km/h): mean = %.2f, std = %.2f, min = %.2f, max = %.2f\n', ...
        mean(allSpeeds), std(allSpeeds), min(allSpeeds), max(allSpeeds));
else
    fprintf('No valid speed measurements were obtained.\n');
end
