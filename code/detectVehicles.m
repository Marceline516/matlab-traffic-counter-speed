% %%CODE I USED TO INSERT INTO SKELETON:  3. Preview loop: video reading + display + detection
% hFig = figure('Name', 'Traffic Video Preview – Step 1 Complete');
% frameIdx = 0;
% 
% % --- MOVE LINES DOWN TO ROAD LEVEL (Optional but Recommended) ---
% countLineY  = 600;   
% speedLineY1 = 500;
% speedLineY2 = 700;
% 
% while hasFrame(v)
%     frame = readFrame(v);
%     frameIdx = frameIdx + 1;
% 
%     % --- STEP 1: CALL YOUR FUNCTION ---
%     % This line sends the image to your detectVehicles.m file
%     % and gets back the box locations (bboxes) and center points (centroids).
%     [bboxes, centroids] = detectVehicles(frame); 
%     % ----------------------------------
% 
%     % Show raw frame
%     imshow(frame, 'Border', 'tight');
%     hold on;
% 
%     % --- VISUALIZATION: Draw Green Boxes ---
%     if ~isempty(bboxes)
%         for i = 1:size(bboxes, 1)
%             rectangle('Position', bboxes(i,:), 'EdgeColor', 'g', 'LineWidth', 2);
%             plot(centroids(i,1), centroids(i,2), 'r+'); % Red cross at center
%         end
%     end
% 
%     % Get width for drawing horizontal lines
%     [h, w, ~] = size(frame); 
% 
%     % --- Draw Lines (Yellow & Cyan) ---
%     line([1, w], [countLineY, countLineY], 'Color', 'y', 'LineWidth', 2);
%     line([1, w], [speedLineY1, speedLineY1], 'Color', 'c', 'LineWidth', 1, 'LineStyle', '--');
%     line([1, w], [speedLineY2, speedLineY2], 'Color', 'c', 'LineWidth', 1, 'LineStyle', '--');
% 
%     % Title: frame index + time stamp
%     tSec = frameIdx / fps;
%     title(sprintf('Frame %d | t = %.2f s', frameIdx, tSec), 'FontSize', 12, 'Color', 'w');
% 
%     hold off;
%     drawnow;
% 
%     if ~ishandle(hFig), break; end
% end
% disp('Preview finished.');

function [bboxes, centroids] = detectVehicles(frame)
    % This function finds the cars.
    % some numbers need to be adjusted depending on video 
    
    persistent detector blobAnalyzer
    
    % --- INITIALIZATION (Runs once) ---
    if isempty(detector)
        detector = vision.ForegroundDetector(...
            'NumGaussians', 5, ... %handles blurs like trees or moving camera
            'NumTrainingFrames', 50, ...  % <--- Learn fast the lower the number, change if needed
            'MinimumBackgroundRatio', 0.7);
            
        blobAnalyzer = vision.BlobAnalysis(...
            'BoundingBoxOutputPort', true, ...
            'CentroidOutputPort', true, ...
            'AreaOutputPort', false, ...
            'MinimumBlobArea', 250); % increase to remove smaller boxes, decrease to keep smaller boxes 
    end
    
    % --- PROCESSING (Runs every frame) ---
    mask = detector.step(frame);
    
    % Clean up noise
    mask = imclose(mask, strel('rectangle', [4,4])); % "If two moving things are within _ pixels of each other, they are the same car" change number if necessary
    mask = imfill(mask, 'holes');
    
    % Get boxes
    [centroids, bboxes] = blobAnalyzer.step(mask);
end