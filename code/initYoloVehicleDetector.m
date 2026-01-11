function detector = initYoloVehicleDetector()
%INITYOLOVEHICLEDETECTOR Initialize a YOLOv2-based vehicle detector.
%   This uses a pretrained tiny YOLOv2 model on COCO.
%   Requires Computer Vision Toolbox and Deep Learning Toolbox.

    % Try to load from a MAT file first (optional cache)
    cacheFile = 'yolo_tiny_coco_detector.mat';
    if isfile(cacheFile)
        s = load(cacheFile, 'detector');
        detector = s.detector;
        return;
    end

    % Otherwise, create a new detector (MATLAB may download weights)
    detector = yolov2ObjectDetector('tiny-yolov2-coco');

    % Save for faster reuse next time
    save(cacheFile, 'detector');
end
