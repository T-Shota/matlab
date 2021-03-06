clear; clc;
rosshutdown;
rosinit;

global obj;
global option;

ld = load('pedScaleTable.mat', 'pedScaleTable');
obj.pedScaleTable = ld.pedScaleTable;
obj.videoPlayer = vision.VideoPlayer('Position', [29, 597,643,386]);


option.ROI                  = [40 95 400 140];  % A rectangle [x, y, w, h] that limits the processing area to ground locations.
option.scThresh             = 1;              % A threshold to control the tolerance of error in estimating the scale of a detected pedestrian. 
option.gatingThresh         = 0.9;              % A threshold to reject a candidate match between a detection and a track.
option.gatingCost           = 100;              % A large value for the assignment cost matrix that enforces the rejection of a candidate match.
option.costOfNonAssignment  = 20;               % A tuning parameter to control the likelihood of creation of a new track.
option.timeWindowSize       = 16;               % A tuning parameter to specify the number of frames required to stabilize the confidence score of a track.
option.confidenceThresh     = 2;                % A threshold to determine if a track is true positive or false alarm.
option.ageThresh            = 8;                % A threshold to determine the minimum length required for a track being true positive.
option.visThresh            = 0.6;              % A threshold to determine the minimum visibility value for a track being true positive.


node = robotics.ros.Node('test1');
sub = robotics.ros.Subscriber(node, '/image_raw', 'sensor_msgs/Image', @image_callback);

function image_callback(~, msg)
	global obj;
    global option;

    Image = readImage(msg);
    detector = peopleDetectorACF('caltech');

    [centroids, bboxes, scores] = detectPeople();

    function [centroids, bboxes, scores] = detectPeople()
        % Resize the image to increase the resolution of the pedestrian.
        % This helps detect people further away from the camera.
        resizeRatio = 1;
        Image = imresize(Image, resizeRatio, 'Antialiasing',false);

        % Run ACF people detector within a region of interest to produce
        % detection candidates.
        [bboxes, scores] = detect(detector, Image);

        % Look up the estimated height of a pedestrian based on location of their feet.
        height = bboxes(:, 4) / resizeRatio;
        y = (bboxes(:,2)-1) / resizeRatio + 1;
        yfoot = min(length(obj.pedScaleTable), round(y + height));
        estHeight = obj.pedScaleTable(yfoot);

        % Remove detections whose size deviates from the expected size,
        % provided by the calibrated scale estimation.
        invalid = abs(estHeight-height)>estHeight*option.scThresh;
        bboxes(invalid, :) = [];
        scores(invalid, :) = [];

        % Apply non-maximum suppression to select the strongest bounding boxes.
        [bboxes, scores] = selectStrongestBbox(bboxes, scores, ...
                            'RatioType', 'Min', 'OverlapThreshold', 0.6);

        % Compute the centroids
        if isempty(bboxes)
            centroids = [];
        else
            centroids = [(bboxes(:, 1) + bboxes(:, 3) / 2), ...
                (bboxes(:, 2) + bboxes(:, 4) / 2)];
        end
    end

    if ~isempty(scores)
    	Image = insertObjectAnnotation(Image, 'rectangle', bboxes, scores);
    end

    step(obj.videoPlayer, Image);

end