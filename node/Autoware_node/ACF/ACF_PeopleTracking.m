%% initialize
clear; clc;
rosshutdown;
rosinit;

%% define global variable
global detector;
global tracks;
global nextId;
global obj;
global option;

detector = peopleDetectorACF('caltech');
tracks = initializeTracks();
nextId = 1;

ld = load('pedScaleTable.mat', 'pedScaleTable');
obj.pedScaleTable = ld.pedScaleTable;

option.ROI                  = [40 95 400 140];  % A rectangle [x, y, w, h] that limits the processing area to ground locations.
option.scThresh             = 0.3;              % A threshold to control the tolerance of error in estimating the scale of a detected pedestrian. 
option.gatingThresh         = 0.9;              % A threshold to reject a candidate match between a detection and a track.
option.gatingCost           = 100;              % A large value for the assignment cost matrix that enforces the rejection of a candidate match.
option.costOfNonAssignment  = 10;               % A tuning parameter to control the likelihood of creation of a new track.
option.timeWindowSize       = 16;               % A tuning parameter to specify the number of frames required to stabilize the confidence score of a track.
option.confidenceThresh     = 2;                % A threshold to determine if a track is true positive or false alarm.
option.ageThresh            = 8;                % A threshold to determine the minimum length required for a track being true positive.
option.visThresh            = 0.6;              % A threshold to determine the minimum visibility value for a track being true positive.

%% define node
ptm_node = robotics.ros.Node('pedestrian_tracking_matlab');
ptm_sub = robotics.ros.Subscriber(ptm_node, '/image_raw', 'sensor_msgs/Image', @callback_trackingACF);


%% function
function tracks = initializeTracks()
    % Create an empty array of tracks
    tracks = struct(...
        'id', {}, ...
        'color', {}, ...
        'bboxes', {}, ...
        'scores', {}, ...
        'kalmanFilter', {}, ...
        'age', {}, ...
        'totalVisibleCount', {}, ...
        'confidence', {}, ...
        'predPosition', {});
end

%% callback function
function callback_trackingACF(~, msg)
	%% define global variable
	global detector;
	global tracks;
	global nextId;
	global obj;
	global option;

	image = readImage(msg);

	[centroids, bboxes, scores] = detectPeople();

	predictNewLocationsOfTracks();

    [assignments, unassignedTracks, unassignedDetections] = detectionToTrackAssignment();

    updateAssignedTracks();
    updateUnassignedTracks();
    deleteLostTracks();
    createNewTracks();

    displayTrackingResults(image);

%% Nested function

% detect People
    function [centroids, bboxes, scores] = detectPeople()
        % Resize the image to increase the resolution of the pedestrian.
        % This helps detect people further away from the camera.
        resizeRatio = 1.5;
        image = imresize(image, resizeRatio, 'Antialiasing',false);

        % Run ACF people detector within a region of interest to produce
        % detection candidates.
        [bboxes, scores] = detect(detector, image)

%        [bboxes, scores] = detect(detector, image, option.ROI, ...
%            'WindowStride', 2,...
%            'NumScaleLevels', 4, ...
%            'SelectStrongest', false);

        % Look up the estimated height of a pedestrian based on location of their feet.
        height = bboxes(:, 4) / resizeRatio;
        y = (bboxes(:,2)-1) / resizeRatio + 1;
        yfoot = min(length(obj.pedScaleTable), round(y + height));
        estHeight = obj.pedScaleTable(yfoot);

        % Remove detections whose size deviates from the expected size,
        % provided by the calibrated scale estimation.
        invalid = abs(estHeigh - height) > estHeight * option.scThresh;
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

% predict new locations of tracks
	function predictNewLocationsOfTracks()
        for i = 1:length(tracks)
            % Get the last bounding box on this track.
            bbox = tracks(i).bboxes(end, :);

            % Predict the current location of the track.
            predictedCentroid = predict(tracks(i).kalmanFilter);

            % Shift the bounding box so that its center is at the predicted location.
            tracks(i).predPosition = [predictedCentroid - bbox(3:4)/2, bbox(3:4)];
        end
    end

% detection to track assignment
    function [assignments, unassignedTracks, unassignedDetections] =  detectionToTrackAssignment()


        predBboxes = reshape([tracks(:).predPosition], 4, [])';
		% ' is written for sublime
        cost = 1 - bboxOverlapRatio(predBboxes, bboxes);


        cost(cost > option.gatingThresh) = 1 + option.gatingCost;

        % Solve the assignment problem.
        [assignments, unassignedTracks, unassignedDetections] = assignDetectionsToTracks(cost, option.costOfNonAssignment);
    end

        function updateAssignedTracks()
        numAssignedTracks = size(assignments, 1);
        for i = 1:numAssignedTracks
            trackIdx = assignments(i, 1);
            detectionIdx = assignments(i, 2);

            centroid = centroids(detectionIdx, :);
            bbox = bboxes(detectionIdx, :);

            correct(tracks(trackIdx).kalmanFilter, centroid);

            % Stabilize the bounding box by taking the average of the size
            % of recent (up to) 4 boxes on the track.
            T = min(size(tracks(trackIdx).bboxes,1), 4);
            w = mean([tracks(trackIdx).bboxes(end-T+1:end, 3); bbox(3)]);
            h = mean([tracks(trackIdx).bboxes(end-T+1:end, 4); bbox(4)]);
            tracks(trackIdx).bboxes(end+1, :) = [centroid - [w, h]/2, w, h];

            tracks(trackIdx).age = tracks(trackIdx).age + 1;

            tracks(trackIdx).scores = [tracks(trackIdx).scores; scores(detectionIdx)];

            % Update visibility.
            tracks(trackIdx).totalVisibleCount = ...
                tracks(trackIdx).totalVisibleCount + 1;

            % Adjust track confidence score based on the maximum detection
            % score in the past 'timeWindowSize' frames.
            T = min(option.timeWindowSize, length(tracks(trackIdx).scores));
            score = tracks(trackIdx).scores(end-T+1:end);
            tracks(trackIdx).confidence = [max(score), mean(score)];
        end
    end

    function updateUnassignedTracks()
        for i = 1:length(unassignedTracks)
            idx = unassignedTracks(i);
            tracks(idx).age = tracks(idx).age + 1;
            tracks(idx).bboxes = [tracks(idx).bboxes; tracks(idx).predPosition];
            tracks(idx).scores = [tracks(idx).scores; 0];

            % Adjust track confidence score based on the maximum detection
            % score in the past 'timeWindowSize' frames
            T = min(option.timeWindowSize, length(tracks(idx).scores));
            score = tracks(idx).scores(end-T+1:end);
            tracks(idx).confidence = [max(score), mean(score)];
        end
    end

    function deleteLostTracks()
        if isempty(tracks)
            return;
        end

        ages = [tracks(:).age]';
		% ' is written for sublime
        totalVisibleCounts = [tracks(:).totalVisibleCount]';
		% ' is written for sublime
        visibility = totalVisibleCounts ./ ages;

        % Check the maximum detection confidence score.
        confidence = reshape([tracks(:).confidence], 2, [])';
		% ' is written for sublime
        maxConfidence = confidence(:, 1);

        % Find the indices of 'lost' tracks.
        lostInds = (ages <= option.ageThresh & visibility <= option.visThresh) | ...
             (maxConfidence <= option.confidenceThresh);

        % Delete lost tracks.
        tracks = tracks(~lostInds);
    end

    function createNewTracks()
        unassignedCentroids = centroids(unassignedDetections, :);
        unassignedBboxes = bboxes(unassignedDetections, :);
        unassignedScores = scores(unassignedDetections);

        for i = 1:size(unassignedBboxes, 1)
            centroid = unassignedCentroids(i,:);
            bbox = unassignedBboxes(i, :);
            score = unassignedScores(i);

            % Create a Kalman filter object.
            kalmanFilter = configureKalmanFilter('ConstantVelocity', ...
                centroid, [2, 1], [5, 5], 100);

            % Create a new track.
            newTrack = struct(...
                'id', nextId, ...
                'color', 255*rand(1,3), ...
                'bboxes', bbox, ...
                'scores', score, ...
                'kalmanFilter', kalmanFilter, ...
                'age', 1, ...
                'totalVisibleCount', 1, ...
                'confidence', [score, score], ...
                'predPosition', bbox);

            % Add it to the array of tracks.
            tracks(end + 1) = newTrack; %#ok<AGROW>

            % Increment the next id.
            nextId = nextId + 1;
        end
    end

   function image = displayTrackingResults(image)

        displayRatio = 4/3;
        image = imresize(image, displayRatio);

        if ~isempty(tracks),
            ages = [tracks(:).age]';
            confidence = reshape([tracks(:).confidence], 2, [])';
            maxConfidence = confidence(:, 1);
            avgConfidence = confidence(:, 2);
            opacity = min(0.5,max(0.1,avgConfidence/3));
            noDispInds = (ages < option.ageThresh & maxConfidence < option.confidenceThresh) | ...
                       (ages < option.ageThresh / 2);

            for i = 1:length(tracks)
                if ~noDispInds(i)

                    % scale bounding boxes for display
                    bb = tracks(i).bboxes(end, :);
                    bb(:,1:2) = (bb(:,1:2)-1)*displayRatio + 1;
                    bb(:,3:4) = bb(:,3:4) * displayRatio;


                    image = insertShape(image, ...
                                            'FilledRectangle', bb, ...
                                            'Color', tracks(i).color, ...
                                            'Opacity', opacity(i));
                    image = insertObjectAnnotation(image, ...
                                            'rectangle', bb, ...
                                            num2str(avgConfidence(i)), ...
                                            'Color', tracks(i).color);
                end
            end
        end

        % image = insertShape(image, 'Rectangle', option.ROI * displayRatio, 'Color', [255, 0, 0], 'LineWidth', 3);

        step(obj.videoPlayer, image);


    end

end
