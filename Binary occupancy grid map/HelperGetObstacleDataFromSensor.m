%% Helper function : HelperGetObstacleDataFromSensor
% This helper function is to detect obstacles and road boundaries using
% vision detection generator object. These detections comes in ego
% co-ordinates which will convert to world co-ordinates. Detections
% information will be further processed in the next function
% exampleHelperFilterObstacles()

% This function takes scenario(Driving Scenario handle), egoMap(ego centric
% occupancy map), egoVehicle(ego vehicle handle), sensor(configured vision
% detection sensor object)

% Copyright 2019 The MathWorks, Inc.

function [obstacleInfo, roadBorders, isValidLaneTime] = ...
      HelperGetObstacleDataFromSensor(scenario, egoMap, ...
                               egoVehicle, sensor)

    % The road boundaries information can also be obtained from the
    % scenario(i.e. ground-truth). 
    % Creates an option to switch between sensor and ground-truth for road
    % boundary information. By default(false) it uses ground-truth as it
    % gives exact info
    roadBoundariesFromTruth = true;

    % Initializing output arrays
    obstacleInfo = [];
    roadBorders = [];

    % A struct array of target vehicles information relative to ego vehicle
    targetVehiclesPoses = targetPoses(egoVehicle);
    
    % Generate time-stamped sensor detections
    simTime = scenario.SimulationTime;
    
    % Create a parametric variable look ahead distance for lane
    % detection algorithm
    lookaheadDistance = 0:(1/egoMap.Resolution):egoMap.XLocalLimits(2);
    
    % Configure lane boundaries function to get all boundaries
    lbs = laneBoundaries(egoVehicle, 'XDistance', lookaheadDistance, ...
                        'AllBoundaries', true);
                    
    % Invoke vision detection generator object 
    % This function returns detected obstacles info and number of obstacles 
    % detected, lane detections info and number of lanes detected, 
    % other validation flags
    [objectDetections, numObjects, ~, laneDetections, ...
    numValidLaneDetections, isValidLaneTime] = ...
                                 sensor(targetVehiclesPoses, lbs, simTime);

    % Discarding invalid detections by using numObjects output
    objectDetections = objectDetections(1:numObjects);

    % Get ego orientation
    egoOrientation = [deg2rad(egoVehicle.Yaw), ...
                      deg2rad(egoVehicle.Pitch), ...
                      deg2rad(egoVehicle.Roll)];

    % Calculate transformation matrix, which is used for transforming
    % detections from ego co-ordinates to world co-ordinates
    rotation = eul2rotm(egoOrientation);

    % Create transformation matrix
    transfromToWorld = [[rotation , egoVehicle.Position']; [0 0 0 1]];

    % Transforming all detections from ego co-ordinates to world
    % co-ordinates. Assuming that all the obstacles are in parallel with
    % ego vehicle (as it is straight road scenario)
    obstacleInfo = zeros(numObjects, 4);
    if numObjects
        for i = 1:numObjects
            obstaclePosition = [objectDetections{i}.Measurement(1:3); 1];
            transformedPose = transfromToWorld * obstaclePosition;
            obstacleInfo(i, :) = transformedPose';
        end
    end


    % Check which source needs to be used to get road boundary information
    if(~roadBoundariesFromTruth)
        % If sensor detections considered, then check for the validity of
        % lane detections, considers only valid detections, returns if it
        % is invalid
        if(~isValidLaneTime)
            return;
        end
        % Define empty array to hold lanes information
        laneInfo = [];
        
        % Check valid detections using the flag numValidLaneDetections
        if(numValidLaneDetections)
            % Lane boundaries will have all detected lanes, this example
            % needs only road boundaries, need to discard lanes which are
            % inside those boundaries.
            % Here, it finds road boundaries using BoundaryType flag,
            % stores it into output array
            boundaries = laneDetections.LaneBoundaries;
            boundaryCount = 0;
            for i = 1:numValidLaneDetections
                % Check for boundary type. Usually the Boundary type is
                % 'Solid' for outer road boundaries.
                if (strcmp(boundaries(i).BoundaryType, 'Solid') == true)
                    boundaryCount = boundaryCount + 1;
                    % Extract lane boundary points using the boundary model
                    queryX = 1:0.5:100;
                    laneY = computeBoundaryModel(boundaries(i), queryX);
                    laneX = queryX;
                    laneInfo = [laneX' laneY'];
                    laneInfo = laneInfo(~isnan(laneInfo(:,1))...
                        & ~isnan(laneInfo(:,2)),:);
                    
                    % Transform lane points to world coordinates
                    laneInfo = horzcat(laneInfo, zeros(...
                        length(laneInfo), 1),ones(length(laneInfo), 1));
                    if ~isempty(laneInfo)
                        laneInfo = laneInfo * transfromToWorld';
                        
                        % Remove added invalid columns 
                        laneInfo(:, [3, 4]) = [];
                        
                        % Add left and right road boundaries to output
                        % array
                        if(boundaryCount == 1)
                            roadBorders = vertcat(roadBorders, laneInfo);
                        else
                            roadBorders = vertcat(roadBorders,...
                                flipud(laneInfo));
                        end
                    end
                end
            end
        end
    else
        % Storing road boundary information from scenario(ground-truth)
        rbs = roadBoundaries(scenario);
        for index =1:length(rbs)
            boundary = rbs{index};
            roadBorders = [roadBorders, boundary(:, [1, 2])];
        end
    end

end                            