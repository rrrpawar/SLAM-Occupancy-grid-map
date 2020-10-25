%% Helper function : HelperFilterObstacles
% This functions is to filter detections generated from vision detection
% generator. Depending upon the range of the vision detection generator,
% the detections may fall outside the ego centric occupancy map bounds.
% Using the egoMap bounds, this filtering step will considers only
% detections which are falling inside the current egoMap and discards which
% are not fall inside.

% This function takes egoMap(ego centric occupancy map object),
% obstacleInfo (obstacle information), laneBorders(lane boundaries),
% isValidLaneTime(based on this flag, it takes only valid lane detections
% at each time), egoVehicle(ego vehicle) and outputs obstacles which are
% fall inside the ego centric occupancy map and unoccupied space

% Copyright 2019 The MathWorks, Inc.

function [obstaclePoints, unoccupiedSpace] = ...
        HelperFilterObstacles(egoMap, obstacleInfo, ...
                                     laneBorders, ...
                                     isValidLaneTime, egoVehicle)
    % Initializing the output arrays as empty
    obstaclePoints = [];
    obstacleData = [];
    unoccupiedSpace = [];
    
    % Create a variable to hold vehicle dimensions. Assuming all the
    % obstacles are of the same dimensions
    vehiclePolygon = [egoVehicle.Length, egoVehicle.Width/2; ...
                  egoVehicle.Length, -egoVehicle.Width/2; ...
                  0, -egoVehicle.Width/2; ...
                  0, egoVehicle.Width/2]'; 
    
    % If there are obstacles detected, then check for obstacle points which
    % are falling inside the ego centric occupancy map from the vehicle
    % location
    if ~isempty(obstacleInfo)
        % If obstacleInfo has valid detections
        for j = 1:size(obstacleInfo, 1)
            % obstacleInfo will have only one point for each detected
            % vehicle and it is needed to construct vehicle polygon using
            % the vehicle center info
            vehicleCenter = [obstacleInfo(j, 1), obstacleInfo(j, 2)];
            obstaclePolygon = repmat(vehicleCenter, 4, 1)'+vehiclePolygon;
            obstacleData = [obstacleData; obstaclePolygon, ...
                            obstaclePolygon(:, 1),[NaN; NaN]];
        end
        if(~isempty(obstacleData))
            % Find bounds of egoMap
            [xPoints , yPoints] = meshgrid(0:0.5:100);
            xPoints = xPoints(:) + egoMap.XWorldLimits(1);
            yPoints = yPoints(:) + egoMap.YWorldLimits(1);
            
            % Filter the detections which are fall inside the egoMap bounds
            % which are calculated in above step
            [in, on] = inpolygon(xPoints, yPoints, obstacleData(1, :), ...
            obstacleData(2, :));
            obstaclePoints = [obstaclePoints; xPoints(in), yPoints(in); ...
                xPoints(on), yPoints(on)];
            obstaclePoints = obstaclePoints(obstaclePoints(:, 1) < ...
                egoMap.XWorldLimits(2)  & ...
                obstaclePoints(:, 1) > egoMap.XWorldLimits(1) & ...
                obstaclePoints(:, 2) < egoMap.YWorldLimits(2)  & ...
                obstaclePoints(:, 2) > egoMap.YWorldLimits(1), :);
        end
    end
    
    % This step is to find the road boundaries which are fall inside egoMap
    % bounds and discards which are not fall inside
    if(isValidLaneTime)
        % Goes inside if vision detection generator gives valid lane
        % detections
        if ~isempty(laneBorders)
            % Find bounds of egoMap
            [xPoints, yPoints] = meshgrid(0:0.5:100);
            xPoints = (xPoints(:) + egoMap.XWorldLimits(1))';
            yPoints = (yPoints(:) + egoMap.YWorldLimits(1))';
            % Using inpolygon function will get the points inside road
            % boundaries
            % This will form the road region
            in = inpolygon(xPoints, yPoints, ...
                           laneBorders(:, 1),laneBorders(:, 2));

            % Find unoccupied space using egoMap bounds
            unoccupiedSpace = [xPoints(in)', yPoints(in)'];            
            unoccupiedSpace = unoccupiedSpace(unoccupiedSpace(:, 1) < ...
                egoMap.XWorldLimits(2) & ...
                unoccupiedSpace(:, 1) > egoMap.XWorldLimits(1) & ...
                unoccupiedSpace(:, 2) < egoMap.YWorldLimits(2) & ...
                unoccupiedSpace(:, 2) > egoMap.YWorldLimits(1), :);
        end
    end
                                     
end