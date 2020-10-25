[scenario, egoVehicle] = HelperCreateStraightRoadScenario;                           

profiles = actorProfiles(scenario);

% Configure vision detection generator to detect lanes and obstacles
sensor = visionDetectionGenerator('SensorIndex', 1, ...
    'SensorLocation', [1.9 0], ...
    'DetectionProbability', 1, ...
    'MinObjectImageSize', [5 5], ...
    'FalsePositivesPerImage', 0, ...
    'DetectorOutput', 'Lanes and objects', ...
    'Intrinsics', cameraIntrinsics([700 1814],[320 240],[480 640]), ...
    'ActorProfiles', profiles,...
    'UpdateInterval', 0.1);

egoMap = binaryOccupancyMap(100, 100, 2);
setOccupancy(egoMap,ones(200, 200));

egoMap.GridOriginInLocal = [-egoMap.XLocalLimits(2)/2, ...
                            -egoMap.YLocalLimits(2)/2];
                        
hAxes = HelperSetupVisualization(scenario);

% Advancing the scenario
while advance(scenario)
    % Ego vehicle position
    egoPose = egoVehicle.Position;
    egoYaw = deg2rad(egoVehicle.Yaw);
    
    % Move the origin of grid to the face of the ego vehicle.
    move(egoMap, [egoPose(1), egoPose(2)]);
    
    %Reset the egoMap before updating with obstacle information
    setOccupancy(egoMap, ones(egoMap.GridSize));
    
     [obstacleInfo, roadBorders, isValidLaneTime] = ...
      HelperGetObstacleDataFromSensor(scenario, egoMap, ...
                                egoVehicle, sensor);
                            
    [obstaclePoints, unoccupiedSpace] = HelperFilterObstacles(...
                                         egoMap, obstacleInfo, ...
                                         roadBorders, ...
                                         isValidLaneTime, egoVehicle);
                                     
      % Set the occupancy of free space to 0
        if ~isempty(unoccupiedSpace)
            setOccupancy(egoMap, unoccupiedSpace, 0);
        end
        
        % Set the occupancy of occupied space to 1
        if ~isempty(obstaclePoints)
            setOccupancy(egoMap, obstaclePoints, 1);
        end
        
        % Updating the visualization
        HelperUpdateVisualization(hAxes, egoVehicle, egoPose, egoYaw, egoMap);
end