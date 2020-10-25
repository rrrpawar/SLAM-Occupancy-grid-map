%% Function: HelperUpdateVisualization 
% This helper function is to update the sub-panel2 of MATLAB figure window
% This function takes sub-panel2 handle to show the ego centric occupancy
% map (egoMap), and ego vehicle data i.e. ego vehicle dimensions, ego pose
% and ego vehicle yaw.

% Copyright 2019 The MathWorks, Inc.

function HelperUpdateVisualization(hAxes2, egoVehicle, egoPose, ...
                                          egoYaw, egoMap)
    % Show the updated ego centric occupancy map
    show(egoMap, 'Parent', hAxes2);
    
    % hold the shown output until placing the ego vehicle on to the map 
    hold on;
    
    % Create a variable to hold vehicle polygon dimensions
    vehiclePolygon = [egoVehicle.Length, egoVehicle.Width/2; ...
                      egoVehicle.Length, -egoVehicle.Width/2; ...
                      0, -egoVehicle.Width/2; ...
                      0, egoVehicle.Width/2]'; 
    
	% Create a variable to hold current vehicle center point
    vehicleCenter = [egoPose(1), egoPose(2)];
    
    % Replicate current vehicle center values to the size equals to
    % vehiclePolygon
    pointPose = repmat(vehicleCenter, 4, 1)';
    
    % Create a ego vehicle polygon and move it to current ego vehicle pose
    egoPolygon = pointPose + vehiclePolygon;
    
    % Create a rotation matrix R from the current ego vehicle yaw
    R = [cos(egoYaw), -sin(egoYaw); sin(egoYaw), cos(egoYaw)];
    
    % Rotate the ego polygon with rotation matrix R
    egoPolygon = (R * (egoPolygon - pointPose)) + pointPose;
    
    % Create a polygon of egoPolygon size and place it onto above shown
    % image
    patch(egoPolygon(1, :), egoPolygon(2, :), 'b', 'Parent', hAxes2);
    
    % Set the orientation of output to upside
    hAxes2.CameraUpVector = [1 0 0];
end