%% Helper function : HelperSetupVisualization 
% This helper function creates a MATLAB figure window panel divided with
% two sub panels(hPanel1, hPanel2) 
% hPanel1 shows Ego Centric Occupancy Map
% hPanel2 shows Driving Scenario
% This function takes scenario handle is to plot the scenario and 
% outputs sub-panel2 handle: sub-panel2 window will be updated in the
% helper function exampleHelperUpdateVisualization().

% Copyright 2019 The MathWorks, Inc.

function hAxes2 = HelperSetupVisualization(scenario)
    % Create a handle for figure function 
    hFigure = figure('units', 'normalized', 'outerposition', [0 0 1 1]);
    
    % Create sub-panel1 to show Ego Centric Occupancy Map
    hPanel1 = uipanel(hFigure, 'Units', 'Normalized', 'Position',...
        [1/2 0 1/2 1], 'Title', 'EgoCentricOccupancyMap');
    
    % Create sub-panel2 to show Driving Scenario
    hPanel2 = uipanel(hFigure, 'Units', 'Normalized', 'Position',...
        [0 0 1/2 1], 'Title', 'DrivingScenario');
    
    % Create a graphics handle for Panel2
    hAxes1 = axes('Parent', hPanel2);
    
    % Create a graphics handle for Panel1
    hAxes2 = axes('Parent', hPanel1);
    
    % Ploting the driving scenario in Panel1
    plot(scenario, 'Parent', hAxes1);
end