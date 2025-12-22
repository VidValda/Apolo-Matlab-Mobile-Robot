function test_connection
    % ApoloSensorTestGUI_Updated
    % Updates:
    % 1. Default robot name set to 'Marvin' (standard in Apolo docs).
    % 2. apoloUpdate() added to all sensor callbacks to refresh the simulator.

    % --- Create Main Figure ---
    fig = uifigure('Name', 'Apolo Sensor Tester (Refresh Enabled)', 'Position', [100, 100, 800, 600]);
    
    % --- Create Main Grid Layout ---
    mainGrid = uigridlayout(fig, [3, 1]);
    mainGrid.RowHeight = {'1.2x', '3x', '2x'};
    
    % --- Store Handles and Data ---
    % Changed default from 'Elder' to 'Marvin' to match standard docs
    fig.UserData.robotName = 'Marvin'; 
    fig.UserData.laserName = 'LMS100'; 
    
    % --- Create UI Panels ---
    createControlPanel(mainGrid, fig);
    createPlotPanel(mainGrid, fig);
    createOutputPanel(mainGrid, fig);
    
    % --- Initial Log Message ---
    logMessage(fig, 'GUI Started. Added apoloUpdate() to all reads.');
    logMessage(fig, 'Default Robot: ''Marvin'', Default Laser: ''LMS100''');
end

% =========================================================================
% UI Panel Creation Functions
% =========================================================================

function createControlPanel(parentGrid, fig)
    panel = uipanel(parentGrid, 'Title', 'Controls');
    grid = uigridlayout(panel, [3, 4]);
    grid.ColumnWidth = {'1.2x', '1x', '1x', '1x'};
    
    % --- Row 1: Name Inputs ---
    uilabel(grid, 'Text', 'Robot Name:');
    fig.UserData.RobotNameField = uieditfield(grid, 'Value', fig.UserData.robotName);
    
    uilabel(grid, 'Text', 'Laser Name:');
    fig.UserData.LaserNameField = uieditfield(grid, 'Value', fig.UserData.laserName);
    
    % --- Row 2: Movement Buttons ---
    uibutton(grid, 'Text', 'Forward', 'ButtonPushedFcn', @(src,evt) moveRobot(fig, 'forward'));
    uibutton(grid, 'Text', 'Turn Left', 'ButtonPushedFcn', @(src,evt) moveRobot(fig, 'left'));
    uibutton(grid, 'Text', 'Turn Right', 'ButtonPushedFcn', @(src,evt) moveRobot(fig, 'right'));
    uibutton(grid, 'Text', 'Stop', 'ButtonPushedFcn', @(src,evt) moveRobot(fig, 'stop'));
    
    % --- Row 3: Sensor/Action Buttons ---
    uibutton(grid, 'Text', 'Backward', 'ButtonPushedFcn', @(src,evt) moveRobot(fig, 'backward'));
    uibutton(grid, 'Text', 'Get Odometry', 'ButtonPushedFcn', @(src,evt) getOdometryData(fig));
    uibutton(grid, 'Text', 'Reset Odometry', 'ButtonPushedFcn', @(src,evt) resetOdometry(fig));
    uibutton(grid, 'Text', 'Get All Sonars', 'ButtonPushedFcn', @(src,evt) getAllSonars(fig));
end

function createPlotPanel(parentGrid, fig)
    panel = uipanel(parentGrid, 'Title', 'Laser Scan (apoloGetLaserData)');
    panelGrid = uigridlayout(panel, [1, 2]);
    panelGrid.ColumnWidth = {'3x', '1x'};
    
    ax = uiaxes(panelGrid);
    ax.DataAspectRatio = [1 1 1];
    title(ax, 'Polar Laser Scan (Cartesian View)');
    grid(ax, 'on');
    fig.UserData.Axes = ax;
    
    uibutton(panelGrid, 'Text', 'Get Laser Scan', ...
             'ButtonPushedFcn', @(src,evt) getLaserScan(fig));
end

function createOutputPanel(parentGrid, fig)
    panel = uipanel(parentGrid, 'Title', 'Sensor Data Output');
    panelGrid = uigridlayout(panel, [1, 2]);
    panelGrid.ColumnWidth = {'3x', '1x'};
    
    ta = uitextarea(panelGrid, 'Editable', 'off', 'Value', {''});
    fig.UserData.TextArea = ta;
    
    uibutton(panelGrid, 'Text', 'Get Landmarks', ...
             'ButtonPushedFcn', @(src,evt) getLaserLandmarks(fig));
end

% =========================================================================
% Helper Functions
% =========================================================================

function updateNames(fig)
    fig.UserData.robotName = fig.UserData.RobotNameField.Value;
    fig.UserData.laserName = fig.UserData.LaserNameField.Value;
end

function logMessage(fig, message)
    try
        ta = fig.UserData.TextArea;
        timestamp = datestr(now, 'HH:MM:SS');
        newMessage = sprintf('[%s] %s', timestamp, message);
        ta.Value = [newMessage; ta.Value];
    catch ME
        disp(['Error logging message: ' ME.message]);
    end
end

% =========================================================================
% Callback Functions (Button Actions)
% =========================================================================

function moveRobot(fig, direction)
    updateNames(fig);
    robotName = fig.UserData.robotName;
    
    % Standard speed settings
    speed = 0.1; % m/s
    rotSpeed = 0.2; % rad/s
    time = 0.1; % s
    
    try
        switch direction
            case 'forward'
                apoloMoveMRobot(robotName, [speed, 0], time);
            case 'backward'
                apoloMoveMRobot(robotName, [-speed, 0], time);
            case 'left'
                apoloMoveMRobot(robotName, [0, rotSpeed], time);
            case 'right'
                apoloMoveMRobot(robotName, [0, -rotSpeed], time);
            case 'stop'
                apoloMoveMRobot(robotName, [0, 0], time);
        end
        
        % [UPDATED] Critical for refreshing the view after movement
        apoloUpdate(); 
        logMessage(fig, ['Move: ' direction]);
        
        getOdometryData(fig); 
        
    catch ME
        logMessage(fig, ['Error moving robot: ' ME.message]);
    end
end

function getOdometryData(fig)
    updateNames(fig);
    robotName = fig.UserData.robotName;
    
    try
        pos = apoloGetOdometry(robotName);
        
        % [UPDATED] Refresh view after reading data
        apoloUpdate(); 
        
        logMessage(fig, sprintf('Odometry: X=%.3f, Y=%.3f, Theta=%.3f', pos(1), pos(2), pos(3)));
    catch ME
        logMessage(fig, ['Error getting odometry: ' ME.message]);
    end
end

function resetOdometry(fig)
    updateNames(fig);
    robotName = fig.UserData.robotName;
    
    try
        apoloResetOdometry(robotName);
        
        % [UPDATED] Refresh view after reset
        apoloUpdate(); 
        
        logMessage(fig, 'Odometry Reset to [0, 0, 0].');
        getOdometryData(fig); 
    catch ME
        logMessage(fig, ['Error resetting odometry: ' ME.message]);
    end
end

function getLaserScan(fig)
    updateNames(fig);
    laserName = fig.UserData.laserName;
    ax = fig.UserData.Axes;
    cla(ax);
    
    try
        data = apoloGetLaserData(laserName);
        
        % [UPDATED] Refresh view after heavy sensor read
        apoloUpdate(); 
        
        b = size(data);
        numReadings = b(2);
        
        if numReadings == 0
            logMessage(fig, 'Laser Scan: No readings received.');
            return;
        end
        
        % Formula from EjemploUso.pdf
        t = 1:numReadings;
        t = t*(1.5*pi/numReadings); 
        
        [x, y] = pol2cart(t, data);
        
        plot(ax, x, y, 'b.'); 
        hold(ax, 'on');
        plot(ax, 0, 0, 'ro', 'MarkerFaceColor', 'r'); 
        hold(ax, 'off');
        axis(ax, 'equal');
        title(ax, 'Laser Scan (Cartesian View)');
        
        logMessage(fig, sprintf('Laser Scan: %d readings plotted.', numReadings));
        
    catch ME
        logMessage(fig, ['Error getting laser scan: ' ME.message]);
    end
end

function getLaserLandmarks(fig)
    updateNames(fig);
    laserName = fig.UserData.laserName;
    
    try
        data = apoloGetLaserLandMarks(laserName);
        
        % [UPDATED] Refresh view after reading landmarks
        apoloUpdate();
        
        logMessage(fig, '--- Laser Landmarks ---');
        if isempty(data.id)
            logMessage(fig, 'No landmarks found.');
            return;
        end
        
        for i = 1:length(data.id)
            logMessage(fig, sprintf('ID: %d, Angle: %.3f rad, Dist: %.3f m', ...
                                    data.id(i), data.angle(i), data.distance(i)));
        end
        
    catch ME
        logMessage(fig, ['Error getting landmarks: ' ME.message]);
    end
end

function getAllSonars(fig)
    updateNames(fig);
    robotName = fig.UserData.robotName;
    
    try
        readings = apoloGetAllultrasonicSensors(robotName);
        
        % [UPDATED] Refresh view after reading sonars
        apoloUpdate();
        
        logMessage(fig, '--- All Ultrasonic Sensors ---');
        % Display nicely formatted
        str = sprintf('%.2f  ', readings);
        logMessage(fig, ['Readings: ' str]);
    catch ME
        logMessage(fig, ['Error getting sonars: ' ME.message]);
    end
end