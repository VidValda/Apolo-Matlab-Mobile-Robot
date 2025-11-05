
function ApoloSensorTestGUI
%hola
%hola

    % ApoloSensorTestGUI Creates a MATLAB GUI to test Apolo simulator sensors.
    %
    % This GUI provides controls to move a mobile robot and buttons to query
    % various sensors, displaying the results in text and graphical formats.
    %
    % Assumes:
    % 1. Apolo simulator is running.
    % 2. An Apolo world is loaded (e.g., GyNRpractica1.xml from the docs).
    % 3. The Apolo MATLAB functions are in the MATLAB path.
    
    % --- Create Main Figure ---
    fig = uifigure('Name', 'Apolo Sensor Tester', 'Position', [100, 100, 800, 600]);
    
    % --- Create Main Grid Layout ---
    mainGrid = uigridlayout(fig, [3, 1]);
    mainGrid.RowHeight = {'1.2x', '3x', '2x'};
    
    % --- Store Handles and Data ---
    % We use the 'UserData' property of the figure to store shared data
    % like object names and handles to UI components.
    fig.UserData.robotName = 'Marvin'; % Default robot name from docs
    fig.UserData.laserName = 'LMS100'; % Default laser name from docs
    
    % --- Create UI Panels ---
    createControlPanel(mainGrid, fig);
    createPlotPanel(mainGrid, fig);
    createOutputPanel(mainGrid, fig);
    
    % --- Initial Log Message ---
    logMessage(fig, 'GUI Started. Make sure Apolo is running with a world loaded.');
    logMessage(fig, 'Default Robot: ''Marvin'', Default Laser: ''LMS100''');

end

% =========================================================================
% UI Panel Creation Functions
% =========================================================================

function createControlPanel(parentGrid, fig)
    % Creates the top panel with text inputs and buttons
    
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
    % Creates the middle panel for the laser scan polar plot
    
    panel = uipanel(parentGrid, 'Title', 'Laser Scan (apoloGetLaserData)');
    % Rename 'grid' to 'panelGrid' to avoid conflict with the 'grid' function
    panelGrid = uigridlayout(panel, [1, 2]);
    panelGrid.ColumnWidth = {'3x', '1x'};
    
    % --- Axes for Plot ---
    ax = uiaxes(panelGrid);
    ax.DataAspectRatio = [1 1 1];
    title(ax, 'Polar Laser Scan');
    grid(ax, 'on'); % This will now call the built-in grid function
    fig.UserData.Axes = ax; % Store axes handle
    
    % --- Plot Button ---
    % Removed 'LayoutPosition' property
    uibutton(panelGrid, 'Text', 'Get Laser Scan', ...
             'ButtonPushedFcn', @(src,evt) getLaserScan(fig));
end

function createOutputPanel(parentGrid, fig)
    % Creates the bottom panel for text logs
    
    panel = uipanel(parentGrid, 'Title', 'Sensor Data Output');
    % Rename 'grid' to 'panelGrid' to avoid conflict
    panelGrid = uigridlayout(panel, [1, 2]);
    panelGrid.ColumnWidth = {'3x', '1x'};
    
    % --- Text Area for Logs ---
    ta = uitextarea(panelGrid, 'Editable', 'off', 'Value', {''});
    fig.UserData.TextArea = ta; % Store text area handle
    
    % --- Landmark Button ---
    % Removed 'LayoutPosition' property
    uibutton(panelGrid, 'Text', 'Get Landmarks', ...
             'ButtonPushedFcn', @(src,evt) getLaserLandmarks(fig));
end

% =========================================================================
% Helper Functions
% =========================================================================

function updateNames(fig)
    % Reads the current values from the text fields and updates UserData
    fig.UserData.robotName = fig.UserData.RobotNameField.Value;
    fig.UserData.laserName = fig.UserData.LaserNameField.Value;
end

function logMessage(fig, message)
    % Prepends a message to the text log area
    % This ensures the newest message is always at the top
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
    % Called by movement buttons
    updateNames(fig);
    robotName = fig.UserData.robotName;
    
    % Define movement parameters [speed, rot_speed], time
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
        
        apoloUpdate(); % Refresh the Apolo view
        logMessage(fig, ['Move: ' direction]);
        
        % Automatically update odometry after moving
        getOdometryData(fig); 
        
    catch ME
        logMessage(fig, ['Error moving robot: ' ME.message]);
        logMessage(fig, 'Is Apolo running and is the robot name correct?');
    end
end

function getOdometryData(fig)
    % Gets and displays the robot's odometry
    updateNames(fig);
    robotName = fig.UserData.robotName;
    
    try
        pos = apoloGetOdometry(robotName); % Returns [x, y, theta]
        logMessage(fig, sprintf('Odometry: X=%.3f, Y=%.3f, Theta=%.3f', pos(1), pos(2), pos(3)));
    catch ME
        logMessage(fig, ['Error getting odometry: ' ME.message]);
    end
end

function resetOdometry(fig)
    % Resets the robot's odometry to [0, 0, 0]
    updateNames(fig);
    robotName = fig.UserData.robotName;
    
    try
        apoloResetOdometry(robotName);
        logMessage(fig, 'Odometry Reset to [0, 0, 0].');
        getOdometryData(fig); % Log the new odom value
    catch ME
        logMessage(fig, ['Error resetting odometry: ' ME.message]);
    end
end

function getLaserScan(fig)
    % Gets laser data and creates a polar plot
    updateNames(fig);
    laserName = fig.UserData.laserName;
    ax = fig.UserData.Axes;
    cla(ax); % Clear the axes
    
    try
        data = apoloGetLaserData(laserName);
        
        % Recreate the plot logic from the 'testLaserData' example PDF
        numReadings = length(data);
        
        % The PDF example uses 1.5*pi (270 degrees) for the LMS100.
        % We will assume this is centered, from -135 to +135 degrees.
        angles = linspace(-1.5*pi/2, 1.5*pi/2, numReadings);
        
        % Use polarplot on the GUI's axes
        polarplot(ax, angles, data, 'b.-');
        title(ax, 'Laser Scan');
        
        logMessage(fig, sprintf('Laser Scan: %d readings plotted.', numReadings));
        
    catch ME
        logMessage(fig, ['Error getting laser scan: ' ME.message]);
        logMessage(fig, 'Is the laser name correct?');
    end
end

function getLaserLandmarks(fig)
    % Gets and logs visible landmarks
    updateNames(fig);
    laserName = fig.UserData.laserName;
    
    try
        data = apoloGetLaserLandMarks(laserName);
        
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
    % Gets and logs all ultrasonic sensor readings from the robot
    updateNames(fig);
    robotName = fig.UserData.robotName;
    
    try
        readings = apoloGetAllultrasonicSensors(robotName);
        logMessage(fig, '--- All Ultrasonic Sensors ---');
        logMessage(fig, ['Readings: ' num2str(readings)]);
    catch ME
        logMessage(fig, ['Error getting sonars: ' ME.message]);
    end
end