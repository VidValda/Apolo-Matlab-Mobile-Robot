laserName = 'LMS100';
disp(['Attempting to get LandMark data from: ' laserName]);
try
    beaconData = apoloGetLaserLandMarks(laserName);
    
    % 3. Check if any beacons were detected.
    if isempty(beaconData.id)
        disp('No beacons (LandMarks) were detected.');
        return;
    end
    
    % 4. Display the retrieved data in the console.
    disp('Beacon data retrieved successfully:');
    disp(beaconData);
    
    % 5. Create a new figure and plot the beacons.
    % We use a polar plot, which is ideal for angle and distance data. 
    % 'ro' plots each beacon as a red 'o'.
    figure;
    polarplot(beaconData.angle, beaconData.distance, 'ro', 'MarkerFaceColor', 'r');
    
    % 6. Add labels and a title for clarity.
    title(['Detected Beacons (LandMarks) from ' laserName]);
    
    % 7. Add text labels for each beacon's ID.
    % This adds context to the plot.
    for i = 1:length(beaconData.id)
        % Add a small offset to the distance for label readability
        text(beaconData.angle(i), beaconData.distance(i) + 0.2, ...
             ['ID: ' num2str(beaconData.id(i))], ...
             'HorizontalAlignment', 'center');
    end
    
    disp('Beacon plot created.');
    
catch ME
    % Error handling in case Apolo isn't running or the laser name is wrong.
    disp(' ');
    disp('Error: Could not retrieve beacon data.');
    disp(ME.message);
    disp('Please ensure Apolo is running and the laser name is correct.');
end