laserName = 'LMS100';
disp(['Attempting to get LandMark data from: ' laserName]);
try
    beaconData = apoloGetLaserLandMarks(laserName);
    
    if isempty(beaconData.id)
        disp('No beacons (LandMarks) were detected.');
        return;
    end
    
    disp('Beacon data retrieved successfully:');
    disp(beaconData);
    
    figure;
    polarplot(beaconData.angle, beaconData.distance, 'ro', 'MarkerFaceColor', 'r');
    
    title(['Detected Beacons (LandMarks) from ' laserName]);
        for i = 1:length(beaconData.id)
        text(beaconData.angle(i), beaconData.distance(i) + 0.2, ...
             ['ID: ' num2str(beaconData.id(i))], ...
             'HorizontalAlignment', 'center');
    end
    
    disp('Beacon plot created.');
    
catch ME
    disp(' ');
    disp('Error: Could not retrieve beacon data.');
    disp(ME.message);
    disp('Please ensure Apolo is running and the laser name is correct.');
end