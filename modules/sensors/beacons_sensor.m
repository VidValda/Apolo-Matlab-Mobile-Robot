function [z,beacons_xy] = beacons_sensor(laserName, params)
    
    beacons_reading = apoloGetLaserLandMarks(laserName);
    beacons_xy = params.beacons_pos(beacons_reading.id, :);

    bias_estimado = params.correction_factor ./ beacons_reading.distance;
    angle_beacons = mod(beacons_reading.angle + pi, 2*pi) - pi;
    temp_matrix = [beacons_reading.distance-bias_estimado; angle_beacons];
    z = temp_matrix(:);

end