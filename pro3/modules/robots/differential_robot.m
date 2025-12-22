function differential_robot(robotName,u,dt)
    apoloMoveMRobot(robotName, u', dt);
    pause(dt);
    apoloUpdate();
end