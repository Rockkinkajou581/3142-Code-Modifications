
void lemlib::Chassis::fullpower(float x, float y, float maxtimeout, float maxspeed, fullpowerparams params, bool async) {
    this->requestMotionStart();
    // were all motions cancelled?
    if (!this->motionRunning) return;
    // if the function is async, run it in a new task
    if (async) {
        pros::Task task([&]() {fullpower(x, y, maxtimeout, maxspeed, params, false);});
        this->endMotion();
        pros::delay(10); // delay to give the task time to start
        return; 
    }
//Get the intial error for the gain scheduler 
    std::uint8_t compState = pros::competition::get_status();
    distTravelled = 0;
    Timer timer(maxtimeout);
    angularSmallExit.reset();
    angularPID.reset();

    lateralLargeExit.reset();
    lateralSmallExit.reset();
    angularPID.prevcrossing = 0;

    Pose pose = getPose();
    Pose target (x, y);
    Pose lastPose (pose);

    float deltaTheta, motorPower, distTarget;
    
    float adjustedRobotTheta = (maxspeed > 0) ? pose.theta : pose.theta + 180.0F;    
    const float targetTheta = fmod(radToDeg(M_PI_2 - atan2((y - pose.y), (x - pose.x))), 360);
    const float setpoint = angleError(targetTheta, adjustedRobotTheta, false);


    // main loop
    while (!timer.isDone() && !lateralLargeExit.getExit() && !lateralSmallExit.getExit() && !angularPID.exitderivative(params.exitvelocity, params.exitrange, distTarget) && this->motionRunning) {
        // update variables


        pose = getPose();
        adjustedRobotTheta = (maxspeed > 0) ? pose.theta : pose.theta + 180.0F;    
       
        deltaTheta = angleError(targetTheta, adjustedRobotTheta, false);

        distTarget = pose.distance(target);
        distTravelled += pose.distance(lastPose);
        lastPose = pose;

        // calculate the speed
        motorPower = angularPID.update(deltaTheta, setpoint);
        motorPower = std::clamp(motorPower, -params.maxangularspeed, params.maxangularspeed);
        lateralLargeExit.update(distTarget);
        lateralSmallExit.update(distTarget);
    
        if(angularSmallExit.update(deltaTheta)) {
            motorPower = 0;
        }

        float leftPower = maxspeed + motorPower;
        float rightPower = maxspeed - motorPower;
        const float ratio = std::max(std::fabs(leftPower), std::fabs(rightPower)) / maxspeed;
        if (ratio > 1) {
            leftPower /= ratio;
            rightPower /= ratio;
        }

        // move the drivetrain
        drivetrain.leftMotors->move(leftPower);
        drivetrain.rightMotors->move(rightPower);

        pros::delay(10);
    }

    // stop the drivetrain
    drivetrain.leftMotors->move(0);
    drivetrain.rightMotors->move(0);
    // set distTraveled to -1 to indicate that the function has finished
    distTravelled = -1;
    this->endMotion();
}
