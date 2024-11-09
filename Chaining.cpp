
void lemlib::Chassis::turnTochain(float x, float y, int timeout, turnchaining chainparams, bool forwards, float maxSpeed, bool async) {
    this->requestMotionStart();
    // were all motions cancelled?
    if (!this->motionRunning) return;
    // if the function is async, run it in a new task
    if (async) {
        pros::Task task([&]() { turnTochain(x, y, timeout, chainparams, forwards, maxSpeed, false); });
        this->endMotion();
        pros::delay(10); // delay to give the task time to start
        return; 
    }
//Get the intial error for the gain scheduler 
    Pose startpose = getPose();
    const float adjustedRobotTheta = forwards ? startpose.theta : startpose.theta + 180.0F;

    float targetTheta = fmod(radToDeg(M_PI_2 - atan2((y - startpose.y), (x - startpose.x))), 360);
    float setpoint = angleError(targetTheta, adjustedRobotTheta, false);

//use interpolated timeout if not provided by the user
    if (timeout == 0) {
        timeout = angularSettings.lerpit(setpoint, 180.0F);
    }

    float deltaX, deltaY;
    float motorPower;
    float prevMotorPower = 0;
    float startTheta = getPose().theta;
   

    std::uint8_t compState = pros::competition::get_status();
    distTravelled = 0;
    Timer timer(timeout);
    angularLargeExit.reset();
    angularSmallExit.reset();
    angularPID.reset();


    angularPID.prevcrossing = setpoint;
    float deltaTheta = setpoint;
    
    // main loop
    while (!timer.isDone() && !angularLargeExit.getExit() && !angularSmallExit.getExit() && !angularPID.passing(deltaTheta, chainparams.exitrange) && this->motionRunning) {
        // update variables
        Pose pose = getPose();
        pose.theta = (forwards) ? fmod(pose.theta, 360) : fmod(pose.theta - 180, 360);

        // update completion vars
        distTravelled = fabs(angleError(pose.theta, startTheta));

        deltaX = x - pose.x;
        deltaY = y - pose.y;
        targetTheta = fmod(radToDeg(M_PI_2 - atan2(deltaY, deltaX)), 360);

        // calculate deltaTheta
        deltaTheta = angleError(targetTheta, pose.theta, false);

        // calculate the speed
        motorPower = angularPID.update(deltaTheta, setpoint);
        angularLargeExit.update(deltaTheta);
        angularSmallExit.update(deltaTheta);

        // cap the speed
        if (motorPower > maxSpeed) motorPower = maxSpeed;
        else if (motorPower < -maxSpeed) motorPower = -maxSpeed;
        if (fabs(deltaTheta) > 20) motorPower = slew(motorPower, prevMotorPower, angularSettings.slew);
        prevMotorPower = 0;

        if(setpoint > 0) {motorPower = fmax(motorPower, chainparams.minspeed);}
        if (setpoint < 0) {motorPower = fmin(motorPower, -1 * chainparams.minspeed);}

        // move the drivetrain
        drivetrain.leftMotors->move(motorPower);
        drivetrain.rightMotors->move(-motorPower);

        pros::delay(10);
    }

    // stop the drivetrain
   // drivetrain.leftMotors->move(0);
   // drivetrain.rightMotors->move(0);
    // set distTraveled to -1 to indicate that the function has finished
    distTravelled = -1;   
    this->endMotion();
}
