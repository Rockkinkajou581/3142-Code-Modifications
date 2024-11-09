
void lemlib::Chassis::moveToPointghost(float x, float y, float g_x, float g_y, int timeout, MoveToPointParams params, bool async) {
    this->requestMotionStart();
    // were all motions cancelled?
    if (!this->motionRunning) return;
    // if the function is async, run it in a new task             
    if (async) {
        pros::Task task([&]() { moveToPointghost(x, y, g_x, g_y, timeout, params, false); });
        this->endMotion();
        pros::delay(10); // delay to give the task time to start
        return;
    }

    // reset PIDs and exit conditions
    lateralPID.reset();
    lateralLargeExit.reset();
    lateralSmallExit.reset();
    angularPID.reset();
    Pose lastPose = getPose();
    Pose target(x, y);
    Pose ghost(g_x, g_y);
    target.theta = lastPose.angle(target);
    ghost.theta = lastPose.angle(ghost);

    const Pose pose = getPose(true, true);
    const float adjustedRobotTheta = params.forwards ? pose.theta : pose.theta + M_PI;

    const float angularError = angleError(adjustedRobotTheta, pose.angle(target));

    float initialdistTarget = pose.distance(target) * cos(angularError);
    float intialangleerror = radToDeg(angularError);

    if (timeout == 0) {
        timeout = lateralSettings.lerpit(initialdistTarget, 72.0F);
    }

    // initialize vars used between iterations
    distTravelled = 0;
    Timer timer(timeout);
    bool close = false;
    float prevLateralOut = 0; // previous lateral power
    float prevAngularOut = 0; // previous angular power
    const int compState = pros::competition::get_status();
    std::optional<bool> prevSide = std::nullopt;
    bool g_prevSide = false;


    // calculate target pose in standard form
    

    // main loop
    while (!timer.isDone() && ((!lateralSmallExit.getExit() && !lateralLargeExit.getExit()) || !close) &&
           this->motionRunning) {
        // update position
        const Pose pose = getPose(true, true);

        // update distance travelled
        distTravelled += pose.distance(lastPose);
        lastPose = pose;

        // calculate distance to the target point
        const float distTarget = pose.distance(target);
        const float distGhost = pose.distance(ghost);
        // check if the robot is close enough to the target to start settling
        if (distTarget < 7.5 && close == false) {
            close = true;
            params.lateralmaxSpeed = fmax(fabs(prevLateralOut), 60);
        }

        const bool side =
            (pose.y - target.y) * -sin(target.theta) <= (pose.x - target.x) * cos(target.theta) + params.earlyExitRange;
        if (prevSide == std::nullopt) prevSide = side;
        const bool sameSide = side == prevSide; 
        // exit if close
        if (!sameSide && close && params.minSpeed != 0) break;
        prevSide = side;

         const bool ghostside =
            (pose.y - ghost.y) * -sin(ghost.theta) <= (pose.x - ghost.x) * cos(ghost.theta);
        const bool g_sameSide = ghostside == g_prevSide;
        // exit if cross
        if (!g_sameSide) {
            const float g_adjustedRobotTheta = params.forwards ? pose.theta : pose.theta + M_PI;
            intialangleerror = radToDeg(angleError(adjustedRobotTheta, pose.angle(ghost)));
            ghost = target;
        }
        g_prevSide = ghostside;


        // calculate error
        const float adjustedRobotTheta = params.forwards ? pose.theta : pose.theta + M_PI;
        const float angularError = angleError(adjustedRobotTheta, pose.angle(ghost));
        float lateralError = pose.distance(target) * cos(angleError(pose.theta, pose.angle(target)));

        // update exit conditions
        lateralSmallExit.update(lateralError);
        lateralLargeExit.update(lateralError);

        // get output from PIDs
        float lateralOut = lateralPID.update(lateralError, initialdistTarget);
        float angularOut = angularPID.update(radToDeg(angularError), intialangleerror);
        if (close) angularOut = 0;
        // apply restrictions on angular speed
        angularOut = std::clamp(angularOut, -params.angularmaxSpeed, params.angularmaxSpeed);
        angularOut = slew(angularOut, prevAngularOut, angularSettings.slew);

        // apply restrictions on lateral speed
        lateralOut = std::clamp(lateralOut, -params.lateralmaxSpeed, params.lateralmaxSpeed);
        // constrain lateral output by max accel
        // but not for decelerating, since that would interfere with settling
        if (!close) lateralOut = slew(lateralOut, prevLateralOut, lateralSettings.slew);

        // prevent moving in the wrong direction
        if (params.forwards && !close) lateralOut = std::fmax(lateralOut, 0);
        else if (!params.forwards && !close) lateralOut = std::fmin(lateralOut, 0);

        // constrain lateral output by the minimum speed
        if (params.forwards && lateralOut < fabs(params.minSpeed) && lateralOut > 0) lateralOut = fabs(params.minSpeed);
        if (!params.forwards && -lateralOut < fabs(params.minSpeed) && lateralOut < 0)
            lateralOut = -fabs(params.minSpeed);

        // update previous output
        prevAngularOut = angularOut;
        prevLateralOut = lateralOut;

        // ratio the speeds to respect the max speed
        float leftPower = lateralOut + angularOut;
        float rightPower = lateralOut - angularOut;
        const float ratio = std::max(std::fabs(leftPower), std::fabs(rightPower)) / params.maxSpeed;
        if (ratio > 1) {
            leftPower /= ratio;
            rightPower /= ratio;
        }

        // move the drivetrain
        drivetrain.leftMotors->move(leftPower);
        drivetrain.rightMotors->move(rightPower);

        // delay to save resources
        pros::delay(10);
    }

    // stop the drivetrain
    drivetrain.leftMotors->move(0);
    drivetrain.rightMotors->move(0);
    // set distTraveled to -1 to indicate that the function has finished
    distTravelled = -1;
    this->endMotion();
}
