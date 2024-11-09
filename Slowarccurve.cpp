//theta in radians
void lemlib::Chassis::curveslowly(float r, float theta, int timeout, bool forwards, float maxSpeed, bool async) {
    this->requestMotionStart();
    // were all motions cancelled?
    if (!this->motionRunning) return;
    // if the function is async, run it in a new task
    if (async) {
        pros::Task task([&]() {curveslowly(r, theta, timeout, forwards, maxSpeed, false); });
        this->endMotion();
        pros::delay(10); // delay to give the task time to start
        return;
    }
    theta = theta * M_PI/180.0;
    float targetright = maxSpeed * (1 - r);
    float targetleft = maxSpeed * (1 + r);
    float angularerror {};
    float localY = 2 * sin(theta / 2) * (r);

    Pose initialpose(getPose());
    Pose targetPose(initialpose.x + localY * sin(initialpose.theta + theta/2), initialpose.y + localY * cos(initialpose.theta + theta/2), initialpose.theta + theta);

    Timer timer(timeout);
    // main loop
    while (!timer.isDone() && (crossingexit(getPose(), targetPose, initialpose, theta) > 0) && !this->motionRunning) {
        // update variables

        float deltaright = drivetrain.rightMotors->get_positions()[0] - slowcurve.lastright;
        float deltaleft = drivetrain.leftMotors->get_positions()[0] - slowcurve.lastleft;
        slowcurve.lastright = drivetrain.rightMotors->get_positions()[0];
        slowcurve.lastleft = drivetrain.leftMotors->get_positions()[0];

        angularerror = ((r + drivetrain.trackWidth / 2) / (r - drivetrain.trackWidth / 2)) - (deltaright/deltaleft);

        // move the drivetrain
        drivetrain.leftMotors->move(targetleft + slowcurve.update(angularerror));
        drivetrain.rightMotors->move(targetright - slowcurve.update(angularerror));
        pros::delay(10);
    };

    // stop the drivetrain
    drivetrain.leftMotors->move(0);
    drivetrain.rightMotors->move(0);
    // set distTraveled to -1 to indicate that the function has finished
    distTravelled = -1;
    this->endMotion();
}

float lemlib::ControllerSettings::lerpit (float input, float maxinput) {
    float m = (maxtimeout - mintimeout)/(maxinput);
    return (fabs(input))*m + mintimeout;
}
