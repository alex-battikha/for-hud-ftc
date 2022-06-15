package org.firstinspires.ftc.teamcode;

public class Pose2D {
    public double x;
    public double y;
    public double heading;
    public double headingOffset;
    public double radius;
    public double speed;

    public Pose2D (double x, double y, double heading, double headingOffset, double radius, double speed) { // stores robot's current location and direction
        this.x = x;
        this.y = y;
        this.heading = heading;
        this.headingOffset = headingOffset;
        this.radius = radius;
        this.speed = speed;
    }

    public Pose2D (double x, double y) {
        this(x, y, 0, 0, 0, 0);
    }

    public Pose2D (double x, double y, double heading) {
        this(x, y, heading, 0, 0, 0);
    }
    public Pose2D (double x, double y, double heading, double speed) {
        this(x, y, heading, 0, 0, speed);
    }

    public double getX() {
        return(x);
    }

    public double getY() {
        return(y);
    }

    public double getHeading() {
        return(heading);
    }

}