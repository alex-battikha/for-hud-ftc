package org.firstinspires.ftc.teamcode;

public class Encoder {
    public double ticksToInches = 0;
    public int lastVal = 0;
    public int currentVal = 0;
    public double scaleFactor = 0;
    public double x = 0;
    public double y = 0;

    public Encoder(Pose2D point, double scaleFactor) {
        double ticksPerRotation = 8192;
        double wheelRadius = 0.6889764;
        double ticksToInches = (wheelRadius * Math.PI * 2.0) / ticksPerRotation;
        x = point.getX();
        y = point.getY();
        double currentVal = 0;
        double lastVal = currentVal;
        this.scaleFactor = scaleFactor;
    }

    public void update(int currentPos) {
        lastVal = currentVal;
        currentVal = currentPos;
    }

    public double getDelta() {
        return (double) (currentVal - lastVal) * ticksToInches * scaleFactor;
    }

    public double getCurrentDistance() {
        return (double) currentVal * ticksToInches * scaleFactor;
    }
}
