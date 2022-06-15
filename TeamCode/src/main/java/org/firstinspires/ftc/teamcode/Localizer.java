package org.firstinspires.ftc.teamcode;

import java.lang.reflect.Array;
import java.util.ArrayList;

public class Localizer {
    public Encoder[] encoders;
    public long lastTime = System.nanoTime();
    public double x = 0, y = 0, heading = 0, startingHeading = 0;

    public Pose2D currentPos = new Pose2D(0, 0, 0);
    public Pose2D currentVel = new Pose2D(0, 0, 0);
    public Pose2D relativeCurrentVel = new Pose2D(0, 0, 0);
    public Pose2D currentPowerVector = new Pose2D(0, 0, 0); // lets us know what vector the motors are moving in (1, 0) would be straightforward

    public ArrayList<Pose2D> poseHistory = new ArrayList<>();
    public ArrayList<Pose2D> relativeHistory = new ArrayList<>(); // very long array list that stores the values were the robot has been
    public ArrayList<Double> loopTimes = new ArrayList<>();

    public Localizer() {
        encoders = new Encoder[3];
        encoders[0] = new Encoder(new Pose2D(0, -5.2195710226290455), 1); //rightFront odo
        encoders[1] = new Encoder(new Pose2D(0, -4.087365365470633), -1); //leftFront odo
        encoders[2] = new Encoder(new Pose2D(2.1001917100567495, 0), -1); //rightRear odo

        Pose2D pose2D = new Pose2D(0,0,0);
    }

    public void update() {
        long currentTime = System.nanoTime();
        double loopTime = (currentTime - lastTime) / 1e9;
        lastTime = currentTime;

        double deltaRight = encoders[0].getDelta();
        double deltaLeft = encoders[1].getDelta();
        double deltaBack = encoders[2].getDelta();

        double relativeDeltaX = ((deltaRight * encoders[1].y) - (deltaLeft*encoders[0].y)) / (encoders[1].y - encoders[0].y);
        double deltaHeading = (deltaRight - deltaLeft) / (encoders[1].y - encoders[0].y);
        double relativeDeltaY = (deltaBack) - (encoders[2].x * deltaHeading);
        heading += deltaHeading;

        relativeHistory.add(0, new Pose2D(relativeDeltaX, relativeDeltaY, deltaHeading));
        loopTimes.add(0, loopTime);

        if(deltaHeading != 0) {
            double r1 = relativeDeltaX / deltaHeading;
            double r2 = relativeDeltaY / deltaHeading;
            relativeDeltaX = Math.sin(deltaHeading) * r1 + (1.0 - Math.cos(deltaHeading)) * r2;
            relativeDeltaY = Math.sin(deltaHeading) * r2 + (1.0 - Math.cos(deltaHeading)) * r1;
        }

        double lastHeading = heading - deltaHeading;
        x += relativeDeltaX * Math.cos(lastHeading) - relativeDeltaY * Math.sin(lastHeading);
        y += relativeDeltaY * Math.cos(lastHeading) + relativeDeltaX * Math.sin(lastHeading);

        currentPos = new Pose2D(x, y, heading);
        poseHistory.add(0, currentPos);
        updateVelocities();
    }

    public void updateVelocities() {
        double targetVelTimeEstimate = 0.2; //target velocity estimate over a certain period of time
        double actualVelTime = 0;
        double relDeltaXTotal = 0;
        double relDeltaYTotal = 0;
        double totalTime = 0;
        int lastIndex = 0;

        for (int i = 0; i< loopTimes.size(); i++) {
            totalTime += loopTimes.get(i);
            if (totalTime <= targetVelTimeEstimate) { //as long as your current time is less than your estimation period, then you will continually update your relative value's x and y
                actualVelTime += loopTimes.get(i);
                relDeltaXTotal += relativeHistory.get(i).getX();
                relDeltaYTotal += relativeHistory.get(i).getY();
                lastIndex = i;
            }
        }

        currentVel = new Pose2D(poseHistory.get(0).getX() - poseHistory.get(lastIndex).getX()/actualVelTime, //currentVel is global
                poseHistory.get(0).getY() - poseHistory.get(lastIndex).getY()/actualVelTime,
                poseHistory.get(0).getHeading() - poseHistory.get(lastIndex).getHeading()/actualVelTime); // delta between the last value in array and the very first value for x, y, and heading

        relativeCurrentVel = new Pose2D(relDeltaXTotal / actualVelTime, //relativeCurrentVel is relative to the robot's direction
                relDeltaYTotal / actualVelTime,
                poseHistory.get(0).getHeading() - poseHistory.get(lastIndex).getHeading()/actualVelTime);

        while(lastIndex + 1 < loopTimes.size()) { //shrinks array list
            loopTimes.remove(loopTimes.size()-1);
            poseHistory.remove(poseHistory.size()-1);
            relativeHistory.remove(relativeHistory.size()-1);
        }

    }
}
