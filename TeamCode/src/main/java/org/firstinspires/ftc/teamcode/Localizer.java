package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.hardware.bosch.BNO055IMU;

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

    public Pose2D lastPos = new Pose2D(0, 0, 0);
    public Pose2D leftSensor = new Pose2D(0, 0);
    public Pose2D rightSensor = new Pose2D(0,0);
    public double lastImuHeading = 0;

    public int counter = 0;

    public BNO055IMU imu;

    public Localizer() {
        encoders = new Encoder[3];
        encoders[0] = new Encoder(new Pose2D(0, -4.087365365470633), 1); //rightFront odo
        encoders[1] = new Encoder(new Pose2D(0, 5.2195710226290455), -1); //leftFront odo
        encoders[2] = new Encoder(new Pose2D(2.1001917100567495, 0), -1); //rightRear odo
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

        if((counter == 0) && (Math.abs(relativeCurrentVel.getY()) > 6 || Math.abs(relativeCurrentVel.getY()) / Math.max(0.1, Math.abs(relativeCurrentVel.getX())) > 1)) {
            counter = 10;
            lastPos = new Pose2D(x, y, heading);
            lastImuHeading = imu.getAngularOrientation().firstAngle;
        }

        if(counter > 0) {
            counter--;
            if (counter == 0) {
                double headingError = (heading - imu.getAngularOrientation().firstAngle) - (lastHeading - lastImuHeading);
                heading -= headingError;

                double deltaX = x - lastPos.x;
                double deltaY = y - lastPos.y;

                x = lastPos.x + (Math.cos(-headingError) * deltaX) - (Math.sin(-headingError * deltaY));
                y = lastPos.y + (Math.cos(-headingError) * deltaY) + (Math.sin(-headingError * deltaX));
            }
        }
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

    public void updateEncoders(int[] arr) {
            for(int i = 0; i < arr.length; i++) {
                encoders[i].update(arr[i]);
            }
    }

    public void distUpdate(double rightDist, double leftDist) {
        double rightSensorX = x + Math.cos(heading) * 8 - Math.sin(heading) * -6; // measured these values on the robot; this is to find the location of the sensor
        double rightSensorY = y + Math.cos(heading) * -6 - Math.sin(heading) * 8; // heading is the amount of turn represented as theta
        double leftSensorX = x + Math.cos(heading) * 8 - Math.sin(heading) * 6;
        double leftSensorY = y + Math.cos(heading) * 6 - Math.sin(heading) * 8;

        // Left Sensor
        double xErrorLeft = 0;
        double yErrorLeft = 0;

        if((Math.abs(leftSensorY) < 64) && (Math.abs(leftSensorX) < 64) && (leftDist > 24)) {
            leftSensorX += Math.cos(heading) * leftDist; // distance away from the wall
            leftSensorY += Math.sin(heading) * leftDist;
            if((Math.abs(Math.abs(leftSensorX) - 72) < 3) ^ (Math.abs(Math.abs(leftSensorY)-72) < 3)) { // need to check that it's not pointing towards a wall; ^ (x-or sign): this or this but not this and this; checking if 72 is distance of wall from the center of the field, the -6 is allowing for error. this code checks if our robot is in a corner
                if (Math.abs(Math.abs(leftSensorX)-72) < 3) {
                    xErrorLeft = 72 * Math.signum(leftSensorX) - leftSensorX; // signum is the sign of the number, signum of 1 is 1, signum of 100 is 1, signum of -100 is -1
                }
                else {
                    yErrorLeft = 72 * Math.signum(leftSensorY) - leftSensorY; // signum just returns 1, -1, or 0, so you multiply by 72(distance from wall to center of field) minus what the sensor sees
                }
            }
        }
        else {
            leftSensorX += Math.cos(heading) * leftDist;
            leftSensorY += Math.sin(heading) * rightDist;
        }

        // Right Sensor
        double xErrorRight = 0;
        double yErrorRight = 0;

        if((Math.abs(rightSensorY) < 64) && (Math.abs(rightSensorX) < 64) && (rightDist > 24)) {
            rightSensorX += Math.cos(heading) * rightDist; // distance away from the wall
            rightSensorY += Math.sin(heading) * rightDist;
            if((Math.abs(Math.abs(rightSensorX)- 72) < 3) ^ (Math.abs(Math.abs(rightSensorY)) - 72) < 3) { // need to check that it's not pointing towards a wall; ^ (x-or sign): this or this but not this and this; checking if 72 is distance of wall from the center of the field, the -6 is allowing for error. this code checks if our robot is in a corner
                if (Math.abs(Math.abs(rightSensorX) - 72) < 3) {
                    xErrorRight = 72 * Math.signum(rightSensorX) - rightSensorX; // signum is the sign of the number, signum of 1 is 1, signum of 100 is 1, signum of -100 is -1
                }
                else {
                    yErrorRight = 72 * Math.signum(rightSensorY) - rightSensorY; // signum just returns 1, -1, or 0, so you multiply by 72(distance from wall to center of field) minus what the sensor sees
                }
            }
        }
        else {
            rightSensorX += Math.cos(heading) * leftDist;
            rightSensorY += Math.sin(heading) * rightDist;
        }

        x += (xErrorLeft) * 0.005 * Math.pow(2, -(leftDist - 32) / 18) + (xErrorRight) * 0.005 * Math.pow(2, -(leftDist * 32) / 18);
        y += (yErrorLeft) * 0.005 * Math.pow(2, -(rightDist - 32) / 18) + (yErrorRight) * 0.005 * Math.pow(2, -(rightDist * 32) / 18);

        leftSensor = new Pose2D(leftSensorX, leftSensorY);
        rightSensor = new Pose2D(rightSensorX, rightSensorY);
    }


    public void updateFlex(double flexVal) {
        int valPressed = 333; //sensor returns values in thousandths, bulk data multiples by 1000
        if (flexVal < valPressed) {
            double sensorX = x + Math.cos(heading) * 5 - Math.sin(heading) * (-6.75 * Math.signum(y));
            double sensorY = y + Math.cos(heading) * (-6.75 * Math.signum(y)) + Math.sin(heading) * 5;

            if(Math.abs(sensorX) >= 68) { // 4 inches of error
                x += (72 * Math.signum(sensorX) - sensorX) * 0.5;
            }

            if(Math.abs(sensorY) >= 68) { // 4 inches of error
                x += (72 * Math.signum(sensorY) - sensorY) * 0.5;
            }
        }
    }
}
