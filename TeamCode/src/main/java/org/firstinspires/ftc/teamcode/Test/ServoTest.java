package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.SampleMecanumDrive;

public @TeleOp(group="ServoTest") class ServoTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        waitForStart();

        int i = 0;

        boolean leftBumper = false;
        boolean rightBumper = false;
        boolean y = false;
        boolean a = false;
        boolean x = false;
        boolean b = false;

        boolean lastLeftBumper = false;
        boolean lastRightBumper = false;
        boolean lastY = false;
        boolean lastA = false;
        boolean lastX = false;
        boolean lastB = false;

        double servoPos = 0.5;

        while(opModeIsActive()) {

            leftBumper = gamepad1.left_bumper;
            rightBumper = gamepad1.right_bumper;
            y = gamepad1.y;
            a = gamepad1.a;
            x = gamepad1.x;
            b = gamepad1.b;

            lastLeftBumper = leftBumper;
            lastRightBumper = rightBumper;
            lastY = y;
            lastA = a;
            lastX = x;
            lastB = b;

            while(leftBumper && leftBumper != lastLeftBumper) {
                i = (i + drive.servos.size()-1) % (drive.servos.size());
                servoPos = 0.5;
            }

            while(rightBumper && rightBumper != lastRightBumper) {
                i = (i+1) % (drive.servos.size());
                servoPos = 0.5;
            }

            while(y && y != lastY) {
                if (servoPos >= 0.0 && servoPos <= 1) {
                    servoPos -= 0.001;
                }
            }

            while(a && a != lastA) {
                if (servoPos >= 0.0 && servoPos <= 1) {
                    servoPos += 0.001;
                }
            }

            while(x && x != lastX) {
                if (servoPos >= 0.0 && servoPos <= 1) {
                    servoPos += 0.0001;
                }
            }

            while(b && b != lastB) {
                if (servoPos >= 0.0 && servoPos <= 1) {
                    servoPos += 0.0001;
                }
            }

            drive.servos.get(i).setPosition(servoPos);

            telemetry.addData("Servo Number: ", i);
            telemetry.addData("Servo Position: ", servoPos);
            telemetry.update();

        }
    }
}
