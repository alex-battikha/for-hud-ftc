package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.SampleMecanumDrive;

public @TeleOp(group="MotorTest") class MotorTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        waitForStart(); // wait until user presses play button

        double motorPower[] = new double[drive.motors.size()];

        int i= 0;

        boolean a = gamepad1.a;
        boolean x = gamepad1.x;

        boolean lastA = false;
        boolean lastX = false;

        while (opModeIsActive()) {
            motorPower[i] = gamepad1.right_stick_y * -1;
            drive.motors.get(i).setPower(motorPower[i]);

            switch(i) {
                case 0:
                    telemetry.addData("Motor: leftFront", i);
                    telemetry.addData("Motor Number: ", i);
                    telemetry.addData("Motor Power: ", i);
                    break;
                case 1:
                    telemetry.addData("Motor: rightFront", i);
                    telemetry.addData("Motor Number: ", i);
                    telemetry.addData("Motor Power: ", i);
                    break;
                case 2:
                    telemetry.addData("Motor: leftRear", i);
                    telemetry.addData("Motor Number: ", i);
                    telemetry.addData("Motor Power: ", i);
                    break;
                case 3:
                    telemetry.addData("Motor: rightRear", i);
                    telemetry.addData("Motor Number: ", i);
                    telemetry.addData("Motor Power: ", i);
                    break;
                case 4:
                    telemetry.addData("Motor: Intake", i);
                    telemetry.addData("Motor Number: ", i);
                    telemetry.addData("Motor Power: ", i);
                    break;
                case 5:
                    telemetry.addData("Motor: Slides", i);
                    telemetry.addData("Motor Number: ", i);
                    telemetry.addData("Motor Power: ", i);
                    break;
                case 6:
                    telemetry.addData("Motor: Slides2", i);
                    telemetry.addData("Motor Number: ", i);
                    telemetry.addData("Motor Power: ", i);
                    break;
                case 7:
                    telemetry.addData("Motor: Turret", i);
                    telemetry.addData("Motor Number: ", i);
                    telemetry.addData("Motor Power: ", i);
                    break;
            }

            telemetry.update();

            if(a && a != lastA) {
                i = (i+1) % 8;
            }

            if (x && x != lastX) {
                i = (i + 7) % 8;
            }

            lastA = a;
            lastX = x;
        }
    }
}
