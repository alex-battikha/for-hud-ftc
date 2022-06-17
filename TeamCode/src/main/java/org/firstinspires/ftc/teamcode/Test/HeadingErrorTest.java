package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.SampleMecanumDrive;

@TeleOp(group="Heading Error Test")
public class HeadingErrorTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        double ticksPerRotation = 8192.0;
        double wheelRadius = 0.6889764; //wheel radius of odo
        double ticksToInches = (wheelRadius * Math.PI * 2.0) / ticksPerRotation;
        waitForStart();

        double lastAngle = drive.imu.getAngularOrientation().firstAngle; // there are 3 axis of the IMU; firstAngle & secondAngle are the two axis we want
        double currentCumAngle = 0; //cumulative sum of the change in angles

        while(opModeIsActive()) {
            double turn = gamepad1.right_stick_x * 0.35;
            //drive.drive(this);

            double currentAngle = drive.imu.getAngularOrientation().firstAngle;
            double deltaAngle = currentAngle - lastAngle;

            lastAngle = currentAngle;

            while(deltaAngle >= Math.PI) { // when IMU goes past 180 it becomes -180 and start counting up to 0
                deltaAngle -= (2 * Math.PI);
            }
            while(deltaAngle <= Math.PI) {
                deltaAngle += (2 * Math.PI);
            }
            currentCumAngle += deltaAngle;

            double right = drive.rf.getCurrentPosition() * ticksToInches * drive.localizer.encoders[0].scaleFactor;
            double left = drive.lf.getCurrentPosition() * ticksToInches * drive.localizer.encoders[1].scaleFactor;
            double back = drive.rr.getCurrentPosition() * ticksToInches * drive.localizer.encoders[2].scaleFactor;

            telemetry.addData("Heading: ", Math.toDegrees(drive.localizer.heading));
            telemetry.addData("Current Cumulative Angle: ", Math.toDegrees(currentCumAngle));

            drive.update();
            telemetry.update();
        }
    }
}
