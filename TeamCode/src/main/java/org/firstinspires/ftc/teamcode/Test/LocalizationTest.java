package org.firstinspires.ftc.teamcode.Test;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Localizer;
import org.firstinspires.ftc.teamcode.SampleMecanumDrive;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.opMode;

@TeleOp(group="LocalizationTest")
public class LocalizationTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Log.e("ALEX", "alex");

        waitForStart();

        while(opModeIsActive()) {
            double forward = gamepad1.left_stick_y * -0.4;
            double left =  gamepad1.left_stick_x * 0.6;
            double turn = gamepad1.right_stick_x * 0.35;

            double p1 = forward + left + turn;
            double p2 = forward - left + turn;
            double p3 = forward + left - turn;
            double p4 = forward - left - turn;

            double powers[] = {p1, p2, p3, p4};
            drive.setMotorPowers(powers);

            telemetry.addData("X Position: ", drive.localizer.x);
            telemetry.addData("Y Position: ", drive.localizer.y);
            telemetry.addData("Heading Position: ", drive.localizer.heading);
            telemetry.update();

            drive.update();
        }
    }
}
