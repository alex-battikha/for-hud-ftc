package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.SampleMecanumDrive;

public @TeleOp(group="EncoderTest") class EncoderTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        waitForStart();

        while(opModeIsActive()){
//            drive.drive(this);

            telemetry.addData("Left Front Motor: ", drive.motors.get(0).getCurrentPosition());
            telemetry.addData("Left Rear Motor: ", drive.motors.get(1).getCurrentPosition());
            telemetry.addData("Right Rear Motor: ", drive.motors.get(2).getCurrentPosition());
            telemetry.addData("Right Front Motor: ", drive.motors.get(3).getCurrentPosition());
            telemetry.update();
        }
    }
}
