package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.SampleMecanumDrive;

public @TeleOp(group="SensorTest") class SensorTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        waitForStart();

        while(opModeIsActive()){
            double forward = gamepad1.left_stick_y * -0.4;
            double left =  gamepad1.left_stick_x * 0.6;
            double turn = gamepad1.right_stick_x * 0.35;

            double p1 = forward + left + turn;
            double p2 = forward - left + turn;
            double p3 = forward + left - turn;
            double p4 = forward - left - turn;

            double powers[] = {p1, p2, p3, p4};
            drive.setMotorPowers(powers);

            telemetry.addData("Left Intake: ", drive.leftIntake.getVoltage());
            telemetry.addData("Right Intake: ", drive.rightIntake.getVoltage());
            telemetry.addData("Deposit Sensor: ", drive.depositSensor.getVoltage());
            telemetry.addData("Dist Left: ", drive.distLeft.getVoltage());
            telemetry.addData("Dist Right: ", drive.distRight.getVoltage());
            telemetry.addData("magLeft", drive.magLeft.getVoltage());
            telemetry.addData("magRight: ", drive.magRight.getVoltage());
            telemetry.addData("Flex: ", drive.flex.getVoltage());
            telemetry.update();

            drive.update();
        }
    }
}