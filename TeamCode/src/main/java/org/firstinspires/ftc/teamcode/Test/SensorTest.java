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
            drive.drive(gamepad1);

            telemetry.addData("Left Intake: ", drive.leftIntake.getVoltage());
            telemetry.addData("Right Intake: ", drive.rightIntake.getVoltage());
            telemetry.addData("Deposit Sensor: ", drive.depositSensor.getVoltage());
            telemetry.addData("Dist Left: ", drive.distLeft.getVoltage());
            telemetry.addData("Dist Right: ", drive.distRight.getVoltage());
            telemetry.addData("magLeft", drive.magLeft.getVoltage());
            telemetry.addData("magRight: ", drive.magRight.getVoltage());
            telemetry.addData("Flex: ", drive.flex.getVoltage());
            telemetry.update();
        }
    }
}
