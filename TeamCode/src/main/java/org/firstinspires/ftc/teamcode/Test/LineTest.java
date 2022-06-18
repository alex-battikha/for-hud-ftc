package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Pose2D;
import org.firstinspires.ftc.teamcode.SampleMecanumDrive;

public @TeleOp(group="LineTest") class LineTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        waitForStart();
        drive.driveToPoint(this, new Pose2D(48, 24, 45));
    }
}
