package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Pose2D;
import org.firstinspires.ftc.teamcode.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.Trajectory;

public @TeleOp(group="Heading PID Tuner") class HeadingPIDTuner extends LinearOpMode {
    long startTime = System.currentTimeMillis();

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        long sleepTime = 1000;

        waitForStart();

        while(opModeIsActive()) {
            drive.followTrajectory(this, new Trajectory(new Pose2D(0, 0, 0), true)
                .addLine(new Pose2D(0,0, Math.toRadians(90),0,0,0.1)).end());
            drive.followTrajectory(this, new Trajectory(new Pose2D(0, 0, 0), true)
                    .addLine(new Pose2D(0,0, Math.toRadians(-90),0,0,0.1)).end());
        }
    }
}
