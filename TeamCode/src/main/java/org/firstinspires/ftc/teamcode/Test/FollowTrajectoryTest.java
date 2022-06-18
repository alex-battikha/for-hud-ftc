package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Pose2D;
import org.firstinspires.ftc.teamcode.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.Trajectory;

public @TeleOp(group="Follow Trajectory Test") class FollowTrajectoryTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        waitForStart();

        /**
         *              +X
         *            +Y 0 -Y
         *              -X
         */
        drive.followTrajectory(this,
            new Trajectory(new Pose2D(0, 0,0,0.3), true)
                    .addLine(new Pose2D(30,-30, Math.toDegrees(70), 0, 20, 0.5)) // top right corner
                    .addLine(new Pose2D(30,30, Math.toDegrees(-70), 0, 20, 0.5)) // top left corner
                    .addLine(new Pose2D(-30,30, Math.toDegrees(70), 0, 20, 0.5)) //
                    .addLine(new Pose2D(-30,-30, Math.toDegrees(-70), 0, 20, 0.5))
                    .addLine(new Pose2D(0, 0, Math.toDegrees(0), 0,20,0.5))
                    .end()
        );
    }
}
