package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Config
@Autonomous(name = "AutoCSRedRight")
public class AutoCSRedRight extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d startPose = new Pose2d(0,0,0);

        MecanumDrive robot = new MecanumDrive(hardwareMap, startPose);

        Action doEverything;

        doEverything = robot.actionBuilder(robot.pose)

                .build();

        waitForStart();
        if (isStopRequested()) return;


    }
}
