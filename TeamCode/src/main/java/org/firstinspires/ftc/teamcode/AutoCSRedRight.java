package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.*;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.lang.Math;

@Config
@Autonomous(name = "AutoCSRedRight")
public class AutoCSRedRight extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d startPose = new Pose2d(62.00, 12.00, Math.toRadians(180.00));

        MecanumDrive robot = new MecanumDrive(hardwareMap, startPose);
        ConnectedDevices util = new ConnectedDevices(hardwareMap, robot);

        Action left = robot.actionBuilder(startPose)
                //Move to team prop
                .strafeToLinearHeading(new Vector2d(25.00, 20.00), Math.toRadians(269.99))
                //open left claw and then raise arm
                .afterTime(0,
                    new SequentialAction(
                        util.setLeftGrabber(false),
                        util.setPivot(0.3)
                    )
                )
                //drive to board
                .strafeToLinearHeading(new Vector2d(35.00, 57.00), Math.toRadians(90.00))
                //open claw
                .afterTime(0, new SequentialAction(util.setRightGrabber(false)))
                .build();

        Action middle = robot.actionBuilder(startPose)
                //Move to team prop
                .strafeTo(new Vector2d(43.00, 12.00))
                //open left claw and then raise arm
                .afterTime(0,
                    new SequentialAction(
                        util.setLeftGrabber(false),
                        util.setPivot(0.3)
                    )
                )
                //drive to board
                .strafeToLinearHeading(new Vector2d(42.00, 57.00), Math.toRadians(90.00))
                //open claw
                .afterTime(0, new SequentialAction(util.setRightGrabber(false)))
                .build();

        Action right = robot.actionBuilder(startPose)
                .waitSeconds(3)
                //Move to team prop
                .strafeTo(new Vector2d(45.00, 29.00))
                //open left claw and then raise arm
                .afterTime(0, util.openLeftGrabber())
                //.afterTime(1, util.setPivot(0.3))
                .waitSeconds(4)
                //drive to board
                .strafeToLinearHeading(new Vector2d(48.00, 57.00), Math.toRadians(90.00))
                //open claw
                .afterTime(0, util.setRightGrabber(false))
                .build();

        Action chosen;

        waitForStart();
        if (isStopRequested()) return;

        // Find team prop pos with camera and then put correct trajectory into chosen
        switch(util.getObjectPosition()) {
            case 1:
                chosen = left;
                break;
            case 2:
                chosen = middle;
                break;
            case 3:
            default:
                chosen = right;
                break;
        }

        Actions.runBlocking(
            new ParallelAction(
                util.closeLeftGrabber(),
                util.setRightGrabber(true),
                util.setPivot(0.03),
                chosen
            )
        );
        sleep(2000);
    }
}
