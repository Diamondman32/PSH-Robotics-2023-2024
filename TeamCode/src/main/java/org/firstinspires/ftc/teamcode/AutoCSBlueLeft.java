package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.*;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.lang.Math;

@Config
@Autonomous(name = "AutoCSBlueLeft")
public class AutoCSBlueLeft extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d startPose = new Pose2d(-62.00, 12.00, Math.toRadians(0.00));

        MecanumDrive robot = new MecanumDrive(hardwareMap, startPose);
        ConnectedDevices util = new ConnectedDevices(hardwareMap, robot);

        Action right = robot.actionBuilder(startPose)
                //Move to team prop
                .splineToSplineHeading(new Pose2d(-32.00, 19.00, Math.toRadians(270.00)), Math.toRadians(0.00))
                //open left claw and then raise arm
                .afterTime(0,
                        new SequentialAction(
                                util.openLeftGrabber(),
                                util.closeRightGrabber(),
                                util.setPivotMed(),
                                util.closeLeftGrabber()
                        )
                )
                .waitSeconds(.5)
                //drive to board
                .strafeToLinearHeading(new Vector2d(-30.00, 52.00), Math.toRadians(90.00))
                .waitSeconds(.5)
                //open claw
                .afterTime(0,
                        new SequentialAction(
                                util.openRightGrabber(),
                                util.setPivotHigh()
                        )
                )
                .waitSeconds(.5)
                .build();

        Action middle = robot.actionBuilder(startPose)
                //Move to team prop
                .strafeTo(new Vector2d(-40.00, 12.00))
                //open left claw and then raise arm
                .afterTime(0,
                        new SequentialAction(
                                util.openLeftGrabber(),
                                util.closeRightGrabber(),
                                util.setPivotMed()
                        )
                )
                .waitSeconds(.5)
                //drive to board
                .strafeToLinearHeading(new Vector2d(-33.50, 52.00), Math.toRadians(90.00))
                .waitSeconds(.5)
                //open claw
                .afterTime(0,
                        new SequentialAction(
                                util.openRightGrabber(),
                                util.setPivotHigh()
                        )
                )
                .waitSeconds(.5)
                .build();

        Action left = robot.actionBuilder(startPose)
                //Move to team prop
                .strafeTo(new Vector2d(-45.00, 29.00))
                //open left claw and then raise arm
                .afterTime(0, new SequentialAction(
                                util.openLeftGrabber(),
                                util.closeRightGrabber(),
                                util.setPivotMed()
                        )
                )
                .waitSeconds(.5)
                //drive to board
                .strafeToLinearHeading(new Vector2d(-34.00, 54.00), Math.toRadians(90.00))
                .waitSeconds(.5)
                //open claw
                .afterTime(0,
                        new SequentialAction(
                                util.openRightGrabber(),
                                util.setPivotHigh()
                        )
                )
                .waitSeconds(.5)
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
                new SequentialAction(
                        util.closeLeftGrabber(),
                        util.closeRightGrabber(),
                        util.setPivotDown(),
                        chosen
                )
        );
        sleep(2000);
    }
}
