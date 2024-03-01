package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.*;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.lang.Math;

@Config
@Autonomous(name = "AutoCSBlueRight")
public class AutoCSBlueRight extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d startPose = new Pose2d(-62.00,-38.00,Math.toRadians(0.00));

        MecanumDrive robot = new MecanumDrive(hardwareMap, startPose);
        ConnectedDevices util = new ConnectedDevices(hardwareMap, robot);

        Action right = robot.actionBuilder(startPose)
                //Move to team prop
                //.strafeToLinearHeading(new Vector2d(-30.00, -37.00), Math.toRadians(269.99))
                //.splineToSplineHeading(new Pose2d(-33.00, -33.00, Math.toRadians(269.51)), Math.toRadians(160.15))
                .splineToSplineHeading(new Pose2d(-43.00, -38.00, Math.toRadians(-50.61)), Math.toRadians(24.50))
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
                //move to board
                .strafeToLinearHeading(new Vector2d(-12.00, -35.00), Math.toRadians(90.00))
                .strafeTo(new Vector2d(-12.00, 36.00))
                .strafeTo(new Vector2d(-29.00, 51.00))
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
                .strafeToLinearHeading(new Vector2d(-25.00, -51.00), Math.toRadians(90.00))
                //open left claw and then raise arm
                .afterTime(0,
                        new SequentialAction(
                                util.openLeftGrabber(),
                                util.closeRightGrabber(),
                                util.setPivotMed()
                        )
                )
                .waitSeconds(.5)
                //move to board
                .strafeTo(new Vector2d(-12.00, -51.00))
                .strafeTo(new Vector2d(-12.00, 36.00))
                .strafeTo(new Vector2d(-36.00, 52.00))
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
                //.strafeToLinearHeading(new Vector2d(33.00, -35.00), Math.toRadians(90.00))
                .splineToSplineHeading(new Pose2d(-30.00, -38.00, Math.toRadians(90.00)), Math.toRadians(90.00))
                //open left claw and then raise arm
                .afterTime(0,
                        new SequentialAction(
                                util.openLeftGrabber(),
                                util.closeRightGrabber(),
                                util.setPivotMed()
                        )
                )
                .waitSeconds(.5)
                //move to board
                .strafeTo(new Vector2d(-12.00, -35.00))
                .strafeTo(new Vector2d(-12.00, 36.00))
                .strafeTo(new Vector2d(-42.00, 52.00))
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
