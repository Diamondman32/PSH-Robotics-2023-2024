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
                .splineToSplineHeading(new Pose2d(-35.00, 16.00, Math.toRadians(320.00)), Math.toRadians(325.00))//-40, -35
                //.splineToSplineHeading(new Pose2d(-34.00, 17.00, Math.toRadians(225.00)), Math.toRadians(225.00))
                //open left claw and then raise arm
                .afterTime(0,
                        new SequentialAction(
                                util.openLeftGrabber(),
                                util.closeRightGrabber(),
                                util.wait1(),
                                util.setPivotMed()
                        )
                )
                .afterTime(1, util.closeLeftGrabber())
                .waitSeconds(1.2)
                //drive to board
                //.strafeToLinearHeading(new Vector2d(-22.00, 54.00), Math.toRadians(90.00))
                .strafeToLinearHeading(new Vector2d(-21.50, 48.00), Math.toRadians(87.00))
                .strafeToLinearHeading(new Vector2d(-21.50, 48.00), Math.toRadians(90.00),
                        new TranslationalVelConstraint(10.0), new ProfileAccelConstraint(-5.0,5.0))
                .waitSeconds(.5)
                //open claw
                .afterTime(0, util.openRightGrabber())
                .afterTime(.3, util.setPivotHigh())
                .waitSeconds(.5)
                .strafeTo(new Vector2d(-62.00,50.00))
                .build();

        Action middle = robot.actionBuilder(startPose)
                //Move to team prop
                //.strafeTo(new Vector2d(-40.00, 12.00))
                .strafeTo(new Vector2d(-39.00, 12.00))

                //open left claw and then raise arm
                .afterTime(0,
                        new SequentialAction(
                                util.openLeftGrabber(),
                                util.closeRightGrabber(),
                                util.wait1(),
                                util.setPivotMed()
                        )
                )
                .waitSeconds(1.2)
                //drive to board
                //.strafeToLinearHeading(new Vector2d(-33.50, 52.00), Math.toRadians(90.00))
                .strafeToLinearHeading(new Vector2d(-33.50, 51.00), Math.toRadians(90.00))
                .strafeToLinearHeading(new Vector2d(-33.50, 53.00), Math.toRadians(90.00),
                        new TranslationalVelConstraint(10.0), new ProfileAccelConstraint(-5.0,5.0))
                .waitSeconds(.5)
                //open claw
                .afterTime(0, util.openRightGrabber())
                .afterTime(.3, util.setPivotHigh())
                .waitSeconds(.5)
                .strafeTo(new Vector2d(-62.00,50.00))
                .build();

        Action left = robot.actionBuilder(startPose)
                //Move to team prop
                //.strafeTo(new Vector2d(-45.00, 29.00))
                .strafeTo(new Vector2d(-45.00, 31.00))
                //open left claw and then raise arm
                .afterTime(0,
                        new SequentialAction(
                                util.openLeftGrabber(),
                                util.closeRightGrabber(),
                                util.wait1(),
                                util.setPivotMed()
                        )
                )
                .afterTime(1, util.closeLeftGrabber())
                .waitSeconds(1.2)
                //drive to board
                //.strafeToLinearHeading(new Vector2d(-34.00, 54.00), Math.toRadians(90.00))
                .strafeToLinearHeading(new Vector2d(-34.00, 52.00), Math.toRadians(85.00))
                .strafeToLinearHeading(new Vector2d(-34.00, 54.00), Math.toRadians(90.00),
                        new TranslationalVelConstraint(10.0), new ProfileAccelConstraint(-5.0,5.0))
                .waitSeconds(.5)
                //open claw
                .afterTime(0, util.openRightGrabber())
                .afterTime(.3, util.setPivotHigh())
                .waitSeconds(.5)
                .strafeTo(new Vector2d(-62.00,50.00))
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
                        util.wait1(),
                        chosen
                )
        );
        sleep(2000);
    }
}
