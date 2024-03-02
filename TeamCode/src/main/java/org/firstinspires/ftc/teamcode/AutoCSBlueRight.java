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
                //.splineToSplineHeading(new Pose2d(-43.00, -38.00, Math.toRadians(-50.61)), Math.toRadians(24.50))
                .splineToSplineHeading(new Pose2d(-46.00, -38.00, Math.toRadians(-35)), Math.toRadians(12.5))
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
                //move to board
                .strafeToLinearHeading(new Vector2d(-4.00, -38.00), Math.toRadians(90.00))
                .strafeTo(new Vector2d(-12.00, 36.00))
                .strafeTo(new Vector2d(-28.00, 44.00))
                .strafeToLinearHeading(new Vector2d(-28.00, 48.00), Math.toRadians(90.00),
                        new TranslationalVelConstraint(10.0), new ProfileAccelConstraint(-5.0,5.0))
                .waitSeconds(.5)
                //open claw
                .afterTime(0, util.openRightGrabber())
                .afterTime(.3, util.setPivotHigh())
                .waitSeconds(.5)
                //.strafeTo(new Vector2d(-62.00,50.00)) //PARK
                .build();

        Action middle = robot.actionBuilder(startPose)
                //Move to team prop
                //.strafeToLinearHeading(new Vector2d(-25.00, -51.00), Math.toRadians(90.00))
                .strafeToLinearHeading(new Vector2d(-16.00, -63.00), Math.toRadians(90.00))
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
                //move to board
                .strafeTo(new Vector2d(-2.00, -63.00))
                .strafeTo(new Vector2d(-18.00, 33.00))
                .strafeTo(new Vector2d(-32, 40.00))
                .strafeToLinearHeading(new Vector2d(-32.00, 44.00), Math.toRadians(90.00),
                        new TranslationalVelConstraint(10.0), new ProfileAccelConstraint(-5.0,5.0))
                .waitSeconds(.5)
                //open claw
                .afterTime(0, util.openRightGrabber())
                .afterTime(.3, util.setPivotHigh())
                .waitSeconds(.5)
                //.strafeTo(new Vector2d(-62.00,50.00))
                .build();

        Action left = robot.actionBuilder(startPose)
                //Move to team prop
                //.splineToSplineHeading(new Pose2d(-30.00, -38.00, Math.toRadians(90.00)), Math.toRadians(90.00))
                .strafeToLinearHeading(new Vector2d(-25.00, -44.00), Math.toRadians(90.00))
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
                //move to board
                .strafeTo(new Vector2d(-4.00, -50.00))
                .strafeTo(new Vector2d(-8.00, 36.00))
                .waitSeconds(5)
                .strafeTo(new Vector2d(-44.00, 42.00))
                .strafeToLinearHeading(new Vector2d(-43.00, 44.00), Math.toRadians(90.00),
                        new TranslationalVelConstraint(10.0), new ProfileAccelConstraint(-5.0,5.0))
                .waitSeconds(.5)
                //open claw
                .afterTime(0, util.openRightGrabber())
                .afterTime(.3, util.setPivotHigh())
                .waitSeconds(.5)
                //.strafeTo(new Vector2d(-62.00,50.00))
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
/*
.strafeToLinearHeading(new Vector2d(-12.00, -35.00), Math.toRadians(90.00))
                .strafeTo(new Vector2d(-12.00, 36.00))
                .strafeTo(new Vector2d(-29.00, 51.00))
                .strafeToLinearHeading(new Vector2d(-29.00, 53.00), Math.toRadians(90.00),
                        new TranslationalVelConstraint(10.0), new ProfileAccelConstraint(-5.0,5.0))
.strafeTo(new Vector2d(-12.00, -51.00))
                .strafeTo(new Vector2d(-12.00, 36.00))
                .strafeTo(new Vector2d(-36.00, 52.00))
                .strafeToLinearHeading(new Vector2d(-36.00, 54.00), Math.toRadians(90.00),
                        new TranslationalVelConstraint(10.0), new ProfileAccelConstraint(-5.0,5.0))
.strafeTo(new Vector2d(-12.00, -35.00))
                .strafeTo(new Vector2d(-12.00, 36.00))
                .strafeTo(new Vector2d(-42.00, 52.00))
                .strafeToLinearHeading(new Vector2d(-42.00, 54.00), Math.toRadians(90.00),
                        new TranslationalVelConstraint(10.0), new ProfileAccelConstraint(-5.0,5.0))
 */