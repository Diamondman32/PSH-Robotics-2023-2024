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
        Pose2d startPose = new Pose2d(-63.00,-35.00,Math.toRadians(0.00));

        MecanumDrive robot = new MecanumDrive(hardwareMap, startPose);
        ConnectedDevices util = new ConnectedDevices(hardwareMap, robot);

        Action left = robot.actionBuilder(startPose)
                //Move to team prop
                .strafeToLinearHeading(new Vector2d(-33.00, -35.00), Math.toRadians(90.00))
                //open left claw and then raise arm
                .afterTime(0,
                        new SequentialAction(
                                util.openLeftGrabber(),
                                util.setPivotMed()
                        )
                )
                //move to board
                .strafeTo(new Vector2d(-11.00, -35.00))
                .strafeTo(new Vector2d(-11.00, 36.00))
                .strafeTo(new Vector2d(-42.00, 51.00))
                //open claw
                .afterTime(0, util.openRightGrabber())
                .build();

        Action middle = robot.actionBuilder(startPose)
                //Move to team prop
                .strafeToLinearHeading(new Vector2d(-25.00, -49.00), Math.toRadians(90.00))
                //open left claw and then raise arm
                .afterTime(0,
                        new SequentialAction(
                                util.openLeftGrabber(),
                                util.setPivotMed()
                        )
                )
                //move to board
                .strafeTo(new Vector2d(-11.00, -37.00))
                .strafeTo(new Vector2d(-11.00, 36.00))
                .strafeTo(new Vector2d(-36.00, 51.00))
                //open claw
                .afterTime(0, util.openRightGrabber())
                .build();

        Action right = robot.actionBuilder(startPose)
                //Move to team prop
                .strafeToLinearHeading(new Vector2d(-30.00, -37.00), Math.toRadians(269.99))
                //open left claw and then raise arm
                .afterTime(0,
                        new SequentialAction(
                                util.openLeftGrabber(),
                                util.setPivotMed()
                        )
                )
                //move to board
                .strafeToLinearHeading(new Vector2d(-11.00, -37.00), Math.toRadians(90.00))
                .strafeTo(new Vector2d(-11.00, 36.00))
                .strafeTo(new Vector2d(-29.00, 51.00))
                //open claw
                .afterTime(0, util.openRightGrabber())
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
