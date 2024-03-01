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
                .strafeToLinearHeading(new Vector2d(50.00, 20.00), Math.toRadians(269.99))
                //open left claw and then raise arm
                .afterTime(0,
                    new SequentialAction(
                        util.openLeftGrabber(),
                        util.closeRightGrabber(),
                        util.setPivotMed(),
                        util.closeLeftGrabber()
                    )
                )
                .waitSeconds(1)
                //drive to board
                .strafeToLinearHeading(new Vector2d(30.00, 54.00), Math.toRadians(90.00))
                .waitSeconds(1)
                //open claw
                .afterTime(0,
                    new SequentialAction(
                        util.openRightGrabber(),
                        util.setPivotHigh()
                    )
                )
                .waitSeconds(1)
                .build();

        Action middle = robot.actionBuilder(startPose)
                //Move to team prop
                .strafeTo(new Vector2d(40.00, 12.00))
                //open left claw and then raise arm
                .afterTime(0,
                    new SequentialAction(
                        util.openLeftGrabber(),
                        util.closeRightGrabber(),
                        util.setPivotMed()
                    )
                )
                .waitSeconds(1)
                //drive to board
                .strafeToLinearHeading(new Vector2d(33.50, 52.00), Math.toRadians(90.00))
                .waitSeconds(1)
                //open claw
                .afterTime(0,
                        new SequentialAction(
                                util.openRightGrabber(),
                                util.setPivotHigh()
                        )
                )
                .waitSeconds(1)
                .build();

        Action right = robot.actionBuilder(startPose)
                //Move to team prop
                .strafeTo(new Vector2d(45.00, 29.00))
                //open left claw and then raise arm
                .afterTime(0, new SequentialAction(
                        util.openLeftGrabber(),
                        util.closeRightGrabber(),
                        util.setPivotMed()
                    )
                )
                .waitSeconds(1)
                //drive to board
                .strafeToLinearHeading(new Vector2d(34.00, 54.00), Math.toRadians(90.00))
                .waitSeconds(1)
                //open claw
                .afterTime(0,
                        new SequentialAction(
                                util.openRightGrabber(),
                                util.setPivotHigh()
                        )
                )
                .waitSeconds(1)
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
