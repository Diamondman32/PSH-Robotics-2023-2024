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
        ConnectedDevices util = new ConnectedDevices(hardwareMap);

        Action left1 = robot.actionBuilder(robot.pose)
                .strafeTo(new Vector2d(39.00, 23.00)) //Move to team prop
                .build();
        Action left2 = robot.actionBuilder(robot.pose)
                .strafeToLinearHeading(new Vector2d(42.00, 51.00), Math.toRadians(90.00)) //Move to board
                .build();

        Action middle1 = robot.actionBuilder(robot.pose)
                .strafeTo(new Vector2d(35.00, 12.00)) //Move to team prop
                .build();
        Action middle2 = robot.actionBuilder(robot.pose)
                .strafeToLinearHeading(new Vector2d(36.00, 51.00), Math.toRadians(90.00)) //Move to board
                .build();

        Action right1 = robot.actionBuilder(robot.pose)
                .strafeToLinearHeading(new Vector2d(30.00, 12.00), Math.toRadians(269.99)) //Move to team prop
                .build();
        Action right2 = robot.actionBuilder(robot.pose)
                .strafeToLinearHeading(new Vector2d(29.00, 51.00), Math.toRadians(90.00)) //Move to board
                .build();

        Action chosen1, chosen2;

        Action park = robot.actionBuilder(robot.pose)
                .strafeTo(new Vector2d(0,0)) //Get out of teammates way (bad coords, obviously)
                .build();

        waitForStart();
        if (isStopRequested()) return;

        // Find team prop pos with camera and then put correct trajectory into chosen
        switch(util.getObjectPosition()) {
            case 1:
                chosen1 = left1;
                chosen2 = left2;
                break;
            case 2:
                chosen1 = middle1;
                chosen2 = middle2;
                break;
            case 3:
            default:
                chosen1 = right1;
                chosen2 = right2;
                break;
        }

        Actions.runBlocking(
                new SequentialAction(
                        util.setLeftGrabber(true),
                        util.setRightGrabber(true),
                        util.setPivot(0.1),
                        chosen1,
                        util.setLeftGrabber(false),
                        util.setPivot(0.24),
                        util.setArmPos(1),
                        util.setLeftGrabber(true),
                        chosen2,
                        util.setRightGrabber(false)
                )
        );
    }
}
