package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.*;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.lang.Math;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

@Config
@Autonomous(name = "AutoCSRedLeft")
public class AutoCSRedLeft extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d startPose = new Pose2d(63.00,-35.00,Math.toRadians(180.00));

        MecanumDrive robot = new MecanumDrive(hardwareMap, startPose);
        ConnectedDevices util = new ConnectedDevices(hardwareMap, robot);

        Pose2d left1Pose = new Pose2d(new Vector2d(30.00, -37.00), Math.toRadians(269.99));
        Action left1 = robot.actionBuilder(startPose)
                .strafeToLinearHeading(left1Pose.position, left1Pose.heading) //Move to team prop
                .build();
        Action left2 = robot.actionBuilder(left1Pose)
                .strafeToLinearHeading(new Vector2d(11.00, -35.00), Math.toRadians(90.00))
                .strafeTo(new Vector2d(11.00, 36.00))
                .strafeTo(new Vector2d(29.00, 51.00)) //Move to board
                .build();

        Pose2d middle1Pose = new Pose2d(new Vector2d(25.00, -49.00), Math.toRadians(90.00));
        Action middle1 = robot.actionBuilder(startPose)
                .strafeToLinearHeading(middle1Pose.position, middle1Pose.heading) //Move to team prop
                .build();
        Action middle2 = robot.actionBuilder(middle1Pose)
                .strafeTo(new Vector2d(11.00, -37.00))
                .strafeTo(new Vector2d(11.00, 36.00))
                .strafeTo(new Vector2d(36.00, 51.00)) //Move to board
                .build();

        Pose2d right1Pose = new Pose2d(new Vector2d(33.00, -35.00), Math.toRadians(90.00));
        Action right1 = robot.actionBuilder(startPose)
                .strafeToLinearHeading(right1Pose.position, right1Pose.heading) //Move to team prop
                .build();
        Action right2 = robot.actionBuilder(right1Pose)
                .strafeTo(new Vector2d(11.00, -37.00))
                .strafeTo(new Vector2d(11.00, 36.00))
                .strafeTo(new Vector2d(42.00, 51.00)) //Move to board
                .build();

        Action chosen1, chosen2;

        Action park = robot.actionBuilder(robot.pose)
                .strafeTo(new Vector2d(0,0)) //Get out of teammates way (bad coords, obviously)
                .build();

        waitForStart();
        if (isStopRequested()) return;

        // Find team prop pos with camera and then put correct trajectory into chosen
        /*switch(util.getObjectPosition()) {
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
        }*/
        chosen1 = right1;
        chosen2 = right2;

        Actions.runBlocking(
                new SequentialAction(
                        //util.setLeftGrabber(true),
                        //util.setRightGrabber(true),
                        //util.setPivot(0.1),
                        chosen1,
                        //util.setLeftGrabber(false),
                        //util.setPivot(0.24),
                        //util.setArmPos(1),
                        //util.setLeftGrabber(true),
                        chosen2
                        //util.setRightGrabber(false)
                )
        );
    }
}
