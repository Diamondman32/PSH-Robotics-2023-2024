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
    public static double left1PoseX = 25.00;
    public static double left1PoseY = 20.00;
    public static double left1Poseθ = 269.99;
    public static double left2PoseX = 35.00;
    public static double left2PoseY = 57.00;
    public static double left2Poseθ = 90.00;

    public static double right1PoseX = 39.00;
    public static double right1PoseY = 23.00;
    public static double right1Poseθ = 180.00;
    public static double right2PoseX = 48.00;
    public static double right2PoseY = 57.00;
    public static double right2Poseθ = 90.00;




    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d startPose = new Pose2d(62.00, 12.00, Math.toRadians(180.00));

        MecanumDrive robot = new MecanumDrive(hardwareMap, startPose);
        ConnectedDevices util = new ConnectedDevices(hardwareMap, robot);

        //Move to left tick mark
        //Pose2d left1Pose = new Pose2d(new Vector2d(25.00, 20.00), Math.toRadians(269.99));
        Pose2d left1Pose = new Pose2d(new Vector2d(left1PoseX, left1PoseY), Math.toRadians(left1Poseθ));
        Pose2d left2Pose = new Pose2d(new Vector2d(left2PoseX, left2PoseY), Math.toRadians(left2Poseθ));
        Action left = robot.actionBuilder(startPose)
                .afterDisp(0, () -> {
                    util.setLeftGrabber(true);
                    util.setRightGrabber(true);
                    util.setPivot(0.03);
                })
                .strafeToLinearHeading(left1Pose.position, left1Pose.heading) //Move to team prop
                .afterDisp(0, () -> {
                    util.setLeftGrabber(false);
                    util.setPivot(0.3);
                })
                //.strafeToLinearHeading(new Vector2d(35.00, 57.00), Math.toRadians(90.00)) //Move to board
                .strafeToLinearHeading(left2Pose.position, left2Pose.heading)
                .afterDisp(0, () -> util.setRightGrabber(false))
                .build();

        Action middle = robot.actionBuilder(startPose)
                .afterDisp(0, () -> {
                    util.setLeftGrabber(true);
                    util.setRightGrabber(true);
                    util.setPivot(0.03);
                })
                .strafeTo(new Vector2d(43.00, 12.00)) //Move to team prop
                .afterDisp(0, () -> {
                    util.setLeftGrabber(false);
                    util.setPivot(0.3);
                })
                .strafeToLinearHeading(new Vector2d(42.00, 57.00), Math.toRadians(90.00)) //Move to board
                .afterDisp(0, () -> util.setRightGrabber(false))
                .build();

        //Pose2d right1Pose = new Pose2d(new Vector2d(39.00, 23.00), Math.toRadians(180.00));
        Pose2d right1Pose = new Pose2d(new Vector2d(right1PoseX, right1PoseY), Math.toRadians(right1Poseθ));
        Pose2d right2Pose = new Pose2d(new Vector2d(right2PoseX, right2PoseY), Math.toRadians(right2Poseθ));
        Action right = robot.actionBuilder(startPose)
                .afterDisp(0, () -> {
                    util.setLeftGrabber(true);
                    util.setRightGrabber(true);
                    util.setPivot(0.03);
                })
                .strafeTo(right1Pose.position) //Move to team prop
                .afterDisp(0, () -> {
                    util.setLeftGrabber(false);
                    util.setPivot(0.3);
                })
                //.strafeToLinearHeading(new Pose2d(new Vector2d(48.00, 57.00), Math.toRadians(90.00)))
                .strafeToLinearHeading(right2Pose.position, right2Pose.heading) //Move to board
                .afterDisp(0, () -> util.setRightGrabber(false))
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

        Actions.runBlocking(chosen);
    }
}
