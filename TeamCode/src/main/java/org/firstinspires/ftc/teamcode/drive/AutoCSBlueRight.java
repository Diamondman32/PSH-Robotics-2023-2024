package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.ArrayList;

@Config
@Autonomous(name = "AutoCSBlueRight")
public class AutoCSBlueRight extends LinearOpMode {
    //private final ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap);

        robot.resetYaw();

        //TrajSeq
        Pose2d startPose = new Pose2d(-63.00,-35.00,Math.toRadians(0.00));
        TrajectorySequence left = robot.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-33.00, -35.00, Math.toRadians(90.00)))
                .lineToLinearHeading(new Pose2d(-11.00, -35.00, Math.toRadians(90.00)))
                .lineToLinearHeading(new Pose2d(-11.00, 36.00, Math.toRadians(90.00)))
                .lineToLinearHeading(new Pose2d(-42.00, 51.00, Math.toRadians(90.00)))
                .build();
        TrajectorySequence middle = robot.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-25.00, -49.00, Math.toRadians(90.00)))
                .lineToLinearHeading(new Pose2d(-11.00, -37.00, Math.toRadians(90.00)))
                .lineToLinearHeading(new Pose2d(-11.00, 36.00, Math.toRadians(90.00)))
                .lineToLinearHeading(new Pose2d(-36.00, 51.00, Math.toRadians(90.00)))
                .build();
        TrajectorySequence right = robot.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-30.00, -37.00, Math.toRadians(269.99)))
                .lineToLinearHeading(new Pose2d(-11.00, -37.00, Math.toRadians(90.00)))
                .lineToLinearHeading(new Pose2d(-11.00, 36.00, Math.toRadians(90.00)))
                .lineToLinearHeading(new Pose2d(-29.00, 51.00, Math.toRadians(90.00)))
                .build();


        //Dodging Pose Errors
        Trajectory error = robot.trajectoryBuilder(new Pose2d())
                .forward(1)
                .build();

        //Based on team prop position only one of these 3 will run
        Trajectory capMiddle = robot.trajectoryBuilder(error.end())
                .lineToConstantHeading(new Vector2d(31,0))
                .build();
        Trajectory capLeft = robot.trajectoryBuilder(error.end())
                .lineToLinearHeading(new Pose2d(20,5,Math.toRadians(35)))
                .build();
        Trajectory capRight = robot.trajectoryBuilder(error.end())
                .lineToLinearHeading(new Pose2d(20,-5,Math.toRadians(35)))
                .build();

        //Move To go under truss
        Trajectory goToBoard1FromLeft = robot.trajectoryBuilder(capLeft.end())
                .lineToConstantHeading(new Vector2d(52,0))
                .build();
        Trajectory goToBoard1FromMiddle = robot.trajectoryBuilder(capMiddle.end())
                .splineToConstantHeading(new Vector2d(52,0),Math.toRadians(90))
                .build();
        Trajectory goToBoard1FromRight = robot.trajectoryBuilder(capRight.end())
                .lineToConstantHeading(new Vector2d(52,0))
                .build();

        //Move Towards Board
        Trajectory goToBoard2 = robot.trajectoryBuilder(new Pose2d(52,0,0))
                .lineToLinearHeading(new Pose2d(52,-60,Math.toRadians(-135)))
                .build();

        //Based on location of team prop, move to 1 of the 3 correct QR codes
        // (NOTE: MIGHT NOT USE ROADRUNNER FOR THIS)
        Trajectory goToLeftBoard = robot.trajectoryBuilder(goToBoard2.end())
                .lineToLinearHeading(new Pose2d(-5,-75,Math.toRadians(-90)))//strafe
                .build();
        Trajectory goToMiddleBoard = robot.trajectoryBuilder(goToBoard2.end())
                .lineToLinearHeading(new Pose2d(-5,-75,Math.toRadians(-90)))//strafe
                .build();
        Trajectory goToRightBoard = robot.trajectoryBuilder(goToBoard2.end())
                .lineToLinearHeading(new Pose2d(-5,-75,Math.toRadians(-90)))//strafe
                .build();

        robot.setPoseEstimate(new Pose2d());

        waitForStart();
        if (isStopRequested()) return;
//----------------------------------------START OF AUTO---------------------------------------------

        //Tighten Grabbers
        robot.setLeftGrabber(0.37);
        robot.setRightGrabber(1.00);

        //Based on team prop position, cap a pixel on the correct tick mark
        switch(robot.getObjectPosition()) {//Note: getObjectPosition doesn't work
            case 1:
                if (robot.trajSeq) {
                    robot.setPoseEstimate(left.start());
                    robot.followTrajectorySequence(left);
                } else {
                    robot.setPivot(0.1);//lower pix-arm
                    sleep(500);
                    robot.followTrajectory(capLeft);//move over tick mark
                    sleep(100);
                    robot.setLeftGrabber(0.61);//open claw (drop pixel)
                    sleep(150);
                    robot.setPivot(0.6);//raise pix-arm
                    sleep(50);
                    robot.setLeftGrabber(0.37);//close claw
                    robot.followTrajectory(goToBoard1FromLeft);
                    robot.followTrajectory(goToBoard2);
                    robot.moveToAprilTag(1);
                }
                break;
            case 2:
                if (robot.trajSeq) {
                    robot.setPoseEstimate(middle.start());
                    robot.followTrajectorySequence(middle);
                } else {
                    robot.setPivot(0.1);//lower pix-arm
                    sleep(500);
                    robot.followTrajectory(capMiddle);//move over tick mark
                    sleep(100);
                    robot.setLeftGrabber(0.61);//open claw (drop pixel)
                    sleep(150);
                    robot.setPivot(0.6);//raise pix-arm
                    sleep(50);
                    robot.setLeftGrabber(0.37);//close claw
                    robot.followTrajectory(goToBoard1FromMiddle);
                    robot.followTrajectory(goToBoard2);
                    robot.moveToAprilTag(2);
                }
                break;
            case 3:
                if (robot.trajSeq) {
                    robot.setPoseEstimate(right.start());
                    robot.followTrajectorySequence(right);
                } else {
                    robot.setPivot(0.1);//lower pix-arm
                    sleep(500);
                    robot.followTrajectory(capRight);//move over tick mark
                    sleep(100);
                    robot.setLeftGrabber(0.61);//open claw (drop pixel)
                    sleep(150);
                    robot.setPivot(0.6);//raise pix-arm
                    sleep(50);
                    robot.setLeftGrabber(0.37);//close claw
                    robot.followTrajectory(goToBoard1FromRight);
                    robot.followTrajectory(goToBoard2);
                    robot.moveToAprilTag(3);
                }
                break;
        }
        sleep(3000);
    }
}