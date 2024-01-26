package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "AutoCSBlueRight")
public class AutoCSBlueRight extends LinearOpMode {
    private ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive robot = new SampleMecanumDrive(hardwareMap);

        boolean objFound = false;
        int objPos = 0;

        Trajectory error = robot.trajectoryBuilder(new Pose2d())
                .forward(1)
                .build();
        Trajectory moveToCones = robot.trajectoryBuilder(error.end())
                .lineToLinearHeading(new Pose2d(20,0,Math.toRadians(0)))
                .build();
        Trajectory capMiddle = robot.trajectoryBuilder(moveToCones.end())
                .lineToLinearHeading(new Pose2d(28,0,Math.toRadians(0)))
                .build();
        Trajectory checkRight = robot.trajectoryBuilder(moveToCones.end())
                .lineToLinearHeading(new Pose2d(17,0,Math.toRadians(-24)))
                .build();
        Trajectory capLeft = robot.trajectoryBuilder(checkRight.end())
                .lineToLinearHeading(new Pose2d(20,5,Math.toRadians(32)))
                .build();
        Trajectory capRight = robot.trajectoryBuilder(checkRight.end())
                .forward(10)
                .build();
        Trajectory rotate1 = robot.trajectoryBuilder(capLeft.end())
                .lineToLinearHeading(new Pose2d(25,0, Math.toRadians(-80)))
                .build();
        Trajectory moveToBoard1 = robot.trajectoryBuilder(rotate1.end())
                .lineToLinearHeading(new Pose2d(25,-45, Math.toRadians(-80)))
                .build();


        waitForStart();
        if (isStopRequested()) return;

        //Set Pose Estimate
        robot.setPoseEstimate(new Pose2d());

        //Tighten Grabbers
        robot.setLeftGrabber(0.37);
        robot.setRightGrabber(1.00);

        //Drive to scan
        robot.followTrajectory(moveToCones);
        //scan forward if object cap it
        telemetry.addData("dist",robot.getDist());
        telemetry.update();
        if (robot.isDistClose(15)) {
            objFound = true;
            objPos = 2;
            robot.setPivot(0.05);//lower pixarm
            sleep(500);
            robot.followTrajectory(capMiddle);//push team prop
            sleep(100);
            robot.setLeftGrabber(0.61);//drop pixel
            sleep(150);
            robot.setPivot(0.57);//raise pixarm
            sleep(3000);
        }
        //scan left ~90 deg
        if (!objFound) //ASYNC FINDING
            robot.followTrajectoryAsync(checkRight);
        timer.reset();
        while (timer.milliseconds() < 1000 && !objFound) {
            robot.update();
            if (robot.isDistClose(14)) {
                robot.breakFollowing();
                objFound = true;
                objPos = 3;
                robot.setPivot(0.05);
                sleep(500);
                robot.followTrajectory(capRight);
                robot.setLeftGrabber(0.61);
                sleep(100);
                robot.setPivot(0.57);
                sleep(3000);
            }
        }
        //if object found cap left otherwise cap right
        if (!objFound) {
            objPos = 1;
            robot.setPivot(0.05);
            sleep(100);
            robot.followTrajectory(capLeft);
            robot.setLeftGrabber(0.61);
            sleep(100);
            robot.setPivot(0.57);
            sleep(3000);
        }
        //go forward to clear pole
        if (objPos == 1) {
            //robot.turn(Math.toRadians(90));
            robot.followTrajectory(rotate1);
            robot.followTrajectory(moveToBoard1);
        }
        /*if (objPos == 2) {
            //robot.followTrajectory(park0FromMiddle);
            robot.followTrajectory(park1FromMiddle);
            robot.followTrajectory(turnFromMiddle);
            robot.followTrajectory(park2FromMiddle);
        }
        if (objPos == 3) {
            robot.followTrajectory(park1FromRight);
            robot.followTrajectory(turnFromRight);
            robot.followTrajectory(park2FromRight);
        }*/

        //robot.followTrajectory(traj1);
        //robot.followTrajectory(traj2);
    }
}