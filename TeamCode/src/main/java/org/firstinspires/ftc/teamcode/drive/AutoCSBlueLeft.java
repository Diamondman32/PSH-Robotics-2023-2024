package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "AutoCSBlueLeft")
public class AutoCSBlueLeft extends LinearOpMode {
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
        Trajectory checkLeft = robot.trajectoryBuilder(moveToCones.end())
                .lineToLinearHeading(new Pose2d(17,0,Math.toRadians(20)))
                .build();
        Trajectory capLeft = robot.trajectoryBuilder(checkLeft.end())
                .forward(5)
                .build();
        Trajectory capRight = robot.trajectoryBuilder(checkLeft.end())
                .lineToLinearHeading(new Pose2d(20,-5,Math.toRadians(-32)))
                .build();
        Trajectory park1FromLeft = robot.trajectoryBuilder(capLeft.end())
                .lineToLinearHeading(new Pose2d(53,0, Math.toRadians(0)))
                .build();
        Trajectory park0FromMiddle = robot.trajectoryBuilder(capMiddle.end())
                .lineToLinearHeading(new Pose2d(56,30, Math.toRadians(0)))
                .build();
        Trajectory park1FromMiddle = robot.trajectoryBuilder(park0FromMiddle.end())
                .lineToLinearHeading(new Pose2d(53,0, Math.toRadians(0)))
                .build();
        Trajectory park1FromRight = robot.trajectoryBuilder(capRight.end())
                .lineToLinearHeading(new Pose2d(53,0, Math.toRadians(0)))
                .build();
        Trajectory turnFromLeft = robot.trajectoryBuilder(park1FromLeft.end())
                .lineToLinearHeading(new Pose2d(54,0, Math.toRadians(-70)))
                .build();
        Trajectory turnFromMiddle = robot.trajectoryBuilder(park1FromMiddle.end())
                .lineToLinearHeading(new Pose2d(54,0, Math.toRadians(-70)))//90 degrees too much
                .build();
        Trajectory turnFromRight = robot.trajectoryBuilder(park1FromRight.end())
                .lineToLinearHeading(new Pose2d(54,0, Math.toRadians(-70)))
                .build();
        Trajectory park2FromLeft = robot.trajectoryBuilder(turnFromLeft.end())
                .lineToLinearHeading(new Pose2d(54,-92, Math.toRadians(-70)))
                .build();
        Trajectory park2FromMiddle = robot.trajectoryBuilder(turnFromMiddle.end())
                .lineToLinearHeading(new Pose2d(54,-92, Math.toRadians(-70)))
                .build();
        Trajectory park2FromRight = robot.trajectoryBuilder(turnFromRight.end())
                .lineToLinearHeading(new Pose2d(54,-92, Math.toRadians(-70)))
                .build();

        /*Trajectory traj1 = robot.trajectoryBuilder(new Pose2d())
                .forward(1)
                .build();
        Trajectory traj2 = robot.trajectoryBuilder(traj1.end())
                .lineToLinearHeading(new Pose2d(-40,0, Math.toRadians(90)))
                .build();*/


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
            robot.followTrajectoryAsync(checkLeft);
        timer.reset();
        while (timer.milliseconds() < 1000 && !objFound) {
            robot.update();
            if (robot.isDistClose(14)) {
                robot.breakFollowing();
                objFound = true;
                objPos = 1;
                robot.setPivot(0.05);
                sleep(500);
                robot.followTrajectory(capLeft);
                robot.setLeftGrabber(0.61);
                sleep(100);
                robot.setPivot(0.57);
                sleep(3000);
            }
        }
        //if object found cap left otherwise cap right
        if (!objFound) {
            objPos = 3;
            robot.setPivot(0.05);
            sleep(100);
            robot.followTrajectory(capRight);
            robot.setLeftGrabber(0.61);
            sleep(100);
            robot.setPivot(0.57);
            sleep(3000);
        }
        //go forward to clear pole
        /*if (objPos == 1) {
            //robot.turn(Math.toRadians(90));
            robot.followTrajectory(park1FromLeft);
            robot.followTrajectory(turnFromLeft);
            robot.followTrajectory(park2FromLeft);
        }
        if (objPos == 2) {
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