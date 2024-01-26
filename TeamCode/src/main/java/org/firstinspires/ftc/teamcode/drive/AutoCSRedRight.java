package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "AutoCSRedRight")
public class AutoCSRedRight extends LinearOpMode {
    private final ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive robot = new SampleMecanumDrive(hardwareMap);

        boolean objFound = false;
        int objPos = 0;

        robot.resetYaw();

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
                .lineToLinearHeading(new Pose2d(30,0, Math.toRadians(-100)))
                .build();
        Trajectory moveToBoard1 = robot.trajectoryBuilder(rotate1.end())
                .lineTo(new Vector2d(30,-35))
                .build();
        Trajectory park1 = robot.trajectoryBuilder(moveToBoard1.end())
                .lineTo(new Vector2d(0,-35))
                .build();
        Trajectory rotate2 = robot.trajectoryBuilder(capMiddle.end())
                .lineToLinearHeading(new Pose2d(30,0, Math.toRadians(-100)))
                .build();
        Trajectory moveToBoard2 = robot.trajectoryBuilder(rotate2.end())
                .lineTo(new Vector2d(30,-30))
                .build();
        Trajectory park2 = robot.trajectoryBuilder(moveToBoard2.end())
                .lineTo(new Vector2d(0,-30))
                .build();
        Trajectory rotate3 = robot.trajectoryBuilder(capRight.end())
                .lineToLinearHeading(new Pose2d(30,0, Math.toRadians(-100)))
                .build();
        Trajectory moveToBoard3 = robot.trajectoryBuilder(rotate3.end())
                .lineTo(new Vector2d(30,-30))
                .build();
        Trajectory park3 = robot.trajectoryBuilder(moveToBoard3.end())
                .lineTo(new Vector2d(0,-30))
                .build();


        waitForStart();
        if (isStopRequested()) return;

        telemetry.addData("yaw",robot.getYaw());
        telemetry.update();

        //Set Pose Estimate
        robot.setPoseEstimate(new Pose2d());

        //Tighten Grabbers
        robot.setLeftGrabber(0.37);
        robot.setRightGrabber(1.00);

        //Drive to scan
        robot.followTrajectory(moveToCones);
        //scan forward if object cap it
        if (robot.isDistClose(10)) {
            objFound = true;
            objPos = 2;
            robot.setPivot(0.1);//lower pixarm
            sleep(500);
            robot.followTrajectory(capMiddle);//push team prop
            sleep(100);
            robot.setLeftGrabber(0.61);//drop pixel
            sleep(150);
            robot.setPivot(0.6);//raise pixarm
            sleep(3000);
        }

        //scan left ~90 deg
        if (!objFound) //ASYNC FINDING
            robot.followTrajectoryAsync(checkRight);
        timer.reset();
        while (timer.milliseconds() < 1000 && !objFound) {
            robot.update();
            if (robot.isDistClose(10)) {
                robot.breakFollowing();
                objFound = true;
                objPos = 3;
                robot.setPivot(0.1);
                sleep(500);
                robot.followTrajectory(capRight);
                robot.setLeftGrabber(0.61);
                sleep(100);
                robot.setPivot(0.6);
                sleep(3000);
            }
        }

        //if object found cap left otherwise cap right
        if (!objFound) {
            objPos = 1;
            robot.setPivot(0.1);
            sleep(100);
            robot.followTrajectory(capLeft);
            robot.setLeftGrabber(0.61);
            sleep(100);
            robot.setPivot(0.6);
            sleep(3000);
        }

        //go forward to clear pole
        if (objPos == 1) {
            robot.followTrajectoryAsync(rotate1);
            robot.update();
            telemetry.addData("initYaw", robot.getYaw());
            telemetry.update();
            robot.setIsRotatedStartDeg(robot.getYaw());
            while (!robot.isRotated(-86)) {
                robot.update();
                telemetry.addData("yaw", robot.getYaw());
                telemetry.update();
            }
            robot.breakFollowing();
            //robot.setArmPos(1);
            //robot.setPivot(0.24);
            robot.followTrajectory(moveToBoard1);
        }
            /*sleep(100);
            robot.setRightGrabber(0.7);
            sleep(100);
            robot.followTrajectory(park1);
        }
        if (objPos == 2) {
            robot.followTrajectoryAsync(rotate2);
            robot.update();
            telemetry.addData("yaw",robot.getYaw());
            telemetry.update();
            robot.setIsRotatedStartDeg(robot.getYaw());
            while(!robot.isRotated(-86))
                robot.update();
            telemetry.addData("yaw2",robot.getYaw());
            telemetry.update();
            robot.breakFollowing();
            robot.followTrajectory(moveToBoard2);
            robot.setPivot(0.24);
            sleep(100);
            robot.setRightGrabber(0.7);
            sleep(100);
            robot.followTrajectory(park2);
        }
        if (objPos == 3) {
            robot.followTrajectoryAsync(rotate3);
            robot.update();
            telemetry.addData("yaw",robot.getYaw());
            telemetry.update();
            robot.setIsRotatedStartDeg(robot.getYaw());
            while(!robot.isRotated(-86))
                robot.update();
            telemetry.addData("yaw2",robot.getYaw());
            telemetry.update();
            robot.breakFollowing();
            robot.followTrajectory(moveToBoard3);
            robot.setPivot(0.24);
            sleep(100);
            robot.setRightGrabber(0.7);
            sleep(100);
            robot.followTrajectory(park3);
        }*/

        sleep(3000);
    }
}