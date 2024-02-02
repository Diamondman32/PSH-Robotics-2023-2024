package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "AutoCSRedLeft")
public class AutoCSRedLeft extends LinearOpMode {
    private final ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap);

        boolean objFound = false;
        int objPos = 0;

        robot.resetYaw();

        Trajectory error = robot.trajectoryBuilder(new Pose2d())
                .forward(1)
                .build();
        Trajectory moveToCones = robot.trajectoryBuilder(error.end())
                .lineToConstantHeading(new Vector2d(21,0))
                .build();
        Trajectory capMiddle = robot.trajectoryBuilder(moveToCones.end())
                .lineToConstantHeading(new Vector2d(31,0))
                .build();
        Trajectory checkLeft = robot.trajectoryBuilder(moveToCones.end())
                .lineToLinearHeading(new Pose2d(18,-2,Math.toRadians(40)))
                .build();
        Trajectory capLeft = robot.trajectoryBuilder(new Pose2d(moveToCones.end().getX(),moveToCones.end().getY(),63))
                .forward(8)
                .build();
        Trajectory capRight = robot.trajectoryBuilder(new Pose2d(moveToCones.end().getX(),moveToCones.end().getY(),63))
                .lineToLinearHeading(new Pose2d(20,-5,Math.toRadians(-25)))
                .build();

        Trajectory rotate1L = robot.trajectoryBuilder(capLeft.end())
                .lineToLinearHeading(new Pose2d(30,0, Math.toRadians(-15)))
                .build();
        Trajectory goToBoard1L = robot.trajectoryBuilder(new Pose2d(moveToCones.end().getX(),moveToCones.end().getY(),63))
                .lineToConstantHeading(new Vector2d(50,0))
                .build();
//        Trajectory rotate2L = robot.trajectoryBuilder(goToBoard1L.end())
//                .lineToLinearHeading(new Pose2d(50,0, Math.toRadians(-100)))
//                .build();
        Trajectory goToBoard2L = robot.trajectoryBuilder(new Pose2d(goToBoard1L.end().getX(),goToBoard1L.end().getY(),-90))
                .lineToConstantHeading(new Vector2d(50,-60))
                .build();
        Trajectory alignWithBoard1 = robot.trajectoryBuilder(goToBoard2L.end())
                .lineToConstantHeading(new Vector2d(-15,-75))
                .build();

        Trajectory rotate2 = robot.trajectoryBuilder(capMiddle.end())
                .lineToLinearHeading(new Pose2d(30,0, Math.toRadians(-100)))
                .build();
        Trajectory alignWithBoard2 = robot.trajectoryBuilder(capMiddle.end())
                .lineToConstantHeading(new Vector2d(56,-35))
                .build();
        Trajectory goToBoard2 = robot.trajectoryBuilder(alignWithBoard2.end())
                .forward(28)
                .build();
//        Trajectory park2 = robot.trajectoryBuilder(moveToBoard2.end())
//                .lineTo(new Vector2d(0,-30))
//                .build();

        Trajectory rotate3 = robot.trajectoryBuilder(capRight.end())
                .lineToLinearHeading(new Pose2d(30,0, Math.toRadians(-100)))
                .build();
        Trajectory goToBoard3 = robot.trajectoryBuilder(rotate3.end())
                .lineToConstantHeading(new Vector2d(50,-35))
                .build();
//        Trajectory park3 = robot.trajectoryBuilder(moveToBoard3.end())
//                .lineTo(new Vector2d(0,-30))
//                .build();


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
        /*if (robot.isDistClose(10)) {
            telemetry.addData("dist",robot.getDist());
            telemetry.update();
            objFound = true;
            objPos = 2;
            robot.setPivot(0.1);//lower pixarm
            sleep(500);
            robot.followTrajectory(capMiddle);//push team prop
            sleep(100);
            robot.setLeftGrabber(0.61);//drop pixel
            sleep(150);
            robot.setPivot(0.6);//raise pixarm
            sleep(50);
            robot.setLeftGrabber(0.37);
            sleep(500);
        }*/

        if (!objFound) {
            robot.turnDegrees(42);
            telemetry.addData("dist",robot.getDist());
            telemetry.update();
            //if (robot.isDistClose(10)) {
                objFound = true;
                objPos = 1;
                robot.setPivot(0.1);
                sleep(500);
                robot.followTrajectory(capLeft);
                robot.setLeftGrabber(0.61);
                sleep(100);
                robot.setPivot(0.6);
                sleep(50);
                robot.setLeftGrabber(0.37);
                sleep(500);
            //}
        }

        //if object found cap left otherwise cap right
        /*if (!objFound) {
            objPos = 3;
            robot.setPivot(0.1);
            sleep(100);
            robot.followTrajectory(capRight);
            robot.setLeftGrabber(0.61);
            sleep(100);
            robot.setPivot(0.6);
            sleep(50);
            robot.setLeftGrabber(0.37);
            sleep(500);
        }*/

        //go forward to clear pole
        if (objPos == 1) {
            robot.turnDegrees(0);
            robot.followTrajectory(goToBoard1L);
            robot.setIsRotatedStartDeg(robot.getYaw());
            robot.turnDegrees(-90);
            robot.followTrajectory(goToBoard2L);
            robot.setArmPos(1);
            robot.setPivot(0.24);
            robot.followTrajectory(alignWithBoard1);
            robot.setRightGrabber(0.7);
            sleep(100);
            robot.setPivot(0.6);
            robot.setArmPos(0);
            sleep(100);
            robot.manualMotorPower(0.4,-1.0,-1.0,0.4);
            sleep(2500);
            robot.manualMotorPower(1.0,-1.0,-1.0,1.0);
            sleep(1000);
            robot.manualMotorPower(0,0,0,0);
            robot.setArmPos(-1);
        }

        if (objPos == 2) {
            robot.followTrajectoryAsync(rotate2);
            robot.update();
            double initYaw = robot.getYaw();
            robot.setIsRotatedStartDeg(robot.getYaw());
            while (!robot.isRotated(-72)) {//-78
                robot.update();
                telemetry.addData("initYaw", initYaw);
                telemetry.addData("yaw", robot.getYaw());
                telemetry.update();
            }
            robot.breakFollowing();
            robot.followTrajectory(alignWithBoard2);
            robot.setArmPos(1);
            robot.setPivot(0.27);
            //while(robot.getDist() > 5)
            //    robot.manualMotorPower(1.0,1.0,1.0,1.0);
            //robot.manualMotorPower(0,0,0,0);
            robot.followTrajectory(goToBoard2);
            robot.setRightGrabber(0.7);
            sleep(100);
            robot.setPivot(0.6);
            robot.setArmPos(0);
            sleep(100);
            //robot.followTrajectory(park1);
            robot.manualMotorPower(-.4,-.4,-.4,-.4);
            sleep(100);
            robot.manualMotorPower(0.9,-1.0,-1.0,0.9);
            sleep(2600);
            robot.manualMotorPower(0,0,0,0);
            robot.setArmPos(-1);
        }
        if (objPos == 3) {
            robot.followTrajectoryAsync(rotate3);
            robot.update();
            double initYaw = robot.getYaw();
            robot.setIsRotatedStartDeg(robot.getYaw());
            while (!robot.isRotated(-76)) {
                robot.update();
                telemetry.addData("initYaw", initYaw);
                telemetry.addData("yaw", robot.getYaw());
                telemetry.update();
            }
            robot.breakFollowing();
            robot.setArmPos(1);
            robot.setPivot(0.24);
            robot.followTrajectory(goToBoard3);
            robot.setRightGrabber(0.7);
            sleep(100);
            robot.setPivot(0.6);
            robot.setArmPos(0);
            sleep(100);
            //robot.followTrajectory(park1);
            robot.manualMotorPower(0.9,-1.0,-1.0,0.9);
            sleep(1700);
            robot.manualMotorPower(0,0,0,0);
            robot.setArmPos(-1);
        }

        sleep(3000);
    }
}