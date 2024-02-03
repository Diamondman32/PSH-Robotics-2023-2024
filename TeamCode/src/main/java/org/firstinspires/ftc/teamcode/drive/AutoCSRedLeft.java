package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;

@Config
@Autonomous(name = "AutoCSRedLeft")
public class AutoCSRedLeft extends LinearOpMode {
    private final ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    public static int x = -5;
    public static int y = -75;

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
        Trajectory capLeft = robot.trajectoryBuilder(new Pose2d(moveToCones.end().getX(),moveToCones.end().getY(),63))
                .forward(1)
                .build();
        Trajectory capRight = robot.trajectoryBuilder(new Pose2d(moveToCones.end().getX(),moveToCones.end().getY(),63))
                .lineToLinearHeading(new Pose2d(20,-5,Math.toRadians(-25)))
                .build();


        Trajectory goToBoard1L = robot.trajectoryBuilder(new Pose2d(moveToCones.end().getX(),moveToCones.end().getY(),63))
                .lineToConstantHeading(new Vector2d(52,0))
                .build();
        Trajectory goToBoard2L = robot.trajectoryBuilder(new Pose2d(goToBoard1L.end().getX(),goToBoard1L.end().getY(),-90))
                .lineToConstantHeading(new Vector2d(52,-60))
                .build();
        Trajectory alignWithBoard1 = robot.trajectoryBuilder(goToBoard2L.end())
                .lineToConstantHeading(new Vector2d(-5,-75))
                .build();
        Trajectory arriveAtBoard1 = robot.trajectoryBuilder(alignWithBoard1.end())
                .forward(15)
                .build();

        Trajectory goToBoard1M = robot.trajectoryBuilder(new Pose2d(capMiddle.end().getX(),capMiddle.end().getY(),45))
                .forward(15)
                .build();
        Trajectory goToBoard2M = robot.trajectoryBuilder(new Pose2d(goToBoard1M.end().getX(),goToBoard1M.end().getY(),-45))
                .lineToConstantHeading(new Vector2d(52,0))
                .build();
        Trajectory goToBoard3M = robot.trajectoryBuilder(new Pose2d(goToBoard2M.end().getX(),goToBoard2M.end().getY(),-90))
                .lineToConstantHeading(new Vector2d(52,-60))
                .build();
        Trajectory alignWithBoard2 = robot.trajectoryBuilder(goToBoard2L.end())
                .lineToConstantHeading(new Vector2d(-25,-75))
                .build();
        Trajectory arriveAtBoard2 = robot.trajectoryBuilder(alignWithBoard1.end())
                .forward(15)
                .build();

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

        //Find hanging Distance Value
        ArrayList<Double> distance = new ArrayList<Double>();
        double averageDist = 0;
        timer.reset();
        while (timer.seconds() < 1) {
            distance.add(robot.getDist());
        }
        for (double dist : distance) {
            averageDist+= dist;
        }
        averageDist /= distance.size();

        //Tighten Grabbers
        robot.setLeftGrabber(0.37);
        robot.setRightGrabber(1.00);

        //Drive to scan
        robot.followTrajectory(moveToCones);
        //scan forward if object cap it
        if (robot.isDistClose(10)) {
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
            robot.turnDegrees(45);
        }

        if (!objFound) {
            robot.turnDegrees(38);
            telemetry.addData("dist",robot.getDist());
            telemetry.update();
            if (robot.isDistClose(10)) {
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
            }
        }

        //if object found cap left otherwise cap right
        if (!objFound) {
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
        }

        if (objPos == 1) {
            robot.turnDegrees(0);
            robot.followTrajectory(goToBoard1L);
            robot.setIsRotatedStartDeg(robot.getYaw());
            robot.turnDegrees(-90);
            robot.followTrajectory(goToBoard2L);
            robot.turnDegrees(-90);
            robot.followTrajectory(alignWithBoard1);
            robot.setArmPos(1);
            robot.setPivot(0.24);
            sleep(500);
            robot.followTrajectory(arriveAtBoard1);
            robot.turnDegrees(-90,3);
            robot.setRightGrabber(0.7);
            sleep(100);
            robot.setPivot(0.6);
            robot.setArmPos(0);
            robot.manualMotorPower(-.2,-.2,-.2,-.2);
            sleep(500);
            robot.manualMotorPower(0,0,0,0);
            /*sleep(100);
            robot.manualMotorPower(0.4,-1.0,-1.0,0.4);
            sleep(1000);
            robot.manualMotorPower(1.0,-1.0,-1.0,1.0);
            sleep(2500);
            robot.manualMotorPower(0,0,0,0);
            robot.setArmPos(-1);

             */
        }

        if (objPos == 2) {
            robot.turnDegrees(45);
            robot.followTrajectory(goToBoard1M);
            robot.turnDegrees(-45);
            robot.followTrajectory(goToBoard2M);
            robot.turnDegrees(-90);
            robot.followTrajectory(goToBoard3M);
            robot.followTrajectory(alignWithBoard2);
            robot.setArmPos(1);
            robot.setPivot(0.27);
            sleep(500);
            robot.followTrajectory(arriveAtBoard2);
            robot.setRightGrabber(0.7);
            sleep(100);
            robot.setPivot(0.6);
            robot.setArmPos(0);
        }
        if (objPos == 3) {
            robot.followTrajectoryAsync(rotate3);
            robot.update();
            double initYaw = robot.getYaw();
            robot.turnDegrees(0);


            //robot.setIsRotatedStartDeg(robot.getYaw());
            /*while (!robot.isRotated(-76)) {
                robot.update();
                telemetry.addData("initYaw", initYaw);
                telemetry.addData("yaw", robot.getYaw());
                telemetry.update();
            }
             */
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