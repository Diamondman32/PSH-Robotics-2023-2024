package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "AutoCSRedRight")
@Config
public class AutoCSRedRight extends LinearOpMode {
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
                .lineToConstantHeading(new Vector2d(31,0))//28
                .build();
        Trajectory checkRight = robot.trajectoryBuilder(moveToCones.end())
                .lineToLinearHeading(new Pose2d(18,2,Math.toRadians(-40)))//-28
                .build();
        Trajectory capLeft = robot.trajectoryBuilder(checkRight.end())
                .lineToLinearHeading(new Pose2d(20,5,Math.toRadians(25)))//32
                .build();
        Trajectory capRight = robot.trajectoryBuilder(checkRight.end())
                .forward(10)
                .build();

        Trajectory rotate1 = robot.trajectoryBuilder(capLeft.end())
                .lineToLinearHeading(new Pose2d(30,0, Math.toRadians(-100)))
                .build();
        Trajectory alignWithBoard1 = robot.trajectoryBuilder(rotate1.end())
                .lineToConstantHeading(new Vector2d(65,-35))//90
                .build();
        Trajectory goToBoard1 = robot.trajectoryBuilder(alignWithBoard1.end())
                .forward(30)
                .build();
        //                .lineToConstantHeading(new Vector2d(85,-60))
//        Trajectory park1 = robot.trajectoryBuilder(alignWithBoard1.end())
//                .lineToConstantHeading(new Vector2d(0,10))
//                .build();
//        Trajectory spline = robot.trajectoryBuilder(capLeft.end())
//                .splineToLinearHeading(new Pose2d(x,y,Math.toRadians(heading)),0)
//                .build();

        Trajectory rotate2 = robot.trajectoryBuilder(capMiddle.end())
                .lineToLinearHeading(new Pose2d(30,0, Math.toRadians(-100)))
                .build();
        Trajectory alignWithBoard2 = robot.trajectoryBuilder(rotate2.end())
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
            sleep(500);
        }

        //scan left ~90 deg WHY DOESNT THIS WORK
        /*if (!objFound) { //ASYNC FINDING
            robot.followTrajectoryAsync(checkRight);
            timer.reset();
            while (timer.milliseconds() < 1000 && !objFound) {
                robot.update();
                telemetry.addData("dist", robot.getDist());
                telemetry.update();
                if (robot.isDistClose(12)) {//10
                    robot.breakFollowing();
                    objFound = true;
                    objPos = 3;
                    robot.setPivot(0.1);
                    sleep(500);
                    robot.followTrajectory(capRight);
                    robot.setLeftGrabber(0.61);
                    sleep(100);
                    robot.setPivot(0.6);
                    sleep(50);
                    robot.setLeftGrabber(0.37);
                    sleep(500);
                }
            }
        }*/

        /*if (!objFound) //ASYNC FINDING
            robot.followTrajectoryAsync(checkRight);
        timer.reset();
        while (timer.milliseconds() < 1000 && !objFound) {
            robot.update();
            telemetry.addData("dist",robot.getDist());
            telemetry.update();
                if (robot.isDistClose(15)) {//10
                    robot.breakFollowing();
                    objFound = true;
                    objPos = 3;
                    robot.setPivot(0.1);
                    sleep(500);
                    robot.followTrajectory(capRight);
                    robot.setLeftGrabber(0.61);
                    sleep(100);
                    robot.setPivot(0.6);
                    sleep(50);
                    robot.setLeftGrabber(0.37);
                    sleep(500);
                }
            }*/

        if (!objFound) {
            robot.setIsRotatedStartDeg(0);
            robot.followTrajectoryAsync(checkRight);
            while (!robot.isRotated(-30)) {
                robot.update();
            }
            robot.breakFollowing();
            robot.manualMotorPower(0, 0, 0, 0);
            if (robot.isDistClose(10)) {
                objFound = true;
                objPos = 3;
                robot.setPivot(0.1);
                sleep(500);
                robot.followTrajectory(capRight);
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
            objPos = 1;
            robot.setPivot(0.1);
            sleep(100);
            robot.followTrajectory(capLeft);
            robot.setLeftGrabber(0.61);
            sleep(100);
            robot.setPivot(0.6);
            sleep(50);
            robot.setLeftGrabber(0.37);
            sleep(500);
        }

        //go forward to clear pole
        if (objPos == 1) {
            robot.followTrajectoryAsync(rotate1);
            robot.update();
            double initYaw = robot.getYaw();
            robot.setIsRotatedStartDeg(robot.getYaw());
            while (!robot.isRotated(-78)) {
                robot.update();
                telemetry.addData("initYaw", initYaw);
                telemetry.addData("yaw", robot.getYaw());
                telemetry.update();
            }
            robot.breakFollowing();
            robot.followTrajectory(alignWithBoard1);
            robot.setArmPos(1);
            robot.setPivot(0.24);
            robot.followTrajectory(goToBoard1);
            robot.setRightGrabber(0.7);
            sleep(100);
            robot.setPivot(0.6);
            robot.setArmPos(0);
            sleep(100);
            //robot.followTrajectory(park1);
            robot.manualMotorPower(0.9,-1.0,-1.0,0.9);
            sleep(3500);
            robot.manualMotorPower(0,0,0,0);
            robot.setArmPos(-1);
//            sleep(200);
//            robot.manualMotorPower(0,0,0,0);
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