package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;
import org.jetbrains.annotations.NotNull;

public class ConnectedDevices {
    private final Servo leftGrabber, rightGrabber, leftGrabberPivot, rightGrabberPivot;
    private final DcMotorEx pixArm;
    private final VisionPortal webcam;
    private final TfodProcessor objectProcessor;
    private final AprilTagProcessor QRProcessor;

    public ConnectedDevices(HardwareMap hardwareMap) {
        // pixArm Motor
        pixArm = hardwareMap.get(DcMotorEx.class, "arm");
        pixArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pixArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pixArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        pixArm.setTargetPosition(0);
        pixArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Servos
        leftGrabber = hardwareMap.get(Servo.class, "grabLeft");
        rightGrabber = hardwareMap.get(Servo.class, "grabRight");

        leftGrabberPivot = hardwareMap.get(Servo.class, "grabPivotLeft");
        rightGrabberPivot = hardwareMap.get(Servo.class, "grabPivotRight");

        //   Image Processing
        // QR Code Processor
        QRProcessor = AprilTagProcessor.easyCreateWithDefaults();

        // TensorFlow Processor
        objectProcessor = new TfodProcessor.Builder()
                .setMaxNumRecognitions(1)
                .setUseObjectTracker(true)
                .build();

        // Webcam
        webcam = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(QRProcessor)
                .addProcessor(objectProcessor)
                .enableLiveView(true)
                .setAutoStopLiveView(false)
                .build();
    }

    //------------------------------------------------------------------------------------------------------------------
    private int level = 0;
    private int armValue = 0;
    public class SetArmPos implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            double power;
            if (level == 0) {
                armValue = 0;
                power = 0.5;
            } else if (level == 1) {
                armValue = 75;
                power = 0.7;
            } else if (level == 2) {
                armValue = 161;
                power = 0.7;
            } else {
                power = 0;
            }

            pixArm.setTargetPosition(armValue);
            pixArm.setPower(power);
            return false;
        }
    }
    public Action setArmPos(int level) {
        this.level = level;
        return new SetArmPos();
    }

    //------------------------------------------------------------------------------------------------------------------
    private double pos;
    private class SetPivot implements Action {
        @Override
        public boolean run(@NotNull TelemetryPacket telemetryPacket) {
            rightGrabberPivot.setPosition(pos);
            leftGrabberPivot.setPosition(1-pos);
            return false;
        }
    }
    public Action setPivot(double pos) {
        this.pos = pos;
        return new SetPivot();
    }

    //------------------------------------------------------------------------------------------------------------------
    private boolean rightGrabberPos;
    private class SetRightGrabber implements Action {
        @Override
        public boolean run(@NotNull TelemetryPacket telemetryPacket) {
            if (rightGrabberPos)
                rightGrabber.setPosition(.37); //Close
            else
                rightGrabber.setPosition(.61); //Open
            return false;
        }
    }
    public Action setRightGrabber(boolean rightGrabberPos) {
        this.rightGrabberPos = rightGrabberPos;
        return new SetRightGrabber();
    }

    //------------------------------------------------------------------------------------------------------------------
    private boolean leftGrabberPos;
    private class SetLeftGrabber implements Action {
        @Override
        public boolean run(@NotNull TelemetryPacket telemetryPacket) {
            if (leftGrabberPos)
                leftGrabber.setPosition(.37); //Close
            else
                leftGrabber.setPosition(.61); //Open
            return false;
        }
    }
    public Action setLeftGrabber(boolean leftGrabberPos) {
        this.leftGrabberPos = leftGrabberPos;
        return new SetLeftGrabber();
    }
    //------------------------------------------------------------------------------------------------------------------
    private class Example implements Action {
        @Override
        public boolean run(@NotNull TelemetryPacket telemetryPacket) {

            return false;
        }
    }
    public Action example() {
        return new Example();
    }
}
/*
public double getYaw() {
    return lazyImu.get().getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
}
public int getObjectPosition() {
    List<Recognition> recognitions = objectProcessor.getRecognitions();
    return 1;
}
public void manualMotorPower(double frontLeft, double frontRight, double backLeft, double backRight) { // THIS MIGHT MESS UP ROADRUNNER
    leftFront.setPower(frontLeft);
    leftBack.setPower(backLeft);
    rightFront.setPower(frontRight);
    rightBack.setPower(backRight);
}
public void moveToAprilTag(int pos) {
    final double DESIRED_DISTANCE = 12.0; //  this is how close the camera should get to the target (inches)
    //  Set the GAIN constants to control the relationship between the measured position error, and how much power is
    //  applied to the drive motors to correct the error.
    //  Drive = Error * Gain    Make these values smaller for smoother control, or larger for a more aggressive response.
    final double SPEED_GAIN  =  0.02  ;   //  Forward Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    final double STRAFE_GAIN =  0.015 ;   //  Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error.   (0.25 / 25.0)
    final double TURN_GAIN   =  0.01  ;   //  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

    final double MAX_AUTO_SPEED = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_STRAFE= 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_TURN  = 0.3;   //  Clip the turn speed to this max value (adjust for your robot)

    final int DESIRED_TAG_ID = pos;

    while (true) {
        boolean targetFound = false;
        double x, y, yaw = 0;
        AprilTagDetection desiredTag = null;

        List<AprilTagDetection> currentDetections = QRProcessor.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            // Look to see if we have size info on this tag.
            if (detection.metadata != null) {
                //  Check to see if we want to track towards this tag.
                if ((DESIRED_TAG_ID < 0) || (detection.id == DESIRED_TAG_ID)) {
                    // Yes, we want to use this tag.
                    targetFound = true;
                    desiredTag = detection;
                    break;  // don't look any further.
                } else {
                    // This tag is in the library, but we do not want to track it right now.
                    telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id);
                }
            } else {
                // This tag is NOT in the library, so we don't have enough information to track to it.
                telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", detection.id);
            }
        }

        // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
        double rangeError = (desiredTag.ftcPose.range - DESIRED_DISTANCE);
        double headingError = desiredTag.ftcPose.bearing;
        double yawError = desiredTag.ftcPose.yaw;

        // Use the speed and turn "gains" to calculate how we want the robot to move.
        x = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
        y = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);
        yaw = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);

        double leftFrontPower    =  x -y -yaw;
        double rightFrontPower   =  x +y +yaw;
        double leftBackPower     =  x +y -yaw;
        double rightBackPower    =  x -y +yaw;

        // Normalize wheel powers to be less than 1.0
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        // Send powers to the wheels.
        manualMotorPower(leftFrontPower, rightFrontPower, leftBackPower, rightBackPower);

        // Exit condition
        if (desiredTag.ftcPose.range < DESIRED_DISTANCE) {
            break;
        }
    }
    manualMotorPower(0,0,0,0);
}
 */