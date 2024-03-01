package org.firstinspires.ftc.teamcode;

import static java.lang.Thread.sleep;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.NullAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;
import org.jetbrains.annotations.NotNull;

import java.util.ArrayList;
import java.util.List;

public class ConnectedDevices {
    private final Servo leftGrabber, rightGrabber, leftGrabberPivot, rightGrabberPivot;
    private final DcMotorEx pixArm;
    private final TfodProcessor objectProcessor;
    private final AprilTagProcessor QRProcessor;
    private final VisionPortal webcam;
    private final MecanumDrive robot;
    private ElapsedTime timer;

    public ConnectedDevices(HardwareMap hardwareMap, MecanumDrive robot) {
        //Making robot accessible in Action Classes
        this.robot = robot;

        //Timer
        timer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);

        // pixArm
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
        //QRProcessor = AprilTagProcessor.easyCreateWithDefaults();
        QRProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .build();

        // TensorFlow Processor
        objectProcessor = new TfodProcessor.Builder()
                .setModelAssetName("CenterStageModel.tflite")
                .setMaxNumRecognitions(1)
                .setUseObjectTracker(true)
                .setModelLabels(new String[]{"prop"})
                .build();

        // Webcam
        webcam = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(QRProcessor)
                .addProcessor(objectProcessor)
                .enableLiveView(true)
                .setAutoStopLiveView(false)
                .build();
        webcam.setProcessorEnabled(QRProcessor,false);
    }

    //------------------------------------------------------------------------------------------------------------------
    /*
    TODO:
        The static Action classes can only access non-static variables once. So either these classes
        can only run once or a lot of static stuff has to happen.
     */

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
                armValue = 400;
                power = 0.7;
            } else if (level == 2) {
                armValue = 728;
                power = 0.7;
            } else if (level == 3) {
                armValue = 983;
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
    private class SetPivotDown implements Action {
        @Override
        public boolean run(@NotNull TelemetryPacket telemetryPacket) {
            rightGrabberPivot.setPosition(0.03);
            leftGrabberPivot.setPosition(1-0.03);
            return false;
        }
    }
    public Action setPivotDown() {
        return new SetPivotDown();
    }

    //------------------------------------------------------------------------------------------------------------------
    private class SetPivotMed implements Action {
        @Override
        public boolean run(@NotNull TelemetryPacket telemetryPacket) {
            rightGrabberPivot.setPosition(0.2);
            leftGrabberPivot.setPosition(1-0.25);
            return false;
        }
    }
    public Action setPivotMed() {
        return new SetPivotMed();
    }

    //------------------------------------------------------------------------------------------------------------------
    private class SetPivotHigh implements Action {
        @Override
        public boolean run(@NotNull TelemetryPacket telemetryPacket) {
            rightGrabberPivot.setPosition(0.53);
            leftGrabberPivot.setPosition(1-0.53);
            return false;
        }
    }
    public Action setPivotHigh() {
        return new SetPivotHigh();
    }

    //------------------------------------------------------------------------------------------------------------------
    public class OpenRightGrabber implements Action {
        @Override
        public boolean run(@NotNull TelemetryPacket telemetryPacket) {
            rightGrabber.setPosition(0.70); //Open
            return false;
        }
    }
    public Action openRightGrabber() {
        return new OpenRightGrabber();
    }

    //------------------------------------------------------------------------------------------------------------------
    public class CloseRightGrabber implements Action {
        @Override
        public boolean run(@NotNull TelemetryPacket telemetryPacket) {
            rightGrabber.setPosition(1.00); //Close
            return false;
        }
    }
    public Action closeRightGrabber() {
        return new CloseRightGrabber();
    }

    //------------------------------------------------------------------------------------------------------------------
    public class OpenLeftGrabber implements Action {
        @Override
        public boolean run(@NotNull TelemetryPacket telemetryPacket) {
            leftGrabber.setPosition(.61); //Open
            return false;
        }
    }
    public Action openLeftGrabber() {
        return new OpenLeftGrabber();
    }

    //------------------------------------------------------------------------------------------------------------------
    public class CloseLeftGrabber implements Action {
        @Override
        public boolean run(@NotNull TelemetryPacket telemetryPacket) {
            leftGrabber.setPosition(.37); //Close
            return false;
        }
    }
    public Action closeLeftGrabber() {
        return new CloseLeftGrabber();
    }

    public Action wait1() {
        return new SleepAction(1);
    }

    //------------------------------------------------------------------------------------------------------------------
    public int getObjectPosition() {
        while (timer.seconds()<4) { //Give time to camera from init to now to start up
            //>:(
        }
        ArrayList<Integer> numList = new ArrayList<>();
        List<Recognition> recognitions = null;
        for (int i=0;i<10;i++) {
            recognitions = objectProcessor.getRecognitions();
            for (Recognition detection : recognitions) {
                float center = (detection.getLeft() + detection.getRight()) / 2;
                if (detection.getConfidence() > 0.8 && center < 357.5)
                    numList.add(1);
                else if (detection.getConfidence() > 0.8 && center > 357.5)
                    numList.add(2);
            }
        }
        if (recognitions.isEmpty()) // Position 3 is out of camera view
            numList.add(3);

        double total = 0;
        for (int x : numList)
            total += x;
        return (int) Range.clip(Math.round(total / numList.size()),1,3);
    }

    //------------------------------------------------------------------------------------------------------------------
    // THIS MIGHT MESS UP ROADRUNNER
    public void moveToAprilTag(int DESIRED_TAG_ID) {
        final double DESIRED_DISTANCE = 12.0; //  this is how close the camera should get to the target (inches)
        //  Set the GAIN constants to control the relationship between the measured position error, and how much power is
        //  applied to the drive motors to correct the error.
        //  Drive = Error * Gain    Make these values smaller for smoother control, or larger for a more aggressive response.
        final double SPEED_GAIN  =  0.02  ;   //  Forward Speed Control "Gain". eg: Ramp up to 50% power at a 25-inch error.   (0.50 / 25.0)
        final double STRAFE_GAIN =  0.015 ;   //  Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error.   (0.25 / 25.0)
        final double TURN_GAIN   =  0.01  ;   //  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

        final double MAX_AUTO_SPEED = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
        final double MAX_AUTO_STRAFE= 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
        final double MAX_AUTO_TURN  = 0.3;   //  Clip the turn speed to this max value (adjust for your robot)

        while (true) {
            double x, y, yaw;
            AprilTagDetection desiredTag;

            While: while (true) { // NOTE: IF THE APRIL TAG PROCESSOR CANT FIND THE APRIL TAG, IT WILL BE STUCK IN THE LOOP FOREVER
                List<AprilTagDetection> currentDetections = QRProcessor.getDetections();
                for (AprilTagDetection detection : currentDetections) {
                    // Look to see if we have size info on this tag.
                    if (detection.metadata != null) {
                        //  Check to see if we want to track towards this tag.
                        if ((DESIRED_TAG_ID < 0) || (detection.id == DESIRED_TAG_ID)) {
                            // Yes, we want to use this tag.
                            desiredTag = detection;
                            break While;  // don't look any further.
                        }
                    }
                }
            }

            // Determine heading, range and Yaw (tag image rotation) error, so we can use them to control the robot automatically.
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
            robot.setMotorPower(leftFrontPower, rightFrontPower, leftBackPower, rightBackPower);

            // Exit condition
            if (desiredTag.ftcPose.range < DESIRED_DISTANCE) {
                break;
            }
        }
        robot.setMotorPower(0,0,0,0);
    }

    //------------------------------------------------------------------------------------------------------------------
    public void setAprilTagEnabled(boolean enabled) {
        webcam.setProcessorEnabled(QRProcessor, enabled);
    }

    //------------------------------------------------------------------------------------------------------------------

    public void setTFODEnabled(boolean enabled) {
        webcam.setProcessorEnabled(objectProcessor, enabled);
    }

    //------------------------------------------------------------------------------------------------------------------
    public double getYaw() {
        return robot.lazyImu.get().getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    //------------------------------------------------------------------------------------------------------------------
}