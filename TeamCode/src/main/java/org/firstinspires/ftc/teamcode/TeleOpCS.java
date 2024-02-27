package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.ArrayList;
import java.util.List;

@TeleOp(group="drive")
public class TeleOpCS extends OpMode {
    /*
    private DcMotorEx frontLeft, frontRight, backLeft, backRight;
    private DcMotor liftMotor1, liftMotor2, pixArm;
    private Servo leftGrabber, rightGrabber, leftGrabberPivot, rightGrabberPivot, droneLauncher;
    private double rightGrabPos, leftGrabPos, pivotPos, dronePos, speedMultiplier; //for Grabber Arm
    private boolean leftGrabberToggle, rightGrabberToggle, droneToggle, pivotToggle, armToggle1, armToggle2, speedToggle = true;
    private final double pos = 0.0;
    private ElapsedTime rightGrabTimer, leftGrabTimer, droneTimer, pivotTimer, armTimer, speedTimer;
     */
    private TfodProcessor objectProcessor;
    private AprilTagProcessor QRProcessor;
    private VisionPortal webcam;

    /*private int armValue = 0;
    private void armToHeightEncoders(int level, double power) {
        if (level == 0) {
            armValue = 0;
        } else if (level == 1) {
            armValue = 75;
        } else if (level == 2) {
            armValue = 161;
        }

        pixArm.setTargetPosition(armValue);

        pixArm.setPower(power);
    }
    private void armToHeightEncoders(int level) {
        if (level == 0) {
            armValue = 0;
        } else if (level == 1) {
            armValue = 75;
        } else if (level == 2) {
            armValue = 161;
        } else if (level > -1) {
            armValue += 1;
        } else {
            armValue -= 1;
        }
        pixArm.setTargetPosition(armValue);

        pixArm.setPower(0.7);
    }*/

    public void init() {
        // Motors
        /*rightGrabTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        leftGrabTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        droneTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        pivotTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        armTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        speedTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

        frontLeft = hardwareMap.get(DcMotorEx.class, "tl"); //top-left
        frontRight = hardwareMap.get(DcMotorEx.class, "tr"); //top-right
        backLeft = hardwareMap.get(DcMotorEx.class, "bl"); //back-left
        backRight = hardwareMap.get(DcMotorEx.class, "br"); //back-right

        frontLeft.setDirection(DcMotorEx.Direction.REVERSE);
        backLeft.setDirection(DcMotorEx.Direction.REVERSE);

        frontLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        speedMultiplier = 1.00;

        liftMotor1 = hardwareMap.get(DcMotor.class, "lm1"); //lift-motor-1 LEFT
        liftMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        liftMotor2 = hardwareMap.get(DcMotor.class, "lm2"); //lift-motor-2 RIGHT
        liftMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        pixArm = hardwareMap.get(DcMotor.class, "arm");
        pixArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pixArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pixArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        pixArm.setTargetPosition(0);
        pixArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Servo
        leftGrabber = hardwareMap.get(Servo.class, "grabLeft");
        rightGrabber = hardwareMap.get(Servo.class, "grabRight");
        rightGrabPos = 1.00;
        leftGrabPos = 0.37;

        leftGrabberPivot = hardwareMap.get(Servo.class, "grabPivotLeft");
        rightGrabberPivot = hardwareMap.get(Servo.class, "grabPivotRight");
        pivotPos = 0.57;

        droneLauncher = hardwareMap.get(Servo.class, "plane");
        dronePos = 0.86;*/

        // Vision
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

    public void loop() {
        // Drive
        /*float yPosLeft01 = -gamepad1.left_stick_y;
        float yPosRight01 = -gamepad1.right_stick_y;
        float yPosLeft02 = -gamepad2.left_stick_y;
        float yPosRight02 = -gamepad2.right_stick_y;

        if(gamepad1.right_trigger > 0) {
            frontLeft.setPower(gamepad1.right_trigger*speedMultiplier);
            frontRight.setPower(-gamepad1.right_trigger*speedMultiplier);
            backLeft.setPower(-gamepad1.right_trigger*speedMultiplier);
            backRight.setPower(gamepad1.right_trigger*speedMultiplier);
        } else if(gamepad1.left_trigger > 0) {
            frontLeft.setPower(-gamepad1.left_trigger*speedMultiplier);
            frontRight.setPower(gamepad1.left_trigger*speedMultiplier);
            backLeft.setPower(gamepad1.left_trigger*speedMultiplier);
            backRight.setPower(-gamepad1.left_trigger*speedMultiplier);
        } else {
            frontLeft.setPower(yPosLeft01*speedMultiplier);
            frontRight.setPower(yPosRight01*speedMultiplier);
            backLeft.setPower(yPosLeft01*speedMultiplier);
            backRight.setPower(yPosRight01*speedMultiplier);
        }

        if (speedToggle && gamepad1.y && speedTimer.milliseconds()>500){
            //slow
            speedMultiplier = 0.5;
            speedToggle = false;
            speedTimer.reset();
        } else if (!speedToggle && gamepad1.y && speedTimer.milliseconds()>500) {
            //normal
            speedMultiplier = 1.00;
            speedToggle = true;
            speedTimer.reset();
        }

        // Lift
        liftMotor1.setPower(yPosLeft02);
        liftMotor2.setPower(yPosRight02);


        //if (yPosLeft02 > 0.5) { //TESTER FOR SERVO VALUES
        //    pivotPos += 0.01;
        //}
        //if (yPosLeft02 < -0.5) {
        //    pivotPos -= 0.01;
        //}


        // Pixel Arm
        if (armToggle1 && gamepad2.x && armTimer.milliseconds()>1000){
            //up med
            armToHeightEncoders(1,0.7);
            pivotPos = 0.24;
            armToggle1 = false;
            armToggle2 = true;
            armTimer.reset();
        } else if (armToggle2 && gamepad2.y && armTimer.milliseconds()>1000){
            //up high
            armToHeightEncoders(2,0.7);
            pivotPos = 0.15;
            armToggle1 = true;
            armToggle2 = false;
            armTimer.reset();
        } else if ((!armToggle1 || !armToggle2) && (gamepad2.x || gamepad2.y) && armTimer.milliseconds()>500) {
            //down
            armToHeightEncoders(0,0.5);
            pivotPos = 0.6;
            armToggle1 = true;
            armToggle2 = true;
            armTimer.reset();
        }
        if (5 > Math.abs(pixArm.getCurrentPosition()-armValue) && pixArm.getCurrentPosition() > 50) {
            armToHeightEncoders(-1, 0.5);//Level -1 doesn't change liftValue, sets power to 0
        } else if(5 > Math.abs(pixArm.getCurrentPosition()-armValue)) {
            armToHeightEncoders(-1,0);
        }


        // Grabber
        if (rightGrabberToggle && gamepad1.right_bumper && rightGrabTimer.milliseconds()>500){
            //open
            rightGrabPos = 0.70;
            rightGrabberToggle = false;
            rightGrabTimer.reset();
        } else if (!rightGrabberToggle && gamepad1.right_bumper && rightGrabTimer.milliseconds()>500) {
            //closed
            rightGrabPos = 1.00;
            rightGrabberToggle = true;
            rightGrabTimer.reset();
        }
        rightGrabber.setPosition(rightGrabPos);

        if (leftGrabberToggle && gamepad1.left_bumper && leftGrabTimer.milliseconds()>500){
            //open
            leftGrabPos = 0.61;
            leftGrabberToggle = false;
            leftGrabTimer.reset();
        } else if (!leftGrabberToggle && gamepad1.left_bumper && leftGrabTimer.milliseconds()>500) {
            //closed
            leftGrabPos = 0.37;
            leftGrabberToggle = true;
            leftGrabTimer.reset();
        }
        leftGrabber.setPosition(leftGrabPos);

        if (pivotToggle && gamepad2.a && pivotTimer.milliseconds()>500){
            //down
            pivotPos = 0.1;
            pivotToggle = false;
            pivotTimer.reset();
        } else if (!pivotToggle && gamepad2.a && pivotTimer.milliseconds()>500) {
            //up
            pivotPos = 0.6;
            pivotToggle = true;
            pivotTimer.reset();
        }
        rightGrabberPivot.setPosition(pivotPos);
        leftGrabberPivot.setPosition(1-pivotPos);

        //Drone Launcher
        if (droneToggle && gamepad1.x && droneTimer.milliseconds()>500) {
            //open
            dronePos = 0.5;
            droneToggle = false;
            droneTimer.reset();
        } else if (!droneToggle && gamepad1.x && droneTimer.milliseconds()>500) {
            //closed
            dronePos = 0.86;
            droneToggle = true;
            droneTimer.reset();
        }
        droneLauncher.setPosition(dronePos);*/

//        telemetry.addData("LiftPos1", liftMotor1.getCurrentPosition());
//        telemetry.addData("LiftPos2", liftMotor2.getCurrentPosition());
//        telemetry.addData("Encoder Diff", Math.abs(liftMotor1.getCurrentPosition()-liftMotor2.getCurrentPosition()));
//        telemetry.addData("Arm target", armValue);
//        telemetry.addData("difference (5)",pixArm.getCurrentPosition()-armValue);
//        telemetry.addData("Arm Encoder", pixArm.getCurrentPosition());
//        telemetry.addData("busy", pixArm.isBusy());
//        telemetry.addData("frontLeft",frontLeft.getCurrentPosition());
//        telemetry.addData("frontRight",frontRight.getCurrentPosition());
//        telemetry.addData("backLeft",backLeft.getCurrentPosition());
//        telemetry.addData("backRight",backRight.getCurrentPosition());
//        telemetry.addData("pivotPos",rightGrabberPivot.getPosition());
//        telemetry.addData("pixArm",pixArm.getCurrentPosition());
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
        telemetry.addData("pos", (int) Range.clip(Math.round(total / numList.size()),1,3));
        telemetry.update();
    }
}