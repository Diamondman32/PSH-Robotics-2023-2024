package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.ArrayList;
import java.util.List;

@TeleOp(group="drive")
public class TeleOpCS extends OpMode {

    private DcMotorEx frontLeft, frontRight, backLeft, backRight;
    private DcMotor liftMotor1, liftMotor2, pixArm, armExtend;
    private Servo leftGrabber, rightGrabber, leftGrabberPivot, rightGrabberPivot, droneLauncher;
    private double rightGrabPos, leftGrabPos, pivotPos, dronePos, speedMultiplier;
    private boolean leftGrabberToggle, rightGrabberToggle, droneToggle, pivotToggle,
            armToggle1, armToggle2, armToggle3, speedToggle, increasing, decreasing;
    private double pos = 0.0;
    private ElapsedTime rightGrabTimer, leftGrabTimer, droneTimer, pivotTimer, armTimer, speedTimer;
    private TfodProcessor objectProcessor;
    private VisionPortal webcam;

    private int armValue = 0;
    private void armToHeightEncoders(int level, double power) {
        if (level == 0)
            armValue = 0;
        else if (level == 1)
            armValue = 400;
        else if (level == 2)
            armValue = 728;
        else if (level == 3)
            armValue = 983;

        pixArm.setTargetPosition(armValue);
        pixArm.setPower(power);
    }
    public float getObjectCenter() {
        ArrayList<Integer> numList = new ArrayList<>();
        List<Recognition> recognitions = null;
        recognitions = objectProcessor.getRecognitions();
        for (Recognition recognition : recognitions) {
            return (recognition.getLeft()+recognition.getRight())/2;
        }
        return 0;
    }

    public void init() {
        objectProcessor = new TfodProcessor.Builder()
                .setModelAssetName("CenterStageModel.tflite")
                .setMaxNumRecognitions(1)
                .setUseObjectTracker(true)
                .setModelLabels(new String[]{"prop"})
                .build();

        // Webcam
        webcam = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(objectProcessor)
                .enableLiveView(true)
                .setAutoStopLiveView(false)
                .build();        // Motors
        rightGrabTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        leftGrabTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        droneTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        pivotTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        armTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        speedTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

        //Toggles
        leftGrabberToggle = true;
        rightGrabberToggle = true;
        droneToggle = true;
        pivotToggle = true;
        armToggle1 = true;
        armToggle2 = true;
        armToggle3 = true;
        speedToggle = true;
        increasing = false;
        decreasing = false;

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

        armExtend = hardwareMap.get(DcMotor.class, "armSlide");
        armExtend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armExtend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armExtend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Servo
        leftGrabber = hardwareMap.get(Servo.class, "grabLeft");
        rightGrabber = hardwareMap.get(Servo.class, "grabRight");
        rightGrabPos = 1.00;
        leftGrabPos = 0.37;

        leftGrabberPivot = hardwareMap.get(Servo.class, "grabPivotLeft");
        rightGrabberPivot = hardwareMap.get(Servo.class, "grabPivotRight");
        pivotPos = 0.53;

        droneLauncher = hardwareMap.get(Servo.class, "plane");
        dronePos = 0.90;
    }

    public void loop() {
        telemetry.addData("center", getObjectCenter());
        // Drive
        float yPosLeft01 = -gamepad1.left_stick_y;
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


//        if (gamepad1.dpad_up) { //TESTER FOR SERVO VALUES
//            pos += 0.01;
//        }
//        if (gamepad1.dpad_down) {
//            pos -= 0.01;
//        }
//        telemetry.addData("drone", pos);


        // Pixel Arm
        if (armToggle1 && gamepad2.a && armTimer.milliseconds()>1000){
            //up med
            armToHeightEncoders(1,1);
            pivotPos = 0.27;
            armToggle1 = false; //false when at specified level, true otherwise
            armToggle2 = true;
            armToggle3 = true;
            armTimer.reset();
        } else if (armToggle2 && gamepad2.x && armTimer.milliseconds()>1000) {
            //up high
            armToHeightEncoders(2, 1);
            pivotPos = 0.18;
            armToggle1 = true;
            armToggle2 = false;
            armToggle3 = true;
            armTimer.reset();
        } else if (armToggle3 && gamepad2.y && armTimer.milliseconds()>1000) {
            armToHeightEncoders(3, 1);
            pivotPos = 0.03;
            armToggle1 = true;
            armToggle2 = true;
            armToggle3 = false;
            armTimer.reset();
        } else if ((!armToggle1 || !armToggle2 || !armToggle3)
                && (gamepad2.x || gamepad2.y || gamepad2.a)
                && armTimer.milliseconds()>500) {
            //down
            armToHeightEncoders(0,0.5);
            pivotPos = 0.3;
            armToggle1 = true;
            armToggle2 = true;
            armToggle3 = true;
            armTimer.reset();
        }
        //Slow power when at destination, if at destination near ground turn off power
        //Level -1 doesn't change armValue, sets power to 0
        if (15 > Math.abs(pixArm.getCurrentPosition()-armValue) && pixArm.getCurrentPosition() > 50)
            armToHeightEncoders(-1, 0.5);
        else if(pixArm.getCurrentPosition() < 50 && 15 > armValue-pixArm.getCurrentPosition())
            armToHeightEncoders(-1,0);
        telemetry.addData("pixArm", pixArm.getCurrentPosition());

        // Pixel Arm Extender
        //Set Direction
        if (gamepad2.right_bumper && armExtend.getCurrentPosition()<370) {
            armExtend.setPower(1);
            increasing = true;
            decreasing = false;
        } else if (gamepad2.left_bumper && armExtend.getCurrentPosition()>5) {
            armExtend.setPower(-1);
            increasing = false;
            decreasing = true;
        }
        //Stop when min or max is reached
        if (increasing && armExtend.getCurrentPosition()>=370) {
            armExtend.setPower(0);
            increasing = false;
        } else if (decreasing && armExtend.getCurrentPosition()<=5) {
            armExtend.setPower(0);
            decreasing = false;
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
            leftGrabPos = 0.3;
            leftGrabberToggle = true;
            leftGrabTimer.reset();
        }
        leftGrabber.setPosition(leftGrabPos);

        if (pivotToggle && gamepad2.b && pivotTimer.milliseconds()>500){
            //down
            pivotPos = 0.01;
            pivotToggle = false;
            pivotTimer.reset();
        } else if (!pivotToggle && gamepad2.b && pivotTimer.milliseconds()>500) {
            //up
            pivotPos = 0.3;//midpoint.  Highpoint (starting pos) is: .53
            pivotToggle = true;
            pivotTimer.reset();
        }
        rightGrabberPivot.setPosition(pivotPos);
        leftGrabberPivot.setPosition(1-pivotPos);

        //Drone Launcher
        if (droneToggle && gamepad1.x && droneTimer.milliseconds()>500) {
            //open
            dronePos = 0.71;
            droneToggle = false;
            droneTimer.reset();
        } else if (!droneToggle && gamepad1.x && droneTimer.milliseconds()>500) {
            //closed
            dronePos = 0.90;
            droneToggle = true;
            droneTimer.reset();
        }
        droneLauncher.setPosition(dronePos);
    }
}