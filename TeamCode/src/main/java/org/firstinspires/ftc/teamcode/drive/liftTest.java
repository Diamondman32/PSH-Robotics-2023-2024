package org.firstinspires.ftc.teamcode.drive;

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

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(group="drive")
public class liftTest extends OpMode {
    private DcMotor liftMotor1, liftMotor2;
    private double liftMultiplier1, liftMultiplier2 = 1;

    public void liftToHeightEncoders(int level, double power){
        liftMotor1.setPower(0);
        liftMotor2.setPower(0);
        int liftValue = 0;
        if (level == 2) {
            liftValue = -1200;
        } else if (level == 3) {
            liftValue = -3800;
        } else if (level == 4) {
            liftValue = -4000;
        }

        liftMotor1.setTargetPosition(-liftValue);
        liftMotor2.setTargetPosition(liftValue);
        liftMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        liftMotor1.setPower(power);
        liftMotor2.setPower(power);

        while (liftMotor1.isBusy()||liftMotor2.isBusy()){
            telemetry.addData("Left Lift Encoder Position", liftMotor1.getCurrentPosition());
            telemetry.addData("Right Lift Encoder Position", liftMotor2.getCurrentPosition());
            telemetry.update();
        }

        liftMotor1.setPower(0);
        liftMotor2.setPower(0);
        liftMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void init() {
        liftMotor1 = hardwareMap.get(DcMotor.class, "lm1"); //lift-motor-1 LEFT
        liftMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        liftMotor2 = hardwareMap.get(DcMotor.class, "lm2"); //lift-motor-2 RIGHT
        liftMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftMultiplier1 = 1.00;
        liftMultiplier2 = 1.00;
    }

    public void loop() {
        // Lift
        if (gamepad2.right_trigger > 0.03) {
            liftMotor1.setPower(gamepad2.right_trigger);
            liftMotor2.setPower(gamepad2.right_trigger);
        } else if (gamepad2.left_trigger > 0.03) {
            liftMotor1.setPower(-1 * gamepad2.left_trigger);
            liftMotor2.setPower(-1 * gamepad2.left_trigger);
        } else {
            liftMotor1.setPower(0);
            liftMotor2.setPower(0);
        }

        telemetry.addData("LiftPos1", liftMotor1.getCurrentPosition());
        telemetry.addData("LiftPos2", liftMotor2.getCurrentPosition());
        telemetry.update();
    }
}