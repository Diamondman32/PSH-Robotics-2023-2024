package org.firstinspires.ftc.teamcode.drive;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.followers.HolonomicPIDVAFollower;
import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceRunner;
import org.firstinspires.ftc.teamcode.util.LynxModuleUtil;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ACCEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ANG_ACCEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ANG_VEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_VEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MOTOR_VELO_PID;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.RUN_USING_ENCODER;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.TRACK_WIDTH;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.encoderTicksToInches;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.kA;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.kStatic;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.kV;
import static java.lang.Thread.sleep;

/*
 * Simple mecanum drive hardware implementation for REV hardware.
 */
@Config
public class Robot extends MecanumDrive {
    public final double pi = Math.PI;
    public static PIDCoefficients TRANSLATIONAL_PID = new PIDCoefficients(0, 0, 0);
    public static PIDCoefficients HEADING_PID = new PIDCoefficients(0, 0, 0);

    public static double LATERAL_MULTIPLIER = 1;

    public static double VX_WEIGHT = 1;
    public static double VY_WEIGHT = 1;
    public static double OMEGA_WEIGHT = 1;

    private final TrajectorySequenceRunner trajectorySequenceRunner;

    private static final TrajectoryVelocityConstraint VEL_CONSTRAINT = getVelocityConstraint(MAX_VEL, MAX_ANG_VEL, TRACK_WIDTH);
    private static final TrajectoryAccelerationConstraint ACCEL_CONSTRAINT = getAccelerationConstraint(MAX_ACCEL);

    private final TrajectoryFollower follower;

    private final DcMotorEx leftFront;
    private final DcMotorEx leftRear;
    private final DcMotorEx rightRear;
    private final DcMotorEx rightFront;
    private final List<DcMotorEx> motors;

    private final IMU imu;
    private final VoltageSensor batteryVoltageSensor;

    private final List<Integer> lastEncPositions = new ArrayList<>();
    private final List<Integer> lastEncVels = new ArrayList<>();

    private final Servo leftGrabber;
    private final Servo rightGrabber;
    private final Servo leftGrabberPivot;
    private final Servo rightGrabberPivot;
    private final DcMotor liftMotor1;
    private final DcMotor liftMotor2;
    private final DcMotor pixArm;
    private final DistanceSensor distanceSensor;

    public Robot(HardwareMap hardwareMap) {
        super(kV, kA, kStatic, TRACK_WIDTH, TRACK_WIDTH, LATERAL_MULTIPLIER);

        follower = new HolonomicPIDVAFollower(TRANSLATIONAL_PID, TRANSLATIONAL_PID, HEADING_PID,
                new Pose2d(0.5, 0.5, Math.toRadians(5.0)), 0.5);

        LynxModuleUtil.ensureMinimumFirmwareVersion(hardwareMap);

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                DriveConstants.LOGO_FACING_DIR, DriveConstants.USB_FACING_DIR));
        imu.initialize(parameters);

        leftFront = hardwareMap.get(DcMotorEx.class, "tl");
        leftRear = hardwareMap.get(DcMotorEx.class, "bl");
        rightRear = hardwareMap.get(DcMotorEx.class, "br");
        rightFront = hardwareMap.get(DcMotorEx.class, "tr");

        motors = Arrays.asList(leftFront, leftRear, rightRear, rightFront);

        for (DcMotorEx motor : motors) {
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(motorConfigurationType);
        }

        if (RUN_USING_ENCODER) {
            //setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if (RUN_USING_ENCODER && MOTOR_VELO_PID != null) {
            //setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, MOTOR_VELO_PID);
        }

        // Setting Motor Directions
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightRear.setDirection(DcMotor.Direction.FORWARD);

        //Initializing lift motors
        liftMotor1 = hardwareMap.dcMotor.get("lm1");
        liftMotor1.setDirection(DcMotor.Direction.FORWARD);
        liftMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        liftMotor2 = hardwareMap.dcMotor.get("lm2");
        liftMotor2.setDirection(DcMotor.Direction.REVERSE);
        liftMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        pixArm = hardwareMap.get(DcMotorEx.class, "arm");
        pixArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pixArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pixArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        pixArm.setTargetPosition(0);
        pixArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //Servos
        leftGrabber = hardwareMap.get(Servo.class, "grabLeft");
        rightGrabber = hardwareMap.get(Servo.class, "grabRight");

        leftGrabberPivot = hardwareMap.get(Servo.class, "grabPivotLeft");
        rightGrabberPivot = hardwareMap.get(Servo.class, "grabPivotRight");

        //Initializing Distance Sensor
        distanceSensor = hardwareMap.get(DistanceSensor.class, "distance");

        List<Integer> lastTrackingEncPositions = new ArrayList<>();
        List<Integer> lastTrackingEncVels = new ArrayList<>();

        // TODO: if desired, use setLocalizer() to change the localization method
        // setLocalizer(new StandardTrackingWheelLocalizer(hardwareMap, lastTrackingEncPositions, lastTrackingEncVels));

        trajectorySequenceRunner = new TrajectorySequenceRunner(
                follower, HEADING_PID, batteryVoltageSensor,
                lastEncPositions, lastEncVels, lastTrackingEncPositions, lastTrackingEncVels
        );
    }

    public void breakFollowing() {
        trajectorySequenceRunner.breakFollowing();
    }
    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose) {
        return new TrajectoryBuilder(startPose, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, boolean reversed) {
        return new TrajectoryBuilder(startPose, reversed, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, double startHeading) {
        return new TrajectoryBuilder(startPose, startHeading, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
    }

    public TrajectorySequenceBuilder trajectorySequenceBuilder(Pose2d startPose) {
        return new TrajectorySequenceBuilder(
                startPose,
                VEL_CONSTRAINT, ACCEL_CONSTRAINT,
                MAX_ANG_VEL, MAX_ANG_ACCEL
        );
    }

    public void turnAsync(double angle) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(
                trajectorySequenceBuilder(getPoseEstimate())
                        .turn(angle)
                        .build()
        );
    }

    public void turn(double angle) {
        turnAsync(angle);
        waitForIdle();
    }

    public void followTrajectoryAsync(Trajectory trajectory) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(
                trajectorySequenceBuilder(trajectory.start())
                        .addTrajectory(trajectory)
                        .build()
        );
    }

    public void followTrajectory(Trajectory trajectory) {
        followTrajectoryAsync(trajectory);
        waitForIdle();
    }

    public void followTrajectorySequenceAsync(TrajectorySequence trajectorySequence) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(trajectorySequence);
    }

    public void followTrajectorySequence(TrajectorySequence trajectorySequence) {
        followTrajectorySequenceAsync(trajectorySequence);
        waitForIdle();
    }

    public Pose2d getLastError() {
        return trajectorySequenceRunner.getLastPoseError();
    }

    public void update() {
        updatePoseEstimate();
        DriveSignal signal = trajectorySequenceRunner.update(getPoseEstimate(), getPoseVelocity());
        if (signal != null) setDriveSignal(signal);
    }

    public void waitForIdle() {
        while (!Thread.currentThread().isInterrupted() && isBusy())
            update();
    }

    public boolean isBusy() {
        return trajectorySequenceRunner.isBusy();
    }

    public void setMode(DcMotor.RunMode runMode) {
        for (DcMotorEx motor : motors) {
            motor.setMode(runMode);
        }
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(zeroPowerBehavior);
        }
    }

    public void setPIDFCoefficients(DcMotor.RunMode runMode, PIDFCoefficients coefficients) {
        PIDFCoefficients compensatedCoefficients = new PIDFCoefficients(
                coefficients.p, coefficients.i, coefficients.d,
                coefficients.f * 12 / batteryVoltageSensor.getVoltage()
        );

        for (DcMotorEx motor : motors) {
            motor.setPIDFCoefficients(runMode, compensatedCoefficients);
        }
    }

    public void setWeightedDrivePower(Pose2d drivePower) {
        Pose2d vel = drivePower;

        if (Math.abs(drivePower.getX()) + Math.abs(drivePower.getY())
                + Math.abs(drivePower.getHeading()) > 1) {
            // re-normalize the powers according to the weights
            double denom = VX_WEIGHT * Math.abs(drivePower.getX())
                    + VY_WEIGHT * Math.abs(drivePower.getY())
                    + OMEGA_WEIGHT * Math.abs(drivePower.getHeading());

            vel = new Pose2d(
                    VX_WEIGHT * drivePower.getX(),
                    VY_WEIGHT * drivePower.getY(),
                    OMEGA_WEIGHT * drivePower.getHeading()
            ).div(denom);
        }

        setDrivePower(vel);
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        lastEncPositions.clear();

        List<Double> wheelPositions = new ArrayList<>();
        for (DcMotorEx motor : motors) {
            int position = motor.getCurrentPosition();
            lastEncPositions.add(position);
            wheelPositions.add(encoderTicksToInches(position));
        }
        return wheelPositions;
    }

    @Override
    public List<Double> getWheelVelocities() {
        lastEncVels.clear();

        List<Double> wheelVelocities = new ArrayList<>();
        for (DcMotorEx motor : motors) {
            int vel = (int) motor.getVelocity();
            lastEncVels.add(vel);
            wheelVelocities.add(encoderTicksToInches(vel));
        }
        return wheelVelocities;
    }

    @Override
    public void setMotorPowers(double v, double v1, double v2, double v3) {
        leftFront.setPower(v);
        leftRear.setPower(v1);
        rightRear.setPower(v2);
        rightFront.setPower(v3);
    }

    @Override
    public double getRawExternalHeading() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }

    @Override
    public Double getExternalHeadingVelocity() {
        return (double) imu.getRobotAngularVelocity(AngleUnit.RADIANS).zRotationRate;
    }

    public static TrajectoryVelocityConstraint getVelocityConstraint(double maxVel, double maxAngularVel, double trackWidth) {
        return new MinVelocityConstraint(Arrays.asList(
                new AngularVelocityConstraint(maxAngularVel),
                new MecanumVelocityConstraint(maxVel, trackWidth)
        ));
    }

    public static TrajectoryAccelerationConstraint getAccelerationConstraint(double maxAccel) {
        return new ProfileAccelerationConstraint(maxAccel);
    }

    //Our Methods
    public void liftToHeightEncoders(int level, double power){
        int liftValue = 0;
        if (level == 2) {
            liftValue = -1200;
        } else if (level == 3) {
            liftValue = -3800;
        } else if (level == 4) {
            liftValue = -4000;
        }

        liftMotor1.setTargetPosition(liftValue);
        liftMotor2.setTargetPosition(liftValue);
        liftMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        liftMotor1.setPower(power);
        liftMotor2.setPower(power);

        while (liftMotor1.isBusy() || liftMotor2.isBusy()){
            telemetry.addData("Lift Encoder Position 1", liftMotor1.getCurrentPosition());
            telemetry.addData("Lift Encoder Position 2", liftMotor2.getCurrentPosition());
            telemetry.update();
        }

        liftMotor1.setPower(0);
        liftMotor2.setPower(0);
        liftMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public boolean isDistClose(int i) {
        return distanceSensor.getDistance(DistanceUnit.INCH) < i;
    }
    public double getDist() {return distanceSensor.getDistance(DistanceUnit.INCH);}
    public void setArmPos(int level) {
        int armValue = 0;
        double power = 0;
        if (level == -1) {
            power = 0;
        } else if (level == 0) {
            armValue = 0;
            power = 0.5;
        } else if (level == 1) {
            armValue = 75;
            power = 0.7;
        } else if (level == 2) {
            armValue = 161;
            power = 0.7;
        }

        pixArm.setTargetPosition(armValue);
        pixArm.setPower(power);
    }
    public void setPivot(double pos) {
        rightGrabberPivot.setPosition(pos);
        leftGrabberPivot.setPosition(1-pos);
    }
    public void setRightGrabber(double pos) {
        rightGrabber.setPosition(pos);
    }
    public void setLeftGrabber(double pos) {
        leftGrabber.setPosition(pos);
    }

    private double startDeg = 0.0;
    public void setIsRotatedStartDeg(double startDeg) {
        this.startDeg = startDeg;
    }
    // Returns false until the robot has turned to the desired angle [old imu]
    public boolean isRotated(double degrees) {
        if (startDeg>degrees) //turning right
            return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS)-Math.toRadians(startDeg+1e6)<=Math.toRadians(degrees-(startDeg+1e6));
        else //turning left
            return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS)-Math.toRadians(startDeg+1e6)>=Math.toRadians(degrees-(startDeg+1e6));
    }
    /*public void turnDegrees(double targetDegrees, double degrees) {
        while (true) {
            if (degrees>targetDegrees) { //turning right
                if (Math.abs(getYaw())-Math.abs(targetDegrees)<25) {
                    manualMotorPower(1*(Math.abs(getYaw())/Math.abs(targetDegrees)),-0.2,0.2,-0.2);
                }
                //once it has reacher a certain degree that is close, set power to some relation of the degree
            } else {//turning left
                if (Math.abs(getYaw())-Math.abs(targetDegrees)<25) {
                    manualMotorPower(-0.2,0.2,-0.2,0.2);
                }
            }
        }
    }*/
    public void turnDegrees(double targetDegrees) {
        double leftPower;
        double rightPower;
        while (1==1) {
            if (getYaw()>targetDegrees) { //turning right
                leftPower = (Math.abs(Math.atan(0.5*(getYaw()-targetDegrees))))/pi;
                rightPower = -(Math.abs(Math.atan(0.5*(getYaw()-targetDegrees))))/pi;
            }
            else {//turning left
                leftPower = -(Math.abs(Math.atan(0.5*(getYaw()-targetDegrees))))/pi;
                rightPower = (Math.abs(Math.atan(0.5*(getYaw()-targetDegrees))))/pi;
            }
            manualMotorPower(leftPower,rightPower,leftPower,rightPower);
            if (Math.abs(Math.abs(getYaw()) - Math.abs(targetDegrees)) < 4) {
                manualMotorPower(0,0,0,0);
                return;
            }
        }
    }
    public void turnDegrees(double targetDegrees, int margin) {
        double leftPower;
        double rightPower;
        while (1==1) {
            if (getYaw()>targetDegrees) { //turning right
                leftPower = (Math.abs(Math.atan(0.5*(getYaw()-targetDegrees))))/pi;
                rightPower =-(Math.abs(Math.atan(0.5*(getYaw()-targetDegrees))))/pi;
            }
            else {//turning left
                leftPower = -(Math.abs(Math.atan(0.5*(getYaw()-targetDegrees))))/pi;
                rightPower = (Math.abs(Math.atan(0.5*(getYaw()-targetDegrees))))/pi;
            }
            manualMotorPower(leftPower,rightPower,leftPower,rightPower);
            if (Math.abs(getYaw()-targetDegrees) < margin) {
                manualMotorPower(0,0,0,0);
                return;
            }
        }
    }
    public double getYaw() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }
    public void resetYaw() {
        imu.resetYaw();
    }
    public void manualMotorPower(double frontLeft, double frontRight, double backLeft, double backRight) {
        leftFront.setPower(frontLeft);
        leftRear.setPower(backLeft);
        rightFront.setPower(frontRight);
        rightRear.setPower(backRight);
    }
}
