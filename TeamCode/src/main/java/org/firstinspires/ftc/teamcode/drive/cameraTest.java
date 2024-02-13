package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

@TeleOp(group="drive")
public class cameraTest extends OpMode {
    private Robot robot;

    @Override
    public void init() {
        //robot = new Robot(hardwareMap);
        AprilTagProcessor QRProcessor = AprilTagProcessor.easyCreateWithDefaults();

        // TensorFlow Processor
        TfodProcessor objectProcessor = new TfodProcessor.Builder()
                .setMaxNumRecognitions(1)
                .setUseObjectTracker(true)
                .build();

        //Webcam
        VisionPortal webcam = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(QRProcessor)
                .addProcessor(objectProcessor)
                .enableLiveView(true)
                .setAutoStopLiveView(false)
                .build();
    }

    @Override
    public void loop() {

    }
}
