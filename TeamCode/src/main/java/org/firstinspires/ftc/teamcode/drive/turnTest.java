package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "test")
public class turnTest extends LinearOpMode {
    private final ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap);
        robot.resetYaw();

        waitForStart();
        if (isStopRequested()) return;

        robot.turnDegrees(-60,robot.getYaw());
        sleep(10000);
    }
}
