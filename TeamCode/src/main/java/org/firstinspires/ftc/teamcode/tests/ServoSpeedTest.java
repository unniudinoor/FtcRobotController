package org.firstinspires.ftc.teamcode.tests;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.EruditeUtils;


@TeleOp(name="ServoSpeedTest")
public class ServoSpeedTest extends LinearOpMode {

    private final ElapsedTime runtime = new ElapsedTime();

    // motors for claw
    private Servo droneLauncher = null;
    EruditeUtils utilities = new EruditeUtils();

    int targetPosition = 0;
    int position = 0;

    private double servoDelta = 0.05;
    private double servoDelayTime = 1;

    @Override
    public void runOpMode() throws InterruptedException {
        droneLauncher = hardwareMap.get(Servo.class, "Drone Launcher Servo");
        waitForStart();
        runtime.reset();
        while (opModeIsActive()) {
            boolean finger = false;
            if (gamepad1.a) {
                targetPosition = 0;
                runtime.reset();
                if(runtime.time() > servoDelayTime) {
                    if(droneLauncher.getPosition() < targetPosition) {
                        position += servoDelta;
                        droneLauncher.setPosition(position);
                    }
                    else if(droneLauncher.getPosition() > targetPosition) {
                        position -= servoDelta;
                        droneLauncher.setPosition(position);
                    }
                    runtime.reset();
                }
            }
            if (gamepad1.b) {
                targetPosition = 1;
                runtime.reset();
                if(runtime.time() > servoDelayTime) {
                    if(droneLauncher.getPosition() < targetPosition) {
                        position += servoDelta;
                        droneLauncher.setPosition(position);
                    }
                    else if(droneLauncher.getPosition() > targetPosition) {
                        position -= servoDelta;
                        droneLauncher.setPosition(position);
                    }
                    runtime.reset();
                }
            }
            telemetry.update();
        }
    }
}



