package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;




@TeleOp(name="DroneTest")
public class DroneTest extends LinearOpMode {

    private final ElapsedTime runtime = new ElapsedTime();

    // motors for claw
    private Servo droneLauncher = null;
    EruditeUtils utilities = new EruditeUtils();


    @Override
    public void runOpMode() throws InterruptedException {
        droneLauncher = hardwareMap.get(Servo.class, "Drone Launcher Servo");
        waitForStart();
        runtime.reset();
        while (opModeIsActive()) {
            boolean finger = false;
            if (gamepad1.a) {
                droneLauncher.setPosition(-05);
            }
            if (gamepad1.b) {
                droneLauncher.setPosition(.9);

            telemetry.update();
}       }
    }
}



