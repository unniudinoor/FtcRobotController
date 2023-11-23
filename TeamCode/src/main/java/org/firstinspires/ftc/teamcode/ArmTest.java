package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="ArmTest")
public class ArmTest extends LinearOpMode {

    // motors for claw
    private Servo leftArm = null;
    private Servo rightArm = null;
    EruditeUtils utilities = new EruditeUtils();
    private final ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException{
        leftArm = hardwareMap.get(Servo.class, "LeftArm Servo");
        rightArm = hardwareMap.get(Servo.class, "RightArm Servo");

        leftArm.setDirection(Servo.Direction.FORWARD);
        rightArm.setDirection(Servo.Direction.FORWARD);

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            boolean finger = false;
            if(gamepad1.a) {
                rightArm.setPosition(0.02);
                leftArm.setPosition(0.99);

            }
            if(gamepad1.b) {
                rightArm.setPosition(0.88);
                leftArm.setPosition(0.13);

            }
            telemetry.update();
        }
    }

}
