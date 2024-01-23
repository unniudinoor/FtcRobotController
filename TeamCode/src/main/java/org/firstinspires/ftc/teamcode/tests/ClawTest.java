package org.firstinspires.ftc.teamcode.tests;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.EruditeUtils;


@TeleOp(name="ClawTest")
public class ClawTest extends LinearOpMode {

    static final double LEFT_CLAW_INITIAL_POSITION = 0.6;
    static final double RIGHT_CLAW_INITIAL_POSITION = 0.35;
    static final double CLAW_RETRACT_DIFF = 0.2;



    private final ElapsedTime runtime = new ElapsedTime();

    // motors for claw
    private Servo leftClaw = null;
    private Servo rightClaw = null;
    EruditeUtils utilities = new EruditeUtils();

    @Override
    public void runOpMode() throws InterruptedException{

        leftClaw = hardwareMap.get(Servo.class, "LeftClaw Servo");
        rightClaw = hardwareMap.get(Servo.class, "RightClaw Servo");

        // prepare DCmotor direction

        leftClaw.setDirection(Servo.Direction.FORWARD);
        rightClaw.setDirection(Servo.Direction.FORWARD);

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        double leftClawPosition = LEFT_CLAW_INITIAL_POSITION; // left claw: 1 to 0
        double rightClawPosition = RIGHT_CLAW_INITIAL_POSITION; // right claw: 0 to 1
        double diffClaw = CLAW_RETRACT_DIFF;

        while (opModeIsActive()) {
            boolean finger = false;

            if (gamepad1.a) {
                leftClaw.setPosition(0.075);
                rightClaw.setPosition(0.425);
            }
            else if (gamepad1.b){
                leftClaw.setPosition(0.3);
                rightClaw.setPosition(0.15);
            }

            telemetry.update();
        }
    }

}