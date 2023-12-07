package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="ServoTest")
public class ServoTest extends LinearOpMode {
    static final double LEFT_CLAW_INTIIAL_POSITION = 0.5;
    static final double RIGHT_CLAW_INTIIAL_POSITION = 0.55;
    static final double CLAW_RETRACT_DIFF = 0.3;


    private final ElapsedTime runtime = new ElapsedTime();
    // motors for claw
    private Servo leftClaw = null;
    private Servo rightClaw = null;
    EruditeUtils utilities = new EruditeUtils();

    // expected final position for claw
    // left position: 0.4
    // right position: 0.55

    @Override
    public void runOpMode() throws InterruptedException {
        leftClaw = hardwareMap.get(Servo.class, "LeftClaw Servo");
        rightClaw = hardwareMap.get(Servo.class, "RightClaw Servo");
        leftClaw.setDirection(Servo.Direction.FORWARD);
        rightClaw.setDirection(Servo.Direction.FORWARD);

//        LinearSlide.setPower(0.5);
//        LinearSlide.setTargetPosition(LinearSlide.getCurrentPosition());
//        LinearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        double leftClawPosition = LEFT_CLAW_INTIIAL_POSITION; // left claw: 1 to 0
        double rightClawPosition = RIGHT_CLAW_INTIIAL_POSITION; // right claw: 0 to 1
        double diffClaw = CLAW_RETRACT_DIFF;


        // run until the end of the match (driver presses STOP

        /*
            Left Bumper = move linear slide down
            Right Bumper = move linear slide up
            Left Joystick = move forward/backward/move left/move right
            Right Joystick = Rotation
            Button A: Open claw
            Button B: close claw
        */

        while (opModeIsActive()) {
            boolean finger = false;

            if (gamepad1.a) {
                telemetry.addData("GP2 Input", "Button A");
                leftClaw.setPosition(leftClawPosition + 0.4);
                rightClaw.setPosition(rightClawPosition - 0.4);
            } else if (gamepad1.b) {
                telemetry.addData("GP2 Input", "Button B");
                leftClaw.setPosition(leftClawPosition - 0.4);
                rightClaw.setPosition(rightClawPosition + 0.4);

                // Show the elapsed game time and wheel power.
                //telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
                //telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
                telemetry.update();
            }
        }
    }
}