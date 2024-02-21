package org.firstinspires.ftc.teamcode.tests;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.EruditeUtils;


@TeleOp(name="WristTest")
public class WristTest extends LinearOpMode {

    static final double ARM_TOP_POSITION = 0.2;
    static final double ARM_BOTTOM_POSITION = 1;

    static final double LEFT_CLAW_INITIAL_POSITION = 0.21;
    static final double RIGHT_CLAW_INITIAL_POSITION = 0.425;
    static final double CLAW_RETRACT_DIFF = 0.15;


    private Servo leftClaw = null;
    private Servo rightClaw = null;
    private Servo arm = null;
    private Servo wrist = null;

    double pos = ARM_TOP_POSITION;
    boolean arm_up = true;

    EruditeUtils utilities = new EruditeUtils();
    private final ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException{
        arm = hardwareMap.get(Servo.class, "ArmServo");
        wrist = hardwareMap.get(Servo.class, "WristServo");
        leftClaw = hardwareMap.get(Servo.class, "LeftClaw Servo");
        rightClaw = hardwareMap.get(Servo.class, "RightClaw Servo");

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            boolean finger = false;
            if(gamepad1.left_bumper){
                // intake position
                arm.setPosition(0.7);
                wrist.setPosition(0.9);

            }
            if(gamepad1.right_bumper){
                // driving position
                wrist.setPosition(0.3);
                arm.setPosition(0.7);
            }

            if(gamepad1.x) {
                // high macro
                arm.setPosition(0.03);
                sleep(1000);
                wrist.setPosition(0.4);
            }
            if(gamepad1.y){
                // low macro
                arm.setPosition(0.2);
                sleep(1000);
                wrist.setPosition(0.6);
            }
            if(gamepad1.a){
                leftClaw.setPosition(LEFT_CLAW_INITIAL_POSITION);
                rightClaw.setPosition(RIGHT_CLAW_INITIAL_POSITION);
            }
            if(gamepad1.b){
                leftClaw.setPosition(LEFT_CLAW_INITIAL_POSITION + CLAW_RETRACT_DIFF);
                rightClaw.setPosition(RIGHT_CLAW_INITIAL_POSITION - CLAW_RETRACT_DIFF);
            }
            telemetry.addData("ARM position", arm.getPosition());
            telemetry.update();
        }
    }

}
