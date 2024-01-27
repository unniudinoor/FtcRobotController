package org.firstinspires.ftc.teamcode.tests;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.EruditeUtils;


@TeleOp(name="AxonTest")
public class AxonTest extends LinearOpMode {

    //  static final double LEFT_ARM_BOTTOM_POSITION = 0.130;
//    static final double RIGHT_ARM_BOTTOM_POSITION = 0.840;
    static final double ARM_TOP_POSITION = 0;
    static final double ARM_BOTTOM_POSITION = 0.61;

    static final double LEFT_CLAW_INITIAL_POSITION = 0.21;
    static final double RIGHT_CLAW_INITIAL_POSITION = 0.425;
    static final double CLAW_RETRACT_DIFF = 0.15;


    private Servo leftClaw = null;
    private Servo rightClaw = null;
    private Servo arm = null;

    double pos = ARM_TOP_POSITION;
    boolean arm_up = true;

    EruditeUtils utilities = new EruditeUtils();
    private final ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException{
        arm = hardwareMap.get(Servo.class, "ArmServo");
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
                    arm.setPosition(ARM_BOTTOM_POSITION);
            }
            if(gamepad1.right_bumper){
                arm.setPosition(ARM_TOP_POSITION);
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
