package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="ArmTest")
public class ArmTest extends LinearOpMode {

//  static final double LEFT_ARM_BOTTOM_POSITION = 0.130;
//    static final double RIGHT_ARM_BOTTOM_POSITION = 0.840;
    static final double LEFT_ARM_BOTTOM_POSITION = 0.150;
    static final double RIGHT_ARM_BOTTOM_POSITION = 0.785;
    static final double RIGHT_ARM_DIFF = 0.740;
    static final double LEFT_ARM_DIFF = RIGHT_ARM_DIFF + 0.005  ;

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
                leftArm.setPosition(LEFT_ARM_BOTTOM_POSITION);
                rightArm.setPosition(RIGHT_ARM_BOTTOM_POSITION);

            }
            if(gamepad1.b) {
                leftArm.setPosition(LEFT_ARM_BOTTOM_POSITION + LEFT_ARM_DIFF);
                rightArm.setPosition(RIGHT_ARM_BOTTOM_POSITION - RIGHT_ARM_DIFF);

            }
            telemetry.update();
        }
    }

}
