package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="CenterStageManualV2")
public class CenterStageManualV2 extends LinearOpMode {

    static final double LEFT_LINEAR_SLIDE_POWER = 0.75;
    static final double RIGHT_LINEAR_SLIDE_POWER = LEFT_LINEAR_SLIDE_POWER * 312/435;

    static final int LINEAR_SLIDE_MAX_POSITION = 2;
    static final int LINEAR_SLIDE_MIN_POSITION = 0;

    static final int LINEAR_SLIDE_POSITION_0 = 30;
    static final int LINEAR_SLIDE_POSITION_1 = 1500;
    static final int LINEAR_SLIDE_POSITION_2 = 2600;

    static final double LEFT_CLAW_INITIAL_POSITION = 0.21;
    static final double RIGHT_CLAW_INITIAL_POSITION = 0.425;
    static final double CLAW_RETRACT_DIFF = 0.15;

    static final double LEFT_ARM_BOTTOM_POSITION = 0.21;
    static final double RIGHT_ARM_BOTTOM_POSITION = 0.77;
    static final double ARM_DIFF = 0.69;

    static final int INCREMENT = 100;

    static final double DRIVE_SPEED = -0.8;


    private final ElapsedTime runtime = new ElapsedTime();

    // motors for the wheels
    private DcMotor leftFrontMotor = null;
    private DcMotor leftBackMotor = null;
    private DcMotor rightFrontMotor = null;
    private DcMotor rightBackMotor = null;

    // motor for linear slide
    private DcMotor leftSlide = null;
    private DcMotor rightSlide = null;

    // motors for claw
    private Servo leftArm = null;
    private Servo rightArm = null;
    private Servo leftClaw = null;
    private Servo rightClaw = null;
    private Servo droneLauncher = null;
    EruditeUtils utilities = new EruditeUtils();

    int linearLevel = 0; // level of linear slide

    int[] leftLevels = {
            LINEAR_SLIDE_POSITION_0, LINEAR_SLIDE_POSITION_1,
            LINEAR_SLIDE_POSITION_2
    };
    int[] rightLevels = {
            utilities.convertSlideValues(LINEAR_SLIDE_POSITION_0),
            utilities.convertSlideValues(LINEAR_SLIDE_POSITION_1),
            utilities.convertSlideValues(LINEAR_SLIDE_POSITION_2)
    };

    int currentPosLeftSlide = 0;
    int currentPosRightSlide = 0;

    boolean leftBumper = false;
    boolean rightBumper = false;

    boolean rt = false;
    boolean lt = false;

    public void LinearSlideUp() {
        if(linearLevel < LINEAR_SLIDE_MAX_POSITION){
            linearLevel+=1;
        }
        rightBumper = false;
    }

    public void LinearSlideDown() {
        if (linearLevel > LINEAR_SLIDE_MIN_POSITION){
            linearLevel-=1;
        }
        leftBumper = false;
    }

    @Override
    public void runOpMode() throws InterruptedException{
        rightSlide = hardwareMap.get(DcMotor.class, "RightLinearSlideMotor");
        leftSlide = hardwareMap.get(DcMotor.class, "LeftLinearSlideMotor");

        leftArm = hardwareMap.get(Servo.class, "LeftArm Servo");
        rightArm = hardwareMap.get(Servo.class, "RightArm Servo");

        leftClaw = hardwareMap.get(Servo.class, "LeftClaw Servo");
        rightClaw = hardwareMap.get(Servo.class, "RightClaw Servo");

        leftFrontMotor = hardwareMap.get(DcMotor.class, "LeftFrontMotor");
        leftBackMotor = hardwareMap.get(DcMotor.class, "LeftBackMotor");
        rightFrontMotor = hardwareMap.get(DcMotor.class, "RightFrontMotor");
        rightBackMotor = hardwareMap.get(DcMotor.class, "RightBackMotor");

        droneLauncher = hardwareMap.get(Servo.class, "Drone Launcher Servo");

        // prepare DCmotor direction
        utilities.centerStageInitialize(leftFrontMotor, rightFrontMotor, leftBackMotor, rightBackMotor, leftSlide, rightSlide);

        leftClaw.setDirection(Servo.Direction.FORWARD);
        rightClaw.setDirection(Servo.Direction.FORWARD);

        leftArm.setDirection(Servo.Direction.FORWARD);
        rightArm.setDirection(Servo.Direction.FORWARD);

        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        double driveSensitivity = DRIVE_SPEED;


        while (opModeIsActive()) {
            boolean finger = false;

            if (gamepad2.right_bumper) {
                rightBumper = true;
            }
            if (!gamepad2.right_bumper && rightBumper) {
                LinearSlideUp();
                currentPosLeftSlide = leftLevels[linearLevel];
                currentPosRightSlide = leftLevels[linearLevel];
                rightArm.setPosition(RIGHT_ARM_BOTTOM_POSITION - ARM_DIFF + 0.005);
                leftArm.setPosition(LEFT_ARM_BOTTOM_POSITION + ARM_DIFF);
            }

            if (gamepad2.left_bumper) {
                leftBumper = true;
            }
            if (leftBumper && !gamepad2.left_bumper) {
                LinearSlideDown();
                currentPosLeftSlide = leftLevels[linearLevel];
                currentPosRightSlide = leftLevels[linearLevel];
                if(linearLevel == 0){
                    rightArm.setPosition(RIGHT_ARM_BOTTOM_POSITION - 0.2);
                    leftArm.setPosition(LEFT_ARM_BOTTOM_POSITION + 0.2);
                }
            }
            if(leftSlide.getCurrentPosition() <= 100 && linearLevel == 0){
                rightArm.setPosition(RIGHT_ARM_BOTTOM_POSITION);
                leftArm.setPosition(LEFT_ARM_BOTTOM_POSITION);
            }

            utilities.linearArmCenterStage(leftSlide, currentPosLeftSlide, LEFT_LINEAR_SLIDE_POWER);
            utilities.linearArmCenterStage(rightSlide, currentPosRightSlide, LEFT_LINEAR_SLIDE_POWER);

            if (gamepad2.a) {
                leftClaw.setPosition(LEFT_CLAW_INITIAL_POSITION);
                rightClaw.setPosition(RIGHT_CLAW_INITIAL_POSITION);
            }
            else if (gamepad2.b){
                leftClaw.setPosition(LEFT_CLAW_INITIAL_POSITION + CLAW_RETRACT_DIFF);
                rightClaw.setPosition(RIGHT_CLAW_INITIAL_POSITION - CLAW_RETRACT_DIFF);
            }

            if (gamepad2.right_trigger > 0.5) {
                rt = true;
            }
            if (gamepad2.right_trigger < 0.5 && rt) {
                currentPosLeftSlide += INCREMENT;
                currentPosLeftSlide = currentPosLeftSlide > leftLevels[2] ? leftLevels[2] : currentPosLeftSlide;
                currentPosRightSlide += INCREMENT;
                currentPosRightSlide = currentPosRightSlide > rightLevels[2] ? rightLevels[2] : currentPosRightSlide;
                rt = false;
            }
            if (gamepad2.left_trigger > 0.5) {
                lt = true;
            }
            if (gamepad2.left_trigger < 0.5 && lt) {
                currentPosLeftSlide -= INCREMENT;
                currentPosLeftSlide = currentPosLeftSlide < leftLevels[0] ? leftLevels[0] : currentPosLeftSlide;
                currentPosRightSlide -= INCREMENT;
                currentPosRightSlide = currentPosRightSlide < rightLevels[0] ? rightLevels[0] : currentPosRightSlide;
                lt = false;
            }

            telemetry.addData("Left Pos", leftSlide.getCurrentPosition());
            telemetry.addData("Right Pos", rightSlide.getCurrentPosition());
            if (gamepad1.a) {
                droneLauncher.setPosition(0);
            }
            else if (gamepad1.b) {
                droneLauncher.setPosition(1);
            }

            double max;

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial   = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral =  gamepad1.left_stick_x;
            double yaw     =  -gamepad1.right_stick_x * 0.5;

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double leftFrontPower  = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower   = axial - lateral + yaw;
            double rightBackPower  = axial + lateral - yaw;

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower  /= max;
                rightFrontPower /= max;
                leftBackPower   /= max;
                rightBackPower  /= max;
            }

            // Send calculated power to wheels
            leftFrontMotor.setPower(leftFrontPower * driveSensitivity);
            rightFrontMotor.setPower(rightFrontPower * driveSensitivity);
            leftBackMotor.setPower(leftBackPower * driveSensitivity);
            rightBackMotor.setPower(rightBackPower * driveSensitivity);

            // Show the elapsed game time and wheel power.
            //telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            //telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.update();
        }
    }

}
