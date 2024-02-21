package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="CenterStageManualV5")
public class CenterStageManualV5 extends LinearOpMode {

    static final double LEFT_LINEAR_SLIDE_POWER = 0.75;

    static final int LINEAR_SLIDE_MAX_POSITION = 2;
    static final int LINEAR_SLIDE_MIN_POSITION = 0;

    static final int LINEAR_SLIDE_POSITION_0 = 100;
    static final int LINEAR_SLIDE_POSITION_1 = 1900;
    static final int LINEAR_SLIDE_POSITION_2 = 2600;

    static final double LEFT_CLAW_INITIAL_POSITION = 0.21;
    static final double RIGHT_CLAW_INITIAL_POSITION = 0.425;
    static final double CLAW_RETRACT_DIFF = 0.15;

    static final double ARM_BOTTOM_POSITION = 0.61;
    static final double ARM_TOP_POSITION = 0;

    static final double DRIVE_SPEED = -1;


    private final ElapsedTime runtime = new ElapsedTime();
    private DistanceSensor sensorRange = null;
    // motors for the wheels
    private DcMotor leftFrontMotor = null;
    private DcMotor leftBackMotor = null;
    private DcMotor rightFrontMotor = null;
    private DcMotor rightBackMotor = null;

    // motor for linear slide
    private DcMotor leftSlide = null;
    private DcMotor rightSlide = null;

    // motors for claw
    private Servo arm = null;
    private Servo leftClaw = null;
    private Servo rightClaw = null;
    private Servo droneLauncher = null;
    private Servo wrist = null;

    EruditeUtils utilities = new EruditeUtils();
    ElapsedTime timer = new ElapsedTime();
    boolean complete = false;
    int linearLevel = 0; // level of linear slide

    int[] leftLevels = {
            LINEAR_SLIDE_POSITION_0, LINEAR_SLIDE_POSITION_1,
            LINEAR_SLIDE_POSITION_2
    };

    int currentPosLeftSlide = 0;
    int currentPosRightSlide = 0;

    boolean leftBumper = false;
    boolean leftBumper1 = false;
    boolean rightBumper = false;
    boolean rightBumper1 = false;

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

        arm = hardwareMap.get(Servo.class, "ArmServo");
        wrist = hardwareMap.get(Servo.class, "WristServo");
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

        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();
        timer.reset();

        double driveSensitivity = DRIVE_SPEED;
        boolean arm_up = true;
        boolean hanging = false;


        while (opModeIsActive()) {
            boolean finger = false;

            if (gamepad1.dpad_up) {
                droneLauncher.setPosition(0);
            }
            else if (gamepad1.dpad_down) {
                droneLauncher.setPosition(1);
            }
            if (gamepad1.x) {
                arm_up = true;
            }
            if (gamepad1.y) {
                arm_up = false;
            }
            if (gamepad2.right_bumper) {
                rightBumper = true;
            }
            if (!gamepad2.right_bumper && rightBumper) {
                LinearSlideUp();
                currentPosLeftSlide = leftLevels[linearLevel];
                currentPosRightSlide = leftLevels[linearLevel];
            }
            if(gamepad2.dpad_up){
                arm.setPosition(0.2);
                sleep(2000);
                wrist.setPosition(0.6);
            }
            else if(gamepad2.dpad_right){
                arm.setPosition(0.03);
                sleep(2000);
                wrist.setPosition(0.4);
            }

            if (gamepad2.left_bumper) {
                leftBumper = true;
            }
            if (leftBumper && !gamepad2.left_bumper && !complete) {
                timer.reset();
                complete = true;
            }
            if(complete){
                if (linearLevel == 1){
                    LinearSlideDown();
                    wrist.setPosition(0.3);
                    timer.reset();
                    if (timer.time() > 2){
                        arm.setPosition(0.7);
                        if(timer.time() > 4){
                            currentPosLeftSlide = leftLevels[linearLevel];
                            currentPosRightSlide = leftLevels[linearLevel];
                            complete = false;
                        }
                    }
                }
                else {
                    LinearSlideDown();
                    currentPosLeftSlide = leftLevels[linearLevel];
                    currentPosRightSlide = leftLevels[linearLevel];
                    complete = false;
                }
            }
            if(leftSlide.getCurrentPosition() <= 100 && linearLevel == 0){
                if (arm_up){
                    arm.setPosition(0.7);
                    wrist.setPosition(0.3);
                }
                else {
                    arm.setPosition(0.7);
                    wrist.setPosition(0.9);
                }
            }
            if(gamepad2.y){
                leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }
            if(gamepad2.dpad_left){
                hanging = true;
            }
            if(hanging) {
                leftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                rightSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                leftSlide.setPower(0);
                rightSlide.setPower(0);
                if (gamepad2.right_trigger > 0.5) {
                    leftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    rightSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    leftSlide.setPower(LEFT_LINEAR_SLIDE_POWER);
                    rightSlide.setPower(LEFT_LINEAR_SLIDE_POWER);
                }
                if (gamepad2.dpad_down) {
                    leftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    rightSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    leftSlide.setPower(-1 * LEFT_LINEAR_SLIDE_POWER);
                    rightSlide.setPower(-1 * LEFT_LINEAR_SLIDE_POWER);
                }
            }
            else if(linearLevel == 0 && leftSlide.getCurrentPosition() <= 300 && rightSlide.getCurrentPosition() <= 300){
                leftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                rightSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                leftSlide.setPower(0);
                rightSlide.setPower(0);
                if(gamepad2.left_trigger > 0.5){
                    leftSlide.setPower(-0.3);
                    rightSlide.setPower(-0.3);
                }
                if(gamepad2.right_trigger > 0.5){
                    leftSlide.setPower(0);
                    rightSlide.setPower(0);
                }
            }
            else {
                if(!hanging) {
                    utilities.linearArmCenterStage(leftSlide, currentPosLeftSlide, LEFT_LINEAR_SLIDE_POWER);
                    utilities.linearArmCenterStage(rightSlide, currentPosRightSlide, LEFT_LINEAR_SLIDE_POWER);
                }
            }

            if (gamepad2.a) {
                leftClaw.setPosition(LEFT_CLAW_INITIAL_POSITION);
                rightClaw.setPosition(RIGHT_CLAW_INITIAL_POSITION);
            }
            else if (gamepad2.b){
                leftClaw.setPosition(LEFT_CLAW_INITIAL_POSITION + CLAW_RETRACT_DIFF);
                rightClaw.setPosition(RIGHT_CLAW_INITIAL_POSITION - CLAW_RETRACT_DIFF);
            }

            telemetry.addData("Left Pos", leftSlide.getCurrentPosition());
            telemetry.addData("Right Pos", rightSlide.getCurrentPosition());
            if (gamepad1.a) {
                droneLauncher.setPosition(0);
            }
            else if (gamepad1.b) {
                droneLauncher.setPosition(1);
            }

            telemetry.addData("linear level", linearLevel);
            double max;

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial   = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral =  gamepad1.left_stick_x;
            double yaw     =  -gamepad1.right_stick_x * 0.5;

            if (gamepad1.left_bumper) {
                leftBumper1 = true;
            }
            if (leftBumper1 && !gamepad1.left_bumper) {

                driveSensitivity *= -1;

                leftBumper1 = false;
            }


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