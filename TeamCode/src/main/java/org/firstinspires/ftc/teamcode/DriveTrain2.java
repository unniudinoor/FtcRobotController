package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.ColorSensor;


@TeleOp(name="DriveTrain2")
public class DriveTrain2 extends LinearOpMode {

    static final double LINEAR_SLIDE_POWER_UP = 0.75;
    static final double LINEAR_SLIDE_POWER_DOWN = -0.6;

    static final int LINEAR_SLIDE_POSITION_0 = 50;
    static final int LINEAR_SLIDE_POSITION_1 = 1800;
    static final int LINEAR_SLIDE_POSITION_2 = 3000;
    static final int LINEAR_SLIDE_POSITION_3 = 4200;

    static final int LINEAR_SLIDE_MAX_POSITION = 3;
    static final int LINEAR_SLIDE_MIN_POSITION = 0;

    static final double DRIVE_SPEED = 0.6;
    static final double LEFT_CLAW_INTIIAL_POSITION = 0.6;
    static final double RIGHT_CLAW_INTIIAL_POSITION = 0.35;
    static final double CLAW_RETRACT_DIFF = 0.2;



    private final ElapsedTime runtime = new ElapsedTime();

    // motors for the wheels
    private DcMotor leftFrontMotor = null;
    private DcMotor leftBackMotor = null;
    private DcMotor rightFrontMotor = null;
    private DcMotor rightBackMotor = null;

    // motor for linear slide
    private DcMotor linearSlide = null;

    // motors for claw
    private Servo leftClaw = null;
    private Servo rightClaw = null;
    private ColorSensor colorSensor = null;
    EruditeUtils utilities = new EruditeUtils();

    // expected final position for claw
    // left position: 0.4
    // right position: 0.55

    int linearLevel = 0; // level of linear slide

    int[] levels = {
            LINEAR_SLIDE_POSITION_0, LINEAR_SLIDE_POSITION_1,
            LINEAR_SLIDE_POSITION_2, LINEAR_SLIDE_POSITION_3
    }; // linear slide positions for each pole
    boolean leftBumper = false;
    boolean rightBumper = false;


    public void LinearSlideUp() {
        if(linearLevel < LINEAR_SLIDE_MAX_POSITION){
            linearSlide.setPower(LINEAR_SLIDE_POWER_UP);
            linearLevel+=1;
        }
        rightBumper = false;
    }

    public void LinearSlideDown() {
        if (linearLevel > LINEAR_SLIDE_MIN_POSITION){
            linearSlide.setPower(LINEAR_SLIDE_POWER_DOWN);
            linearLevel-=1;
        }
        leftBumper = false;
    }

    @Override
    public void runOpMode() throws InterruptedException{

        linearSlide = hardwareMap.get(DcMotor.class, "LinearSlideMotor");
        leftClaw = hardwareMap.get(Servo.class, "LeftClaw Servo");
        rightClaw = hardwareMap.get(Servo.class, "RightClaw Servo");
        leftFrontMotor = hardwareMap.get(DcMotor.class, "LeftFrontMotor");
        leftBackMotor = hardwareMap.get(DcMotor.class, "LeftBackMotor");
        rightFrontMotor = hardwareMap.get(DcMotor.class, "RightFrontMotor");
        rightBackMotor = hardwareMap.get(DcMotor.class, "RightBackMotor");
        colorSensor = hardwareMap.get(ColorSensor.class, "Color Sensor");

        // prepare DCmotor direction
        utilities.initializeDCMotors(leftFrontMotor, rightFrontMotor, leftBackMotor, rightBackMotor, linearSlide);

        leftClaw.setDirection(Servo.Direction.FORWARD);
        rightClaw.setDirection(Servo.Direction.FORWARD);

        linearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

//        LinearSlide.setPower(0.5);
//        LinearSlide.setTargetPosition(LinearSlide.getCurrentPosition());
//        LinearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        double driveSensitivity = DRIVE_SPEED;

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
            colorSensor.enableLed(false);
            boolean finger = false;

            int position = linearSlide.getCurrentPosition();

            if (gamepad1.right_bumper) {
                rightBumper = true;
            }
            if (!gamepad1.right_bumper && rightBumper) {
                LinearSlideUp();
            }

            if (gamepad1.left_bumper) {
                leftBumper = true;
            }
            if (leftBumper && !gamepad1.left_bumper) {
                LinearSlideDown();
            }

            utilities.linearArmManual(linearSlide, linearLevel, levels);

            if (gamepad1.a) {
                telemetry.addData("GP2 Input", "Button A");
                utilities.claw(leftClaw, rightClaw, leftClawPosition, rightClawPosition, 0);
            }
            else if (gamepad1.b){
                telemetry.addData("GP2 Input", "Button B");
                utilities.claw(leftClaw, rightClaw, leftClawPosition, rightClawPosition, diffClaw);
            }

            double max;

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial   = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral =  gamepad1.left_stick_x;
            double yaw     =  gamepad1.right_stick_x;

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