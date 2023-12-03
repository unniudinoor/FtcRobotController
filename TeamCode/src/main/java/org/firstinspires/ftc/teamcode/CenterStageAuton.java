package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;


@Autonomous(name="CenterStageAuton", group="Robot")
public class CenterStageAuton extends LinearOpMode {

    private DcMotor leftFrontMotor = null;
    private DcMotor leftBackMotor = null;
    private DcMotor rightFrontMotor = null;
    private DcMotor rightBackMotor = null;
    private DcMotor leftSlide = null;
    private DcMotor rightSlide = null;
    private Servo leftArm = null;
    private Servo rightArm = null;
    private Servo leftClaw = null;
    private Servo rightClaw = null;
    private Servo droneLauncher = null;
    private ColorSensor color_sensor = null;
    EruditeUtils Util = new EruditeUtils();

    private ElapsedTime runtime = new ElapsedTime();


    static final double LINEAR_SLIDE_POWER = 0.75;

    static final int LINEAR_SLIDE_MAX_POSITION = 3;
    static final int LINEAR_SLIDE_MIN_POSITION = 0;

    static final int LINEAR_SLIDE_POSITION_0 = 100;
    static final int LINEAR_SLIDE_POSITION_1 = 800;
    static final int LINEAR_SLIDE_POSITION_2 = 1700;
    static final int LINEAR_SLIDE_POSITION_3 = 2600;

    static final double LEFT_CLAW_INITIAL_POSITION = 0.21;
    static final double RIGHT_CLAW_INITIAL_POSITION = 0.425;
    static final double CLAW_RETRACT_DIFF = 0.15;

    static final double LEFT_ARM_BOTTOM_POSITION = 0.19;
    static final double RIGHT_ARM_BOTTOM_POSITION = 0.79;
    static final double ARM_DIFF = 0.69;

    static final double DRIVE_SPEED = -0.6;

    @Override
    public void runOpMode() {

        // Initialize the drive system variables.
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
        color_sensor = hardwareMap.get(ColorSensor.class, "Color Sensor");

        leftFrontMotor.setDirection(DcMotor.Direction.FORWARD);
        leftBackMotor.setDirection(DcMotor.Direction.FORWARD);
        rightFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        rightBackMotor.setDirection(DcMotor.Direction.REVERSE);
        leftSlide.setDirection(DcMotor.Direction.REVERSE);
        rightSlide.setDirection(DcMotor.Direction.FORWARD);

        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);




        DcMotor[] motors = {leftFrontMotor, rightFrontMotor, leftBackMotor, rightBackMotor};
        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips


        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        runtime.reset();
        while (opModeIsActive()) {

            //insert code to detect object
            Util.encoderDriveForward(10, motors);
            sleep(100);
            Util.servos(leftArm, rightArm, LEFT_ARM_BOTTOM_POSITION + 0.1, RIGHT_ARM_BOTTOM_POSITION - 0.1);
            Util.servos(leftClaw, rightClaw, LEFT_CLAW_INITIAL_POSITION + CLAW_RETRACT_DIFF, RIGHT_CLAW_INITIAL_POSITION - CLAW_RETRACT_DIFF);
            Util.encoderDriveForward(4, motors);
            sleep(100);
            Util.servos(leftArm, rightArm, LEFT_ARM_BOTTOM_POSITION, RIGHT_ARM_BOTTOM_POSITION);
            Util.encoderDriveForward(2, motors);
            sleep(100);
            Util.servos(leftClaw, rightClaw, LEFT_CLAW_INITIAL_POSITION, RIGHT_CLAW_INITIAL_POSITION);
            sleep(1000);
            Util.servos(leftArm, rightArm, LEFT_ARM_BOTTOM_POSITION + 0.1, RIGHT_ARM_BOTTOM_POSITION - 0.1);
            sleep(100);
            Util.encoderDriveForward(15, motors);
            sleep(100);
            Util.encoderDriveRight(10, motors);
            sleep(100);
            Util.encoderRotate(90, motors);
            sleep(100);
            Util.encoderDriveLeft(5, motors);
            sleep(100);
            Util.encoderDriveBackward(5, motors);
            sleep(100);
            Util.servos(leftArm, rightArm, LEFT_ARM_BOTTOM_POSITION, RIGHT_ARM_BOTTOM_POSITION);
            sleep(100);
            leftClaw.setPosition(LEFT_CLAW_INITIAL_POSITION + CLAW_RETRACT_DIFF);
            sleep(100);
            Util.servos(leftArm, rightArm, LEFT_ARM_BOTTOM_POSITION + 0.1, RIGHT_ARM_BOTTOM_POSITION - 0.1);
            sleep(100);


            while (color_sensor.red() < 150 && (color_sensor.red() > color_sensor.blue() + color_sensor.green())){
                leftFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                leftBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                rightFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                rightBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                leftFrontMotor.setPower(0.5);
                leftBackMotor.setPower(0.5);
                rightFrontMotor.setPower(0.5);
                rightBackMotor.setPower(0.5);
            }
            leftFrontMotor.setPower(0);
            leftBackMotor.setPower(0);
            rightFrontMotor.setPower(0);
            rightBackMotor.setPower(0);


            Util.servos(leftArm, rightArm, LEFT_ARM_BOTTOM_POSITION + ARM_DIFF, RIGHT_ARM_BOTTOM_POSITION - ARM_DIFF);
            Util.linearArmAutonCS(leftSlide, rightSlide, LINEAR_SLIDE_POSITION_2);
            Util.servos(leftClaw, rightClaw, LEFT_CLAW_INITIAL_POSITION + CLAW_RETRACT_DIFF, RIGHT_CLAW_INITIAL_POSITION - CLAW_RETRACT_DIFF);


            break;

        }
    }
}

