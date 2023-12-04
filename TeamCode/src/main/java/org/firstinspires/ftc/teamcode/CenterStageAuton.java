package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


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
    private DistanceSensor sensorRange = null;
//    private ColorSensor color_sensor = null;
    EruditeUtils Util = new EruditeUtils();

    private ElapsedTime runtime = new ElapsedTime();


    static final double LINEAR_SLIDE_POWER = 0.75;

    static final int LINEAR_SLIDE_MAX_POSITION = 3;
    static final int LINEAR_SLIDE_MIN_POSITION = 0;

    static final int LINEAR_SLIDE_POSITION_0 = 100;
    static final int LINEAR_SLIDE_POSITION_2 = 1500;
    static final int LINEAR_SLIDE_POSITION_3 = 2600;

    static final double LEFT_CLAW_INITIAL_POSITION = 0.21;
    static final double RIGHT_CLAW_INITIAL_POSITION = 0.425;
    static final double CLAW_RETRACT_DIFF = 0.15;

    static final double LEFT_ARM_BOTTOM_POSITION = 0.19;
    static final double RIGHT_ARM_BOTTOM_POSITION = 0.79;
    static final double ARM_DIFF = 0.69;

    static final double DRIVE_SPEED = 0.5;

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
        droneLauncher = hardwareMap.get(Servo.class, "Drone Launcher Servo");
        sensorRange = hardwareMap.get(DistanceSensor.class, "sensor_range");

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

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        leftFrontMotor.setPower(DRIVE_SPEED);
        rightFrontMotor.setPower(DRIVE_SPEED);
        leftBackMotor.setPower(DRIVE_SPEED);
        rightBackMotor.setPower(DRIVE_SPEED);

        leftFrontMotor.setTargetPosition(leftFrontMotor.getCurrentPosition());
        leftBackMotor.setTargetPosition(leftFrontMotor.getCurrentPosition());
        rightFrontMotor.setTargetPosition(leftFrontMotor.getCurrentPosition());
        rightBackMotor.setTargetPosition(leftFrontMotor.getCurrentPosition());

        leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        runtime.reset();
        while (opModeIsActive()) {

            //insert code to detect object
            Util.encoderDriveForward(15, motors);
            sleep(500);
            Util.servos(leftArm, rightArm, LEFT_ARM_BOTTOM_POSITION + 0.1, RIGHT_ARM_BOTTOM_POSITION - 0.1);
            sleep(500);
            Util.servos(leftClaw, rightClaw, LEFT_CLAW_INITIAL_POSITION + CLAW_RETRACT_DIFF, RIGHT_CLAW_INITIAL_POSITION - CLAW_RETRACT_DIFF);
            sleep(500);
            Util.encoderDriveBackward(5, motors);
            sleep(500);
            Util.servos(leftArm, rightArm, LEFT_ARM_BOTTOM_POSITION, RIGHT_ARM_BOTTOM_POSITION);
            sleep(500);
            Util.encoderDriveBackward(2, motors);
            sleep(500);
            Util.servos(leftClaw, rightClaw, LEFT_CLAW_INITIAL_POSITION, RIGHT_CLAW_INITIAL_POSITION);

            sleep(500);
            Util.servos(leftArm, rightArm, LEFT_ARM_BOTTOM_POSITION + 0.1, RIGHT_ARM_BOTTOM_POSITION - 0.1);
            sleep(500);
            Util.encoderDriveForward(20, motors);
            sleep(500);
            Util.encoderDriveRight(15, motors);
            sleep(500);
            Util.encoderRotate(-103, motors);
            sleep(500);
            Util.encoderDriveBackward(8, motors);
            sleep(500);
            Util.servos(leftArm, rightArm, LEFT_ARM_BOTTOM_POSITION, RIGHT_ARM_BOTTOM_POSITION);
            sleep(500);
            leftClaw.setPosition(LEFT_CLAW_INITIAL_POSITION + CLAW_RETRACT_DIFF);
            sleep(500);
            Util.encoderDriveForward(5, motors);
            sleep(500);
            Util.servos(leftArm, rightArm, LEFT_ARM_BOTTOM_POSITION + 0.1, RIGHT_ARM_BOTTOM_POSITION - 0.1);
            sleep(500);

            Util.encoderDriveForward(33, motors);

            Util.servos(leftArm, rightArm, LEFT_ARM_BOTTOM_POSITION + ARM_DIFF, RIGHT_ARM_BOTTOM_POSITION - ARM_DIFF);
            sleep(500);
            Util.linearArmAutonCS(leftSlide, rightSlide, LINEAR_SLIDE_POSITION_2);
            sleep(500);
            leftFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            leftBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            while (sensorRange.getDistance(DistanceUnit.CM) > 2.5){
                leftFrontMotor.setPower(0.3);
                leftBackMotor.setPower(0.3);
                rightFrontMotor.setPower(0.3);
                rightBackMotor.setPower(0.3);
            }
            leftFrontMotor.setPower(0);
            leftBackMotor.setPower(0);
            rightFrontMotor.setPower(0);
            rightBackMotor.setPower(0);

            leftFrontMotor.setPower(DRIVE_SPEED);
            rightFrontMotor.setPower(DRIVE_SPEED);
            leftBackMotor.setPower(DRIVE_SPEED);
            rightBackMotor.setPower(DRIVE_SPEED);

            leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            Util.servos(leftClaw, rightClaw, LEFT_CLAW_INITIAL_POSITION + CLAW_RETRACT_DIFF, RIGHT_CLAW_INITIAL_POSITION - CLAW_RETRACT_DIFF);
            sleep(500);
            Util.encoderDriveBackward(10, motors);
            sleep(500);
            Util.linearArmAutonCS(leftSlide, rightSlide, LINEAR_SLIDE_POSITION_0);
            sleep(500);
            Util.servos(leftArm, rightArm, LEFT_ARM_BOTTOM_POSITION + 0.1, RIGHT_ARM_BOTTOM_POSITION - 0.1);
            sleep(500);
            Util.encoderDriveLeft(30, motors);
            Util.encoderDriveForward(20, motors);
            break;

        }
    }
}

