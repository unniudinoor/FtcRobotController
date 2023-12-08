package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;


@Autonomous(name="CenterStageCloseRed", group="Robot")
public class CenterStageAutonCloseRed extends LinearOpMode {

    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    private static final String TFOD_MODEL_ASSET = "model_20231205_001401.tflite";

    private static final String[] LABELS = {
            "Beacon 1",
    };

    /**
     * The variable to store our instance of the TensorFlow Object Detection processor.
     */
    private TfodProcessor tfod;

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;

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

    static final double LEFT_CLAW_INITIAL_POSITION = 0.21;
    static final double RIGHT_CLAW_INITIAL_POSITION = 0.425;
    static final double CLAW_RETRACT_DIFF = 0.15;

    static final double LEFT_ARM_BOTTOM_POSITION = 0.19;
    static final double RIGHT_ARM_BOTTOM_POSITION = 0.79;
    static final double ARM_DIFF = 0.69;

    static final double DRIVE_SPEED = 0.8;


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

        boolean detected = false;

        // Create the TensorFlow processor by using a builder.
        tfod = new TfodProcessor.Builder()

                .setModelAssetName(TFOD_MODEL_ASSET)
                .setModelLabels(LABELS)
                .build();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        // Set and enable the processor.
        builder.addProcessor(tfod);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Set confidence threshold for TFOD recognitions, at any time.
        tfod.setMinResultConfidence(0.75f);

        // Disable or re-enable the TFOD processor at any time.
        visionPortal.setProcessorEnabled(tfod, true);


        runtime.reset();
        if (opModeIsActive()) {
            telemetry.addData("Works", "1");
            try {
                int new_position = 1;
                int current_position = 0;
                int counter = 0;
                //insert code to detect object
                sleep(5000);
                while (!detected) {
                    current_position = new_position;
                    new_position = telemetryTfod();
                    if (current_position == new_position) {
                        counter++;
                        telemetry.addLine(" if counter:" + counter);
                    } else {
                        counter = 0;
                        telemetry.addLine("else counter:" + counter);
                    }

                    telemetry.update();

                    if (counter > 25) {
                        detected = true;
                    }
                    sleep(20);

                }
                if (new_position == 1) {
                    Util.servos(leftClaw, rightClaw, LEFT_CLAW_INITIAL_POSITION, RIGHT_CLAW_INITIAL_POSITION);
                    sleep(500);
                    Util.encoderDriveForward(5, motors);
                    Util.servos(leftArm, rightArm, LEFT_ARM_BOTTOM_POSITION + 0.1, RIGHT_ARM_BOTTOM_POSITION - 0.1);
                    Util.encoderDriveLeft(15, motors);
                    Util.encoderDriveForward(20, motors);
                    Util.encoderRotate(100, motors);
                    Util.servos(leftArm, rightArm, LEFT_ARM_BOTTOM_POSITION, RIGHT_ARM_BOTTOM_POSITION);
                    Util.encoderDriveBackward(8, motors);
                    sleep(100);
                    rightClaw.setPosition(RIGHT_CLAW_INITIAL_POSITION - CLAW_RETRACT_DIFF);
                    sleep(500);
                    Util.encoderDriveForward(5, motors);
                    Util.servos(leftArm, rightArm, LEFT_ARM_BOTTOM_POSITION + 0.1, RIGHT_ARM_BOTTOM_POSITION - 0.1);
                    Util.encoderDriveLeft(6, motors);
                    Util.encoderDriveForward(22, motors);
                    Util.encoderRotate(-200, motors);
                    Util.servos(leftArm, rightArm, LEFT_ARM_BOTTOM_POSITION + 0.25, RIGHT_ARM_BOTTOM_POSITION - 0.25);

                    Util.encoderDriveBackward(15, motors);
                    Util.servos(leftClaw, rightClaw, LEFT_CLAW_INITIAL_POSITION + CLAW_RETRACT_DIFF, RIGHT_CLAW_INITIAL_POSITION - CLAW_RETRACT_DIFF);
                    sleep(500);
                    Util.encoderDriveForward(5, motors);
                } else if (new_position == 2) {
                    telemetry.addData("center", "2");
                    Util.servos(leftClaw, rightClaw, LEFT_CLAW_INITIAL_POSITION, RIGHT_CLAW_INITIAL_POSITION);
                    sleep(500);
                    Util.encoderDriveForward(5, motors);
                    Util.servos(leftArm, rightArm, LEFT_ARM_BOTTOM_POSITION + 0.1, RIGHT_ARM_BOTTOM_POSITION - 0.1);
                    Util.encoderDriveLeft(5, motors);
                    Util.encoderDriveForward(17, motors);
                    Util.servos(leftArm, rightArm, LEFT_ARM_BOTTOM_POSITION, RIGHT_ARM_BOTTOM_POSITION);
                    Util.encoderRotate(-200, motors);
                    sleep(100);
                    rightClaw.setPosition(RIGHT_CLAW_INITIAL_POSITION - CLAW_RETRACT_DIFF);
                    sleep(500);
                    Util.encoderDriveForward(9, motors);
                    Util.servos(leftArm, rightArm, LEFT_ARM_BOTTOM_POSITION + 0.1, RIGHT_ARM_BOTTOM_POSITION - 0.1);
                    Util.encoderRotate(100, motors);
                    Util.encoderDriveRight(7, motors);
                    Util.servos(leftArm, rightArm, LEFT_ARM_BOTTOM_POSITION + 0.2, RIGHT_ARM_BOTTOM_POSITION - 0.2);
                    sleep(500);

                    leftFrontMotor.setPower(0.5);
                    rightFrontMotor.setPower(0.5);
                    leftBackMotor.setPower(0.5);
                    rightBackMotor.setPower(0.5);

                    Util.encoderDriveBackward(42, motors);

                    Util.servos(leftClaw, rightClaw, LEFT_CLAW_INITIAL_POSITION + CLAW_RETRACT_DIFF, RIGHT_CLAW_INITIAL_POSITION - CLAW_RETRACT_DIFF);
                    sleep(500);
                    leftFrontMotor.setPower(1);
                    rightFrontMotor.setPower(1);
                    leftBackMotor.setPower(1);
                    rightBackMotor.setPower(1);
                    Util.encoderDriveForward(5, motors);
                } else if (new_position == 3) {
                    Util.servos(leftClaw, rightClaw, LEFT_CLAW_INITIAL_POSITION, RIGHT_CLAW_INITIAL_POSITION);
                    sleep(500);
                    Util.encoderDriveForward(5, motors);
                    Util.servos(leftArm, rightArm, LEFT_ARM_BOTTOM_POSITION + 0.1, RIGHT_ARM_BOTTOM_POSITION - 0.1);
                    Util.encoderDriveLeft(30.5, motors);
                    Util.encoderDriveForward(20, motors);
                    Util.servos(leftArm, rightArm, LEFT_ARM_BOTTOM_POSITION, RIGHT_ARM_BOTTOM_POSITION);
                    Util.encoderRotate(100, motors);
                    rightClaw.setPosition(RIGHT_CLAW_INITIAL_POSITION - CLAW_RETRACT_DIFF);
                    sleep(200);
                    Util.encoderDriveForward(7, motors);
                    Util.servos(leftArm, rightArm, LEFT_ARM_BOTTOM_POSITION + 0.1, RIGHT_ARM_BOTTOM_POSITION - 0.1);
                    Util.encoderDriveRight(14, motors);
                    Util.encoderRotate(200, motors);
                    Util.servos(leftArm, rightArm, LEFT_ARM_BOTTOM_POSITION + 0.25, RIGHT_ARM_BOTTOM_POSITION - 0.25);
                    sleep(500);

                    leftFrontMotor.setPower(0.5);
                    rightFrontMotor.setPower(0.5);
                    leftBackMotor.setPower(0.5);
                    rightBackMotor.setPower(0.5);

                    Util.encoderDriveBackward(13, motors);

                    Util.servos(leftClaw, rightClaw, LEFT_CLAW_INITIAL_POSITION + CLAW_RETRACT_DIFF, RIGHT_CLAW_INITIAL_POSITION - CLAW_RETRACT_DIFF);
                    sleep(500);
                    Util.encoderDriveForward(5, motors);
                }
            } catch (Exception e) {
                telemetry.addData("Error:", e.toString());
                telemetry.update();
                throw e;
            }
        }
    }

    public int  telemetryTfod() {
        int position = 3;
        List<Recognition> currentRecognitions = tfod.getRecognitions();
        telemetry.addData("# Objects Detected", currentRecognitions.size());

        // Step through the list of recognitions and display info for each one.
        for (Recognition recognition : currentRecognitions) {
            double x = (recognition.getLeft() + recognition.getRight()) / 2;
            double y = (recognition.getTop() + recognition.getBottom()) / 2;
            //telemetry.addData("objects", currentRecognitions.size());
            //telemetry.update();
            if (currentRecognitions.size() > 0) {
                if (x > 0 && x < 190) {
                    position = 1;
                    //telemetry.addLine("left:" + position);
                    telemetry.addData("left", position);
                }

                else if (x >= 190 && x < 590) {
                    position = 2;
                    //telemetry.addData("middle",position);
                    telemetry.addData("middle",position);
                }
            }


            telemetry.addData(""," ");
            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            telemetry.addData("- Position", "%.0f / %.0f", x, y);
            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
        }   // end for() loop
        telemetry.addData("hi",position);
        return position;
    }   // end method telemetryTfod()
}