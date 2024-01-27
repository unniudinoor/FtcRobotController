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
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;


@Autonomous(name="AutonFarBlueRR", group="Robot")
public class AutonFarBlueRR extends LinearOpMode {

    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    private static final String TFOD_MODEL_ASSET = "model_blue.tflite";

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
    private Servo arm = null;
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

    static final double ARM_BOTTOM_POSITION = 0.61;
    static final double ARM_TOP_POSITION = 0;

    static final double DRIVE_SPEED = 0.8;


    @Override
    public void runOpMode() {

        // Initialize the drive system variables.
        rightSlide = hardwareMap.get(DcMotor.class, "RightLinearSlideMotor");
        leftSlide = hardwareMap.get(DcMotor.class, "LeftLinearSlideMotor");
        arm = hardwareMap.get(Servo.class, "ArmServo");
        leftClaw = hardwareMap.get(Servo.class, "LeftClaw Servo");
        rightClaw = hardwareMap.get(Servo.class, "RightClaw Servo");
        leftSlide.setDirection(DcMotor.Direction.REVERSE);
        rightSlide.setDirection(DcMotor.Direction.FORWARD);

        leftSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);


        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        leftClaw.setPosition(LEFT_CLAW_INITIAL_POSITION);
        rightClaw.setPosition(RIGHT_CLAW_INITIAL_POSITION);

        Trajectory leftTraj1 = drive.trajectoryBuilder(new Pose2d())
                .lineToSplineHeading(new Pose2d(-26.25, 10, Math.toRadians(-90)))
                .build();

        Trajectory leftTraj2 = drive.trajectoryBuilder(leftTraj1.end())
                .forward(7)
                .build();


        Trajectory leftTraj3 = drive.trajectoryBuilder(leftTraj2.end())
                .lineToConstantHeading(new Vector2d(-26.25, -80))
                .build();

        Trajectory leftTraj4 = drive.trajectoryBuilder(leftTraj3.end())
                .lineToConstantHeading(new Vector2d(-21, -83))
                .build();

        Trajectory leftTraj41 = drive.trajectoryBuilder(leftTraj4.end())
                .forward(5)
                .build();

        Trajectory leftTraj5 = drive.trajectoryBuilder(leftTraj41.end())
                .back(5)
                .build();


        Trajectory centerTraj1 = drive.trajectoryBuilder(new Pose2d())
                .lineToConstantHeading(new Vector2d(-19.25,12))
                .build();

        Trajectory centerTraj2 = drive.trajectoryBuilder(centerTraj1.end())
                .lineToSplineHeading(new Pose2d(-25.25, 2, Math.toRadians(-180)))
                .build();

        Trajectory centerTraj21 = drive.trajectoryBuilder(centerTraj2.end())
                .lineToConstantHeading(new Vector2d(-20.25, 2))
                .build();


        Trajectory centerTraj3 = drive.trajectoryBuilder(centerTraj21.end())
                .lineToSplineHeading(new Pose2d(-26.25, 6, Math.toRadians(-90)))
                .build();


        Trajectory centerTraj4 = drive.trajectoryBuilder(centerTraj3.end())
                .lineToConstantHeading(new Vector2d(-27, -84))
                .build();


        Trajectory centerTraj5 = drive.trajectoryBuilder(centerTraj4.end())
                .back(5)
                .build();



        Trajectory rightTraj1 = drive.trajectoryBuilder(new Pose2d())
                .lineToSplineHeading(new Pose2d(-31.25, 10, Math.toRadians(90)))
                .build();


        Trajectory rightTraj2 = drive.trajectoryBuilder(rightTraj1.end())
                .back(9)
                .build();

        Trajectory rightTraj21 = drive.trajectoryBuilder(rightTraj2.end())
                .strafeRight(5)
                .build();

        Trajectory rightTraj3 = drive.trajectoryBuilder(rightTraj21.end())
                .lineToConstantHeading(new Vector2d(-26.25, -80))
                .build();

        Trajectory rightTraj4 = drive.trajectoryBuilder(rightTraj3.end())
                .lineToSplineHeading(new Pose2d(-34.25, -79, Math.toRadians(-90)))
                .build();

        Trajectory rightTraj41 = drive.trajectoryBuilder(rightTraj4.end())
                .forward(5)
                .build();

        Trajectory rightTraj5 = drive.trajectoryBuilder(rightTraj41.end())
                .back(5)
                .build();


        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        boolean detected = false;
        double t = 0;

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
                int new_position = 3;
                int current_position = 0;
//                int counter = 0;
                //insert code to detect object
                sleep(6000);
                while (!detected) {
                    current_position = new_position;
                    new_position = telemetryTfod();
//                    if (current_position == new_position) {
//                        counter++;
//                        telemetry.addLine(" if counter:" + counter);
//                    } else {
//                        counter = 0;
//                        telemetry.addLine("else counter:" + counter);
//                    }
                    detected = true;
                    telemetry.update();

//                    if (counter > 10) {
//                        detected = true;
//                    }
//                    sleep(20);

                }
                if (new_position == 1) {

                    drive.followTrajectory(leftTraj1);
                    arm.setPosition(ARM_BOTTOM_POSITION - 0.1);
                    sleep(200);
                    leftSlide.setPower(-0.5);
                    rightSlide.setPower(-0.5);

                    t = getRuntime();

                    while (t > getRuntime() - 1.5);

                    leftSlide.setPower(0);
                    rightSlide.setPower(0);

                    arm.setPosition(ARM_BOTTOM_POSITION - 0.05);

                    drive.followTrajectory(leftTraj2);

                    arm.setPosition(ARM_BOTTOM_POSITION);
                    sleep(500);

                    rightClaw.setPosition(RIGHT_CLAW_INITIAL_POSITION - CLAW_RETRACT_DIFF);
                    sleep(500);
                    arm.setPosition(ARM_BOTTOM_POSITION - 0.15);
                    sleep(500);

                    drive.followTrajectory(leftTraj3);
                    drive.followTrajectory(leftTraj4);
                    drive.followTrajectory(leftTraj41);

                    leftClaw.setPosition(LEFT_CLAW_INITIAL_POSITION + CLAW_RETRACT_DIFF);
                    sleep(500);

                    drive.followTrajectory(leftTraj5);

                } else if (new_position == 2) {

                    drive.followTrajectory(centerTraj1);
                    arm.setPosition(ARM_BOTTOM_POSITION - 0.1);
                    sleep(200);
                    leftSlide.setPower(-0.5);
                    rightSlide.setPower(-0.5);

                    t = getRuntime();

                    while (t > getRuntime() - 1.5);

                    leftSlide.setPower(0);
                    rightSlide.setPower(0);

                    arm.setPosition(ARM_BOTTOM_POSITION - 0.05);


                    drive.followTrajectory(centerTraj2);

                    arm.setPosition(ARM_BOTTOM_POSITION);
                    sleep(500);
                    rightClaw.setPosition(RIGHT_CLAW_INITIAL_POSITION - CLAW_RETRACT_DIFF);
                    sleep(500);
                    arm.setPosition(ARM_BOTTOM_POSITION - 0.15);

                    drive.followTrajectory(centerTraj21);

                    drive.followTrajectory(centerTraj3);
                    drive.followTrajectory(centerTraj4);

                    leftClaw.setPosition(LEFT_CLAW_INITIAL_POSITION + CLAW_RETRACT_DIFF);
                    sleep(500);

                    drive.followTrajectory(centerTraj5);

                } else if (new_position == 3) {
                    drive.followTrajectory(rightTraj1);
                    arm.setPosition(ARM_BOTTOM_POSITION - 0.1);
                    sleep(200);
                    leftSlide.setPower(-0.5);
                    rightSlide.setPower(-0.5);

                    t = getRuntime();

                    while (t > getRuntime() - 1.5);

                    leftSlide.setPower(0);
                    rightSlide.setPower(0);

                    arm.setPosition(ARM_BOTTOM_POSITION - 0.05);


                    drive.followTrajectory(rightTraj2);

                    arm.setPosition(ARM_BOTTOM_POSITION);
                    sleep(500);
                    rightClaw.setPosition(RIGHT_CLAW_INITIAL_POSITION - CLAW_RETRACT_DIFF);
                    sleep(500);
                    arm.setPosition(ARM_BOTTOM_POSITION - 0.15);

                    drive.followTrajectory(rightTraj21);
                    drive.followTrajectory(rightTraj3);
                    drive.followTrajectory(rightTraj4);
                    drive.followTrajectory(rightTraj41);

                    leftClaw.setPosition(LEFT_CLAW_INITIAL_POSITION + CLAW_RETRACT_DIFF);
                    sleep(500);

                    drive.followTrajectory(rightTraj5);
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
                if (recognition.getRight() - recognition.getLeft() < 180) {
                    if (x > 0 && x < 190) {
                        position = 1;
                        //telemetry.addLine("left:" + position);
                        telemetry.addData("left", position);
                    } else if (x >= 190 && x < 590) {
                        position = 2;
                        //telemetry.addData("middle",position);
                        telemetry.addData("middle", position);
                    }
                }
            }

            telemetry.addData("Position", position);
            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            telemetry.addData("- Position", "%.0f / %.0f", x, y);
            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
        }   // end for() loop
        telemetry.addData("hi",position);
        return position;
    }   // end method telemetryTfod()
}