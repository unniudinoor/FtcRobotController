package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;



@TeleOp(name="DriveTrain2")
public class DriveTrain2 extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor LeftFrontMotor = null;
    private DcMotor LeftBackMotor = null;
    private DcMotor RightFrontMotor = null;
    private DcMotor RightBackMotor = null;
    private DcMotor LinearSlide = null;
    private Servo LeftClaw = null;
    private Servo RightClaw = null;
    static final double LeftClawClose_Position = 5/90;
    static final double RightClawClose_Position = -5/90;

    @Override
    public void runOpMode() {

        LinearSlide = hardwareMap.get(DcMotor.class, "LinearSlideMotor");
        LeftClaw = hardwareMap.get(Servo.class, "LeftClaw Servo");
        RightClaw = hardwareMap.get(Servo.class, "RightClaw Servo");
        LeftFrontMotor = hardwareMap.get(DcMotor.class, "LeftFrontMotor");
        LeftBackMotor = hardwareMap.get(DcMotor.class, "LeftBackMotor");
        RightFrontMotor = hardwareMap.get(DcMotor.class, "RightFrontMotor");
        RightBackMotor = hardwareMap.get(DcMotor.class, "RightBackMotor");

        LeftFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        LeftBackMotor.setDirection(DcMotor.Direction.REVERSE);
        RightFrontMotor.setDirection(DcMotor.Direction.FORWARD);
        RightBackMotor.setDirection(DcMotor.Direction.FORWARD);
        LinearSlide.setDirection(DcMotor.Direction.REVERSE);
        LeftClaw.setDirection(Servo.Direction.FORWARD);
        RightClaw.setDirection(Servo.Direction.FORWARD);
        LinearSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            boolean finger = false;
            int position = LinearSlide.getCurrentPosition();

            if (gamepad1.right_bumper) {
                telemetry.addData("GP1 Input", "Right Bumper");
                telemetry.addData("Encoder Position", position);
                LinearSlide.setPower(0.4);

            }
            else if (gamepad1.left_bumper) {
                telemetry.addData("GP1 Input", "Left Bumper");
                telemetry.addData("Encoder Position", position);
                LinearSlide.setPower(-0.4);
            }
            else {
                telemetry.addData("GP1 Input", "Not Input");
                telemetry.addData("Encoder Position", position);
                LinearSlide.setPower(0);

            }
            // leftclaw range is 0.75(open) to 0.5(closed)
            // rightclaw range is 0.5(open) to 0.25(closed)
            //leftclaw: 1 to 0
            //right claw: 0 to 1
            if(gamepad1.a) {
                telemetry.addData("GP2 Input", "Button A");
                LeftClaw.setPosition(.75);
                RightClaw.setPosition(.25);
            }
            else if (gamepad1.b){
                telemetry.addData("GP2 Input", "Button B");
                LeftClaw.setPosition(0.65);
                RightClaw.setPosition(0.35);
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
            LeftFrontMotor.setPower(leftFrontPower);
            RightFrontMotor.setPower(rightFrontPower);
            LeftBackMotor.setPower(leftBackPower);
            RightBackMotor.setPower(rightBackPower);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.update();

        }}}