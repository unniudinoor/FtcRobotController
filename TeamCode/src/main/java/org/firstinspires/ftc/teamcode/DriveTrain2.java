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


    @Override
    public void runOpMode() throws InterruptedException{

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

        telemetry.addData("Encoder Position: before", LinearSlide.getCurrentPosition());

        LinearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        telemetry.addData("Encoder Position: after", LinearSlide.getCurrentPosition());

        LinearSlide.setTargetPosition(LinearSlide.getCurrentPosition() + 100);
        LinearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        double linear_slide_speed = 0.4;

        double left_claw_pos = 0.5; // left claw: 1 to 0
        double right_claw_pos = 0.45; // right claw: 0 to 1

        // expected final position
        // left position: 0.33
        // right position: 0.62
        double diff_claw = 0.17;

        double strafe_sens = 0.5;

        double linear_position = 0; // level of linear slide

        int base_level = 0;
        int lvl1_pole = 669;
        int lvl2_pole = 1420;
        int lvl3_pole = 2048;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            boolean finger = false;

            int position = LinearSlide.getCurrentPosition();

            if (gamepad1.right_bumper) {
                telemetry.addData("GP1 Input", "Right Bumper");
                telemetry.addData("Encoder Position", position);
//                LinearSlide.setPower(linear_slide_speed);
                if (linear_position < 3){
                    linear_position += 1;
                }
            }
            else if (gamepad1.left_bumper) {
                telemetry.addData("GP1 Input", "Left Bumper");
                telemetry.addData("Encoder Position", position);
//                LinearSlide.setPower(-linear_slide_speed);
                if (linear_position > 0){
                    linear_position -= 1;
                }
            }

            if (linear_position == 0){
                LinearSlide.setTargetPosition(base_level);
                LinearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            else if (linear_position == 1){
                LinearSlide.setTargetPosition(lvl1_pole);
                LinearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            else if (linear_position == 2){
                LinearSlide.setTargetPosition(lvl2_pole);
                LinearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            else if (linear_position == 3){
                LinearSlide.setTargetPosition(lvl3_pole);
                LinearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }

            else {
                telemetry.addData("GP1 Input", "Not Input");
                telemetry.addData("Encoder Position", position);
                LinearSlide.setPower(0);
            }

            if(gamepad1.a) {
                telemetry.addData("GP2 Input", "Button A");
                LeftClaw.setPosition(left_claw_pos);
                RightClaw.setPosition(right_claw_pos);
            }
            else if (gamepad1.b){
                telemetry.addData("GP2 Input", "Button B");
                LeftClaw.setPosition(left_claw_pos - diff_claw);
                RightClaw.setPosition(right_claw_pos + diff_claw);
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
            LeftFrontMotor.setPower(leftFrontPower * strafe_sens);
            RightFrontMotor.setPower(rightFrontPower * strafe_sens);
            LeftBackMotor.setPower(leftBackPower * strafe_sens);
            RightBackMotor.setPower(rightBackPower * strafe_sens);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.update();

        }
    }
}