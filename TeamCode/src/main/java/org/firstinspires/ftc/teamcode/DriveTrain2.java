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
    private final ElapsedTime runtime = new ElapsedTime();
    private DcMotor LeftFrontMotor = null;
    private DcMotor LeftBackMotor = null;
    private DcMotor RightFrontMotor = null;
    private DcMotor RightBackMotor = null;
    private DcMotor LinearSlide = null;
    private Servo LeftClaw = null;
    private Servo RightClaw = null;

    // expected final position
    // left position: 0.33
    // right position: 0.62

    public void OpenClaw (){
        double left_claw_pos = 0.6; // left claw: 1 to 0
        double right_claw_pos = 0.35; // right claw: 0 to 1
        LeftClaw.setPosition(left_claw_pos);
        RightClaw.setPosition(right_claw_pos);
    }
    public void CloseClaw(){
        double left_claw_pos = 0.6; // left claw: 1 to 0
        double right_claw_pos = 0.35; // right claw: 0 to 1
        double diff_claw = 0.17;
        LeftClaw.setPosition(left_claw_pos - diff_claw);
        RightClaw.setPosition(right_claw_pos + diff_claw);
    }
    int linearLevel = 0; // level of linear slide

    int base_level = 50;
    int lvl1_pole = 1650;
    int lvl2_pole = 3000;
    int lvl3_pole = 4000;
    int[] levels = {base_level, lvl1_pole, lvl2_pole, lvl3_pole};
    boolean leftBumper = false;
    boolean rightBumper = false;


    public void LinearSlideUp() {
        if(linearLevel < 3){
            linearLevel++;
        }
        rightBumper = false;
    }
    public void LinearSlideDown() {
        if (linearLevel > 0){
            linearLevel--;
        }
        leftBumper = false;
    }
    public void UpdateSlidePosition() {
        LinearSlide.setTargetPosition(levels[linearLevel]);
        LinearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

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

        LinearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LinearSlide.setPower(0.4);
        LinearSlide.setTargetPosition(LinearSlide.getCurrentPosition());
        LinearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        double linear_slide_speed = 0.4;

        double drive_sens = 0.5;

        // run until the end of the match (driver presses STOP

        while (opModeIsActive()) {
            boolean finger = false;

            int position = LinearSlide.getCurrentPosition();
            telemetry.addData("Encoder Position", position);

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

            UpdateSlidePosition();

            if (gamepad1.a) {
                telemetry.addData("GP2 Input", "Button A");
                OpenClaw();
            }
            else if (gamepad1.b){
                telemetry.addData("GP2 Input", "Button B");
                CloseClaw();
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
            LeftFrontMotor.setPower(leftFrontPower * drive_sens);
            RightFrontMotor.setPower(rightFrontPower * drive_sens);
            LeftBackMotor.setPower(leftBackPower * drive_sens);
            RightBackMotor.setPower(rightBackPower * drive_sens);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            //telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            //telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.update();
        }
    }
}