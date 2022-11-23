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

        double left_claw_pos = 0.6; // left claw: 1 to 0
        double right_claw_pos = 0.35; // right claw: 0 to 1

        // expected final position
        // left position: 0.33
        // right position: 0.62
        double diff_claw = 0.17;

        double drive_sens = 0.5;

        int linearLevel = 0; // level of linear slide

        int base_level = 69;
        int lvl1_pole = 669;
        int lvl2_pole = 1420;
        int lvl3_pole = 2048;
       // int[] levels = {69, 669, 1420, 2048};

        boolean leftBumper = false;
        boolean rightBumper = false;

        // run until the end of the match (driver presses STOP

        while (opModeIsActive()) {
            boolean finger = false;

            int position = LinearSlide.getCurrentPosition();
            telemetry.addData("Encoder Position", position);
            if (gamepad1.right_bumper) {
                rightBumper = true;
            }
            if (!gamepad1.right_bumper && rightBumper) {
//                telemetry.addData("GP1 Input", "Right Bumper");
//                telemetry.addData("Encoder Position", position);
                telemetry.addData("Increased Level", linearLevel);
                if(linearLevel < 3){
                    linearLevel++;
                }
                rightBumper = false;
//                telemetry.addData("Linear Position", linearLevel);
            }
            if (gamepad1.left_bumper) {
                leftBumper = true;
            }
            if (leftBumper && !gamepad1.left_bumper) {
//                telemetry.addData("GP1 Input", "Left Bumper");
//                telemetry.addData("Encoder Position", position);
                if (linearLevel > 0){
                    linearLevel--;
                }
                leftBumper = false;
                telemetry.addData("Decreased Level", linearLevel);
//                telemetry.addData("Linear Position", linearLevel);
            }

            if  (linearLevel == 0){
                    LinearSlide.setTargetPosition(base_level);
                    LinearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            }
            if (linearLevel == 1){
                    LinearSlide.setTargetPosition(lvl1_pole);
                    LinearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            if (linearLevel == 2){
                    LinearSlide.setTargetPosition(lvl2_pole);
                    LinearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            if (linearLevel == 3) {
                LinearSlide.setTargetPosition(lvl3_pole);
                LinearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }

            if (gamepad1.a) {
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