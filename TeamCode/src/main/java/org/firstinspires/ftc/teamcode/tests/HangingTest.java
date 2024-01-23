package org.firstinspires.ftc.teamcode.tests;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.EruditeUtils;

@TeleOp(name="HangingTest")
public class HangingTest extends LinearOpMode {

    static final double LINEAR_SLIDE_POWER = 1;


    private final ElapsedTime runtime = new ElapsedTime();

    // motor for linear slide
    private DcMotor leftSlide = null;
    private DcMotor rightSlide = null;

    EruditeUtils utilities = new EruditeUtils();

    boolean bumper = false;


    @Override
    public void runOpMode() throws InterruptedException{
        rightSlide = hardwareMap.get(DcMotor.class, "RightLinearSlideMotor");
        leftSlide = hardwareMap.get(DcMotor.class, "LeftLinearSlideMotor");


        // prepare DCmotor direction
        leftSlide.setDirection(DcMotor.Direction.REVERSE);
        rightSlide.setDirection(DcMotor.Direction.FORWARD);

        leftSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            boolean finger = false;

            if (gamepad1.right_bumper) {
                leftSlide.setPower(LINEAR_SLIDE_POWER);
                rightSlide.setPower(LINEAR_SLIDE_POWER);
            }
            else if (gamepad1.left_bumper) {
                leftSlide.setPower(-1 * LINEAR_SLIDE_POWER);
                rightSlide.setPower(-1 * LINEAR_SLIDE_POWER);
            }
            else {
                leftSlide.setPower(0);
                rightSlide.setPower(0);
            }
        }
    }
}
