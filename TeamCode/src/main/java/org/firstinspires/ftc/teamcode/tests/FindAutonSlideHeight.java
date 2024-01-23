package org.firstinspires.ftc.teamcode.tests;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.EruditeUtils;

@TeleOp(name="AutonSlideTest")
public class FindAutonSlideHeight extends LinearOpMode {

    static final double LEFT_LINEAR_SLIDE_POWER = 0.75;
    static final double RIGHT_LINEAR_SLIDE_POWER = LEFT_LINEAR_SLIDE_POWER * 312/435;

    static final int LINEAR_SLIDE_POSITION_0 = 50;
    static final int LINEAR_SLIDE_POSITION_1 = 1500;
    static final int LINEAR_SLIDE_POSITION_2 = 2600;

    static final int LINEAR_SLIDE_MAX_POSITION = 2;
    static final int LINEAR_SLIDE_MIN_POSITION = 0;

    private final ElapsedTime runtime = new ElapsedTime();

    // motor for linear slide
    private DcMotor leftSlide = null;
    private DcMotor rightSlide = null;

    EruditeUtils utilities = new EruditeUtils();

    @Override
    public void runOpMode() throws InterruptedException{
        rightSlide = hardwareMap.get(DcMotor.class, "RightLinearSlideMotor");
        leftSlide = hardwareMap.get(DcMotor.class, "LeftLinearSlideMotor");


        // prepare DCmotor direction
        leftSlide.setDirection(DcMotor.Direction.REVERSE);
        rightSlide.setDirection(DcMotor.Direction.FORWARD);

        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            boolean finger = false;

            telemetry.addData("Left Slide Height", leftSlide.getCurrentPosition());
            telemetry.addData("Right Slide Height", rightSlide.getCurrentPosition());
            telemetry.update();

        }
    }
}
