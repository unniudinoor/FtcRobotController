package org.firstinspires.ftc.teamcode.tests;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.EruditeUtils;

@TeleOp(name="LinearSlideTest")
public class LinearSlideTest extends LinearOpMode {

    static final double LEFT_LINEAR_SLIDE_POWER = 0.75;
    static final double RIGHT_LINEAR_SLIDE_POWER = LEFT_LINEAR_SLIDE_POWER * 312/435;

    static final int LINEAR_SLIDE_POSITION_0 = 0;
    static final int LINEAR_SLIDE_POSITION_1 = 1500;
    static final int LINEAR_SLIDE_POSITION_2 = 2500;

    static final int LINEAR_SLIDE_MAX_POSITION = 2;
    static final int LINEAR_SLIDE_MIN_POSITION = 0;

    private final ElapsedTime runtime = new ElapsedTime();

    // motor for linear slide
    private DcMotorEx leftSlide = null;
    private DcMotorEx rightSlide = null;

    EruditeUtils utilities = new EruditeUtils();

    int linearLevel = 0; // level of linear slide

    int[] leftLevels = {
            LINEAR_SLIDE_POSITION_0, LINEAR_SLIDE_POSITION_1,
            LINEAR_SLIDE_POSITION_2
    };

    boolean leftBumper = false;
    boolean rightBumper = false;


    public void LinearSlideUp() {
        if(linearLevel < LINEAR_SLIDE_MAX_POSITION){
            linearLevel+=1;
        }
        rightBumper = false;
    }

    public void LinearSlideDown() {
        if (linearLevel > LINEAR_SLIDE_MIN_POSITION){
            linearLevel-=1;
        }
        leftBumper = false;
    }

    @Override
    public void runOpMode() throws InterruptedException{
        rightSlide = hardwareMap.get(DcMotorEx.class, "RightLinearSlideMotor");
        leftSlide = hardwareMap.get(DcMotorEx.class, "LeftLinearSlideMotor");


        // prepare DCmotor direction
        leftSlide.setDirection(DcMotor.Direction.REVERSE);
        rightSlide.setDirection(DcMotor.Direction.FORWARD);

        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        boolean leftReached = false;
        boolean rightReached = false;
        double leftBuffer = 0;
        double rightBuffer = 0;

        int currentPosLeftSlide = 0;
        int currentPosRightSlide = 0;

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            boolean finger = false;

            if (gamepad1.right_bumper) {
                rightBumper = true;
            }
            if (!gamepad1.right_bumper && rightBumper) {
                LinearSlideUp();
                currentPosLeftSlide = leftLevels[linearLevel];
                currentPosRightSlide = leftLevels[linearLevel];
            }

            if (gamepad1.left_bumper) {
                leftBumper = true;
            }
            if (leftBumper && !gamepad1.left_bumper) {
                LinearSlideDown();
                currentPosLeftSlide = leftLevels[linearLevel];
                currentPosRightSlide = leftLevels[linearLevel];
            }

            if(gamepad1.y) {
                leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }


            utilities.linearArmCenterStage(leftSlide, currentPosLeftSlide, LEFT_LINEAR_SLIDE_POWER);
            utilities.linearArmCenterStage(rightSlide, currentPosRightSlide, LEFT_LINEAR_SLIDE_POWER);

            telemetry.addData("LeftSLideSpeed", leftSlide.getVelocity());
            telemetry.addData("LeftSLidePos", leftSlide.getCurrentPosition());
            telemetry.addData("RightSLideSpeed", rightSlide.getVelocity());
            telemetry.addData("RightSLidePos", rightSlide.getCurrentPosition());
            telemetry.addData("leftReached", leftReached);
            telemetry.addData("rightReached", rightReached);
            telemetry.update();


        }
    }
}
