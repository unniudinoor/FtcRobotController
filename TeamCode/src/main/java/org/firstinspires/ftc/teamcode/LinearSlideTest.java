package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="LinearSlideTest")
public class LinearSlideTest extends LinearOpMode {

    static final double LEFT_LINEAR_SLIDE_POWER = 0.75;
    static final double RIGHT_LINEAR_SLIDE_POWER = LEFT_LINEAR_SLIDE_POWER * 312/435;
    static final double ENCODER_RATIO = 537.7/384.5;

    static final int LINEAR_SLIDE_POSITION_0 = 50;
    static final int LINEAR_SLIDE_POSITION_1 = 1600;
    static final int LINEAR_SLIDE_POSITION_2 = 2800;
    static final int LINEAR_SLIDE_POSITION_3 = 4000;

    static final int LINEAR_SLIDE_MAX_POSITION = 3;
    static final int LINEAR_SLIDE_MIN_POSITION = 0;

    private final ElapsedTime runtime = new ElapsedTime();

    // motor for linear slide
    private DcMotor leftSlide = null;
    private DcMotor rightSlide = null;

    EruditeUtils utilities = new EruditeUtils();

    int linearLevel = 0; // level of linear slide

    int[] leftLevels = {
            50, 1600,
            2800, 4000
    };
    int[] rightLevels = {
            36, 1144,
            2002, 2860
    };// linear slide positions for each pole

    boolean leftBumper = false;
    boolean rightBumper = false;


    public void LinearSlideUp() {
        if(linearLevel < LINEAR_SLIDE_MAX_POSITION){
            linearLevel += 1;
        }
        rightBumper = false;
    }

    public void LinearSlideDown() {
        if (linearLevel > LINEAR_SLIDE_MIN_POSITION){
            linearLevel -= 1;
        }
        leftBumper = false;
    }

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

            utilities.linearArmTestManual(leftSlide, linearLevel, leftLevels, LEFT_LINEAR_SLIDE_POWER);
            utilities.linearArmTestManual(rightSlide, linearLevel, rightLevels, RIGHT_LINEAR_SLIDE_POWER);


        }
    }
}
