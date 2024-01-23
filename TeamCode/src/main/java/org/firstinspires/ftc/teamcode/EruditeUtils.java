package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import java.lang.Math;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class EruditeUtils {
    public void launchDrone(Servo launcher, double servoStart, double servoDiff){
        launcher.setPosition(servoStart + servoDiff);
    }
    public int convertSlideValues(int left){
        return (int)Math.round(left * 384.5/537.7);
    }
    public void centerStageInitialize(DcMotor leftFront, DcMotor rightFront,
                                   DcMotor leftRear, DcMotor rightRear, DcMotor leftSlide, DcMotor rightSlide
    ) {
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightRear.setDirection(DcMotor.Direction.FORWARD);
        leftSlide.setDirection(DcMotor.Direction.REVERSE);
        rightSlide.setDirection(DcMotor.Direction.FORWARD);
    }
    public void initializeDCMotors(DcMotor leftFront, DcMotor rightFront,
                                   DcMotor leftRear, DcMotor rightRear, DcMotor linearSlide
    ) {
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightRear.setDirection(DcMotor.Direction.FORWARD);
        linearSlide.setDirection(DcMotor.Direction.REVERSE);
    }

    public void claw(Servo leftClaw, Servo rightClaw, double left_claw_position,
                     double right_claw_position, double diffClaw){
        leftClaw.setPosition(left_claw_position - diffClaw);
        rightClaw.setPosition(right_claw_position + diffClaw);
    }

    public void linearArmManual(DcMotor slideMotor, int linearLevel, int [] levels){
        slideMotor.setPower(0.8);
        slideMotor.setTargetPosition(levels[linearLevel]);
        slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void linearArmAutonCS(DcMotor leftSlide, DcMotor rightSlide, int position){
        leftSlide.setPower(0.8);
        rightSlide.setPower(0.8);
        leftSlide.setTargetPosition(position);
        rightSlide.setTargetPosition(position);
        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (leftSlide.isBusy() || rightSlide.isBusy());
    }
    public void linearArmTestManual(DcMotor slideMotor, int linearLevel, int [] levels, double power){
        slideMotor.setPower(power);
        slideMotor.setTargetPosition(levels[linearLevel]);
        slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void linearArmCenterStage(DcMotor slideMotor, int pos, double power){
        slideMotor.setPower(power);
        slideMotor.setTargetPosition(pos);
        slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void linearArmAutonomous(DcMotor slideMotor, int position){
        slideMotor.setPower(0.8);
        slideMotor.setTargetPosition(position);
        slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (slideMotor.isBusy());
    }

    public void servos (Servo left, Servo right, double leftPos, double rightPos){
        left.setPosition(leftPos);
        right.setPosition(rightPos);
    }

    public Integer coneSleeve (ColorSensor color_sensor) {
        if (color_sensor.blue() > color_sensor.red() && color_sensor.blue() > color_sensor.green()) {
//            telemetry.addData("Color Detected", "Blue: go to parking position 1");
            return 1;
        } else if (color_sensor.red() > color_sensor.blue() && color_sensor.red() > color_sensor.green()) {
//            telemetry.addData("Color Detected", "Red: go to parking position 2");
            return 2;
        } else if (color_sensor.green() > color_sensor.blue() && color_sensor.green() > color_sensor.red()) {
//            telemetry.addData("Color Detected", "Green: go to parking position 3");
            return 3;
        }
        else {
            return 4;
        }
    }
    public Integer strafeForParking (int parkingPosition){
        int defaultPosition = 66;
        int[] positionsInInches = {14, 40, 66, defaultPosition}; //defaulted to position 3
        return positionsInInches[parkingPosition - 1];
    }

    public void encoderDriveForward(double inches, DcMotor motors[]) {

        double     COUNTS_PER_MOTOR_REV    = 537.7 ;    // goBILDA Motor Encoder
        double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
        double     WHEEL_DIAMETER_INCHES   = 3.77953 ;     // For figuring circumference
        double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.14159);
        int [] newTarget = {0,0, 0, 0}; // stores target positions for motors


        // Determine new target position, and pass to motor controller
        newTarget[0] = motors[0].getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
        newTarget[1] = motors[1].getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
        newTarget[2] = motors[2].getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
        newTarget[3] = motors[3].getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);

//        telemetry = new Telemetry();

        for (int i = 0; i < 4; i+=1){
            motors[i].setTargetPosition(newTarget[i]);
        }

        for (int i = 0; i < 4; i+=1){
            motors[i].setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        while (motors[0].isBusy() || motors[1].isBusy() || motors[2].isBusy() || motors[3].isBusy());
    }
    public void encoderDriveBackward(double inches, DcMotor motors[]) {

        double     COUNTS_PER_MOTOR_REV    = 537.7;    // goBILDA Motor Encoder
        double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
        double     WHEEL_DIAMETER_INCHES   = 3.77953 ;     // For figuring circumference
        double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.14159);
        int [] newTarget = {0,0, 0, 0};


        // Determine new target position, and pass to motor controller
        newTarget[0] = motors[0].getCurrentPosition() - (int)(inches * COUNTS_PER_INCH);
        newTarget[1] = motors[1].getCurrentPosition() - (int)(inches * COUNTS_PER_INCH);
        newTarget[2] = motors[2].getCurrentPosition() - (int)(inches * COUNTS_PER_INCH);
        newTarget[3] = motors[3].getCurrentPosition() - (int)(inches * COUNTS_PER_INCH);

        for (int i = 0; i < 4; i+=1){
            motors[i].setTargetPosition(newTarget[i]);
        }

        for (int i = 0; i < 4; i+=1){
            motors[i].setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        while (motors[0].isBusy() || motors[1].isBusy() || motors[2].isBusy() || motors[3].isBusy());
    }
    public void encoderDriveBackwardInaccurate(double inches, DcMotor motors[]) {

        double     COUNTS_PER_MOTOR_REV    = 537.7;    // goBILDA Motor Encoder
        double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
        double     WHEEL_DIAMETER_INCHES   = 3.77953 ;     // For figuring circumference
        double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.14159);
        int [] newTarget = {0,0, 0, 0};


        // Determine new target position, and pass to motor controller
        newTarget[0] = motors[0].getCurrentPosition() - (int)(inches * COUNTS_PER_INCH);
        newTarget[1] = motors[1].getCurrentPosition() - (int)(inches * COUNTS_PER_INCH);
        newTarget[2] = motors[2].getCurrentPosition() - (int)(inches * COUNTS_PER_INCH);
        newTarget[3] = motors[3].getCurrentPosition() - (int)(inches * COUNTS_PER_INCH);

        for (int i = 0; i < 4; i+=1){
            motors[i].setTargetPosition(newTarget[i]);
        }

        for (int i = 0; i < 4; i+=1){
            motors[i].setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        while (motors[0].isBusy() && motors[1].isBusy() && motors[2].isBusy() && motors[3].isBusy());
    }
    public void encoderDriveRight(double inches, DcMotor motors[]) {

        double     COUNTS_PER_MOTOR_REV    = 537.7 ;    // goBILDA Motor Encoder
        double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
        double     WHEEL_DIAMETER_INCHES   = 3.77953 ;     // For figuring circumference
        double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.14159);
        int [] newTarget = {0,0, 0, 0}; // stores target positions for motors


        // Determine new target position, and pass to motor controller
        newTarget[0] = motors[0].getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
        newTarget[1] = motors[1].getCurrentPosition() - (int)(inches * COUNTS_PER_INCH);
        newTarget[2] = motors[2].getCurrentPosition() - (int)(inches * COUNTS_PER_INCH);
        newTarget[3] = motors[3].getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);

        for (int i = 0; i < 4; i+=1){
            motors[i].setTargetPosition(newTarget[i]);
        }

        for (int i = 0; i < 4; i+=1){
            motors[i].setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        // ensure movement to position is complete
        while (motors[0].isBusy() || motors[1].isBusy() || motors[2].isBusy() || motors[3].isBusy());

    }

    public void encoderDriveLeft(double inches, DcMotor motors[]) {

        double     COUNTS_PER_MOTOR_REV    = 537.7 ;    // goBILDA Motor Encoder
        double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
        double     WHEEL_DIAMETER_INCHES   = 3.77953 ;     // For figuring circumference
        double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.14159);
        int [] newTarget = {0,0, 0, 0}; // stores target positions for motors


        // Determine new target position, and pass to motor controller
        newTarget[0] = motors[0].getCurrentPosition() - (int)(inches * COUNTS_PER_INCH);
        newTarget[1] = motors[1].getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
        newTarget[2] = motors[2].getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
        newTarget[3] = motors[3].getCurrentPosition() - (int)(inches * COUNTS_PER_INCH);

        for (int i = 0; i < 4; i+=1){
            motors[i].setTargetPosition(newTarget[i]);
        }

        for (int i = 0; i < 4; i+=1){
            motors[i].setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        while (motors[0].isBusy() || motors[1].isBusy() || motors[2].isBusy() || motors[3].isBusy());

    }
    public void encoderRotate(double degrees, DcMotor motors[]) {

        double     COUNTS_PER_MOTOR_REV    = 537.7 ;    // goBILDA Motor Encoder
        double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
        double     WHEEL_DIAMETER_INCHES   = 3.77953 ;     // For figuring circumference
        double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.14159);
        int [] newTarget = {0,0, 0, 0}; // stores target positions for motors


        // Determine new target position, and pass to motor controller
        newTarget[0] = motors[0].getCurrentPosition() + (int)(2 * degrees * 16.5 * 3.14159 * COUNTS_PER_INCH / 360);
        newTarget[1] = motors[1].getCurrentPosition() - (int)(2 * degrees * 16.5 * 3.14159 * COUNTS_PER_INCH / 360);
        newTarget[2] = motors[2].getCurrentPosition() + (int)(2 * degrees * 16.5 * 3.14159 * COUNTS_PER_INCH / 360);
        newTarget[3] = motors[3].getCurrentPosition() - (int)(2 * degrees * 16.5 * 3.14159 * COUNTS_PER_INCH / 360);

        for (int i = 0; i < 4; i+=1){
            motors[i].setTargetPosition(newTarget[i]);
        }

        for (int i = 0; i < 4; i+=1){
            motors[i].setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        while (motors[0].isBusy() || motors[1].isBusy() || motors[2].isBusy() || motors[3].isBusy());



    }
}

