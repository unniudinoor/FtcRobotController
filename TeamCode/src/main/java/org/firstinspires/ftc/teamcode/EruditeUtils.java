package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class EruditeUtils {
    public void claw(Servo leftClaw, Servo rightClaw, double left_claw_position,
                     double right_claw_position, double diffClaw){
        leftClaw.setPosition(left_claw_position - diffClaw);
        rightClaw.setPosition(right_claw_position + diffClaw);
    }

    public void linearArmManual(DcMotor slideMotor, int linearLevel, int [] levels){
        slideMotor.setPower(0.6);
        slideMotor.setTargetPosition(levels[linearLevel]);
        slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void linearArmAutonomous(DcMotor slideMotor, int linearLevel, int [] levels){
        slideMotor.setPower(0.8);
        slideMotor.setTargetPosition(levels[linearLevel]);
        slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (slideMotor.isBusy());
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
        newTarget[0] = motors[0].getCurrentPosition() + (int)(2 * degrees * 13.5 * 3.14159 * COUNTS_PER_INCH / 360);
        newTarget[1] = motors[1].getCurrentPosition() - (int)(2 * degrees * 13.5 * 3.14159 * COUNTS_PER_INCH / 360);
        newTarget[2] = motors[2].getCurrentPosition() + (int)(2 * degrees * 13.5 * 3.14159 * COUNTS_PER_INCH / 360);
        newTarget[3] = motors[3].getCurrentPosition() - (int)(2 * degrees * 13.5 * 3.14159 * COUNTS_PER_INCH / 360);

        for (int i = 0; i < 4; i+=1){
            motors[i].setTargetPosition(newTarget[i]);
        }

        for (int i = 0; i < 4; i+=1){
            motors[i].setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        while (motors[0].isBusy() || motors[1].isBusy() || motors[2].isBusy() || motors[3].isBusy());

    }
}

