package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;

public class EruditeUtils {
    public void claw(Servo leftClaw, Servo rightClaw, double left_claw_position,
                     double right_claw_position, double diffClaw){
        leftClaw.setPosition(left_claw_position - diffClaw);
        rightClaw.setPosition(right_claw_position + diffClaw);
    }

    public void linearArm(DcMotor slideMotor, int linearLevel, int [] levels){
        slideMotor.setPower(0.6);
        slideMotor.setTargetPosition(levels[linearLevel]);
        slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
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

        for (int i = 0; i < 4; i+=1){
            motors[i].setTargetPosition(newTarget[i]);
        }

        for (int i = 0; i < 4; i+=1){
            motors[i].setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

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

    }
}

