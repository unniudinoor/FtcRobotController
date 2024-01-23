package org.firstinspires.ftc.teamcode.tests;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.EruditeUtils;


@TeleOp(name="MotorTest")
public class MotorTest extends LinearOpMode {


    private final ElapsedTime runtime = new ElapsedTime();

    // motors for the wheels
    private DcMotor motor1 = null;
    private DcMotor motor2 = null;

    EruditeUtils utilities = new EruditeUtils();

    int linearLevel = 0; // level of linear slide

    boolean leftBumper = false;
    boolean rightBumper = false;


    @Override
    public void runOpMode() throws InterruptedException{
        motor1 = hardwareMap.get(DcMotor.class, "motor1");
        motor2 = hardwareMap.get(DcMotor.class, "motor2");


        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();


        while (opModeIsActive()) {
            boolean finger = false;

            if(gamepad1.a){
                motor1.setPower(0.1);
                motor2.setPower(0.1);
            }
            if(gamepad1.b){
                motor1.setPower(0.2);
                motor2.setPower(0.2);
            }
            if(gamepad1.x){
                motor1.setPower(1);
                motor2.setPower(1);
            }
            if(gamepad1.y){
                motor1.setPower(0);
                motor2.setPower(0);
            }
        }
    }

}
