/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file illustrates the concept of driving a path based on time.
 * The code is structured as a LinearOpMode
 *
 * The code assumes that you do NOT have encoders on the wheels,
 *   otherwise you would use: RobotAutoDriveByEncoder;
 *
 *   The desired path in this example is:
 *   - Drive forward for 3 seconds
 *   - Spin right for 1.3 seconds
 *   - Drive Backward for 1 Second
 *
 *  The code is written in a simple form with no optimizations.
 *  However, there are several ways that this type of sequence could be streamlined,
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@Autonomous(name="AutonomousNoPole", group="Robot")
public class AutonomousNoPole extends LinearOpMode {

    /* Declare OpMode members. */
    private DcMotor LeftFrontMotor = null;
    private DcMotor RightFrontMotor = null;
    private DcMotor LeftBackMotor = null;
    private DcMotor RightBackMotor = null;
    private DcMotor LinearSlideMotor = null;
    private Servo LeftClaw = null;
    private Servo RightClaw = null;
    private ColorSensor color_sensor = null;
    EruditeUtils Util = new EruditeUtils();

    private ElapsedTime runtime = new ElapsedTime();


    static final double Drive_Speed = 0.4;
    static final double Turn_Speed = 0.5;


    int[] levels = {50, 1800, 3000, 4200};
    int parking_position;
    double distanceToPosition;
    double left_claw_pos = 0.6; // left claw: 1 to 0
    double right_claw_pos = 0.35; // right claw: 0 to 1
    double diffClaw = 0.2;

    @Override
    public void runOpMode() {

        // Initialize the drive system variables.
        LeftFrontMotor = hardwareMap.get(DcMotor.class, "LeftFrontMotor");
        RightFrontMotor = hardwareMap.get(DcMotor.class, "RightFrontMotor");
        LeftBackMotor = hardwareMap.get(DcMotor.class, "LeftBackMotor");
        RightBackMotor = hardwareMap.get(DcMotor.class, "RightBackMotor");
        LinearSlideMotor = hardwareMap.get(DcMotor.class, "LinearSlideMotor");
        LeftClaw = hardwareMap.get(Servo.class, "LeftClaw Servo");
        RightClaw = hardwareMap.get(Servo.class, "RightClaw Servo");
        color_sensor = hardwareMap.get(ColorSensor.class, "Color Sensor");

        LeftFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        LeftBackMotor.setDirection(DcMotor.Direction.REVERSE);
        RightFrontMotor.setDirection(DcMotor.Direction.FORWARD);
        RightBackMotor.setDirection(DcMotor.Direction.FORWARD);
        LinearSlideMotor.setDirection(DcMotor.Direction.REVERSE);

        LinearSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LeftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LeftBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        DcMotor[] motors = {LeftFrontMotor, RightFrontMotor, LeftBackMotor, RightBackMotor};
        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips


        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Step through each leg of the path, ensuring that the Auto mode has not been stopped along the way

        // Step 1:  Drive forward for 3 seconds

        LeftFrontMotor.setPower(Drive_Speed);
        RightFrontMotor.setPower(Drive_Speed);
        LeftBackMotor.setPower(Drive_Speed);
        RightBackMotor.setPower(Drive_Speed);

        LeftFrontMotor.setTargetPosition(LeftFrontMotor.getCurrentPosition());
        LeftBackMotor.setTargetPosition(LeftFrontMotor.getCurrentPosition());
        RightFrontMotor.setTargetPosition(LeftFrontMotor.getCurrentPosition());
        RightBackMotor.setTargetPosition(LeftFrontMotor.getCurrentPosition());

        LeftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LeftBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RightBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        runtime.reset();
        while (opModeIsActive()) {
            sleep(100);
            color_sensor.enableLed(true);

            // to move back to pick up cone (not needed)

            Util.linearArmAutonomous(LinearSlideMotor, levels[0]);
//            Util.linearArmAutonomous(LinearSlideMotor, 1, levels);
            Util.encoderDriveBackward(17, motors);
            sleep(1000);

            parking_position = Util.coneSleeve(color_sensor);
            distanceToPosition = Util.parkingPosNoCone(parking_position);

            telemetry.addData("parking position", parking_position);
            telemetry.update();

            color_sensor.enableLed(false);

            Util.encoderDriveBackward(34, motors);

            Util.encoderDriveRight(distanceToPosition, motors);

            break;
        }
    }
}




