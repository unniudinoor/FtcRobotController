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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
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

@Autonomous(name="AutonomousBasicMV", group="Robot")
public class AutonomousBasicMovement extends LinearOpMode {

    /* Declare OpMode members. */
    private DcMotor LeftFrontMotor = null;
    private DcMotor RightFrontMotor = null;
    private DcMotor LeftBackMotor = null;
    private DcMotor RightBackMotor = null;
    private DcMotor LinearSlideMotor = null;
    EruditeUtils Util = new EruditeUtils();

    private ElapsedTime runtime = new ElapsedTime();


    static final double Drive_Speed = 0.6;
    static final double Turn_Speed = 0.5;
    DcMotor[] motors = {LeftFrontMotor, RightFrontMotor, LeftBackMotor, RightBackMotor};
    static final double linear_slide_speed = 0.4;
    int[] levels = {50, 1800, 3000, 4200};

    @Override
    public void runOpMode() {

        // Initialize the drive system variables.
        LeftFrontMotor = hardwareMap.get(DcMotor.class, "LeftFrontMotor");
        RightFrontMotor = hardwareMap.get(DcMotor.class, "RightFrontMotor");
        LeftBackMotor = hardwareMap.get(DcMotor.class, "LeftBackMotor");
        RightBackMotor = hardwareMap.get(DcMotor.class, "RightBackMotor");
        LinearSlideMotor = hardwareMap.get(DcMotor.class, "LinearSlideMotor");
        LeftFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        LeftBackMotor.setDirection(DcMotor.Direction.REVERSE);
        RightFrontMotor.setDirection(DcMotor.Direction.FORWARD);
        RightBackMotor.setDirection(DcMotor.Direction.FORWARD);
        LinearSlideMotor.setDirection(DcMotor.Direction.REVERSE);
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
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < 5.0) {
            Util.encoderDriveForward(4, motors);
            Util.linearArm(LinearSlideMotor, 1, levels);
        }
    }
}

//        while (opModeIsActive() && (runtime.seconds() < 2.0)) {
//            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
//            telemetry.update();
//        }
//
//        // Step 3:  Drive Backward for 1 Second
//        LeftBackMotor.setPower(Turn_Speed);
//        LeftFrontMotor.setPower(Turn_Speed);
//        RightBackMotor.setPower(-Turn_Speed);
//        RightFrontMotor.setPower(-Turn_Speed);
//        runtime.reset();
//        while (opModeIsActive() && (runtime.seconds() < 1)) {
//            telemetry.addData("Path", "Leg 3: %4.1f S Elapsed", runtime.seconds());
//            telemetry.update();
//        }
//
//
//        LinearSlideMotor.setPower(linear_slide_speed);
//        runtime.reset();
//        while (opModeIsActive() && (runtime.seconds() < 2.0)) {
//            telemetry.addData("Path", "Leg 3: %4.1f S Elapsed", runtime.seconds());
//            telemetry.update();
//        }
//
//        LinearSlideMotor.setPower(-linear_slide_speed);
//        runtime.reset();
//        while (opModeIsActive() && (runtime.seconds() < 2.0 )) {
//            telemetry.addData("Path", "Leg 3: %4.1f S Elapsed", runtime.seconds());
//            telemetry.update();
//        }
//        LeftBackMotor.setPower(FORWARD_SPEED);
//        LeftFrontMotor.setPower(FORWARD_SPEED);
//        RightBackMotor.setPower(FORWARD_SPEED);
//        RightFrontMotor.setPower(FORWARD_SPEED);
//        runtime.reset();
//        while (opModeIsActive() && (runtime.seconds() < 2.0)) {
//            telemetry.addData("Path", "Leg 3: %4.1f S Elapsed", runtime.seconds());
//            telemetry.update();
//        }
//        // Step 4:  Stop
//        LeftFrontMotor.setPower(0);
//        RightFrontMotor.setPower(0);
//        LeftBackMotor.setPower(0);
//        RightBackMotor.setPower(0);


