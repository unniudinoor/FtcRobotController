//package org.firstinspires.ftc.teamcode;
//
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.hardware.ColorSensor;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//public class TestColorSensor extends OpMode {
//    private ColorSensor color_sensor;
//    public String coneSleeve (){
//        if (color_sensor.blue() > color_sensor.red() && color_sensor.blue() > color_sensor.green()) {
//            telemetry.addData("Color Detected", "Blue: go to parking position 1");
//            return "blue";
//        }
//        else if (color_sensor.red() > color_sensor.blue() && color_sensor.red() > color_sensor.green()){
//            telemetry.addData("Color Detected", "Red: go to parking position 2");
//            return "red";
//        }
//        else if (color_sensor.green() > color_sensor.blue() && color_sensor.green() > color_sensor.red()) {
//            telemetry.addData("Color Detected", "Green: go to parking position 3");
//            return "green";
//        }
//        else {
//            return "none";
//        }
//
//    }
//
////    @Override
////    public void runOpMode() {
////        color_sensor = hardwareMap.get(ColorSensor.class, "Color Sensor");
//////        int[] hue = {45, 75, 165, 180, 240, 250, 300, 330}; //180-240(Blue), 330-45(Red), 75-165(Green)
////        waitForStart();
////        while (opModeIsActive()) {
////
////            if(coneSleeve().equals("blue")){
////                /
////            }
////
////
//////            if (color_sensor.argb() >= hue[5] && color_sensor.argb() <= hue[6]) {
//////                telemetry.addData("Color Detected", "Purple");
//////            }
//////
//////             else if (color_sensor.argb() >= hue[5] || color_sensor.argb() <= hue[0]) {
//////                telemetry.addData("Color Detected", "Red");
//////            }
//////
//////            else if (color_sensor.argb() >= hue[1] && color_sensor.argb() <= hue[2]) {
//////                telemetry.addData("Color Detected", "Green");
//////            }
////
////            //telemetry.addData("Color:", "Blue");
////            //telemetry.update();
////
////            //telemetry.addData("Alpha Reading");
////            //telemetry.addData("ARGB Reading", color.argb());
////            //telemetry.addData("Color = Red", color.red());
////            //telemetry.addData("Color = Green", color.green());
////            //telemetry.addData("Color = Blue", color.blue());
////            telemetry.update();
////        }
////    }
//}