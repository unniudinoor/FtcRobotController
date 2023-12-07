package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.internal.network.DeviceNameListener;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

@Autonomous
public class cameraTest extends OpMode  {

    OpenCvCamera webcam1;

    @Override
    public void init() {

        WebcamName webcamName = hardwareMap.get(WebcamName.class, "B9241040" );
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId","id", hardwareMap.appContext.getPackageName());
        webcam1 = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);

        webcam1.setPipeline(new examplePipeline());

        webcam1.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened () {
                webcam1.startStreaming(640,360, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError (int errorCode) {

            }
        });



    }

    @Override
    public void loop() {

    }
    class examplePipeline extends OpenCvPipeline{
        Mat YCbCr = new Mat();
        Mat leftCrop;
        Mat middleCrop;
        Mat rightCrop;
        double leftAvg;
        double middleAvg;
        double rightAvg;
        double leftAvgfinal;
        double rightAvgfinal;
        double middleAvgfinal;
        Mat outPut = new Mat();
        Scalar rectColor = new Scalar(255.0,0.0,0.0);


        public Mat processFrame(Mat input) {
            Imgproc.cvtColor(input,YCbCr,Imgproc.COLOR_RGB2YCrCb);
            telemetry.addLine("pipeline running");

            Rect leftRect = new Rect(1, 1, 319, 359);
            //Rect middleRect = new Rect(213,1,319,359);
            Rect rightRect = new Rect(320,1,319,359);

            input.copyTo(outPut);
            Imgproc.rectangle(outPut,leftRect,rectColor,2);
            Imgproc.rectangle(outPut,rightRect,rectColor,2);
            //Imgproc.rectangle(outPut,middleRect,rectColor,2);

            leftCrop = YCbCr.submat(leftRect);
            //middleCrop = YCbCr.submat(middleRect);
            rightCrop = YCbCr.submat(rightRect);

            Core.extractChannel(leftCrop,leftCrop,2);
            Core.extractChannel(rightCrop,rightCrop,2);
            Core.extractChannel(middleCrop,middleCrop,2);

            Scalar leftAvg = Core.mean(leftCrop);
            Scalar middleAvg = Core.mean(middleCrop);
            Scalar rightAvg = Core.mean(rightCrop);

            leftAvgfinal = leftAvg.val[0];
            middleAvgfinal = middleAvg.val[0];
            rightAvgfinal = rightAvg.val[0];

            if (leftAvgfinal > rightAvgfinal && leftAvgfinal > middleAvgfinal){
                telemetry.addLine("Left");
            }
            else if (rightAvgfinal > leftAvgfinal && rightAvgfinal > middleAvgfinal){
                telemetry.addLine("right");
            }
            else {
                telemetry.addLine("middle");
            }
            return(outPut);











        }
    }

}

