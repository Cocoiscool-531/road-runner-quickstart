package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.*;
import org.firstinspires.ftc.teamcode.auto.WebcamPipeline;

public class BaseAuto extends LinearOpMode {

    private OpenCvCamera webcam1;

    @Override
    public void runOpMode() throws InterruptedException {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam1 = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam1"), cameraMonitorViewId);

        telemetry.addLine("Camera Starting; Please Wait To Start");

        webcam1.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                telemetry.addLine("Camera Init Successful: Ready To Start");
                telemetry.update();

                webcam1.startStreaming(800, 600, OpenCvCameraRotation.UPSIDE_DOWN);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Error", errorCode);
                telemetry.update();
            }
        });
    }

    public String detectPropPosition(String colorToDetect) throws InterruptedException {
        String propPos = "Right";

        int count = 0;

        WebcamPipeline colorDetector = new WebcamPipeline(colorToDetect);
        webcam1.setPipeline(colorDetector);

        Thread.sleep(2000);

        propPos = colorDetector.getPropPos();

        while(propPos == "" && opModeIsActive()){

            propPos = colorDetector.getPropPos();

            count++;

            if (count > 5) {
                propPos = "Right";
                break;
            }
        }

        webcam1.closeCameraDevice();
        return propPos;
    }
}
