package org.firstinspires.ftc.teamcode.robotParts.vision;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;
@Config
@TeleOp(name = "Camera Tune",group = "TeleOp")
public class sampleCameraTune extends LinearOpMode {
    OpenCvCamera camera;
    final SampleDetectionPipeline sampleDetectionPipeline = new SampleDetectionPipeline(true);

    public static int x = 0, y = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        camera.setPipeline(sampleDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {camera.startStreaming(800,448, OpenCvCameraRotation.SIDEWAYS_LEFT);}

            @Override
            public void onError(int errorCode) {}
        });
        telemetry.setMsTransmissionInterval(25);

        FtcDashboard.getInstance().startCameraStream(camera, 0);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        while (!isStarted() && !isStopRequested()) {
            ArrayList<SampleDetectionPipeline.Sample> currentDetections = sampleDetectionPipeline.getDetectedStones();
            for (SampleDetectionPipeline.Sample sample : currentDetections) {
                telemetry.addData("x", sample.actualX);
                telemetry.addData("y", sample.actualY);
                telemetry.addData("standard y", 28.4 * Math.tan(Math.toRadians(42.47)));
                telemetry.addData("cos", 28.4 / Math.cos(Math.toRadians(42.47)));
                telemetry.addData("standard x", (28.4 / Math.cos(Math.toRadians(42.47)))*Math.tan(Math.toRadians(sample.cameraXAngle)));
                telemetry.update();
            }

        }

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            telemetry.update();
        }
    }
}
