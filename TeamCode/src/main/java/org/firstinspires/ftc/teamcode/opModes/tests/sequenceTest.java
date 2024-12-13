package org.firstinspires.ftc.teamcode.opModes.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.robotParts.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.robotParts.clawIntake;
import org.firstinspires.ftc.teamcode.robotParts.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.robotParts.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.robotParts.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.robotParts.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.robotParts.servoPositions;
import org.firstinspires.ftc.teamcode.robotParts.vision.SampleDetectionPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "sequenceTest",group = "TeleOp")
public class sequenceTest extends LinearOpMode {
    OpenCvCamera camera;
    final SampleDetectionPipeline sampleDetectionPipeline = new SampleDetectionPipeline(true);

    clawIntake intake = new clawIntake();
    MecanumDrivetrain drive = new MecanumDrivetrain();

    private Follower follower;
    private Path sampleY;

    volatile Gamepad last = new Gamepad();
    volatile Gamepad current = new Gamepad();

    ArrayList<SampleDetectionPipeline.Sample> currentDetections;
    double[] bestSampleInformation;

    double slideTarget, timer, slideDelta;

    int intakeCase = 0;

    boolean starting;

    @Override
    public void runOpMode() throws InterruptedException {
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

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

        follower = new Follower(hardwareMap);

        intake.init(hardwareMap);
        drive.init(hardwareMap);

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            last.copy(current);
            current.copy(gamepad1);

            switch (intakeCase) {
                case 0:
                    if (current.x && !last.x) {
                        sampleDetectionPipeline.desiredColor = SampleDetectionPipeline.RED;
                        telemetry.addLine("red");
                        starting = true;
                    } else if (current.a && !last.a) {
                        sampleDetectionPipeline.desiredColor = SampleDetectionPipeline.BLUE;
                        telemetry.addLine("blue");
                        starting = true;
                    } else if (current.b && !last.b) {
                        sampleDetectionPipeline.desiredColor = SampleDetectionPipeline.YELLOW;
                        telemetry.addLine("yellow");
                        starting = true;
                    }
                    if (starting) {
                        starting = false;
                        bestSampleInformation = null;
                        currentDetections = sampleDetectionPipeline.getDetectedStones();
                        bestSampleInformation = sampleDetectionPipeline.getBestSampleInformation(currentDetections);
                        if (bestSampleInformation != null) {
                            if (Math.abs(bestSampleInformation[0]) > 1.5) {
                                sampleY = new Path(new BezierLine(new Point(0, 0, Point.CARTESIAN), new Point(0, -bestSampleInformation[0] / 2.54, Point.CARTESIAN)));
                                sampleY.setConstantHeadingInterpolation(0);
                                follower.followPath(sampleY);
                            }

                            intake.setClaw(servoPositions.intakeRelease.getPosition());
                            intake.setDiffy(servoPositions.clawDrop.getDifferential());
                            intake.setKeepSlides(servoPositions.releaseSlides.getPosition());

                            timer = System.currentTimeMillis();

                            intakeCase++;
                        }
                    }
                    drive.robotCentric(-current.left_stick_y, current.left_stick_x, -current.right_stick_x);
                    break;
                case 1:
                    follower.update();
                    if (timer + 100 > System.currentTimeMillis()) {
                        intakeCase++;
                    }
                case 2:
                    follower.update();
                    slideTarget = bestSampleInformation[1];
                    intake.setDiffyAngle(bestSampleInformation[2]);
                    if (!follower.isBusy() && Math.abs(slideDelta) < 3.5) {
                        intakeCase++;
                        timer = System.currentTimeMillis();
                    }
                    break;
                case 3:
                    if (timer + 200 < System.currentTimeMillis()) {
                        intake.setClaw(servoPositions.intakeGrip.getPosition());
                        timer = System.currentTimeMillis();
                        intakeCase++;
                    }
                    break;
                case 4:
                    if (timer + 100 < System.currentTimeMillis()) {
                        intake.setDiffy(servoPositions.clawDrop.getDifferential());
                        slideTarget = 0;
                        intakeCase++;
                    }
                case 5:
                    if (slideDelta > -1) {
                        intakeCase = 0;
                        intake.setKeepSlides(servoPositions.keepSlides.getPosition());
                    }
                    break;
                case 6:
                    if (current.y && !last.y) intakeCase = 0;
                    drive.robotCentric(-current.left_stick_y, current.left_stick_x, -current.right_stick_x);
                    break;
            }

//            intake.manualSequence(current.x && !last.x, current.right_trigger - current.left_trigger, current.y);

            slideDelta = intake.slideToCentimeter(slideTarget);

//            if (bestSampleInformation != null) {
//                telemetry.addData("x", bestSampleInformation[0]);
//                telemetry.addData("y", bestSampleInformation[1]);
//                telemetry.addData("angle", bestSampleInformation[2]);
//                telemetry.addData("delta", slideDelta);
//            }
            telemetry.addData("case", intakeCase);
            telemetry.update();
        }
    }
}
