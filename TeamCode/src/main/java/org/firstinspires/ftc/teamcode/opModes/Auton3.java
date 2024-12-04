package org.firstinspires.ftc.teamcode.opModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robotParts.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.robotParts.clawIntake;
import org.firstinspires.ftc.teamcode.robotParts.outtake;
import org.firstinspires.ftc.teamcode.robotParts.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.robotParts.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.robotParts.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.robotParts.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.robotParts.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.robotParts.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.robotParts.servoPositions;

import java.util.List;

@Autonomous(name = "0+2",group = "Auton")
public class Auton3 extends LinearOpMode {
    clawIntake intake = new clawIntake();
    outtake outtake = new outtake();
    MecanumDrivetrain drive = new MecanumDrivetrain();

    int state = 0;
    double timer;
    private Follower follower;

    Point start = new Point(0, 0, Point.CARTESIAN);
    Point rung = new Point(30.3, 4, Point.CARTESIAN);
    Point rungLeave = new Point(15, -10, Point.CARTESIAN);
    Point rungLeave2 = new Point(15, -60, Point.CARTESIAN);
    Point spikeLeftBack = new Point(55, -20, Point.CARTESIAN);
    Point spike1Back = new Point(48, -37, Point.CARTESIAN);
    Point observationZone1 = new Point(8, -37, Point.CARTESIAN);
    Point spike2Back = new Point(45, -50, Point.CARTESIAN);
    Point observationZone2 = new Point(8, -50, Point.CARTESIAN);
    Point controlSpecimen2 = new Point(10, -33, Point.CARTESIAN);
    Point collectSpecimen2 = new Point(-2,-33, Point.CARTESIAN);
    Point rung2 = new Point(30.3, 6, Point.CARTESIAN);
//    Point spike3Back = new Point(45, -63, Point.CARTESIAN);
//    Point observationZone3 = new Point(8, -56, Point.CARTESIAN);

    @Override
    public void runOpMode() throws InterruptedException {
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        follower = new Follower(hardwareMap);

        PathChain firstSpecimen = follower.pathBuilder()
                .addPath(new BezierLine(start, rung))
                .setConstantHeadingInterpolation(0)
                .setPathEndTimeoutConstraint(2.5)
                .build();


        PathChain getToSample = follower.pathBuilder()
                .addPath(new BezierCurve(rung, rungLeave, rungLeave2, spikeLeftBack, spike1Back))
                .setConstantHeadingInterpolation(0)
                .build();

        PathChain collectSamples = follower.pathBuilder()
                .addPath(new BezierLine(spike1Back, observationZone1))
                .setConstantHeadingInterpolation(0)
                .addPath(new BezierCurve(observationZone1, spike1Back, spike2Back))
                .setConstantHeadingInterpolation(0)
                .addPath(new BezierLine(spike2Back, observationZone2))
                .setConstantHeadingInterpolation(0)
                .build();

        PathChain getSecondSpecimen = follower.pathBuilder()
                .addPath(new BezierCurve(observationZone2, controlSpecimen2, collectSpecimen2))
                .setConstantHeadingInterpolation(0)
                .setPathEndTimeoutConstraint(4.0)
                .build();

        PathChain scoreSecondSpecimen = follower.pathBuilder()
                .addPath(new BezierLine(collectSpecimen2, rung2))
                .setConstantHeadingInterpolation(0)
                .build();

        follower.followPath(firstSpecimen, true);
        follower.setMaxPower(0.75);

        outtake.init(hardwareMap);
        intake.init(hardwareMap);
        drive.init(hardwareMap);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();
        if (isStopRequested()) return;
        while (opModeIsActive()) {
            switch (state) {
                case 0:
                    follower.update();
                    intake.setDiffy(servoPositions.clawDrop.getDifferential());
                    if (!follower.isBusy()) {
                        state++;
                        outtake.autoSequenceState = 0;
                    }
                    break;
                case 1:
                    outtake.autoSpecimenSequence();
                    if (outtake.autoSequenceState == 3) {
                        follower.followPath(getToSample, true);
                        follower.setMaxPower(0.7);
                        state++;
                    }
                    break;
                case 2:
                    follower.update();
                    if (!follower.isBusy()) {
                        follower.followPath(collectSamples, true);
                        follower.setMaxPower(0.5);
                        state++;
                    }
                    break;
                case 3:
                    follower.update();
                    if (!follower.isBusy()) {
                        timer = System.currentTimeMillis();
                        state++;
                    }
                    break;
                case 4:
                    follower.update();
                    if (timer + 1000 < System.currentTimeMillis()) {
                        follower.followPath(getSecondSpecimen, true);
                        follower.setMaxPower(0.75);
                        state++;
                    }
                case 5:
                    follower.update();
                    if (!follower.isBusy()) {
                        timer = System.currentTimeMillis();
                        follower.followPath(getSecondSpecimen, true);
                        follower.setMaxPower(0.5);
                        state++;
                    }
                    break;
                    //hier gaat hij tweede specimen pakken
                case 6:
                    follower.update();
                    if (!follower.isBusy()) {
                        outtake.setClaw(servoPositions.outtakeGrip.getPosition());
                        timer = System.currentTimeMillis();
                        state++;
                    }
                    break;
                case 7:
                    if (timer + 200 < System.currentTimeMillis()) {
                        follower.followPath(scoreSecondSpecimen, true);
                        follower.setMaxPower(0.75);
                        state++;
                    }
                    break;
                case 8:
                    follower.update();
                    if (!follower.isBusy()) {
                        state++;
                        outtake.autoSequenceState = 0;
                    }
                    break;
                case 9:
                    outtake.autoSpecimenSequence();
                    if (outtake.autoSequenceState == 3) {
                        state++;
                    }
                    break;
            }

            telemetry.addData("follower", !follower.isBusy());
            telemetry.addData("case", state);
            telemetry.addData("outtakeCase", outtake.autoSequenceState);
            telemetry.addData("pos", outtake.arm.getCurrentPosition());
            telemetry.addData("intakePos", intake.slides.getCurrentPosition());
            telemetry.update();
        }
    }
}
