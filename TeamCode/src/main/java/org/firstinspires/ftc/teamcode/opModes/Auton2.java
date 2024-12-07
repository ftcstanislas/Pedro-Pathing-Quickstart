package org.firstinspires.ftc.teamcode.opModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robotParts.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.robotParts.clawIntake;
import org.firstinspires.ftc.teamcode.robotParts.outtake;
import org.firstinspires.ftc.teamcode.robotParts.pedroPathing.follower.*;
import org.firstinspires.ftc.teamcode.robotParts.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.robotParts.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.robotParts.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.robotParts.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.robotParts.servoPositions;

import java.util.List;

@Autonomous(name = "0+1",group = "Auton")
public class Auton2 extends LinearOpMode {
    clawIntake intake = new clawIntake();
    outtake outtake = new outtake();
    private Follower follower;

    Point start = new Point(0, 0, Point.CARTESIAN);
    Point rung = new Point(31, 6, Point.CARTESIAN);
    Point collectSpecimen4 = new Point(-5,-33, Point.CARTESIAN);

    private Path[] paths = {
        new Path(new BezierLine(start, rung)),
    };

    int state = 0;

    double timer, endTimer;

    @Override
    public void runOpMode() throws InterruptedException {
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        follower = new Follower(hardwareMap);

        for (Path path : paths) {
            path.setConstantHeadingInterpolation(0);
        }

        PathChain chain = follower.pathBuilder()
                .addPath(paths[0])
                .setConstantHeadingInterpolation(0)
                .build();

        Path park = new Path(new BezierLine(rung, collectSpecimen4));
        park.setConstantHeadingInterpolation(0);

        follower.followPath(chain, true);

        outtake.init(hardwareMap);
        intake.init(hardwareMap);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();
        endTimer = System.currentTimeMillis();
        if (isStopRequested()) return;
        while (opModeIsActive()) {
            switch (state) {
                case 0:
                    follower.update();
                    intake.setDiffy(servoPositions.clawDrop.getDifferential());
                    if (!follower.isBusy() || endTimer + 2000 < System.currentTimeMillis()) {
                        outtake.autoSequenceState = 0;
                        state++;
                    }
                    break;
                case 1:
                    outtake.autoSpecimenSequence();
                    if (outtake.autoSequenceState == 4) state++;
                case 2:
                    if (outtake.autoSequenceState == 3) {
                        follower.followPath(park);
                        follower.setMaxPower(1.0);
                        endTimer = System.currentTimeMillis();
                        state++;
                    }
                    break;
                case 3:
                    follower.update();
                    if (!follower.isBusy()) state++;
                    if (endTimer + 2000 < System.currentTimeMillis()) {
                        follower.breakFollowing();
                        state++;
                    }
            }
            telemetry.addData("follower", !follower.isBusy());
            telemetry.addData("case", state);
            telemetry.addData("outtakeCase", outtake.autoSequenceState);
            telemetry.addData("pos", outtake.arm.getCurrentPosition());
            telemetry.update();
        }
    }
}
