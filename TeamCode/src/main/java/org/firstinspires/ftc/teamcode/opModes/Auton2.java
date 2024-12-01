package org.firstinspires.ftc.teamcode.opModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robotParts.clawIntake;
import org.firstinspires.ftc.teamcode.robotParts.outtake;
import org.firstinspires.ftc.teamcode.robotParts.pedroPathing.follower.*;
import org.firstinspires.ftc.teamcode.robotParts.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.robotParts.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.robotParts.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.robotParts.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.robotParts.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.robotParts.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.robotParts.pedroPathing.util.Timer;

import java.util.List;

@Autonomous(name = "Auton2",group = "Auton")
public class Auton2 extends LinearOpMode {
    clawIntake intake = new clawIntake();
    outtake outtake = new outtake();
    private Follower follower;
    private Path forwards;
    private Path backwards;

    @Override
    public void runOpMode() {
        /*
            Sets the hubs to using BulkReads. Any read will read all non-I2C sensors from a hub at once.
         */
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        follower = new Follower(hardwareMap);

        forwards = new Path(new BezierLine(new Point(0,0, Point.CARTESIAN), new Point(33,14, Point.CARTESIAN)));
        forwards.setConstantHeadingInterpolation(0);

        follower.followPath(forwards);

        outtake.init(hardwareMap);
        intake.init(hardwareMap);

        /*
            This line makes the telemetry available for FTC Dashboard by ACME Robotics.
         */
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();
        if (isStopRequested()) return;
        while (opModeIsActive()) {
            follower.update();
            telemetry.update();
        }
    }
}
