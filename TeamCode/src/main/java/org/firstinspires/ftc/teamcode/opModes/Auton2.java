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
import org.firstinspires.ftc.teamcode.robotParts.servoPositions;

import java.util.List;

@Autonomous(name = "Auton2",group = "Auton")
public class Auton2 extends LinearOpMode {
    clawIntake intake = new clawIntake();
    outtake outtake = new outtake();
    private Follower follower;
    private Path forwards;
    private Path backwards;

    int state = 0;

    double timer;

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

        forwards = new Path(new BezierLine(new Point(0,0, Point.CARTESIAN), new Point(32,14, Point.CARTESIAN)));
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
            switch (state) {
                case 0:
                    intake.setDiffy(servoPositions.clawDrop.getDifferential());
                    follower.update();
                    if (!follower.isBusy()) {
                        state++;
                    }
                    break;
                case 1:
                    outtake.armPID(600);
                    if (outtake.arm.getCurrentPosition() > 590) {
                        state++;
                    }
                    break;
                case 2:
                    outtake.setClaw(servoPositions.outtakeRelease.getPosition());
                    timer = System.currentTimeMillis();
                    state++;
                    break;
                case 3:
                    if (timer + 100 < System.currentTimeMillis()){
                        state++;
                    }
                case 4:
                    outtake.armPID(0);
            }
            telemetry.addData("follower", !follower.isBusy());
            telemetry.addData("pos", outtake.arm.getCurrentPosition() > 690);
            telemetry.addData("pos", outtake.arm.getCurrentPosition());
            telemetry.update();
        }
    }
}
