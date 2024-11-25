package org.firstinspires.ftc.teamcode.opModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.robotParts.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.robotParts.clawIntake;
import org.firstinspires.ftc.teamcode.robotParts.outtake;

import java.util.List;

@Autonomous(name = "Auton",group = "Auton")
public class Auton extends LinearOpMode {
    clawIntake intake = new clawIntake();
    org.firstinspires.ftc.teamcode.robotParts.outtake outtake = new outtake();
    MecanumDrivetrain drive = new MecanumDrivetrain();
    long timer;

    @Override
    public void runOpMode() {
        /*
            Sets the hubs to using BulkReads. Any read will read all non-I2C sensors from a hub at once.
         */
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        outtake.init(hardwareMap);
        intake.init(hardwareMap);
        drive.init(hardwareMap);

        /*
            This line makes the telemetry available for FTC Dashboard by ACME Robotics.
         */
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();
        if (isStopRequested()) return;
        timer = System.currentTimeMillis();
        while (opModeIsActive()) {
            while (System.currentTimeMillis() < timer + 1500) {
                drive.robotCentric(0.3,0.0, 0);
            }
            drive.robotCentric(0.0,0.0,0);
            telemetry.update();
        }
    }
}
