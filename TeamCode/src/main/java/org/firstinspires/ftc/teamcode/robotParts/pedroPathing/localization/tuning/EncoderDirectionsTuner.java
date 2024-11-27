package org.firstinspires.ftc.teamcode.robotParts.pedroPathing.localization.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robotParts.pedroPathing.localization.Encoder;
import org.firstinspires.ftc.teamcode.robotParts.pedroPathing.localization.PoseUpdater;
import org.firstinspires.ftc.teamcode.robotParts.pedroPathing.util.DashboardPoseTracker;
import org.firstinspires.ftc.teamcode.robotParts.pedroPathing.util.Drawing;

import java.util.List;

@Config
@Autonomous(name = "Encoder Directions Tuner", group = "Autonomous Pathing Tuning")
public class EncoderDirectionsTuner extends LinearOpMode {
    DcMotorEx left, right, strafe;

    @Override
    public void runOpMode() throws InterruptedException {
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        left = hardwareMap.get(DcMotorEx.class, "right_front");
        right = hardwareMap.get(DcMotorEx.class, "left_back");
        strafe = hardwareMap.get(DcMotorEx.class, "right_back");

        left.setDirection(DcMotorEx.Direction.REVERSE);
        right.setDirection(DcMotorEx.Direction.FORWARD);
        strafe.setDirection(DcMotorEx.Direction.REVERSE);

        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        strafe.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            telemetry.addData("left", left.getCurrentPosition());
            telemetry.addData("right", right.getCurrentPosition());
            telemetry.addData("strafe", strafe.getCurrentPosition());
            telemetry.update();
        }
    }
}
