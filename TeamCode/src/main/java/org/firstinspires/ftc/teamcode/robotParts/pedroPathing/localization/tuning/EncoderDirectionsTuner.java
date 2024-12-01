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
    DcMotorEx left_front, right_front, left_back, right_back;

    @Override
    public void runOpMode() throws InterruptedException {
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        left_front = hardwareMap.get(DcMotorEx.class, "left_front");
        right_front = hardwareMap.get(DcMotorEx.class, "right_front");
        left_back = hardwareMap.get(DcMotorEx.class, "left_back");
        right_back = hardwareMap.get(DcMotorEx.class, "right_back");

        left_front.setDirection(DcMotorEx.Direction.FORWARD);
        right_front.setDirection(DcMotorEx.Direction.REVERSE);
        left_back.setDirection(DcMotorEx.Direction.FORWARD);
        right_back.setDirection(DcMotorEx.Direction.REVERSE);

        left_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            telemetry.addData("left_front", left_front.getCurrentPosition());
            telemetry.addData("right_front", right_front.getCurrentPosition());
            telemetry.addData("left_back", left_back.getCurrentPosition());
            telemetry.addData("right_back", right_back.getCurrentPosition());
            telemetry.update();
        }
    }
}
