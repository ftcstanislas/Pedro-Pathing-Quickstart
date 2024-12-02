package org.firstinspires.ftc.teamcode.opModes.tests;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robotParts.outtake;

@Config
@TeleOp(name = "clawTest",group = "TeleOp")
public class outtakeTest extends LinearOpMode {
    double power;
    int pos;
    public static int target;
    @Override
    public void runOpMode() throws InterruptedException {
        outtake outtake = new outtake();

        waitForStart();
        if (isStopRequested()) return;

        outtake.init(hardwareMap);

        while (opModeIsActive()) {
            outtake.armPID(target);
            telemetry.addData("power",power);
            telemetry.addData("pos",pos);
            telemetry.update();
        }
    }
}
