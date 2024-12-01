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
    public static double p = 0, i = 0, d = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        outtake outtake = new outtake();

        PIDController pid = new PIDController(p,i,d);

        waitForStart();
        if (isStopRequested()) return;

        outtake.init(hardwareMap);

        while (opModeIsActive()) {
            pid.setPID(p,i,d);
            pos = outtake.arm.getCurrentPosition();
            power = pid.calculate(pos,target);
            if (power > 0.5) power = 0.5;
            else if (power < -0.5) power = -0.5;
            outtake.moveArm(power);

            telemetry.addData("power",power);
            telemetry.addData("pos",pos);
            telemetry.update();
        }
    }
}
