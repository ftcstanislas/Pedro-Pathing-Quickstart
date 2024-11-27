package org.firstinspires.ftc.teamcode.opModes.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.robotParts.outtake;

import java.util.List;

@Disabled
@TeleOp(name = "PIDFTest",group = "TeleOp")
public class PIDFTest extends LinearOpMode {
    outtake outtake = new outtake();

    volatile Gamepad last = new Gamepad();
    volatile Gamepad current = new Gamepad();

    public static double p = 0, i = 0, d = 0, k = 0;
    public static int target;
    int ticks;
    double power;

    PIDController pid = new PIDController(p,i,d);

    @Override
    public void runOpMode() throws InterruptedException {
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        outtake.init(hardwareMap);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        pid.setPID(p,i,d);

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            last.copy(current);
            current.copy(gamepad1);

//            ticks = outtake.barRight.getCurrentPosition();
//            power = pid.calculate(ticks,target);
            power = -current.left_trigger + current.right_trigger;
            outtake.moveBar(power, k);

            telemetry.addData("pos ", ticks);
            telemetry.addData("target ", target);
            telemetry.update();
        }
    }
}
