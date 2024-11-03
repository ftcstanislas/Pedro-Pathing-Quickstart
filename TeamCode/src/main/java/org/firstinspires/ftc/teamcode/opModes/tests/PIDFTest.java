package org.firstinspires.ftc.teamcode.opModes.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.robotParts.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.robotParts.intake;
import org.firstinspires.ftc.teamcode.robotParts.outtake;
import org.firstinspires.ftc.teamcode.robotParts.servoPositions;

import java.util.List;

@Config
@TeleOp(name = "PIDFTest",group = "TeleOp")
public class PIDFTest extends LinearOpMode {
    outtake outtake = new outtake();

    volatile Gamepad last = new Gamepad();
    volatile Gamepad current = new Gamepad();

    public static double p = 0, i = 0, d = 0, f = 0;
    public static int target;
    int ticks;
    final double ticks_per_radian = 1425.1/Math.PI;
    double power, ff;

    PIDController pidf = new PIDController(p,i,d);

    @Override
    public void runOpMode() throws InterruptedException {
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        outtake.init(hardwareMap);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        pidf.setPID(p,i,d);

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            last.copy(current);
            current.copy(gamepad1);

            ticks = outtake.barRight.getCurrentPosition();

//            outtake.moveBar(current.left_trigger - current.right_trigger);
            power = pidf.calculate(ticks,target);
            ff = Math.cos((ticks - 0)/ticks_per_radian) * f;
            power += ff;
            outtake.barRight.setPower(power);

            telemetry.addData("pos ", ticks);
            telemetry.addData("target ", target);
            telemetry.update();
        }
    }
}
