package org.firstinspires.ftc.teamcode.opModes.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robotParts.outtake;
import org.firstinspires.ftc.teamcode.robotParts.MecanumDrivetrain;

@Config
@Disabled
@TeleOp(name = "ArmTest",group = "TeleOp")
public class armTest extends LinearOpMode {
    MecanumDrivetrain drive = new MecanumDrivetrain();
    outtake outtake = new outtake();

    public static double k = 0.0006892;

    @Override
    public void runOpMode() throws InterruptedException {
        outtake.init(hardwareMap);

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            outtake.moveBar(-gamepad1.left_stick_y,k);
            drive.outdatedRobotCentric(drive.toPolar(gamepad1.left_stick_x,-gamepad1.left_stick_y), gamepad1.right_stick_x);

            telemetry.addData("power",-gamepad1.left_stick_y);
            telemetry.addData("leftPos",outtake.barLeft.getCurrentPosition());
            telemetry.addData("rightPos",outtake.barRight.getCurrentPosition());
            telemetry.update();
        }
    }
}
