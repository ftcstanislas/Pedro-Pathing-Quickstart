package org.firstinspires.ftc.teamcode.opModes.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robotParts.outtake;
import org.firstinspires.ftc.teamcode.robotParts.servoPositions;

@Config
@TeleOp(name = "ArmTest",group = "TeleOp")
public class armTest extends LinearOpMode {
    outtake outtake = new outtake();

    public static double claw, arm;

    @Override
    public void runOpMode() throws InterruptedException {
        outtake.init(hardwareMap);

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            claw = -gamepad1.left_stick_y;
            arm = -gamepad1.right_stick_y;
            outtake.setClaw(claw);
            outtake.setArm(arm);

            telemetry.addData("wristLeft", claw);
            telemetry.addData("wristRight", arm);
            telemetry.update();
        }
    }
}
