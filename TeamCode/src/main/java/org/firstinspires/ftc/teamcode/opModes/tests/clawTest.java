package org.firstinspires.ftc.teamcode.opModes.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robotParts.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.robotParts.outtake;
import org.firstinspires.ftc.teamcode.robotParts.servoPositions;

@Config
@TeleOp(name = "clawTest",group = "TeleOp")
public class clawTest extends LinearOpMode {
    int target;
    @Override
    public void runOpMode() throws InterruptedException {
        outtake outtake = new outtake();

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            if (gamepad1.x) target = 0;
            else if (gamepad1.b) target = 500;
            telemetry.addData("power",-gamepad1.left_stick_y);
            telemetry.update();
        }
    }
}
