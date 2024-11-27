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
    Servo claw;
    @Override
    public void runOpMode() throws InterruptedException {
        claw = hardwareMap.get(Servo.class, "claw");
        claw.setPosition(servoPositions.outtakeGrip.getPosition());

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            if (gamepad1.a) {
                claw.setPosition(servoPositions.intakeGrip.getPosition());
            } else if (gamepad1.b) {
                claw.setPosition(servoPositions.intakeRelease.getPosition());
            }
            telemetry.addData("power",-gamepad1.left_stick_y);
            telemetry.update();
        }
    }
}
