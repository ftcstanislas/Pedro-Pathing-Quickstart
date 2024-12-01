package org.firstinspires.ftc.teamcode.opModes.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robotParts.clawIntake;
import org.firstinspires.ftc.teamcode.robotParts.servoPositions;

@Config
@TeleOp(name = "IntakeTest",group = "TeleOp")
public class intakeTest extends LinearOpMode {
    clawIntake intake = new clawIntake();

    double left = 0, right = 0, pos, power;

    public static int cm;

    @Override
    public void runOpMode() throws InterruptedException {
        intake.init(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
//            left = -gamepad1.left_stick_y;
//            right = -gamepad1.right_stick_y;
//            if (gamepad1.dpad_down) {
//                intake.setDiffy(servoPositions.rollerSide.getDifferential());
//            } else if (gamepad1.dpad_left) {
//                intake.setDiffy(servoPositions.rollerBack.getDifferential());
//            } else if (gamepad1.dpad_right) {
//                intake.setDiffy(servoPositions.rollerFront.getDifferential());
//            } else if (gamepad1.dpad_up) {
//                intake.setDiffy(servoPositions.rollerTransfer.getDifferential());
//            }
//            intake.differentialLeft.setPosition(left);
//            intake.differentialRight.setPosition(right);
//            intake.setDiffyAngle(left);
            intake.slideToCentimeter(cm);

            if (gamepad1.a) {
                intake.setClaw(servoPositions.intakeRelease.getPosition());
            } else if (gamepad1.b) {
                intake.setClaw(servoPositions.intakeGrip.getPosition());
            }

            telemetry.addData("wristLeft", left);
            telemetry.addData("wristRight",right);
            telemetry.addData("target", cm);
            telemetry.addData("pos", pos);
            telemetry.addData("power", power);
            telemetry.update();
        }
    }
}
