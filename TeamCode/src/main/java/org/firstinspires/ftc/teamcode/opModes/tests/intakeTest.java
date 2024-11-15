package org.firstinspires.ftc.teamcode.opModes.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robotParts.intake;
import org.firstinspires.ftc.teamcode.robotParts.outtake;
import org.firstinspires.ftc.teamcode.robotParts.servoPositions;

@Config
@TeleOp(name = "IntakeTest",group = "TeleOp")
public class intakeTest extends LinearOpMode {
    intake intake = new intake();

    public static double left, right;

    @Override
    public void runOpMode() throws InterruptedException {
        intake.init(hardwareMap);

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            left = -gamepad1.left_stick_y;
            right = -gamepad1.right_stick_y;
            if (gamepad1.dpad_down) {
                intake.setDiffy(servoPositions.sideTransfer.getDifferential());
            } else if (gamepad1.dpad_left) {
                intake.setDiffy(servoPositions.intakeBack.getDifferential());
            } else if (gamepad1.dpad_right) {
                intake.setDiffy(servoPositions.intakeFront.getDifferential());
            } else if (gamepad1.dpad_up) {
                intake.setDiffy(servoPositions.transfer.getDifferential());
            }

            intake.setScissor(0.52*gamepad1.left_stick_y+0.48);
//            if (gamepad2.x) intake.setScissor(servoPositions.scissorRetract.getPosition());
//            else if (gamepad2.y) intake.setScissor(servoPositions.scissorExtend.getPosition());

            intake.run(gamepad1.right_trigger - gamepad1.left_trigger);

            telemetry.addData("wristLeft", left);
            telemetry.addData("wristRight",right);
            telemetry.addData("scissor pos", intake.scissor.getPosition());
            telemetry.update();
        }
    }
}
