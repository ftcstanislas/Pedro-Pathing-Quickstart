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
            intake.wristLeft.setPosition(left);
            intake.wristRight.setPosition(right);

            if (gamepad2.x) intake.setScissor(servoPositions.scissorRetract.getPosition());
            else if (gamepad2.y) intake.setScissor(servoPositions.scissorExtend.getPosition());


            telemetry.addData("wristLeft", left);
            telemetry.addData("wristRight",right);
            telemetry.addData("scissor pos", intake.scissor.getPosition());
            telemetry.update();
        }
    }
}
