package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robotParts.intake;
import org.firstinspires.ftc.teamcode.robotParts.outtake;

@TeleOp(name = "IntakeTest",group = "TeleOp")
public class intakeTest extends LinearOpMode {
    intake intake = new intake();
    outtake outtake = new outtake();

    @Override
    public void runOpMode() throws InterruptedException {
        intake.init(hardwareMap);
        outtake.init(hardwareMap);

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            intake.run(gamepad2.left_stick_y);

            if (gamepad2.a) intake.wrist(0.0);
            else if (gamepad2.b) intake.wrist(0.45);
            if (gamepad2.x) intake.setScissor(1.0);
            else if (gamepad2.y) intake.setScissor(0.75);
            outtake.moveBar(gamepad1.left_stick_y);

            telemetry.addData("outtakeLeft power", gamepad1.left_stick_y);
            telemetry.addData("intake power: ",intake.CRLeft.getPower());
            telemetry.addData("wrist pos",intake.wrist.getPosition());
            telemetry.addData("scissor pos", intake.scissor.getPosition());
            telemetry.update();
        }
    }
}
