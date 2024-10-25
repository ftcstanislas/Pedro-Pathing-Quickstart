package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robotParts.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.robotParts.intake;
import org.firstinspires.ftc.teamcode.robotParts.outtake;
import org.firstinspires.ftc.teamcode.robotParts.servoPositions;

import java.util.List;

@TeleOp(name = "SoloDrive",group = "TeleOp")
public class soloDrive extends LinearOpMode {
    intake intake = new intake();
    outtake outtake = new outtake();
    MecanumDrivetrain drive = new MecanumDrivetrain();

    boolean currentOuttake, currentScissor, lastOuttake, lastScissor, outtakeOut = false,scissorOut = false;

    @Override
    public void runOpMode() throws InterruptedException {
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        outtake.init(hardwareMap);
        intake.init(hardwareMap);
        drive.init(hardwareMap);

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            lastScissor = currentOuttake;
            lastOuttake = currentOuttake;

            currentOuttake = gamepad1.x;
            currentScissor = gamepad1.y;

            if (gamepad1.left_bumper) intake.run(1);
            else if (gamepad1.right_bumper) intake.run(-1);

            if (gamepad1.dpad_left) intake.wrist(servoPositions.wristTransfer.getPosition());
            else if (gamepad1.dpad_down) intake.wrist(servoPositions.wristIntake.getPosition());
            else if (gamepad1.dpad_up) intake.wrist(servoPositions.wristAway.getPosition());

            if ((!lastScissor && currentScissor) && scissorOut && Math.abs(intake.wrist.getPosition() - servoPositions.wristIntake.getPosition()) > 0.05) {
                intake.setScissor(servoPositions.scissorRetract.getPosition());
                scissorOut = true;
            }
            else if ((!lastScissor && currentScissor) && !scissorOut) {
                intake.setScissor(servoPositions.scissorExtend.getPosition());
                scissorOut = false;
            }

            outtake.moveBar(gamepad2.left_trigger - gamepad2.right_trigger);

            if(!lastOuttake && currentOuttake) {
                if(outtakeOut) {
                    outtake.setOuttake(servoPositions.outtakeReceive.getPosition());
                    outtakeOut = false;
                }
                else {
                    outtake.setOuttake(servoPositions.outtakeDrop.getPosition());
                    outtakeOut = true;
                }
            }

            if (gamepad1.a) outtake.setClaw(servoPositions.clawRelease.getPosition());
            else outtake.setClaw(servoPositions.clawGrip.getPosition());

            drive.robotCentric(-gamepad1.left_stick_y, gamepad1.left_stick_x, -gamepad1.right_stick_x);

            telemetry.addData("maxPower",drive.maxPower);
            telemetry.addData("outtakeLeft power", gamepad1.left_stick_y);
            telemetry.addData("intake power: ",intake.CRLeft.getPower());
            telemetry.addData("wrist pos",intake.wrist.getPosition());
            telemetry.addData("scissor pos", intake.scissor.getPosition());
            telemetry.addData("outtake pos", outtake.outtake.getPosition());
            telemetry.update();
        }
    }
}
