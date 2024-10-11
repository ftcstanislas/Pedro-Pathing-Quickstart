package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robotParts.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.robotParts.outtake;
import org.firstinspires.ftc.teamcode.robotParts.intake;
import org.firstinspires.ftc.teamcode.robotParts.servoPositions;

import java.util.List;

@TeleOp(name = "sequenceTest",group = "TeleOp")
public class sequenceTest extends LinearOpMode {
    intake intake = new intake();
    outtake outtake = new outtake();
    MecanumDrivetrain drive = new MecanumDrivetrain();

    double[] driveVector;
    intakeSequence state = intakeSequence.idle;

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
            switch (state) {
                case idle:
                    if (gamepad1.x) {
                        state = intakeSequence.getReady;
                        intake.wrist.setPosition(servoPositions.wristTransfer.getPosition());
                    }
                    break;
                case getReady:
                    if (Math.abs(intake.wrist.getPosition() - servoPositions.wristTransfer.getPosition()) > 0.05) {
                        state = intakeSequence.extend;
                        intake.scissor.setPosition(servoPositions.scissorExtend.getPosition());
                    }
                    break;
                case extend:
                    if (Math.abs(intake.scissor.getPosition() - servoPositions.scissorExtend.getPosition()) > 0.05) {
                        state = intakeSequence.drop;
                        intake.wrist.setPosition(servoPositions.wristIntake.getPosition());
                    }
            }

            outtake.moveBar(gamepad2.left_trigger - gamepad2.right_trigger);

            if (gamepad2.a) outtake.setOuttake(servoPositions.outtakeReceive.getPosition());
            else if (gamepad2.b) outtake.setOuttake(servoPositions.outtakeDrop.getPosition());

            drive.robotCentric(-gamepad1.left_stick_y, gamepad1.left_stick_x, -gamepad1.right_stick_x);
//            driveVector = drive.toPolar(gamepad1.left_stick_x,-gamepad1.left_stick_y);
//            driveVector[0] = drive.exaggerateJoystick(driveVector[0]);
//            drive.robotCentric(driveVector,gamepad1.right_stick_x);

//            telemetry.addData("FrontL", drive.motorPowers[0]);
//            telemetry.addData("FrontR", drive.motorPowers[1]);
//            telemetry.addData("BackL", drive.motorPowers[2]);
//            telemetry.addData("BackR", drive.motorPowers[3]);
//            telemetry.addData("rotate",gamepad1.right_stick_x);
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
