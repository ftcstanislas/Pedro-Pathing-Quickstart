package org.firstinspires.ftc.teamcode.opModes.tests;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robotParts.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.robotParts.intake;
import org.firstinspires.ftc.teamcode.robotParts.outtake;
import org.firstinspires.ftc.teamcode.robotParts.servoPositions;

import java.util.List;

@Disabled
@TeleOp(name = "DriveTest",group = "TeleOp")
public class driveTest extends LinearOpMode {
    intake intake = new intake();
    outtake outtake = new outtake();
    MecanumDrivetrain drive = new MecanumDrivetrain();

    double[] driveVector;

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
            intake.run(gamepad1.left_trigger - gamepad1.right_trigger);

            if (gamepad1.dpad_left || gamepad1.dpad_right) intake.setWrist(servoPositions.wristTransfer.getPosition());
            else if (gamepad1.dpad_down) intake.setWrist(servoPositions.wristIntake.getPosition());
            else if (gamepad1.dpad_up) intake.setWrist(servoPositions.wristAway.getPosition());

            if (gamepad1.x && Math.abs(intake.wristLeft.getPosition() - servoPositions.wristIntake.getPosition()) > 0.05) intake.setScissor(servoPositions.scissorRetract.getPosition());
            else if (gamepad1.y) intake.setScissor(servoPositions.scissorExtend.getPosition());

            outtake.moveBar(gamepad2.left_trigger - gamepad2.right_trigger);

            if (gamepad2.a) outtake.setBucket(servoPositions.outtakeReceive.getPosition());
            else if (gamepad2.b) outtake.setBucket(servoPositions.outtakeDrop.getPosition());

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
            telemetry.addData("wrist pos",intake.wristLeft.getPosition());
            telemetry.addData("scissor pos", intake.scissor.getPosition());
            telemetry.update();
        }
    }
}
