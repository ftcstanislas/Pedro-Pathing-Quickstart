package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robotParts.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.robotParts.intake;
import org.firstinspires.ftc.teamcode.robotParts.outtake;

import java.util.List;

@TeleOp(name = "DriveTest",group = "TeleOp")
public class driveTest extends LinearOpMode {
    intake intake = new intake();
    outtake outtake = new outtake();
    MecanumDrivetrain drive = new MecanumDrivetrain();

    @Override
    public void runOpMode() throws InterruptedException {
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        intake.init(hardwareMap);
        outtake.init(hardwareMap);
        drive.init(hardwareMap);

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            intake.run(-gamepad1.right_stick_y);

            if (gamepad1.a) intake.wrist(0.0);
            else if (gamepad1.b) intake.wrist(0.45);
            if (gamepad1.x) intake.setScissor(1.0);
            else if (gamepad1.y) intake.setScissor(0.75);
            outtake.moveOuttake(gamepad1.left_stick_y);

            drive.robotCentric(gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);

            telemetry.addData("outtakeLeft power", gamepad1.left_stick_y);
            telemetry.addData("intake power: ",intake.CRLeft.getPower());
            telemetry.addData("wrist pos",intake.wrist.getPosition());
            telemetry.addData("scissor pos", intake.scissor.getPosition());
            telemetry.update();
        }
    }
}
