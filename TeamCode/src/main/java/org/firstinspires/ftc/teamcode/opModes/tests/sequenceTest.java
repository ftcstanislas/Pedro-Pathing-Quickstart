package org.firstinspires.ftc.teamcode.opModes.tests;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.robotParts.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.robotParts.intake;

import java.util.List;

@Disabled
@TeleOp(name = "sequenceTest",group = "TeleOp")
public class sequenceTest extends LinearOpMode {
    intake intake = new intake();
    MecanumDrivetrain drive = new MecanumDrivetrain();

    volatile Gamepad last = new Gamepad();
    volatile Gamepad current = new Gamepad();

    @Override
    public void runOpMode() throws InterruptedException {
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        intake.init(hardwareMap);
        drive.init(hardwareMap);

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            last.copy(current);
            current.copy(gamepad1);

            intake.manualSequence(current.x && !last.x, current.right_trigger - current.left_trigger, current.y);

            drive.robotCentric(-current.left_stick_y, current.left_stick_x, -current.right_stick_x);
            telemetry.addData("outtakeLeft power", gamepad1.left_stick_y);
            telemetry.addData("wrist pos",intake.wrist.getPosition());
            telemetry.addData("scissor pos", intake.scissor.getPosition());
            telemetry.addData("state",String.valueOf(intake.state));
            telemetry.update();
        }
    }
}
