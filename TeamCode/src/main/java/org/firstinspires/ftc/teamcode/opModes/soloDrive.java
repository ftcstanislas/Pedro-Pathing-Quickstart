package org.firstinspires.ftc.teamcode.opModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.robotParts.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.robotParts.intake;
import org.firstinspires.ftc.teamcode.robotParts.outtake;
import org.firstinspires.ftc.teamcode.robotParts.servoPositions;

import java.util.List;

@Config
@TeleOp(name = "SoloDrive",group = "TeleOp")
public class soloDrive extends LinearOpMode {
    intake intake = new intake();
    outtake outtake = new outtake();
    MecanumDrivetrain drive = new MecanumDrivetrain();

    volatile Gamepad last = new Gamepad();
    volatile Gamepad current = new Gamepad();

    boolean clawOpen = false;

    double power;

    @Override
    public void runOpMode() throws InterruptedException {
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        outtake.init(hardwareMap);
        intake.init(hardwareMap);
        drive.init(hardwareMap);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            last.copy(current);
            current.copy(gamepad1);

            if (current.left_bumper && !last.left_bumper) power = -0.5;
            else if (current.right_bumper && !last.right_bumper) power = 1;
            else if ((last.left_bumper || last.right_bumper) && !current.right_bumper && !current.left_bumper) power = 0.0;

            intake.manualSequence(current.x && !last.x, power, current.y);

            outtake.moveBar(current.left_trigger - current.right_trigger);

            if (current.a && !last.a) {
                outtake.setClaw((clawOpen) ? servoPositions.clawGrip.getPosition() : servoPositions.clawRelease.getPosition()); //Toggle using the ternary operator, see GM260c.
                clawOpen ^= true;
            }

            drive.robotCentric(-current.left_stick_y, current.left_stick_x, -current.right_stick_x);

            telemetry.addData("maxPower",drive.maxPower);
            telemetry.addData("outtakeLeft power", current.left_stick_y);
            telemetry.addData("wrist pos",intake.wrist.getPosition());
            telemetry.addData("scissor pos", intake.scissor.getPosition());
            telemetry.addData("outtake pos", outtake.outtake.getPosition());
            telemetry.update();
        }
    }
}
