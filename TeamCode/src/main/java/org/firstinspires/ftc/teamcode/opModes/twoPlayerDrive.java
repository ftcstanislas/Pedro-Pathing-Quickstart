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
@TeleOp(name = "DuoDrive",group = "TeleOp")
public class twoPlayerDrive extends LinearOpMode {
    intake intake = new intake();
    outtake outtake = new outtake();
    MecanumDrivetrain drive = new MecanumDrivetrain();

    volatile Gamepad last1 = new Gamepad();
    volatile Gamepad current1 = new Gamepad();
    volatile Gamepad last2 = new Gamepad();
    volatile Gamepad current2 = new Gamepad();

    boolean clawOpen = false, armScoring = false, scissorOut = false;

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
            last1.copy(current1);
            last2.copy(current2);
            current1.copy(gamepad1);
            current2.copy(gamepad2);

            intake.run(current2.right_trigger - current2.left_trigger);

            if (scissorOut) {
                if (current2.dpad_down) {
                    intake.setDiffy(servoPositions.intakeFront.getDifferential());
                } else if (current2.dpad_right) {
                    intake.setDiffy(servoPositions.sideTransfer.getDifferential());
                }
                if (current2.a && !last2.a) {
                    scissorOut = false;
                    intake.setScissor(servoPositions.scissorRetract.getPosition());
                    intake.setDiffy(servoPositions.transfer.getDifferential());
                }
            } else {
                if (current2.b && !last2.b) {
                    scissorOut = true;
                    intake.setScissor(servoPositions.scissorExtend.getPosition());
                    intake.setDiffy(servoPositions.sideTransfer.getDifferential());
                }
            }

            outtake.moveBar((-current1.left_trigger + current1.right_trigger) * 0.7,0);

            if (current1.a && !last1.a) {
                outtake.setClaw((clawOpen) ? servoPositions.clawGrip.getPosition() : servoPositions.clawRelease.getPosition()); //Toggle using the ternary operator, see GM260c.
                clawOpen ^= true;
            }
            if (current1.x && !last1.x) {
                outtake.setArm((armScoring) ? servoPositions.armIntake.getPosition() : servoPositions.armOuttake.getPosition()); //Toggle using the ternary operator, see GM260c.
                armScoring ^= true;
            }

            drive.robotCentric(-current1.left_stick_y, current1.left_stick_x, -current1.right_stick_x);

            telemetry.addData("maxPower",drive.maxPower);
            telemetry.addData("outtakeLeft power", current1.left_stick_y);
            telemetry.addData("wrist pos",intake.wristLeft.getPosition());
            telemetry.addData("scissor pos", intake.scissor.getPosition());
            telemetry.update();
        }
    }
}
