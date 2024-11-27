package org.firstinspires.ftc.teamcode.opModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.robotParts.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.robotParts.clawIntake;
import org.firstinspires.ftc.teamcode.robotParts.outtake;
import org.firstinspires.ftc.teamcode.robotParts.servoPositions;

import java.util.List;

@Config
@TeleOp(name = "DuoDrive",group = "TeleOp")
public class twoPlayerDrive extends LinearOpMode {
    clawIntake intake = new clawIntake();
    outtake outtake = new outtake();
    MecanumDrivetrain drive = new MecanumDrivetrain();

    volatile Gamepad last1 = new Gamepad();
    volatile Gamepad current1 = new Gamepad();
    volatile Gamepad last2 = new Gamepad();
    volatile Gamepad current2 = new Gamepad();

    boolean clawOpen = false, armScoring = false, scissorOut = false, intakeOpen = true;

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

            if (current2.y && !last2.y) {
                intake.setClaw((intakeOpen) ? servoPositions.intakeGrip.getPosition() : servoPositions.intakeRelease.getPosition()); //Toggle using the ternary operator, see GM260c.
                intakeOpen ^= true;
            }
            if (scissorOut) {
                if (current2.dpad_left) {
                    intake.setDiffy(servoPositions.clawIntakeNarrow.getDifferential());
                } else if (current2.dpad_right) {
                    intake.setDiffy(servoPositions.clawIntakeWide.getDifferential());
                }
                if (current2.left_bumper && !last2.left_bumper) {
                    scissorOut = false;
                    intake.setScissor(servoPositions.scissorRetract.getPosition());
                    intake.setDiffy(servoPositions.clawDrop.getDifferential());
                }
            } else {
                if (current2.right_bumper && !last2.right_bumper) {
                    scissorOut = true;
                    intake.setScissor(servoPositions.scissorExtend.getPosition());
                    intake.setDiffy(servoPositions.clawIntakeNarrow.getDifferential());
                    intake.setClaw(servoPositions.intakeRelease.getPosition());
                }
            }

            outtake.moveBar((-current1.left_trigger + current1.right_trigger) * 0.7,0);

            if (current1.a && !last1.a) {
                outtake.setClaw((clawOpen) ? servoPositions.outtakeGrip.getPosition() : servoPositions.outtakeRelease.getPosition()); //Toggle using the ternary operator, see GM260c.
                clawOpen ^= true;
            }
            if (current1.x && !last1.x) {
                outtake.setArmServo((armScoring) ? servoPositions.armIntake.getPosition() : servoPositions.armOuttake.getPosition()); //Toggle using the ternary operator, see GM260c.
                armScoring ^= true;
            }

            drive.robotCentric(-current1.left_stick_y, current1.left_stick_x, -current1.right_stick_x);

            telemetry.addData("maxPower",drive.maxPower);
            telemetry.addData("outtakeLeft power", current1.left_stick_y);
            telemetry.addData("wrist pos",intake.differentialLeft.getPosition());
            telemetry.addData("scissor pos", intake.scissor.getPosition());
            telemetry.update();
        }
    }
}
