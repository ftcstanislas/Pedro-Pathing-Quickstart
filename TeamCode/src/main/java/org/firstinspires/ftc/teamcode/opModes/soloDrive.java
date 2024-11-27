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
import org.firstinspires.ftc.teamcode.robotParts.rollerIntake;
import org.firstinspires.ftc.teamcode.robotParts.outtake;
import org.firstinspires.ftc.teamcode.robotParts.servoPositions;

import java.util.List;

@Config
@TeleOp(name = "SoloDrive",group = "TeleOp")
public class soloDrive extends LinearOpMode {
    clawIntake intake = new clawIntake();
    outtake outtake = new outtake();
    MecanumDrivetrain drive = new MecanumDrivetrain();

    volatile Gamepad last = new Gamepad();
    volatile Gamepad current = new Gamepad();
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
            last.copy(current);
            current.copy(gamepad1);

            if (current.y && !last.y) {
                intake.setClaw((intakeOpen) ? servoPositions.intakeGrip.getPosition() : servoPositions.intakeRelease.getPosition()); //Toggle using the ternary operator, see GM260c.
                intakeOpen ^= true;
            }
            if (scissorOut) {
                if (current.dpad_left) {
                    intake.setDiffy(servoPositions.clawIntakeNarrow.getDifferential());
                } else if (current.dpad_right) {
                    intake.setDiffy(servoPositions.clawIntakeWide.getDifferential());
                }
                if (current.left_bumper && !last.left_bumper) {
                    scissorOut = false;
                    intake.setScissor(servoPositions.scissorRetract.getPosition());
                    intake.setDiffy(servoPositions.clawDrop.getDifferential());
                }
            } else {
                if (current.right_bumper && !last.right_bumper) {
                    scissorOut = true;
                    intake.setScissor(servoPositions.scissorExtend.getPosition());
                    intake.setClaw(servoPositions.intakeRelease.getPosition());
                }
            }

            outtake.moveBar((-current.left_trigger + current.right_trigger) * 0.7,0);

            if (current.a && !last.a) {
                outtake.setClaw((clawOpen) ? servoPositions.outtakeGrip.getPosition() : servoPositions.outtakeRelease.getPosition()); //Toggle using the ternary operator, see GM260c.
                clawOpen ^= true;
            }
            if (current.x && !last.x) {
                outtake.setArmServo((armScoring) ? servoPositions.armIntake.getPosition() : servoPositions.armOuttake.getPosition()); //Toggle using the ternary operator, see GM260c.
                armScoring ^= true;
            }

            outtake.moveHook(gamepad2.left_trigger - gamepad2.right_trigger);

//            drive.robotCentric(-current.left_stick_y, current.left_stick_x, -current.right_stick_x);
            drive.outdatedRobotCentric(drive.toPolar(current.left_stick_x,-current.left_stick_y), -current.right_stick_x);

            drive.robotCentric(-current.left_stick_y, current.left_stick_x, -current.right_stick_x);

            telemetry.addData("maxPower",drive.maxPower);
            telemetry.addData("outtakeLeft power", current.left_stick_y);
            telemetry.addData("wrist pos",intake.wristLeft.getPosition());
            telemetry.addData("scissor pos", intake.scissor.getPosition());
            telemetry.update();
        }
    }
}
