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

    int outtakeCase = 0;

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
            last2.copy(current2);
            last1.copy(current1);
            current2.copy(gamepad2);
            current1.copy(gamepad1);

            if (current2.y && !last2.y) {
                intake.setClaw((intakeOpen) ? servoPositions.intakeGrip.getPosition() : servoPositions.intakeRelease.getPosition()); //Toggle using the ternary operator, see GM260c.
                intakeOpen ^= true;
            }

            if (current2.dpad_up && !last2.dpad_up)
                intake.setDiffy(servoPositions.clawDrop.getDifferential());
            else if (current2.dpad_left && !last2.dpad_left)
                intake.setDiffy(servoPositions.clawIntakeNarrow.getDifferential());
            else if (current2.dpad_right && !last2.dpad_right)
                intake.setDiffy(servoPositions.clawIntakeWide.getDifferential());

            switch (outtakeCase) {
                case 0:
                    if (current1.a && !last1.a) {
                        outtake.setClaw(servoPositions.outtakeGrip.getPosition());
                        outtakeCase++;
                    }
                    break;
                case 1:
                    if (current1.a && !last1.a) {
                        outtakeCase++;
                    }
                    break;
                case 2:
                    outtake.moveArm(0.7);
                    if (!current1.a && last1.a) {
                        outtake.setClaw(servoPositions.outtakeRelease.getPosition());
                        outtakeCase++;
                    }
                    break;
                case 3:
                    outtake.armPID(0);
                    if (outtake.arm.getCurrentPosition() < 20) {
                        outtake.moveArm(0);
                        outtakeCase = 0;
                    }

                    if (current1.a && !last1.a) {
                        outtake.setClaw((clawOpen) ? servoPositions.outtakeGrip.getPosition() : servoPositions.outtakeRelease.getPosition()); //Toggle using the ternary operator, see GM260c.
                        clawOpen ^= true;
                    }

                    intake.setSlidesWithLimit(0.4 * current2.right_stick_y);

                    outtake.moveHook(-gamepad2.left_trigger + gamepad2.right_trigger);

//            drive.outdatedRobotCentric(drive.toPolar(current.left_stick_x,-current.left_stick_y), -current.right_stick_x);
                    drive.robotCentric(-current1.left_stick_y, current1.left_stick_x, -current1.right_stick_x);

                    telemetry.addData("arm pos", outtake.arm.getCurrentPosition());
                    telemetry.addData("slides pos", intake.slides.getCurrentPosition());
                    telemetry.update();
            }
        }
    }
}
