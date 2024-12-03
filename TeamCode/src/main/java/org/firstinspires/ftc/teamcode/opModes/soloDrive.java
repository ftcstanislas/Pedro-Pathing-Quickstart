package org.firstinspires.ftc.teamcode.opModes;

import com.acmerobotics.dashboard.FtcDashboard;
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

@TeleOp(name = "SoloDrive",group = "TeleOp")
public class soloDrive extends LinearOpMode {
    clawIntake intake = new clawIntake();
    outtake outtake = new outtake();
    MecanumDrivetrain drive = new MecanumDrivetrain();

    volatile Gamepad last = new Gamepad();
    volatile Gamepad current = new Gamepad();
    boolean clawOpen = false, intakeOpen = true, armManual = false;
    int target;

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

            if (current.dpad_up && !last.dpad_up) intake.setDiffy(servoPositions.clawDrop.getDifferential());
            else if (current.dpad_left && !last.dpad_left) intake.setDiffy(servoPositions.clawIntakeNarrow.getDifferential());
            else if (current.dpad_right && !last.dpad_right) intake.setDiffy(servoPositions.clawIntakeWide.getDifferential());

            if (current.left_trigger != 0 || current.right_trigger != 0) armManual = true;

            if (current.b) {
                target = 650;
                armManual = false;
            }
            else if (current.x) {
                target = 0;
                armManual = false;
            }

            if (armManual) {
                outtake.moveArm(-current.left_trigger + current.right_trigger);
            } else {
                outtake.armPID(target);
            }

            if (current.a && !last.a) {
                outtake.setClaw((clawOpen) ? servoPositions.outtakeGrip.getPosition() : servoPositions.outtakeRelease.getPosition()); //Toggle using the ternary operator, see GM260c.
                clawOpen ^= true;
            }

            intake.setSlidesWithLimit(0.4*current.right_stick_y);

            outtake.moveHook(-gamepad2.left_trigger + gamepad2.right_trigger);

//            drive.outdatedRobotCentric(drive.toPolar(current.left_stick_x,-current.left_stick_y), -current.right_stick_x);
            drive.robotCentric(-current.left_stick_y, current.left_stick_x, -current.right_stick_x);

            telemetry.addData("arm pos", outtake.arm.getCurrentPosition());
            telemetry.addData("slides pos", intake.slides.getCurrentPosition());
            telemetry.update();
        }
    }
}
