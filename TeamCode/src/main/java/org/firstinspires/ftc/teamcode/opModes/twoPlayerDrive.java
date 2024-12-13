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

    boolean clawOpen = false, intakeOpen = true, armManual = true, keepIntake;
    int target;
    volatile Gamepad last1 = new Gamepad();
    volatile Gamepad current1 = new Gamepad();
    volatile Gamepad last2 = new Gamepad();
    volatile Gamepad current2 = new Gamepad();

    boolean armScoring = false, scissorOut = false;

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

            if (current2.dpad_up && !last2.dpad_up) intake.setDiffy(servoPositions.clawDrop.getDifferential());
            else if (current2.dpad_left && !last2.dpad_left) intake.setDiffy(servoPositions.clawIntakeNarrow.getDifferential());
            else if (current2.dpad_right && !last2.dpad_right) intake.setDiffy(servoPositions.clawIntakeWide.getDifferential());


//            if (current1.left_trigger != 0 || current1.right_trigger != 0) armManual = true;
//
//            if (current1.b) {
//                target = 650;
//                armManual = false;
//            }
//            else if (current1.x) {
//                target = 0;
//                armManual = false;
//            }

            if (armManual) {
                outtake.moveArm(-current1.left_trigger + current1.right_trigger);
            } else {
                outtake.armPID(target);
            }

            if (current1.a && !last1.a) {
                outtake.setClaw((clawOpen) ? servoPositions.outtakeGrip.getPosition() : servoPositions.outtakeRelease.getPosition()); //Toggle using the ternary operator, see GM260c.
                clawOpen ^= true;
            }

            if (current2.right_bumper && !last2.right_bumper) {
                intake.setKeepSlides((keepIntake) ? servoPositions.keepSlides.getPosition() : servoPositions.releaseSlides.getPosition()); //Toggle using the ternary operator, see GM260c.
                keepIntake ^= true;
            }

            intake.setSlidesWithLimit(0.4*current2.right_stick_y);

            outtake.moveHook(-gamepad2.left_trigger + gamepad2.right_trigger);

//            drive.outdatedRobotCentric(drive.toPolar(current.left_stick_x,-current.left_stick_y), -current.right_stick_x);
            drive.robotCentric(-current1.left_stick_y, current1.left_stick_x, current1.right_stick_x);

            telemetry.addData("arm pos", outtake.arm.getCurrentPosition());
            telemetry.addData("slides pos", intake.slides.getCurrentPosition());
            telemetry.update();
        }
    }
}