package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robotParts.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.robotParts.outtake;
import org.firstinspires.ftc.teamcode.robotParts.intake;

@TeleOp(name = "STTdrive",group = "TeleOp")
public class STTdrive extends LinearOpMode {
    MecanumDrivetrain drivetrain = new MecanumDrivetrain();
    outtake outtake = new outtake();
    intake intake = new intake();

    @Override
    public void runOpMode() throws InterruptedException {
        drivetrain.init(hardwareMap);
        outtake.init(hardwareMap);
        intake.init(hardwareMap);

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_x; // y direction is reversed
            double x = gamepad1.left_stick_y;
            double rotate = -gamepad1.right_stick_x;

            boolean clawOn = gamepad1.x;
            boolean clawOff = gamepad1.y;

            double slidePower = (gamepad1.right_trigger - gamepad1.left_trigger);
            double slidePos = outtake.slide.getCurrentPosition();
            double outtakePower = (gamepad2.right_trigger - gamepad2.left_trigger);

            if(gamepad1.left_bumper) {
                outtake.intake(1.0);
            } else if(gamepad1.right_bumper) {
                outtake.intake(0.77);
            }

            if (slidePos > 0 && slidePos < 2300) {
                outtake.moveSlide(slidePower);
            } else if (slidePos < 0 && slidePower > 0){
                outtake.moveSlide(slidePower);
            } else if (slidePos > 2300 && slidePower < 0) {
                outtake.moveSlide(slidePower);
            }

            if (clawOn) {
                outtake.claw(0.4);
            } else if (clawOff) {
                outtake.claw(0);
            }

            intake.run(gamepad2.left_stick_y);
            intake.wrist(gamepad2.right_stick_x);

            drivetrain.drive(x, y, rotate);
            outtake.moveOuttake(outtakePower);
            telemetry.addData("slidehoogte: ",slidePos);
            telemetry.update();
        }
    }
}
