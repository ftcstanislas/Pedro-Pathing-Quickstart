package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robotParts.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.robotParts.STTslides;

@TeleOp(name = "STTdrive",group = "TeleOp")
public class STTdrive extends LinearOpMode {
    MecanumDrivetrain drivetrain = new MecanumDrivetrain();
    STTslides slides = new STTslides();

    @Override
    public void runOpMode() throws InterruptedException {
        drivetrain.init(hardwareMap);
        slides.init(hardwareMap);

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_x; // y direction is reversed
            double x = gamepad1.left_stick_y;
            double rotate = -gamepad1.right_stick_x;

            boolean wristOn = gamepad1.a;
            boolean wristOff = gamepad1.b;
            boolean clawOn = gamepad1.x;
            boolean clawOff = gamepad1.y;

            double slidePower = (gamepad1.right_trigger - gamepad1.left_trigger);
            double slidePos = slides.slide.getCurrentPosition();
            double outtakePower = (gamepad2.right_trigger - gamepad2.left_trigger);

            if (slidePos > 0 && slidePos < 2300) {
                slides.move(slidePower);
            } else if (slidePos < 0 && slidePower > 0){
                slides.move(slidePower);
            } else if (slidePos > 2300 && slidePower < 0) {
                slides.move(slidePower);
            }

            if (wristOn) {
                slides.wrist(0.2);
            } else if (wristOff) {
                slides.wrist(0);
            }

            if (clawOn) {
                slides.claw(0.4);
            } else if (clawOff) {
                slides.claw(0);
            }

            drivetrain.drive(x, y, rotate);
            slides.run(outtakePower);
            telemetry.addData("slidehoogte: ",slidePos);
            telemetry.update();
        }
    }
}
