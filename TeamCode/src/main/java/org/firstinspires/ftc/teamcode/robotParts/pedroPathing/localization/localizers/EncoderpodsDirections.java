package org.firstinspires.ftc.teamcode.robotParts.pedroPathing.localization.localizers;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.robotParts.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.robotParts.outtake;
import org.firstinspires.ftc.teamcode.robotParts.intake;

import java.util.List;

@TeleOp(name = "EncoderpodsDirections",group = "TeleOp")
public class EncoderpodsDirections extends LinearOpMode {
    intake intake = new intake();
    outtake outtake = new outtake();
    MecanumDrivetrain drive = new MecanumDrivetrain();

    @Override
    public void runOpMode() throws InterruptedException {

        drive.init(hardwareMap);

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            telemetry.addData("leftFront", drive.FrontL.getCurrentPosition());
            telemetry.addData("rightFront", drive.FrontR.getCurrentPosition());
            telemetry.addData("leftBack", drive.BackL.getCurrentPosition());
            telemetry.addData("rightBack", drive.BackR.getCurrentPosition());

            telemetry.update();
        }
    }
}
