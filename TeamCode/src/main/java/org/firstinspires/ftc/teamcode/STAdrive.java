package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robotParts.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.robotParts.intake;
import org.firstinspires.ftc.teamcode.robotParts.outtake;
import org.firstinspires.ftc.teamcode.robotParts.servoPositions;

import java.util.List;

@TeleOp(name = "SoloDrive",group = "TeleOp")
public class STAdrive extends LinearOpMode {
    MecanumDrivetrain drive = new MecanumDrivetrain();

    Servo servo;
    DcMotorEx arm;

    @Override
    public void runOpMode() throws InterruptedException {
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        drive.init(hardwareMap);
        servo = hardwareMap.get(Servo.class, "servo");
        arm = hardwareMap.get(DcMotorEx.class, "arm");

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            if (gamepad1.left_bumper) servo.setPosition(0.0);
            else if (gamepad1.right_bumper) servo.setPosition(1.0);

            arm.setPower(gamepad1.right_trigger - gamepad1.left_trigger);

            drive.robotCentric(-gamepad1.left_stick_y, gamepad1.left_stick_x, -gamepad1.right_stick_x);

            telemetry.addData("maxPower",drive.maxPower);
            telemetry.addData("outtakeLeft power", gamepad1.left_stick_y);
            telemetry.update();
        }
    }
}
