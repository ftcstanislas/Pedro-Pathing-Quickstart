package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robotParts.MecanumDrivetrain;

import java.util.List;

@TeleOp(name = "STADrive",group = "TeleOp")
public class STAdrive extends LinearOpMode {
    MecanumDrivetrain drive = new MecanumDrivetrain();

    Servo servo;
    DcMotorEx arm;
    Gamepad last = new Gamepad();
    Gamepad current = new Gamepad();
    boolean toggle;

    @Override
    public void runOpMode() throws InterruptedException {
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        drive.init(hardwareMap);
        servo = hardwareMap.get(Servo.class, "servo");
        arm = hardwareMap.get(DcMotorEx.class, "arm");
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            last.copy(current);
            current.copy(gamepad1);
            if (current.right_bumper && !last.right_bumper) {
                if (toggle) {
                    servo.setPosition(0.1);
                    toggle = false;
                } else {
                    servo.setPosition(0.7);
                    toggle = true;
                }
            }
//            if (gamepad1.left_bumper) servo.setPosition(0.1);
//            else if (gamepad1.right_bumper) servo.setPosition(0.7);

            arm.setPower(-gamepad1.right_trigger + gamepad1.left_trigger);

            drive.robotCentric(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

            telemetry.addData("maxPower",drive.maxPower);
            telemetry.addData("outtakeLeft power", gamepad1.left_stick_y);
            telemetry.update();
        }
    }
}
