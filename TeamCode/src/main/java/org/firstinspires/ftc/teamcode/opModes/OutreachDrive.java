package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robotParts.twoWheelDrivetrain;

@TeleOp(name = "OutreachDrive",group = "TeleOp")
public class OutreachDrive extends LinearOpMode {
    twoWheelDrivetrain drive = new twoWheelDrivetrain();

    @Override
    public void runOpMode() throws InterruptedException {
        drive.init(hardwareMap);
        waitForStart();

        while (opModeIsActive()){
            if (isStopRequested()){
                break;
            }
            drive.twoWheelDrive(gamepad1.left_stick_y, gamepad1.right_stick_x);
        }
    }
}


