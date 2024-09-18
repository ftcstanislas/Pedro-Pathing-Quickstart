package org.firstinspires.ftc.teamcode.robotParts;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class outtake {

    public DcMotorEx slide,outtake;

    private Servo claw,intake;

    public void init(HardwareMap map) {
        slide = map.get(DcMotorEx.class, "slide");
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        outtake = map.get(DcMotorEx.class, "outtake");
        outtake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        outtake.setDirection(DcMotorSimple.Direction.REVERSE);

        claw = map.get(Servo.class, "claw");
        intake = map.get(Servo.class, "intake");
    }
    public void claw(double position){claw.setPosition(position);}

    public void moveSlide(double slidePowerLocal){slide.setPower(slidePowerLocal);}

    public void moveOuttake(double outtakePowerLocal){outtake.setPower(outtakePowerLocal);}

    public void intake(double pos) {intake.setPosition(pos);}
}
