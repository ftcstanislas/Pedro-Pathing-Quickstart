package org.firstinspires.ftc.teamcode.robotParts;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class STTslides {

    public DcMotorEx slide,outtake;

    private Servo wrist,claw,intake;

    public void init(HardwareMap map) {
        slide = map.get(DcMotorEx.class, "slide");
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        outtake = map.get(DcMotorEx.class, "outtake");
        outtake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        outtake.setDirection(DcMotorSimple.Direction.REVERSE);

        wrist = map.get(Servo.class, "wrist");
        claw = map.get(Servo.class, "claw");
        intake = map.get(Servo.class, "intake");
    }

    public void wrist(double position){wrist.setPosition(position);}
    public void claw(double position){claw.setPosition(position);}

    public void move(double slidePowerLocal){slide.setPower(slidePowerLocal);}

    public void run(double outtakePowerLocal){outtake.setPower(outtakePowerLocal);}
    public void intake(double pos) {intake.setPosition(pos);}
}
