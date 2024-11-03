package org.firstinspires.ftc.teamcode.robotParts;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class outtake {

    public DcMotorEx barLeft, barRight, winch;

    public Servo outtake, claw;

    public void init(HardwareMap map) {
        barLeft = map.get(DcMotorEx.class, "outtakeLeft");
        barLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        barLeft.setDirection(DcMotorSimple.Direction.FORWARD);

        barRight = map.get(DcMotorEx.class, "outtakeRight");
        barRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        barRight.setDirection(DcMotorSimple.Direction.FORWARD);

        winch = map.get(DcMotorEx.class, "winch");
        winch.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        winch.setDirection(DcMotorSimple.Direction.FORWARD);

        outtake = map.get(Servo.class, "outtake");
        outtake.setPosition(servoPositions.outtakeInit.getPosition());

        claw = map.get(Servo.class, "claw");
        claw.setPosition(servoPositions.clawGrip.getPosition());
    }
    public void setOuttake(double position){outtake.setPosition(position);}

    //TODO: PIDF
    public void moveBar(double outtakePowerLocal){
        barLeft.setPower(outtakePowerLocal);
        barRight.setPower(outtakePowerLocal);
    }

    public void setWinch(double power) {winch.setPower(power);}

    public void setClaw(double position) {claw.setPosition(position);}
}
