package org.firstinspires.ftc.teamcode.robotParts;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class outtake {

    public DcMotorEx outtakeLeft, outtakeRight, winch;

    public Servo outtake;

    public void init(HardwareMap map) {
        outtakeLeft = map.get(DcMotorEx.class, "outtakeLeft");
        outtakeLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        outtakeLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        outtakeRight = map.get(DcMotorEx.class, "outtakeRight");
        outtakeRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        outtakeRight.setDirection(DcMotorSimple.Direction.REVERSE);

        winch = map.get(DcMotorEx.class, "winch");
        winch.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        winch.setDirection(DcMotorSimple.Direction.FORWARD);

        outtake = map.get(Servo.class, "outtake");
        outtake.setPosition(servoPositions.outtakeInit.getPosition());
    }
    public void setOuttake(double position){outtake.setPosition(position);}

    public void moveBar(double outtakePowerLocal){
        outtakeLeft.setPower(outtakePowerLocal);
        outtakeRight.setPower(outtakePowerLocal);
    }

    public void setWinch(double power) {winch.setPower(power);}
}
