package org.firstinspires.ftc.teamcode.robotParts;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class outtake {

    public DcMotorEx barLeft, barRight, winch;

    Servo bucket, claw, arm;

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

        bucket = map.get(Servo.class, "outtake");
        bucket.setPosition(servoPositions.outtakeInit.getPosition());

        claw = map.get(Servo.class, "claw");
        claw.setPosition(servoPositions.clawGrip.getPosition());

        arm = map.get(Servo.class, "arm");
        arm.setPosition(servoPositions.armIntake.getPosition());
    }
    public void setBucket(double position){
        bucket.setPosition(position);}

    //TODO: PIDF
    public void moveBar(double outtakePowerLocal){
        barLeft.setPower(outtakePowerLocal);
        barRight.setPower(outtakePowerLocal);
    }

    public void setArm(double position) {arm.setPosition(position);}

    public void setWinch(double power) {winch.setPower(power);}

    public void setClaw(double position) {claw.setPosition(position);}
}
