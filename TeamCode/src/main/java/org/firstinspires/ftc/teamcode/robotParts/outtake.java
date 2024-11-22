package org.firstinspires.ftc.teamcode.robotParts;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class outtake {
    public enum specimenSequence {
        IDLING,
        GRAB,
        UP,
        FIDDLE,
        RELEASE,
        DROP
    }
    enum armPositions {
        DOWN(0),
        LOW_RUNG(0),
        HIGH_RUNG(1000),
        LOW_BASKET(0),
        HIGH_BASKET(2000);

        private int position;
        public int getPosition(){return this.position;}

        armPositions(int position) {
            this.position = position;
        }
    }

    public specimenSequence specimenState = specimenSequence.IDLING;

    public DcMotorEx barLeft, barRight;

    Servo bucket, claw, arm;

    double leftPos, rightPos, time;

    final double k = 0, p = 0, i = 0, d = 0;
    int ticks;
    double power;

    PIDController pid = new PIDController(p,i,d);

    public void init(HardwareMap map) {
        barLeft = map.get(DcMotorEx.class, "outtakeLeft");
        barLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        barLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        barRight = map.get(DcMotorEx.class, "outtakeRight");
        barRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        barRight.setDirection(DcMotorSimple.Direction.FORWARD);

        bucket = map.get(Servo.class, "outtake");
        bucket.setPosition(servoPositions.bucketInit.getPosition());

        claw = map.get(Servo.class, "claw");
        claw.setPosition(servoPositions.outtakeGrip.getPosition());

        arm = map.get(Servo.class, "arm");
        arm.setPosition(servoPositions.armIntake.getPosition());

        pid.setPID(p,i,d);
    }
    public void setBucket(double position){
        bucket.setPosition(position);}

    public void moveBar(double outtakePowerLocal, double k){//TODO: tune k
        leftPos = barLeft.getCurrentPosition();
        rightPos = barRight.getCurrentPosition();
        if (leftPos > rightPos) {
            barLeft.setPower(outtakePowerLocal);
            barRight.setPower(outtakePowerLocal + k * (leftPos - rightPos));
        }
        else {
            barLeft.setPower(outtakePowerLocal - k * (leftPos - rightPos));
            barRight.setPower(outtakePowerLocal);
        }
    }

    //TODO: tune values
    public void barPID(int target) {
        rightPos = barRight.getCurrentPosition();
        power = pid.calculate(ticks,target);
        moveBar(power,k);
    }

    public void setArm(double position) {arm.setPosition(position);}

    public void setClaw(double position) {claw.setPosition(position);}

    public void specimenSequence(boolean toggle, double power, boolean reset) {
        switch (specimenState) {
            case IDLING:
                if (toggle) {
                    specimenState = specimenSequence.GRAB;
                    setClaw(servoPositions.outtakeGrip.getPosition());
                    //claw should be open
                    //bar should be down
                    time = System.currentTimeMillis();
                }
                break;
            case GRAB:
                if (time + 300 < System.currentTimeMillis()) {
                    specimenState = specimenSequence.UP;
                }
                break;
            case UP:
                if (leftPos - armPositions.HIGH_RUNG.getPosition() < 10) {
                    specimenState = specimenSequence.FIDDLE;
                } else {
                    barPID(armPositions.HIGH_RUNG.getPosition());
                }
                break;
            case FIDDLE:
                moveBar(power,k);
                if (toggle) {
                    specimenState = specimenSequence.RELEASE;
                }
                break;
            case RELEASE:
                setClaw(servoPositions.outtakeRelease.getPosition());
                time = System.currentTimeMillis();
                specimenState = specimenSequence.DROP;
                break;
            case DROP:
                if (time + 300 < System.currentTimeMillis()) {
                    if (leftPos - armPositions.DOWN.getPosition() < 10) {
                        specimenState = specimenSequence.IDLING;
                    } else {
                        barPID(armPositions.DOWN.getPosition());
                    }
                }
                break;
        }
        if (reset) {
            setClaw(servoPositions.outtakeRelease.getPosition());
            specimenState = specimenSequence.DROP;
        }
    }
}
