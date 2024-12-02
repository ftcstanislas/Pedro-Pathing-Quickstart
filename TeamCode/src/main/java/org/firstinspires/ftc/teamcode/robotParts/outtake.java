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
        UP,
        FIDDLE
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

    public DcMotorEx barLeft, barRight, arm, hookLeft, hookRight;

    Servo bucket, claw, armServo;

    double leftPos, rightPos, armPos, time;

    final double p = 0.3, i = 0, d = 0.015;
    int ticks;
    double power;

    PIDController pid = new PIDController(p,i,d);

    public void init(HardwareMap map) {
        arm = map.get(DcMotorEx.class, "arm");
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm.setDirection(DcMotorSimple.Direction.REVERSE);

        hookLeft = map.get(DcMotorEx.class,"hookLeft");
        hookLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hookLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        hookRight = map.get(DcMotorEx.class,"hookRight");
        hookRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hookRight.setDirection(DcMotorSimple.Direction.REVERSE);

//        bucket = map.get(Servo.class, "outtake");
//        bucket.setPosition(servoPositions.bucketInit.getPosition());

        claw = map.get(Servo.class, "claw");
        claw.setPosition(servoPositions.outtakeGrip.getPosition());

//        armServo = map.get(Servo.class, "armServo");
//        armServo.setPosition(servoPositions.armIntake.getPosition());

        pid.setPID(p,i,d);
    }
    @Deprecated
    public void setBucket(double position){bucket.setPosition(position);}

    @Deprecated
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
    @Deprecated
    public void barPID(int target) {
        rightPos = barRight.getCurrentPosition();
        power = pid.calculate(ticks,target);
        moveBar(power,0);
    }

    public void moveArm(double outtakePowerLocal) {arm.setPower(outtakePowerLocal);}

    public void moveHook(double hookPowerLocal) {
        hookLeft.setPower(hookPowerLocal);
        hookRight.setPower(hookPowerLocal);
    }

    public void armPID(int target) {
        armPos = arm.getCurrentPosition();
        power = pid.calculate(armPos,target);
        if (power > 0.5) power = 0.5;
        else if (power < -0.5) power = -0.5;
        moveArm(power);
    }

    @Deprecated
    public void setArmServo(double position) {armServo.setPosition(position);}

    public void setClaw(double position) {claw.setPosition(position);}

    public void specimenSequence(boolean toggle, double power, boolean reset) {
        switch (specimenState) {
            case IDLING:
                if (toggle) {
                    specimenState = specimenSequence.UP;
                    setClaw(servoPositions.outtakeGrip.getPosition());
                    //claw should be open
                    //arm should be down
                    time = System.currentTimeMillis();
                }
                break;
            case UP:
                if (leftPos - armPositions.HIGH_RUNG.getPosition() < 10) {
                    specimenState = specimenSequence.FIDDLE;
                } else {
                    armPID(armPositions.HIGH_RUNG.getPosition());
                }
                break;
            case FIDDLE:
                moveArm(power);
                if (toggle) {
                    specimenState = specimenSequence.IDLING;
                    setClaw(servoPositions.outtakeRelease.getPosition());
                }
                break;
        }
        if (reset) {
            setClaw(servoPositions.outtakeRelease.getPosition());
            specimenState = specimenSequence.IDLING;
        }
    }
}
