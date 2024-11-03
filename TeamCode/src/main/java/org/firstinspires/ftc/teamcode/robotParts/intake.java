package org.firstinspires.ftc.teamcode.robotParts;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class intake {
    public enum intakeSequence {
        IDLING,
        GET_READY,
        ALIGN,
        DROP,
        ROLLING,
        RAISE,
        RETRACT,
        TRANSFER,
        AWAY
    }

    double time;

    public intakeSequence state = intakeSequence.IDLING;

    public Servo wrist,scissor;

    CRServo intakeServo;

    public void init(HardwareMap map) {
        wrist = map.get(Servo.class, "wrist");
        wrist.setPosition(servoPositions.wristTransfer.getPosition());

        scissor = map.get(Servo.class,"scissor");
        scissor.setDirection(Servo.Direction.REVERSE);
        scissor.setPosition(servoPositions.scissorRetract.getPosition());

        intakeServo = map.get(CRServo.class,"intakeRight");
    }

    public void setWrist(double position){wrist.setPosition(position);}
    public void setScissor(double position){scissor.setPosition(position);}
    public void run(double power){intakeServo.setPower(power);}

    public void manualSequence(boolean toggle, double power, boolean reset) {
        switch (state) {
            case IDLING:
                if (toggle) {
                    state = intakeSequence.GET_READY;
                    //wrist should be in transfer
                    //scissor should be retracted
                    time = System.currentTimeMillis();
                }
                break;
            case GET_READY:
                if (time + 500 < System.currentTimeMillis()) {
                    state = intakeSequence.ALIGN;
                    setScissor(servoPositions.scissorExtend.getPosition());
                    time = System.currentTimeMillis();
                }
                break;
            case ALIGN:
                if (toggle) {
                    state = intakeSequence.DROP;
                    setWrist(servoPositions.wristIntake.getPosition());
                    time = System.currentTimeMillis();
                }
                break;
            case DROP:
                state = intakeSequence.ROLLING;
                break;
            case ROLLING:
                run(power);
                if (toggle) {
                    state = intakeSequence.RAISE;
                    run(0);
                    setWrist(servoPositions.wristTransfer.getPosition());
                    time = System.currentTimeMillis();
                }
                break;
            case RAISE:
                if (time + 300 < System.currentTimeMillis()) {
                    state = intakeSequence.RETRACT;
                    setScissor(servoPositions.scissorRetract.getPosition());
                    time = System.currentTimeMillis();
                }
                break;
            case RETRACT:
                if (toggle) {
                    state = intakeSequence.TRANSFER;
                    setScissor(0.8);
                    time = System.currentTimeMillis();
                }
                break;
            case TRANSFER:
                if (time + 600 < System.currentTimeMillis()) {
                    run(-0.5);
                    state = intakeSequence.AWAY;
                } else if (time + 300 < System.currentTimeMillis()) {
                    setWrist(servoPositions.wristAway.getPosition());
                }
                break;
            case AWAY:
                if (toggle) {
                    state = intakeSequence.IDLING;
                    run(0);
                    setWrist(servoPositions.wristTransfer.getPosition());
                    setScissor(servoPositions.scissorRetract.getPosition());
                    //Camera 10 frames/second
                }
                break;
        }
        if (reset) {
            state = intakeSequence.IDLING;
            run(0);
            setWrist(servoPositions.wristTransfer.getPosition());
            setScissor(servoPositions.scissorRetract.getPosition());
            //Camera 10 frames/second
        }
    }
    public void sequence(boolean toggle, boolean reset) {
        switch (state) {
            case IDLING:
                if (toggle) {
                    state = intakeSequence.GET_READY;
                    //wrist should be in transfer
                    //scissor should be retracted
                    time = System.currentTimeMillis();
                    //TODO: put outtake out of the way
                    //TODO: Camera full frames/second
                }
                break;
            case GET_READY:
                //TODO: find best sample
                if (time + 500 < System.currentTimeMillis()) {
                    state = intakeSequence.ALIGN;
                    //TODO: extend scissor with determined amount
                    setScissor(servoPositions.scissorExtend.getPosition());
                    time = System.currentTimeMillis();
                }
                break;
            case ALIGN:
                //TODO: drive and rotate
                if (toggle) {
                    state = intakeSequence.DROP;
                    setWrist(servoPositions.wristIntake.getPosition());
                    time = System.currentTimeMillis();
                }
                break;
            case DROP:
                state = intakeSequence.ROLLING;
                break;
            case ROLLING:
                run(1);
                if (toggle) {
                    state = intakeSequence.RAISE;
                    run(0);
                    setWrist(servoPositions.wristTransfer.getPosition());
                    time = System.currentTimeMillis();
                }
                break;
            case RAISE:
                if (time + 300 < System.currentTimeMillis()) {
                    state = intakeSequence.RETRACT;
                    setScissor(servoPositions.scissorRetract.getPosition());
                    time = System.currentTimeMillis();
                }
                break;
            case RETRACT:
                if (toggle) {
                    state = intakeSequence.TRANSFER;
                    setScissor(0.8);
                    time = System.currentTimeMillis();
                }
                break;
            case TRANSFER:
                if (time + 600 < System.currentTimeMillis()) {
                    run(-0.5);
                    state = intakeSequence.AWAY;
                } else if (time + 300 < System.currentTimeMillis()) {
                    setWrist(servoPositions.wristAway.getPosition());
                }
                break;
            case AWAY:
                if (toggle) {
                    state = intakeSequence.IDLING;
                    run(0);
                    setWrist(servoPositions.wristTransfer.getPosition());
                    setScissor(servoPositions.scissorRetract.getPosition());
                    //Camera 10 frames/second
                }
                break;
        }
        if (reset) {
            state = intakeSequence.IDLING;
            run(0);
            setWrist(servoPositions.wristTransfer.getPosition());
            setScissor(servoPositions.scissorRetract.getPosition());
            //Camera 10 frames/second
        }
    }
}
