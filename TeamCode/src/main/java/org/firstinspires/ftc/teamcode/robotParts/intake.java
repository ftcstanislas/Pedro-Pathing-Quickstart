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

    public Servo wristLeft, wristRight,scissor;//TODO unpublic

    CRServo intakeServo;

    public void init(HardwareMap map) {
        wristLeft = map.get(Servo.class, "wristLeft");

        wristRight = map.get(Servo.class, "wristRight");
        setDiffy(servoPositions.transfer.getDifferential());

        scissor = map.get(Servo.class,"scissor");
        scissor.setDirection(Servo.Direction.REVERSE);
        scissor.setPosition(servoPositions.scissorRetract.getPosition());

        intakeServo = map.get(CRServo.class,"intake");
    }

    @Deprecated
    public void setWrist(double position){wristLeft.setPosition(position);}
    public void setDiffy(double[] positions) {
        wristLeft.setPosition(positions[0]);
        wristRight.setPosition(positions[1]);
    }
    public void setScissor(double position){scissor.setPosition(position);}
    public void run(double power){intakeServo.setPower(power);}

    public void dualSequence(boolean toggle, double power, double deltaScissor, boolean reset) {
        switch (state) {
            case IDLING:
                //wrist should be in transfer
                //scissor should be retracted
                if (toggle) {
                    state = intakeSequence.GET_READY;
                }
                break;
            case GET_READY:
                state = intakeSequence.ALIGN;
                time = System.currentTimeMillis();
                break;
            case ALIGN:
                setScissor(scissor.getPosition() + deltaScissor);
                if (toggle && time + 500 < System.currentTimeMillis()) {
                    state = intakeSequence.DROP;
                    setDiffy(servoPositions.intakeFront.getDifferential());
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
                    setDiffy(servoPositions.transfer.getDifferential());
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
                    setDiffy(servoPositions.sideTransfer.getDifferential());
                    time = System.currentTimeMillis();
                }
                break;
            case TRANSFER:
                if (time + 500 < System.currentTimeMillis()) {
                    run(-0.5);
                    state = intakeSequence.AWAY;
                }
                break;
            case AWAY:
                if (toggle) {
                    state = intakeSequence.IDLING;
                    run(0);
                    setDiffy(servoPositions.transfer.getDifferential());
                    setScissor(servoPositions.scissorRetract.getPosition());
                    //Camera 10 frames/second
                }
                break;
        }
        if (reset) {
            state = intakeSequence.IDLING;
            run(0);
            setDiffy(servoPositions.transfer.getDifferential());
            setScissor(servoPositions.scissorRetract.getPosition());
            //Camera 10 frames/second
        }
    }

    public void manualSequence(boolean toggle, double power, boolean reset) {
        switch (state) {
            case IDLING:
                //wrist should be in transfer
                //scissor should be retracted
                if (toggle) {
                    state = intakeSequence.GET_READY;
                }
                break;
            case GET_READY:
                state = intakeSequence.ALIGN;
                setScissor(servoPositions.scissorExtend.getPosition());
                time = System.currentTimeMillis();
                break;
            case ALIGN:
                if (toggle && time + 500 < System.currentTimeMillis()) {
                    state = intakeSequence.DROP;
                    setDiffy(servoPositions.intakeFront.getDifferential());
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
                    setDiffy(servoPositions.transfer.getDifferential());
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
                    setDiffy(servoPositions.sideTransfer.getDifferential());
                    time = System.currentTimeMillis();
                }
                break;
            case TRANSFER:
                if (time + 500 < System.currentTimeMillis()) {
                    run(-0.5);
                    state = intakeSequence.AWAY;
                }
                break;
            case AWAY:
                if (toggle) {
                    state = intakeSequence.IDLING;
                    run(0);
                    setDiffy(servoPositions.transfer.getDifferential());
                    setScissor(servoPositions.scissorRetract.getPosition());
                    //Camera 10 frames/second
                }
                break;
        }
        if (reset) {
            state = intakeSequence.IDLING;
            run(0);
            setDiffy(servoPositions.transfer.getDifferential());
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
                    //TODO: put outtake out of the way
                    //TODO: Camera full frames/second
                }
                break;
            case GET_READY:
                //TODO: find best sample
                state = intakeSequence.ALIGN;
                setScissor(servoPositions.scissorExtend.getPosition());
                //TODO: extend scissor with determined amount
                time = System.currentTimeMillis();
                break;
            case ALIGN:
                //TODO: drive and rotate
                if (toggle && time + 500 < System.currentTimeMillis()) {
                    state = intakeSequence.DROP;
                    setDiffy(servoPositions.intakeFront.getDifferential());
                    time = System.currentTimeMillis();
                }
                break;
            case DROP:
                state = intakeSequence.ROLLING;
                break;
            case ROLLING:
                run(1);
                if (toggle) {//TODO: touch sensor
                    state = intakeSequence.RAISE;
                    run(0);
                    setDiffy(servoPositions.transfer.getDifferential());
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
                    setDiffy(servoPositions.sideTransfer.getDifferential());
                    time = System.currentTimeMillis();
                }
                break;
            case TRANSFER:
                if (time + 500 < System.currentTimeMillis()) {
                    run(-0.5);
                    state = intakeSequence.AWAY;
                }
                break;
            case AWAY:
                if (toggle) {
                    state = intakeSequence.IDLING;
                    run(0);
                    setDiffy(servoPositions.transfer.getDifferential());
                    setScissor(servoPositions.scissorRetract.getPosition());
                    //Camera 10 frames/second
                }
                break;
        }
        if (reset) {
            state = intakeSequence.IDLING;
            run(0);
            setDiffy(servoPositions.transfer.getDifferential());
            setScissor(servoPositions.scissorRetract.getPosition());
            //Camera 10 frames/second
        }
    }
}
