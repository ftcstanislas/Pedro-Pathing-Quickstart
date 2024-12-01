package org.firstinspires.ftc.teamcode.robotParts;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class clawIntake {
    public enum intakeSequence {
        IDLING,
        GET_READY,
        ALIGN,
        DROP,
        GRAB,
        RAISE,
        RETRACT,
        TRANSFER,
        AWAY
    }

    double time;

    public intakeSequence state = intakeSequence.IDLING;

    public Servo differentialLeft, differentialRight, scissor, intakeClaw;//TODO unpublic

    public DcMotorEx slides;

    public static double k = 0, p = 0, i = 0, d = 0;

    PIDController pid = new PIDController(p,i,d);

    final int slideMax = 0, slideMin = -480;

    final double maxExtend = 41.0, maxWristAngle = 390.0, minWristAngle = 60.0;

    int slidePos;

    public void init(HardwareMap map) {
        differentialLeft = map.get(Servo.class, "wristLeft");

        differentialRight = map.get(Servo.class, "wristRight");
//        setDiffy(servoPositions.clawIntakeNarrow.getDifferential()); //TODO: starting pos

        slides = map.get(DcMotorEx.class, "slides");
        slides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        intakeClaw = map.get(Servo.class,"intake");

        pid.setPID(p,i,d);
    }

    @Deprecated
    public void setWrist(double position){differentialLeft.setPosition(position);}

    public void setDiffy(double[] positions) {
        differentialLeft.setPosition(positions[0]);
        differentialRight.setPosition(positions[1]);
    }
    public void setDiffyAngle(double angle) {
        while (angle > maxWristAngle) {
            angle -= 180;
        }
        while (angle < minWristAngle) {
            angle += 180;
        }
        setDiffy(new double[] {(angle - minWristAngle) / (maxWristAngle - minWristAngle), (angle - minWristAngle) / (maxWristAngle - minWristAngle)});
    }
    public void setScissor(double position){scissor.setPosition(position);}

    public void setClaw(double position){intakeClaw.setPosition(position);}

    public void setSlidesWithLimit(double power){
        slidePos = slides.getCurrentPosition();
        slides.setPower(((power < 0 && slidePos < slideMin) || (power > 0 && slidePos > slideMax)) ? 0 : power);
    }

    public void slidesPID(int position) {
        slidePos = slides.getCurrentPosition();
        pid.calculate(position,slidePos);
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
                    setDiffy(servoPositions.rollerFront.getDifferential());
                    time = System.currentTimeMillis();
                }
                break;
            case DROP:
                state = intakeSequence.GRAB;
                setClaw(servoPositions.intakeGrip.getPosition());
                break;
            case GRAB:
                if (toggle) {
                    state = intakeSequence.RAISE;
                    setClaw(servoPositions.intakeRelease.getPosition());
                    setDiffy(servoPositions.rollerTransfer.getDifferential());
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
                    setDiffy(servoPositions.rollerSide.getDifferential());
                    time = System.currentTimeMillis();
                }
                break;
            case TRANSFER:
                if (time + 500 < System.currentTimeMillis()) {
                    setClaw(servoPositions.intakeRelease.getPosition());
                    state = intakeSequence.AWAY;
                }
                break;
            case AWAY:
                if (toggle) {
                    state = intakeSequence.IDLING;
                    setDiffy(servoPositions.rollerTransfer.getDifferential());
                    setScissor(servoPositions.scissorRetract.getPosition());
                    //Camera 10 frames/second
                }
                break;
        }
        if (reset) {
            state = intakeSequence.IDLING;
            setClaw(servoPositions.intakeRelease.getPosition());
            setDiffy(servoPositions.rollerTransfer.getDifferential());
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
                    setDiffy(servoPositions.rollerFront.getDifferential());
                    time = System.currentTimeMillis();
                }
                break;
            case DROP:
                state = intakeSequence.GRAB;
                setClaw(servoPositions.intakeGrip.getPosition());
                break;
            case GRAB:
                if (toggle) {//TODO: touch sensor
                    state = intakeSequence.RAISE;
                    setClaw(servoPositions.intakeRelease.getPosition());
                    setDiffy(servoPositions.rollerTransfer.getDifferential());
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
                    setDiffy(servoPositions.rollerSide.getDifferential());
                    time = System.currentTimeMillis();
                }
                break;
            case TRANSFER:
                if (time + 500 < System.currentTimeMillis()) {
                    setClaw(servoPositions.intakeRelease.getPosition());
                    state = intakeSequence.AWAY;
                }
                break;
            case AWAY:
                if (toggle) {
                    state = intakeSequence.IDLING;
                    setDiffy(servoPositions.rollerTransfer.getDifferential());
                    setScissor(servoPositions.scissorRetract.getPosition());
                    //Camera 10 frames/second
                }
                break;
        }
        if (reset) {
            state = intakeSequence.IDLING;
            setClaw(servoPositions.intakeRelease.getPosition());
            setDiffy(servoPositions.rollerTransfer.getDifferential());
            setScissor(servoPositions.scissorRetract.getPosition());
            //Camera 10 frames/second
        }
    }
}
