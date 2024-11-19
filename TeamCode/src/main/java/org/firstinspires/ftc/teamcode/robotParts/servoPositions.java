package org.firstinspires.ftc.teamcode.robotParts;

public enum servoPositions {
    intakeGrip(0.15),
    intakeRelease(0.5),
    scissorExtend(0.40),
    scissorRetract(1.0),
    outtakeDrop(1.0),
    outtakeReceive(0.29),
    outtakeInit(0.5),
    clawGrip(0.15),
    clawRelease(0.36),
    armIntake(0),
    armOuttake(0.8),
    intakeFront(new double[]{0.0,0.0}),
    intakeBack(new double[]{1,0}),
    transfer(new double[]{0.85,0.23}),
    sideTransfer(new double[]{0.59,0.0});

    private double position;
    public double getPosition(){return this.position;}

    servoPositions(double position) {
        this.position = position;
    }

    private double[] differential;
    public double[] getDifferential(){return this.differential;}
    servoPositions(double[] differential){this.differential = differential;}
}
