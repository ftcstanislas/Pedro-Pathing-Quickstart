package org.firstinspires.ftc.teamcode.robotParts;

public enum servoPositions {
    intakeGrip(0.5),
    intakeRelease(0.05),
    scissorExtend(0.40),
    scissorRetract(1.0),
    bucketDrop(1.0),
    bucketReceive(0.29),
    bucketInit(0.5),
    outtakeGrip(0.5),
    outtakeRelease(0.05),
    armIntake(0),
    armOuttake(0.8),
    rollerFront(new double[]{0.0,0.0}),
    rollerBack(new double[]{1,0}),
    rollerTransfer(new double[]{0.85,0.23}),
    rollerSide(new double[]{0.59,0.0}),
    clawDrop(new double[]{1.0, 0.5}),
    clawIntakeWide(new double[]{0.2,0.2}),
    clawIntakeNarrow(new double[]{0.5,0.5});

    private double position;
    public double getPosition(){return this.position;}

    servoPositions(double position) {
        this.position = position;
    }

    private double[] differential;
    public double[] getDifferential(){return this.differential;}
    servoPositions(double[] differential){this.differential = differential;}
}
