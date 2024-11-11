package org.firstinspires.ftc.teamcode.robotParts;

public enum servoPositions {
    wristIntake(0.0),
    wristAway(0.62),
    wristTransfer(0.25),
    scissorExtend(0.60),
    scissorRetract(1.0),
    outtakeDrop(1.0),
    outtakeReceive(0.29),
    outtakeInit(0.5),
    clawGrip(0.15),
    clawRelease(0.36),
    armIntake(0),
    armOuttake(0.8);

    private double position;
    public double getPosition(){return this.position;}

    servoPositions(double position) {
        this.position = position;
    }
}
