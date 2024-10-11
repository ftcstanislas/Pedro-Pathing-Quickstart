package org.firstinspires.ftc.teamcode.robotParts;

public enum servoPositions {
    wristIntake(0.0),
    wristAway(0.4),
    wristTransfer(0.25),
    scissorExtend(0.60),
    scissorRetract(1.0),
    outtakeDrop(1.0),
    outtakeReceive(0.29),
    outtakeInit(0.5);

    private double position;
    public double getPosition(){return this.position;}

    servoPositions(double position) {
        this.position = position;
    }
}
