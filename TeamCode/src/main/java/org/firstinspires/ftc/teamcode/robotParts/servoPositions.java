package org.firstinspires.ftc.teamcode.robotParts;

public enum servoPositions {
    wristOpen(0.45),
    wristClosed(0.0),
    scissorExtend(0.75),
    scissorRetract(1.0);

    private double position;
    public double getPosition(){return this.position;}

    servoPositions(double position) {
        this.position = position;
    }
}
