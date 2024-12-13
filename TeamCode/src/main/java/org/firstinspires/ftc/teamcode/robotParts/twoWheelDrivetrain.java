package org.firstinspires.ftc.teamcode.robotParts;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class twoWheelDrivetrain extends StandardFunctions{
    public DcMotorEx Left,Right;

    public void init(HardwareMap map) {
        Left = map.get(DcMotorEx.class, "left");
        Right = map.get(DcMotorEx.class, "right");

        Left.setDirection(DcMotorSimple.Direction.REVERSE);
        Right.setDirection(DcMotorSimple.Direction.FORWARD);

        Left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public void twoWheelDrive(double forward, double rotate) {
        double leftPower = forward - rotate;
        double rightPower = forward + rotate;
        double maxPower = 1.0;

        maxPower = Math.max(maxPower, Math.abs(leftPower));
        maxPower = Math.max(maxPower, Math.abs(rightPower));

        Left.setPower(leftPower / maxPower);
        Right.setPower(rightPower / maxPower);
    }


}
