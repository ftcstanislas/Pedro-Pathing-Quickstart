package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class drive {
    public static class drivetrain {

        private DcMotorEx leftFront;
        private DcMotorEx rightFront;
        private DcMotorEx leftBack;
        private DcMotorEx rightBack;

        public void init(HardwareMap map) {
            leftFront = map.get(DcMotorEx.class, "left_front");
            rightFront = map.get(DcMotorEx.class, "right_front");
            leftBack = map.get(DcMotorEx.class, "left_back");
            rightBack = map.get(DcMotorEx.class, "right_back");

            leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        public void drive(double forward, double right, double rotate) {
            double leftFrontPower = forward + right + rotate;
            double rightFrontPower = forward - right - rotate;
            double rightRearPower = forward + right - rotate;
            double leftRearPower = forward - right + rotate;
            double maxPower = 1.0;

            maxPower = Math.max(maxPower, Math.abs(leftFrontPower));
            maxPower = Math.max(maxPower, Math.abs(rightFrontPower));
            maxPower = Math.max(maxPower, Math.abs(rightRearPower));
            maxPower = Math.max(maxPower, Math.abs(leftRearPower));

            leftFront.setPower(leftFrontPower / maxPower);
            rightFront.setPower(rightFrontPower / maxPower);
            rightBack.setPower(rightRearPower / maxPower);
            leftBack.setPower(leftRearPower / maxPower);
        }
    }
}