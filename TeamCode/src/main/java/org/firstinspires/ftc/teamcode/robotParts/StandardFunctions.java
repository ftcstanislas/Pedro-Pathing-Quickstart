package org.firstinspires.ftc.teamcode.robotParts;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class StandardFunctions {
    public DcMotorEx FrontL,FrontR,BackL,BackR;

    int j;

    final double
            //TODO: tune angle
            angle = Math.atan(Math.sqrt(2)),
            xConstant = toCartesian(1,angle)[0],
            yConstant = toCartesian(1,angle)[1];

    final double[]
            baseLVector = toPolar(xConstant, yConstant),
            baseRVector = toPolar(-xConstant, yConstant),
            motorWeights = {1.0,1.0,1.0,1.0};

    double yL,yR,minYValue,powerMultiplier,maxPower;
    double[]
            motorPowers = {0,0,0,0},
            LVector,RVector,sumVector = {0,0};

    public void init(HardwareMap map) {
        FrontL = map.get(DcMotorEx.class, "left_front");
        FrontR = map.get(DcMotorEx.class, "right_front");
        BackL = map.get(DcMotorEx.class, "left_back");
        BackR = map.get(DcMotorEx.class, "right_back");

        FrontR.setDirection(DcMotorSimple.Direction.REVERSE);
        BackR.setDirection(DcMotorSimple.Direction.REVERSE);

        FrontL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    /**
     * TODO: documentation, EN
     * @param cartesian
     * @param normalise
     * @return
     */
    public double[] toPolar(double[] cartesian, boolean normalise) {
        return toPolar(cartesian[0],cartesian[1],normalise);
    }

    /**
     * TODO: documentation, EN
     * @param cartesian
     * @return
     */
    public double[] toPolar(double[] cartesian) {
        return toPolar(cartesian[0],cartesian[1],false);
    }

    /**
     * TODO: documentation, EN
     * @param x
     * @param y
     * @return
     */
    public double[] toPolar(double x, double y) {
        return toPolar(x, y, false);
    }

    /**
     * TODO: documentation, EN
     * @param x
     * @param y
     * @param normalise
     * @return
     */
    public double[] toPolar(double x, double y, boolean normalise) {
        double r = Math.sqrt(x * x + y * y);
        double theta = Math.atan2(y,x);
        if (normalise) {
            while (theta > Math.PI) {
                theta -= 2 * Math.PI;
            }
            while (theta <= -Math.PI) {
                theta += 2 * Math.PI;
            }
        }
        return new double[]{r,theta};
    }

    /**
     * TODO: documentation, EN
     * @param polar
     * @return
     */
    public double[] toCartesian(double[] polar) {
        return toCartesian(polar[0],polar[1]);
    }

    /**
     * TODO: documentation, EN
     * @param r
     * @param theta
     * @return
     */
    public double[] toCartesian(double r, double theta) {
        double x = r * Math.cos(theta);
        double y = r * Math.sin(theta);
        return new double[]{x,y};
    }
    public double exaggerateJoystick(double r) {return Math.sin(2*Math.PI*r) / 9 + r;}
}