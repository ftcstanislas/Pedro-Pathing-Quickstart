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

    public void drive(double forward, double right, double rotate) {
        double rotatePower = 0.5;
        double leftFrontPower = forward + right + rotatePower*rotate;
        double rightFrontPower = forward - right - rotatePower*rotate;
        double rightRearPower = forward + right - rotatePower*rotate;
        double leftRearPower = forward - right + rotatePower*rotate;
        double maxPower = 1.0;

        maxPower = Math.max(maxPower, Math.abs(leftFrontPower));
        maxPower = Math.max(maxPower, Math.abs(rightFrontPower));
        maxPower = Math.max(maxPower, Math.abs(rightRearPower));
        maxPower = Math.max(maxPower, Math.abs(leftRearPower));

        FrontL.setPower(leftFrontPower / maxPower);
        FrontR.setPower(rightFrontPower / maxPower);
        BackR.setPower(rightRearPower / maxPower);
        BackL.setPower(leftRearPower / maxPower);
    }
    /**
     * This is more useful for use in OpModes.
     * @param values An array containing a drivePower, driveAngle and rotatePower.
     * @see <a href="#drive(double[], double)">drive()</a>, values follows the same limitations.
     */
    public void drive(double[] values) {drive(new double[] {values[0], values[1]}, values[2]);}

    /**
     * <p>This method does the kinematics to transfer a drive vector and a rotate power into usable motor powers for a mecanum drivetrain.
     * It also corrects for the friction in mecanum wheels which causes forward to be quicker than strafe.
     * Within the method, there is documentation for what each line does.
     * If you want a visual explanation, check out Wolfpack Machina's <a href="https://www.youtube.com/watch?v=ri06orPFaKo&t=135s">"Wolfpack Movement Breakdown</a> on Youtube,
     * from 2:15 to 4:34. We do not do that last thing with the scalar, as our kinematics do not get separate perpendicular powers, but only one drivePower.
     * It can receive these parameters from the pathFollower, the pidFollower, or a driver in Tele-Op.</p><p></p>
     * <h2>Tuning</h2>
     * <h3>Step 1</h3>
     * <p>Make sure initialization works correctly, and that all motors have been added to the configuration file.
     * If strafing doesn't drive straight, you should change the motorWeights. Downscale the side which is going too fast.</p><p></p>
     * <h3>Step 2</h3>
     * <p>Drive forwards at full speed, and drive backwards at full speed. Your angle value is the ratio forward / strafe, which is the inverse tangent.</p>
     * @param drivePower has to be a polar vector with two values: r and theta. R is a number between -1 and 1, but preferably between 0 and 1.
     *                   Theta is the angle, in radians, between the vector and the positive x-axis.
     * @param rotatePower should be an already calculated power, preferably using a PID.
     * @see <a href="#unoptimizedDrive(double[], double)">unoptimisedDrive</a>, as it may be slightly more readable.
     */
    public void drive(double[] drivePower, double rotatePower) {
        // Normalizes drivePower theta, because the math only works between -90 and 270 degrees (in radians).
        while (drivePower[1] > 1.5 * Math.PI) {
            drivePower[1] -= 2 * Math.PI;
        }
        while (drivePower[1] <= -0.5 * Math.PI) {
            drivePower[1] += 2 * Math.PI;
        }

        // Creates two new polar vectors. You only need two because the front left and back right vector are equal, and vice versa.
        // The drivePower theta is taken away from this, so that the drivePower theta aligns with the x-axis.
        // This makes getting the component of the motor vectors perpendicular to the drivePower really easy, as it's just the sine.
        LVector = new double[]{0.5 * drivePower[0], angle - drivePower[1]};
        RVector = new double[]{0.5 * drivePower[0], Math.PI - angle - drivePower[1]};

        // If the motor theta is more than 90 degrees away from the drivePower theta, it should be negated, because it's taking away from the robot speed.
        // Negating it makes the vector add to the robot speed.
        // Also, if you don't do this, you can't strafe.
        if (Math.abs(LVector[1]) > 0.5 * Math.PI) {
            LVector[0] *= -1;
        }
        if (Math.abs(RVector[1]) > 0.5 * Math.PI) {
            RVector[0] *= -1;
        }

        // This gets the smallest y-component from both motor vectors. The reason for this might seem vague, but the vector with the larger y-component is scaled down,
        // so the forces perpendicular to the drivePower cancel each other out, and you drive in the direction of the drivePower vector.
        yL = Math.abs(LVector[0] * Math.sin(LVector[1]));
        yR = Math.abs(RVector[0] * Math.sin(RVector[1]));
        minYValue = Math.min(yL, yR);
        LVector[0] *= minYValue / yL;
        RVector[0] *= minYValue / yR;

        // This code adds the vectors together. To do this easily, you need cartesian coordinates.
        sumVector = toPolar(LVector[0]*Math.cos(LVector[1])+RVector[0]*Math.cos(RVector[1]),LVector[0]*Math.sin(LVector[1])+RVector[0]*Math.sin(RVector[1]));
        //Alternative sumVector from https://www.youtube.com/watch?v=vr71A_UFt0A. No clue if this is more efficient or not. Definitely not as readable.
//        sumVector[1] = RVector[1]-LVector[1]+Math.PI;
//        sumVector[0] = Math.sqrt(LVector[0]*LVector[0]*RVector[0]*RVector[0]-2*LVector[0]*RVector[0]*Math.cos(sumVector[1]));
//        sumVector[1] = Math.asin(RVector[0]*Math.sin(sumVector[1])/sumVector[0]);

        // SumVector theta should be exactly the drivePower theta, but r might differ.
        // We then multiply by that difference, so that the sumVector is completely equal to the driveVector.
        // I have a feeling you can delete or optimise this bit, because we correct for the same thing again later and the drive vectors aren't changed from the beginning.
        // If you can delete this, do, because the calculation sucks.
        //TODO: can you live without this correction
        powerMultiplier = drivePower[0]/sumVector[0];
        LVector[0] = LVector[0] * powerMultiplier;
        RVector[0] = RVector[0] * powerMultiplier;
        sumVector[0] = sumVector[0] * powerMultiplier;

        // Next, the motor powers are calculated. As stated before, the vector for front left and back right is the same, but the rotation is different.
        // You also might want to add a different weight for each motor, this might be useful when your robots center of gravity is very much to one side.
        motorPowers[0] = (LVector[0] - rotatePower) * motorWeights[0];
        motorPowers[1] = (RVector[0] + rotatePower) * motorWeights[1];
        motorPowers[2] = (RVector[0] - rotatePower) * motorWeights[2];
        motorPowers[3] = (LVector[0] + rotatePower) * motorWeights[3];

        // All motors are scaled in such a way that the largest power is the same value as the length of the drivePower parameter.
        maxPower = Math.max(Math.abs(LVector[0]), Math.abs(RVector[0]));
        maxPower = Math.max(Math.abs(maxPower + rotatePower), Math.abs(maxPower - rotatePower));
        for (int i = 0; i < 4; i++) {
            motorPowers[i] /= (maxPower*drivePower[0]);
        }

        // Motor powers are sent to the motors.
        j = 0;
        for (DcMotorEx motor : new DcMotorEx[]{FrontL,FrontR,BackL,BackR}) {
            motor.setPower(motorPowers[j]);
            j++;
        }
    }

    /**
     * @see <a href="#drive(double[], double)">drive(double[], double)</a>. This method does the same thing in a slightly more comprehensible, but slower way.
     */
    @Deprecated
    public void unoptimizedDrive(double[] drivePower, double rotatePower) {
        LVector = new double[]{baseLVector[0],baseLVector[1]-drivePower[1]};
        RVector = new double[]{baseRVector[0],baseRVector[1]-drivePower[1]};

        if (Math.abs(LVector[1]) > 0.5 * Math.PI) {
            LVector[0] *= -1;
        }
        if (Math.abs(RVector[1]) > 0.5 * Math.PI) {
            RVector[0] *= -1;
        }

        yL = Math.abs(toCartesian(LVector)[1]);
        yR = Math.abs(toCartesian(RVector)[1]);
        minYValue = Math.min(yL, yR);
        LVector[0] *= minYValue/ yL;
        RVector[0] *= minYValue/ yR;

        sumVector = toPolar(2 * toCartesian(LVector)[0] + 2 * toCartesian(RVector)[0],2 * toCartesian(LVector)[1] + 2 * toCartesian(RVector)[1]);
        powerMultiplier = 2*drivePower[0]/sumVector[0];
        LVector[0] = LVector[0] * powerMultiplier;
        RVector[0] = RVector[0] * powerMultiplier;
        sumVector[0] = sumVector[0] * powerMultiplier;

        motorPowers[0] = (LVector[0] - rotatePower) * motorWeights[0];
        motorPowers[1] = (RVector[0] + rotatePower) * motorWeights[1];
        motorPowers[2] = (RVector[0] - rotatePower) * motorWeights[2];
        motorPowers[3] = (LVector[0] + rotatePower) * motorWeights[3];

        maxPower = Math.max(Math.abs(motorPowers[0]),Math.abs(motorPowers[1]));
        maxPower = Math.max(maxPower,Math.abs(motorPowers[2]));
        maxPower = Math.max(maxPower,Math.abs(motorPowers[3]));

        for (int i = 0; i < 4;i++) {
            motorPowers[i] /= (maxPower*drivePower[0]);
        }

        FrontL.setPower(motorPowers[0]);
        FrontR.setPower(motorPowers[1]);
        BackL.setPower(motorPowers[2]);
        BackR.setPower(motorPowers[3]);
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