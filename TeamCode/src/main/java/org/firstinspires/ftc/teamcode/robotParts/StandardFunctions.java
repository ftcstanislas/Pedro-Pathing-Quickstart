package org.firstinspires.ftc.teamcode.robotParts;

public class StandardFunctions {
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
    /**
     * Alts:
     * (r*r+r)/2 (only positive)
     * (r*r*r+r)/2
     * Math.sin(2*Math.PI*r)/12 + r
     * Math.sin(2*Math.PI*r)/8 + r*r
     * @param r
     * @return
     */
    public double exaggerateJoystick(double r) {return Math.sin(2*Math.PI*r) / 9 + r;}

    //TODO: one hardwareMap list for all the stuff on the robot, including ports
}