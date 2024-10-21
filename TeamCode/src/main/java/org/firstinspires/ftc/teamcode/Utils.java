package org.firstinspires.ftc.teamcode;

public class Utils {
    public static class Unwrap{
        double lastAngle = 0;
        double update(double angle) {
            double delta = angle - lastAngle;
            if(delta > 180){
                angle -=  360;
            } else if (delta < -180){
                angle +=  360;
            }
            lastAngle = angle;
            return angle;
        }
    }
    public static double signRoot(double x){
        return Math.sqrt(Math.abs(x)) * Math.signum(x);
    }
}
