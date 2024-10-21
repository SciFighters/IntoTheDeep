package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;

import lombok.Getter;
import lombok.Setter;

public class SteeringServo {

    private double power = 0;
    private final double w = -0.7;
    private CRServo servo;
    private AnalogInput encoder;

    @Getter
    double currentAngle ;
    @Getter double targetAngle;

    public static @Getter @Setter double p = 0.5;
    double min=0.007, max=3.277 ;

    double unwrapAngle = 0;

    SteeringServo(CRServo servo, AnalogInput encoder) {
        this.servo = servo;
        this.encoder = encoder;
    }

    void setPower( double power) {
        this.power = power;
        servo.setPower(power);
    }

    void setTargetAngle(double target){
        targetAngle = target;
    }

    double getTargetAngle(){
        return targetAngle;
    }
    double getEncoderVoltage(){
        double v = encoder.getVoltage();
//        if(v > max){
//            max = v;
//        }else if(v < min){
//            min = v;
//        }
        return v;
    }

    double getCurrentAngle(){
        double v = getEncoderVoltage();
        currentAngle = ((v-min)/(max-min))*360;
        return currentAngle;
    }

    double calcDeltaAngle(double target, double current) {
        double delta = target - current;
        if(delta > 180){
            delta = delta - 360;
        }else if(delta < -180){
            delta = 360+ delta;
        }
        return delta;
    }

    void update() {
        double currentAngle = getCurrentAngle();

        double error = calcDeltaAngle(targetAngle, currentAngle);
        double ne = -error /90;

        power = (w >= 0) ? (Utils.signRoot(ne) * (w) + ne * (1 - w)) * p :
                (ne * Math.abs(ne) *(Math.abs(w)) + ne *(1 + w)) * p ;
//        power = -error/180 * p;
        setPower(power);
    }

    double getUnwrappedAngle(){
        return unwrapAngle;
    }
}
