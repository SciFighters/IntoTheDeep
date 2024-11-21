package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.Utils;

import lombok.Getter;
import lombok.Setter;

@Config
public class SteeringServo {
    public static double noiseGain = 0.05;
    public static double noiseCuttoff = 0.5;
    public static double w = -0.7;
    public static double p = 0.5;

    private boolean idle = false;
    private double power = 0;
    private CRServo servo;
    private AnalogInput encoder;
    private double angleOffset;
    @Getter
    double currentAngle ;
    @Getter double targetAngle;

    double min=0.007, max=3.277 ;

    public SteeringServo(CRServo servo, AnalogInput encoder, double headingOffset) {
        this.servo = servo;
        this.encoder = encoder;
        this.angleOffset = headingOffset;
    }

    public void setPower(double power) {
        this.power = power;
        servo.setPower(power);
    }

    public void setTargetAngle(double target){
        targetAngle = target;
    }

    public double getTargetAngle(){
        return targetAngle;
    }
    double getEncoderVoltage(){
        double v = encoder.getVoltage();
        return v;
    }

    public double getCurrentAngle(){
        double v = getEncoderVoltage();
        currentAngle = ((v-min)/(max-min))*360 - angleOffset;
        return currentAngle;
    }
    public void setAngleOffset(double angle){
        angleOffset = angle;
    }
    public double getAngleOffset(){
        return angleOffset;
    }
    public void zeroHeading() {
        angleOffset = 0;
        angleOffset = getCurrentAngle();
    }

    public double calcDeltaAngle(double target, double current) {
        double delta = target - current;
        if(delta > 180){
            delta = delta - 360;
        }else if(delta < -180){
            delta = 360+ delta;
        }
        return delta;
    }

    public void update() {
        double currentAngle = getCurrentAngle();

        double error = calcDeltaAngle(targetAngle, currentAngle);
        double ne = -error /90;  // negative normalized error (-1..1)

        power = (w >= 0) ? (Utils.signRoot(ne) * (w) + ne * (1 - w)) * p :
                (ne * Math.abs(ne) *(Math.abs(w)) + ne *(1 + w)) * p ;

        if(Math.abs(error) > noiseCuttoff) {
            power += Math.signum(power) * Math.random() * noiseGain;
        }
        setPower(power);
    }


}
