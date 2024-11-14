package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.SteeringServo;

public class SwerveModule {
    public SteeringServo servo;
    DcMotor motor;
    double isMotorFlipped = 1;
    double headingOffset;

    public SwerveModule(DcMotor motor, CRServo servo, AnalogInput encoder, double headingOffset){
        this.motor = motor;
        this.motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.servo = new SteeringServo(servo, encoder,headingOffset);
        this.headingOffset = headingOffset;
    }
    public void zeroHeading(){
        isMotorFlipped = 1;
        servo.zeroHeading();
    }

    public void setHeading(double angle){
        double delta = servo.calcDeltaAngle(angle, servo.getCurrentAngle());
        if (Math.abs(delta) > 90) {
            angle = (angle + 180) % 360;
            isMotorFlipped = -1;
        } else {
            isMotorFlipped = 1;
        }
        servo.setTargetAngle(angle);
    }
    public void setServoPower(double power){
        servo.setPower(power);
    }
    public double getCurrentHeading(){
        return servo.getCurrentAngle();
    }
    public double getTargetHeading(){
        return servo.getTargetAngle();
    }
    public void setPower(double power){
        motor.setPower(power * isMotorFlipped);
    }
    public double getPower(){
        return motor.getPower();
    }
    public double getPosition(){
        return (double)motor.getCurrentPosition() / 100;
    }
    public void update(){
        servo.update();
    }
}
