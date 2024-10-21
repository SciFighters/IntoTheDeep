package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class SwerveModule {
    SteeringServo servo;
    DcMotor motor;
    double isMotorFlipped = 1;
    SwerveModule(DcMotor motor, CRServo servo, AnalogInput encoder){
        this.motor = motor;
        this.motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.servo = new SteeringServo(servo, encoder);
    }

    void setHeading(double angle){
//        if (isMotorFlipped < 0) {
//            angle = (angle + 180) % 360;
//        }
        double delta = servo.calcDeltaAngle(angle, servo.getCurrentAngle());
        if (Math.abs(delta) > 90) {
            angle = (angle + 180) % 360;
            isMotorFlipped = -1;
        } else {
            isMotorFlipped = 1;
        }
        servo.setTargetAngle(angle);
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
    public double getPosition(){
        return (double)motor.getCurrentPosition() / 100;
    }
    public void update(){
        servo.update();
    }
}
