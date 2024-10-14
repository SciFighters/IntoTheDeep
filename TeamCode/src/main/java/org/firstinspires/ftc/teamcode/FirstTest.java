/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import lombok.Getter;
import lombok.Setter;

class Unwrap{
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
@Config
class SteeringServo {

    private double power = 0;
    private CRServo servo;
    private AnalogInput encoder;

    @Getter double currentAngle ;
    @Getter double targetAngle;

    public static @Getter @Setter double p = 1;
    double min=0.007, max=3.277 ;

    Unwrap unwrap = new Unwrap();
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
        unwrapAngle = unwrap.update(currentAngle);
        double error = calcDeltaAngle(targetAngle, currentAngle);
        power = -error/180 * p;
        setPower(power);
    }

    double getUnwrappedAngle(){
        return unwrapAngle;
    }
}

class SteeringMotor {
    double power = 0;
    DcMotor motor;
    SteeringMotor(DcMotor motor) {
        this.motor = motor;
    }

    void setPower( double power) {
        this.power = power;
        motor.setPower(power);
    }

    double getPower() {
        return power;
    }

    void setDirection(DcMotor.Direction direction){
        motor.setDirection(direction);
    }

    double getEncoderValue() {
        return motor.getCurrentPosition();
    }
}

class SwerveWheel{
    SteeringServo servo;
    SteeringMotor motor;
    SwerveWheel(DcMotor motor, CRServo servo, AnalogInput encoder){
        this.motor = new SteeringMotor(motor);
        this.servo = new SteeringServo(servo, encoder);
    }

    void setHeading(double angle){
        double delta = servo.calcDeltaAngle(angle, servo.getCurrentAngle());
        if(Math.abs(delta) > 90) {
            angle = (angle + 180) % 360;
        }
        servo.setTargetAngle(angle);
    }
    void setPower(){

    }
}


@TeleOp(name="First", group="Tests")
//@Disabled
@Config
public class FirstTest extends LinearOpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime timer = new ElapsedTime();
    double servoPower = 0, motorPower = 0;
    private SteeringMotor motor = null;
    private SteeringServo servo = null;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();
    MultipleTelemetry multipleTelemetry = new MultipleTelemetry(telemetry, dashboardTelemetry);
    public static double t1 =0, t2 =90;


    @Override
    public void runOpMode() {
        multipleTelemetry.addData("Status", "Initialized");
        multipleTelemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        motor = new SteeringMotor(hardwareMap.get(DcMotor.class, "motor"));
        servo = new SteeringServo(hardwareMap.get(CRServo.class, "servo"), hardwareMap.analogInput.get("encoder"));

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        motor.setDirection(DcMotor.Direction.REVERSE);

        // Wait for the game to start (driver presses START)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            if (timer.seconds() > 2) {
                if(motorPower == 0){
                    servo.setTargetAngle(t1);
                    motorPower = 0.2;
                } else {
                    motorPower = 0;
                    servo.setTargetAngle(t2);
                }
                timer.reset();
            }

            motor.setPower(motorPower);
            servo.update();
            // Show the elapsed game time and wheel power.
            multipleTelemetry.addData("servo voltage", servo.getEncoderVoltage());
            multipleTelemetry.addData("motor encoder", motor.getEncoderValue());
            multipleTelemetry.addData("Max", servo.max);
            multipleTelemetry.addData("Min", servo.min);
            multipleTelemetry.addData("current angle", servo.getUnwrappedAngle());
            multipleTelemetry.addData("target angle", servo.targetAngle);
            multipleTelemetry.update();
        }
    }
}
