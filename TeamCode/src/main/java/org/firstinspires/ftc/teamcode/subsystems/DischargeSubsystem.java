package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

public class DischargeSubsystem extends SubsystemBase {
    private final DcMotor motor1, motor2;
    private final Servo gearBoxServo;
    double servoDischargePos = 0, servoClimbPos = 1;
    double clawServoHoldPos = 0.1, clawServoReleasePos = 0;
    MultipleTelemetry telemetry;
    int gearBoxRatio = 1;

    public DischargeSubsystem(HardwareMap hardwareMap, MultipleTelemetry telemetry) {
        motor1 = hardwareMap.dcMotor.get("dischargeMotor1");
        motor2 = hardwareMap.dcMotor.get("dischargeMotor2");
        gearBoxServo = hardwareMap.servo.get("gearBoxServo");
//        clawServo = hardwareMap.servo.get("dischargeServo");
        motor1.setDirection(DcMotorSimple.Direction.REVERSE);

        this.telemetry = telemetry;
    }

    public void setPower(double power) {
        telemetry.addData("tiiick", motor2.getCurrentPosition());
        telemetry.addData("gearboxRatio",gearBoxRatio);
        telemetry.addData("servoPos",gearBoxServo.getPosition());

        telemetry.update();
        if (motor2.getCurrentPosition() <= 0) {
            motor1.setPower((Range.clip(power, 0, 1)));
            motor2.setPower((Range.clip(power, 0, 1)));
        }else if (motor2.getCurrentPosition() >= 1628 * gearBoxRatio) {
            motor1.setPower((Range.clip(power, -1, 0)));
            motor2.setPower((Range.clip(power, -1, 0)));
        } else {
            motor1.setPower(power);
            motor2.setPower(power);
        }
    }

    public void climbMode() {
        gearBoxServo.setPosition(servoClimbPos);
        gearBoxRatio = 9;
    }

    public void dischargeMode() {
        gearBoxServo.setPosition(servoDischargePos);
        gearBoxRatio = 1;
    }
    public int getGearBoxRatio(){
        return gearBoxRatio;
    }

    public void resetEncoders() {
        motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

//    public void holdSample() {
//        clawServo.setPosition(clawServoHoldPos);
//    }
//
//    public void releaseSample() {
//        clawServo.setPosition(clawServoReleasePos);
//    }

}
