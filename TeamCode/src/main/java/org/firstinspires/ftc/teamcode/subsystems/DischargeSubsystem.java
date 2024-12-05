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
    private final Servo gearBoxServo, clawServo;
    double servoDischargePos = 0, servoClimbPos = 0.1;
    double clawServoHoldPos = 0.1, clawServoReleasePos = 0;
    MultipleTelemetry telemetry;

    public DischargeSubsystem(HardwareMap hardwareMap, MultipleTelemetry telemetry) {
        motor1 = hardwareMap.dcMotor.get("dischargeMotor1");
        motor2 = hardwareMap.dcMotor.get("dischargeMotor2");
        gearBoxServo = hardwareMap.servo.get("gearBoxServo");
        clawServo = hardwareMap.servo.get("dischargeServo");
        motor2.setDirection(DcMotorSimple.Direction.REVERSE);

        this.telemetry = telemetry;
    }

    public void setPower(double power) {
        if (motor1.getCurrentPosition() <= 0 || motor2.getCurrentPosition() <= 0) {
            motor1.setPower((Range.clip(power, 0, 1)));
            motor2.setPower((Range.clip(power, 0, 1)));
        } else {
            motor1.setPower(power);
            motor2.setPower(power);
        }
    }

    public void climbMode() {
        gearBoxServo.setPosition(servoClimbPos);
    }

    public void dischargeMode() {
        gearBoxServo.setPosition(servoDischargePos);
    }

    public void holdSample() {
        clawServo.setPosition(clawServoHoldPos);
    }

    public void releaseSample() {
        clawServo.setPosition(clawServoReleasePos);
    }

}
