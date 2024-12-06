package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class IntakeSubsystem extends SubsystemBase {
    private final DcMotor motor;
    private final Servo xServo, zServo, gripServo;
    private final CRServo spinServo;
    private final int maxArmLength = 357;
    MultipleTelemetry telemetry;

    //[][][][][][][][][][]
    public IntakeSubsystem(HardwareMap hardwareMap, MultipleTelemetry telemetry) {
        motor = hardwareMap.dcMotor.get("intakeMotor");
        xServo = hardwareMap.servo.get("xAxisServo");
        zServo = hardwareMap.servo.get("zAxisServo");
        gripServo = hardwareMap.servo.get("gripServo");
        spinServo = hardwareMap.crservo.get("spinServo");
        this.telemetry = telemetry;

    }    // make one button that extends the arm and lowers the claw while opening it

    public void setArmPower(double power) {
        if (motor.getCurrentPosition() <= 0) {
            motor.setPower(Range.clip(power, 0, 1));
        } else if (motor.getCurrentPosition() >= maxArmLength) {
            motor.setPower(Range.clip(power, -1, 0));
        } else {
            motor.setPower(power);
        }
    }

    public void setZServoPosition(double position) {
        zServo.setPosition(position);
    }

    public void setXServoPosition(double position) {
        xServo.setPosition(position);
    }

    public void setGripStage(GripStages stage) {
        gripServo.setPosition(stage.POSITION);
    }
    public double getGripServoPosition(){
        return gripServo.getPosition();
    }

    public void setSpinPower(double power) {
        spinServo.setPower(power);
    }

    public int getMotorPosition() {
        return motor.getCurrentPosition();
    }

    public double getXServoPosition() {
        return xServo.getPosition();
    }
    // make second button that while pressing it it goes to half of height and pushes things
    // away and then lowers one more stage and picks up the sample
    // button 3: checks angle and returns to base position
}
