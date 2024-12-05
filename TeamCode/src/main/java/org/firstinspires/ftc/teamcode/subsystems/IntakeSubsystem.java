package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class IntakeSubsystem extends SubsystemBase {
    private final DcMotor motor;
    private final Servo xServo, zServo, gripServo;
    private final CRServo spinServo;

    public IntakeSubsystem(HardwareMap hardwareMap,  MultipleTelemetry telemetry){
        motor = hardwareMap.dcMotor.get("intakeMotor");
        xServo = hardwareMap.servo.get("xAxisServo");
        zServo = hardwareMap.servo.get("zAxisServo");
        gripServo = hardwareMap.servo.get("gripServo");
        spinServo = hardwareMap.crservo.get("spinServo");

    }    // make one button that extends the arm and lowers the claw while opening it
    // make second button that while pressing it it goes to half of height and pushes things
    // away and then lowers one more stage and picks up the sample
    // button 3: checks angle and returns to base position
}
