package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsystems.SwerveDrive;

public class DriveSubsystem extends SubsystemBase {

    SwerveDrive driveHandler;

    public DriveSubsystem(HardwareMap hardwareMap, MultipleTelemetry telemetry, LinearOpMode opMode) {
        driveHandler = new SwerveDrive(hardwareMap, telemetry, opMode);
    }

    public void drive(double x, double y, double rotation, double _boost) {
        driveHandler.drive(x, y, rotation, _boost);
    }

    public double getHeading() {
        return driveHandler.getHeading();
    }

    public double[] getPosition(){
        return driveHandler.getPosition();
    }

    @Override
    public void periodic() {
        driveHandler.localizer.update();
    }
}
