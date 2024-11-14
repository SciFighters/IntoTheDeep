

package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commands.SwerveCommands;
import org.firstinspires.ftc.teamcode.subsystems.SwerveDrive;


@TeleOp(group = "swerve")
public class BasicSwerveOpMode extends CommandOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private SwerveDrive swerveDrive;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();
    MultipleTelemetry multipleTelemetry = new MultipleTelemetry(telemetry, dashboardTelemetry);
    GamepadEx driverGamepad;

    @Override
    public void initialize() {
        driverGamepad = new GamepadEx(gamepad1);
        swerveDrive = new SwerveDrive(hardwareMap, multipleTelemetry);
        register(swerveDrive);

//        CommandScheduler.getInstance().setDefaultCommand(swerveDrive,new SwerveCommands.PowerCmd(telemetry, swerveDrive, driverGamepad::getLeftX, driverGamepad::getLeftY, driverGamepad::getRightX, () -> driverGamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)));
        swerveDrive.setDefaultCommand(new SwerveCommands.PowerCmd(telemetry, swerveDrive,
                driverGamepad::getLeftX, driverGamepad::getLeftY, driverGamepad::getRightX,
                () -> driverGamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)));

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void run() {
        super.run();
        telemetry.addData("e", "e");
        telemetry.update();
//
//        if (gamepad1.dpad_up) {
//            swerveDrive.kp += 0.001;
//        }
//        if (gamepad1.dpad_down) {
//            swerveDrive.kp -= 0.001;
//        }
        telemetry.addData("ly: ", driverGamepad.getLeftX());
        telemetry.addData("rx: ", driverGamepad.getLeftY());
        telemetry.addData("lx: ", driverGamepad.getRightX());
        telemetry.addData("heading", swerveDrive.getHeading());
        telemetry.update();
    }
}
