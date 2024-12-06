package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commands.DischargeCommands;
import org.firstinspires.ftc.teamcode.subsystems.DischargeSubsystem;
import org.firstinspires.ftc.teamcode.commands.DischargeCommands.DischargePowerCmd;
import org.firstinspires.ftc.teamcode.commands.DischargeCommands.GearBoxSwapCmd;
@TeleOp
public class BasicDischargeTest extends CommandOpMode {
    GamepadEx systemGamepad;
    private DischargeSubsystem dischargeSubsystem;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();
    MultipleTelemetry multipleTelemetry = new MultipleTelemetry(telemetry, dashboardTelemetry);
    private DischargePowerCmd dischargePowerCmd;

    @Override
    public void initialize() {

        systemGamepad = new GamepadEx(gamepad2);

        dischargeSubsystem = new DischargeSubsystem(hardwareMap, multipleTelemetry);
        register(dischargeSubsystem);
        dischargePowerCmd = new DischargePowerCmd(() -> systemGamepad.getLeftY(), dischargeSubsystem);
        dischargeSubsystem.setDefaultCommand(dischargePowerCmd);
        Button a = new GamepadButton(systemGamepad,GamepadKeys.Button.A);
        a.whenPressed(new GearBoxSwapCmd(dischargeSubsystem));
    }

}
