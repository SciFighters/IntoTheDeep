package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.DischargeSubsystem;

import java.util.function.Supplier;

public class DischargeCommands{

    public static class DischargePowerCmd extends CommandBase{
        Supplier<Double> upPower;
        DischargeSubsystem dischargeSubsystem;
        public DischargePowerCmd(Supplier<Double> upPower, DischargeSubsystem dischargeSubsystem){
            this.upPower = upPower;
            this.dischargeSubsystem = dischargeSubsystem;
            addRequirements(dischargeSubsystem);
        }

        @Override
        public void initialize() {
            dischargeSubsystem.resetEncoders();
        }

        @Override
        public void execute() {
            dischargeSubsystem.setPower((upPower.get()));
        }
    }
    public static class GearBoxSwapCmd extends CommandBase{
        DischargeSubsystem dischargeSubsystem;
        public GearBoxSwapCmd(DischargeSubsystem dischargeSubsystem){
            this.dischargeSubsystem = dischargeSubsystem;
            addRequirements(dischargeSubsystem);
        }

        @Override
        public void initialize() {
            if(dischargeSubsystem.getGearBoxRatio() == 1){
                dischargeSubsystem.climbMode();
            } else{
                dischargeSubsystem.dischargeMode();
            }
        }

        @Override
        public boolean isFinished() {
            return true;
        }
    }
}
