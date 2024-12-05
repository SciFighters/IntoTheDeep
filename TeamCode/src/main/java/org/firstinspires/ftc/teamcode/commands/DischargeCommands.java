package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.DischargeSubsystem;

import java.util.function.Supplier;

public class DischargeCommands{

    public static class DischargePowerCmd extends CommandBase{
        Supplier<Double> upPower, downPower;
        DischargeSubsystem dischargeSubsystem;
//        , Supplier<Double> downPower
        public DischargePowerCmd(Supplier<Double> upPower, DischargeSubsystem dischargeSubsystem){
            this.upPower = upPower;
            this.downPower = downPower;
            this.dischargeSubsystem = dischargeSubsystem;
            addRequirements(dischargeSubsystem);
        }

        @Override
        public void execute() {
            dischargeSubsystem.setPower((upPower.get()-downPower.get())/5);
        }
    }

}
