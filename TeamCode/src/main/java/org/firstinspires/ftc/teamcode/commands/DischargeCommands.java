package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandOpMode;

import org.firstinspires.ftc.teamcode.subsystems.DischargeSubsystem;

import java.util.function.Supplier;

public class DischargeCommands{

    public static class dischargeBaseCmd extends CommandBase{
        Supplier<Double> upPower, downPower;
        DischargeSubsystem dischargeSubsystem;
        public dischargeBaseCmd(Supplier<Double> upPower,Supplier<Double> downPower,DischargeSubsystem dischargeSubsystem){
            this.upPower = upPower;
            this.downPower = downPower;
            this.dischargeSubsystem = dischargeSubsystem;
            addRequirements(dischargeSubsystem);
        }
    }

}
