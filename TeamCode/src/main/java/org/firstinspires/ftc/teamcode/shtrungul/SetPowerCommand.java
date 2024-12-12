package org.firstinspires.ftc.teamcode.shtrungul;

import com.arcrobotics.ftclib.command.CommandBase;

import java.util.function.Supplier;

public class SetPowerCommand extends CommandBase {
    SampleSubsystem sub;
    double _powerSupplier;

    public SetPowerCommand(SampleSubsystem subsystem, double powerSupplier) {
        this.sub = subsystem;
        this._powerSupplier = powerSupplier;
        addRequirements(sub);
    }

    @Override
    public void execute() {
        this.sub.setPower(_powerSupplier);
    }

    @Override
    public void end(boolean interrupted) {
        this.sub.setPower(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
