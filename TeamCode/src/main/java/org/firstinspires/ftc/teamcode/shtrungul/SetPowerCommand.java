package org.firstinspires.ftc.teamcode.shtrungul;

import com.arcrobotics.ftclib.command.CommandBase;

import java.util.function.Supplier;

public class SetPowerCommand extends CommandBase {
    SampleSubsystem sub;
    Supplier<Double> _powerSupplier;

    public SetPowerCommand(SampleSubsystem subsystem, Supplier<Double> powerSupplier) {
        this.sub = subsystem;
        this._powerSupplier = powerSupplier;
        addRequirements(sub);
    }

    @Override
    public void execute() {
        this.sub.setPower(_powerSupplier.get());
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
