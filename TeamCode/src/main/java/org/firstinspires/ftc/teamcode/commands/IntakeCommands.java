package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.GripStages;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

import java.util.function.Supplier;

public class IntakeCommands {
    public static class IntakeArmCmd extends CommandBase {
        IntakeSubsystem subsystem;
        final int position = 314;
        final double kp = 0.01;

        public IntakeArmCmd(IntakeSubsystem subsystem) {
            this.subsystem = subsystem;
            addRequirements(subsystem);
        }

        @Override
        public void execute() {
            subsystem.setArmPower((position - subsystem.getMotorPosition()) * kp);
        }

        @Override
        public boolean isFinished() {
            if (Math.abs(position - subsystem.getMotorPosition()) <= 5) {
                return true;
            }
            return false;
        }

        @Override
        public void end(boolean interrupted) {
            subsystem.setArmPower(0);
        }
    }

    public static class LoweredXRotationCmd extends CommandBase {
        IntakeSubsystem subsystem;

        public LoweredXRotationCmd(IntakeSubsystem subsystem) {
            this.subsystem = subsystem;
            addRequirements(subsystem);
        }

        @Override
        public void initialize() {
            subsystem.setXServoPosition(1);
        }

        @Override
        public boolean isFinished() {
            if (subsystem.getXServoPosition() >= 0.99) {
                return true;
            }
            return false;
        }


    }

    public static class SetGripStageCmd extends CommandBase {
        IntakeSubsystem subsystem;
        GripStages gripStage;

        public SetGripStageCmd(GripStages gripStage, IntakeSubsystem subsystem) {
            this.subsystem = subsystem;
            this.gripStage = gripStage;
            addRequirements(subsystem);
        }

        @Override
        public void initialize() {
            subsystem.setGripStage(gripStage);
        }

        @Override
        public boolean isFinished() {
            if (Math.abs(subsystem.getGripServoPosition() - gripStage.POSITION) < 0.02){
                return true;
            }
            return false;
        }
    }
}
