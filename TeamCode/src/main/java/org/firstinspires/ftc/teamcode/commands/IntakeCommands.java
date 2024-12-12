package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.GripStages;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

public class IntakeCommands {
    public static class IntakeGotoCmd extends CommandBase {
        IntakeSubsystem subsystem;
        final int position;
        final int maxPosition = 400;
        final double kp = 0.01;

        public IntakeGotoCmd(IntakeSubsystem subsystem, int position) {
            this.subsystem = subsystem;
            if (position > maxPosition) {
                position = maxPosition;
            } else if (position < 0) {
                position = 0;
            }
            this.position = position;
            addRequirements(subsystem);
        }

        @Override
        public void execute() {
            subsystem.setArmPower((position - subsystem.getMotorPosition()) * kp);
        }

        @Override
        public boolean isFinished() {
            return Math.abs(position - subsystem.getMotorPosition()) <= 5;
        }

        @Override
        public void end(boolean interrupted) {
            subsystem.setArmPower(0);
        }
    }


    public static class LowerXRotationCmd extends CommandBase {
        IntakeSubsystem subsystem;

        public LowerXRotationCmd(IntakeSubsystem subsystem) {
            this.subsystem = subsystem;
            addRequirements(subsystem);
        }

        @Override
        public void initialize() {
            subsystem.setXServoPosition(1);
        }

        @Override
        public boolean isFinished() {
            return subsystem.getXServoPosition() >= 0.98;
        }


    }

    public static class UpperXRotationCmd extends CommandBase {
        IntakeSubsystem subsystem;

        public UpperXRotationCmd(IntakeSubsystem subsystem) {
            this.subsystem = subsystem;
            addRequirements(subsystem);
        }

        @Override
        public void initialize() {
            subsystem.setXServoPosition(0);
        }

        @Override
        public boolean isFinished() {
            return subsystem.getXServoPosition() <= 0.02;
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
            return Math.abs(subsystem.getGripServoPosition() - gripStage.POSITION) < 0.02;
        }
    }

    public static class SetZRotationCmd extends CommandBase {
        IntakeSubsystem intakeSubsystem;
        double position;

        public SetZRotationCmd(IntakeSubsystem intakeSubsystem, double position) {
            this.intakeSubsystem = intakeSubsystem;
            this.position = position;
            addRequirements(intakeSubsystem);
        }

        @Override
        public void initialize() {
            intakeSubsystem.setZServoPosition(position);
        }

        @Override
        public boolean isFinished() {
            return Math.abs(intakeSubsystem.getZServoPosition() - position) < -0.02;
        }
    }

    public static class SpinOutCmd extends CommandBase {
        IntakeSubsystem intakeSubsystem;
        ElapsedTime runtime;
        double startTime;

        public SpinOutCmd(IntakeSubsystem intakeSubsystem, ElapsedTime runtime) {
            this.intakeSubsystem = intakeSubsystem;
            addRequirements(intakeSubsystem);
        }

        @Override
        public void initialize() {
            intakeSubsystem.setSpinPower(-0.5);
            startTime = runtime.seconds();
        }

        @Override
        public boolean isFinished() {
            return (runtime.seconds() - startTime) > 0.5;
        }

        @Override
        public void end(boolean interrupted) {
            intakeSubsystem.setSpinPower(0);
        }
    }

    public static class SpinInCmd extends CommandBase {
        IntakeSubsystem intakeSubsystem;
        ElapsedTime runtime;
        double startTime;

        public SpinInCmd(IntakeSubsystem intakeSubsystem, ElapsedTime runtime) {
            this.intakeSubsystem = intakeSubsystem;
            addRequirements(intakeSubsystem);
        }

        @Override
        public void initialize() {
            intakeSubsystem.setSpinPower(0.5);
            startTime = runtime.seconds();
        }

        @Override
        public boolean isFinished() {
            return (runtime.seconds() - startTime) > 0.5;
        }

        @Override
        public void end(boolean interrupted) {
            intakeSubsystem.setSpinPower(0);
        }
    }
//    public static class ResetIntakeCmd extends SequentialCommandGroup{
//        public ResetIntakeCmd(IntakeSubsystem intakeSubsystem){
//            addCommands(new ParallelCommandGroup(new SetZRotationCmd(intakeSubsystem)));
//        }
//    }
}
