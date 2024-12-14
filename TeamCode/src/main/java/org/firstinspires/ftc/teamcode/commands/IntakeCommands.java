package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.DischargeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.GripStages;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.XRotationStages;

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
        public void initialize() {
            subsystem.armGoToPos(position);
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

    public static class SetXRotationCmd extends CommandBase {
        IntakeSubsystem subsystem;
        XRotationStages stage;

        public SetXRotationCmd(IntakeSubsystem subsystem, XRotationStages stage) {
            this.stage = stage;
            this.subsystem = subsystem;
            addRequirements(subsystem);
        }

        @Override
        public void initialize() {
            subsystem.setXServoPosition(stage.POSITION);
        }

        @Override
        public boolean isFinished() {
            return Math.abs(subsystem.getXServoPosition() - stage.POSITION) <= 0.02;
        }
    }

    public static class SetGripStageCmd extends CommandBase {
        IntakeSubsystem subsystem;
        GripStages gripStage;

        public SetGripStageCmd(IntakeSubsystem subsystem, GripStages gripStage) {
            this.subsystem = subsystem;
            this.gripStage = gripStage;
//            addRequirements(subsystem);
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
    public static class StartIntake extends SequentialCommandGroup{
        private final int pos = 500;
        public StartIntake(IntakeSubsystem subsystem){
            addCommands(new IntakeGotoCmd(subsystem,pos),
                    new SetXRotationCmd(subsystem,XRotationStages.MIDDLE),
                    new ParallelCommandGroup(new SetGripStageCmd(subsystem, GripStages.MIDDLE),
                            new SetXRotationCmd(subsystem,XRotationStages.LOWER)));
            addRequirements(subsystem);
        }

    }
}
