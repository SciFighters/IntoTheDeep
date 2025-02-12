package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.subsystems.DischargeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Pipelines;

public class LimelightCommands {
    public static class AlignXCmd extends CommandBase {
        LimelightSubsystem limelight;
        MecanumDrive mecanumDrive;
        final double kp = 0.01;
        double currentPipeline;

        public AlignXCmd(LimelightSubsystem limelight, MecanumDrive mecanumDrive) {
            this.limelight = limelight;
            this.mecanumDrive = mecanumDrive;
            addRequirements(limelight,mecanumDrive);

        }

        @Override
        public void initialize() {
            limelight.startLimelight();
        }

        @Override
        public void execute() {
            mecanumDrive.drive(limelight.getXDistance() * kp, 0, 0, 0.2);
        }

        @Override
        public boolean isFinished() {
            return limelight.getXDistance() <= 1;
        }

        @Override
        public void end(boolean interrupted) {
            limelight.alignedY = limelight.getYDistance();
            limelight.alignedAngle = limelight.getAngle();
            limelight.stopLimelight();

        }
    }

    public static class LimelightIntake extends SequentialCommandGroup {
        LimelightSubsystem limelightSubsystem;
        IntakeSubsystem intakeSubsystem;
        DischargeSubsystem dischargeSubsystem;

        public LimelightIntake(LimelightSubsystem limelightSubsystem, IntakeSubsystem intakeSubsystem, DischargeSubsystem dischargeSubsystem, MecanumDrive mecanumDrive) {
            this.intakeSubsystem = intakeSubsystem;
            this.limelightSubsystem = limelightSubsystem;
            addCommands(new AlignXCmd(limelightSubsystem, mecanumDrive),
                    new IntakeCommands.StartIntakeCmd(intakeSubsystem, true, (int) limelightSubsystem.alignedY),//todo: make in ticks
                    new IntakeCommands.SetRotationCmd(intakeSubsystem, limelightSubsystem.alignedAngle),//todo: convert to servo(0 - 1)
                    new WaitCommand(500),
                    new IntakeCommands.SampleIntakeCmd(intakeSubsystem),//todo: not related but make the intake max bigger
                    new IntakeCommands.Transfer(intakeSubsystem, dischargeSubsystem));
        }

        @Override
        public void initialize() {
            limelightSubsystem.startLimelight();
            super.initialize();
        }
    }

}
