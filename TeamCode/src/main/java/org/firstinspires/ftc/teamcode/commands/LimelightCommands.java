package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.ArmsStages;
import org.firstinspires.ftc.teamcode.subsystems.ClawStages;
import org.firstinspires.ftc.teamcode.subsystems.DischargeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Pipelines;

public class LimelightCommands {
    @Config
    public static class AlignXCmd extends CommandBase {
        LimelightSubsystem limelight;
        MecanumDrive mecanumDrive;
        public static double kp = 0.0008;
        public static double minPower = 0.09;
        public static double ki = 0;
        public static double kd = -0.00005;
        double currentPipeline;
        double error, lastError;
        ElapsedTime elapsedTime = new ElapsedTime();
        double time, lastTime;
        double Integral = 0;
        public static double derivative;

        public AlignXCmd(LimelightSubsystem limelight, MecanumDrive mecanumDrive) {
            this.limelight = limelight;
            this.mecanumDrive = mecanumDrive;
            addRequirements(mecanumDrive, limelight);
        }

        @Override
        public void initialize() {
            Integral = 0;
            limelight.startLimelight();
            limelight.updateResults();
            mecanumDrive.setFieldOriented(false);
            error = limelight.getXDistance();
            lastError = error;
            lastTime = elapsedTime.seconds();
        }

        @Override
        public void execute() {

            time = elapsedTime.seconds();
            error = limelight.getXDistance();
            double deltaTime = time - lastTime;
            if (deltaTime > 0.01) {
                double proportional = error * kp;
                Integral += error * deltaTime * ki;
                derivative = (lastError - error) * kd / deltaTime;
                double feedForward = Math.signum(error) * minPower;
                mecanumDrive.drive(proportional + Integral + derivative + feedForward, 0, 0, 0.5);
            }

            lastTime = time;
            lastError = error;
        }

        @Override
        public boolean isFinished() {
            return Math.abs(limelight.getXDistance()) <= 5;
        }

        @Override
        public void end(boolean interrupted) {
            mecanumDrive.setFieldOriented(true);
            mecanumDrive.drive(0, 0, 0, 0);
            limelight.alignedY = limelight.getYDistance();
            limelight.stopLimelight();
        }
    }

    public static class LimelightCompleteSubIntake extends SequentialCommandGroup {
        LimelightSubsystem limelightSubsystem;
        IntakeSubsystem intakeSubsystem;
        DischargeSubsystem dischargeSubsystem;
        MecanumDrive mecanumDrive;
        double wantedAngle;
        boolean outOfRange = false;

        public LimelightCompleteSubIntake(LimelightSubsystem limelightSubsystem, IntakeSubsystem intakeSubsystem, DischargeSubsystem dischargeSubsystem, MecanumDrive mecanumDrive) {
            this.intakeSubsystem = intakeSubsystem;
            this.limelightSubsystem = limelightSubsystem;
            this.dischargeSubsystem = dischargeSubsystem;
            this.mecanumDrive = mecanumDrive;
            addCommands(
                    new LimelightStartIntake(limelightSubsystem, intakeSubsystem, dischargeSubsystem, mecanumDrive),
                    new IntakeCommands.Transfer(intakeSubsystem, dischargeSubsystem));
            addRequirements(limelightSubsystem, intakeSubsystem, dischargeSubsystem, mecanumDrive);
        }


        @Override
        public void initialize() {
            super.initialize();
            limelightSubsystem.startLimelight();
            wantedAngle = limelightSubsystem.getAngle();
        }

        @Override
        public void end(boolean interrupted) {
            limelightSubsystem.stopLimelight();
        }
    }

    public static class LimelightStartIntake extends SequentialCommandGroup {
        LimelightSubsystem limelightSubsystem;
        IntakeSubsystem intakeSubsystem;
        DischargeSubsystem dischargeSubsystem;
        MecanumDrive mecanumDrive;
        double wantedAngle;
        boolean outOfRange = false;
        double angle;

        public LimelightStartIntake(LimelightSubsystem limelightSubsystem, IntakeSubsystem intakeSubsystem, DischargeSubsystem dischargeSubsystem, MecanumDrive mecanumDrive) {
            this.intakeSubsystem = intakeSubsystem;
            this.limelightSubsystem = limelightSubsystem;
            this.dischargeSubsystem = dischargeSubsystem;
            this.mecanumDrive = mecanumDrive;
            addCommands(
//                    new InstantCommand(() -> {
//                        while (limelightSubsystem.getYDistance() >= 1700) {
//                            addCommands(new InstantCommand(() -> mecanumDrive.drive(-0.3, 0, 0, 0.2)).withTimeout(300));
//                        }
//                    }),
                    new AlignXCmd(limelightSubsystem, mecanumDrive)/*.withTimeout(1000)*/,
                    new AlignXCmd(limelightSubsystem, mecanumDrive).withTimeout(250),
                    new WaitCommand(100),
                    new InstantCommand(() -> {
                        double position = (limelightSubsystem.getAngle() > 0) ? limelightSubsystem.getAngle() : 180 + limelightSubsystem.getAngle();
                        angle = ((position / 180 - 0.5) * 2 / 3 + 0.5);
                        if (angle > 0.75) {
                            angle = 0;
                        }
                    }),
                    new IntakeCommands.StartIntakeCmd(intakeSubsystem, limelightSubsystem::getYDistance, angle > 0.5),
                    new WaitCommand(1000),
//                    new IntakeCommands.SetRotationCmd(intakeSubsystem, limelightSubsystem::getAngle),
//                    new WaitCommand(300),
//                    new IntakeCommands.OpenScrewCmd(intakeSubsystem, true),
                    new IntakeCommands.SampleSubmIntakeCmd(intakeSubsystem, limelightSubsystem::getAngle),


                    new InstantCommand(new Runnable() {
                        @Override
                        public void run() {
                            intakeSubsystem.setArmPower(0);
                            intakeSubsystem.runWithoutEncoders();
                        }
                    }));

            addRequirements(limelightSubsystem, intakeSubsystem, dischargeSubsystem, mecanumDrive);
        }


        @Override
        public void initialize() {
            super.initialize();
            limelightSubsystem.startLimelight();
            wantedAngle = limelightSubsystem.getAngle();
        }

        @Override
        public void end(boolean interrupted) {
            limelightSubsystem.stopLimelight();
        }
    }

}
