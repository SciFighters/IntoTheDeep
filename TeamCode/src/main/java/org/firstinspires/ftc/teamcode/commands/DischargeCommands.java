package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.DischargeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SampleSubsystem;

import java.util.function.Supplier;

public class DischargeCommands {

    public static class DischargePowerCmd extends CommandBase {
        Supplier<Double> upPower;
        DischargeSubsystem dischargeSubsystem;

        public DischargePowerCmd(Supplier<Double> upPower, DischargeSubsystem dischargeSubsystem) {
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
//            dischargeSubsystem.setPower((upPower.get()));
        }
    }

    public static class DischargeGotoCmd extends CommandBase {
        DischargeSubsystem dischargeSubsystem;
        private final int pos, sensitivity;
        Telemetry telemetry;
        public DischargeGotoCmd(DischargeSubsystem dischargeSubsystem, int pos, int sensitivity, Telemetry telemetry) {
            this.dischargeSubsystem = dischargeSubsystem;
            this.pos = pos;
            this.sensitivity = sensitivity;
            addRequirements(dischargeSubsystem);
            this.telemetry = telemetry;
        }

        @Override
        public void initialize() {
            telemetry.addData("onCommand","yes");
            telemetry.update();
            dischargeSubsystem.setPosition(pos);
//            dischargeSubsystem.setPower(0.5);
        }

        @Override
        public boolean isFinished() {
            return Math.abs(dischargeSubsystem.getPosition() - pos) <= sensitivity;
        }
    }

    public static class DischargeHoldCmd extends CommandBase {
        DischargeSubsystem dischargeSubsystem;

        public DischargeHoldCmd(DischargeSubsystem dischargeSubsystem) {
            this.dischargeSubsystem = dischargeSubsystem;
            addRequirements(dischargeSubsystem);
        }

        @Override
        public void initialize() {
//            dischargeSubsystem.holdSample();
        }

//        @Override
//        public boolean isFinished() {
//            return dischargeSubsystem.getClawServoPos() < 0.05;
//        }
    }

    public static class DischargeReleaseCmd extends CommandBase {
        DischargeSubsystem dischargeSubsystem;

        public DischargeReleaseCmd(DischargeSubsystem dischargeSubsystem) {
            this.dischargeSubsystem = dischargeSubsystem;
            addRequirements(dischargeSubsystem);
        }

//        @Override
//        public void initialize() {
//            dischargeSubsystem.releaseSample();
//        }

//        @Override
//        public boolean isFinished() {
//            return dischargeSubsystem.getClawServoPos() > 0.45;
//        }
    }

    public static class GearBoxSwapCmd extends CommandBase {
        DischargeSubsystem dischargeSubsystem;

        public GearBoxSwapCmd(DischargeSubsystem dischargeSubsystem) {
            this.dischargeSubsystem = dischargeSubsystem;
            addRequirements(dischargeSubsystem);
        }

        @Override
        public void initialize() {
            if (dischargeSubsystem.getGearBoxRatio() == 1) {
                dischargeSubsystem.climbMode();
            } else {
                dischargeSubsystem.dischargeMode();
            }
        }

        @Override
        public boolean isFinished() {
            return true;
        }
    }

    public static class HighBasketDischargeCmd extends SequentialCommandGroup {
//        public HighBasketDischargeCmd(DischargeSubsystem dischargeSubsystem) {
//            addCommands(new DischargeGotoCmd(dischargeSubsystem, 600, 5),
//                    new DischargeReleaseCmd(dischargeSubsystem),
//                    new DischargeGotoCmd(dischargeSubsystem, 0, 5));
//            addRequirements(dischargeSubsystem);
//        }
    }

}
