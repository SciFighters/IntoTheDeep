package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.SwerveDrive;

import java.util.function.Supplier;

public class SwerveCommands {
    public static class PowerCmd extends CommandBase {
        Supplier<Double> x;
        Supplier<Double> y;
        Supplier<Double> r;
        Supplier<Double> boost;
        SwerveDrive swerveDrive;
        Telemetry telemetry;

        public PowerCmd(Telemetry telemetry, SwerveDrive swerveDrive, Supplier<Double> x, Supplier<Double> y, Supplier<Double> r, Supplier<Double> boost) {
            this.x = x;
            this.y = y;
            this.r = r;
            this.boost = boost;
            this.swerveDrive = swerveDrive;
            this.telemetry = telemetry;

            addRequirements(swerveDrive);
        }

        @Override
        public void execute() {
            telemetry.addData("X", x.get());
            telemetry.addData("Y", y.get());
            telemetry.addData("TURN", r.get());
            swerveDrive.drive(x.get(), y.get(), r.get(), boost.get()/2);
        }
    }
}
