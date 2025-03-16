package org.firstinspires.ftc.teamcode.auto;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.ParallelDeadlineGroup;
import com.arcrobotics.ftclib.command.ParallelRaceGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commands.DischargeCommands;
import org.firstinspires.ftc.teamcode.commands.IntakeCommands;
import org.firstinspires.ftc.teamcode.commands.LimelightCommands;
import org.firstinspires.ftc.teamcode.commands.MecanumCommands;
import org.firstinspires.ftc.teamcode.subsystems.DischargeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;
import org.opencv.core.Point;

@Autonomous
public class BasketOnly extends CommandOpMode {
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();
    MultipleTelemetry multipleTelemetry = new MultipleTelemetry(telemetry, dashboardTelemetry);
    MecanumDrive mecanumDrive;
    DischargeSubsystem dischargeSubsystem;
    IntakeSubsystem intakeSubsystem;
    LimelightSubsystem limelightSubsystem;

    @Override
    public void initialize() {
        limelightSubsystem = new LimelightSubsystem(hardwareMap, multipleTelemetry);

        dischargeSubsystem = new DischargeSubsystem(hardwareMap, multipleTelemetry);
        intakeSubsystem = new IntakeSubsystem(hardwareMap, multipleTelemetry);
        mecanumDrive = new MecanumDrive(multipleTelemetry, hardwareMap, new Point(0.8, 0.22), 180, this);
        register(mecanumDrive, dischargeSubsystem, intakeSubsystem, limelightSubsystem);
        mecanumDrive.setHeading(0);
//        AutoUtils.initCommands(this, dischargeSubsystem, intakeSubsystem);
        schedule(new DischargeCommands.MotorControl(dischargeSubsystem, () -> 0.0, true, telemetry));
        schedule(new IntakeCommands.ReturnArmForTransferCmd(intakeSubsystem, true));


        while (opModeInInit()) {
            super.run();
        }

        schedule(new SequentialCommandGroup(

                new DischargeCommands.GoToTarget(dischargeSubsystem, dischargeSubsystem.highBasketHeight),
                new WaitCommand(900),
                new ParallelCommandGroup( // discharge
                        new SequentialCommandGroup(
//                                new MecanumCommands.GotoCmd(telemetry, mecanumDrive, 0.31, 0.43, 203, 0.03, 0.8),
                                new MecanumCommands.GotoCmd(telemetry, mecanumDrive, 0.26, 0.32, 203, 0.03, 0.8)//preload discharge
                        ),
                        new IntakeCommands.StartIntakeCmd(intakeSubsystem, true, 2100)),
                new DischargeCommands.DischargeReleaseCmd(dischargeSubsystem),

                intakeAction(0.355, 0.542, 203),

                new ParallelDeadlineGroup( // before discharge
                        new DischargeCommands.SequentialRaceWrapper(
                                new IntakeCommands.Transfer(intakeSubsystem, dischargeSubsystem),
                                new ParallelCommandGroup(
                                        new DischargeCommands.GoToTargetWait(dischargeSubsystem, dischargeSubsystem.highBasketHeight),
                                        new IntakeCommands.StartIntakeCmd(intakeSubsystem, true, 2100))

                        ),
                        new DischargeCommands.SequentialRaceWrapper(
//                                new WaitCommand(400),
                                new MecanumCommands.GotoCmd(telemetry, mecanumDrive, 0.255, 0.5, 189, 0.03, 0.8)//before second discharge
                        )

                ),

                // discharge
                dischargeAction(0.24, 0.355, 189),

                intakeAction(0.26, 0.5, 189),


                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                new IntakeCommands.Transfer(intakeSubsystem, dischargeSubsystem),
//                                new ParallelCommandGroup(
                                new DischargeCommands.GoToTargetWait(dischargeSubsystem, dischargeSubsystem.highBasketHeight)),
//                              new IntakeCommands.StartIntakeCmd(intakeSubsystem, true, 1200))),
                        new SequentialCommandGroup(
                                new WaitCommand(1200),
                                new MecanumCommands.GotoCmd(telemetry, mecanumDrive, 0.3, 0.44, 197, 0.03, 1))
                ),


                dischargeAction(0.31, 0.34, 197),

                new ParallelCommandGroup(
                        new InstantCommand(() -> mecanumDrive.setMoverServo(0.5)),
                        new SequentialCommandGroup(
                                new MecanumCommands.GotoCmd(telemetry, mecanumDrive, 0.23, 0.8, 180, 0.03, 1.2),
                                new MecanumCommands.GotoCmd(telemetry, mecanumDrive, 0.4, 0.8, 180, 0.03, 1.2)
                        )
                        ,
                        new SequentialCommandGroup(
                                new WaitCommand(500),
                                new DischargeCommands.GoHomeCmd(dischargeSubsystem)
                        ),
                        new IntakeCommands.StartIntakeCmd(intakeSubsystem, true, 1800)
                ),
                new MecanumCommands.GotoCmd(telemetry, mecanumDrive, 0.275, 0.645, 180, 0.015, 0.9),
                new WaitCommand(100),
                new IntakeCommands.SampleGroundIntakeCmd(intakeSubsystem),
                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                new IntakeCommands.Transfer(intakeSubsystem, dischargeSubsystem),
                                new DischargeCommands.GoToTargetWait(dischargeSubsystem, dischargeSubsystem.highBasketHeight)
                        ),
                        new SequentialCommandGroup(
                                new WaitCommand(1200),
                                new MecanumCommands.GotoCmd(telemetry, mecanumDrive, 0.4, 0.45, 225, 0.03, 1)
                        )

                ),

                dischargeAction(0.34, 0.285, 225),

                new ParallelCommandGroup(
                        new InstantCommand(() -> mecanumDrive.setMoverServo(0)),
                        new SequentialCommandGroup(
                                new MecanumCommands.ConstantVelocityGoPastCmd(telemetry, mecanumDrive, 0.8, 1.6, 270, 0.01, 1, 0.65, 0.2),
                                new MecanumCommands.ConstantVelocityGotoCmd(telemetry, mecanumDrive, 4, 1.6, 270, 0, 1).withTimeout(750)),
                        new SequentialCommandGroup(
                                new WaitCommand(1500),
                                new DischargeCommands.GoHomeCmd(dischargeSubsystem)
                        )

                ),
                new LimelightCommands.LimelightStartIntake(limelightSubsystem, intakeSubsystem, dischargeSubsystem, mecanumDrive),

                new ParallelCommandGroup( // discharge
                        new SequentialCommandGroup(
                                new WaitCommand(500),
                                new MecanumCommands.GotoCmd(telemetry, mecanumDrive, 0.26, 0.32, 203, 0.03, 0.8)//preload discharge
                        ),
                        new SequentialCommandGroup(
                                new IntakeCommands.Transfer(intakeSubsystem, dischargeSubsystem),
                                new DischargeCommands.GoToTarget(dischargeSubsystem, dischargeSubsystem.highBasketHeight)
                        )),
                new DischargeCommands.DischargeReleaseCmd(dischargeSubsystem),

                new ParallelCommandGroup(
                        new InstantCommand(() -> mecanumDrive.setMoverServo(0)),
                        new SequentialCommandGroup(
                                new MecanumCommands.ConstantVelocityGoPastCmd(telemetry, mecanumDrive, 0.8, 1.6, 270, 0.01, 1, 0.65, 0.2),
                                new MecanumCommands.ConstantVelocityGotoCmd(telemetry, mecanumDrive, 4, 1.6, 270, 0, 1).withTimeout(750)),
                        new SequentialCommandGroup(
                                new WaitCommand(1500),
                                new DischargeCommands.GoHomeCmd(dischargeSubsystem)
                        )

                )


//                new ParallelCommandGroup(
//                    new SequentialCommandGroup(
//                                new WaitCommand(800),
//                                new DischargeCommands.GoHomeCmd(dischargeSubsystem)),
//                    new SequentialCommandGroup(
//                        new MecanumCommands.GotoCmd(telemetry, mecanumDrive, 0.6, 0.285, 270, 0.1,1),
//                        new MecanumCommands.GotoCmd(telemetry,mecanumDrive,0.6,1.5,270,0.1,0.6))
//                ),
//                new MecanumCommands.GotoCmd(telemetry,mecanumDrive,1.21,1.5,270,0.03,0.7),
//                new LimelightCommands.LimelightIntake(limelightSubsystem, intakeSubsystem, dischargeSubsystem, mecanumDrive),
//                new ParallelCommandGroup(
//                    new MecanumCommands.GotoCmd(telemetry, mecanumDrive, 0.6, 1.5, 270, 0.2,1),
//                    new DischargeCommands.GoToTargetWait(dischargeSubsystem, dischargeSubsystem.highBasketHeight)),
//                new MecanumCommands.GotoCmd(telemetry, mecanumDrive, 0.34, 0.285, 225, 0.1, 1),
//                new DischargeCommands.DischargeReleaseCmd(dischargeSubsystem)
        ));


    }


    private CommandBase dischargeAction(double x, double y, double angle) {
        return new SequentialCommandGroup(
                //new DischargeCommands.DischargeHalfReleaseCmd(dischargeSubsystem),
                new MecanumCommands.GotoCmd(telemetry, mecanumDrive, x, y, angle, 0.045, 1.6),//second discharge pos
                new DischargeCommands.DischargeReleaseCmd(dischargeSubsystem),
                new MecanumCommands.GotoCmd(telemetry, mecanumDrive, x, y, angle, 0.025, 1.6)//second discharge pos
        );
    }

    private CommandBase intakeAction(double x, double y, double angle) {
        return new SequentialCommandGroup(
                new ParallelDeadlineGroup( // intake
                        new DischargeCommands.SequentialRaceWrapper(
                                new MecanumCommands.GotoCmd(telemetry, mecanumDrive, x, y, angle, 0.0175, 1.55),
//                                new WaitCommand(100),
                                new IntakeCommands.SampleSubmIntakeCmd(intakeSubsystem, angle / 180.0 - 1)),
                        new DischargeCommands.SequentialRaceWrapper(
                                new WaitCommand(1000),
                                new DischargeCommands.GoHomeCmd(dischargeSubsystem)))
        );
    }

    @Override
    public void run() {
        AutoUtils.savePosition(mecanumDrive);
        super.run();
        multipleTelemetry.addData("x", mecanumDrive.getPosition().x);
        multipleTelemetry.addData("y", mecanumDrive.getPosition().y);
        multipleTelemetry.addData("angle", mecanumDrive.getHeading());
        multipleTelemetry.addData("angle adjusted", mecanumDrive.getAdjustedHeading());
        // multipleTelemetry.addData("lx", limelightSubsystem.getXDistance());
        // multipleTelemetry.addData("yx", limelightSubsystem.getYDistance());

        //multipleTelemetry.addData("currentIntake", intakeSubsystem.getCurrent());
        //multipleTelemetry.addData("isTouching", dischargeSubsystem.isHome());
        //multipleTelemetry.addData("discharge current command", dischargeSubsystem.getCurrentCommand().getName());
        //multipleTelemetry.addData("intake current command", intakeSubsystem.getCurrentCommand().getName());
        //multipleTelemetry.addData("current", dischargeSubsystem.getCurrent());
    }
}
