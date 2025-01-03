

package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commands.SwerveCommands;
import org.firstinspires.ftc.teamcode.subsystems.SwerveDrive;
import org.firstinspires.ftc.teamcode.utils.Utils;

import java.util.Random;


@TeleOp(group = "swerve")
public class BasicSwerveOpMode extends CommandOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private SwerveDrive swerveDrive;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();
    MultipleTelemetry multipleTelemetry = new MultipleTelemetry(telemetry, dashboardTelemetry);
    GamepadEx driverGamepad;
    //public static double kp = 0.0068;
    //public static double ki = 0.00000072; // 0.00000052;
    //public static double kd = 0.068;
    public static double kp = 0.0068;
    public static double ki = 0.0; // 0.00000052;
    public static double kd = 0.0;
    public static double minPower = 0.055;
    private double kChange = 1.02;
    final int[] targetChanges = {0, 0, 45, 0, 0, 20, 20, -50, -50, -50, 0, 0, 0, 30, 30, 30};
    int count = 0;
    double targetAngle;
    long lastTargetChangeTime;
    final double TARGET_INTERVAL = 650;
    Random random = new Random();

    @Override
    public void initialize() {
        driverGamepad = new GamepadEx(gamepad1);
        swerveDrive = new SwerveDrive(hardwareMap, multipleTelemetry, this, true);
        register(swerveDrive);

//        CommandScheduler.getInstance().setDefaultCommand(swerveDrive,new SwerveCommands.PowerCmd(telemetry, swerveDrive, driverGamepad::getLeftX, driverGamepad::getLeftY, driverGamepad::getRightX, () -> driverGamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)));
        swerveDrive.setDefaultCommand(new SwerveCommands.PowerCmd(telemetry, swerveDrive,
                driverGamepad::getLeftX, driverGamepad::getLeftY, driverGamepad::getRightX,
                () -> driverGamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER), true));

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        runtime.reset();
    }

    @Override
    public void run() {
        super.run();
        double[] targetAngles = swerveDrive.getTargetAngles();
        double[] currentAngles = swerveDrive.getCurrentAngles();
        dashboardTelemetry.addData("wanted fl", targetAngles[0]);
        dashboardTelemetry.addData("wanted fr", targetAngles[1]);
        dashboardTelemetry.addData("wanted br", targetAngles[2]);
        dashboardTelemetry.addData("wanted bl", targetAngles[3]);
        dashboardTelemetry.addData("current fl", currentAngles[0]);
        dashboardTelemetry.addData("current fr", currentAngles[1]);
        dashboardTelemetry.addData("current br", currentAngles[2]);
        dashboardTelemetry.addData("current bl", currentAngles[3]);
        dashboardTelemetry.addData("difference fl",Utils.calcDeltaAngle(targetAngles[0],currentAngles[0]));
        dashboardTelemetry.addData("difference fr",Utils.calcDeltaAngle(targetAngles[1],currentAngles[1]));
        dashboardTelemetry.addData("difference br",Utils.calcDeltaAngle(targetAngles[2],currentAngles[2]));
        dashboardTelemetry.addData("difference bl",Utils.calcDeltaAngle(targetAngles[3],currentAngles[3]));

        dashboardTelemetry.update();

//        long currentTime = System.currentTimeMillis();
//        //setHeading
//        if (currentTime - lastTargetChangeTime > TARGET_INTERVAL) {
//            targetAngle += targetChanges[count%targetChanges.length];//random.nextInt(40) -20;
//            count++;
//
//            targetAngle %= 360; // Keep within 0-360
//            lastTargetChangeTime = currentTime;
//        }
//        swerveDrive.fl.servo.setTargetAngle(targetAngle);
//
//        if (gamepad1.dpad_up) {
//            if (gamepad1.x) kp *= kChange;
//            else if (gamepad1.a) ki *= kChange;
//            else if (gamepad1.b) kd *= kChange;
//            else if (gamepad1.y) minPower *= kChange;
//        } else if (gamepad1.dpad_down) {
//            if (gamepad1.x) kp /= kChange;
//            else if (gamepad1.a) ki /= kChange;
//            else if (gamepad1.b) kd /= kChange;
//            else if (gamepad1.y) minPower /= kChange;
//        }
//
//
//        super.run();
//        telemetry.addData("e", "e");
//        telemetry.update();
//
//        telemetry.addData("kp", kp);
//        telemetry.addData("ki", ki * 10000);
//        telemetry.addData("kd", kd);
//        telemetry.addData("minPower", minPower);
//
//        telemetry.addData("ly: ", driverGamepad.getLeftX());
//        telemetry.addData("rx: ", driverGamepad.getLeftY());
//        telemetry.addData("lx: ", driverGamepad.getRightX());
//        telemetry.addData("heading", swerveDrive.getHeading());
//        TelemetryPacket packet = new TelemetryPacket();
//        packet.put("Error", (swerveDrive.fl.servo.error));
//        packet.put("targer", Math.abs(targetAngle));
//        packet.put("Min Bound", -25);
//        packet.put("Max Bound", 90);
//        packet.put("Min Error", -10);
//        packet.put("Max Error", 8);
//
//
//        dashboard.sendTelemetryPacket(packet);
    }

    public static double getKp() {
        return kp;
    }

    public static double getKi() {
        return ki;
    }

    public static double getKd() {
        return kd;
    }

    public static double getMin() {
        return minPower;
    }

}





