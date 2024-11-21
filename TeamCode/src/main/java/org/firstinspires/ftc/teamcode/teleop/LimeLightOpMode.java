package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;
@Config
@TeleOp(group = "limelight")
public class LimeLightOpMode extends LinearOpMode {
    public static int pipeLine = 0;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();
    MultipleTelemetry multipleTelemetry = new MultipleTelemetry(telemetry, dashboardTelemetry);
    private Limelight3A limelight;

    @Override
    public void runOpMode() throws InterruptedException {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        telemetry.setMsTransmissionInterval(11);

        limelight.start();
        waitForStart();
        while (opModeIsActive()) {
            limelight.pipelineSwitch(pipeLine);
            LLResult result = limelight.getLatestResult();
           if (result != null) {
                if (result.isValid()) {
                    Pose3D botpose = result.getBotpose();
                    multipleTelemetry.addData("tx", result.getTx());
                    multipleTelemetry.addData("ty", result.getTy());
                    multipleTelemetry.addData("Object size", result.getTa());
                    multipleTelemetry.addData("Botpose", botpose.toString());
                    multipleTelemetry.addData("Current Index", result.getPipelineIndex());
                    // Access fiducial results
                    List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
                    for (LLResultTypes.FiducialResult fr : fiducialResults) {
                        multipleTelemetry.addData("Fiducial", "ID: %d, Family: %s, X: %.2f, Y: %.2f",
                                fr.getFiducialId(), fr.getFamily(),fr.getTargetXDegrees(), fr.getTargetYDegrees());
                    dashboardTelemetry.update();
                    telemetry.update();
                }
            }
//            // print some data for each detected target
//            /*else*/ if (result.isValid()) {
//                // Access fiducial results
//                List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
//                for (LLResultTypes.FiducialResult fr : fiducialResults) {
//                    multipleTelemetry.addData("Fiducial", "ID: %d, Family: %s, X: %.2f, Y: %.2f",
//                            fr.getFiducialId(), fr.getFamily(),fr.getTargetXDegrees(), fr.getTargetYDegrees());
//                }

                // Access color results
                List<LLResultTypes.ColorResult> colorResults = result.getColorResults();
                for (LLResultTypes.ColorResult cr : colorResults) {
                    telemetry.addData("Color", "X: %.2f, Y: %.2f",
                            cr.getTargetXDegrees(), cr.getTargetYDegrees());
                }
                multipleTelemetry.update();
            }
        }
    }
}