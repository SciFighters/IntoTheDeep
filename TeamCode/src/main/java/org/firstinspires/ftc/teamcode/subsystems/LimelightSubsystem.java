package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.limelightvision.*;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.opencv.core.Mat;

public class LimelightSubsystem extends SubsystemBase {
    public final Limelight3A limelight;
    MultipleTelemetry telemetry;
    int pipeline = Pipelines.BLUE.PIPELINE;
    LLResult result;
    final double limelightH = 41.5, sampleH = 3.8, limelightAngle = 27.6;
    int distance;
    public final double middleOfScreen = 300, tickPerCM = 19.34, distanceFromArmStart = 25;
    public double alignedY = 0;
    public double limelightInCm;
    public double xToOdometer = 8.76;

    public LimelightSubsystem(HardwareMap hardwareMap, MultipleTelemetry telemetry) {
        this.telemetry = telemetry;
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        telemetry.setMsTransmissionInterval(11);
        limelight.pipelineSwitch(pipeline);
        limelight.start();
    }

    public double getXDistance() {
        updateResults();
        result = limelight.getLatestResult();
        return result.getPythonOutput()[0];
    }

    public double getXDistanceOdometer() {
//        double alpha = getXDistance() / 320 * 54.5;
//        return  Math.tan(Math.toRadians(alpha)) * getYDistance();
        double rawY = getRawY() + 240;
        double y = getYDistance();
        if (rawY == 0) {
            rawY = 1;
        }
        double x = getXDistance();

        return x / rawY * y;

    }

    public double getAngle() {
        updateResults();
        return -result.getPythonOutput()[4];
    }

    public void setPipeline(Pipelines pipeline) {
        this.pipeline = pipeline.PIPELINE;
        limelight.pipelineSwitch(this.pipeline);
    }

    public double getCurrentPipeline() {
        return pipeline;
    }

    public int getYDistance() {
        updateResults();
        distance = (int) (((limelightH - sampleH) * Math.tan(Math.toRadians(-result.getPythonOutput()[1] / 240 * 42 + limelightAngle)) + distanceFromArmStart + 31.5) * tickPerCM);
        limelightInCm = (distance - 580) / 48.2;
        distance = (int) (limelightInCm * 31.85 + 861);
        alignedY = distance;
        return distance;
    }

    public double getRawY() {
        updateResults();
        return result.getPythonOutput()[1];
    }

    public void startLimelight() {
        limelight.start();
    }

    public void stopLimelight() {
        limelight.stop();
    }

    public void updateResults() {
        result = limelight.getLatestResult();
    }
}
