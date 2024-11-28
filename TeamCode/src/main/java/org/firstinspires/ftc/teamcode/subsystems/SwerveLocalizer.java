package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.kinematics.Kinematics;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.acmerobotics.roadrunner.util.Angle;

import org.firstinspires.ftc.teamcode.subsystems.SwerveDrive;
import org.firstinspires.ftc.teamcode.subsystems.SwerveKinematics;

import java.util.ArrayList;
import java.util.List;

public class SwerveLocalizer implements Localizer {
    private final SwerveDrive drive;
    private final boolean useExternalHeading;
    private Pose2d _poseEstimate = new Pose2d();
    private List<Double> lastWheelPositions = new ArrayList<>();
    private double lastExtHeading = Double.NaN;

    public SwerveLocalizer(SwerveDrive drive) {
        this(drive, true);
    }

    public SwerveLocalizer(SwerveDrive drive, boolean useExternalHeading) {
        this.drive = drive;
        this.useExternalHeading = useExternalHeading;
    }

    @Override
    public Pose2d getPoseEstimate() {
        return _poseEstimate;
    }

    @Override
    public void setPoseEstimate(Pose2d value) {
        lastWheelPositions = new ArrayList<>();
        lastExtHeading = Double.NaN;
        if (useExternalHeading) {
            drive.setExternalHeading(value.getHeading());
        }
        _poseEstimate = value;
    }

    @Override
    public Pose2d getPoseVelocity() {
        return poseVelocity;
    }

    private Pose2d poseVelocity = null;

    @Override
    public void update() {
        // order - frontLeft, backLeft, frontRight, backRight
        List<Double> wheelPositions = drive.getWheelPositions();
        List<Double> moduleOrientations = drive.getModuleOrientations();
        double extHeading = useExternalHeading ? drive.getExternalHeading() : Double.NaN;

        if (!lastWheelPositions.isEmpty()) {
            List<Double> wheelDeltas = new ArrayList<>();
            for (int i = 0; i < wheelPositions.size(); i++) {
                wheelDeltas.add(wheelPositions.get(i) - lastWheelPositions.get(i));
            }

            Pose2d robotPoseDelta = SwerveKinematics.wheelToRobotVelocities(
                    wheelDeltas,
                    moduleOrientations,
                    drive.getWheelBase(),
                    drive.getTrackWidth()
            );

            double finalHeadingDelta = useExternalHeading
                    ? Angle.normDelta(extHeading - lastExtHeading)
                    : robotPoseDelta.getHeading();

            _poseEstimate = Kinematics.relativeOdometryUpdate(
                    _poseEstimate,
                    new Pose2d(robotPoseDelta.vec(), finalHeadingDelta)
            );
        }

        List<Double> wheelVelocities = drive.getWheelVelocities();
        Double extHeadingVel = drive.getExternalHeadingVelocity();

        if (wheelVelocities != null) {
            poseVelocity = SwerveKinematics.wheelToRobotVelocities(
                    wheelVelocities,
                    moduleOrientations,
                    drive.getWheelBase(),
                    drive.getTrackWidth()
            );

            if (useExternalHeading && extHeadingVel != null) {
                poseVelocity = new Pose2d(poseVelocity.vec(), extHeadingVel);
            }
        }

        lastWheelPositions = wheelPositions;
        lastExtHeading = extHeading;
    }
}
