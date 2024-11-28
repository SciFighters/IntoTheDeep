package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import java.util.ArrayList;
import java.util.List;

public class SwerveKinematics {

    /**
     * Computes the wheel velocity vectors corresponding to [robotVel] given the provided [trackWidth] and
     * [wheelBase].
     *
     * @param robotVel velocity of the robot in its reference frame
     * @param trackWidth lateral distance between pairs of wheels on different sides of the robot
     * @param wheelBase distance between pairs of wheels on the same side of the robot
     * @return List of wheel velocity vectors
     */
    public static List<Vector2d> robotToModuleVelocityVectors(
            Pose2d robotVel,
            double trackWidth,
            double wheelBase) {
        double x = wheelBase / 2;
        double y = trackWidth / 2;

        double vx = robotVel.getX();
        double vy = robotVel.getY();
        double omega = robotVel.getHeading();

        List<Vector2d> velocities = new ArrayList<>();
        velocities.add(new Vector2d(vx - omega * y, vy + omega * x));
        velocities.add(new Vector2d(vx - omega * y, vy - omega * x));
        velocities.add(new Vector2d(vx + omega * y, vy + omega * x));
        velocities.add(new Vector2d(vx + omega * y, vy - omega * x));

        return velocities;
    }

    /**
     * Computes the wheel velocities corresponding to [robotVel] given the provided [trackWidth] and
     * [wheelBase].
     *
     * @param robotVel velocity of the robot in its reference frame
     * @param trackWidth lateral distance between pairs of wheels on different sides of the robot
     * @param wheelBase distance between pairs of wheels on the same side of the robot
     * @return List of wheel velocities
     */
    public static List<Double> robotToWheelVelocities(
            Pose2d robotVel,
            double trackWidth,
            double wheelBase) {
        List<Vector2d> moduleVelocities = robotToModuleVelocityVectors(robotVel, trackWidth, wheelBase);
        List<Double> wheelVelocities = new ArrayList<>();

        for (Vector2d velocity : moduleVelocities) {
            wheelVelocities.add(velocity.norm());
        }

        return wheelVelocities;
    }

    /**
     * Computes the module orientations (in radians) corresponding to [robotVel] given the provided
     * [trackWidth] and [wheelBase].
     *
     * @param robotVel velocity of the robot in its reference frame
     * @param trackWidth lateral distance between pairs of wheels on different sides of the robot
     * @param wheelBase distance between pairs of wheels on the same side of the robot
     * @return List of module orientations (angles)
     */
    public static List<Double> robotToModuleOrientations(
            Pose2d robotVel,
            double trackWidth,
            double wheelBase) {
        List<Vector2d> moduleVelocities = robotToModuleVelocityVectors(robotVel, trackWidth, wheelBase);
        List<Double> orientations = new ArrayList<>();

        for (Vector2d velocity : moduleVelocities) {
            orientations.add(velocity.angle());
        }

        return orientations;
    }

    /**
     * Computes the acceleration vectors corresponding to [robotAccel] given the provided [trackWidth] and
     * [wheelBase].
     *
     * @param robotAccel velocity of the robot in its reference frame
     * @param trackWidth lateral distance between pairs of wheels on different sides of the robot
     * @param wheelBase distance between pairs of wheels on the same side of the robot
     * @return List of acceleration vectors
     */
    public static List<Vector2d> robotToModuleAccelerationVectors(
            Pose2d robotAccel,
            double trackWidth,
            double wheelBase) {
        double x = wheelBase / 2;
        double y = trackWidth / 2;

        double ax = robotAccel.getX();
        double ay = robotAccel.getY();
        double alpha = robotAccel.getHeading();

        List<Vector2d> accelerations = new ArrayList<>();
        accelerations.add(new Vector2d(ax - alpha * y, ay + alpha * x));
        accelerations.add(new Vector2d(ax - alpha * y, ay - alpha * x));
        accelerations.add(new Vector2d(ax + alpha * y, ay - alpha * x));
        accelerations.add(new Vector2d(ax + alpha * y, ay + alpha * x));

        return accelerations;
    }

    /**
     * Computes the wheel accelerations corresponding to [robotAccel] given the provided [trackWidth] and
     * [wheelBase].
     *
     * @param robotVel velocity of the robot in its reference frame
     * @param robotAccel acceleration of the robot in its reference frame
     * @param trackWidth lateral distance between pairs of wheels on different sides of the robot
     * @param wheelBase distance between pairs of wheels on the same side of the robot
     * @return List of wheel accelerations
     */
    public static List<Double> robotToWheelAccelerations(
            Pose2d robotVel,
            Pose2d robotAccel,
            double trackWidth,
            double wheelBase) {
        List<Vector2d> moduleVelocities = robotToModuleVelocityVectors(robotVel, trackWidth, wheelBase);
        List<Vector2d> moduleAccelerations = robotToModuleAccelerationVectors(robotAccel, trackWidth, wheelBase);

        List<Double> wheelAccelerations = new ArrayList<>();
        for (int i = 0; i < moduleVelocities.size(); i++) {
            Vector2d velocity = moduleVelocities.get(i);
            Vector2d acceleration = moduleAccelerations.get(i);
            wheelAccelerations.add((velocity.getX() * acceleration.getX() + velocity.getY() * acceleration.getY()) / velocity.norm());
        }

        return wheelAccelerations;
    }

    /**
     * Computes the module angular velocities corresponding to [robotAccel] given the provided [trackWidth]
     * and [wheelBase].
     *
     * @param robotVel velocity of the robot in its reference frame
     * @param robotAccel acceleration of the robot in its reference frame
     * @param trackWidth lateral distance between pairs of wheels on different sides of the robot
     * @param wheelBase distance between pairs of wheels on the same side of the robot
     * @return List of module angular velocities
     */
    public static List<Double> robotToModuleAngularVelocities(
            Pose2d robotVel,
            Pose2d robotAccel,
            double trackWidth,
            double wheelBase) {
        List<Vector2d> moduleVelocities = robotToModuleVelocityVectors(robotVel, trackWidth, wheelBase);
        List<Vector2d> moduleAccelerations = robotToModuleAccelerationVectors(robotAccel, trackWidth, wheelBase);

        List<Double> angularVelocities = new ArrayList<>();
        for (int i = 0; i < moduleVelocities.size(); i++) {
            Vector2d velocity = moduleVelocities.get(i);
            Vector2d acceleration = moduleAccelerations.get(i);
            angularVelocities.add((velocity.getX() * acceleration.getY() - velocity.getY() * acceleration.getX()) /
                    (velocity.getX() * velocity.getX() + velocity.getY() * velocity.getY()));
        }

        return angularVelocities;
    }

    /**
     * Computes the robot velocities corresponding to [wheelVelocities], [moduleOrientations], and the drive parameters.
     *
     * @param wheelVelocities wheel velocities (or wheel position deltas)
     * @param moduleOrientations wheel orientations (in radians)
     * @param trackWidth lateral distance between pairs of wheels on different sides of the robot
     * @param wheelBase distance between pairs of wheels on the same side of the robot
     * @return Pose2d representing the robot's velocity
     */
    public static Pose2d wheelToRobotVelocities(
            List<Double> wheelVelocities,
            List<Double> moduleOrientations,
            double trackWidth,
            double wheelBase) {
        double x = wheelBase / 2;
        double y = trackWidth / 2;

        List<Vector2d> vectors = new ArrayList<>();
        for (int i = 0; i < wheelVelocities.size(); i++) {
            double vel = wheelVelocities.get(i);
            double orientation = moduleOrientations.get(i);
            vectors.add(new Vector2d(vel * Math.cos(orientation), vel * Math.sin(orientation)));
        }

        double vx = 0;
        double vy = 0;
        for (Vector2d vector : vectors) {
            vx += vector.getX();
            vy += vector.getY();
        }
        vx /= 4;
        vy /= 4;

        Vector2d frontLeft = vectors.get(0);
        Vector2d rearLeft = vectors.get(1);
        Vector2d rearRight = vectors.get(2);
        Vector2d frontRight = vectors.get(3);

        double omega = (y * (rearRight.getX() + frontRight.getX() - frontLeft.getX() - rearLeft.getX()) +
                x * (frontLeft.getY() + frontRight.getY() - rearLeft.getY() - rearRight.getY())) / (4 * (x * x + y * y));

        return new Pose2d(vx, vy, omega);
    }
}
