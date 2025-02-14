package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;

public class AllianceUtilities {
    /**
     * @return whether the robot is on the blue alliance
     */
    public static boolean isBlueAlliance() {
        return DriverStation.getAlliance().get() == DriverStation.Alliance.Blue;
    }

    /**
     * Converts a pose to the pose relative to the current driver station alliance.
     *
     * @param pose the current blue alliance pose
     * @return the converted pose
     */
    public static Pose2d toAlliancePose(Pose2d pose) {
        if (isBlueAlliance())
            return pose;

        return new Pose2d(
                FieldConstants.fieldLength - pose.getX(),
                FieldConstants.fieldWidth - pose.getY(),
                pose.getRotation().minus(Rotation2d.fromRotations(0.5))
        );
    }
}
