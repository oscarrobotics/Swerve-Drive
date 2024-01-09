package frc.robot.Swerve;

import edu.wpi.first.math.geometry.Rotation2d;

public class SwerveModuleConstants {
    public final String moduleName;
    public final int driveCANId;
    public final int steerCANId;
    public final Rotation2d angularOffset;
    public final boolean isReversed;

    public SwerveModuleConstants(String moduleName, int driveCANId, int steerCANId, Rotation2d angularOffset, boolean isReversed){
        this.moduleName = moduleName;
        this.driveCANId = driveCANId;
        this.steerCANId = steerCANId;
        this.angularOffset = angularOffset;
        this.isReversed = isReversed;
    }
}
