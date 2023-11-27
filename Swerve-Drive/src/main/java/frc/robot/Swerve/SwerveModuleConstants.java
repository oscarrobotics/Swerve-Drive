package frc.robot.Swerve;

public class SwerveModuleConstants {
    public final String moduleName;
    public final int driveCANId;
    public final int steerCANId;
    public final double angularOffset;
    public final boolean isReversed;

    public SwerveModuleConstants(String moduleName, int driveCANId, int steerCANId, double angularOffset, boolean isReversed){
        this.moduleName = moduleName;
        this.driveCANId = driveCANId;
        this.steerCANId = steerCANId;
        this.angularOffset = angularOffset;
        this.isReversed = isReversed;
    }
}
