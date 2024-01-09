package frc.robot.Swerve;

import edu.wpi.first.math.geometry.Rotation2d;

public class SwerveConstants {

    final double kGearRatio = 4.71; //4.71:1 presumably
    final int kWheelDiameterInches = 3;
    final double kPhysicalMaxSpeedMetersPerSecond = 0.0;


    public static final class Mod0{
        public static final String moduleName = "";
        public static final int driveCANId = 0;
        public static final int steerCANId = 0;
        public static final Rotation2d angularOffset = Rotation2d.fromDegrees(0.0);
        public static final boolean isReversed = true;
        public static final SwerveModuleConstants constants = new SwerveModuleConstants(moduleName, driveCANId, steerCANId, angularOffset, isReversed);
    }
    
    public static final class Mod1{
        public static final String moduleName = "";
        public static final int driveCANId = 0;
        public static final int steerCANId = 0;
        public static final Rotation2d angularOffset = Rotation2d.fromDegrees(0.0);
        public static final boolean isReversed = true;
        public static final SwerveModuleConstants constants = new SwerveModuleConstants(moduleName, driveCANId, steerCANId, angularOffset, isReversed);
    }
    
    public static final class Mod2{
        public static final String moduleName = "";
        public static final int driveCANId = 0;
        public static final int steerCANId = 0;
        public static final Rotation2d angularOffset = Rotation2d.fromDegrees(0.0);
        public static final boolean isReversed = true;
        public static final SwerveModuleConstants constants = new SwerveModuleConstants(moduleName, driveCANId, steerCANId, angularOffset, isReversed);
    }
    
    public static final class Mod3{
        public static final String moduleName = "";
        public static final int driveCANId = 0;
        public static final int steerCANId = 0;
        public static final Rotation2d angularOffset = Rotation2d.fromDegrees(0.0);
        public static final boolean isReversed = true;
        public static final SwerveModuleConstants constants = new SwerveModuleConstants(moduleName, driveCANId, steerCANId, angularOffset, isReversed);
    }
}
