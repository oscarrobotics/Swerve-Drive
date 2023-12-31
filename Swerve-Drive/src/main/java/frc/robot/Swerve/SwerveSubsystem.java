package frc.robot.Swerve;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveSubsystem{

    //set can IDs
    public final SwerveModule flModule = new SwerveModule("Front Left", 0, 1, 2, 0, false);
    public final SwerveModule frModule = new SwerveModule("Front Right", 1, 3, 4, 0, false);
    public final SwerveModule rlModule = new SwerveModule("Rear Left", 2, 5, 6, 0, false);
    public final SwerveModule rrModule = new SwerveModule("Rear Right", 3, 7, 8, 0, false);

    public final Pigeon2 m_gyro = new Pigeon2(0);

    // public final SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(m_kinematics, m_gyro.getYaw(), getModulePositions(), new Pose2d(), );

    public final SwerveModule[] m_modules = new SwerveModule[]{
        flModule, frModule, rlModule, rrModule
    };

    public SwerveModuleState[] getModuleStates() {
		SwerveModuleState[] states = new SwerveModuleState[4];
		for (SwerveModule mod : m_modules) {
			states[mod.moduleNumber] = mod.getState();
		}
		return states;
	}

    public SwerveModulePosition[] getModulePositions() {
		SwerveModulePosition[] positions = new SwerveModulePosition[4];
		for (SwerveModule mod : m_modules) {
			positions[mod.moduleNumber] = mod.getPosition();
		}
		return positions;
	}

    public void setModuleStates(SwerveModuleState[] desiredStates, boolean isOpenLoop, boolean steerInPlace){
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, 14);

        for (SwerveModule mod : m_modules){
            mod.setDesiredState(desiredStates[mod.moduleNumber], isOpenLoop, steerInPlace);
        }
    }
}


