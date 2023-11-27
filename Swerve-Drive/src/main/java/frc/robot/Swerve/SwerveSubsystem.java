package frc.robot.Swerve;

import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveSubsystem{


    public void setModuleStates(){
        
    }

    public SwerveModuleState[] getModuleStates() {
		SwerveModuleState[] states = new SwerveModuleState[4];
		for (SwerveModule mod : m_modules) {
			states[mod.moduleNumber] = mod.getState();
		}
		return states;
	}
}

