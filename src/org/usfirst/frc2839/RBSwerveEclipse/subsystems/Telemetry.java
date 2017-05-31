package org.usfirst.frc2839.RBSwerveEclipse.subsystems;

import org.usfirst.frc2839.RBSwerveEclipse.OI;
import org.usfirst.frc2839.RBSwerveEclipse.Robot;
import org.usfirst.frc2839.RBSwerveEclipse.RobotMap;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class Telemetry extends Subsystem {

    // Put methods for controlling this subsystem
    // here. Call these from Commands.
	public Telemetry(){
		
	}
	
	public void update(){
		SmartDashboard.putNumber("LFD Encoder", Robot.lFDPIDSubsystem.getLFDEncoderRate());  //adds sensor output to SmartDashboard
		SmartDashboard.putNumber("RFD Encoder", Robot.rFDPIDSubsystem.getRFDEncoderRate()); 
		SmartDashboard.putNumber("RRD Encoder", Robot.rRDPIDSubsystem.getRRDEncoderRate()); 
		SmartDashboard.putNumber("LRD Encoder", Robot.lRDPIDSubsystem.getLRDEncoderRate());
		SmartDashboard.putNumber("LFS Pot", Robot.lFSPIDSubsystem.getLFSPotAvgVolt());
		SmartDashboard.putNumber("RFS Pot", Robot.rFSPIDSubsystem.getRFSPotAvgVolt());
		SmartDashboard.putNumber("RRS Pot", Robot.rRSPIDSubsystem.getRRSPotAvgVolt());
		SmartDashboard.putNumber("LRS Pot", Robot.lRSPIDSubsystem.getLRSPotAvgVolt());
		
    	SmartDashboard.putNumber("JS Magnitude",OI.joystick.getMagnitude());  //adds sensor output to SmartDashboard
    	SmartDashboard.putNumber("JS X dir",OI.joystick.getX());  //adds sensor output to SmartDashboard
    	SmartDashboard.putNumber("JS Y dir",OI.joystick.getY());  //adds sensor output to SmartDashboard
    	SmartDashboard.putNumber("JS Throttle",OI.joystick.getThrottle());  //adds sensor output to SmartDashboard
    	SmartDashboard.putNumber("JS Twist",OI.joystick.getTwist());  //adds sensor output to SmartDashboard
    	
		SmartDashboard.putNumber("get LFDsetpoint", Robot.lFDPIDSubsystem.getLFDSetpoint());
		SmartDashboard.putNumber("get RFDsetpoint", Robot.rFDPIDSubsystem.getSetpoint());
		SmartDashboard.putNumber("get RRDsetpoint", Robot.rRDPIDSubsystem.getSetpoint());
		SmartDashboard.putNumber("get LRDsetpoint", Robot.lRDPIDSubsystem.getSetpoint());
		SmartDashboard.putNumber("get LFSsetpoint", Robot.lFSPIDSubsystem.getLFSSetpoint());
		SmartDashboard.putNumber("get RFSsetpoint", Robot.rFSPIDSubsystem.getSetpoint());
		SmartDashboard.putNumber("get RRSsetpoint", Robot.rRSPIDSubsystem.getSetpoint());
		SmartDashboard.putNumber("get LRSsetpoint", Robot.lRSPIDSubsystem.getSetpoint());
		SmartDashboard.putNumber("get LFD SP error?", Robot.lFDPIDSubsystem.getLFDError());
		SmartDashboard.putNumber("get LFS SP error?", Robot.lFSPIDSubsystem.getLFSError());
		
		//revist the math below only if PID speed loop is not functioning as desired
//		SmartDashboard.putNumber("Addition",Robot.lFDPIDSubsystem.getLFDEncoderRate()+RobotMap.rFDPIDSubsystemRFDEncoder.getRate());
//		SmartDashboard.putNumber("Minimum",Math.min(  		Math.min(Robot.lFDPIDSubsystem.getLFDEncoderRate(),RobotMap.rFDPIDSubsystemRFDEncoder.getRate()),  			Math.min(RobotMap.rRDPIDSubsystemRRDEncoder.getRate(), RobotMap.lRDPIDSubsystemLRDEncoder.getRate()) ));
//		SmartDashboard.putNumber("Minimum2",Math.min(Math.min(5, 2),				Math.min(3, 4)));
	
//	    double min_count_rate = Math.min(  Math.min(RobotMap.lFDPIDSubsystemLFDEncoder.getRate(),RobotMap.rFDPIDSubsystemRFDEncoder.getRate()),  			Math.min(RobotMap.rRDPIDSubsystemRRDEncoder.getRate(), RobotMap.lRDPIDSubsystemLRDEncoder.getRate()) );
//		SmartDashboard.putNumber("min count rate", min_count_rate);
//		double lf_correction = min_count_rate/RobotMap.lFDPIDSubsystemLFDEncoder.getRate();
//		SmartDashboard.putNumber("lf_correction", lf_correction);
//		double rf_correction = min_count_rate/RobotMap.rFDPIDSubsystemRFDEncoder.getRate();
//		SmartDashboard.putNumber("rf_correction", lf_correction);
//		double rr_correction = min_count_rate/RobotMap.rRDPIDSubsystemRRDEncoder.getRate();
//		SmartDashboard.putNumber("rr_correction", lf_correction);
//		double lr_correction = min_count_rate/RobotMap.lRDPIDSubsystemLRDEncoder.getRate();
//		SmartDashboard.putNumber("lr_correction", lf_correction);
//		double lf_to_rf = lf_correction/rf_correction;
//		SmartDashboard.putNumber("lf_to_rf", lf_to_rf);
	}

    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
}

