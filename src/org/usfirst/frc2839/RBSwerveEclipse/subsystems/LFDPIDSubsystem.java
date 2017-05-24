
package org.usfirst.frc2839.RBSwerveEclipse.subsystems;

import org.usfirst.frc2839.RBSwerveEclipse.Robot;
import org.usfirst.frc2839.RBSwerveEclipse.RobotMap;
import org.usfirst.frc2839.RBSwerveEclipse.OI;
import org.usfirst.frc2839.RBSwerveEclipse.commands.*;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.command.PIDSubsystem;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class LFDPIDSubsystem extends PIDSubsystem {
    // Put methods for controlling this subsystem
    // here. Call these from Commands.
    Encoder lFDEncoder = RobotMap.lFDPIDSubsystemLFDEncoder;//removed private final
    SpeedController lFDSpeedController = RobotMap.lFDPIDSubsystemLFDSpeedController;//removed private final


    // Initialize your subsystem here
    public LFDPIDSubsystem() {
        super("LFDPIDSubsystem", 0.003, 0.0, 0.0);//0.003, 0.0, 0.0, 2.2 apparently only PI&D are needed
        setAbsoluteTolerance(1);
        getPIDController().setContinuous(false);
        LiveWindow.addActuator("LFD PID Subsystem", "PIDSubsystem Controller", getPIDController());
        getPIDController().setInputRange(-250, 250);  //should match joystick input
        //getPIDController().setOutputRange(-250, 250); //apparently no impact
        // Use these to get going:
        // setSetpoint() -  Sets where the PID controller should move the system to
        // enable() - Enables the PID controller.
    }

    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    	setDefaultCommand(new SwerveSetpointCommand());
    }

    protected double returnPIDInput() {
        // Return your input value for the PID loop
        // e.g. a sensor, like a potentiometer:
        // yourPot.getAverageVoltage() / kYourMaxVoltage;
        return lFDEncoder.pidGet();
    }

    protected void usePIDOutput(double output) {
        // Use output to drive your system, like a motor
        // e.g. yourMotor.set(output);
        lFDSpeedController.pidWrite(output);
    }
    public void updateStatus(){    // added in SD video
        SmartDashboard.putNumber("LFD Encoder",lFDEncoder.getRate());  //adds sensor output to SmartDashboard
    	SmartDashboard.putNumber("JS Magnitude",OI.joystick.getMagnitude());  //adds sensor output to SmartDashboard
    	SmartDashboard.putNumber("JS X dir",OI.joystick.getX());  //adds sensor output to SmartDashboard
    	SmartDashboard.putNumber("JS Y dir",OI.joystick.getY());  //adds sensor output to SmartDashboard
    	SmartDashboard.putNumber("JS Throttle",OI.joystick.getThrottle());  //adds sensor output to SmartDashboard
    	SmartDashboard.putNumber("JS Twist",OI.joystick.getTwist());  //adds sensor output to SmartDashboard
		SmartDashboard.putNumber("get LFDsetpoint", Robot.lFDPIDSubsystem.getSetpoint());
		SmartDashboard.putNumber("get RFDsetpoint", Robot.rFDPIDSubsystem.getSetpoint());
		SmartDashboard.putNumber("get RRDsetpoint", Robot.rRDPIDSubsystem.getSetpoint());
		SmartDashboard.putNumber("get LRDsetpoint", Robot.lRDPIDSubsystem.getSetpoint());
		SmartDashboard.putNumber("get LFSsetpoint", Robot.lFSPIDSubsystem.getSetpoint());
		SmartDashboard.putNumber("get RFSsetpoint", Robot.rFSPIDSubsystem.getSetpoint());
		SmartDashboard.putNumber("get RRSsetpoint", Robot.rRSPIDSubsystem.getSetpoint());
		SmartDashboard.putNumber("get LRSsetpoint", Robot.lRSPIDSubsystem.getSetpoint());
		SmartDashboard.putNumber("Addition",lFDEncoder.getRate()+RobotMap.rFDPIDSubsystemRFDEncoder.getRate());
		SmartDashboard.putNumber("Minimum",Math.min(  Math.min(lFDEncoder.getRate(),RobotMap.rFDPIDSubsystemRFDEncoder.getRate()),  			Math.min(RobotMap.rRDPIDSubsystemRRDEncoder.getRate(), RobotMap.lRDPIDSubsystemLRDEncoder.getRate()) ));
		SmartDashboard.putNumber("Minimum2",Math.min(Math.min(5, 2),				Math.min(3, 4)));
	
	    double min_count_rate = Math.min(  Math.min(RobotMap.lFDPIDSubsystemLFDEncoder.getRate(),RobotMap.rFDPIDSubsystemRFDEncoder.getRate()),  			Math.min(RobotMap.rRDPIDSubsystemRRDEncoder.getRate(), RobotMap.lRDPIDSubsystemLRDEncoder.getRate()) );
		SmartDashboard.putNumber("min count rate", min_count_rate);
		double lf_correction = min_count_rate/RobotMap.lFDPIDSubsystemLFDEncoder.getRate();
		SmartDashboard.putNumber("lf_correction", lf_correction);
		double rf_correction = min_count_rate/RobotMap.rFDPIDSubsystemRFDEncoder.getRate();
		SmartDashboard.putNumber("rf_correction", lf_correction);
		double rr_correction = min_count_rate/RobotMap.rRDPIDSubsystemRRDEncoder.getRate();
		SmartDashboard.putNumber("rr_correction", lf_correction);
		double lr_correction = min_count_rate/RobotMap.lRDPIDSubsystemLRDEncoder.getRate();
		SmartDashboard.putNumber("lr_correction", lf_correction);
		double lf_to_rf = lf_correction/rf_correction;
		SmartDashboard.putNumber("lf_to_rf", lf_to_rf);
    }
}
