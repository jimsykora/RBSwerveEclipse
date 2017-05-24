
package org.usfirst.frc2839.RBSwerveEclipse.commands;

import edu.wpi.first.wpilibj.command.Command;
import org.usfirst.frc2839.RBSwerveEclipse.Robot;
import org.usfirst.frc2839.RBSwerveEclipse.RobotMap;

import edu.wpi.first.wpilibj.Joystick;

/**
 *
 */
public class SwerveSetpointCommand extends Command {
    Joystick joystick = new Joystick(1);   //jim added

    public SwerveSetpointCommand() {
       requires(Robot.lFSPIDSubsystem);
       requires(Robot.rFSPIDSubsystem);//aded so that one button controls all steer or drive motors
       requires(Robot.rRSPIDSubsystem);
       requires(Robot.lRSPIDSubsystem);     
       requires(Robot.lFDPIDSubsystem);
       requires(Robot.rFDPIDSubsystem);
       requires(Robot.rRDPIDSubsystem);
       requires(Robot.lRDPIDSubsystem);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
        Robot.lFSPIDSubsystem.enable();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	double setpoint = (5-(joystick.getDirectionDegrees()+180)/72);//(5-(joystick.getDirectionDegrees()+180)/72);
    	Robot.lFSPIDSubsystem.enable();
		Robot.lFSPIDSubsystem.setSetpoint(setpoint);//+RobotMap.offsetLFS
		Robot.lRSPIDSubsystem.enable();
		Robot.lRSPIDSubsystem.setSetpoint(setpoint);//+RobotMap.offsetLRS
		Robot.rFSPIDSubsystem.enable();
		Robot.rFSPIDSubsystem.setSetpoint(setpoint);//RobotMap.offsetRFS
		Robot.rRSPIDSubsystem.enable();
		Robot.rRSPIDSubsystem.setSetpoint(setpoint);//RobotMap.offsetRRS
		
		double speed = joystick.getMagnitude()*joystick.getMagnitude()*(1+joystick.getThrottle()*-1)/2*250*-1;//for better control @ low speed. Multiplier to match PID input range
		Robot.lFDPIDSubsystem.enable();
		Robot.lFDPIDSubsystem.setSetpoint(speed); // 
		Robot.lRDPIDSubsystem.enable();
		Robot.lRDPIDSubsystem.setSetpoint(speed);
		Robot.rFDPIDSubsystem.enable();
		Robot.rFDPIDSubsystem.setSetpoint(speed);
		Robot.rRDPIDSubsystem.enable();
		Robot.rRDPIDSubsystem.setSetpoint(speed);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
