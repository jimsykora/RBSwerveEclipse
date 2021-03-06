
package org.usfirst.frc2839.RBSwerveEclipse.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.Joystick;
import org.usfirst.frc2839.RBSwerveEclipse.Robot;
import org.usfirst.frc2839.RBSwerveEclipse.RobotMap;

/**
 *
 */
public class SpinSetpointCommand extends Command {
	Joystick joystick = new Joystick(1);  //need to add this and import Joystick

    public SpinSetpointCommand() {
        // Use requires() here to declare subsystem dependencies
        requires(Robot.lFSPIDSubsystem);
        requires(Robot.rFSPIDSubsystem); //aded so that one button controls all steer or drive motors
        requires(Robot.rRSPIDSubsystem);
        requires(Robot.lRSPIDSubsystem);
        requires(Robot.lFDPIDSubsystem);
        requires(Robot.rFDPIDSubsystem);
        requires(Robot.rRDPIDSubsystem);
        requires(Robot.lRDPIDSubsystem);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	Robot.lFSPIDSubsystem.enable();
		Robot.lFSPIDSubsystem.setSetpoint(RobotMap.spinLFS + 2.5);//2.5 is the setpoint steer voltage going forward
		Robot.rFSPIDSubsystem.enable();
		Robot.rFSPIDSubsystem.setSetpoint(RobotMap.spinRFS + 2.5);
		Robot.rRSPIDSubsystem.enable();
		Robot.rRSPIDSubsystem.setSetpoint(RobotMap.spinRRS + 2.5);
		Robot.lRSPIDSubsystem.enable();
		Robot.lRSPIDSubsystem.setSetpoint(RobotMap.spinLRS + 2.5);
		
		double speed = joystick.getTwist()*Math.abs(joystick.getTwist())*(1+joystick.getThrottle()*-1)/2*250;//for better control @ low speed. Multiplier to match PID input range
		Robot.lFDPIDSubsystem.enable();
		Robot.lFDPIDSubsystem.setSetpoint(speed);		
		Robot.rFDPIDSubsystem.enable();
		Robot.rFDPIDSubsystem.setSetpoint(speed);
		Robot.rRDPIDSubsystem.enable();
		Robot.rRDPIDSubsystem.setSetpoint(speed);
		Robot.lRDPIDSubsystem.enable();
		Robot.lRDPIDSubsystem.setSetpoint(speed);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return Robot.lFSPIDSubsystem.onTarget();
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
