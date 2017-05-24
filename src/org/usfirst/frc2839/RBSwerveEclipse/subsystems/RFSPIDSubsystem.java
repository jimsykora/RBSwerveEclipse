
package org.usfirst.frc2839.RBSwerveEclipse.subsystems;

import org.usfirst.frc2839.RBSwerveEclipse.RobotMap;
import org.usfirst.frc2839.RBSwerveEclipse.commands.*;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.VictorSP;

import edu.wpi.first.wpilibj.command.PIDSubsystem;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class RFSPIDSubsystem extends PIDSubsystem {
    // Put methods for controlling this subsystem
    // here. Call these from Commands.
    AnalogInput rFSPot = RobotMap.rFSPIDSubsystemRFSPot;// was preceded by private final
    SpeedController rFSSpeedController = RobotMap.rFSPIDSubsystemRFSSpeedController;// was preceded by private final

    // Initialize your subsystem here
    public RFSPIDSubsystem() {
        super("RFSPIDSubsystem", -1.0, 0.0, 0.0);
        setAbsoluteTolerance(0.05);
        getPIDController().setContinuous(true);
        LiveWindow.addActuator("RFS PID Subsystem", "PIDSubsystem Controller", getPIDController());
        getPIDController().setOutputRange(-1.0, 1.0);
        getPIDController().setInputRange(0.0, 5.0);
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
        return rFSPot.getAverageVoltage()+2.5-RobotMap.offsetRFS;//2.5 is the joystick signal when pushed forward
    }

    protected void usePIDOutput(double output) {
        // Use output to drive your system, like a motor
        // e.g. yourMotor.set(output);
        rFSSpeedController.pidWrite(output);
    }
    public void updateStatus(){    // added in SD video
        SmartDashboard.putNumber("RFSPot",rFSPot.getAverageVoltage()); //adds sensor output to SmartDashboard
    }
}
