
package org.usfirst.frc2839.RBSwerveEclipse.subsystems;

import org.usfirst.frc2839.RBSwerveEclipse.commands.SwerveSetpointCommand;
import org.usfirst.frc2839.RBSwerveEclipse.RobotMap;
import org.usfirst.frc2839.RBSwerveEclipse.commands.*;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.VictorSP;

import edu.wpi.first.wpilibj.command.PIDSubsystem;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class LRDPIDSubsystem extends PIDSubsystem {
    // Put methods for controlling this subsystem
    // here. Call these from Commands.
    private final Encoder lRDEncoder = RobotMap.lRDPIDSubsystemLRDEncoder;
    private final SpeedController lRDSpeedController = RobotMap.lRDPIDSubsystemLRDSpeedController;

    // Initialize your subsystem here
    public LRDPIDSubsystem() {
        super("LRDPIDSubsystem", 0.003, 0.0, 0.0);//0.003, 0.0, 0.0, 2.2 apparently only PI&D are needed
        setAbsoluteTolerance(1);
        getPIDController().setContinuous(false);
        LiveWindow.addActuator("LRD PID Subsystem", "PIDSubsystem Controller", getPIDController());
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
        return lRDEncoder.pidGet();
    }

    protected void usePIDOutput(double output) {
        // Use output to drive your system, like a motor
        // e.g. yourMotor.set(output);
        lRDSpeedController.pidWrite(output);
    }
    public void updateStatus(){    // added in SD video
        SmartDashboard.putNumber("LRD Encoder",lRDEncoder.getRate());  //adds sensor output to SmartDashboard
    }
}
