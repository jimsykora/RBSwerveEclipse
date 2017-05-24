
package org.usfirst.frc2839.RBSwerveEclipse;

import edu.wpi.first.wpilibj.IterativeRobot;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;

//import org.usfirst.frc2839.robot.subsystems.Vision;   //apparently not needed
import org.usfirst.frc2839.RBSwerveEclipse.commands.*;
import org.usfirst.frc2839.RBSwerveEclipse.subsystems.*;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {

	public static final Vision vision = new Vision();
    public static OI oi;
    Command autonomousCommand;//was above
    
    public static LFSPIDSubsystem lFSPIDSubsystem;
    public static LFDPIDSubsystem lFDPIDSubsystem;
    public static RFSPIDSubsystem rFSPIDSubsystem;
    public static RFDPIDSubsystem rFDPIDSubsystem;
    public static RRSPIDSubsystem rRSPIDSubsystem;
    public static RRDPIDSubsystem rRDPIDSubsystem;
    public static LRSPIDSubsystem lRSPIDSubsystem;
    public static LRDPIDSubsystem lRDPIDSubsystem;
    //public static GyroRangefinder gyroRangefinder;
    
    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    public void robotInit() {
    RobotMap.init();
        lFSPIDSubsystem = new LFSPIDSubsystem();
        lFDPIDSubsystem = new LFDPIDSubsystem();
        rFSPIDSubsystem = new RFSPIDSubsystem();
        rFDPIDSubsystem = new RFDPIDSubsystem();
        rRSPIDSubsystem = new RRSPIDSubsystem();
        rRDPIDSubsystem = new RRDPIDSubsystem();
        lRSPIDSubsystem = new LRSPIDSubsystem();
        lRDPIDSubsystem = new LRDPIDSubsystem();

        // OI must be constructed after subsystems. If the OI creates Commands
        //(which it very likely will), subsystems are not guaranteed to be
        // constructed yet. Thus, their requires() statements may grab null
        // pointers. Bad news. Don't move it.
        oi = new OI();

        // instantiate the command used for the autonomous period

        autonomousCommand = new AutonomousCommand();

    }

	/**
     * This function is called once each time the robot enters Disabled mode.
     * You can use it to reset any subsystem information you want to clear when
	 * the robot is disabled.
     */
    public void disabledInit(){
    }

    public void disabledPeriodic() {
        Scheduler.getInstance().run();
        updateStatus();    // added in SD video
    }
    
	/**
	 * This autonomous (along with the chooser code above) shows how to select between different autonomous modes
	 * using the dashboard. The sendable chooser code works with the Java SmartDashboard. If you prefer the LabVIEW
	 * Dashboard, remove all of the chooser code and uncomment the getString code to get the auto name from the text box
	 * below the Gyro
	 *
	 * You can add additional auto modes by adding additional commands to the chooser code above (like the commented example)
	 * or additional comparisons to the switch structure below with additional strings & commands.
	 */
    public void autonomousInit() {
    	
    	//autonomousCommand = (Command) chooser.getSelected();
    	
		/* String autoSelected = SmartDashboard.getString("Auto Selector", "Default");
		switch(autoSelected) {
		case "My Auto":
			autonomousCommand = new MyAutoCommand();
			break;
		case "Default Auto":
		default:
			autonomousCommand = new ExampleCommand();
			break;
		} */
    	
    	// schedule the autonomous command (example)
        if (autonomousCommand != null) autonomousCommand.start();
    }

    /**
     * This function is called periodically during autonomous
     */
    public void autonomousPeriodic() {
        Scheduler.getInstance().run();
    }

    public void teleopInit() {
        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
        if (autonomousCommand != null) autonomousCommand.cancel();
        updateStatus();            // added in SD video
    }

    /**
     * This function is called periodically during operator control
     */
    public void teleopPeriodic() {
        Scheduler.getInstance().run();
        updateStatus();            // added in SD video
    }

    /**
     * This function is called periodically during test mode
     */
    public void testPeriodic() {
        LiveWindow.run();
    }
    
    public void updateStatus() {   //added in SD video
        lFDPIDSubsystem.updateStatus();
        lFSPIDSubsystem.updateStatus();
        rFDPIDSubsystem.updateStatus();
        rFSPIDSubsystem.updateStatus();
        rRDPIDSubsystem.updateStatus();
        rRSPIDSubsystem.updateStatus();
        lRDPIDSubsystem.updateStatus();
        lRSPIDSubsystem.updateStatus();
        //gyroRangefinder.updateStatus();
    }
}
