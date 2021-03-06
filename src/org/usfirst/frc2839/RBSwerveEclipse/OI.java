
package org.usfirst.frc2839.RBSwerveEclipse;

import org.usfirst.frc2839.RBSwerveEclipse.OI;
import org.usfirst.frc2839.RBSwerveEclipse.Robot;
import org.usfirst.frc2839.RBSwerveEclipse.commands.*;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
    //// CREATING BUTTONS
    // One type of button is a joystick button which is any button on a joystick.
    // You create one by telling it which joystick it's on and which button
    // number it is.
    // Joystick stick = new Joystick(port);
    // Button button = new JoystickButton(stick, buttonNumber);

    // There are a few additional built in buttons you can use. Additionally,
    // by subclassing Button you can create custom triggers and bind those to
    // commands the same as any other Button.

    //// TRIGGERING COMMANDS WITH BUTTONS
    // Once you have a button, it's trivial to bind it to a button in one of
    // three ways:

    // Start the command when the button is pressed and let it run the command
    // until it is finished as determined by it's isFinished method.
    // button.whenPressed(new ExampleCommand());

    // Run the command while the button is being held down and interrupt it once
    // the button is released.
    // button.whileHeld(new ExampleCommand());

    // Start the command when the button is released  and let it run the command
    // until it is finished as determined by it's isFinished method.
    // button.whenReleased(new ExampleCommand());

    public JoystickButton joystickButton1;
    public JoystickButton joystickButton2;
    public static final Joystick joystick = new Joystick(1); ///

    public OI() {        
        joystickButton1 = new JoystickButton(joystick, 1);
        joystickButton1.whileHeld(new StartSetpointCommand(0));//hold trigger to move straight FWD or REV
        joystickButton1.whenReleased(new SwerveSetpointCommand());//releasing button gets back to swerve mode
        joystickButton2 = new JoystickButton(joystick, 2);
        joystickButton2.whileHeld(new SpinSetpointCommand());//hold thumb button & twist joystick to spin in place
        joystickButton2.whenReleased(new SwerveSetpointCommand());//releasing button gets back to swerve mode
    }

    public void updateStatus() {   //added in SD video
    }
}

