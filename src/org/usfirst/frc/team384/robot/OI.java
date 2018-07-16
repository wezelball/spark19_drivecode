package org.usfirst.frc.team384.robot;

import edu.wpi.first.wpilibj.Joystick;
//import edu.wpi.first.wpilibj.buttons.JoystickButton;

public class OI {
	
	// Logitech Attack 3 joysticks
	public Joystick joystick0 = new Joystick(0);
	public Joystick joystick1 = new Joystick(1);
	
	/*
	 * Need to keep track of whether a joystick button was pressed for some cases.
	 * Allocate space for 2 joysticks, 10 buttons each, for a total 
	 * of 20 elements.  The first element of the array is the joystick number, 0 or 1,
	 * and the 2nd element is the button number, 1 thru 10 (0 is not used).
	 * Note that there are 12 elements instead of 11 for the 2nd array dimension.
	 * This is because buttons are numbered 1 thru 11, whereas array elements are 
	 * numbered 0 thru 10.  The zeroth element is not used, so the array size must be
	 * one greater than the number of buttons, or an array out of bounds exception will
	 * will be thrown. 
	 * 
	 * For an array reference example, joystick1, button 4 would be joyButtonStatus [1] [4]
	 */ 
	boolean joyButtonStatus[] []  = {
			{false, false, false, false, false, false, false, false, false, false, false, false},	// Joysyick 0, buttons 1 thru 11
			{false, false, false, false, false, false, false, false, false, false, false, false}	// Joysyick 1, buttons 1 thru 11
	};
	
	
	/*
	 * Return the value of the joystick button pressed
	 * This one "sticks"
	 */
	public boolean getButtonHeld(int stick, int b)	{
		boolean result = false;
		
		if (stick == 0) {
			result = joystick0.getRawButton(b);
		} else if (stick == 1)	{
			result = joystick1.getRawButton(b);
		}
		//if (stick > 0 && stick <= 1 && b > 0 && b <= 9)
		if (result)
			joyButtonStatus[stick] [b] = true;	// store the pressed state of the button
		//System.out.println("Recorded button held for stick " + stick + " button " + b);
		return result;
	}
	
	
	/*
	 * Return the value of the joystick button pressed
	 * This one "sticks"
	 */
	public boolean getButtonPressed(int stick, int b)	{
		boolean result = false;
		
		if (stick == 0) {
			result = joystick0.getRawButtonPressed(b);
		} else if (stick == 1)	{
			result = joystick1.getRawButtonPressed(b);
		}
		if (result)
		//if (stick > 0 && stick <= 1 && b > 0 && b <= 9)
			joyButtonStatus[stick] [b] = true;	// store the pressed state of the button
		//System.out.println("Recorded button pressed for stick " + stick + " button " + b);
		return result;
	}

	
	// This is not working
	public boolean getButtonReleased(int stick, int b)	{
		boolean result = false;
		
		if (stick == 0) {
			// True if button was pressed but now released
			if (!joystick0.getRawButton(b) && joyButtonStatus[stick][b])	{
				joyButtonStatus[stick][b] = false;
				result = true;
			}
		} else if (stick == 1)	{
			if (!joystick0.getRawButton(b) && joyButtonStatus[stick][b])	{
				joyButtonStatus[stick][b] = false;
				result = true;
			}
		} 
		
		if (result)
			System.out.println("Button released for stick " + stick + " button " + b);
		return result;
	}
	
	
	/*
	 * Returns the value of the joystick axis.
	 * requires the stick number, 0 or 1, and the axis number
	 * 0=x, 1=y, 2=z
	 */
	public double getAxis(int stick, int axis) {
		double axisval = 0.0;
		
		if(stick == 0) {
			if (axis == 0)	{
				axisval = joystick0.getX();
			} else if (axis == 1)	{
				axisval = joystick0.getY();
			} else if (axis == 2)	{
				axisval = joystick0.getZ();
			}
			
		}	else if (stick == 1) {
			if (axis == 0)	{
				axisval = joystick1.getX();
			} else if (axis == 1)	{
				axisval = joystick1.getY();
			} else if (axis == 2)	{
				axisval = joystick1.getZ();
			}
		}	else	{	// an illegal value of stick was selected
			axisval = 0.0;
		}
		
		return axisval;
	}
}

