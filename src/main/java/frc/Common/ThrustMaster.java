package frc.Common;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/** 
 * <p>Creates a ThrustMaster specific {@link Joystick} including methods for every button and axis</p>
 */
public class ThrustMaster  {

    Joystick joy;

//==============================================================================
//=============================== Joystick IDs ===================================
    public static final int TRIGGER = 1;
    public static final int MIDDLE = 2;
    public static final int LEFT = 3;
    public static final int RIGHT = 4;

    public static final int LEFT_FRONT_LEFT = 5;
    public static final int LEFT_FRONT_MIDDLE = 6;
    public static final int LEFT_FRONT_RIGHT = 7;
    public static final int LEFT_BACK_RIGHT = 8;
    public static final int LEFT_BACK_MIDDLE = 9;
    public static final int LEFT_BACK_LEFT = 10;

    public static final int RIGHT_FRONT_RIGHT = 11;
    public static final int RIGHT_FRONT_MIDDLE = 12;
    public static final int RIGHT_FRONT_LEFT = 13;
    public static final int RIGHT_BACK_LEFT = 14;
    public static final int RIGHT_BACK_MIDDLE = 15;
    public static final int RIGHT_BACK_RIGHT = 16;

    public static final int xAxis = 1;
    public static final int yAxis = 2;
    public static final int zAxis = 3;
    public static final int throttleAxis = 4;
    public static final int hatXAxis = 5;
    public static final int hatYAxis = 6;


//==============================================================================
//============================== Constructor ===================================

    public ThrustMaster(int port) {
        this.joy = new Joystick(port);
    }

//==============================================================================
//============================ Create Buttons ==================================

    JoystickButton trigger = new JoystickButton(joy, TRIGGER);
    JoystickButton middle = new JoystickButton(joy, MIDDLE);
    JoystickButton left = new JoystickButton(joy, LEFT);
    JoystickButton right = new JoystickButton(joy, RIGHT);
    JoystickButton leftFrontLeft = new JoystickButton(joy, LEFT_FRONT_LEFT);
    JoystickButton leftFrontMiddle = new JoystickButton(joy, LEFT_FRONT_MIDDLE);
    JoystickButton leftFrontRight = new JoystickButton(joy, LEFT_FRONT_RIGHT);
    JoystickButton leftBackRight = new JoystickButton(joy, LEFT_BACK_RIGHT);
    JoystickButton leftBackMiddle = new JoystickButton(joy, LEFT_BACK_MIDDLE);
    JoystickButton leftBackLeft = new JoystickButton(joy, LEFT_BACK_LEFT);
    JoystickButton rightFrontRight = new JoystickButton(joy, RIGHT_FRONT_RIGHT);
    JoystickButton rightFrontMiddle = new JoystickButton(joy, RIGHT_FRONT_MIDDLE);
    JoystickButton rightFrontLeft = new JoystickButton(joy, RIGHT_FRONT_LEFT);
    JoystickButton rightBackLeft = new JoystickButton(joy, RIGHT_BACK_LEFT);
    JoystickButton rightBackMiddle = new JoystickButton(joy, RIGHT_BACK_MIDDLE);
    JoystickButton rightBackRight = new JoystickButton(joy, RIGHT_BACK_RIGHT);



//==============================================================================
//================================ Getters =====================================

    /**
     * @return Joystick Object
     */
    public Joystick getJoy() {
        return joy;
    }

    /**
     * @return Joystick's Trigger button
     */    
    public JoystickButton getTrigger() {
        return trigger;
    }

    /**
     * @return Middle/bottom button on the top of the joystick
     */    
    public JoystickButton getMiddle() {
        return middle;
    }

    /**
     * @return left button on top of joystick
     */    
    public JoystickButton getLeft() {
        return left;
    }

    /**
     * @return Right button on top of joystick
     */    
    public JoystickButton getRight() {
        return right;
    }

    /**
     * @return Return the front left button on the left side of the joystick.
     */    
    public JoystickButton getLeftFrontLeft() {
        return leftFrontLeft;
    }

    /**
     * @return Return the front middle button on the left side of the joystick.
     */    
    public JoystickButton getLeftFrontMiddle() {
        return leftFrontMiddle;
    }

    /**
     * @return Return the front right button on the left side of the joystick.
     */    
    public JoystickButton getLeftFrontRight() {
        return leftFrontRight;
    }

    /**
     * @return Return the back right button on the left side of the joystick.
     */    
    public JoystickButton getLeftBackRight() {
        return leftBackRight;
    }

    /**
     * @return Return the back middle button on the left side of the joystick.
     */    
    public JoystickButton getLeftBackMiddle() {
        return leftBackMiddle;
    }

    /**
     * @return Return the back left button on the left side of the joystick.
     */    
    public JoystickButton getLeftBackLeft() {
        return leftBackLeft;
    }

    /**
     * @return Return the front right button on the right side of the joystick.
     */    
    public JoystickButton getRightFrontRight() {
        return rightFrontRight;
    }

    /**
     * @return Return the front middle button on the right side of the joystick.
     */    
    public JoystickButton getRightFrontMiddle() {
        return rightFrontMiddle;
    }

    /**
     * @return Return the front left button on the right side of the joystick.
     */    
    public JoystickButton getRightFrontLeft() {
        return rightFrontLeft;
    }

    /**
     * @return Return the back left button on the right side of the joystick.
     */    
    public JoystickButton getRightBackLeft() {
        return rightBackLeft;
    }

    /**
     * @return Return the back middle button on the right side of the joystick.
     */    
    public JoystickButton getRightBackMiddle() {
        return rightBackMiddle;
    }

    /**
     * @return Return the back right button on the right side of the joystick.
     */    
    public JoystickButton getRightBackRight() {
        return rightBackRight;
    } 
    
    /**
     * @return Get joystick's X (horizontal) raw axis value
     */    
    public double getX() {
        return joy.getRawAxis(xAxis);
    }

    /**
     * @return Get joystick's Y (vertical) raw axis value
     */    
    public double getY() {
        return joy.getRawAxis(yAxis);
    }

    /**
     * @return Get joystick's Z (twist) raw axis value
     */    
    public double getZ() {
        return joy.getRawAxis(zAxis);
    }

    /**
     * @return Get joystick's throttle value mapped to a range of <code>0 -> 1</code>
     */    
    public double getThrottle() {
        return Utilities.map(joy.getRawAxis(throttleAxis), 1, -1, 0, 1);
    }

    /**
     * Note: Hat or sometimes called POV, is the clicky thumbstick on the top of the joystick. These axis values can be used like a gamepad's dpad
     * @return Get joystick HAT X value. 
     */    
    public double getHatX() {
        return joy.getRawAxis(hatXAxis);
    }

    /**
     * Note: Hat or sometimes called POV, is the clicky thumbstick on the top of the joystick. These axis values can be used like a gamepad's dpad
     * @return Get joystick HAT Y value. 
     */  
    public double getHatY() {
        return joy.getRawAxis(hatYAxis);
    }

}
