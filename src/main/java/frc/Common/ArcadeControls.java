package frc.Common;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/** Creates a custom Joystick object for our Arcade driverstation */
public class ArcadeControls {

    // Note for future people. I hate the naming for these buttons so I suggest painting on Letters for each button so that
    // it will be A button, B button, rather than Right side 3rd from top button which is just terrible.

    Joystick joy;
//==============================================================================
//=============================== Joystick IDs ===================================
    private final int SMALL_TOP = 3;
    private final int SMALL_BOTTOM = 4;

    private final int RIGHT_TOP = 5;
    private final int RIGHT_MIDDLE_UPPER = 6;
    private final int RIGHT_MIDDLE_LOWER = 7;
    private final int RIGHT_BOTTOM = 8;

    private final int LEFT_TOP = 9;
    private final int LEFT_MIDDLE_UPPER = 10;
    private final int LEFT_MIDDLE_LOWER = 11;
    private final int LEFT_BOTTOM = 12;

    private final int xAxis = 0;
    private final int yAxis = 1;

//==============================================================================
//============================== Constructor ===================================

    public ArcadeControls(int port) {
        this.joy = new Joystick(port);
    }

//==============================================================================
//============================ Create Buttons ==================================

    JoystickButton smallTop = new JoystickButton(joy, SMALL_TOP);
    JoystickButton smallBottom = new JoystickButton(joy, SMALL_BOTTOM);

    JoystickButton rightTop = new JoystickButton(joy, RIGHT_TOP);
    JoystickButton rightMiddleUpper = new JoystickButton(joy, RIGHT_MIDDLE_UPPER);
    JoystickButton rightMiddleLower = new JoystickButton(joy, RIGHT_MIDDLE_LOWER);
    JoystickButton rightBottom = new JoystickButton(joy, RIGHT_BOTTOM);

    JoystickButton leftTop = new JoystickButton(joy, LEFT_TOP);
    JoystickButton leftMiddleUpper = new JoystickButton(joy, LEFT_MIDDLE_UPPER);
    JoystickButton leftMiddleLower = new JoystickButton(joy, LEFT_MIDDLE_LOWER);
    JoystickButton leftBottom = new JoystickButton(joy, LEFT_BOTTOM);

//==============================================================================
//================================ Getters =====================================

    /**
     * @return Joystick Object
     */
    public Joystick getJoy() {
        return joy;
    }

    /**
     * Notice: All positions are relative to change, so names are temporary.
     * @return Far right small top button.
     */
    public JoystickButton getSmallTop() {
        return smallTop;
    }

    /**
     * Notice: All positions are relative to change, so names are temporary.
     * @return Far right small bottom button.
     */
    public JoystickButton getSmallBottom() {
        return smallBottom;
    }

    /**
     * Notice: All positions are relative to change, so names are temporary.
     * @return Right side top button.
     */  
    public JoystickButton getRightTop() {
        return rightTop;
    }

    /**
     * Notice: All positions are relative to change, so names are temporary.
     * @return Right side 2nd from top button.
     */  
    public JoystickButton getRightMiddleUpper() {
        return rightMiddleUpper;
    }

    /**
     * Notice: All positions are relative to change, so names are temporary.
     * @return Right side 3rd from top button.
     */      
    public JoystickButton getRightMiddleLower() {
        return rightMiddleLower;
    }

    /**
     * Notice: All positions are relative to change, so names are temporary.
     * @return Right side bottom button
     */  
    public JoystickButton getRightBottom() {
        return rightBottom;
    }

    /**
     * Notice: All positions are relative to change, so names are temporary.
     * @return Left side top button
     */   
    public JoystickButton getLeftTop() {
        return leftTop;
    }

    /**
     * Notice: All positions are relative to change, so names are temporary.
     * @return Left side 2nd from top button.
     */  
    public JoystickButton getLeftMiddleUpper() {
        return leftMiddleUpper;
    }

    /**
     * Notice: All positions are relative to change, so names are temporary.
     * @return Left side 3rd from top button.
     */  
    public JoystickButton getLeftMiddleLower() {
        return leftMiddleLower;
    }

    /**
     * Notice: All positions are relative to change, so names are temporary.
     * @return Left side bottom button.
     */  
    public JoystickButton getLeftBottom() {
        return leftBottom;
    }

    /**
     * Returns raw X axis value. Bear in mind this will only ever be 1 or -1. 
     * @return X axis joystick value
     */   
    public double getX() {
        return joy.getRawAxis(xAxis);
    }

    /**
     * Returns raw Y axis value. Bear in mind this will only ever be 1 or -1. 
     * @return Y axis joystick value
     */   
    public double getY() {
        return -joy.getRawAxis(yAxis);
    }


//==============================================================================
//============================== Functions =====================================
/**
 * @return Returns true if the Y axis is moved positive or negative.
 */
    public boolean isYAxisActive() {
        return getY() > 0.1 || getY() < -0.1;
    }

    /**
     * @return Returns true if the X axis is moved positive or negative.
     */
    public boolean isXAxisActive() {
        return getX() > 0.1 || getX() < -0.1;
    }

}
