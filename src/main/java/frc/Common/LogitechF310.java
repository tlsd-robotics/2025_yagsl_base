package frc.Common;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

/** 
 * <p>Creates a Logitech F310 specific {@link Joystick} including methods for every button and axis</p>
 */
public class LogitechF310  {

    private Joystick joy;

//==============================================================================
//=============================== Joystick IDs ===================================
    
    private final int btnA = 1;
    private final int btnB = 2;
    private final int btnX = 3;
    private final int btnY = 4;
    private final int btnL1 = 5;
    private final int btnR1 = 6;   
    private final int btnSelect = 8;       
    private final int btnR3 = 9;
    private final int btnL3 = 10;
    private final int leftXAxis = 0;
    private final int leftYAxis = 1;
    private final int leftTrigger = 2;
    private final int rightTrigger = 3;
    private final int rightXAxis = 4;
    private final int rightYAxis = 5;

//==============================================================================
//============================== Constructor ===================================

    /**
     * Create Logitech F310 Gamepad object
     * @param port
     */
    public LogitechF310(int portID) {
        this.joy = new Joystick(portID);
    }

//==============================================================================
//============================ Create Buttons ==================================

    JoystickButton buttonA = new JoystickButton(joy, btnA);
    JoystickButton buttonB = new JoystickButton(joy, btnB);
    JoystickButton buttonX = new JoystickButton(joy, btnX);
    JoystickButton buttonY = new JoystickButton(joy, btnY);
    JoystickButton buttonL1 = new JoystickButton(joy, btnL1);
    JoystickButton buttonR1 = new JoystickButton(joy, btnR1);
    JoystickButton buttonSelect = new JoystickButton(joy, btnSelect);
    JoystickButton buttonR3 = new JoystickButton(joy, btnR3);
    JoystickButton buttonL3 = new JoystickButton(joy, btnL3);

    POVButton dPadUp = new POVButton(joy, 0);
    POVButton dPadRight = new POVButton(joy, 90);
    POVButton dPadDown = new POVButton(joy, 180);
    POVButton dPadLeft = new POVButton(joy, 270);

//==============================================================================
//================================ Getters =====================================
    /**
     * 
     * @return Gamepad Object
     */
    public Joystick getJoy() {
        return joy;
    }

    /**
     * 
     * @return Returns A button object
     */
    public JoystickButton getButtonA() {
        return buttonA;
    }

    /**
     * 
     * @return Returns B button Object
     */
    public JoystickButton getButtonB() {
        return buttonB;
    }

    /**
     * 
     * @return Returns X button object
     */
    public JoystickButton getButtonX() {
        return buttonX;
    }

    /**
     * 
     * @return Returns Y button object
     */
    public JoystickButton getButtonY() {
        return buttonY;
    }

    /**
     * 
     * @return Returns Left bumper button object
     */
    public JoystickButton getButtonL1() {
        return buttonL1;
    }

    /**
     * 
     * @return Returns right bumper button object
     */
    public JoystickButton getButtonR1() {
        return buttonR1;
    }

    /**
     * 
     * @return Return select button object
     */
    public JoystickButton getButtonSelect() {
        return buttonSelect;
    }

    /**
     * 
     * @return Returns Right joystick click button object
     */
    public JoystickButton getButtonR3() {
        return buttonR3;
    }

    /**
     * 
     * @return Returns Left joystick click button object
     */
    public JoystickButton getButtonL3() {
        return buttonL3;
    }

    /**
     * 
     * @return Returns D-PAD Up button object
     */
    public POVButton getDPadUp() {
        return dPadUp;
    }

    /**
     * 
     * @return Returns D-PAD Right button object
     */
    public POVButton getDPadRight() {
        return dPadRight;
    }

    /**
     * 
     * @return Returns D-PAD Down button
     */
    public POVButton getDPadDown() {
        return dPadDown;
    }

    /**
     * 
     * @return Returns D-PAD Left Button
     */
    public POVButton getDPadLeft() {
        return dPadLeft;
    }

    /**
     * 
     * @return Returns Leftside X axis joystick value
     */
    public double getLeftXAxis() {
        return joy.getRawAxis(leftXAxis);
    }

    /**
     * 
     * @return Returns Leftside Y axis joystick value
     */
    public double getLeftYAxis() {
        return joy.getRawAxis(leftYAxis);
    }

    /**
     * 
     * @return Returns Rightside X axix joystick value
     */
    public double getRightXAxis() {
        return joy.getRawAxis(rightXAxis);
    }

    /**
     * 
     * @return Returns Rightside Y Axis joystick value
     */
    public double getRightYAxis() {
        return joy.getRawAxis(rightYAxis);
    }

    /**
     * 
     * @return Returns left trigger button value. <hr> For future reference, create a button form of the trigger for that mode
     */
    public double getLeftTrigger() {
        return joy.getRawAxis(leftTrigger);
    }

    /**
     * 
     * @return Returns Right trigger button value. <hr> For future reference, create a button form of the trigger for that mode
     */
    public double getRightTrigger() {
        return joy.getRawAxis(rightTrigger);
    }
    

}
