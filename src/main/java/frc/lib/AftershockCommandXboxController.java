package frc.lib;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class AftershockCommandXboxController extends CommandXboxController {

    public AftershockCommandXboxController(final int port) {
        super(port);
    }

    public boolean getDPadPressed() {
        return this.getHID().getPOV() != -1;
    }

    /**
     * Gets the currently pressed angle on the D-Pad
     * 
     * @return degree measure of the D-Pad Button Pressed (-1 if not pressed)
     */
    public int getDPadAngle() {
        return this.getHID().getPOV();
    }

    /**
     * Gets if Up on the D-Pad is pressed
     * 
     * @return whether the D-Pad angle is 0deg
     */
    public boolean getDPadUp() {
        return this.getHID().getPOV() == DPadDirection.eUp.getAngle();
    }

    /**
     * Gets if Up-Right on the D-Pad is pressed
     * 
     * @return whether the D-Pad angle is 45deg
     */
    public boolean getDPadUpRight() {
        return this.getHID().getPOV() == DPadDirection.eUpRight.getAngle();
    }

    /**
     * Gets if Right on the D-Pad is pressed
     * 
     * @return whether the D-Pad angle is 90deg
     */
    public boolean getDPadRight() {
        return this.getHID().getPOV() == DPadDirection.eRight.getAngle();
    }

    private boolean pressed = false;
    public boolean getDPadRightPressed(){
        if(getDPadRight()){
            if(pressed){
                pressed = false;
            }
            else{
                pressed = true;
            }
        }
        return pressed;
    }

    /**
     * Gets if Down-Right on the D-Pad is pressed
     * 
     * @return whether the D-Pad angle is 135deg
     */
    public boolean getDPadDownRight() {
        return this.getHID().getPOV() == DPadDirection.eDownRight.getAngle();
    }

    /**
     * Gets if Down on the D-Pad is pressed
     * 
     * @return whether the D-Pad angle is 180deg
     */
    public boolean getDPadDown() {
        return this.getHID().getPOV() == DPadDirection.eDown.getAngle();
    }

    /**
     * Gets if Down-Left on the D-Pad is pressed
     * 
     * @return whether the D-Pad angle is 225deg
     */
    public boolean getDPadDownLeft() {
        return this.getHID().getPOV() == DPadDirection.eDownLeft.getAngle();
    }

    /**
     * Gets if Left on the D-Pad is pressed
     * 
     * @return whether the D-Pad angle is 270deg
     */
    public boolean getDPadLeft() {
        return this.getHID().getPOV() == DPadDirection.eLeft.getAngle();
    }

    /**
     * Gets if Up-Left on the D-Pad is pressed
     * 
     * @return whether the D-Pad angle is 315deg
     */
    public boolean getDPadUpLeft() {
        return this.getHID().getPOV() == DPadDirection.eUpLeft.getAngle();
    }

    /**
     * Lookup Table for matching direction of D-Pad on Xbox Controller pressed to an angle measure
     * 
     * @author Shreyas Prasad
     */
    private enum DPadDirection {
        eUp(0), eUpRight(45), eRight(90), eDownRight(135), eDown(180),
        eDownLeft(225), eLeft(270), eUpLeft(315);

        private final int angle;

        private DPadDirection(int angle) {
            this.angle = angle;
        }

        /**
         * Gets angle for D-Pad Direction Pressed
         * 
         * @return angle [0,360) corresponding to the appropriate 45deg interval on the D-Pad
         */
        private int getAngle() {
            return this.angle;
        }
    }

}
