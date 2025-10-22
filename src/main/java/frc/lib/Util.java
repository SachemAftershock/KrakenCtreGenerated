package frc.lib;

/**
 * Class to hold utility methods to be used throughout the codebase
 */
public class Util {

    /**
     * Applies a deadband to the value with given tolerance
     * 
     * @param value the value to apply the deadband to
     * 
     * @param tolerance minimum value
     * 
     * @return <b> value </b> if greater than <b> tolerance </b>; 0.0 otherwise
     */
    public static double deadband(double value, double tolerance) {
        return Math.abs(value) >= tolerance ? value : 0.0;
    }
    
    //TODO move and test this in place of the other deadband functions, as it it intended to work better (more precisely)
    /** This function provides for zeroing when the value is near zero, and
     *  a rescales so that the truncated deadband then ramps up from zero rather 
     *  than the truncation point (epsilon).  This allows for a very precise and 
     *  continuous funcion from -1 through 1.
     */
    private double deadband(double inValue, double epsilon, double pow){
        double power = (pow <= 0) ? 1 : pow; // fix erroneous "power of" superscript, keep > 0.
        if (Math.abs(inValue) < epsilon ) 
        return 0;  // apply deadband
        else {
        // rescale from epsilon..1 to 0..1
        double sign = ((inValue < 0) ? -1 : 1);
        final double kMinSpeedAvoidMotorSqueal = 0.03;
        double slope = (1-kMinSpeedAvoidMotorSqueal)/(1-epsilon); // >1
        double reScaleLinearValue = (slope * (Math.abs(inValue)-epsilon))+kMinSpeedAvoidMotorSqueal;  // result range 0..1
        // if linear result is too punchy at low end, apply a power > 1.0. Two seems best. To keep linear, set pow to 1.0
        double exponentialSupressionNearZero = sign * Math.pow(reScaleLinearValue,power); // result range 0..1
        return (exponentialSupressionNearZero);
        }
    }

    /**
	* Gets rotational error on [-180, 180]
	* 
    * @param alpha First angle
    
    * @param beta Second Angle
    
	* @return Rotational error
	*/
    public static double rotationalError(double alpha, double beta) {
        double ret = alpha - beta;
        if (ret < -180) {
            ret += 360;
        }
        if (ret > 180) {
            ret -= 360;
        }
        return -ret;
    }

    /**
     * Converts angles from [0,360] to [-180, 180]
     * 
     * @param theta Angle in range [0,360]
     * 
     * @return Angle converted to [-180,180]
     */
    public static double normalizeAngle(double theta) {
        if(theta > 180) {
            theta -= 360;
        } else if(theta < -180) {
            theta += 360;
        }

        return theta;
    }
}