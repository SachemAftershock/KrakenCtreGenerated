package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import au.grapplerobotics.interfaces.LaserCanInterface.Measurement;
import au.grapplerobotics.interfaces.LaserCanInterface.RangingMode;
import au.grapplerobotics.interfaces.LaserCanInterface.RegionOfInterest;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ElevatorConstants;
import frc.robot.enums.ElevatorPosEnum;
import frc.lib.PID;

public class ElevatorSubsystem extends SubsystemBase {
    private static ElevatorSubsystem mInstance;

    private SparkFlex mElevatorMotor;
    private static SparkFlex mGrappleMotor;

    private RelativeEncoder mGrappleMotorEncoder;

    private ElevatorPosEnum mCurrentElevatorState = ElevatorPosEnum.eStowed;
    private ElevatorPosEnum mDesiredElevatorState = ElevatorPosEnum.eStowed;

    private DigitalInput mElevatorBottomLimitSwitch;
    private DigitalInput mElevatorTopLimitSwitch;

    private static LaserCan mElevatorLaserCan;
    private static LaserCan mGrappleLaserCan;

    private SparkFlexConfig ElevatorSparkConfig;
    private SparkFlexConfig GrappleSparkConfig;

    private PID ElevatorPID;
    private boolean nudging;
    private double nudgeadd;

    public ElevatorSubsystem() {
        mElevatorMotor = new SparkFlex(ElevatorConstants.kElevatorMotorID, MotorType.kBrushless);
        mGrappleMotor = new SparkFlex(ElevatorConstants.kGrappleMotorID, MotorType.kBrushless);

        mElevatorTopLimitSwitch = new DigitalInput(ElevatorConstants.kTopLimitSwitchID);
        mElevatorBottomLimitSwitch = new DigitalInput(ElevatorConstants.kBottomLimitSwitchID);

        mElevatorLaserCan = new LaserCan(ElevatorConstants.kElevatorLaserCanID);
        mGrappleLaserCan = new LaserCan(ElevatorConstants.kGrappleLaserCanID);

        mGrappleMotorEncoder = mGrappleMotor.getEncoder();

        ElevatorSparkConfig = new SparkFlexConfig();
        GrappleSparkConfig = new SparkFlexConfig();

        ElevatorPID = new PID();

        double[] kPIDVals = { 0.9, 0.0, 0.0 };
        ElevatorPID.start(kPIDVals);

        ElevatorSparkConfig.idleMode(IdleMode.kBrake);
        GrappleSparkConfig.idleMode(IdleMode.kBrake);
        mElevatorMotor.configure(ElevatorSparkConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        mGrappleMotor.configure(GrappleSparkConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void initialize() {
        CalibrateLaserCan();
        mGrappleMotorEncoder.setPosition(0);
    }

    public void CalibrateLaserCan() {
        try {
            mGrappleLaserCan.setRegionOfInterest(new RegionOfInterest(ElevatorConstants.kROIx, ElevatorConstants.kROIy,
                    ElevatorConstants.kROIw, ElevatorConstants.kROIh));
        } catch (ConfigurationFailedException e) {
            System.out.println("Failed to configure ROI for GrappleLaserCan");
            e.printStackTrace();
        }

        try {
            mGrappleLaserCan.setRangingMode(RangingMode.SHORT);
        } catch (ConfigurationFailedException e) {
            System.out.println("Failed to set ranging mode for GrappleLaserCan");
            e.printStackTrace();
        }

        try {
            mElevatorLaserCan.setRegionOfInterest(new RegionOfInterest(ElevatorConstants.kROIx, ElevatorConstants.kROIy,
                    ElevatorConstants.kROIw, ElevatorConstants.kROIh));
        } catch (ConfigurationFailedException e) {
            System.out.println("Failed to configure ROI for ElevatorLaserCan");
            e.printStackTrace();
        }

        try {
            mElevatorLaserCan.setRangingMode(RangingMode.SHORT);
        } catch (ConfigurationFailedException e) {
            System.out.println("Failed to set ranging mode for ElevatorLaserCan");
            e.printStackTrace();
        }
    }

    /**
     * Checks if the coral is in the grapple with a LaserCan
     * 
     * @return true if coral is in the grapple
     */
    public boolean isCoralIn() {
        Measurement measurementCoral = mGrappleLaserCan.getMeasurement();
        if (measurementCoral != null) {
            SmartDashboard.putBoolean("Is Coral in",
                    measurementCoral.distance_mm < ElevatorConstants.kCoralInLaserCanDistance);
            if (measurementCoral.distance_mm < ElevatorConstants.kCoralInLaserCanDistance) {
                return true;
            }
        }
        return false;
    }

    private static double MMtoFeet(double mm) {
        return mm / 25.4 / 12.0;
    }

    /**
     * 
     * @return distance from lidar to bottom of elevator in mm
     */
    public static double getElevatorHeight() {
        Measurement measurement = mElevatorLaserCan.getMeasurement();
        if (measurement != null) {
            return MMtoFeet(measurement.distance_mm);
        }
        return -1;
    }

    public void setElevatorHeight(double targetHeight) {
        double currentHeight = getElevatorHeight();

        // Check if valid height first
        // Returns -1 if issues, only spin motors when valid behavior
        if (currentHeight < 0)
            return;

        double speed = ElevatorPID.update(currentHeight, targetHeight);

        // System.out.println("Current Height: " + getElevatorHeight());
        // System.out.println("Target Height: " + Targetheight);

        if (Math.abs(currentHeight - targetHeight) <= ElevatorConstants.kTolerance) {
            mCurrentElevatorState = mDesiredElevatorState;
            speed = 0.0;
        }

        speed = MathUtil.clamp(speed, -ElevatorConstants.kMaxSpeed, ElevatorConstants.kMaxSpeed);
        setElevatorSpeed(-speed);
    }

    /**
     * Set the Elevator state eRecive, L1, eL2, eL3
     * 
     * @param ElevatorState
     */
    public void setDesiredElevatorState(ElevatorPosEnum ElevatorState) {
        mDesiredElevatorState = ElevatorState;
    }

    /**
     * if the elevator starts to differ from set heights nudge the height up a
     * little
     * 
     * @param millimeters, millimiter nudge diffrence
     */
    public void nudgeElevatorHeight(double feet) {
        nudgeadd = feet;
        nudging = true;
    }

    /**
     * Sets the grapple motor to a constant speed
     */
    public void setGrappleSpeed(double speed) {
        mGrappleMotor.set(speed);
    }

    /**
     * 
     * @return current state of the elevator eRecive, L1, eL2, eL3
     */
    public ElevatorPosEnum getElevatorState() {

        return mCurrentElevatorState;
    }

    /**
     * Sets the speed of the elevator motor to a constant speed
     * 
     * @param speed
     */
    public void setElevatorSpeed(double speed) {
        if (!mElevatorBottomLimitSwitch.get() && speed < 0) {
            speed = 0.0;
        }

        if (!mElevatorTopLimitSwitch.get() && speed > 0) {
            speed = 0.0;
        }

        mElevatorMotor.set(speed);
    }

    @Override
    public void periodic() {
        setElevatorHeight(mDesiredElevatorState.getElevatorHeightFromEnum());

        if (nudging) {
            setElevatorSpeed(nudgeadd);
            nudging = false;
        }
    }

    public synchronized static ElevatorSubsystem getInstance() {
        if (mInstance == null) {
            mInstance = new ElevatorSubsystem();
        }
        return mInstance;
    }
}
