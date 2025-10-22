package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.PusherConstants;


public class AlgaePushSubsystem extends SubsystemBase {
    private static AlgaePushSubsystem mInstance;
    private final SparkFlex mPushMotor;
    private final RelativeEncoder mPushMotorEncoder;

    public AlgaePushSubsystem() {
        mPushMotor = new SparkFlex(PusherConstants.kPushmotorID, MotorType.kBrushless);
        mPushMotorEncoder = mPushMotor.getEncoder();
        SparkFlexConfig PushSparkConfig = new SparkFlexConfig();

        PushSparkConfig.idleMode(IdleMode.kBrake);
        mPushMotor.configure(PushSparkConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void initialize() {
        mPushMotorEncoder.setPosition(0);
    }

    public void setPushSpeed(double speed) {
        mPushMotor.set(speed);
    }

    public double getPushPosition() {
        return mPushMotorEncoder.getPosition();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Push Position", getPushPosition());
    }

    public synchronized static AlgaePushSubsystem getInstance() {
        if (mInstance == null) {
            mInstance = new AlgaePushSubsystem();
        }
        return mInstance;
    }
}
