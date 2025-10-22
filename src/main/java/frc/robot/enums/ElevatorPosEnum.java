package frc.robot.enums;

public enum ElevatorPosEnum {
    eStowed(0.33),
    eReceive(1.0),
    eL1(2.46),
    eL2(2.62), 
    eL3(4.02);

    private double mElevatorHeight; 

    ElevatorPosEnum(double height) {
        this.mElevatorHeight = height;
    }

    public double getElevatorHeightFromEnum() {
        return mElevatorHeight;
    }
}
