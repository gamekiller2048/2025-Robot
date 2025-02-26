package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {

    private final NetworkTable limelightTable;
    private double             tv, ty, tx, ta, tid, ambiguity;

    public VisionSubsystem() {
        limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
    }

    public double getTX() {
        return tx;
    }

    public double getTY() {
        return ty;
    }

    public double getTV() {
        return tv;
    }

    public double getTA() {
        return ta;
    }

    public double getTID() {
        return tid;
    }

    public double getAmbiguity() {
        return ambiguity;
    }

    /** Safely stop the subsystem from moving */
    public void stop() {
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        tv        = limelightTable.getEntry("tv").getDouble(0);
        tx        = limelightTable.getEntry("tx").getDouble(0);
        ty        = limelightTable.getEntry("ty").getDouble(0);
        ta        = limelightTable.getEntry("ta").getDouble(0);
        ambiguity = limelightTable.getEntry("rawFiducials").getDoubleArray(new double[7])[6];
    }

    @Override
    public String toString() {
        // Create an appropriate text readable string describing the state of the
        // subsystem

        return "";
    }
}