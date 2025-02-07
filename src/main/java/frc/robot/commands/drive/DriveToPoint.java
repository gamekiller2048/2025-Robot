package frc.robot.commands.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.LoggingCommand;
import frc.robot.subsystems.DriveSubsystem;

public class DriveToPoint extends LoggingCommand {
    private final DriveSubsystem driveSubsystem;

    private final Pose2d         target;
    private final PIDController  xController     = new PIDController(1.0, 0, 0);
    private final PIDController  yController     = new PIDController(1.0, 0, 0);
    private final PIDController  thetaController = new PIDController(1.0, 0, 0);

    public DriveToPoint(DriveSubsystem driveSubsystem, Pose2d target) {
        this.driveSubsystem = driveSubsystem;
        this.target         = target;

        addRequirements(driveSubsystem);
    }

    @Override
    public void execute() {
        Pose2d currentPose = driveSubsystem.getPose();

        double xError      = target.getX() - currentPose.getX();
        double yError      = target.getY() - currentPose.getY();
        double angleError  = target.getRotation().getDegrees() - currentPose.getRotation().getDegrees();

        double speed       = xController.calculate(xError);
        double rotation    = thetaController.calculate(angleError);

        setArcadeDriveMotorSpeeds(speed, rotation, DriveConstants.DRIVE_SCALING_NORMAL);
    }

    /**
     * Calculate the scaled arcade drive speeds from the passed in values. In arcade mode, the turn
     * is cut in half to help control the robot more consistently.
     *
     * @param speed
     * @param turn
     * @param driveScalingFactor
     */
    private void setArcadeDriveMotorSpeeds(double speed, double turn, double driveScalingFactor) {

        // Cut the spin in half because it will be applied to both sides.
        // Spinning at 1.0, should apply 0.5 to each side.
        turn = turn / 2.0;

        // Keep the turn, and reduce the forward speed where required to have the
        // maximum turn.
        if (Math.abs(speed) + Math.abs(turn) > 1.0) {
            speed = (1.0 - Math.abs(turn)) * Math.signum(speed);
        }

        double leftSpeed  = (speed + turn) * driveScalingFactor;
        double rightSpeed = (speed - turn) * driveScalingFactor;

        driveSubsystem.setMotorSpeeds(leftSpeed, rightSpeed);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(target.getX() - driveSubsystem.getPose().getX()) < 0.1 &&
            Math.abs(target.getY() - driveSubsystem.getPose().getY()) < 0.1 &&
            Math.abs(target.getRotation().getDegrees() - driveSubsystem.getPose().getRotation().getDegrees()) < 5;
    }
}
