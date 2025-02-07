package frc.robot.commands.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.LoggingCommand;
import frc.robot.subsystems.DriveSubsystem;

public class DriveToTargetCommand extends LoggingCommand {
    private final DriveSubsystem driveSubsystem;

    private final double         targetX;
    private final double         targetY;

    private final PIDController  xController = new PIDController(1.0, 0, 0);
    private final PIDController  yController = new PIDController(1.0, 0, 0);


    public DriveToTargetCommand(double targetX, double targetY, DriveSubsystem driveSubsystem) {
        this.targetX        = targetX;
        this.targetY        = targetY;
        this.driveSubsystem = driveSubsystem;

        Pose2d currentPose  = driveSubsystem.getPose();
        // first rotate to this angle
        double angleInitial = Math.toDegrees(Math.atan2(targetY - currentPose.getY(), targetX - currentPose.getX()));
        // then travel in a straight line

        addRequirements(driveSubsystem);
    }

    @Override
    public void execute() {
        Pose2d currentPose = driveSubsystem.getPose();

        double xError      = targetX - currentPose.getX();
        double yError      = targetY - currentPose.getY();

        double xvel        = xController.calculate(xError);
        double yvel        = yController.calculate(yError);
        System.out.println(currentPose.getX() + " " + currentPose.getY() + " ");
        setArcadeDriveMotorSpeeds(Math.sqrt(xvel * xvel + yvel * yvel), 0, DriveConstants.DRIVE_SCALING_NORMAL);
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
        // TODO: Make into constants
        // finish if robot is close enought to threshold target
        return Math.abs(targetX - driveSubsystem.getPose().getX()) < 0.1 &&
            Math.abs(targetY - driveSubsystem.getPose().getY()) < 0.1; // &&
        // Math.abs(target.getRotation().getDegrees() - driveSubsystem.getPose().getRotation().getDegrees()) < 5;
    }
}
