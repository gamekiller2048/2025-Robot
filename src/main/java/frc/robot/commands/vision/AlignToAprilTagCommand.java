// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.vision;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.LoggingCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;


public class AlignToAprilTagCommand extends LoggingCommand {

    // the target id of the apriltag
    private final int             targetID;

    private final PIDController   headingController = new PIDController(1.0, 0, 0);

    private final DriveSubsystem  driveSubsystem;
    private final VisionSubsystem visionSubsystem;

    /**
     * DriveForTime command drives at the specified heading at the specified speed for the specified
     * time.
     *
     * @param speed in the range -1.0 to +1.0
     * @param driveSubsystem
     */

    public AlignToAprilTagCommand(int targetID, DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem) {

        this.targetID        = targetID;
        this.visionSubsystem = visionSubsystem;
        this.driveSubsystem  = driveSubsystem;


        // Add required subsystems
        addRequirements(driveSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

        logCommandStart();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double tx = visionSubsystem.getTX();

        // TODO: Use PIDController to calculate heading direction
        // TODO: arcadeDrive to turn in correct heading
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {

    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {

        // if the tx is close enough to be aligned
        if (Math.abs(visionSubsystem.getTX()) < VisionConstants.ALIGN_TX_MINIMUM_THRESHOLD)
            return true;
        // TODO: finish if the targetID is not found or no targets
        return false;
    }
}