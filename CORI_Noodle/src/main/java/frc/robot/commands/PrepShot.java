// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmConstants;

public class PrepShot extends Command {
    /** Creates a new PrepShot. */
    public PrepShot() {
        // Use addRequirements() here to declare subsystem dependencies.
    }

    private double getVerticalMetersToPivot(Translation3d targetLocation) {
        return targetLocation.getZ() - ArmConstants.pivotHeightMeters;
    }

    private double getHorizontalMetersToPivot(Translation3d targetLocation) {
        return 0;
        // Translation2d robotLocation = drivetrain.getPoseMeters().getTranslation();
        // double robotToTargetDistance = targetLocation.toTranslation2d().minus(robotLocation).getNorm();
        // return robotToTargetDistance + ArmConstants.pivotOffsetMeters;
    }

    private double getVerticalMetersToFlywheels(Translation3d targetLocation, double armAngleRadians) {
        return getVerticalMetersToPivot(targetLocation) - (ArmConstants.armLengthMeters * Math.sin(armAngleRadians));
    }

    private double getHorizontalMetersToFlywheels(Translation3d targetLocation, double armAngleRadians) {
        return getHorizontalMetersToPivot(targetLocation) - (ArmConstants.armLengthMeters * Math.cos(armAngleRadians));
    }

    /** Calculates the angle the arm would aim at when pointing directly at the target. */
    private double getSimpleArmDesiredDegrees(Translation3d targetLocation) {
        double verticalDistance = getVerticalMetersToPivot(targetLocation);
        double horizontalDistance = getHorizontalMetersToPivot(targetLocation);
        double radians = Math.atan2(verticalDistance, horizontalDistance); // prob don't need arctan2 here, regular arctan will do.
        return Math.toDegrees(radians);
    }

    /**
     * Gets the angle that the shooter needs to aim at in order for a note to hit the target
     * This accounts for distance and gravity.
     * @return - Angle in degrees, with 0 being straight forward and a positive angle being pointed upwards.
    */
    private double getGravCompensatedArmDesiredRadians(Translation3d targetLocation, double exitVelocityMetersPerSecond, boolean isLobShot) {
        //see https://www.desmos.com/calculator/jhuanigvbs
        double g = 9.81;
        double verticalDistance = getVerticalMetersToPivot(targetLocation);
        double horizontalDistance = getHorizontalMetersToPivot(targetLocation);
        double armAngleRadians = 0;

        // TODO: documentation for taking arm length into account
        for (int approximationCount = 0; approximationCount < 4; approximationCount += 1) {
            double a = (g*g)/4.;
            double b = (verticalDistance * g) - (exitVelocityMetersPerSecond * exitVelocityMetersPerSecond);
            double c = (horizontalDistance * horizontalDistance) + (verticalDistance * verticalDistance);

            double tShort = Math.sqrt((-b - Math.sqrt((b*b) - (4*a*c)))/(2*a));
            double tLong =  Math.sqrt((-b + Math.sqrt((b*b) - (4*a*c)))/(2*a));
            double timeToImpact = tShort; // seconds
            if (isLobShot) {
                timeToImpact = tLong;
            }

            double horizontalVelocity = horizontalDistance / timeToImpact;
            double verticalVelocity = (verticalDistance / timeToImpact) + (0.5*g*timeToImpact);
            armAngleRadians = Math.atan2(verticalVelocity, horizontalVelocity);

            // update for next iteration.
            // TODO: should I take the note length into account for considering the distance it travels when in free fall?
            verticalDistance = getVerticalMetersToFlywheels(targetLocation, armAngleRadians);
            horizontalDistance = getHorizontalMetersToFlywheels(targetLocation, armAngleRadians);
        }
        return armAngleRadians;
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {}

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return false;
    }
}
