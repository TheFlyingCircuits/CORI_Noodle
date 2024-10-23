// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.FieldElement;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.leds.LEDs;

public class PrepShot extends Command {
    /** Creates a new PrepShot. */

    private Arm arm;
    private Shooter flywheels;
    private Drivetrain drivetrain;
    private Supplier<ChassisSpeeds> translationController;
    private FieldElement target;
    
    public boolean setpointsAreFresh = false;

    /** Command to aim all parts of the robot at the speaker in preparation for a shot.
     *  This command never finishes; instead use the readyToShoot() method to determine when
     *  a shot should be fired. This command also provides the ability to control translation
     *  as the robot aims.
     * 
     *  @param translationController - Supplier that provides chassisSpeeds so that the robot can be controlled while aiming. Set this to null when prepping a shot in auto
     *                                 (this means that the drivetrain will not move at all, only the arm and flywheels.)
     */
    public PrepShot(Drivetrain drivetrain, Arm arm, Shooter flywheels, Supplier<ChassisSpeeds> translationController, LEDs leds, FieldElement target) {
        this.drivetrain = drivetrain;
        this.arm = arm;
        this.flywheels = flywheels;
        this.translationController = translationController;
        this.target = target;

        super.addRequirements(arm, flywheels);
        // ledFeedbackCommand = leds.playAimingAnimationCommand(arm::getErrorDegrees, flywheels::getWorstError, () -> {return 0.;});
        if (translationController != null) {
            super.addRequirements(drivetrain);
            // ledFeedbackCommand = leds.playAimingAnimationCommand(arm::getErrorDegrees, flywheels::getWorstError, drivetrain::getAngleError);
        }
        
        //       This will prob be part of the LED re-write to use progress instead of error?
    }

    private double getVerticalMetersToPivot(Translation3d targetLocation) {
        return targetLocation.getZ() - ArmConstants.pivotHeightMeters;
    }

    private double getHorizontalMetersToPivot(Translation3d targetLocation) {
        Translation2d robotLocation = drivetrain.getPoseMeters().getTranslation();
        double robotToTargetDistance = targetLocation.toTranslation2d().minus(robotLocation).getNorm();
        return robotToTargetDistance + ArmConstants.pivotOffsetMeters;
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

    
    public boolean readyToShoot() {
        boolean ready = setpointsAreFresh && arm.isCloseToTarget() && flywheels.flywheelsAtSetpoints();
        if (translationController != null) {
            ready = ready && drivetrain.isAligned();
        }
        return ready;
    }

    @Override
    public void initialize() {
        setpointsAreFresh = false;
        Logger.recordOutput("shootOnTheMove/aiming", true);
    }

    @Override
    public void execute() {
        // Flywheels
        double leftFlywheelMetersPerSecond = 24;
        double rightFlywheelMetersPerSecond = 19;

        if (target == FieldElement.LOB_TARGET) {
            leftFlywheelMetersPerSecond = 15; // 13 and 9 was too low
            rightFlywheelMetersPerSecond = 10;
        }
        else if (target == FieldElement.AMP) {
            leftFlywheelMetersPerSecond = 12;
            rightFlywheelMetersPerSecond = 12;
        }

        
        flywheels.setLeftFlywheelsMetersPerSecond(leftFlywheelMetersPerSecond);
        flywheels.setRightFlywheelsMetersPerSecond(rightFlywheelMetersPerSecond);

        /* Experiment reveals that simply averaging the requested velocities gives a really solid
         * estimate for the actual exit velocity (i.e. within 1 m/s) TODO: link desmos graph/experimental data.
         * This approach is preferend over actually measuring the exit velocity, because we don't want the arm
         * to go nuts when the note passes through the flywheels and their speed suddenly changes.
         */
        double estimatedExitVelocity = (leftFlywheelMetersPerSecond + rightFlywheelMetersPerSecond) / 2.;

        // Arm & Drivetrain
        double armDesiredDegrees = arm.getDegrees();
        Rotation2d driveDesiredAngle = drivetrain.getPoseMeters().getRotation();
        if (target == FieldElement.SPEAKER) {
            Translation3d shootOnTheMoveTarget = target.getLocation();//getShootOnTheMoveTarget(target.getLocation(), estimatedExitVelocity, false);
            double armDesiredRadians = getGravCompensatedArmDesiredRadians(shootOnTheMoveTarget, estimatedExitVelocity, false);
            armDesiredDegrees = Math.toDegrees(armDesiredRadians);
            driveDesiredAngle = shootOnTheMoveTarget.toTranslation2d().minus(drivetrain.getPoseMeters().getTranslation()).getAngle();
        }
        else if (target == FieldElement.AMP) {
            armDesiredDegrees = 110;
            driveDesiredAngle = Rotation2d.fromDegrees(-90);
            // facing the back of the robot at the amp (not dependent on alliance color)

            //       or maybe just full on auto scoring for the amp?
        }
        else if (target == FieldElement.LOB_TARGET) {
            armDesiredDegrees = 35;
            driveDesiredAngle = target.getLocation().toTranslation2d().minus(drivetrain.getPoseMeters().getTranslation()).getAngle();
            // TODO: try a more sophisticated lob shot with variable arm angle/speed
            //       that takes gravity into account? We tried this briefly at competition,
            //       but ran out of time for more testing, so we setteled on this solution
            //       that's good enough for now. We may want to come back to this if we
            //       want more precise lob shots.
        }
        else if (target == FieldElement.CARPET) {
            // shart
            // Translation3d unitVectorOnCarpet = drivetrain.fieldCoordsFromRobotCoords(new Translation3d(1, 0, 0));
            // Translation3d targetOnCarpet = unitVectorOnCarpet.times(1.2);
            // armDesiredDegrees = getSimpleArmDesiredDegrees(targetOnCarpet);

            armDesiredDegrees = ArmConstants.armMinAngleDegrees;
        }



        arm.setDesiredDegrees(armDesiredDegrees);
        if (translationController != null) {
            if (target == FieldElement.AMP) {
                drivetrain.fieldOrientedDriveOnALine(translationController.get(), FieldElement.AMP.getPose().toPose2d());
            }
            else {
                drivetrain.fieldOrientedDriveWhileAiming(translationController.get(), driveDesiredAngle);
            }
        }

        setpointsAreFresh = true;
    }

    @Override
    public void end(boolean interrupted) {
        setpointsAreFresh = false;
        Logger.recordOutput("shootOnTheMove/aiming", false);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
