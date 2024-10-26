package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.VendorWrappers.Kraken;
import frc.robot.VendorWrappers.Neo;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;


/**
 * IMPORTANT NOTE TO BENJI:
 * 1. MOTORS HAVE NO OVERHEAT PROTECTION, VOLTAGE WILL NOT STOP BEING APPLIED IF MOTORS GET DANGEOUSLY HOT
 * not so important notes to benji:
 * 2. inversions may be wrong in configMotors(), elaboration in method
 * 3. auto commands were not replicated here
 * 4. idk what the spring control thing was but i did not add that either
 * 5. shooterconstraints i believe are for krakens so PID could be off
*/


public class Shooter extends SubsystemBase {
    
    /**
     * Linear surface speed of the left set of flywheels.
     */
    private double bottomLeftMetersPerSecond = 0.0;
    /**
     * Linear surface speed of the right set of flywheels.
     */
    private double bottomRightMetersPerSecond = 0.0;
    private double topLeftMetersPerSecond = 0.0;
    private double topRightMetersPerSecond = 0.0;

    private double leftSetpointMetersPerSecond = 0.0;
    private double rightSetpointMetersPerSecond = 0.0;

    private Neo bottomLeftMotor;
    private Neo topLeftMotor;
    private Neo bottomRightMotor;
    private Neo topRightMotor;


    /** Feedforward model for an individual set of flywheels. 
     * This assumes that the shooter's flywheel physics is 
     * symmetric on the left and right sides.
     * The feedforward constants for this use a velocity of rotations per second
     * in order to be compatable with WPILib's SimpleMotorFeedforward class.
     */
    private SimpleMotorFeedforward flywheelsFeedforward;



    public Shooter() {

        bottomLeftMotor = new Neo(16);
        
        bottomRightMotor = new Neo(14);

        topLeftMotor = new Neo(17);

        topRightMotor = new Neo(15);

        configMotors();


        flywheelsFeedforward = new SimpleMotorFeedforward(
            ShooterConstants.kSFlywheelsVolts,
            ShooterConstants.kVFlywheelsVoltsSecondsPerMeter,
            ShooterConstants.kAFlywheelsVoltsSecondsSquaredPerMeter
        );

    }

    private void configMotors() {
        bottomLeftMotor.setInverted(false); //needs to be clockwise positive
        bottomLeftMotor.setIdleMode(IdleMode.kCoast);
        bottomLeftMotor.setSmartCurrentLimit(ShooterConstants.currentLimitAmps);
        bottomLeftMotor.setVelocityConversionFactor(ShooterConstants.motorRPMToFlywheelMPS);

        //TODO: conversion factors for encoders
        
        bottomRightMotor.setInverted(true); //neds to be counterclockwise positive
        bottomRightMotor.setIdleMode(IdleMode.kCoast);
        bottomRightMotor.setSmartCurrentLimit(ShooterConstants.currentLimitAmps);
        bottomRightMotor.setVelocityConversionFactor(ShooterConstants.motorRPMToFlywheelMPS);
        
        topLeftMotor.setInverted(true); //needs to be counterclockwise positive
        topLeftMotor.setIdleMode(IdleMode.kCoast);
        topLeftMotor.setSmartCurrentLimit(ShooterConstants.currentLimitAmps);
        topLeftMotor.setVelocityConversionFactor(ShooterConstants.motorRPMToFlywheelMPS);
        
        topRightMotor.setInverted(false); //neds to be clockwise positive
        topRightMotor.setIdleMode(IdleMode.kCoast);
        topRightMotor.setSmartCurrentLimit(ShooterConstants.currentLimitAmps);
        topRightMotor.setVelocityConversionFactor(ShooterConstants.motorRPMToFlywheelMPS);
    }


    public void setLeftFlywheelsMetersPerSecond(double topDesiredMetersPerSecond, double bottomDesiredMetersPerSecond) {

        this.leftSetpointMetersPerSecond = topDesiredMetersPerSecond;

        double bottomFeedforwardOutput = flywheelsFeedforward.calculate(bottomDesiredMetersPerSecond);
        double bottomPOutput = ShooterConstants.kPFlywheelsVoltsSecondsPerMeter *
            (bottomDesiredMetersPerSecond - bottomLeftMetersPerSecond);
        bottomPOutput = MathUtil.clamp(bottomPOutput, -ShooterConstants.pGainLimitVolts, ShooterConstants.pGainLimitVolts);
        bottomLeftMotor.setVoltage(bottomFeedforwardOutput + bottomPOutput);

        double topFeedforwardOutput = flywheelsFeedforward.calculate(topDesiredMetersPerSecond);
        double topPOutput = ShooterConstants.kPFlywheelsVoltsSecondsPerMeter *
            (topDesiredMetersPerSecond - topLeftMetersPerSecond);
        topPOutput = MathUtil.clamp(topPOutput, -ShooterConstants.pGainLimitVolts, ShooterConstants.pGainLimitVolts);
        topLeftMotor.setVoltage(topFeedforwardOutput + topPOutput);
    }

    /**
     * Sets the surface speed of the left set of flywheels.
     * This is theoretically the speed that this side of the note will have when exiting the shooter.
     * @param metersPerSecond - Desired surface speed in meters per second.
     */
    public void setLeftFlywheelsMetersPerSecond(double metersPerSecond) {
        setLeftFlywheelsMetersPerSecond(metersPerSecond, metersPerSecond);
    }

    /**
     * @return - The shooter's left flywheel surface speed in meters per second
     */
    public double getLeftFlywheelsMetersPerSecond() {
        return (bottomLeftMetersPerSecond + topLeftMetersPerSecond)/2.;
    }

    public void setRightFlywheelsMetersPerSecond(double topDesiredMetersPerSecond, double bottomDesiredMetersPerSecond) {

        this.leftSetpointMetersPerSecond = topDesiredMetersPerSecond;

        double bottomFeedforwardOutput = flywheelsFeedforward.calculate(bottomDesiredMetersPerSecond);
        double bottomPOutput = ShooterConstants.kPFlywheelsVoltsSecondsPerMeter *
            (bottomDesiredMetersPerSecond - bottomRightMetersPerSecond);
        bottomPOutput = MathUtil.clamp(bottomPOutput, -ShooterConstants.pGainLimitVolts, ShooterConstants.pGainLimitVolts);
        bottomRightMotor.setVoltage(bottomFeedforwardOutput + bottomPOutput);

        double topFeedforwardOutput = flywheelsFeedforward.calculate(topDesiredMetersPerSecond);
        double topPOutput = ShooterConstants.kPFlywheelsVoltsSecondsPerMeter *
            (topDesiredMetersPerSecond - topRightMetersPerSecond);
        topPOutput = MathUtil.clamp(topPOutput, -ShooterConstants.pGainLimitVolts, ShooterConstants.pGainLimitVolts);
        topRightMotor.setVoltage(topFeedforwardOutput + topPOutput);
    }

    /**
     * Sets the surface speed of the left set of flywheels.
     * This is theoretically the speed that this side of the note will have when exiting the shooter.
     * @param metersPerSecond - Desired surface speed in meters per second.
     */
    public void setRightFlywheelsMetersPerSecond(double metersPerSecond) {
        setRightFlywheelsMetersPerSecond(metersPerSecond, metersPerSecond);
    }

    /**
     * @return - The shooter's right flywheel surface speed in meters per second
     */
    public double getRightFlywheelsMetersPerSecond() {
        return (bottomRightMetersPerSecond + topRightMetersPerSecond) / 2.;
    }

    public Command setFlywheelSurfaceSpeedCommand(double metersPerSecond) {
        return setFlywheelSurfaceSpeedCommand(metersPerSecond, metersPerSecond);
    }

    public Command setFlywheelSurfaceSpeedCommand(double leftMetersPerSecond, double rightMetersPerSecond) {
        return this.run(
            () -> {
                this.setLeftFlywheelsMetersPerSecond(leftMetersPerSecond);
                this.setRightFlywheelsMetersPerSecond(rightMetersPerSecond);});
    }

    public Command setFlywheelSurfaceSpeedCommand(
        double topLeftMetersPerSecond, double topRightMetersPerSecond,
        double bottomLeftMetersPerSecond, double bottomRightMetersPerSecond) {

        return this.run(
            () -> {
                this.setLeftFlywheelsMetersPerSecond(topLeftMetersPerSecond, bottomLeftMetersPerSecond);
                this.setRightFlywheelsMetersPerSecond(topRightMetersPerSecond, bottomRightMetersPerSecond);});
    }

    /**
     * Returns true if both flywheels are spinning within some threshold of their target speeds.
     */
    public boolean flywheelsAtSetpoints() {
        return Math.abs(getLeftFlywheelsMetersPerSecond() - leftSetpointMetersPerSecond) < 1.5
            && Math.abs(getRightFlywheelsMetersPerSecond() - rightSetpointMetersPerSecond) < 1.5;
    }

    public double getWorstError() {
        double errorLeft = Math.abs(getLeftFlywheelsMetersPerSecond() - leftSetpointMetersPerSecond);
        double errorRight = Math.abs(getRightFlywheelsMetersPerSecond() - rightSetpointMetersPerSecond);

        if (Math.abs(errorLeft) > Math.abs(errorRight)) {
            return errorLeft;
        }
        return errorRight;
    }

    @Override
    public void periodic() {
        bottomLeftMetersPerSecond = bottomLeftMotor.getVelocity();
        bottomRightMetersPerSecond = bottomRightMotor.getVelocity();
        topLeftMetersPerSecond = topLeftMotor.getVelocity();
        topRightMetersPerSecond = topRightMotor.getVelocity();
        

        Logger.recordOutput("shooter/bottomLeft/flywheelMetersPerSecond", bottomLeftMetersPerSecond);
        Logger.recordOutput("shooter/bottomLeft/motorVolts", bottomLeftMotor.getAppliedOutput()*12);

        Logger.recordOutput("shooter/bottomRight/flywheelMetersPerSecond", bottomRightMetersPerSecond);
        Logger.recordOutput("shooter/bottomRight/motorVolts", bottomRightMotor.getAppliedOutput()*12);

        Logger.recordOutput("shooter/topLeft/flywheelMetersPerSecond", topLeftMetersPerSecond);
        Logger.recordOutput("shooter/topLeft/motorVolts", topLeftMotor.getAppliedOutput()*12);

        Logger.recordOutput("shooter/topRight/flywheelMetersPerSecond", topRightMetersPerSecond);
        Logger.recordOutput("shooter/topRight/motorVolts", topRightMotor.getAppliedOutput()*12);
    }
}
