package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.VendorWrappers.Kraken;
import frc.robot.VendorWrappers.Neo;

public class Shooter extends SubsystemBase {
    
    /**
     * Linear surface speed of the left set of flywheels.
     */
    public double leftFlywheelsMetersPerSecond = 0.0;
    /**
     * Linear surface speed of the right set of flywheels.
     */
    public double rightFlywheelsMetersPerSecond = 0.0;

    public double leftMotorAppliedVoltage = 0.0;
    public double rightMotorAppliedVoltage = 0.0;
    
    public double leftMotorOutputCurrent = 0.0;
    public double rightMotorOutputCurrent = 0.0;

    public double leftFlywheelRadians = 0.0;
    public double rightFlywheelRadians = 0.0;

    public double leftFlywheelRadiansPerSecond = 0.0;
    public double rightFlywheelRadiansPerSecond = 0.0;

    public double leftFlywheelRadiansPerSecondSquared = 0.0;
    public double rightFlywheelRadiansPerSecondSquared = 0.0;

    public Neo leftMotor;
    public Neo rightMotor;


    public Shooter() {

        leftMotor = new Neo(99);

        rightMotor = new Neo(99);

        configMotors();

    }

    private void configMotors() {
        leftMotor.setInverted(false); //needs to be clockwise positive
        leftMotor.setIdleMode(IdleMode.kCoast);
        leftMotor.setSmartCurrentLimit(120);
        
        rightMotor.setInverted(true); //neds to be counterclockwise positive
        rightMotor.setIdleMode(IdleMode.kCoast);
        rightMotor.setSmartCurrentLimit(120);
    }

    private void setLeftMotorVolts(double volts) {
        leftMotor.setVoltage(volts);
    }

    private void setRightMotorVolts(double volts) {
        rightMotor.setVoltage(volts);
    }

    @Override
    public void periodic() {
        leftFlywheelsMetersPerSecond = leftMotor.getVelocity(); // need to multiply by flywheel circumference in meters
        rightFlywheelsMetersPerSecond = rightMotor.getVelocity(); // need to mutliply by flywheel circumference in meters

        leftMotorAppliedVoltage = leftMotor.getBusVoltage();
        rightMotorAppliedVoltage = rightMotor.getBusVoltage();

        Logger.recordOutput("shooter/leftFlywheelMetersPerSecond", leftFlywheelsMetersPerSecond);
        Logger.recordOutput("rightFlywheelMetersPerSecond", rightFlywheelsMetersPerSecond);
        Logger.recordOutput("leftMotorAppliedVoltage", leftMotorAppliedVoltage);
        Logger.recordOutput("rightMotorAppliedVoltage", rightMotorAppliedVoltage);
    }

    
}
