// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.FieldElement;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.drivetrain.SwerveModule;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.GyroIOPigeon;
import frc.robot.subsystems.drivetrain.GyroIOSim;
//import frc.robot.subsystems.drivetrain.SwerveModuleIOSim;
import frc.robot.subsystems.drivetrain.SwerveModuleIO;
import frc.robot.subsystems.drivetrain.SwerveModuleIONeo;
import frc.robot.subsystems.leds.LEDs;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.HumanDriver;
import frc.robot.subsystems.Intake;
// import frc.robot.subsystems.leds.LEDs;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonLib;

import edu.wpi.first.wpilibj.util.Color;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
// making drivetrain final was giving error so I removed it
  public Drivetrain drivetrain;
  public Intake intake;
  public Intake indexer;
  public LEDs leds;
  public final HumanDriver charlie = new HumanDriver(0);
  public final HumanDriver ben = new HumanDriver(1);

  /**** INITIALIZE SUBSYSTEMS ****/
public RobotContainer() {
if (RobotBase.isReal()) {
    drivetrain = new Drivetrain(
        new GyroIOPigeon(),
        new SwerveModuleIONeo(1, 2, -0.177978515625, 0),
        new SwerveModuleIONeo(3, 4, 0.33935546875, 1),
        new SwerveModuleIONeo(5, 6, -0.339599609375, 2),
        new SwerveModuleIONeo(7, 8, -0.206787109375, 3),
        new VisionIOPhotonLib()
      );
  }
  // drives I think
  drivetrain.setDefaultCommand(drivetrain.run(() -> {drivetrain.fieldOrientedDrive(charlie.getRequestedFieldOrientedVelocity(), true);}));
  realBindings();
    // Configure the trigger bindings
    realBindings();
  }

  private void realBindings() {
    CommandXboxController controller = charlie.getXboxController();
    CommandXboxController benController = ben.getXboxController();

    controller.rightTrigger()
    .onTrue(
        //intake after note if on other side of the field

        intakeTowardsNote(charlie::getRequestedFieldOrientedVelocity)
    );
    controller.leftTrigger().whileTrue(reverseIntake());
  }
  private Command runIntake() {
    return intake.runIntakeCommand(2,2,2);
}

private Command reverseIntake() {
    return intake.runIntakeCommand(-6,-6,-6);
}
  private Command intakeNote() {
    return new ScheduleCommand(leds.playIntakeAnimationCommand(() -> {return drivetrain.getBestNoteLocationFieldFrame().isPresent();}).withName("intake animation"))
        .alongWith(this.runIntake());
  
}
    /**
     * @param howToDriveWhenNoNoteDetected let's driver have control if the noteCam doesn't see a note
     * @return
     */
  private Command intakeTowardsNote(Supplier<ChassisSpeeds> howToDriveWhenNoNoteDetected) {
    return intakeNote().raceWith(drivetrain.run(() -> {

        // have charlie stay in control when the noteCam doesn't see a note
        if (drivetrain.getBestNoteLocationFieldFrame().isEmpty()) {
            drivetrain.fieldOrientedDrive(howToDriveWhenNoNoteDetected.get(), true);
            return;
        }

        // drive towards the note when the noteCam does see a note.
        drivetrain.driveTowardsNote(drivetrain.getBestNoteLocationFieldFrame().get());
    }));
}
}
