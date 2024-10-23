// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.Supplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.HumanDriver;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.GyroIOPigeon;
import frc.robot.subsystems.drivetrain.SwerveModuleIONeo;
import frc.robot.subsystems.leds.LEDs;
import frc.robot.subsystems.vision.VisionIOPhotonLib;

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
  public LEDs leds;
  public final HumanDriver charlie = new HumanDriver(0);
  public final HumanDriver ben = new HumanDriver(1);

  /**** INITIALIZE SUBSYSTEMS ****/
  public RobotContainer() {
    if (RobotBase.isReal()) {
      drivetrain = new Drivetrain( // fr 0.092041015625, br , 0.0419921875, fl -0.178955078125, bl -0.332763671875
          new GyroIOPigeon(),
          new SwerveModuleIONeo(7, 8, -0.184814453125, 0), 
          new SwerveModuleIONeo(5, 6, 0.044677734375, 3),
          new SwerveModuleIONeo(3, 4, -0.3349609375, 2),
          new SwerveModuleIONeo(1, 2,  0.088134765625, 1),
          new VisionIOPhotonLib()
        );
    }

    leds = new LEDs();
    intake = new Intake();

    // drives I think
    drivetrain.setDefaultCommand(drivetrain.run(() -> {drivetrain.fieldOrientedDrive(charlie.getRequestedFieldOrientedVelocity(), true);}));
    intake.setDefaultCommand(intake.runIntakeCommand(0, 0, 0));

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
    controller.y().onTrue(new InstantCommand(() -> drivetrain.setPoseToVisionMeasurement()).repeatedly().until(drivetrain::seesTag));
  }
  private Command runIntake() {
    return intake.runIntakeCommand(5,5,5);
}

private Command reverseIntake() {
    return intake.runIntakeCommand(-6,-6,-6);
}
  private Command intakeNote() {
    return new ScheduleCommand(leds.playIntakeAnimationCommand(() -> {return drivetrain.getBestNoteLocationFieldFrame().isPresent();}).withName("intake animation"))
        .alongWith(this.runIntake().andThen(new ScheduleCommand(positionNote())));
}

  private Command positionNote() {
    return intake.runIntakeCommand(0,0,-2).withTimeout(3);
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
