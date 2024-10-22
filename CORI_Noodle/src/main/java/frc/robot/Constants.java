// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static class OperatorConstants {
        public static final int kDriverControllerPort = 0;
    }

    public static class ArmConstants {


        //TODO: FILL
        /** Distance from the floor to the center of the pivot. This is used for angle calculations for shoot from anywhere. */
        public final static double pivotHeightMeters = Units.inchesToMeters(15);
        
        /** Horizontal distance from the robot center to the pivot center */
        public final static double pivotOffsetMeters = Units.inchesToMeters(-13);

        public final static double armLengthMeters = Units.inchesToMeters(17.73);



        public static final double armMinAngleDegrees = 1.285;
        public static final double armMaxAngleDegrees = 100;

        public final static double armMaxVelDegreesPerSecond = 360.;
        public final static double armMaxAccelDegreesPerSecondSquared = 660.;

        public static final TrapezoidProfile.Constraints constraints = new Constraints(
            armMaxVelDegreesPerSecond,
            armMaxAccelDegreesPerSecondSquared
        );


        /***** REAL CONSTANTS ******/
        public final static double kSArmVolts = 0.005;
        public final static double kGArmVolts = 0.32;
        public final static double kVArmVoltsSecondsPerRadian = 3.1;
        public final static double kAArmVoltsSecondsSquaredPerRadian = 0;

        public final static double kPArmVoltsPerDegree = 0.3;
        public final static double kDArmVoltsSecondsPerDegree = 0.01;
    }

    public static class VisionConstants {

        //TODO: FILL
        public final static Transform3d robotToNoteCamera = new Transform3d(
            new Translation3d(Units.inchesToMeters(-17.75), 0, Units.inchesToMeters(13.75)),
            new Rotation3d(0, Math.toRadians(0), Math.toRadians(0))
        );

        public final static Transform3d robotToShooterCamera = new Transform3d(
            new Translation3d(Units.inchesToMeters(7.125), 0, Units.inchesToMeters(11)),
            new Rotation3d(0, Math.toRadians(-28), 0)
        );
    }

}
