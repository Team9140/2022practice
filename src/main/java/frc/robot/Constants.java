// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants
{
    public static class OperatorConstants
    {
        public static final int DRIVER_CONTROLLER_PORT = 0;
    }

    public static class IntakeConstants{
        public static final int KP = 0;
        public static final int KI = 0;
        public static final int KD = 0;
        public static final int KS = 0;
        public static final int KV = 0;
        public static final int KA = 0;

        public static final int MOTION_MAGIC_CRUISE_VELOCITY = 0;
        public static final int MOTION_MAGIC_ACCELERATION = 0;

        public static final int SENSOR_TO_MECHANISM_RATIO = 0;

        public static final int STATOR_CURRENT_LIMIT = 0;

        public static final int FORWARD_SOFT_LIMIT_THRESHOLD = 0;
        public static final int REVERSE_SOFT_LIMIT_THRESHOLD = 0;

        public static final int ARM_OUT_POSITION = 0;
        public static final int ARM_IN_POSITION = 0;

        public static final int INTAKE_INTAKE_MOTOR_VOLTAGE = 0;//intakeMotor not zero
        public static final int INTAKE_HOLD_MOTOR_VOLTAGE = 0;//holdMotor always  zero

        public static final int REVERSE_INTAKE_MOTOR_VOLTAGE = 0;//negative
        public static final int REVERSE_HOLD_MOTOR_VOLTAGE = 0;//negative
    }
}
