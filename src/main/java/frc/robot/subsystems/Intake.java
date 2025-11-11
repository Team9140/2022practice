package frc.robot.subsystems;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;

import static frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase{
    private final TalonFX intakeMotor = new TalonFX(0);
    private final MotionMagicTorqueCurrentFOC motionMagic = new MotionMagicTorqueCurrentFOC(0);
    private final TalonFX  holdMotor = new TalonFX(0);
    private final TalonFX armMovementMotor = new TalonFX(0);

    private Intake(){
        Slot0Configs slot0Configs = new Slot0Configs()
                .withKP(IntakeConstants.KP)
                .withKI(IntakeConstants.KI)
                .withKD(IntakeConstants.KD)
                .withKS(IntakeConstants.KS)
                .withKV(IntakeConstants.KV)
                .withKA(IntakeConstants.KA);

        //have controlled motion, max speed and acceleration limits
        MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs()
                .withMotionMagicCruiseVelocity(IntakeConstants.MOTION_MAGIC_CRUISE_VELOCITY)
                .withMotionMagicAcceleration(IntakeConstants.MOTION_MAGIC_ACCELERATION);

        //set units to radians of physical arm
        FeedbackConfigs feedbackConfigs = new FeedbackConfigs()
                .withSensorToMechanismRatio(IntakeConstants.SENSOR_TO_MECHANISM_RATIO);

        //make sure motor doesn't die with current limits
        CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs()
                .withStatorCurrentLimit(IntakeConstants.STATOR_CURRENT_LIMIT)
                .withStatorCurrentLimitEnable(false);

        //motor direction
        MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs()
                .withInverted(InvertedValue.Clockwise_Positive);

        SoftwareLimitSwitchConfigs softwareLimitSwitchConfigs = new SoftwareLimitSwitchConfigs()
                .withForwardSoftLimitThreshold(IntakeConstants.FORWARD_SOFT_LIMIT_THRESHOLD)
                .withForwardSoftLimitEnable(false)
                .withReverseSoftLimitThreshold(IntakeConstants.REVERSE_SOFT_LIMIT_THRESHOLD)
                .withReverseSoftLimitEnable(false);

        TalonFXConfiguration motorConfigs = new TalonFXConfiguration()
                .withSlot0(slot0Configs)
                .withCurrentLimits(currentLimits)
                .withFeedback(feedbackConfigs)
                .withMotionMagic(motionMagicConfigs)
                .withMotorOutput(motorOutputConfigs)
                .withSoftwareLimitSwitch(softwareLimitSwitchConfigs);

        this.intakeMotor.getConfigurator().apply(motorConfigs);
        this.holdMotor.getConfigurator().apply(motorConfigs);
        this.armMovementMotor.getConfigurator().apply(motorConfigs);
    }

    public Command armIn() {
        return this.runOnce(() -> {
            armMovementMotor.setControl(motionMagic.withPosition(IntakeConstants.ARM_IN_POSITION));
        }).andThen(new WaitUntilCommand(() -> armMovementMotor.getPosition().getValueAsDouble() == IntakeConstants.ARM_IN_POSITION));
    }

    public Command armOut() {
        return this.runOnce(() -> {
            armMovementMotor.setControl(motionMagic.withPosition(IntakeConstants.ARM_OUT_POSITION));
        }).andThen(new WaitUntilCommand(() -> armMovementMotor.getPosition().getValueAsDouble() == IntakeConstants.ARM_OUT_POSITION));
    }

    public Command intake(){
        //armMovementMotor out
        //intakeMotor not zero
        //holdMotor always  zero
        return this.armOut().andThen(this.runOnce(() -> {
            this.intakeMotor.setVoltage(IntakeConstants.INTAKE_INTAKE_MOTOR_VOLTAGE);
            this.holdMotor.setVoltage(IntakeConstants.INTAKE_HOLD_MOTOR_VOLTAGE);
        }));
    }

    public Command off(){
        //armMovementMotor in
        //intakeMotor always zero
        //holdMotor always  zero
        return this.armIn().andThen(this.runOnce(() -> {
            this.intakeMotor.setVoltage(0);
            this.holdMotor.setVoltage(0);
        }));
    }

    public Command reverse(){
        //armMovementMotor out
        //intakeMotor negative
        //holdMotor negative
        return this.armOut().andThen(this.runOnce(() -> {
            this.intakeMotor.setVoltage(IntakeConstants.REVERSE_INTAKE_MOTOR_VOLTAGE);
            this.holdMotor.setVoltage(IntakeConstants.REVERSE_HOLD_MOTOR_VOLTAGE);
        }));
    }
}
