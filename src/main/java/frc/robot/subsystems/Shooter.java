package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;


public class Shooter extends SubsystemBase{
    private final double HOOP_HEIGHT = 1;
    private final double ROBOT_HEIGHT = 1;

    private final int YAW_MOTOR_DEVICE_ID = 0;
    private final int PITCH_MOTOR_DEVICE_ID = 0;
    
    private final int YAW_ACCELERATION = 0;
    private final int YAW_CRUISE_VELOCITY = 0;

    private final int YAW_KS = 0;
    private final int YAW_KV = 0;
    private final int YAW_KA = 0;
    private final int YAW_KP = 0;
    private final int YAW_KI = 0;
    private final int YAW_KD = 0;

    private final int PITCH_ACCELERATION = 0;
    private final int PITCH_CRUISE_VELOCITY = 0;

    private final int PITCH_KS = 0;
    private final int PITCH_KV = 0;
    private final int PITCH_KA = 0;
    private final int PITCH_KP = 0;
    private final int PITCH_KI = 0;
    private final int PITCH_KD = 0;

    private final TalonFX yawMotor = new TalonFX(YAW_MOTOR_DEVICE_ID);
    private final TalonFX pitchMotor = new TalonFX(PITCH_MOTOR_DEVICE_ID);

    private Shooter() {
        yawMotor.setPosition(0);
        pitchMotor.setPosition(0);

        MotionMagicConfigs yawMMConfigs = new MotionMagicConfigs()
            .withMotionMagicAcceleration(YAW_ACCELERATION)
            .withMotionMagicCruiseVelocity(YAW_CRUISE_VELOCITY);

        Slot0Configs yawSlotConfigs = new Slot0Configs()
            .withKS(YAW_KS)
            .withKV(YAW_KV)
            .withKA(YAW_KA)
            .withKP(YAW_KP)
            .withKI(YAW_KI)
            .withKD(YAW_KD);

        MotionMagicConfigs pitchMMConfigs = new MotionMagicConfigs()
            .withMotionMagicAcceleration(PITCH_ACCELERATION)
            .withMotionMagicCruiseVelocity(PITCH_CRUISE_VELOCITY);

        Slot0Configs pitchSlotConfigs = new Slot0Configs()
            .withKS(PITCH_KS)
            .withKV(PITCH_KV)
            .withKA(PITCH_KA)
            .withKP(PITCH_KP)
            .withKI(PITCH_KI)
            .withKD(PITCH_KD);

        TalonFXConfiguration yawConfig = new TalonFXConfiguration()
            .withMotionMagic(yawMMConfigs)
            .withSlot0(yawSlotConfigs);
        TalonFXConfiguration pitchConfig = new TalonFXConfiguration()
            .withMotionMagic(pitchMMConfigs)
            .withSlot0(pitchSlotConfigs);

        yawMotor.getConfigurator().apply(yawConfig);
        pitchMotor.getConfigurator().apply(pitchConfig);

        yawMotor.setPosition(0);
        pitchMotor.setPosition(0);
    }

    public Command setYawPosition(double pos) {
        return this.runOnce(() -> {
            yawMotor.setPosition(pos / 2.0 / Math.PI);
        });
    }

    public Command setPitchPosition(double pos) {
        return this.runOnce(() -> {
            pitchMotor.setPosition(pos / 2.0 / Math.PI);
        });
    }

    public Command aimAtHoop(Pose2d currentPosition, Translation2d hoopPosition) {
        double yawAngle = hoopPosition
            .minus(currentPosition.getTranslation())
            .getAngle()
            .getRadians();

        double pitchAngle = Math.atan(
            (HOOP_HEIGHT - ROBOT_HEIGHT) / hoopPosition.getDistance(currentPosition.getTranslation())
            ); 
        
        return this.runOnce(() -> {
            setYawPosition(yawAngle).alongWith(setPitchPosition(pitchAngle));
        });
    }

    public Command shootAtHoop() {
        return this.runOnce(() -> {

        });
    }
}
