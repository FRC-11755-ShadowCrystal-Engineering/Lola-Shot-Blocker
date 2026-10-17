package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Dozer extends SubsystemBase {
    enum DozerState {
        STARTING,
        SHOTBLOCK,
        DOWN
    };
                                                                                            
    PIDController armPID = new PIDController(1, 0, 0);
    ArmFeedforward armFF = new ArmFeedforward(0, 0.5, 0);
    DozerState dozerState = DozerState.STARTING;

    SparkMax dozerSparkMax = new SparkMax(4, MotorType.kBrushless);
    RelativeEncoder dozerEncoder = dozerSparkMax.getEncoder();

    public Dozer() {
        SparkMaxConfig sparkConfig = (SparkMaxConfig)(new SparkMaxConfig()
            .voltageCompensation(12)
            .smartCurrentLimit(40, 40));
        EncoderConfig encoderConfig = sparkConfig.encoder
            .positionConversionFactor(2*Math.PI*(1.0/75.0))
            .velocityConversionFactor(2*Math.PI*(1.0/75.0)/60.0);

        dozerSparkMax.configure(sparkConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

        dozerEncoder.setPosition(Math.PI + 0.1);
        armPID.setSetpoint(Math.PI + 0.1);

        SmartDashboard.putNumber("ArmPosition", 0);
    }

    @Override 
    public void periodic() {
        dozerSparkMax.setVoltage(armPID.calculate(
            dozerEncoder.getPosition()) + 
            armFF.calculate(armPID.getSetpoint(), 
            0));
    }

    public Command toggleArmSetpoint() {
        return new InstantCommand(() -> {
            switch (dozerState) {
                case STARTING: {
                    dozerState = DozerState.SHOTBLOCK;
                    armPID.setSetpoint(Math.PI + 0.1);
                    break;
                }
                case SHOTBLOCK: {
                    dozerState = DozerState.DOWN;
                    armPID.setSetpoint(-0.1);
                    break;
                }
                case DOWN: {
                    dozerState = DozerState.SHOTBLOCK;
                    armPID.setSetpoint(Math.PI/2);
                    break;
                }
            }
        });
    }
}
