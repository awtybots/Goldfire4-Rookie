package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.BeltsConstants;

public class Belts extends SubsystemBase{
    private TalonFX BeltMotor = new TalonFX(BeltsConstants.BELTS_ID);

    private final VelocityVoltage velocityRequest = new VelocityVoltage(0);

    private double goalRPS = 0;

    public Belts()
    {
        TalonFXConfiguration BeltConfig = new TalonFXConfiguration();
        BeltConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        BeltConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        BeltConfig.CurrentLimits.StatorCurrentLimit = 40.0;
        BeltConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        BeltConfig.Slot0.kP = BeltsConstants.p;
        BeltConfig.Slot0.kI = BeltsConstants.i;
        BeltConfig.Slot0.kD = BeltsConstants.d;
        BeltConfig.Slot0.kS = BeltsConstants.s;
        BeltConfig.Slot0.kV = BeltsConstants.v; 
        BeltConfig.Slot0.kA = BeltsConstants.a;
        BeltMotor.getConfigurator().apply(BeltConfig);
    }

    public double RPMToRPS(double RPM)
    {
        return RPM / 60;
    }

    public void RunBelts()
    {
        goalRPS = BeltsConstants.BELTS_RPM;
        // BeltMotor.setControl(velocityRequest.withVelocity(RPMToRPS(BeltsConstants.BELTS_RPM)).withSlot(0));
        BeltMotor.set(1);
    }

    public void RunBeltsReverse()
    {
        goalRPS = BeltsConstants.BELTS_REVERSE_RPM;
        BeltMotor.setControl(velocityRequest.withVelocity(RPMToRPS(BeltsConstants.BELTS_RPM)).withSlot(0));
    }

    public void stopBelts() 
    {
        goalRPS = 0;
        BeltMotor.setControl(velocityRequest.withVelocity(0));
    }

    public Command RunBeltsCommand()
    {
        return this.run(() -> RunBelts()).finallyDo(interrupted -> stopBelts());
    }

    public Command RunBeltsReverseCommand()
    {
        return this.run(() -> RunBeltsReverse()).finallyDo(interrupted -> stopBelts());
    }

    @Override
    public void periodic()
    {
        Logger.recordOutput("Belts/RPS", BeltMotor.getVelocity().getValueAsDouble());
        Logger.recordOutput("Belts/GoalRPS", goalRPS);
    }
}
