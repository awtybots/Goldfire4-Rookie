package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
// import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants.HopperConstants;
import org.littletonrobotics.junction.Logger;

public class Hopper extends SubsystemBase {

    private TalonFX BeltsMotorLeft = new TalonFX(HopperConstants.BELTS_LEFT_ID);
    private TalonFX BeltsMotorRight = new TalonFX(HopperConstants.BELTS_RIGHT_ID);

   
    private final DutyCycleOut dutyCycleRequest = new DutyCycleOut(0);

    public Hopper() {
        TalonFXConfiguration BeltsConfig = new TalonFXConfiguration();
        BeltsConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        BeltsConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; // adjust we have to
        BeltsConfig.CurrentLimits.StatorCurrentLimit = 120.0;
        BeltsConfig.CurrentLimits.SupplyCurrentLimit = 40.0;
        BeltsConfig.Slot0.kP = HopperConstants.p;
        BeltsConfig.Slot0.kI = HopperConstants.i;
        BeltsConfig.Slot0.kD = HopperConstants.d;
        BeltsConfig.Slot0.kS = HopperConstants.s;
        BeltsConfig.Slot0.kV = HopperConstants.v; 
        BeltsConfig.Slot0.kA = HopperConstants.a;
        BeltsMotorLeft.getConfigurator().apply(BeltsConfig);
        BeltsConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; // adjust we have to
        BeltsMotorRight.getConfigurator().apply(BeltsConfig);
        
        BeltsMotorLeft.setControl(new Follower(BeltsMotorRight.getDeviceID(), MotorAlignmentValue.Opposed));


    }

    public void ReverseBelts() {
        // BeltsMotorRight.setControl(velocityRequest.withVelocity(HopperConstants.REVERSE_BELTS_SPEED).withSlot(0));
        BeltsMotorRight.setControl(dutyCycleRequest.withOutput(HopperConstants.REVERSE_BELTS_SPEED));
    }

    public void BeltsToConveyor() {
        // BeltsMotorRight.setControl(velocityRequest.withVelocity(HopperConstants.BELTS_RPS).withSlot(0));
        BeltsMotorRight.setControl(dutyCycleRequest.withOutput(HopperConstants.BELTS_SPEED));
    }

    

    public void stopBelts() {
        BeltsMotorRight.setControl(dutyCycleRequest.withOutput(0.0));
    }


    public Command runBeltsToConveyorCommand() {
        return this.run(() -> {
            BeltsToConveyor();
        }).finallyDo(interrupted -> stopBelts());
    }


    public Command runReverseBeltsCommand() {
        return this.run(() -> {
            ReverseBelts();
        }).finallyDo(interrupted -> stopBelts());
    }

    @Override
    public void periodic() {
        Logger.recordOutput("Belts/DesiredRPS", BeltsMotorRight.getVelocity().getValueAsDouble());
        Logger.recordOutput("Belts/Volts", BeltsMotorRight.getMotorVoltage().getValueAsDouble());
        Logger.recordOutput("Belts/RPS", BeltsMotorRight.getVelocity().getValueAsDouble());
    }
}