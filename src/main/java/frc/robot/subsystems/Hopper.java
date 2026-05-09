package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.Constants.HopperConstants;
import frc.robot.Configs;
import org.littletonrobotics.junction.Logger;

public class Hopper extends SubsystemBase {

    // AdvantageKit logging
    private double TwindexerRightDesiredPercent = 0.0;
    private double TwindexerLeftDesiredPercent = 0.0;

    private double idleCurrent = 3.0f; // Amps <--- Need to find actual value Theoretically should be 0.5A - 3A

    public boolean hasBalls = false; // <--- Based on Power Draw

    // private int six_seven = HopperConstants.six_seven; // <---------- HISTORICAL MONUMENT

    // Instantiates push down and transfer motors
    private SparkFlex TwindexerRightMotor = new SparkFlex(HopperConstants.TWINDEXER_RIGHT_ID, MotorType.kBrushless);
    // private SparkClosedLoopController TwindexerRightController = TwindexerRightMotor.getClosedLoopController(); // idk
                                                                                                                // what
                                                                                                                // this
                                                                                                                // is

    private SparkFlex TwindexerLeftMotor = new SparkFlex(HopperConstants.TWINDEXER_LEFT_ID, MotorType.kBrushless);
    private SparkClosedLoopController TwindexerLeftController = TwindexerLeftMotor.getClosedLoopController(); // idk
                                                                                                              // what
                                                                                                              // this is

    public Hopper() {
        TwindexerRightMotor.configure(Configs.HopperSubsystem.TwindexerRightControllerConfig,
                ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        TwindexerLeftMotor.configure(Configs.HopperSubsystem.TwindexerLeftControllerConfig,
                ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void ReverseHopper() {
        TwindexerLeftDesiredPercent = HopperConstants.REVERSE_TWINDEXER_LEFT_RPM;
        // TwindexerLeftMotor.set(HopperConstants.REVERSE_TWINDEXER_LEFT_RPM);
        TwindexerLeftController.setSetpoint(HopperConstants.REVERSE_TWINDEXER_LEFT_RPM,
                ControlType.kMAXMotionVelocityControl);
        
    }

    public void HopperToShooter() {
        TwindexerLeftDesiredPercent = HopperConstants.REVERSE_TWINDEXER_LEFT_RPM;
        // TwindexerLeftMotor.set(HopperConstants.TWINDEXER_LEFT_RPM);
        TwindexerLeftController.setSetpoint(HopperConstants.REVERSE_TWINDEXER_RIGHT_RPM,
                ControlType.kMAXMotionVelocityControl);

    }

    public boolean checkBalls() // Checks if there are balls in hopper based on output velocity
    {
        return !(Math.abs((TwindexerLeftMotor.getEncoder().getVelocity() + TwindexerRightMotor.getEncoder().getVelocity()) / 2.0 - HopperConstants.REVERSE_TWINDEXER_RIGHT_RPM) < 100);
    }

    public void stopHopper() {
        TwindexerLeftDesiredPercent = 0.0;
        TwindexerLeftController.setSetpoint(0, ControlType.kMAXMotionVelocityControl);
    }

    public Command runDefaultCommand()
    {
        return new RunCommand(() -> stopHopper(), this);
    }

    public Command runHopperToShooterCommand() {
        return new RunCommand(() -> HopperToShooter(), this)
                .finallyDo(interrupted -> stopHopper());
    }

    public Command runReverseHopperCommand() {
        return new RunCommand(() -> ReverseHopper(), this)
                .finallyDo(interrupted -> stopHopper());
    }

    @Override
    public void periodic() {
        
        // AdvantageKit Logging
        // Commanded pushdown motor percent output.
        Logger.recordOutput("Hopper/PushdownDesiredPercent", TwindexerRightDesiredPercent);
        // Commanded transfer motor percent output.
        Logger.recordOutput("Hopper/TransferDesiredPercent", TwindexerLeftDesiredPercent);
        // Applied voltage to pushdown motor.
        Logger.recordOutput("Hopper/TwindexerRightMotor",
                TwindexerRightMotor.getAppliedOutput() * TwindexerRightMotor.getBusVoltage());
        // Applied voltage to transfer motor.
        Logger.recordOutput("Hopper/TwindexerLeftMotor",
                TwindexerLeftMotor.getAppliedOutput() * TwindexerLeftMotor.getBusVoltage());
    }
}
