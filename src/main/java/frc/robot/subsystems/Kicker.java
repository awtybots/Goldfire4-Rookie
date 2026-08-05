package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.units.Units;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.PersistMode;
// import com.revrobotics.spark.ClosedLoopSlot;
// import com.revrobotics.REVLibError;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.ControlType;
// import frc.robot.Constants.ShooterConstants;
// import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import frc.robot.Configs;
// import frc.robot.Configs.KickerSubsystem;

public class Kicker extends SubsystemBase {

    // Instantiating the hopper to shooter motor
    // private SparkFlex KickerLeftMotor = new SparkFlex(KickerConstants.KICKER_LEFT_ID, MotorType.kBrushless);
    // private SparkClosedLoopController kickerLeftController = KickerLeftMotor.getClosedLoopController();

    // private final RelativeEncoder kickerLeftEncoder = KickerLeftMotor.getEncoder();



    public Kicker() {
        // KickerRightMotor.configure(Configs.KickerSubsystem.kickerRightMotorConfig, ResetMode.kResetSafeParameters,
        //         PersistMode.kPersistParameters);
    }


    public void Kick() {
        // kickerLeftController.setSetpoint(KickerConstants.KICKER_RPM_TARGET, ControlType.kMAXMotionVelocityControl);
    }

    // public Command kickCommand() {

    //     return new RunCommand(() -> Kick(), this)
    //             .finallyDo(interrupted -> stopKicking());
    // }


    @Override
    public void periodic() {
        // AdvantageKit Logging
        // double kickerLeftRPM = KickerLeftMotor.getEncoder().getVelocity();
        // Logger.recordOutput("Shooter/KickerRightAppliedVolts",
        //         KickerRightMotor.getAppliedOutput() * KickerRightMotor.getBusVoltage());
    }
}
