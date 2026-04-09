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
import frc.robot.Constants.KickerConstants;
// import frc.robot.Constants.ShooterConstants;
// import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import frc.robot.Configs;
// import frc.robot.Configs.KickerSubsystem;

public class Kicker extends SubsystemBase {

    // Instantiating the hopper to shooter motor
    private SparkFlex KickerLeftMotor = new SparkFlex(KickerConstants.KICKER_LEFT_ID, MotorType.kBrushless);
    private SparkFlex KickerRightMotor = new SparkFlex(KickerConstants.KICKER_RIGHT_ID, MotorType.kBrushless);
    private SparkClosedLoopController kickerLeftController = KickerLeftMotor.getClosedLoopController();
    // private SparkClosedLoopController kickerRightController = KickerRightMotor.getClosedLoopController();

    private final RelativeEncoder kickerLeftEncoder = KickerLeftMotor.getEncoder();
    private final RelativeEncoder kickerRightEncoder = KickerRightMotor.getEncoder();

    private double targetKickerRPM = 0.0;

    private final SysIdRoutine sysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(),
            new SysIdRoutine.Mechanism(
                    voltage -> {
                        KickerLeftMotor.setVoltage(voltage);
                        KickerRightMotor.setVoltage(voltage);
                    },
                    log -> {
                        double kickerLRps = kickerLeftEncoder.getVelocity() / 60.0;
                        double kickerRRps = kickerRightEncoder.getVelocity() / 60.0;

                        double kickerLVolts = KickerLeftMotor.getAppliedOutput() * KickerLeftMotor.getBusVoltage();
                        double kickerRVolts = KickerRightMotor.getAppliedOutput() * KickerRightMotor.getBusVoltage();

                        log.motor("kicker-left")
                                .voltage(Units.Volts.of(kickerLVolts))
                                .angularPosition(Units.Rotations.of(kickerLeftEncoder.getPosition()))
                                .angularVelocity(Units.RotationsPerSecond.of(kickerLRps));

                        log.motor("kicker-right")
                                .voltage(Units.Volts.of(kickerRVolts))
                                .angularPosition(Units.Rotations.of(kickerRightEncoder.getPosition()))
                                .angularVelocity(Units.RotationsPerSecond.of(kickerRRps));
                    },
                    this));

    public Kicker() {
        KickerLeftMotor.configure(Configs.KickerSubsystem.kickerLeftMotorConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
        KickerRightMotor.configure(Configs.KickerSubsystem.kickerRightMotorConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
    }

    public void KickBackwards() {
        // kickerLeftController.setSetpoint(KickerConstants.KICKER_SPEED,
        // ControlType.kMAXMotionVelocityControl);
        kickerLeftController.setSetpoint(KickerConstants.KICKER_REVERSE_RPM_TARGET,
                ControlType.kMAXMotionVelocityControl);
        // kickerRightController.setSetpoint(KickerConstants.KICKER_SPEED,
        // ControlType.kMAXMotionVelocityControl);
    }

    public void Kick() {
        // kickerLeftController.setSetpoint(KickerConstants.KICKER_REVERSE_SPEED,
        // ControlType.kMAXMotionVelocityControl);
        // KickerLeftMotor.set(KickerConstants.KICKER_REVERSE_RPM_TARGET);
        kickerLeftController.setSetpoint(KickerConstants.KICKER_RPM_TARGET, ControlType.kMAXMotionVelocityControl);
        // kickerRightController.setSetpoint(KickerConstants.KICKER_REVERSE_SPEED,
        // ControlType.kMAXMotionVelocityControl);
    }

    public void stopKicking() {
        targetKickerRPM = 0.0;
        kickerLeftController.setSetpoint(KickerConstants.STOP, ControlType.kMAXMotionVelocityControl);
    }

    public Command kickCommand() {

        return new RunCommand(() -> Kick(), this)
                .finallyDo(interrupted -> stopKicking());
    }

    public Command kickBackwardsCommand() {

        return new RunCommand(() -> KickBackwards(), this)
                .finallyDo(interrupted -> stopKicking());
    }

    public Command stopKickingCommand() {
        return new RunCommand(() -> stopKicking(), this);
    }

    public Command sysIdQuasistaticForward() {
        return sysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward);
    }

    public Command sysIdQuasistaticReverse() {
        return sysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse);
    }

    public Command sysIdDynamicForward() {
        return sysIdRoutine.dynamic(SysIdRoutine.Direction.kForward);
    }

    public Command sysIdDynamicReverse() {
        return sysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse);
    }

    public Command runDefaultCommand()
    {
        return stopKickingCommand();
    }

    @Override
    public void periodic() {
        // AdvantageKit Logging
        double kickerLeftRPM = KickerLeftMotor.getEncoder().getVelocity();
        double kickerRightRPM = KickerRightMotor.getEncoder().getVelocity();

        // Kicker wheels speed (RPM).
        Logger.recordOutput("Shooter/KickerLeftRPM", kickerLeftRPM);
        Logger.recordOutput("Shooter/KickerRightRPM", kickerRightRPM);

        // Desired kicker RPM setpoint.
        Logger.recordOutput("Shooter/TargetKickerRPM", targetKickerRPM);

        // Applied voltage to kicker motors
        Logger.recordOutput("Shooter/KickerLeftAppliedVolts",
                KickerLeftMotor.getAppliedOutput() * KickerLeftMotor.getBusVoltage());
        Logger.recordOutput("Shooter/KickerRightAppliedVolts",
                KickerRightMotor.getAppliedOutput() * KickerRightMotor.getBusVoltage());
    }
}
