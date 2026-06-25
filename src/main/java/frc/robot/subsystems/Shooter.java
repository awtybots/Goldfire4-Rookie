package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import com.revrobotics.spark.SparkBase.ControlType;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Configs;

public class Shooter extends SubsystemBase {

    // Instantiating the hopper to shooter motor

    private SparkFlex ShooterRight1Motor = new SparkFlex(ShooterConstants.SHOOTER_R1_ID, MotorType.kBrushless);
    private SparkFlex ShooterRight2Motor = new SparkFlex(ShooterConstants.SHOOTER_R2_ID, MotorType.kBrushless);
    private SparkClosedLoopController shooterright1Controller = ShooterRight1Motor.getClosedLoopController();

    private SparkFlex ShooterLeft1Motor = new SparkFlex(ShooterConstants.SHOOTER_L1_ID, MotorType.kBrushless);
    private SparkFlex ShooterLeft2Motor = new SparkFlex(ShooterConstants.SHOOTER_L2_ID, MotorType.kBrushless);
    private SparkClosedLoopController shooterleft1Controller = ShooterLeft1Motor.getClosedLoopController();

    private final RelativeEncoder shooterRight1Encoder = ShooterRight1Motor.getEncoder();
    private final RelativeEncoder shooterRight2Encoder = ShooterRight2Motor.getEncoder();

    private final RelativeEncoder shooterLeft1Encoder = ShooterLeft1Motor.getEncoder();
    private final RelativeEncoder shooterLeft2Encoder = ShooterLeft2Motor.getEncoder();

    private double targetRPM = 0.0;
    private double targetKickerRPM = 0.0;

    private final SysIdRoutine sysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(),
            new SysIdRoutine.Mechanism(
                    voltage -> {
                        ShooterRight1Motor.setVoltage(voltage);
                        ShooterRight2Motor.setVoltage(voltage);
                        ShooterLeft1Motor.setVoltage(voltage);
                        ShooterLeft2Motor.setVoltage(voltage);
                    },
                    log -> {
                        double right1Rps = shooterRight1Encoder.getVelocity() / 60.0;
                        double right2Rps = shooterRight2Encoder.getVelocity() / 60.0;

                        double left1Rps = shooterLeft1Encoder.getVelocity() / 60.0;
                        double left2Rps = shooterLeft2Encoder.getVelocity() / 60.0;

                        double right1Volts = ShooterRight1Motor.getAppliedOutput() * ShooterRight1Motor.getBusVoltage();
                        double right2Volts = ShooterRight2Motor.getAppliedOutput() * ShooterRight2Motor.getBusVoltage();

                        double left1Volts = ShooterLeft1Motor.getAppliedOutput() * ShooterLeft1Motor.getBusVoltage();
                        double left2Volts = ShooterLeft2Motor.getAppliedOutput() * ShooterLeft2Motor.getBusVoltage();

                        log.motor("shooter-right-1")
                                .voltage(Units.Volts.of(right1Volts))
                                .angularPosition(Units.Rotations.of(shooterRight1Encoder.getPosition()))
                                .angularVelocity(Units.RotationsPerSecond.of(right1Rps));

                        log.motor("shooter-right-2")
                                .voltage(Units.Volts.of(right2Volts))
                                .angularPosition(Units.Rotations.of(shooterRight2Encoder.getPosition()))
                                .angularVelocity(Units.RotationsPerSecond.of(right2Rps));

                        log.motor("shooter-left-1")
                                .voltage(Units.Volts.of(left1Volts))
                                .angularPosition(Units.Rotations.of(shooterLeft1Encoder.getPosition()))
                                .angularVelocity(Units.RotationsPerSecond.of(left1Rps));

                        log.motor("shooter-left-2")
                                .voltage(Units.Volts.of(left2Volts))
                                .angularPosition(Units.Rotations.of(shooterLeft2Encoder.getPosition()))
                                .angularVelocity(Units.RotationsPerSecond.of(left2Rps));
                    },
                    this));

    public Shooter() {

        ShooterRight1Motor.configure(Configs.ShooterSubsystem.ShooterRightMotor1Config, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
        ShooterRight2Motor.configure(Configs.ShooterSubsystem.ShooterRightMotor2Config, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        ShooterLeft1Motor.configure(Configs.ShooterSubsystem.ShooterLeftMotor1Config, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
        ShooterLeft2Motor.configure(Configs.ShooterSubsystem.ShooterLeftMotor2Config, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

    }

    public boolean isShooterFast() 
    {
        double right1RPM = ShooterRight1Motor.getEncoder().getVelocity();
        // double right2RPM = ShooterRight2Motor.getEncoder().getVelocity();

        double left1RPM = ShooterLeft1Motor.getEncoder().getVelocity();
        // double left2RPM = ShooterLeft2Motor.getEncoder().getVelocity();
        double avgShooterRPM = (right1RPM + left1RPM) / 2.0;

        // Return true when average shooter RPM is within ERROR_MARGIN of the target
        // speed.
        // Use java.lang.Math (capital M), and compare absolute difference against the
        // error margin.
        return Math.abs(avgShooterRPM - ShooterConstants.SHOOTER_SPEED) <= ShooterConstants.ERROR_MARGIN;
    }

    public double getRPM() {

        return (shooterLeft1Encoder.getVelocity() + shooterLeft2Encoder.getVelocity()
                + shooterRight1Encoder.getVelocity() + shooterRight2Encoder.getVelocity()) / 4.0;
    }

    public double getShooterSetpoint() {

        return ((shooterleft1Controller.getSetpoint() + shooterright1Controller.getSetpoint()) / 2.0);
    }

    public boolean isShooterRunning() {
        double setpoint = Math.abs(getShooterSetpoint());
        if (Math.abs(setpoint - getRPM()) < 100 && setpoint != 0 && setpoint != ShooterConstants.ALLIANCE_IDLE_RPM) {
            SmartDashboard.putBoolean("Shooter Running", true);
            return true;
        }
        else{            
            SmartDashboard.putBoolean("Shooter Running", false);
            return false;
        }
    }

    // Alright so essentially we made a method to make the motor speed up (a few
    // methods below this) and decided to make
    // shoot fuel call that, as well as the kicker motor so we reduce extra code
    public void shootFuel() {
        shooterright1Controller.setSetpoint(ShooterConstants.SHOOTER_SPEED , ControlType.kMAXMotionVelocityControl);
        shooterleft1Controller.setSetpoint(ShooterConstants.SHOOTER_SPEED, ControlType.kMAXMotionVelocityControl);
    }

    public void stopShooting() {
        targetRPM = 0.0;
        shooterright1Controller.setSetpoint(0.0, ControlType.kMAXMotionVelocityControl);
        shooterleft1Controller.setSetpoint(0.0, ControlType.kMAXMotionVelocityControl);
    }



    public void setTargetRPM(double rpm) {
        targetRPM = rpm;
        shooterright1Controller.setSetpoint(rpm, ControlType.kMAXMotionVelocityControl);
        shooterleft1Controller.setSetpoint(rpm, ControlType.kMAXMotionVelocityControl);
    }

    public void ShooterPassing()
    {
        shooterright1Controller.setSetpoint(ShooterConstants.SHOOTER_PASSING_SPEED , ControlType.kMAXMotionVelocityControl);
        shooterleft1Controller.setSetpoint(ShooterConstants.SHOOTER_PASSING_SPEED, ControlType.kMAXMotionVelocityControl);
    }

    public Command setAllianceIdle()
    {
        return new RunCommand(() -> 
        {
            setTargetRPM(ShooterConstants.ALLIANCE_IDLE_RPM);
        }, this);
    }

    public Command setNeutralIdle()
    {
        return new RunCommand(() -> 
        {
            setTargetRPM(ShooterConstants.NEUTRAL_IDLE_RPM);
        }, this);
    }

    public Command setTargetRPMCommand(double rpm) {
        return new RunCommand(() -> setTargetRPM(rpm), this)
                .finallyDo(interrupted -> setTargetRPM(0.0));
    }

    public Command shootFuelCommand() {

        return new RunCommand(() -> shootFuel(), this)
                .finallyDo(interrupted -> stopShooting());
    }

    public Command stopShootingCommand() {
        return new RunCommand(() -> stopShooting(), this);
    }

    public Command SpeedUpShooterCommand() {
        return new RunCommand(() -> shootFuel(), this);
    }

    public Command ShooterPassingCommand() {
        return new RunCommand(() -> ShooterPassing(), this);
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

    @Override
    public void periodic() {
        getRPM();
        // AdvantageKit Logging
        double right1RPM = ShooterRight1Motor.getEncoder().getVelocity();
        double right2RPM = ShooterRight2Motor.getEncoder().getVelocity();

        double left1RPM = ShooterLeft1Motor.getEncoder().getVelocity();
        double left2RPM = ShooterLeft2Motor.getEncoder().getVelocity();

        isShooterRunning();

        // Shooter right wheel speed (RPM).
        Logger.recordOutput("Shooter/Right1RPM", right1RPM);
        Logger.recordOutput("Shooter/Right2RPM", right2RPM);

        // Shooter left wheel speed (RPM).
        Logger.recordOutput("Shooter/Left1RPM", left1RPM);
        Logger.recordOutput("Shooter/Left2RPM", left2RPM);

        // Desired shooter wheel RPM setpoint.
        Logger.recordOutput("Shooter/TargetRPM", targetRPM);
        // Desired kicker RPM setpoint.
        Logger.recordOutput("Shooter/TargetKickerRPM", targetKickerRPM);

        // Applied voltage to right shooter motor.
        Logger.recordOutput("Shooter/Right1AppliedVolts",
                ShooterRight1Motor.getAppliedOutput() * ShooterRight1Motor.getBusVoltage());
        Logger.recordOutput("Shooter/Right2AppliedVolts",
                ShooterRight2Motor.getAppliedOutput() * ShooterRight2Motor.getBusVoltage());

        // Applied voltage to left shooter motor.
        Logger.recordOutput("Shooter/Left1AppliedVolts",
                ShooterLeft1Motor.getAppliedOutput() * ShooterLeft1Motor.getBusVoltage());
        Logger.recordOutput("Shooter/Left2AppliedVolts",
                ShooterLeft2Motor.getAppliedOutput() * ShooterLeft2Motor.getBusVoltage());
    }
}
