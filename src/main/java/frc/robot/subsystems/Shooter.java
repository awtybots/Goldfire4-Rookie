package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.PersistMode;
// import com.revrobotics.spark.ClosedLoopSlot;
// import com.revrobotics.REVLibError;
import com.revrobotics.spark.SparkLowLevel.MotorType;

// import au.grapplerobotics.LaserCan;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.sim.SparkFlexSim;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants.ShooterConstants;
// import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import frc.robot.Configs;

public class Shooter extends SubsystemBase {

    private SparkFlex ShooterLeft1Motor = new SparkFlex(ShooterConstants.SHOOTER_L1_ID, MotorType.kBrushless);
    private SparkClosedLoopController shooterLeft1Controller = ShooterLeft1Motor.getClosedLoopController();

    private SparkFlex ShooterRight1Motor = new SparkFlex(ShooterConstants.SHOOTER_R1_ID, MotorType.kBrushless);
    //private SparkClosedLoopController shooterRight1Controller = ShooterRight1Motor.getClosedLoopController();

    private SparkFlex ShooterRight2Motor = new SparkFlex(ShooterConstants.SHOOTER_R2_ID, MotorType.kBrushless);
    //private SparkClosedLoopController shooterRight2Controller = ShooterRight2Motor.getClosedLoopController();

    private SparkFlex ShooterLeft2Motor = new SparkFlex(ShooterConstants.SHOOTER_L2_ID, MotorType.kBrushless);
    //private SparkClosedLoopController shooterLeft2Controller = ShooterLeft2Motor.getClosedLoopController();

    private final RelativeEncoder shooterLeft1Encoder = ShooterLeft1Motor.getEncoder();

    private static final double SIM_FLYWHEEL_MOI = 0.004;
    private static final DCMotor SIM_GEARBOX = DCMotor.getNeoVortex(4);
    private final FlywheelSim flywheelSim = RobotBase.isSimulation()
            ? new FlywheelSim(LinearSystemId.createFlywheelSystem(SIM_GEARBOX, SIM_FLYWHEEL_MOI, 1.0),
                    SIM_GEARBOX)
            : null;
    private final SparkFlexSim leaderSim = RobotBase.isSimulation()
            ? new SparkFlexSim(ShooterLeft1Motor, DCMotor.getNeoVortex(1))
            : null;
    private final RelativeEncoder shooterRight1Encoder = ShooterRight1Motor.getEncoder();
    private final RelativeEncoder shooterLeft2Encoder = ShooterLeft2Motor.getEncoder();
    private final RelativeEncoder shooterRight2Encoder = ShooterRight2Motor.getEncoder();
    private double targetShooterRPM = 0;

    public double RPMOffset = 0.0;


    public Shooter() {
        ShooterLeft1Motor.configure(Configs.ShooterSubsystem.ShooterMotorLeft1Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        ShooterRight1Motor.configure(Configs.ShooterSubsystem.ShooterMotorRight1Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        ShooterLeft2Motor.configure(Configs.ShooterSubsystem.ShooterMotorLeft2Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        ShooterRight2Motor.configure(Configs.ShooterSubsystem.ShooterMotorRight2Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }



    public void setShooterSpeed(double speed) {
        targetShooterRPM = speed;
        shooterLeft1Controller.setSetpoint(speed, ControlType.kMAXMotionVelocityControl);
    }

    public boolean isShooterFast() {
        return ShooterConstants.ERROR_MARGIN >
            Math.abs(targetShooterRPM - shooterLeft1Encoder.getVelocity());
    }

    public double getRPM() {
        return shooterLeft1Encoder.getVelocity();
    }

    public double getTargetRPM() {
        return targetShooterRPM;
    }

    public void setTargetRPM(double rpm) {
        setShooterSpeed(rpm);
    }

    public boolean isAtSpeed() {
        return targetShooterRPM > 0 && isShooterFast();
    }

    public void stopShooting() {
        stopShooter();
    }

    public void stopShooter() {
        targetShooterRPM = 0;
        shooterLeft1Controller.setSetpoint(0, ControlType.kDutyCycle);
    }

    public Command setShooterSpeedCommand(double speed) {
        return new RunCommand(() -> setShooterSpeed(speed), this).finallyDo(interrupted -> stopShooter());
    }

    public void setShooterSpeedExample() {
        shooterLeft1Controller.setSetpoint(1200, ControlType.kMAXMotionVelocityControl);
    }

    public Command setShooterSpeedExampleCommand() {
        return new RunCommand(() -> setShooterSpeedExample(), this).finallyDo(interrupted -> stopShooter());
    }

    @Override
    public void periodic() {
        Logger.recordOutput("Shooter/TargetRPM", targetShooterRPM);
        Logger.recordOutput("Shooter/ActualRPM", shooterLeft1Encoder.getVelocity());
        Logger.recordOutput("Shooter/RPMOffset", RPMOffset);
        Logger.recordOutput("Shooter/AtSpeed", isAtSpeed());
        Logger.recordOutput("Shooter/RPMError", targetShooterRPM - shooterLeft1Encoder.getVelocity());

        Logger.recordOutput("Shooter/L1/RPM", shooterLeft1Encoder.getVelocity());
        Logger.recordOutput("Shooter/R1/RPM", shooterRight1Encoder.getVelocity());
        Logger.recordOutput("Shooter/L2/RPM", shooterLeft2Encoder.getVelocity());
        Logger.recordOutput("Shooter/R2/RPM", shooterRight2Encoder.getVelocity());

        Logger.recordOutput("Shooter/L1/AppliedVolts", ShooterLeft1Motor.getAppliedOutput() * ShooterLeft1Motor.getBusVoltage());
        Logger.recordOutput("Shooter/R1/AppliedVolts", ShooterRight1Motor.getAppliedOutput() * ShooterRight1Motor.getBusVoltage());
        Logger.recordOutput("Shooter/L2/AppliedVolts", ShooterLeft2Motor.getAppliedOutput() * ShooterLeft2Motor.getBusVoltage());
        Logger.recordOutput("Shooter/R2/AppliedVolts", ShooterRight2Motor.getAppliedOutput() * ShooterRight2Motor.getBusVoltage());

        Logger.recordOutput("Shooter/L1/Current", ShooterLeft1Motor.getOutputCurrent());
        Logger.recordOutput("Shooter/R1/Current", ShooterRight1Motor.getOutputCurrent());
        Logger.recordOutput("Shooter/L2/Current", ShooterLeft2Motor.getOutputCurrent());
        Logger.recordOutput("Shooter/R2/Current", ShooterRight2Motor.getOutputCurrent());
    }

    public void drawFlywheelEnergy(double joules) {
        if (flywheelSim == null || joules <= 0.0) {
            return;
        }
        double w = flywheelSim.getAngularVelocityRadPerSec();
        double remaining = Math.max(0.0, w * w - 2.0 * joules / SIM_FLYWHEEL_MOI);
        flywheelSim.setAngularVelocity(Math.sqrt(remaining));
    }

    @Override
    public void simulationPeriodic() {
        double vbus = RobotController.getBatteryVoltage();
        flywheelSim.setInputVoltage(leaderSim.getAppliedOutput() * vbus);
        flywheelSim.update(0.020);
        leaderSim.iterate(flywheelSim.getAngularVelocityRPM(), vbus, 0.020);
    }
}
