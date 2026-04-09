package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;
// import com.revrobotics.spark.ClosedLoopSlot;
// import com.revrobotics.spark.SparkBase.ControlType;
// import com.revrobotics.REVLibError;
// import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;

import frc.robot.Constants.HopperConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Configs;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {

    // AdvantageKit logging
    private double desiredPercent = 0.0;

    private SparkFlex IntakeLeftMotor = new SparkFlex(IntakeConstants.INTAKE_LEFT_ID, MotorType.kBrushless);
    private SparkClosedLoopController IntakeLeftController = IntakeLeftMotor.getClosedLoopController();

    private SparkFlex IntakeRightMotor = new SparkFlex(IntakeConstants.INTAKE_RIGHT_ID, MotorType.kBrushless);
    private SparkClosedLoopController IntakeRightController = IntakeRightMotor.getClosedLoopController();
  

    public Intake() {
        IntakeLeftMotor.configure(Configs.IntakeSubsystem.IntakeMotorLeftConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
        IntakeRightMotor.configure(Configs.IntakeSubsystem.IntakeMotorRightConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
        // THE RIGHT INTAKE MOTOR IS FOLLOWING THE LEFT ONE!!!
    }

    public void runOuttake() {
        IntakeLeftController.setSetpoint(IntakeConstants.INTAKE_RPM,
                ControlType.kMAXMotionVelocityControl);

    }

    public void runIntake() {
        IntakeLeftController.setSetpoint(IntakeConstants.OUTTAKE_RPM,
                ControlType.kMAXMotionVelocityControl);
    }

    public void stopIntake() {
        desiredPercent = 0.0;
        IntakeLeftMotor.set(0);
    }

    public Command runIntakeCommand() {
        return new RunCommand(() -> runIntake(), this)
                .finallyDo(interrupted -> stopIntake());
    }

    public Command runOuttakeCommand() {
        return new RunCommand(() -> runOuttake(), this)
                .finallyDo(interrupted -> stopIntake());
    }

    public Command stopIntakeCommand() {
        return new RunCommand(() -> stopIntake(), this);
    }

    public Command runDefaultCommand()
    {
        return stopIntakeCommand();
    }

    @Override
    public void periodic() {
        // AdvantageKit Logging
        // Commanded intake motor percent output.
        double RightRPM = IntakeRightMotor.getEncoder().getVelocity();
        double LeftRPM = IntakeLeftMotor.getEncoder().getVelocity();

        Logger.recordOutput("Intake/DesiredPercent", desiredPercent);
        // Applied voltage to intake motor.
        Logger.recordOutput("Intake/AppliedVolts", IntakeLeftMotor.getAppliedOutput() * IntakeLeftMotor.getBusVoltage());
        Logger.recordOutput("IntakeRightRPM", RightRPM);
        Logger.recordOutput("IntakeLeftRPM", LeftRPM);
        Logger.recordOutput("IntakeTargetRPM", IntakeConstants.INTAKE_RPM);


    }
}