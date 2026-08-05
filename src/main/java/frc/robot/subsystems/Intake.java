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
// import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;

import frc.robot.Constants.IntakeConstants;
import frc.robot.Configs;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {

    // AdvantageKit logging
    public SparkFlex IntakeMotorLeft = new SparkFlex(IntakeConstants.INTAKE_LEFT_ID, MotorType.kBrushless);
    public SparkFlex IntakeMotorRight = new SparkFlex(IntakeConstants.INTAKE_RIGHT_ID, MotorType.kBrushless);

    public SparkClosedLoopController IntakeLeftController = IntakeMotorLeft.getClosedLoopController();
    public SparkClosedLoopController IntakeRightController = IntakeMotorRight.getClosedLoopController();

    public Intake() {
        IntakeMotorLeft.configure(Configs.IntakeSubsystem.IntakeMotorLeftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        IntakeMotorRight.configure(Configs.IntakeSubsystem.IntakeMotorRightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }


    public void runIntake() { // right motor follows the left motor, so only need to set the left motor speed
      IntakeLeftController.setSetpoint(IntakeConstants.INTAKE_RPM,
                ControlType.kMAXMotionVelocityControl);
     }

    public void runOuttake() { // right motor follows the left motor, so only need to set the left motor speed
      IntakeLeftController.setSetpoint(IntakeConstants.OUTTAKE_RPM,
                ControlType.kMAXMotionVelocityControl);
     }     

    public Command runIntakeCommand() {
        return new RunCommand(() -> runIntake(), this)
                .finallyDo(interrupted -> stopIntake());
    }

    public Command runOuttakeCommand() {
        return new RunCommand(() -> runOuttake(), this)
                .finallyDo(interrupted -> stopIntake());
    }

    public void stopIntake() {
        IntakeMotorLeft.set(0);
    }
    
    @Override
    public void periodic() {
        // AdvantageKit Logging
        // Commanded intake motor percent output.
        double RightRPM = IntakeMotorRight.getEncoder().getVelocity();
        double LeftRPM = IntakeMotorLeft.getEncoder().getVelocity();

        Logger.recordOutput("IntakeRightRPM", RightRPM);
        Logger.recordOutput("IntakeLeftRPM", LeftRPM);
        Logger.recordOutput("IntakeTargetRPM", IntakeConstants.INTAKE_RPM);
    }
}