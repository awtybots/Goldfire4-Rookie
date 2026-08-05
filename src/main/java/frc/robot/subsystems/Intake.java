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
import frc.robot.Configs;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {

    // AdvantageKit logging

    // private SparkFlex IntakeLeftMotor = new SparkFlex(IntakeConstants.INTAKE_LEFT_ID, MotorType.kBrushless);
    // private SparkClosedLoopController IntakeLeftController = IntakeLeftMotor.getClosedLoopController();

  

    public Intake() {
        // IntakeLeftMotor.configure(Configs.IntakeSubsystem.IntakeMotorLeftConfig, ResetMode.kResetSafeParameters,
        //         PersistMode.kPersistParameters);
    }


    // public void runIntake() {
    //     IntakeLeftController.setSetpoint(IntakeConstants.OUTTAKE_RPM,
    //             ControlType.kMAXMotionVelocityControl);
    //     IntakeRightController.setSetpoint(IntakeConstants.OUTTAKE_RPM,
    //             ControlType.kMAXMotionVelocityControl);

    // }

  
    // public Command runIntakeCommand() {
    //     return new RunCommand(() -> runIntake(), this)
    //             .finallyDo(interrupted -> stopIntake());
    // }

    
    @Override
    public void periodic() {
        // AdvantageKit Logging
        // Commanded intake motor percent output.
        // double RightRPM = IntakeRightMotor.getEncoder().getVelocity();
       
        // Logger.recordOutput("IntakeTargetRPM", IntakeConstants.INTAKE_RPM);


    }
}