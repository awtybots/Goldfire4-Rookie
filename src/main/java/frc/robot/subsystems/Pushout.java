package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;
// import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
// import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;

import edu.wpi.first.math.filter.Debouncer;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.Configs;
import edu.wpi.first.wpilibj.Timer;

import static edu.wpi.first.units.Units.Amp;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

public class Pushout extends SubsystemBase {

    // AdvantageKit logging
    // private SparkFlex PushoutMotor = new SparkFlex(PushoutConstants.PUSHOUT_ID, MotorType.kBrushless);
    // private SparkClosedLoopController PushoutController = PushoutMotor.getClosedLoopController();

    // private RelativeEncoder pushoutEncoder = PushoutMotor.getEncoder();

    public Pushout() {
        // PushoutMotor.configure(Configs.PushoutSubsystem.PushoutMotorConfig, ResetMode.kResetSafeParameters,
        //         PersistMode.kPersistParameters);
    }

    public void PushIntake() {
        // PushoutController.setSetpoint(PushoutConstants.PUSHOUT_EXTENDED_POS, ControlType.kMAXMotionPositionControl);
    }
 // change
    
    // public Command PushCommand() {
    //     return this.run(() -> PushIntake())
    //             .finallyDo(interrupted -> StopPushout());

    // }

    @Override
    public void periodic() {
        // AdvantageKit Logging
        // Commanded intake motor percent output.
        // Logger.recordOutput("Pushout/DesiredPercent", desiredPercent);
    }
}
