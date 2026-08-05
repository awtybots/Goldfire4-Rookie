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

import frc.robot.Configs;
import org.littletonrobotics.junction.Logger;

public class Hopper extends SubsystemBase {


    // public void HopperToShooter() {
    //     TwindexerLeftDesiredPercent = HopperConstants.REVERSE_TWINDEXER_LEFT_RPM;
    //     // TwindexerLeftMotor.set(HopperConstants.TWINDEXER_LEFT_RPM);
    //     TwindexerLeftController.setSetpoint(HopperConstants.REVERSE_TWINDEXER_RIGHT_RPM,
    //             ControlType.kMAXMotionVelocityControl);

    // }

   

    // public Command runHopperToShooterCommand() {
    //     return new RunCommand(() -> HopperToShooter(), this)
    //             .finallyDo(interrupted -> stopHopper());
    // }


    @Override
    public void periodic() {
        
        // AdvantageKit Logging
        // Commanded pushdown motor percent output.
        // Logger.recordOutput("Hopper/PushdownDesiredPercent", TwindexerRightDesiredPercent);
    }
}
