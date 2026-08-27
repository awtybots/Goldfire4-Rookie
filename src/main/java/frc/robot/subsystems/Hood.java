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

import frc.robot.Constants.HoodConstants;
// import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import frc.robot.Configs;

public class Hood extends SubsystemBase {

    // Instantiating the hopper to shooter motor
    private SparkFlex HoodMotor = new SparkFlex(HoodConstants.HOOD_ID, MotorType.kBrushless);
    private SparkClosedLoopController hoodController = HoodMotor.getClosedLoopController();


    private final RelativeEncoder hoodEncoder = HoodMotor.getEncoder();


    public Hood() {
        HoodMotor.configure(Configs.HoodSubsystem.HoodMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void setHoodPosition(double position) {
        hoodController.setSetpoint(position, ControlType.kMAXMotionPositionControl);
    }

    public Command setHoodPositionCommand(double position) {
            return new RunCommand(() -> setHoodPosition(position), this);
        }

    @Override
    public void periodic() {
    }
}
