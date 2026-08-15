package frc.robot;

import com.revrobotics.spark.config.SparkFlexConfig;

import frc.robot.Constants.IntakeConstants;

import com.revrobotics.spark.config.SparkBaseConfig.*;

import com.ctre.phoenix6.swerve.utility.WheelForceCalculator.Feedforwards;
// import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;

public final class Configs 
{
        public static final class IntakeSubsystem {
                
            public static final SparkFlexConfig IntakeMotorLeftConfig = new SparkFlexConfig();
            public static final SparkFlexConfig IntakeMotorRightConfig = new SparkFlexConfig();
            // public static final SparkFlexConfig IntakeRightMotorConfig = new SparkFlexConfig();

                static {

                        IntakeMotorLeftConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(40).voltageCompensation(12);
                        IntakeMotorRightConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(40).voltageCompensation(12).follow(IntakeConstants.INTAKE_LEFT_ID, true);



                        IntakeMotorLeftConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                            // Set PID values for position control. We don't need to pass a closed
                            // loop slot, as it will default to slot 0.
                            .p(IntakeConstants.p)
                            .i(IntakeConstants.i)
                            .d(IntakeConstants.d)
                            .outputRange(-1, 1)
                            .feedForward
                            .kS(IntakeConstants.s)
                            .kV(IntakeConstants.v)
                            .kA(IntakeConstants.a)
                            ;

                        IntakeMotorLeftConfig.closedLoop
                        .maxMotion.maxAcceleration(1000000);


                        IntakeMotorRightConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                            // Set PID values for position control. We don't need to pass a closed
                            // loop slot, as it will default to slot 0.
                            .p(IntakeConstants.p)
                            .i(IntakeConstants.i)
                            .d(IntakeConstants.d)
                            .outputRange(-1, 1)
                            .feedForward
                            .kS(IntakeConstants.s)
                            .kV(IntakeConstants.v)
                            .kA(IntakeConstants.a)
                            ;

                        IntakeMotorRightConfig.closedLoop
                        .maxMotion.maxAcceleration(1000000);

                }

        };

        public static final class KickerSubsystem {
                
            public static final SparkFlexConfig KickerMotorLeftConfig = new SparkFlexConfig();
            public static final SparkFlexConfig KickerMotorRightConfig = new SparkFlexConfig();
            // public static final SparkFlexConfig IntakeRightMotorConfig = new SparkFlexConfig();

                static {

                        KickerMotorLeftConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(40).voltageCompensation(12);
                        KickerMotorRightConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(40).voltageCompensation(12).follow(KickerConstants.KICKER_LEFT_ID, true);



                        KickerMotorLeftConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                            // Set PID values for position control. We don't need to pass a closed
                            // loop slot, as it will default to slot 0.
                            .p(KickerConstants.p)
                            .i(KickerConstants.i)
                            .d(KickerConstants.d)
                            .outputRange(-1, 1)
                            .feedForward
                            .kS(KickerConstants.s)
                            .kV(KickerConstants.v)
                            .kA(KickerConstants.a)
                            ;

                        KickerMotorLeftConfig.closedLoop
                        .maxMotion.maxAcceleration(1000000);


                        KickerMotorRightConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                            // Set PID values for position control. We don't need to pass a closed
                            // loop slot, as it will default to slot 0.
                            .p(KickerConstants.p)
                            .i(KickerConstants.i)
                            .d(KickerConstants.d)
                            .outputRange(-1, 1)
                            .feedForward
                            .kS(KickerConstants.s)
                            .kV(KickerConstants.v)
                            .kA(KickerConstants.a)
                            ;

                        KickerMotorRightConfig.closedLoop
                        .maxMotion.maxAcceleration(1000000);

                }

        };
        

}
