package frc.robot;

import com.revrobotics.spark.config.SparkFlexConfig;

import com.revrobotics.spark.config.SparkBaseConfig.*;

import frc.robot.Constants.FunnelConstants;
import frc.robot.Constants.HopperConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.KickerConstants;
import frc.robot.Constants.ShooterConstants;

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

                        IntakeMotorLeftConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(50).voltageCompensation(12);
                        IntakeMotorRightConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(50).voltageCompensation(12).follow(IntakeConstants.INTAKE_LEFT_ID, true);



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

        public static final class FunnelSubsystem {
                
            public static final SparkFlexConfig FunnelMotorConfig = new SparkFlexConfig();

                static {

                        FunnelMotorConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(40).voltageCompensation(12);



                        FunnelMotorConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                            // Set PID values for position control. We don't need to pass a closed
                            // loop slot, as it will default to slot 0.
                            .p(FunnelConstants.p)
                            .i(FunnelConstants.i)
                            .d(FunnelConstants.d)
                            .outputRange(-1, 1)
                            .feedForward
                            .kS(FunnelConstants.s)
                            .kV(FunnelConstants.v)
                            .kA(FunnelConstants.a)
                            ;
                        FunnelMotorConfig.closedLoop
                                .maxMotion.maxAcceleration(100000);



                }

        };
        

        public static final class ClimberSubsystem {
                
            public static final SparkFlexConfig ClimbMotorLeftConfig = new SparkFlexConfig();
            

                static {

                        ClimbMotorLeftConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(40).voltageCompensation(12);

                        ClimbMotorLeftConfig.closedLoop
                        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                        .p(0.0002355)
                        .i(0)
                        .d(0)
                        .outputRange(-1, 1);
                        ClimbMotorLeftConfig.closedLoop
                                .maxMotion
                                        .maxAcceleration(600000)
                                        .cruiseVelocity(600000)
                                        .allowedProfileError(0.1); // smooth extension

                                
                }

        };


        public static final class PushoutSubsystem {

            public static final SparkFlexConfig PushoutMotorConfig = new SparkFlexConfig();
        //     public static final SparkFlexConfig PushoutMotorAgitateConfig = new SparkFlexConfig();
            // public static final SparkFlexConfig PushoutRightMotorConfig = new SparkFlexConfig();


                static {
                        PushoutMotorConfig.idleMode(IdleMode.kCoast).smartCurrentLimit(40).voltageCompensation(12);


                        PushoutMotorConfig.closedLoop
                        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                        .p(3.3)
                        .i(0.0)
                        .d(0.001)
                        .outputRange(-1.0, 1.0);

                        

                        PushoutMotorConfig.closedLoop
                        .maxMotion
                                .allowedProfileError(0.5)
                                .cruiseVelocity(400000)
                                .maxAcceleration(400000);   
                        
                        // PushoutMotorAgitateConfig.idleMode(IdleMode.kCoast).smartCurrentLimit(40).voltageCompensation(12);
                        // PushoutMotorAgitateConfig.closedLoop
                        // .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                        // .p(3.3)
                        // .i(0.0)
                        // .d(0.001)
                        // .outputRange(-1.0, 1.0);

                        // PushoutMotorAgitateConfig.closedLoop
                        // .maxMotion
                        //         .allowedProfileError(0.5)
                        //         .cruiseVelocity(3000)
                        //         .maxAcceleration(10000);   
                        
                }

            public static final SparkFlexConfig PushoutMotorPullConfig = new SparkFlexConfig();
        //     public static final SparkFlexConfig PushoutMotorAgitateConfig = new SparkFlexConfig();
            // public static final SparkFlexConfig PushoutRightMotorConfig = new SparkFlexConfig();


                static {
                        PushoutMotorConfig.idleMode(IdleMode.kCoast).smartCurrentLimit(40).voltageCompensation(12);


                        PushoutMotorConfig.closedLoop
                        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                        .p(3.3)
                        .i(0.0)
                        .d(0.001)
                        .outputRange(-1.0, 1.0);

                        PushoutMotorConfig.closedLoop
                        .maxMotion
                                .allowedProfileError(0.5)
                                .cruiseVelocity(10000)
                                .maxAcceleration(10000);   
                        
                        // PushoutMotorAgitateConfig.idleMode(IdleMode.kCoast).smartCurrentLimit(40).voltageCompensation(12);
                        // PushoutMotorAgitateConfig.closedLoop
                        // .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                        // .p(3.3)
                        // .i(0.0)
                        // .d(0.001)
                        // .outputRange(-1.0, 1.0);

                        // PushoutMotorAgitateConfig.closedLoop
                        // .maxMotion
                        //         .allowedProfileError(0.5)
                        //         .cruiseVelocity(3000)
                        //         .maxAcceleration(10000);   
                        
                }


        };

        public static final class KickerSubsystem {
                public static final SparkFlexConfig kickerLeftMotorConfig = new SparkFlexConfig();
                public static final SparkFlexConfig kickerRightMotorConfig = new SparkFlexConfig();

                        static {
                                kickerLeftMotorConfig.idleMode(IdleMode.kCoast).smartCurrentLimit(40).voltageCompensation(12);
                                kickerRightMotorConfig.idleMode(IdleMode.kCoast).smartCurrentLimit(40).voltageCompensation(12).follow(KickerConstants.KICKER_LEFT_ID,true);
                                
                                kickerLeftMotorConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
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
                                kickerLeftMotorConfig.closedLoop
                                .maxMotion.maxAcceleration(1000000);


                                kickerRightMotorConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
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

                                kickerRightMotorConfig.closedLoop
                                .maxMotion.maxAcceleration(1000000);
                        }

        }
        public static final class ShooterSubsystem {
                public static final SparkFlexConfig ShooterRightMotor1Config = new SparkFlexConfig(); 
                public static final SparkFlexConfig ShooterRightMotor2Config = new SparkFlexConfig(); 

                public static final SparkFlexConfig ShooterLeftMotor1Config = new SparkFlexConfig();
                public static final SparkFlexConfig ShooterLeftMotor2Config = new SparkFlexConfig();
                
                        static {


                                ShooterRightMotor1Config.idleMode(IdleMode.kCoast).smartCurrentLimit(50).voltageCompensation(12);
                                ShooterRightMotor2Config.idleMode(IdleMode.kCoast).smartCurrentLimit(50).voltageCompensation(12)
                                .follow(ShooterConstants.SHOOTER_R1_ID, true);
                                
                                ShooterLeftMotor1Config.idleMode(IdleMode.kCoast).smartCurrentLimit(50).voltageCompensation(12);
                                ShooterLeftMotor2Config.idleMode(IdleMode.kCoast).smartCurrentLimit(50).voltageCompensation(12)
                                .follow(ShooterConstants.SHOOTER_L1_ID, true);
                                
                                
                                ShooterRightMotor1Config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                                // Set PID values for position control. We don't need to pass a closed
                                // loop slot, as it will default to slot 0.
                                .p(ShooterConstants.p)
                                .i(ShooterConstants.i)
                                .d(ShooterConstants.d)
                                .outputRange(-1, 1)
                                .feedForward
                                .kS(ShooterConstants.s)
                                .kV(ShooterConstants.v)
                                .kA(ShooterConstants.a)
                                ;
                                ;              

                                ShooterRightMotor1Config.closedLoop
                                .maxMotion.maxAcceleration(10000);

                                ShooterRightMotor2Config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                                // Set PID values for position control. We don't need to pass a closed
                                // loop slot, as it will default to slot 0.
                                .p(ShooterConstants.p)
                                .i(ShooterConstants.i)
                                .d(ShooterConstants.d)
                                .outputRange(-1, 1)
                                .feedForward
                                .kS(ShooterConstants.s)
                                .kV(ShooterConstants.v)
                                .kA(ShooterConstants.a)
                                ;
                                ;

                                ShooterRightMotor2Config.closedLoop
                                .maxMotion.maxAcceleration(10000);
                                
                                ShooterLeftMotor1Config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                                // Set PID values for position control. We don't need to pass a closed
                                // loop slot, as it will default to slot 0.
                                .p(ShooterConstants.p)
                                .i(ShooterConstants.i)
                                .d(ShooterConstants.d)
                                .outputRange(-1, 1)
                                .feedForward
                                .kS(ShooterConstants.s)
                                .kV(ShooterConstants.v)
                                .kA(ShooterConstants.a)
                                ;
                                ;      
                                
                                ShooterLeftMotor1Config.closedLoop
                                .maxMotion.maxAcceleration(10000);

                                ShooterLeftMotor2Config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                                // Set PID values for position control. We don't need to pass a closed
                                // loop slot, as it will default to slot 0.
                                .p(ShooterConstants.p)
                                .i(ShooterConstants.i)
                                .d(ShooterConstants.d)
                                .outputRange(-1, 1)
                                .feedForward
                                .kS(ShooterConstants.s)
                                .kV(ShooterConstants.v)
                                .kA(ShooterConstants.a)
                                ;

                                ShooterLeftMotor2Config.closedLoop
                                .maxMotion.maxAcceleration(10000);

                }

 
        }

        public static final class HopperSubsystem {
                
            public static final SparkFlexConfig TwindexerRightControllerConfig = new SparkFlexConfig();
            public static final SparkFlexConfig TwindexerLeftControllerConfig = new SparkFlexConfig();

                static {

                        TwindexerRightControllerConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(40).voltageCompensation(12).follow(HopperConstants.TWINDEXER_LEFT_ID, true);
                        TwindexerLeftControllerConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(40).voltageCompensation(12);

                        TwindexerRightControllerConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                            // Set PID values for  position control. We don't need to pass a closed
                            // loop slot, as it will default to slot 0.
                                .p(HopperConstants.p)
                                .i(HopperConstants.i)
                                .d(HopperConstants.d)
                                .outputRange(-1, 1)
                                .feedForward
                                .kS(HopperConstants.s)
                                .kV(HopperConstants.v)
                                .kA(HopperConstants.a)
                                ;
                        TwindexerRightControllerConfig.closedLoop
                                .maxMotion.maxAcceleration(100000);
                            // Set PID values for velocity control in slot 1
                        //     .p(0.0001, ClosedLoopSlot.kSlot1)
                        //     .i(0, ClosedLoopSlot.kSlot1)
                        //     .d(0, ClosedLoopSlot.kSlot1)
                        //     .outputRange(-1, 1, ClosedLoopSlot.kSlot1)
                        //     .feedForward
                        //     // kV is now in Volts, so we multiply by the nominal voltage (12V)
                        //     .kV(12.0 / 5767, ClosedLoopSlot.kSlot1);
                        

                            TwindexerLeftControllerConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                            // Set PID values for position control. We don't need to pass a closed
                            // loop slot, as it will default to slot 0.
                                .p(HopperConstants.p)
                                .i(HopperConstants.i)
                                .d(HopperConstants.d)
                                .outputRange(-1, 1)
                                .feedForward
                                .kS(HopperConstants.s)
                                .kV(HopperConstants.v)
                                .kA(HopperConstants.a)
                                ;
                        TwindexerLeftControllerConfig.closedLoop
                                .maxMotion.maxAcceleration(100000);
                            // Set PID values for velocity control in slot 1
                        //     .p(0.0001, ClosedLoopSlot.kSlot1)
                        //     .i(0, ClosedLoopSlot.kSlot1)
                        //     .d(0, ClosedLoopSlot.kSlot1)
                        //     .outputRange(-1, 1, ClosedLoopSlot.kSlot1)
                        //     .feedForward
                        //     // kV is now in Volts, so we multiply by the nominal voltage (12V)
                        //     .kV(12.0 / 5767, ClosedLoopSlot.kSlot1);

                     

                }
        }

}
