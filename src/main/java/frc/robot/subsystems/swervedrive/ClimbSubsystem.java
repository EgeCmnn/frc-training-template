package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkLowLevel.ResetMode;
import com.revrobotics.SparkLowLevel.PersistMode;
import com.revrobotics.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class ClimbSubsystem extends SubsystemBase {

    // Encoder limit values
    private final double MIN_POSITION = 0.0;           // fully closed
    private final double MAX_POSITION = Constants.Climb.maxPosition;  // fully opened

    private double peakForwardVoltage = Constants.Climb.peakForwardVoltage;
    private double fastDeployVoltage = Constants.Climb.fastDeployVoltage; 
    private double slowDeployVoltage = Constants.Climb.slowDeployVoltage;
    private double engageRetractVoltage = Constants.Climb.engageRetractVoltage;
    private double fastRetractVoltage = Constants.Climb.fastRetractVoltage;
    private double slowRetractVoltage = Constants.Climb.slowRetractVoltage;
    private double holdVoltage = Constants.Climb.holdVoltage;

    private final CANSparkMax masterMotor = new CANSparkMax(Constants.Climb.masterID, MotorType.kBrushless);
    private final CANSparkMax slaveMotor = new CANSparkMax(Constants.Climb.slaveID, MotorType.kBrushless);

    private SparkMaxConfig masterConfig;
    private SparkMaxConfig slaveConfig;

    private final DataLog log = DataLogManager.getLog();
    private final StringLogEntry elevatorLog = new StringLogEntry(log, "Climb/Position");

    public ClimbSubsystem() {
        masterConfig = new SparkMaxConfig();
        slaveConfig = new SparkMaxConfig();

        /* Motor Config */
        masterConfig.idleMode(com.revrobotics.SparkLowLevel.IdleMode.kBrake).voltageCompensation(peakForwardVoltage).smartCurrentLimit(40);

        slaveConfig.idleMode(com.revrobotics.SparkLowLevel.IdleMode.kBrake).follow(masterMotor).voltageCompensation(peakForwardVoltage).smartCurrentLimit(40);

        masterMotor.configure(masterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        slaveMotor.configure(slaveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }


    
    public void stop() {
        masterMotor.stopMotor();
    }

    public void hold() {
        masterMotor.setVoltage(holdVoltage);
    }


    public void openFastClimb() {
        var encoder = masterMotor.getEncoder();

        if (encoder.getPosition() >= MAX_POSITION) {
            masterMotor.setVoltage(0);
            return;
        }

        masterMotor.setVoltage(fastDeployVoltage);
    }

    public void openSlowClimb() {
        var encoder = masterMotor.getEncoder();

        if (encoder.getPosition() >= MAX_POSITION) {
            masterMotor.setVoltage(0);
            return;
        }

        masterMotor.setVoltage(slowDeployVoltage);
    }

    public void closeSlowClimb() {
        var encoder = masterMotor.getEncoder();

        if (encoder.getPosition() <= MIN_POSITION) {
            masterMotor.setVoltage(holdVoltage);
            return;
        }

        masterMotor.setVoltage(slowRetractVoltage);
    }

    public void closeFastClimb() {
        var encoder = masterMotor.getEncoder();

        if (encoder.getPosition() <= MIN_POSITION) {
            masterMotor.setVoltage(holdVoltage);
            return;
        }

        masterMotor.setVoltage(fastRetractVoltage);
    }
    
    @Override
    public void periodic() {
        var encoder = masterMotor.getEncoder();
        double position = encoder.getPosition();
        double current = masterMotor.getOutputCurrent();
        double voltage = masterMotor.getAppliedOutput();
        
        
        SmartDashboard.putNumber("Climb Position", position);
        SmartDashboard.putNumber("Climb Motor Current", current);
        SmartDashboard.putNumber("Climb Motor Voltage", voltage);
        SmartDashboard.putNumber("Climb Slave Current", slaveMotor.getOutputCurrent());
        
        elevatorLog.append("Pos:" + position + " Current:" + current + " Volt:" + voltage);
        }
}
