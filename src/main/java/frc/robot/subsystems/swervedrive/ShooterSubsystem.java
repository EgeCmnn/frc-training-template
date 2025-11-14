package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.StringLogEntry;

import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {

    private final CANSparkMax shooterMotor =  new CANSparkMax(Constants.Shooter.motorID, MotorType.kBrushless);
    
    private final double peakForwardVoltage = Constants.Shooter.peakForwardVoltage;

    private final double feedVoltage = Constants.Shooter.feedVoltage;
    private final double advanceVoltage = Constants.Shooter.advanceVoltage;
    private final double scoreL1Voltage = Constants.Shooter.scoreL1Voltage;
    private final double scoreL2L3Voltage = Constants.Shooter.scoreL2L3Voltage;
    private final double scoreL4Voltage = Constants.Shooter.scoreL4Voltage;
    private final double algaeVoltage = Constants.Shooter.algaeVoltage;
    private final double algaeHoldVoltage = Constants.Shooter.algaeHoldVoltage;
    private final double algaeBargeVoltage = Constants.Shooter.algaeBargeVoltage;
    private final double algaeOutVoltage = Constants.Shooter.algaeOutVoltage;


    private final DataLog log = DataLogManager.getLog();
    private final StringLogEntry shooterLog = new StringLogEntry(log, "Shooter/ShooterRunning");


    public ShooterSubsystem() {
        shooterMotor.restoreFactoryDefaults();

        shooterMotor.setVoltageCompensation(peakForwardVoltage); 
        shooterMotor.enableVoltageCompensation(true);

        shooterMotor.setSmartCurrentLimit(40);
    }

    public void setVoltage(double volts) {
        shooterMotor.setVoltage(volts);
    }

    /* Shooter Modes */
    public void feed() { setVoltage(feedVoltage); }
    public void advance() { setVoltage(advanceVoltage); }
    public void scoreL1() { setVoltage(scoreL1Voltage); }
    public void scoreL2L3() { setVoltage(scoreL2L3Voltage); }
    public void scoreL4() { setVoltage(scoreL4Voltage); }
    public void algae() { setVoltage(algaeVoltage); }
    public void algaeHold() { setVoltage(algaeHoldVoltage); }
    public void algaeBarge() { setVoltage(algaeBargeVoltage); }
    public void algaeOut() { setVoltage(algaeOutVoltage); }

     public void stop() {
        shooterMotor.stopMotor();
    }


    @Override
    public void periodic() {

        SmartDashboard.putNumber("Shooter Voltage", shooterMotor.getAppliedOutput());
        SmartDashboard.putNumber("Shooter Current", shooterMotor.getOutputCurrent());

        shooterLog.append(Boolean.toString(shooterMotor.getAppliedOutput() != 0));
    }
    
}
