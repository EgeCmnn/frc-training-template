package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.StringLogEntry;

import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
    /* Motor and Sensor */ 
    private final CANSparkMax intakeMotor =  new CANSparkMax(Constants.Intake.motorID, MotorType.kBrushless);
    private final DigitalInput coralSensor = new DigitalInput(Constants.Intake.sensorID);
    
    private final double peakForwardVoltage = Constants.Intake.peakForwardVoltage;
    private final double peakReverseVoltage = Constants.Intake.peakReverseVoltage; //ReverseForward was not neccesary but added beacause of it was in the Const file
    private final double intakeVoltage = Constants.Intake.intakeVoltage;
    private final double rejectVoltage = Constants.Intake.rejectVoltage;

    private final DataLog log = DataLogManager.getLog();
    private final StringLogEntry coralLog = new StringLogEntry(log, "Intake/CoralDetected");


    /*     Constructor        */
    public IntakeSubsystem() {
        intakeMotor.restoreFactoryDefaults();

        intakeMotor.setVoltageCompensation(peakForwardVoltage); 
        intakeMotor.enableVoltageCompensation(true);

        intakeMotor.setSmartCurrentLimit(40);
    }


    /*  -Public Methods-    */

    public void intake() {
        intakeMotor.setVoltage(intakeVoltage);  
    }

    public void reject() {
        intakeMotor.setVoltage(rejectVoltage);  
    }

    /** Intake volt control */
    public void setVoltage(double volts) {
        intakeMotor.setVoltage(volts);
    }

    /** Stop motor */
    public void stop() {
        intakeMotor.stopMotor();
    }

    /* Detect Coral */
    public boolean hasCoral() {
        return !coralSensor.get(); 
    }


    @Override
    public void periodic() {

        SmartDashboard.putBoolean("Coral Detected", hasCoral());
        SmartDashboard.putBoolean("IR Raw", coralSensor.get());

        coralLog.append(Boolean.toString(hasCoral()));
    }
}
