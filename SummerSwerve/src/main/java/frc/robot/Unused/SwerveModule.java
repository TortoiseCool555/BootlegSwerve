// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.Unused;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogEncoder;
import frc.robot.Constants;
import frc.robot.ExtraMath;

/** Add your docs here. */
public class SwerveModule {
    private TalonFX rotation;
    private TalonFX translation;
    private CANCoder rotEncoder;
    private CANCoder transEncoder;
    private AnalogEncoder rotEnconderAbsolute;
    private final int order;
    private final double modulePositionAngle;

    double moduleLocalAngle;
    double moduleOverallAngle;

    SwerveModuleState state;

    SwerveModule(TalonFX rotation, TalonFX translation, CANCoder rotEncoder, int order) {
        this.rotation = rotation;
        this.translation = translation;
        this.rotEncoder = rotEncoder;
        this.order = order;
        modulePositionAngle = (order * Math.PI / 2) - (Math.PI / 4);    // 90x - 45 = 45x + 45(x-1)
        rotation.setInverted(InvertType.InvertMotorOutput);
    }

    public void drive(double translationAngle, double translationPercent, double turnPercent, double robotAngle) {
        updateModuleAngle(robotAngle);
        double extraTurn = 0;
        if(translationAngle > (modulePositionAngle - (Math.PI / 2)) && translationAngle < (modulePositionAngle + (Math.PI / 2))) {
            extraTurn = turnPercent * (Math.PI / 4);
        }
        double rotationDifference = (ExtraMath.simpleAngleError(moduleOverallAngle, translationAngle) + extraTurn);
        rotation.set(ControlMode.PercentOutput, rotationDifference * Constants.rotationalPVal);
        
        translation.set(ControlMode.PercentOutput, translationPercent);
    }

    public void updateModuleAngle(double robotAngle) {
        // moduleLocalAngle = ExtraMath.mod((rotEncoder.get() / Constants.TICKS_PER_REVOLUTION * Math.PI*2), Math.PI*2);
        moduleLocalAngle = rotEncoder.getAbsolutePosition();
        moduleOverallAngle = (moduleLocalAngle + robotAngle) % (2*Math.PI);
    } 

    public double getModuleAngle() {
        return rotEncoder.getAbsolutePosition();
    }
    public double getModuleAngleLocal() {
        return moduleLocalAngle;
    }
    public double getModuleAngleRobot() {
        return moduleOverallAngle;
    }

    public double getOrder() {
        return order;
    }

    // New Module theme

    public double getMetersPerSec() {
        return transEncoder.getVelocity();
    }

    public double getAngle() {
        return rotEnconderAbsolute.getDistance();
    }

    public SwerveModuleState getModuleState() {
        return new SwerveModuleState(getMetersPerSec(), new Rotation2d(getAngle()));
    } 

    public void driveModule(SwerveModuleState desiredState) {
        getModuleState().optimize(desiredState, new Rotation2d(getAngle()));
        double trans = desiredState.speedMetersPerSecond / 5;
    }
}