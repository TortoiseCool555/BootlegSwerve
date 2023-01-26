// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/** Class that holds x, y, and angle values. Units are in meters and radians */
public class Point {
    private double x;
    private double y;
    private double angleRad;
    private String flag = "ACTIVE";

    public Point(double x, double y, double angleRad) {
        this.x = x;
        this.y = y;
        this.angleRad = angleRad;
    }
    
    public void setX(double x) {
        this.x = x;
    }
    public void setY(double y) {
        this.y = y;
    }
    public void setAngleRad(double angleRad) {
        this.angleRad = angleRad;
    }

    public double getX() {
        return x;
    }
    public double getY() {
        return y;
    }
    public double getAngleRad() {
        return angleRad;
    }
}
