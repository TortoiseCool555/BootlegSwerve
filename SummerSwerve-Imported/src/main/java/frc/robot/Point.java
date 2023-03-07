// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/** Add your docs here. */
public class Point {
    double x;
    double y;
    double a;
    public Point(double x, double y, double a) {
        this.x = x;
        this.y = y;
        this.a = a;
    }

    public void setX(double x) {
        this.x = x;
    }
    public void setY(double y) {
        this.y = y;
    }
    public void setAngleRad(double angle) {
        this.a = angle;
    }

    public double getX() {
        return x;
    }
    public double getY() {
        return y;
    }
    public double getAngleRad() {
        return a;
    }
}
