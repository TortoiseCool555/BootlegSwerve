// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/** Add your docs here. */
public class Toggle {
    double previousInput;
    public Toggle(double startingInput) {
        this.previousInput = startingInput;
    }

    public boolean isToggled(double input) {
        boolean val = input != previousInput;
        previousInput = input;
        return val;
    }
}
