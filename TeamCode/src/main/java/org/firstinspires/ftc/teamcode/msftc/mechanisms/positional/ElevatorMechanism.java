package org.firstinspires.ftc.teamcode.msftc.mechanisms.positional;


import com.arcrobotics.ftclib.controller.wpilibcontroller.ElevatorFeedforward;

import org.firstinspires.ftc.teamcode.msftc.Motor;

public class ElevatorMechanism extends PositionalMechanism<ElevatorMechanism, ElevatorFeedforward> {
    public ElevatorMechanism(String name, Motor leadMotor) {
        super(name, leadMotor);
    }

    @Override
    protected double getFeedforwardOutput(double feedback) {
        return ff.calculate(feedback, 0);
    }
}
