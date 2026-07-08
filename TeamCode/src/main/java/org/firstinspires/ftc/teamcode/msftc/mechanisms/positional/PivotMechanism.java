package org.firstinspires.ftc.teamcode.msftc.mechanisms.positional;

import com.arcrobotics.ftclib.controller.wpilibcontroller.SimpleMotorFeedforward;

import org.firstinspires.ftc.teamcode.msftc.Motor;

public class PivotMechanism extends PositionalMechanism<PivotMechanism, SimpleMotorFeedforward> {
    public PivotMechanism(String name, Motor leadMotor) {
        super(name, leadMotor);
    }

    @Override
    protected double getFeedforwardOutput(double feedback) {
        return ff.calculate(feedback);
    }
}
