package org.firstinspires.ftc.teamcode.msftc.mechanisms;

import com.arcrobotics.ftclib.controller.wpilibcontroller.ArmFeedforward;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.msftc.Motor;

public class ArmMechanism extends PositionalMechanism<ArmMechanism, ArmFeedforward> {
    public ArmMechanism(String name, Motor leadMotor) {
        super(name, leadMotor);
    }

    @Override
    protected double getFeedforwardOutput(double feedback) {
        return ff.calculate(getPosition(), feedback);
    }
}
