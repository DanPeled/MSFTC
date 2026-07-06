package org.firstinspires.ftc.teamcode.msftc.mechanisms;

import android.graphics.Path;

import androidx.core.math.MathUtils;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.msftc.Motor;
import org.firstinspires.ftc.teamcode.msftc.dashboard.DashboardUtils;
import org.firstinspires.ftc.teamcode.msftc.dashboard.TunableNumber;

import java.util.Optional;

public abstract class PositionalMechanism<T extends PositionalMechanism<T, FFType>, FFType> extends Mechanism {
    protected final Motor leadMotor;
    protected PIDController positionPID;
    protected TunableNumber lowerLimit, upperLimit;
    protected Optional<Double> setpoint = Optional.empty();
    protected FFType ff;

    public PositionalMechanism(String name, Motor leadMotor) {
        super(name);
        this.leadMotor = leadMotor;

        lowerLimit = new TunableNumber(getName() + "/Limits", "lower", Double.MIN_VALUE);
        upperLimit = new TunableNumber(getName() + "/Limits", "upper", Double.MAX_VALUE);
    }

    @SuppressWarnings("unchecked")
    public T withLimits(double lowerLimit, double upperLimit) {
        setUpperLimit(upperLimit);
        setLowerLimit(lowerLimit);
        return (T) this;
    }

    @SuppressWarnings("unchecked")
    public T withUpperLimit(double UpperLimit) {
        setUpperLimit(UpperLimit);
        return (T) this;
    }

    @SuppressWarnings("unchecked")
    public T withLowerLimit(double lowerLimit) {
        setLowerLimit(lowerLimit);
        return (T) this;
    }

    @SuppressWarnings("unchecked")
    public T withFeedforward(FFType ff) {
        this.ff = ff;
        return (T) this;
    }

    @SuppressWarnings("unchecked")
    public T withPID(PIDController pid) {
        positionPID = pid;
        return (T) this;
    }

    protected void setLowerLimit(double lower) {
        lowerLimit.setValue(lower);
    }

    protected void setUpperLimit(double upper) {
        upperLimit.setValue(upper);
    }

    protected double calculateClosedLoopControl(double setpoint) {
        return positionPID.calculate(getPosition(), setpoint);
    }

    protected abstract double getFeedforwardOutput(double feedback);

    public void runToPosition(double setpoint) {
        setSetpoint(setpoint);
        update();
    }

    public void update() {
        if (!setpoint.isPresent()) return;

        double actualSetpoint = MathUtils.clamp(setpoint.orElse(lowerLimit.getValueAsDouble()), lowerLimit.getValueAsDouble(), upperLimit.getValueAsDouble());

        double feedback = calculateClosedLoopControl(actualSetpoint);
        double u = feedback + getFeedforwardOutput(feedback);
        if (u < 0 && getPosition() <= lowerLimit.getValueAsDouble()) leadMotor.set(0);
        else if (u > 0 && getPosition() >= upperLimit.getValueAsDouble()) leadMotor.set(0);
        else leadMotor.set(u);
    }

    public void setSetpoint(double setpoint) {
        this.setpoint = Optional.of(setpoint);
    }


    public double getPosition() {
        return leadMotor.getPosition();
    }

    public void initConfig() {
        DashboardUtils.uploadConfig(positionPID, getName() + "/PID");
    }
}
