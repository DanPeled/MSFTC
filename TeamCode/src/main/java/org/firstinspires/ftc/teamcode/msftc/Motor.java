package org.firstinspires.ftc.teamcode.msftc;

import androidx.annotation.NonNull;
import androidx.core.math.MathUtils;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import java.util.Arrays;
import java.util.HashSet;
import java.util.Locale;
import java.util.Optional;
import java.util.Set;

import javax.annotation.Nonnegative;

public class Motor {
    // Maybe add background loop to update motor states that arent modified by this accessors
    private boolean m_enabled = true;
    private boolean m_encoderEnabled = false;
    private DcMotorSimple.Direction m_direction;
    private DcMotor.ZeroPowerBehavior m_zeroPowerBehaviour = DcMotor.ZeroPowerBehavior.FLOAT;
    private double m_minOutput = -1.0, m_maxOutput = 1.0;
    private Optional<Double> m_currentLimit = Optional.empty();
    private boolean m_currentLimitEnabled = false;
    private Set<Motor> m_followers = new HashSet<>();
    private GearBox m_gearbox;
    private DcMotorEx m_internalMotor;
    private final String m_motorName;
    private double m_positionOffset = 0.0;
    private double m_ticksPerRevolution = MotorType.GOBILDA_6000.getTicksPerRevolution();

    private Alert m_overCurrentAlert = new Alert("Motor is over current!", Alert.AlertLevel.WARNING); // Modifies text on runtime
    private Alert m_invalidOutputAlert = new Alert("Motor output is outside of valid range!", Alert.AlertLevel.WARNING); // Modifies text on runtime

    public Motor(@NonNull String configName) {
        m_motorName = configName;
    }

    public void queryMotor(@NonNull HardwareMap hm) {
        m_internalMotor = hm.get(DcMotorEx.class, m_motorName);

        setDirection(m_direction);
        setZeroPowerBehaviour(m_zeroPowerBehaviour);
        setEncoderMode(m_encoderEnabled);

        if (m_internalMotor == null) {
            throw new IllegalStateException("Motor not found in hardware map: " + m_motorName);
        }
    }

    public Motor(@NonNull DcMotorEx m) {
        m_internalMotor = m;
        m_motorName = m.getDeviceName();
    }

    public Motor(@NonNull HardwareMap hm, @NonNull String configName) {
        this(hm.get(DcMotorEx.class, configName));
    }

    public Motor ofType(MotorType type) {
        m_ticksPerRevolution = type.getTicksPerRevolution();
        return this;
    }

    public Motor withCurrentLimit(@Nonnegative double currentLimitAmps) {
        setCurrentLimit(currentLimitAmps);
        return this;
    }

    public Motor withCurrentLimitEnabled() {
        enableCurrentLimit();
        return this;
    }

    public void setCurrentLimit(double currentLimitAmps) {
        if (currentLimitAmps <= 0) {
            throw new IllegalArgumentException(String.format(
                    Locale.US,
                    "Motor (%s) Current limit must be greater than 0 amps, got %.2f",
                    m_motorName,
                    currentLimitAmps
            ));
        }

        m_currentLimit = Optional.of(currentLimitAmps);

        if (m_internalMotor != null) {
            m_internalMotor.setCurrentAlert(currentLimitAmps, CurrentUnit.AMPS);
        }
    }

    public boolean isOverCurrent() {
        return m_currentLimit.isPresent() &&
                getStatorCurrent() > m_currentLimit.get();
    }

    public double getStatorCurrent() {
        return m_internalMotor.getCurrent(CurrentUnit.AMPS);
    }

    public void set(double output) {
        double clampedAndCurrentLimitedOutput = MathUtils.clamp(output, m_minOutput, m_maxOutput);
        double current = getStatorCurrent();

        if (isOverCurrent() &&
                m_currentLimitEnabled) {
            m_overCurrentAlert.text = String.format(Locale.US,
                    "Motor (%s) current limit exceeded: %.2fA > %.2fA, stopping motor",
                    m_motorName,
                    current,
                    m_currentLimit.orElse(-1.0));
            m_overCurrentAlert.show();

            clampedAndCurrentLimitedOutput = 0;
        }
        if (output != clampedAndCurrentLimitedOutput) {
            m_invalidOutputAlert.text = String.format(Locale.US, "Motor (%s) Requested output (%.2f) outside range [%.2f, %.2f], scaling down to %.2f",
                    m_motorName,
                    output,
                    m_minOutput,
                    m_maxOutput,
                    clampedAndCurrentLimitedOutput);

            m_invalidOutputAlert.show();
        }

        final double finalOutput = clampedAndCurrentLimitedOutput; // Because lambdas require a final variable to be used inside
        doActionToMotorAndFollowers((motor) -> motor.setPower(finalOutput));
    }


    public double getPower() {
        return m_internalMotor.getPower();
    }

    public double getPosition() {
        return (m_internalMotor.getCurrentPosition() + m_positionOffset) *
                (360 / m_ticksPerRevolution) / m_gearbox.getRatio();
    }

    public void setEncoderPosition(double position) {
        m_positionOffset = getPosition() - position;
    }

    public Motor withZeroPowerBehaviour(DcMotor.ZeroPowerBehavior behaviour) {
        setZeroPowerBehaviour(behaviour);
        return this;
    }

    public Motor withMaxOutput(double maxOutput) {
        this.m_maxOutput = MathUtils.clamp(maxOutput, -1, 1);
        return this;
    }

    public void setEncoderMode(boolean activated) {
        if (m_internalMotor != null)
            m_internalMotor.setMode(activated ? DcMotor.RunMode.RUN_USING_ENCODER : DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public Motor enableEncoder() {
        m_encoderEnabled = true;
        setEncoderMode(m_encoderEnabled);
        return this;
    }


    public Motor disableEncoder() {
        m_encoderEnabled = false;
        setEncoderMode(m_encoderEnabled);
        return this;
    }

    public Motor withMinOutput(double minOutput) {
        this.m_minOutput = MathUtils.clamp(minOutput, -1, 1);
        return this;
    }

    public Motor withFollowers(Motor... followers) {
        m_followers.addAll(Arrays.asList(followers));
        return this;
    }

    public Motor withDirection(DcMotorSimple.Direction direction) {
        setDirection(direction);
        return this;
    }

    public Motor disabled() {
        disable();
        return this;
    }

    public Motor enabled() {
        enable();
        return this;
    }

    public Motor withGearBox(GearBox gearBox) {
        m_gearbox = gearBox;
        return this;
    }

    public DcMotorSimple.Direction getDirection() {
        return m_direction;
    }

    public void setDirection(DcMotorSimple.Direction direction) {
        m_direction = direction;
        if (m_internalMotor != null)
            m_internalMotor.setDirection(direction);
    }


    private void doActionToMotorAndFollowers(MotorAction action) {
        action.execute(m_internalMotor);
        for (Motor m : m_followers) {
            action.execute(m.m_internalMotor);
        }
    }


    public void disable() {
        m_enabled = false;
        if (m_internalMotor != null)
            m_internalMotor.setMotorDisable();
    }

    public void enable() {
        m_enabled = true;
        if (m_internalMotor != null)
            m_internalMotor.setMotorEnable();
    }

    public boolean isMotorEnabled() {
        return m_enabled;
    }

    public DcMotorEx getInternalMotor() {
        return m_internalMotor;
    }

    public double getVelocity() { // ticks per second, need to check how that works
        return m_internalMotor.getVelocity();
    }

    public void setZeroPowerBehaviour(DcMotor.ZeroPowerBehavior behaviour) {
        m_zeroPowerBehaviour = behaviour;
        if (m_internalMotor != null)
            m_internalMotor.setZeroPowerBehavior(behaviour);
    }

    public void enableCurrentLimit() {
        m_currentLimitEnabled = true;
    }

    public void disableCurrentLimit() {
        m_currentLimitEnabled = false;
    }

    @FunctionalInterface
    private interface MotorAction {
        void execute(DcMotorEx m);
    }
}
