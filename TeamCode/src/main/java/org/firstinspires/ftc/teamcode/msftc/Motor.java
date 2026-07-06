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
import java.util.Set;

public class Motor {
    // Maybe add background loop to update motor states that arent modified by this accessors
    private boolean m_enabled = true;
    private boolean m_encoderEnabled = false;
    private DcMotorSimple.Direction m_direction;
    private DcMotor.ZeroPowerBehavior m_zeroPowerBehaviour = DcMotor.ZeroPowerBehavior.FLOAT;
    private double m_minOutput = -1.0, m_maxOutput = 1.0;
    private Set<Motor> m_followers = new HashSet<>();
    private GearBox m_gearbox;
    private DcMotorEx m_internalMotor;
    private final String m_motorName;
    private double m_positionOffset = 0.0;
    private double m_ticksPerRevolution = MotorType.GOBILDA_6000.getTicksPerRevolution();

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

    public Motor(DcMotorEx m) {
        m_internalMotor = m;
        m_motorName = m.getDeviceName();
    }

    public Motor(@NonNull HardwareMap hm, String configName) {
        this(hm.get(DcMotorEx.class, configName));
    }

    public Motor ofType(MotorType type) {
        m_ticksPerRevolution = type.getTicksPerRevolution();
        return this;
    }

    public double getStatorCurrent() {
        return m_internalMotor.getCurrent(CurrentUnit.AMPS);
    }

    public void set(double output) {
        double finalOutput = MathUtils.clamp(output, m_minOutput, m_maxOutput);
        if (output != finalOutput) {
            GlobalTelemetry.warn(String.format(Locale.US, "Motor (%s) Requested output (%.2f) outside range [%.2f, %.2f], scaling down to %.2f",
                    m_motorName,
                    output,
                    m_minOutput,
                    m_maxOutput,
                    finalOutput));
        }
        doActionToMotorAndFollowers((DcMotorEx m) -> {
            m.setPower(finalOutput);
        });
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

    @FunctionalInterface
    private interface MotorAction {
        void execute(DcMotorEx m);
    }
}
