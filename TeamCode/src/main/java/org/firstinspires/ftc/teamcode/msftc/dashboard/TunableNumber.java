package org.firstinspires.ftc.teamcode.msftc.dashboard;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import java.util.ArrayList;
import java.util.List;

public class TunableNumber {
    private double m_value;
    private String m_name;
    private String m_where;

    private static List<TunableNumber> m_tunables = new ArrayList<>();

    public static void initTunables() {
        for (TunableNumber tune : m_tunables) {
            tune.initConfig();
        }
    }

    public TunableNumber(String where, String name, double defaultValue) {
        m_where = where;
        m_value = defaultValue;
        m_name = name;
        m_tunables.add(this);
    }

    public TunableNumber(OpMode opMode, String name, double defaultValue) {
        this(opMode.getClass().getSimpleName(), name, defaultValue);
    }

    public void initConfig() {
        DashboardUtils.uploadConfig(() -> m_value, (v) -> m_value = v, m_where, m_name);
    }

    public double getValueAsDouble() {
        return m_value;
    }

    public void setValue(double value) {
        m_value = value;
    }
}
