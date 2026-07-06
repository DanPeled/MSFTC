package org.firstinspires.ftc.teamcode.msftc;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.msftc.dashboard.TunableNumber;

public class GlobalTelemetry {
    private static Telemetry m_telemetry;

    public static void init(Telemetry t) {
        m_telemetry = t;
        TunableNumber.initTunables();
    }

    public static void warn(String warning) {
        if (m_telemetry != null) {
            m_telemetry.addLine("⚠ WARNING: " + warning);
            m_telemetry.update();
        }
    }


    public static void error(String warning) {
        if (m_telemetry != null) {
            m_telemetry.addLine("⚠ ERROR:" + warning);
            m_telemetry.update();
        }
    }
}
