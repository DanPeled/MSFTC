package org.firstinspires.ftc.teamcode.msftc.dashboard;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.ValueProvider;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.SimpleMotorFeedforward;

import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

public class DashboardUtils {
    private static FtcDashboard getDashboardInstance() {
        FtcDashboard inst =
                FtcDashboard.getInstance();
        return inst;
    }

    public static void uploadConfig(DoubleSupplier get, DoubleConsumer set, String where, String name) {
        FtcDashboard dashboard = getDashboardInstance();
        dashboard.addConfigVariable(where, name, new ValueProvider<Double>() {
            @Override
            public Double get() {
                return get.getAsDouble();
            }

            @Override
            public void set(Double value) {
                set.accept(value);
            }
        });
    }

    public static void uploadConfig(PIDController pidController, String where) {
        uploadConfig(pidController::getP, pidController::setP, where, "P");
        uploadConfig(pidController::getI, pidController::setI, where, "I");
        uploadConfig(pidController::getD, pidController::setD, where, "D");
    }
}
