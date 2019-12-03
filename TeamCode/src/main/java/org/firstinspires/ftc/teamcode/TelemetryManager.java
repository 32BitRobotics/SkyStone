package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public final class TelemetryManager {
    public class TelemetryRunnable implements Runnable {
        Telemetry t;
        OpBase o;

        public TelemetryRunnable(Telemetry telem, OpBase op) {
            t = telem;
            o = op;
        }

        @Override
        public void run() {
            t.clear();
            t.addData("LeftFront Power", o.leftFront.getPower());
            t.update();
        }
    }

    Thread t;

    public TelemetryManager(OpBase o) {
        t = new Thread(new TelemetryRunnable(o.telemetry, o));
        t.run();
    }

    public void stop() {
        t.interrupt();
    }
}
