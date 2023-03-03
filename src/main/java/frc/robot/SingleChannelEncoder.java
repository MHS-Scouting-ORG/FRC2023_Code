package frc.robot;

import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicReference;

import edu.wpi.first.wpilibj.CounterBase;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SingleChannelEncoder implements CounterBase {

    private boolean m_current; // current read value of encoder
    private final ScheduledExecutorService m_exec;
    private final MotorController m_controller;
    private final DigitalInput m_dio;
    private AtomicReference<Double> m_count = new AtomicReference<>(0.0);
    private AtomicInteger m_num_stable = new AtomicInteger(0);
    private AtomicInteger m_last_stable = new AtomicInteger(-1);

    private static final double DEAD_ZONE = 0.01; // 0.01

    private double sign() {
        double speed = m_controller.get();
        if (Math.abs(speed) > DEAD_ZONE) {
            double sign = speed / Math.abs(speed);

            return sign;
        }
        return 1;
    }

    public SingleChannelEncoder(MotorController sc, DigitalInput dio) {
        this(sc, dio, 500);
    }

    /**
     *
     * @param controller
     *            what directions is the controller going?
     * @param dio
     *            detected changes
     */
    public SingleChannelEncoder(MotorController controller, DigitalInput dio, int sample) {
        m_controller = controller;
        m_dio = dio;
        m_current = dio.get();
        m_exec = Executors.newSingleThreadScheduledExecutor();
        m_exec.scheduleAtFixedRate(new Runnable() {

            @Override
            public void run() {
                boolean current = m_dio.get();
                SmartDashboard.putBoolean("DIO", current);
                if (current == m_current) {

                    // is the motor being run
                    if (Math.abs(m_controller.get()) > DEAD_ZONE)
                        m_num_stable.incrementAndGet();
                    else
                        m_num_stable.set(0);
                } else {
                    // how many samples did we get before current change
                    if (m_num_stable.get() < 2) {
                        System.err.println("Undersampling!!!!!!!");
                    }

                    m_last_stable.set(m_num_stable.getAndSet(0));

                    m_current = current;
                    m_count.getAndAccumulate(1.0 * sign(), (R, V) -> R + V);

                }
            }
        }, 0, sample, TimeUnit.MICROSECONDS);
    }

    @Override
    public int get() {
        return m_count.get().intValue();
    }

    @Override
    public void reset() {
        m_count.set(0.0);

    }

    @Override
    public double getPeriod() {
        throw new UnsupportedOperationException();
    }

    @Override
    public void setMaxPeriod(double maxPeriod) {
        throw new UnsupportedOperationException();

    }

    @Override
    public boolean getStopped() {
        throw new UnsupportedOperationException();
    }

    @Override
    public boolean getDirection() {
        return sign() > 0 ? true : false;
    }

    public int getSampleRate() {
        return m_last_stable.get();
    }
}