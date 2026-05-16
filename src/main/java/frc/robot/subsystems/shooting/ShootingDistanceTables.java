package frc.robot.subsystems.shooting;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class ShootingDistanceTables {
    private static class Table {
        private final InterpolatingDoubleTreeMap sMap;
        private final InterpolatingDoubleTreeMap hMap;

        Table() {
            sMap = new InterpolatingDoubleTreeMap();
            hMap = new InterpolatingDoubleTreeMap();
        }

        public Table add(double key, double shoot, double hood) {
            sMap.put(key, shoot);
            hMap.put(key, hood);
            return this;
        }
    }

    private static final Table tables = new Table()
            .add(1,   1250, 0.4)
            .add(1.6, 1350, 0.4)
            .add(2.4, 1550, 0.2)
            .add(3.6, 1800, 0.2);

    public static final InterpolatingDoubleTreeMap shooter = tables.sMap;
    public static final InterpolatingDoubleTreeMap hood = tables.hMap;
}
