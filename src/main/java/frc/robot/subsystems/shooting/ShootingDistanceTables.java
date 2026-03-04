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
            .add(1.3, 3100, 0)
            .add(2.4, 3300, 30)
            .add(3.7, 3600, 40)
            .add(4.5, 4000, 50);

    public static final InterpolatingDoubleTreeMap shooter = tables.sMap;
    public static final InterpolatingDoubleTreeMap hood = tables.hMap;
}
