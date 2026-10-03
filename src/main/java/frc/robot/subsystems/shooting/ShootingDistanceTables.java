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
            .add(1.5, 1300, 0.4)
            .add(2.0, 1310, 1.4)
            .add(2.5, 1375, 2.5)
            .add(3.0, 1450, 2.9)
            .add(3.5, 1580, 2.9);

    public static final InterpolatingDoubleTreeMap shooter = tables.sMap;
    public static final InterpolatingDoubleTreeMap hood = tables.hMap;
}
