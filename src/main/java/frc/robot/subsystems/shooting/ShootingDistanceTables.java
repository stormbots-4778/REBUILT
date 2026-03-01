package frc.robot.subsystems.shooting;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class ShootingDistanceTables {
    private static class Table {
        private final InterpolatingDoubleTreeMap map;

        Table() {
            map = new InterpolatingDoubleTreeMap();
        }

        public Table add(double key, double value) {
            map.put(key, value);
            return this;
        }
    }

    public static final InterpolatingDoubleTreeMap shooter = new Table().add(0, 0).map;
    public static final InterpolatingDoubleTreeMap hood = new Table().add(0, 0).map;
}
