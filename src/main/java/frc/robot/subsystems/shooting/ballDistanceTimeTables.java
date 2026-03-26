package frc.robot.subsystems.shooting;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class ballDistanceTimeTables {
    private static class Table {
        private final InterpolatingDoubleTreeMap timeMap;

        Table() {
            timeMap = new InterpolatingDoubleTreeMap();
        }

        public Table add(double key, double time) {
            timeMap.put(key, time);
            
            return this;
        }
    }

    private static final Table tables = new Table()
            .add(1.3, 1)
            .add(2.4, 1.5)
            .add(3.4, 2)
            .add(4.4, 2.5);

    public static final InterpolatingDoubleTreeMap time = tables.timeMap;
}
