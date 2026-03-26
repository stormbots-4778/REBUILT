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


    /*
     * .add(1.3, 2850, -20)
            .add(2.4, 3450, 37)
            .add(3.7, 3750, 40)
            .add(4.5, 3925, 46);
     */
    private static final Table tables = new Table()
            // .add(1.3, 2800, -20)
            // .add(2.4, 3200, 46)
            // .add(2.8, 3300, 46)
            // .add(3.2, 3450, 38)
            // .add(3.7, 3600, 40)
            // .add(4.5, 3875, 46);
            .add(1.3, 3100, 15)
            .add(2.4, 3500, 37)
            .add(2.8, 3600, 46)
            .add(3.2, 3750, 38)
            .add(3.7, 3900, 40)
            .add(4.5, 4250, 46);

    public static final InterpolatingDoubleTreeMap shooter = tables.sMap;
    public static final InterpolatingDoubleTreeMap hood = tables.hMap;
}
