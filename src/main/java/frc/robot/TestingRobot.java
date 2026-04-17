package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class TestingRobot implements RunnableRobot {
    public TestingRobot() {
        var controller = new CommandXboxController(0);
        controller.leftBumper().and(controller.rightBumper().negate())
                .whileTrue(Commands.run(() -> System.out.println("left")));
        controller.rightBumper().and(controller.leftBumper().negate())
                .whileTrue(Commands.run(() -> System.out.println("right")));
        controller.leftBumper().and(controller.rightBumper())
                .whileTrue(Commands.run(() -> System.out.println("both")));

        controller.a().whileTrue(Commands.run(() -> System.out.println("AAA")));
    }

    public Command getAutonomousCommand() {
        return Commands.none();
    }
}
