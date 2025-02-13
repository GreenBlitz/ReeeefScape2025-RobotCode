package frc.utils.utilcommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.utils.LoggedTunableNumber;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
public class LoggedDashboardCommandCheck extends FunctionalCommand {

	public LoggedDashboardCommandCheck(Consumer<Double> onExecute, String widgetName, Subsystem... requirements) {
		super(
			// Initialize the command
			() -> {
				// Perform any setup or initialization tasks if necessary
				SmartDashboard.updateValues(); // Push updated values to Shuffleboard (outbound data)
				SmartDashboard.putString("Test Ran", "AHHHHHHHHHHHHHHHHHH"); // Debug message
			},

			// The execute method that runs repeatedly
			() -> {
				// Get the latest value from the SmartDashboard (this will fetch the updated value)
				double voltage = SmartDashboard.getNumber(widgetName, 4);

				// Process the updated value with the provided consumer
				onExecute.accept(voltage);

				// Optionally update the SmartDashboard with the fetched value
				SmartDashboard.putNumber("Test2", voltage);  // Update "Test2" on Shuffleboard with voltage
			},

			// The interrupted method (this is executed if the command is interrupted)
			(interrupted) -> {
				// Handle interruption if necessary (e.g., cleanup)
			},

			// The condition for when the command should end
			() -> false, // This will run the command indefinitely, replace with actual condition if needed

			// List of requirements (subsystems)
			requirements
		);
	}
}

