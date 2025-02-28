package scoringcontroller;

import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.event.EventLoop;

/**
 * Handle input from a custom Arduino Leonardo controller connected to the Driver Station.
 *
 * File based on "XboxController" class provided in the WPILib
 */
public class CustomController extends GenericHID implements Sendable {
  /** Represents a digital button on the custom controller. */
  public enum Button {
    /** 0 button. */
    k0(0),
    /** 1 button. */
    k1(1),
    /** 2 button. */
    k2(2),
    /** 3 button. */
    k3(3),
    /** 4 button. */
    k4(4);

    /** Button value. */
    public final int value;

    Button(int value) {
      this.value = value;
    }

    /**
     * Get the human-friendly name of the button, matching the relevant methods. This is done by
     * stripping the leading `k`, and appending `Button`.
     *
     * <p>Primarily used for automated unit tests.
     *
     * @return the human-friendly name of the button.
     */
    @Override
    public String toString() {
      // Remove leading `k`
      return this.name().substring(1) + "Button";
    }
  }

  /**
   * Construct an instance of a controller.
   *
   * @param port The port index on the Driver Station that the controller is plugged into (0-5).
   */
  public CustomController(final int port) {
    super(port);
    HAL.report(tResourceType.kResourceType_Controller, port + 1);
  }

  /**
   * Read the value of the 0 button on the controller.
   *
   * @return The state of the button.
   */
  public boolean get0Button() {
    return getRawButton(Button.k0.value);
  }

  /**
   * Whether the 0 button was pressed since the last check.
   *
   * @return Whether the button was pressed since the last check.
   */
  public boolean get0ButtonPressed() {
    return getRawButtonPressed(Button.k0.value);
  }

  /**
   * Whether the 0 button was released since the last check.
   *
   * @return Whether the button was released since the last check.
   */
  public boolean get0ButtonReleased() {
    return getRawButtonReleased(Button.k0.value);
  }

  /**
   * Constructs an event instance around the 0 button's digital signal.
   *
   * @param loop the event loop instance to attach the event to.
   * @return an event instance representing the A button's digital signal
   *     attached to the given loop.
   */
  public BooleanEvent loop0(EventLoop loop) {
    return button(Button.k0.value, loop);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("HID");
    builder.publishConstString("ControllerType", "Custom Controller");
    builder.addBooleanProperty("0", this::get0Button, null);
  }
}
