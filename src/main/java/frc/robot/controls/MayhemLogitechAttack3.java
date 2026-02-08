package frc.robot.controls;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class MayhemLogitechAttack3 {
    Joystick m_joystick;

    public enum Axis {
        X(0), Y(1);

        private final int value;

        private Axis(int value) {
            this.value = value;
        }

        public int getValue() {
            return value;
        }
    }

    public MayhemLogitechAttack3(int joystickPort) {
        m_joystick = new Joystick(joystickPort);
    }

    public Trigger Button(int button) {
        return new JoystickButton(m_joystick, button);
    }

    public DoubleSupplier Axis(Axis axis) {
        return () -> m_joystick.getRawAxis(axis.getValue());
    }
}
