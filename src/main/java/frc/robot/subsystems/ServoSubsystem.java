package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ServoSubsystem extends SubsystemBase {
    Servo exampleServo = new Servo(1);

    public ServoSubsystem() {
        Servo exampleServo = new Servo(1);
    }

    public void set(double speed) {
        exampleServo.set(speed);
    }
}
