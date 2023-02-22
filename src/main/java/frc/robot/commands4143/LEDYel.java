package frc.robot.commands4143;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public class LEDYel extends CommandBase{
    private AddressableLED m_led;
    private AddressableLEDBuffer m_ledBuffer;
    
    public LEDYel(AddressableLED led, AddressableLEDBuffer ledBuffer){
        m_led = led;
        m_ledBuffer = ledBuffer;
        for(int i = 0; i < m_ledBuffer.getLength(); i++){
            m_ledBuffer.setRGB(i, 0, 0, 0);
        }
    }

    @Override
    public void execute() {
        color();
    // Set the LEDs
    m_led.setData(m_ledBuffer);
  }

  private void color() {
    // For every pixel
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      // Calculate the hue - hue is easier for rainbows because the color
      // shape is a circle so only one value needs to precess
        m_ledBuffer.setRGB(i, 255, 255, 0);

    }
  }

    @Override
    public void end(boolean interrupted){
        for(int i = 0; i < m_ledBuffer.getLength(); i++){
            m_ledBuffer.setRGB(i, 0, 0, 0);
        }
        m_led.setData(m_ledBuffer);
    }
}
