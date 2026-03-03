from luma.core.interface.serial import i2c
from luma.core.render import canvas
from luma.oled.device import ssd1306
import time

class OLED_Display:
    def __init__(self, num_lines=6, i2c_port=1, i2c_address=0x3C, rotate=0):
        try:
            self.serial = i2c(port=i2c_port, address=i2c_address)
            self.device = ssd1306(self.serial, rotate=rotate)
            self.screen_content = [""] * num_lines  # Initialize screen content with empty lines
        except:
            print('OLED disconnected or not found:')

    def display_text(self):
        """Displays the current content stored in the screen_content list."""
        with canvas(self.device) as draw:
            for i, text in enumerate(self.screen_content):
                draw.text((0, i * 10), text, fill="white")  # Display each line 10 pixels apart

    def update_line(self, line_number, text):
        """Updates a specific line on the screen and refreshes the display."""
        if 0 <= line_number < len(self.screen_content):
            self.screen_content[line_number] = text
            self.display_text()
        else:
            print("Invalid line number!")

    def set_all_lines(self, lines):
        """Sets all lines at once and refreshes the display."""
        for i in range(min(len(self.screen_content), len(lines))):
            self.screen_content[i] = lines[i]
        self.display_text()

if __name__ == '__main__':
    oled = OLED_Display()
    oled.set_all_lines([
        "Line 1: Initial",
        "Line 2: Content",
        "Line 3: Python",
        "Line 4: Raspberry Pi",
        "Line 5: OLED Test",
        "Line 6: Active"
    ])
    time.sleep(2)
    oled.update_line(2, "Line 3: Updated Content")
    oled.update_line(5, "Line 6: New Message")
    print("Press Ctrl+C to exit.")
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\nExiting program.")