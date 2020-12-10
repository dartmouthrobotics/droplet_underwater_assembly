import rospy
import serial

class DisplayController(object):
    def __init__(self, serial_port, baudrate):
        self.serial_connection = serial.Serial(port=serial_port, baudrate=baudrate, timeout=0.1)


    def check_lcd_line(self, lcd_line):
        if "," in lcd_line:
            rospy.logerr(
                "LCD command strings cannot contain commas. Given: {}".format(lcd_line)
            )
            return False

        if len(lcd_line) > 16:
            rospy.logerr(
                "LCD lines cannot be longer than 16 characters. Current line contains {} characters. Given: {}".format(len(lcd_line), lcd_line)
            )
            return False

        return True


    def update_lcd_display(self, line_1, line_2):
        valid = self.check_lcd_line(line_1) and self.check_lcd_line(line_2)

        if not valid:
            return

        lcd_command = "lcd,{},{}\n".format(line_1, line_2)
        self.serial_connection.write(lcd_command)


    def update_buzzer_pattern(self, buzzer_pattern):
        buzz = "buzzer,30,{}\n".format(buzzer_pattern)
        self.serial_connection.write(buzz)


    def update_led_state(self, color, pattern):
        led_command = "led,{},{},{},{}\n".format(color[0], color[1], color[2], pattern)

        self.serial_connection.write(led_command)
