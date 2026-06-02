from time import sleep
from dfplayermini import DFPlayerMini
from machine import Pin

led = Pin('LED', Pin.OUT)

sleep(2)  # Allow time for the DFPlayer to initialize
led.value(1)  # Turn on LED to indicate we're starting the test
player1 = DFPlayerMini(1, 4, 5)
print("DFPlayer Mini Test")
result = player1.select_source('sdcard')
print(f"Select Source Result: {result}")
result = player1.query_num_files()
print(f"Number of Files: {result}")
result = player1.set_volume(20)
print(f"Set Volume Result: {result}")
result = player1.play(2)
print(f"Play Result: {result}")
#sleep(5)  # Let the music play for a while
#result = player1.pause()
#print(f"Pause Result: {result}")
#sleep(2)  # Pause for a moment
#result = player1.play(1)
#print(f"Resume Play Result: {result}")
#sleep(5)  # Let the music play for a while
#result = player1.stop()
#print(f"Stop Result: {result}")