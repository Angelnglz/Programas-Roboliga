from machine import Pin, PWM
import time
from time import sleep_ms
from bleradio import BLERadio

# A board can broadcast small amounts of data on one channel. Here we broadcast
# on channel 5. This board will listen for other boards on channels 4 and 18.
radio = BLERadio(broadcast_channel=5, observe_channels=[4]) # Aqui podes sumar cuantos canales necesites en la lista!!

# You can run a variant of this script on another board, and have it broadcast
# on channel 4 or 18, for example. This board will then receive it.

counter = 0

# Create PWM object for the servo
servo = PWM(Pin(0), freq=50)

#myLED = Pin(0, Pin.OUT)
#myLED_rojo = Pin(48, Pin.OUT)

# Function to map an angle (0 to 180) to PWM duty cycle
def set_angle(angle):
    # Calculate duty cycle from angle, scaled between 1 ms (51) to 2 ms (102)
    min_duty = 26   # Corresponds to 1ms pulse
    max_duty = 123  # Corresponds to 2ms pulse
    # Map the angle (0-180) to duty cycle (51-102)
    duty = min_duty + (max_duty - min_duty) * angle // 180
    servo.duty(duty)

while True:
    try:
      

      # Data observed on channel 4, as broadcast by another board.
      # It gives None if no data is detected.
      observed = radio.observe(4)
      
      if observed != None and observed[0] =="B": # 
        #myLED.value(0)
        set_angle(0)

      if observed != None and observed[0] =="S": #
        #myLED.value(1)
        set_angle(90)
        
      print(f"Llego --> {observed}")
  
      data = ["hello, world!", 3.14, counter]
      # Broadcast some data on our channel, which is 5.
      radio.broadcast(data)
      # print(f"Salio --> {data}")
      counter += 1
      sleep_ms(10)

    except KeyboardInterrupt:
        # Cleanup on program stop
        servo.deinit()  # Turn off PWM to release the resource
