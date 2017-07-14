import threading
import logging
import time
import logging

import RPi.GPIO as GPIO

import alexapi.triggers as triggers
from .basetrigger import BaseTrigger

logger = logging.getLogger(__name__)


class UltrasonicTrigger(BaseTrigger):

    type = triggers.TYPES.OTHER

    def __init__(self, config, trigger_callback):
        super(UltrasonicTrigger, self).__init__(config, trigger_callback, 'ultrasonic')

        self._enabled_lock = threading.Event()
        self._disabled_sync_lock = threading.Event()
        self._player = None

    def setup(self):
        # Setup GPIO
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self._tconfig['gpio_pin_echo'], GPIO.IN)
        GPIO.setup(self._tconfig['gpio_pin_trigger'], GPIO.OUT)
        GPIO.setup(self._tconfig['gpio_pin_lights'], GPIO.IN)

    def run(self):
        # Start Thread
        thread = threading.Thread(target=self.thread, args=())
        thread.setDaemon(True)
        thread.start()

    def thread(self):
        while True:
            self._enabled_lock.wait()

            self.setup()
            triggered = False
            voice_command = self._config['triggers']['pocketsphinx']['phrase']

            while not triggered:
                if not self._enabled_lock.isSet():
                    break
                # Check if lights are on
                if self._checkLights():
                    # Ping
                    self._ping()
                    # Measure Distance
                    distance = self._measure_distance()
                    if distance < self._tconfig['distance_treshold_alexa']:
                        # Gotcha!
                        logger.info('Ultrasonic sensor triggered with distance ' + str(distance) + 'cm')
                        triggered = True
                        # Launch Assistant if distance is even smaller
                        if distance < self._tconfig['distance_treshold_assistant']:
                            logger.debug('Triggered Assistant through proximity sensor')
                            assistant_phrase = self._config['triggers']['pocketsphinx']['phrase_assistant']
                            if assistant_phrase is not None:
                                if isinstance(assistant_phrase, list):
                                    assistant_phrase = assistant_phrase[0]
                                voice_command = assistant_phrase
                    else:
                        # no gesture detected
                        # do not stress CPU and Sensor too much
                        time.sleep(0.75)
                else:
                    # lights are off
                    time.sleep(3)

            self._disabled_sync_lock.set()

            if triggered:
                self._trigger_callback(self, voice_command)


    def enable(self):
        self._enabled_lock.set()
        self._disabled_sync_lock.clear()

    def disable(self):
        self._enabled_lock.clear()
        self._disabled_sync_lock.wait()

    ## Ultrasonic Sensor
    def set_player(self, player=None):
        self._player = player

    def _ping(self):
        # settle Sensor
        GPIO.output(self._tconfig['gpio_pin_trigger'], GPIO.LOW)
        time.sleep(1)
        # do Ping
        GPIO.output(self._tconfig['gpio_pin_trigger'], GPIO.HIGH)
        time.sleep(0.00001)
        GPIO.output(self._tconfig['gpio_pin_trigger'], GPIO.LOW)

    def _measure_distance(self):
        # get length of pulse
        startTime = time.time()
        pulseStart = startTime
        pulseEnd = startTime
        
        while GPIO.input(self._tconfig['gpio_pin_echo']) == GPIO.LOW:
            pulseStart = time.time()
            if (pulseStart > startTime + 1):
                # Timeout, RPi 0 was too slow to capture the pulse
                logger.debug("Forcing Detection. RPi 0 was too slow to capture sensor response.")
                return self._tconfig['distance_treshold_assistant'] + 0.1;
        
        while GPIO.input(self._tconfig['gpio_pin_echo']) == GPIO.HIGH:
            pulseEnd = time.time()
        
        duration = pulseEnd - pulseStart # in seconds
        
        #logger.debug("Duration: "+str(round(duration*1000, 1))+"ms")

        ## speed of sound @ 20degC = 343.5 m/s
        #distance = duration * 34350 / 2 # in centimeters
        distance = duration * 26000 / 2 # adjusted to RPi0 performance
        distance = round(distance, 1)
        logger.debug("Distance: "+str(distance)+"cm")
        
        if distance >= 200 or distance < 0:
            # Sensor gets inaccurate
            logger.debug("Out of range")
            # return the treshold to not trigger anything
            return self._tconfig['distance_treshold_alexa'] * 2
        else:
            return distance

    def _readAnalogSensor(self, pinIn):
        reading = 0
        # Unload Capacitor
        GPIO.setwarnings(False)
        GPIO.setup(pinIn, GPIO.OUT)
        GPIO.output(pinIn, GPIO.LOW)
        time.sleep(0.3)
        
        # measure time until Voltage peaks
        GPIO.setup(pinIn, GPIO.IN)
        ## ~1ms per loop cycle
        while (GPIO.input(pinIn) == GPIO.LOW):
                reading += 1
        logger.debug("Analog Sensor read time: ~"+str(reading)+"ms")
        return reading

    def _checkLights(self):
        # The values we get from the analog sensor differ extremely,
        # wether the Pi is idle or playing (due to the extra CPU usage).
        # So if we are playing, let's assume the lights are on 
        if self._player is not None:
            playing = self._player.is_playing()
            if playing:
                return True
        # read sensor and compare value to treshold
        intensity = self._readAnalogSensor(self._tconfig['gpio_pin_lights'])
        lights_on = intensity < self._tconfig['lights_treshold']
        return lights_on
