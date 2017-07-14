import threading
import logging
import time

import RPi.GPIO as GPIO

import alexapi.triggers as triggers
from .basetrigger import BaseTrigger

logger = logging.getlogger(__name__)


class UltrasonicTrigger(BaseTrigger):

    type = triggers.TYPES.OTHER

    def __init__(self, config, trigger_callback):
        super(UltrasonicTrigger, self).__init__(config, trigger_callback, 'ultrasonic')

        self._enabled_lock = threading.Event()
        self._disabled_sync_lock = threading.Event()

    def setup(self):
        # Setup GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self._tconfig['gpio_pin_echo'], GPIO.IN)
        GPIO.setup(self._tconfig['gpio_pin_trigger'], GPIO.OUT)

    def run(self):
        # Start Thread
        thread = threading.Thread(target=self.thread, args=())
        thread.setDaemon(True)
        thread.start()

    def thread(self):
        while True:
            self._enabled_lock.wait()

            triggered = False
            voice_command = self._config['triggers']['pocketsphinx']['phrase']

            while not triggered:
                if not self._enabled_lock.isSet():
                    break
                # Ping
                self.ping()
                # Measure Distance
                distance = self.measure_distance()
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
                    # do not stress CPU and Sensor too much
                    time.sleep(1)

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
    def ping():
        # settle Sensor
        GPIO.output(self._tconfig['gpio_pin_trigger'], GPIO.LOW)
        time.sleep(0.2)
        # do Ping
        GPIO.output(self._tconfig['gpio_pin_trigger'], GPIO.HIGH)
        time.sleep(0.00001)
        GPIO.output(self._tconfig['gpio_pin_trigger'], GPIO.LOW)

    def measure_distance():
        # get length of pulse
        pulseStart = time.time()
        pulseEnd = time.time()
        
        while GPIO.input(self._tconfig['gpio_pin_echo']) == GPIO.LOW:
            pulseStart = time.time()
        
        while GPIO.input(self._tconfig['gpio_pin_echo']) == GPIO.HIGH:
            pulseEnd = time.time()
        
        duration = pulseEnd - pulseStart
        
        #logger.debug("Duration: "+str(round(duration*1000, 1))+"ms")

        ## speed of sound @ 20degC = 343.5 m/s
        distance = duration * 34350 / 2 # in centimeters
        distance = round(distance, 1)
        logger.debug("Distance: "+str(distance)+"cm")
        
        if distance >= 200 or distance < 0:
            # Sensor gets inaccurate
            logger.debug("Out of range")
            # return the treshold to not trigger anything
            return self._tconfig['distance_treshold_alexa'] * 2
        else:
            return distance
