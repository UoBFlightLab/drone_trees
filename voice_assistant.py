#!/usr/bin/env python

import pyttsx3
from queue import Queue, Empty
import threading
import time

class VoiceAssistant(threading.Thread):
    def __init__(self):
        super(VoiceAssistant, self).__init__(daemon=True)
        self._engine = pyttsx3.init()
        self._q = Queue()
        self._loop_should_exit = False
    
    def kill(self):
        self._loop_should_exit = True
        print('----------Kill-----------')

    def add_say(self, msg):
        self._q.put(msg)
        print("[add_say] Adding: \"" + msg + "\"")

    def run(self):
        while not self._loop_should_exit:
            try:
                self._engine.say(self._q.get(False))
                self._engine.startLoop(False)
                self._engine.iterate()
                time.sleep(1)
                self._engine.endLoop()
                self._q.task_done()
            except Empty:
                pass
