#!/usr/bin/env python

import rospy
from beeper import Beeper
from std_msgs.msg import String
from pbot_beeper.msg import Beep


class BeeperWrapper:
    def __init__(self):
        self.beeper = Beeper()
        rospy.init_node('beeper')
        rospy.Subscriber("/pbot/beeper_topic", Beep, self.callback_beep)

    def callback_beep(self, msg: Beep):
        rospy.loginfo(rospy.get_caller_id() + "I heard %s", msg)

        if msg.type == 2:
            # This is melody
            melody = []
            for note in msg.melody:
                if not self.beeper.note_exists(note.note):
                    rospy.logerr("Note %s not existent", note.note)
                else:
                    melody.append((note.note, note.duration))
            self.beeper.beep_melody(melody, msg.tempo)
        else:
            # This is default beep
            rospy.loginfo("Play beep")
            self.beeper.beep()

    def cleanup(self):
        self.beeper.cleanup()


def start():
    b = BeeperWrapper()
    rospy.on_shutdown(b.cleanup)

    rospy.loginfo("Beeper node started")
    rospy.spin()


if __name__ == '__main__':
    start()
