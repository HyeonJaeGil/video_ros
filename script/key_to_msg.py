#! /usr/bin/env python

from __future__ import print_function

import threading
import rospy
from std_msgs.msg import Empty

import sys, tty, select, termios


keyLists = {'c'}


class PublishThread(threading.Thread):
    def __init__(self, rate):
        super(PublishThread, self).__init__()
        self.publisher = rospy.Publisher("topic_name", Empty, queue_size=1)
        self.condition = threading.Condition()

        self.timeout = None

        self.start()
        self.done = False

    def wait_for_subsciber(self):
        while not rospy.is_shutdown() and self.publisher.get_num_connections()==0:
            pass
        if rospy.is_shutdown():
            raise Exception("Got shutdown request before subscriber is connected")
    
    def run(self):
        empty = Empty()
        while not self.done():
            self.condition.acquire()
            self.condition.wait(self.timeout)

            # do something

            self.condition.release()

            self.publisher.publish(empty)

def getKey(key_timeout):
    tty.setraw(sys.stdin.fineno())
    rlist, _, _ = select.select([sys.stdin], [], [], key_timeout)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)
    rospy.init_node('teleop')

    pub_thread = PublishThread()

    try:
        pub_thread.wait_for_subsciber()
        
        # print(msg)

        while(1):
            key = getKey(key_timeout=0.0)
            if key in keyLists:
                pass

            else:
                if (key == '\x03'):
                    break

    except Exception as e:
        print(e)

    finally:
        # pub_thread.stop()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
            


    




