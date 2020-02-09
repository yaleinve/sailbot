#!/usr/bin/env python2.7

import functools
import json
import os
import sys
import time
import threading

import mraa
import rospy

from captain.msg import LegInfo

POLL_PERIOD = 0.5
PARTNER_ADDR = [0x00, 0x13, 0xA2, 0x00, 0x41, 0x94, 0x16, 0x59]
TOPIC_DEFS = [
    ('/leg_info', LegInfo),
]


class XBee:

    def __init__(self):
        self.recv_mapping = {topic: (type_, rospy.Publisher(topic, type_, queue_size=10))
                        for (topic, type_) in TOPIC_DEFS}

        self.spi = mraa.Spi(0)
        # Setting the freqency is necessary to get meaningful values, but it's unclear what the
        # default frequency is and why it doesn't work.
        self.spi.frequency(1000000)

        self.recv_buf = bytearray()
        self.lock = threading.Lock()

        poll_thread = threading.Thread(target=self.poll)
        poll_thread.daemon = True
        poll_thread.run()

    def poll(self):
        while True:
            self.lock.acquire()
            self.finish_read()
            self.lock.release()

            time.sleep(POLL_PERIOD)

    def generic_transmit(self, topic, type_, msg_obj):
        msg_json = {'__topic__': topic}
        for field in msg_obj.__slots__:
            msg_json[field] = getattr(msg_obj, field)
        self.send_message(json.dumps(msg_json))
        self.finish_read()

    def generic_receive(self, msg_dump):
        msg_json = json.loads(msg_dump)
        type_, pub = self.recv_mapping[msg_json['__topic__']]
        msg_obj = type_()
        for key, value in msg_json.items():
            if key[:2] != '__':
                setattr(msg_obj, key, value)
        pub.publish(msg_obj)

    def send_message(self, msg):
        transmit_header = bytearray([0x10, 0x00]) + bytearray(PARTNER_ADDR) + bytearray([0xFF, 0xFE, 0x00, 0x00])  # See XBee User Guide p. 242

        transmit_contents = transmit_header + bytearray(msg)
        checksum = self.calc_checksum(transmit_contents)

        length = len(transmit_contents) + 1
        general_header = bytearray([0x7E, length >> 8, length & 0xFF])

        bytes_to_transmit = general_header + transmit_contents + checksum
        self.recv_buf += self.spi.write(bytes_to_transmit)


    def finish_read(self):
        start_loc = self.recv_buf.find(0x7E)  # Start delimiter, see XBee User Guide for full format.
        if start_loc != -1:
            self.recv_buf = self.recv_buf[start_loc:]
        else:
            self.recv_buf = bytearray()

        while True:
            start = buffered_read(1)
            if start != 0x7E:
                break

            header, buf = buffered_read(buf, 3)
            length = (header[0] << 8) + header[1]
            assert header[2] == 0x90, "Message type not receive: %02x" % header[2]

            contents, buf = buffered_read(buf, length - 1)
            checksum, buf = buffered_read(buf, 1)[0]
            expected_checksum = calc_checksum(contents)
            assert checksum == expected_checksum, "Incorrect checksum: received %02x, calculated %02x" % (checksum, expected_checksum)

            print "Received: %s" % contents[11:]  # Message text starts at 11th byte
            self.generic_receive(contents[11:])

    def buffered_read(self, count):
        if count <= len(self.recv_buf):
            to_return = self.recv_buf[:count]
            self.recv_buf = self.recv_buf[count:]
            return to_return
        else:
            new_bytes = self.spi.write(bytearray([0] * (count - len(self.recv_buf))))
            to_return = self.recv_buf + new_bytes
            self.recv_buf = bytearray()
            return to_return

    @staticmethod
    def calc_checksum(contents):
        sum_ = sum(contents)
        return 0xFF - (sum_ & 0xFF)


def debug_print(string):
    if os.environ.get('DEBUG') is not None:
        print string


if __name__ == '__main__':
    rospy.init_node('xbee')
    x = XBee()

    transmit_topics = [
        ('/leg_info', LegInfo, x.generic_transmit),
    ]

    for (topic, type_, fn) in transmit_topics:
        rospy.Subscriber(topic, type_, functools.partial(fn, topic, type_))
    rospy.spin()
