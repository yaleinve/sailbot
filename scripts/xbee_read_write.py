#!/usr/bin/python

import sys
import time
import threading
import mraa

POLL_PERIOD = 0.5
DEBUG = False
PARTNER_ADDR = [0x00, 0x13, 0xA2, 0x00, 0x41, 0x94, 0x16, 0x59]


def main():
    spi = mraa.Spi(0)
    # Setting the freqency is necessary to get meaningful values, but it's unclear what the
    # default frequency is and why it doesn't work.
    spi.frequency(1000000)

    read_lock = threading.Lock()

    poll_thread = threading.Thread(target=poll, args=(spi, read_lock))
    poll_thread.daemon = True
    poll_thread.start()

    while True:
        to_send = raw_input("Type a message:")

        read_lock.acquire()

        read_buf = send_message(spi, to_send)
        finish_read(spi, read_buf)

        read_lock.release()


def poll(spi, read_lock):
    while True:
        read_lock.acquire()
        finish_read(spi, bytearray())
        read_lock.release()

        time.sleep(POLL_PERIOD)


def send_message(spi, msg):
    transmit_header = bytearray([0x10, 0x00]) + bytearray(PARTNER_ADDR) + bytearray([0xFF, 0xFE, 0x00, 0x00])  # See XBee User Guide p. 242

    transmit_contents = transmit_header + bytearray(msg)
    checksum = calc_checksum(transmit_contents)

    length = len(transmit_contents)
    general_header = bytearray([0x7E, length >> 8, length & 0xFF])

    bytes_to_transmit = general_header + transmit_contents + bytearray([checksum])
    return spi.write(bytes_to_transmit)


def finish_read(spi, buf):
    start_loc = buf.find(bytearray([0x7E]))  # Start delimiter, see XBee User Guide for full format.
    if start_loc != -1:
        buf = buf[start_loc:]
    else:
        buf = bytearray()

    while True:
        start, buf = buffered_read(spi, buf, 1)
        if start[0] != 0x7E:
            break
        debug_print("Started read")

        header, buf = buffered_read(spi, buf, 3)
        length = (header[0] << 8) + header[1]
        assert header[2] == 0x90, "Message type not receive: %02x" % header[2]

        contents, buf = buffered_read(spi, buf, length - 1)
        checksum, buf = buffered_read(spi, buf, 1)
        expected_checksum = calc_checksum(header[2:] + contents)
        assert checksum[0] == expected_checksum, "Incorrect checksum: received %02x, calculated %02x" % (checksum[0], expected_checksum)

        print "Received: %s" % contents[11:]  # Message text starts at 11th byte


def buffered_read(spi, buf, count):
    if count <= len(buf):
        return buf[:count], buf[count:]
    elif len(buf) > 0:
        new_bytes = spi.write(bytearray([0] * (count - len(buf))))
        debug_print("Read " + repr(new_bytes))
        return buf + new_bytes, bytearray()
    else:
        new_bytes = spi.write(bytearray([0] * count))
        debug_print("Read " + repr(new_bytes))
        return new_bytes, bytearray()


def calc_checksum(contents):
    sum_ = sum(contents)
    return 0xFF - (sum_ & 0xFF)


def debug_print(string):
    if DEBUG:
        print string


if __name__ == "__main__":
    main()
