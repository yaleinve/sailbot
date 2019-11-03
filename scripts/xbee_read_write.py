#!/usr/bin/python

import sys
import time
import mraa

POLL_PERIOD = 0.5
DEBUG = False

def main():
    s = mraa.Spi(0)
    # Setting the freqency is necessary to get meaningful values, but it's unclear what the
    # default frequency is and why it doesn't work.
    s.frequency(1000000)
    while True:
        finish_read(s, bytearray())
        time.sleep(POLL_PERIOD);

def finish_read(spi, buf):
    start_loc = buf.find(0x7E)  # Start delimiter, see XBee User Guide for full format.
    if start_loc != -1:
        buf = buf[start_loc:]
    else:
        buf = bytearray()

    while True:
        start, buf = buffered_read(spi, buf, 1)[0]
        if start != 0x7E:
            break

        header, buf = buffered_read(spi, buf, 3)
        length = (header[0] << 8) + header[1]
        assert header[2] == 0x90, "Message type not receive: %02x" % header[2]

        contents, buf = buffered_read(spi, buf, length - 1)
        checksum, buf = buffered_read(spi, buf, 1)  # TODO: verify checksum

        print "Received: %s" % contents[11:]  # Message text starts at 11th byte

def buffered_read(spi, buf, count):
    if count <= len(buf):
        return buf[:count], buf[count:]
    elif len(buf) > 0:
        new_bytes = spi.write(bytearray([0] * (count - len(buf))))
        return buf + new_bytes, bytearray()
    else:
        return spi.write(bytearray([0] * count)), bytearray()

def debug_print(string):
    if DEBUG:
        print string

if __name__ == "__main__":
    main()
