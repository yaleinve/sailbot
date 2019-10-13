#!/usr/bin/python

import time
import mraa

def main():
    s = mraa.Spi()
    while True:
        header = s.write(bytearray([0]*4))
        if header[0] == 0x7E:  # Start delimiter, see XBee User Guide for full format
            length = header[1] << 8 + header[2]
            msg_type = header[3]
            contents = s.write(bytearray([0]*(length-2)))
            checksum = s.write(bytearray([0]))
            if msg_type == 0x90:
                print "Received: " + contents[11:]
                print "Full contents: " + repr(contents)
            else:
                print "Not a recieve: " + repr(contents)
        else:
            print "No message: " + repr(header)
        time.sleep(500);

if __name__ == "__main__":
    main()