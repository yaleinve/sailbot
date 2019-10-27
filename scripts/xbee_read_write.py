#!/usr/bin/python

import time
import mraa

def main():
    s = mraa.Spi(0)
    s.frequency(1000000)
    while True:
        header = s.write(bytearray([0]*5))
        if header[1] == 0x7E:  # Start delimiter, see XBee User Guide for full format.
            # Header begins at second byte after XBee recieves message, highly likely
            # to be the second byte in a 5-byte sequence.
	    print "Header: " + repr(header)
            length = (header[2] << 8) + header[3]
            print "Length: " + str(length)
            msg_type = header[4]
            contents = s.write(bytearray([0]*(length-1)))
            checksum = s.write(bytearray([0]))
            if msg_type == 0x90:
                print "Received: " + contents[11:]
                print "Full contents: " + repr(contents)
            else:
                print "Not a recieve: " + repr(contents)
        else:
            print "No message: " + repr(header)
        time.sleep(0.5);

if __name__ == "__main__":
    main()
