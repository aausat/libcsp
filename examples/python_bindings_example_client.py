#!/usr/bin/python3

# Build required code:
# $ ./examples/buildall.py
#
# Start zmqproxy (only one instance)
# $ ./build/zmqproxy
#
# Run client against server using ZMQ:
# $ LD_LIBRARY_PATH=build PYTHONPATH=build python3 examples/python_bindings_example_client.py -z localhost
#

import os
import time
import sys
import argparse

import libcsp_py3 as libcsp


def getOptions():
    parser = argparse.ArgumentParser(description="Parses command.")
    parser.add_argument("-a", "--address", type=int, default=10, help="Local CSP address")
    parser.add_argument("-c", "--can", help="Add CAN interface")
    parser.add_argument("-z", "--zmq", help="Add ZMQ interface")
    parser.add_argument("-k", "--kiss", help="Add KISS interface")
    parser.add_argument("-s", "--server-address", type=int, default=27, help="Server address")
    parser.add_argument("-R", "--routing-table", help="Routing table")
    return parser.parse_args(sys.argv[1:])


if __name__ == "__main__":

    options = getOptions()
    libcsp.init(options.address, "host", "model", "1.2.3", 10, 300)

    if options.kiss:
        print("Nick er sej")
        #libcsp.kiss_init(options.kiss)
        libcsp.kiss_init(options.kiss, 115200, 512, "KISS")
        libcsp.rtable_load("0/0 KISS")
    if options.can:
        libcsp.can_socketcan_init(options.can)
    if options.zmq:
        libcsp.zmqhub_init(options.address, options.zmq)
        libcsp.rtable_load("0/0 ZMQHUB")
    if options.routing_table:
        libcsp.rtable_load(options.routing_table)

    print("Options:")
    print(options)

    libcsp.route_start_task()
    time.sleep(0.2)  # allow router task startup

    print("Connections:")
    libcsp.print_connections()

    print("Routes:")
    libcsp.print_routes()

    print("CMP ident:", libcsp.cmp_ident(options.server_address))

    print("Ping: %d mS" % libcsp.ping(options.server_address))
    print("Ping: %d mS" % libcsp.ping(options.server_address))
    print("Ping: %d mS" % libcsp.ping(options.server_address))
    libcsp.reboot(options.server_address)
    print("Ping: %d mS" % libcsp.ping(options.server_address))

    # transaction
    #outbuf = bytearray().fromhex('01')
    #inbuf = bytearray(1)
    #print ("Exchange data with server using csp_transaction ...")
    #libcsp.transaction(0, options.server_address, 10, 1000, outbuf, inbuf)
    #print ("  got reply from server [%s]" % (''.join('{:02x}'.format(x) for x in inbuf)))

    s = "ABCD"
    outbuf = bytearray().fromhex('01')
    outbuf.extend(map(ord, s))
    a = b'fisk'
    conn_cap = libcsp.connect(libcsp.CSP_PRIO_NORM, options.server_address,
            options.address, 1000,  libcsp.CSP_O_NONE)

    # This csp_send segfaults in strcmp of get_capsule_pointer
    data = bytearray()
    mybuf = libcsp.buffer_get(100)
    print(type(mybuf))
    libcsp.packet_set_data(mybuf,outbuf)
    #TODO create connection before send...
    #libcsp.connect(options.server_address:


    libcsp.send(conn_cap, mybuf)
