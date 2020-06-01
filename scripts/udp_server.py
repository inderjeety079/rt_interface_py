#!/usr/bin/env python2.7
from threading import Event
import socket
import json
import struct
import logging
import sys
import Queue



class UdpServer:
    def __init__(self, hostname, port, max_connections=5):
        self.logger = logging.getLogger('UdpServer')
        self.hostname = hostname
        self.port = port
        self.recv_buffer_size = 4096
        self.max_pending_connections = max_connections
        self.raw_data = ''
        self.data_bytearray = bytearray(self.recv_buffer_size)
        self.recv_queue = Queue.Queue()

        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.logger.info('Socket created')
        except socket.error as err:
            self.logger.error("Failed to create socket: {}".format(err[1]))

    def bind(self):
        try:
            self.socket.bind((self.hostname, self.port))
            self.logger.info('Listening at ({}, {})'.format(self.hostname, self.port))

        except socket.error as err:
            self.logger.error('Error binding socket: {}'.format(err[1]))
            self.logger.info('Exiting..')
            # sys.exit(-1)

    def accept_connection(self):
        self.socket.listen(self.max_pending_connections)
        try:
            self.logger.info('Waiting for connections')
            client_sock, address = self.socket.accept()
            self.logger.info('Accepted connection from {}:{}'.format(address[0], address[1]))
            return client_sock
        except socket.error as err:
            self.logger.error('Exception accepting connecton: {}'.format(err[1]))

    def recv_msg(self):
        data, address = self.socket.recvfrom(4096)
        #self.raw_data = self.raw_data + data
        self.data_bytearray = data
        #self.recv_queue.put(self.data_bytearray)
        self.recv_queue.put(data)
        chunk_size = len(data)
        self.logger.info("received data from : {}, {}: data: {}".format(address[0], address[1], self.data_bytearray))
        return data

    def send_msg(self, client, msg, length):
        self.socket.send_to(msg, (self.hostname, self.port))

    def close_socket(self):
        self.logger.info("Closing UDP socket: {}".format(self.hostname, self.port))
        self.socket.close()

