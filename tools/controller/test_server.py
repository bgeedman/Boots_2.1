import socket
import struct
import commands_pb2

def main():
    print "Creating new testing server"
    host = '127.0.0.1'
    port = 15000
    server = socket.socket()
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind((host, port))
    server.listen(1)
    conn, addr = server.accept()

    print "New connection from {}".format(addr)
    while (True):
        data = conn.recv(4) # read in 4 byte length
        if (not data):
            break
        data_len, = struct.unpack('!i', data)
        print "Attempting to read {} bytes".format(data_len)
        data = conn.recv(data_len)
        if (not data):
            break
        command = commands_pb2.Stretch()
        command.ParseFromString(data)
        print "Data: {}".format(command)
    server.close()

if __name__ == "__main__":
    main()
