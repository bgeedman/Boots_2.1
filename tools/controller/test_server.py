import socket
import struct

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
        data = conn.recv(1) # read in 4 byte length
        if (not data):
            break
        print data
        data_len, = struct.unpack('!i', data)
        print "Attempting to read {} bytes".format(data_len)
        data = conn.recv(data_len)
        if (not data):
            break
        data = struct.unpack('!fffbb', data)
        print "Data: {}".format(data)
    server.close()

if __name__ == "__main__":
    main()
