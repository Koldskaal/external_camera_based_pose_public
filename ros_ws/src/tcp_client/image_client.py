import cv2
import socket
import struct

HOST = ""
PORT = 8488


def main():
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    # client_socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1) # experiment to reduce lag
    client_socket.connect((HOST, PORT))
    while True:
        if cv2.waitKey(1) & 0xFF == ord("q"):
            client_socket.close()
            break

        data = client_socket.recv(6)
        x, y, theta = struct.unpack("!hhh", data)
        print(f"x: {x}, y:{y}, theta:{theta}")


if __name__ == "__main__":
    main()
