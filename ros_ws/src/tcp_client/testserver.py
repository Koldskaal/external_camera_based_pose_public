import socket
import pickle

def start_tcp_server():
    # TCP connection parameters
    tcp_ip = '127.0.0.1'  # Server IP address (localhost)
    tcp_port = 5006       # Server port
    buffer_size = 1024    # Maximum packet size

    # Create a TCP socket
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.bind((tcp_ip, tcp_port))
    server_socket.listen(1)  # Listen for incoming connections

    print(f"Server started, listening on {tcp_ip}:{tcp_port}")

    # Wait for a connection
    conn, addr = server_socket.accept()
    print(f"Connection accepted from {addr}")

    try:
        while True:
            # Receive data from the client
            data = conn.recv(buffer_size)
            if not data:
                # If no data is received, break the loop
                print("Connection closed by client")
                break

            # Print the received data
            print("Received data:", pickle.loads(data))

    except KeyboardInterrupt:
        print("\nServer stopped by user")

    finally:
        # Close the connection and socket
        conn.close()
        server_socket.close()
        print("Server shut down")

if __name__ == '__main__':
    start_tcp_server()
