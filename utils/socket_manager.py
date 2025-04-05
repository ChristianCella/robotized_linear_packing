import numpy as np

def send_array(sock, array):
    # Send the shape and type of the array first
    shape = np.array(array.shape, dtype = np.int32)
    sock.sendall(shape.tobytes())
    sock.sendall(array.tobytes())

def send_strings(sock, strings):
    count = np.array([len(strings)], dtype=np.int32)
    sock.sendall(count.tobytes())

    for text in strings:
        encoded = text.encode('utf-8')
        length = np.array([len(encoded)], dtype=np.int32)
        sock.sendall(length.tobytes())
        sock.sendall(encoded)