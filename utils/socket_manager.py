import numpy as np
import struct

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

def recv_msg(sock):
    raw_length = sock.recv(4)
    if not raw_length:
        return None
    msg_len = struct.unpack('I', raw_length)[0]
    data = b''
    while len(data) < msg_len:
        chunk = sock.recv(msg_len - len(data))
        if not chunk:
            raise RuntimeError("Socket connection broken")
        data += chunk
    return data.decode()