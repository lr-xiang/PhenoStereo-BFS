from __future__ import print_function
import socket
import datetime, time
import json
import threading
import sys

import base64
import cv2
import numpy as np

MSGLEN = 1024*1024

class MySocket:
    """demonstration class only
      - coded for clarity, not efficiency
    """

    header_length = 8

    def __init__(self, sock=None):
        if sock is None:
            self.sock = socket.socket(
                            socket.AF_INET, socket.SOCK_STREAM)
        else:
            self.sock = sock

        self.header_length = 8

    def connect(self, host, port):
        self.sock.connect((host, port))

    def compose_msg(self, msg_dict):
        body_str = json.dumps(msg_dict)
        msg_len = len(body_str)
        msg_str = str(msg_len).zfill(self.header_length) + body_str
        return msg_str
    
    def decode_msg(self, msg_str):
        header = msg_str[0:self.header_length]
        body_len = int(header)
        if body_len != (len(msg_str) - self.header_length):
            print("decoded:", body_len, ", actual_len: ", len(msg_str) - self.header_length)
        body_str = msg_str[self.header_length:]
        msg_dict = json.loads(body_str)
        return msg_dict

    def mysend(self, msg):
        sent = self.sock.send(bytes(msg, 'utf8'))
        if sent == 0:
            raise RuntimeError("socket connection broken")

    def mysend_msg(self, msg):
        msg_str = self.compose_msg(msg)
        print("[send]:", msg_str)
        self.mysend(msg_str)

    def myreceive(self):
        chunks = []
        
        bytes_recd = 0
        while self.thread_running:
            try:
                chunk = self.sock.recv(4096)
            except socket.timeout:
                continue
            except:
                print("Unexpected error:", sys.exc_info()[0])
                raise
                
            if chunk == b'':
                raise RuntimeError("socket connection broken")
            chunks.append(chunk)

            if len(chunk) < 4096:
                break;
            bytes_recd = bytes_recd + len(chunk)
        
        msg_part = b''.join(chunks)
        self.msg_recv = self.msg_recv + msg_part.decode("utf8")
        #msg_dict = self.decode(msg_recv)
        #print("Msg recv, start decoding, msg length: ", len(self.msg_recv))
        decoded = self.process_msg(self.msg_recv)
        if decoded:
            self.msg_recv = ""
        

    def run_receive_worker(self):
        self.sock.settimeout(2);
        self.thread_running = True
        self.msg_recv = ""
        self.receive_thread = threading.Thread(target=self.receive_worker, args=())
        self.receive_thread.start()
        print("Recv thread started")

    def receive_worker(self):
        
        try:
            while self.thread_running:
                self.myreceive()
                #time.sleep(.010)
        except:
            print("thread exception", sys.exc_info())
            exit()

    # Decode the messages, and do work
    def process_msg(self, msg):
        if msg =="":
            return False
        try:
            msg_data = json.loads(msg)
        except json.decoder.JSONDecodeError:
            return False

        if msg_data["command"] == "images":
            images = []
            n_image = msg_data["n_imgs"]
            print("Recved ", n_image, " images.")
            for encoded_img in msg_data["imgs"]:
                v_width = encoded_img["width"]
                v_height = encoded_img["height"]
                v_type = encoded_img["type"]
                v_size = encoded_img["size"]
                decoded = base64.b64decode(encoded_img["data"])
                #print(decoded[:16])
                cvmat = np.frombuffer(decoded, dtype=np.uint8)
                cvmat = cvmat.reshape((v_height, v_width))
                cvmat = cv2.cvtColor(cvmat, cv2.COLOR_BAYER_RG2BGR)
                cvmat = cv2.cvtColor(cvmat, cv2.COLOR_BGR2RGB)
                print(cvmat.shape)
                cv2.imshow('image',cvmat)
                cv2.waitKey(100)
        else:
            print("[Recv]:" + msg)
        return True

if __name__ == "__main__":
    
    #print(idx_tp)
    ct = datetime.datetime.now()
    t = ct.strftime("%Y.%m.%d-%H.%M.%S.%f")[:-3]
    print(t)

    # Predefined messages
    msg_connect = {"command":"connect", "data":""}
    msg_sync = {"command":"sync", "data":t}
    msg_set_paras = {"command":"set_paras", "data":""}
    msg_set_exposure = {"command":"set_exposure", "data":10000.0}
    msg_start_capturing = {"command":"start_capturing", "data":""}
    msg_single_trigger = {"command":"single_trigger", "data":""}
    msg_cont_trigger = {"command":"cont_trigger", "data": -1}
    msg_disconnect = {"command":"disconnect", "data":""}

    sock = MySocket();
    sock.connect("192.168.0.120", 2018);
    sock.run_receive_worker()
    map_msg = {
            'a': msg_connect,
            's': msg_sync,
            'd': msg_set_paras,
            'f': msg_set_exposure,
            'g': msg_start_capturing,
            'h': msg_single_trigger,
            'j': msg_cont_trigger,
            'k': msg_disconnect
        }
    while 1:
        print("Input command to do:")
        print("-----'a': Connect")
        print("-----'s': Sync")
        print("-----'d': Set parameters")
        print("-----'f': Set exposure")
        print("-----'g': start capturing")
        print("-----'h': single trigger")
        print("-----'j': Continuous trigger (10)")
        print("-----'k': Disconnect")
        print("----'q': Exit program")
        c = input()

        dict_key = c[0]
        if dict_key in map_msg:
            sock.mysend_msg(map_msg[dict_key])
        else:
            if dict_key == 'q':
                break

            print("Key not implemented")
    sock.thread_running = False
    sock.receive_thread.join()
    print("Program ends")
        #msg = bytes("20180000","utf8")
        #sock.mysend(msg)
        #print(msg)
        
    

