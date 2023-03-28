#!/usr/bin/env python3

# import cv2
# import numpy
# from PIL import Image
import io
import cmd_udp
import time
from datetime import datetime
from threading import Timer, Lock

from netcl_tcp import netcl_tcp
from v720_ap import v720_ap
from config import *

import sys

def eprint(*args, **kwargs):
    print(*args, file=sys.stderr, **kwargs)

WAV_HDR = b'RIFF\x8a\xdc\x01\x00WAVEfmt \x12\x00\x00\x00\x06\x00\x01\x00@\x1f\x00\x00@\x1f\x00\x00\x01\x00\x08\x00\x00\x00fact\x04\x00\x00\x006\xdc\x01\x00LIST\x1a\x00\x00\x00INFOISFT\x0e\x00\x00\x00Lavf58.45.100\x00data6\xdc\x01\x00'

last_img = None
writer_lock = Lock()

frame_time = time.time()
frame_cnt_vid = 0
frame_cnt_aud = 0

def show_img(frame: bytearray):
    global frame_time, last_img, writer_lock
    t = time.time()
    fps = round(1 / (t - frame_time), 2)
    
    """
    writer_lock.acquire()
    last_img = numpy.array(Image.open(io.BytesIO(frame)))
    cv2.putText(last_img, str(datetime.now()), (5, 15),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (100, 255, 0), 1, cv2.LINE_AA)
    cv2.putText(last_img, f'FPS: {fps}', (5, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (100, 255, 0), 1, cv2.LINE_AA)
    writer_lock.release()
    cv2.imshow('Frame', last_img)
    """
    writer_lock.acquire()
    last_img = bytearray(len(frame))
    last_img[:] = frame
    writer_lock.release()

    frame_time = t

def show_live(cam: v720_ap, videofile: str = None, audiofile: str = None):

    global frame_cnt_vid, frame_cnt_aud
###    cv2.startWindowThread()
###    cv2.namedWindow('Frame')

    print('Press CTRL-C to exit')
    _video = None
    if videofile is not None:
###        _video = cv2.VideoWriter(videofile, cv2.VideoWriter_fourcc(
###            'M', 'J', 'P', 'G'), 10, (640, 480))
        _video = open(videofile, 'wb')
        # _video.write(RIFF_HDR)

        def _save_video():
            global writer_lock, last_img
            if last_img is not None:
                writer_lock.acquire()
                _video.write(last_img)
                last_img = None
                writer_lock.release()
            
            _v_writer_tmr = Timer(0.1, _save_video)
            _v_writer_tmr.setDaemon(True)
            _v_writer_tmr.start()

        _save_video()

    _audio = None
    if audiofile is not None:
        _audio = open(audiofile, 'wb')
        _audio.write(WAV_HDR)

    try:
        sync = False
        frame_cnt_vid = 0
        frame_cnt_aud = 0
        frame = bytearray()
        start_time = datetime.now()

        def on_rcv(cmd, data: bytearray):
            nonlocal sync
            global frame_cnt_vid, frame_cnt_aud
            data_len = len(data)
            if data_len == 0:
                print('No data')
                eprint('No data')
                return
            
            if data_len > 32:
                hex_len = 32
            else:
                hex_len = data_len
            
            eprint(f'on_rcv[{data_len}]: {data[:hex_len].hex()}')
            if cmd == cmd_udp.P2P_UDP_CMD_JPEG:
                """
                if not sync:
                    f = data.find(b'\xff\xd8')
                    eprint(f'sync: {f} {len(frame)}')
                    if f >= 0:
                        frame.extend(data[f:])
                        sync = True
                else:  # sync == true
                    f = data.find(b'\xff\xd9')
                    eprint(f'eof: {f} {len(frame)}')
                    if f < 0:
                        frame.extend(data)
                    else:
                        frame.extend(data[:f+2])
                        eprint(f'{len(frame)} {data[f:len(data)].hex()}')
                        if len(data) > (f + 6): 
                            jpeg_len = data[f+3] + (data[f+4] << 8) + (data[f+5] << 16) + (data[f+6] << 24)
                            eprint(f'{jpeg_len}')
                        else:
                            eprint('no jpeg_len!')
                            print(f'no jpeg_len! len={len(frame)} f={f}')
                        show_img(frame)
                        frame.clear()
                        sync = False
                """
                if not sync:
                    if data_len < 2:
                        print('data chunk too small')
                        return
                    # JPEG start marker
                    if (data[0] == 0xff) and (data[1] == 0xd8):
                        sync = True
                    else:
                        return

                frame.extend(data)
                # Sanity
                if len(frame) > 0x400000: # 4MB
                    print(f'jpeg frame too long: {len(frame)}')
                    frame.clear()
                    sync = False
                    pass

                tot = len(frame) - 5
                # JPEG end marker
                if (tot > 2) and (frame[tot-2] == 0xff) and (frame[tot-1] == 0xd9):
                    jpeg_len = frame[tot+1] + (frame[tot+2] << 8) + (frame[tot+3] << 16) + (frame[tot+4] << 24)
                    if jpeg_len != tot:
                        print(f'jpeg_len={jpeg_len} total={tot}')
                    show_img(frame[:tot])
                    frame.clear()
                    sync = False
                    frame_cnt_vid = frame_cnt_vid + 1

            elif cmd == cmd_udp.P2P_UDP_CMD_G711 and _audio is not None:
                _audio.write(data)
                frame_cnt_aud = frame_cnt_aud + 1
                elapsed = datetime.now() - start_time
                print(f"{elapsed}", end = '\r')
        
        cam.cap_live(on_rcv)

    except KeyboardInterrupt:
        if _video is not None:
            ### _video.release()
            _video.flush()
            _video.close()
        if _audio is not None:
            _audio.flush()
            _audio.close()

        print(f"Frames received: video={frame_cnt_vid} audio={frame_cnt_aud}")
        return


if __name__ == '__main__':
    with netcl_tcp(AP_HOST, PORT) as sock:
        cam = v720_ap(sock)
        cam.init_live_motion()
        show_live(cam, 'live.mjpeg', 'live.wav')
