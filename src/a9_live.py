#!/usr/bin/env python3

import io
import cmd_udp
import time
from datetime import datetime
import subprocess
import os
from threading import Timer, Lock

from netcl_tcp import netcl_tcp
from v720_ap import v720_ap
from config import *

import sys

def eprint(*args, **kwargs):
    print(*args, file=sys.stderr, **kwargs)

frame_time = time.time()
last_img = None
writer_lock = Lock()


def show_live(cam: v720_ap):

    frame_cnt_vid = 0
    frame_cnt_aud = 0
    frames_repeat = 0
    last_frame = bytearray()

    print('Press CTRL-C to exit')

    # Names of the "Named pipes"
    pipe1 = "/tmp/pipe_video"
    pipe2 = "/tmp/pipe_audio"

    # Create "named pipes".
    ### if not stat.S_ISFIFO(os.stat(pipe1).st_mode):
    if not os.path.exists(pipe1):
        os.mkfifo(pipe1)  # Create the pipe only if not exist.

    ### if not stat.S_ISFIFO(os.stat(pipe2).st_mode):
    if not os.path.exists(pipe2):
        os.mkfifo(pipe2)

    # Open FFmpeg as sub-process
    # Use two input streams:
    # 1. Named pipe: "pipe_video"
    # 2. Named pipe: "pipe_audio"
    # Merge the two streams.
    # Store the result to output file: output.mp4
    process = subprocess.Popen(["ffmpeg", '-loglevel', 'error',  #'debug'
                                "-f", "mjpeg", "-r", "10",
                                "-i", pipe1,
                                "-f", "alaw", "-ar", "8k", "-ac", "1", "-analyzeduration", "1000",
                                "-i", pipe2,
                                "-y", "output.mp4"],
                                #stdin=subprocess.PIPE)
                                stdin=None)

    fd1 = os.open(pipe1, os.O_WRONLY)  # fd1 is a file descriptor (an integer)
    # we have to delay audio pipe open because ffmpeg will open it after analizing video stream
    #fd2 = os.open(pipe2, os.O_WRONLY)  # fd2 is a file descriptor (an integer)
    fd2 = -1

    try:
        sync = False
        frame = bytearray()
        start_time = datetime.now()
        count = 0

        def on_rcv(cmd, data: bytearray):
            nonlocal fd1, fd2
            nonlocal sync
            nonlocal count

            nonlocal frame_cnt_vid, frame_cnt_aud
            nonlocal frames_repeat
            
            global writer_lock

            data_len = len(data)
            if data_len == 0:
                print('\nNo data')
                return
            
            if data_len > 32:
                hex_len = 32
            else:
                hex_len = data_len
            
            eprint(f'on_rcv({cmd},{data_len}): {data[:hex_len].hex()} ... {data[data_len-8:data_len].hex()}')
            if cmd == cmd_udp.P2P_UDP_CMD_JPEG:
                ### eprint(f'{sync},{len(frame)}')
                if not sync:
                    if data_len < 2:
                        print('\ndata chunk too small')
                        return
                    # JPEG start marker
                    if (data[0] == 0xff) and (data[1] == 0xd8):
                        sync = True
                    else:
                        return

                # Sanity
                if len(frame) > 0x400000: # 4MB
                    print(f'\njpeg frame too long: {len(frame)}')
                    frame.clear()
                    sync = False
                    return

                ### Do we need this lock?
                ### writer_lock.acquire()
                
                frame.extend(data)

                tot = len(frame) - 5
                ### eprint(f'tot={tot} len={len(frame)}  {frame[tot-2]} {frame[tot-1]}')
                # JPEG end marker
                if (tot > 2) and (frame[tot-2] == 0xff) and (frame[tot-1] == 0xd9):
                    jpeg_len = frame[tot+1] + (frame[tot+2] << 8) + (frame[tot+3] << 16) + (frame[tot+4] << 24)
                    if jpeg_len != tot:
                        print(f'\njpeg_len={jpeg_len} total={tot}')

                    ### Using chunks is not required
                    """
                    chunk_size = 1024
                    for i in range(0, tot, chunk_size):
                        print(i)
                        # Write to named pipe as writing to a file (but write the data in small chunks).
                        if chunk_size+i > tot:
                            os.write(fd1, frame[i:tot])  # Write last bytes of data to fd_pipe
                        else:
                            os.write(fd1, frame[i:chunk_size+i])  # Write 1024 bytes of data to fd_pipe
                    """
                    ### print("Write Video ...", end='')
                    os.write(fd1, frame)  # Write video frame to fd_pipe
                    count = count + 1
                    ### print("Done")
                    frame_cnt_vid = frame_cnt_vid + 1

                    last_frame[:] = frame

                    frame.clear()
                    sync = False

                    if (fd2 < 0) and (count == 2):
                        fd2 = os.open(pipe2, os.O_WRONLY)  # fd2 is a file descriptor (an integer)
                
                ### writer_lock.release()

            elif cmd == cmd_udp.P2P_UDP_CMD_G711:
                if (fd2 >= 0): # and (count > 10):
                    # writer_lock.acquire()
                    #print("Write Audio...", end='')
                    os.write(fd2, data)
                    #print("Done")
                    # writer_lock.release()
                    frame_cnt_aud = frame_cnt_aud + 1
                    if frame_cnt_aud > frame_cnt_vid + 2:
                        os.write(fd1, last_frame)  # Write audio frame to fd_pipe
                        frame_cnt_vid = frame_cnt_vid + 1
                        frames_repeat = frames_repeat + 1
            
                    elapsed = datetime.now() - start_time
                    print(f"{elapsed}" " (%4.1f%%)" % (frames_repeat*100/frame_cnt_vid), end = '\r')
        
        cam.cap_live(on_rcv)

    except KeyboardInterrupt:

        # Closing the pipes as closing files.
        os.close(fd1)
        os.close(fd2)

        process.wait()  # Wait for FFmpeg sub-process to finish

        # Remove the "named pipes".
        os.unlink(pipe1)
        os.unlink(pipe2)

        print("\nFrames received: video=%d(lost=%4.1f%%) audio=%d" % (frame_cnt_vid, frames_repeat*100/frame_cnt_vid, frame_cnt_aud))

        return


if __name__ == '__main__':
    with netcl_tcp(AP_HOST, PORT) as sock:
        cam = v720_ap(sock)
        cam.init_live_motion()
        show_live(cam)
