#!/usr/bin/env python3
"""
Wayland screen capture via XDG Desktop Portal + PipeWire
Writes raw RGBA frames to shared memory
"""

import sys
import time
import signal
import random
import string
import os
import mmap

import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib, Gio

SHM_FILE = '/dev/shm/vr_desktop_frame'


class WaylandCaptureTest:
    def __init__(self):
        self.pipeline = None
        self.loop = None
        self.running = True

        self.width = 0
        self.height = 0
        self.frame_count = 0
        self.start_time = None
        self.last_frame_time = 0

        self.session_path = None
        self.pw_node_id = None
        self.pending_requests = {}

        self.shm_fd = None
        self.shm_mmap = None
        self.shm_size = 0

        Gst.init(None)
        self.bus = Gio.bus_get_sync(Gio.BusType.SESSION, None)

        self.bus.signal_subscribe(
            'org.freedesktop.portal.Desktop',
            'org.freedesktop.portal.Request',
            'Response',
            None,
            None,
            Gio.DBusSignalFlags.NONE,
            self._on_portal_response
        )

        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)

    def _signal_handler(self, signum, frame):
        print("\nStopping...")
        self.running = False
        if self.loop:
            self.loop.quit()

    def _on_portal_response(self, connection, sender_name, object_path, interface_name, signal_name, parameters):
        if object_path in self.pending_requests:
            callback = self.pending_requests.pop(object_path)
            response = parameters[0]
            results = parameters[1]
            callback(response, results)

    def _init_shm(self, width, height):
        """Initialize shared memory - header + RGBA data"""
        # Header: width(4) + height(4) + frame_count(8) = 16 bytes
        data_size = width * height * 4
        total_size = 16 + data_size

        if self.shm_mmap:
            self.shm_mmap.close()
        if self.shm_fd:
            os.close(self.shm_fd)

        # Create file
        self.shm_fd = os.open(SHM_FILE, os.O_RDWR | os.O_CREAT | os.O_TRUNC, 0o644)
        os.ftruncate(self.shm_fd, total_size)
        self.shm_mmap = mmap.mmap(self.shm_fd, total_size)
        self.shm_size = total_size
        print(f"Shared memory: {total_size} bytes ({width}x{height})")

    def _on_new_sample(self, sink):
        sample = sink.emit('pull-sample')
        if sample is None:
            return Gst.FlowReturn.OK

        buf = sample.get_buffer()
        caps = sample.get_caps()
        struct = caps.get_structure(0)
        width = struct.get_int('width')[1]
        height = struct.get_int('height')[1]

        # Reinit shm if resolution changed
        if width != self.width or height != self.height:
            self.width = width
            self.height = height
            self._init_shm(width, height)

        self.frame_count += 1
        now = time.time()

        if self.start_time is None:
            self.start_time = now
            print(f"First frame! Resolution: {self.width}x{self.height}")

        # Write to shared memory
        success, map_info = buf.map(Gst.MapFlags.READ)
        if success and self.shm_mmap:
            # Header
            self.shm_mmap.seek(0)
            self.shm_mmap.write(width.to_bytes(4, 'little'))
            self.shm_mmap.write(height.to_bytes(4, 'little'))
            self.shm_mmap.write(self.frame_count.to_bytes(8, 'little'))
            # BGRA pixel data
            self.shm_mmap.write(map_info.data)
            self.shm_mmap.flush()
            buf.unmap(map_info)

        if now - self.last_frame_time >= 1.0:
            elapsed = now - self.start_time
            fps = self.frame_count / elapsed if elapsed > 0 else 0
            print(f"Frames: {self.frame_count}  FPS: {fps:.1f}  Size: {self.width}x{self.height}")
            self.last_frame_time = now

        return Gst.FlowReturn.OK

    def _random_token(self):
        return ''.join(random.choices(string.ascii_lowercase, k=8))

    def _call_portal(self, method, args, callback):
        result = self.bus.call_sync(
            'org.freedesktop.portal.Desktop',
            '/org/freedesktop/portal/desktop',
            'org.freedesktop.portal.ScreenCast',
            method,
            args,
            GLib.VariantType('(o)'),
            Gio.DBusCallFlags.NONE,
            -1,
            None
        )
        request_path = result[0]
        self.pending_requests[request_path] = callback
        return request_path

    def _create_session(self):
        print("Step 1: Creating session...")

        token = self._random_token()
        session_token = self._random_token()

        def on_response(response, results):
            print(f"  Response: {response}")
            if response != 0:
                print("  Failed!")
                self.loop.quit()
                return

            self.session_path = results['session_handle']
            print(f"  Session: {self.session_path}")
            GLib.idle_add(self._select_sources)

        args = GLib.Variant('(a{sv})', ({
            'handle_token': GLib.Variant('s', token),
            'session_handle_token': GLib.Variant('s', session_token),
        },))

        self._call_portal('CreateSession', args, on_response)
        return False

    def _select_sources(self):
        print("Step 2: Select a screen in the dialog...")

        token = self._random_token()

        def on_response(response, results):
            print(f"  Response: {response}")
            if response != 0:
                print("  Cancelled or failed!")
                self.loop.quit()
                return
            GLib.idle_add(self._start_stream)

        args = GLib.Variant('(oa{sv})', (
            self.session_path,
            {
                'handle_token': GLib.Variant('s', token),
                'types': GLib.Variant('u', 1),
                'cursor_mode': GLib.Variant('u', 2),
            }
        ))

        self._call_portal('SelectSources', args, on_response)
        return False

    def _start_stream(self):
        print("Step 3: Starting stream...")

        token = self._random_token()

        def on_response(response, results):
            print(f"  Response: {response}")
            if response != 0:
                print("  Failed!")
                self.loop.quit()
                return

            streams = results.get('streams')
            if not streams:
                print("  No streams!")
                self.loop.quit()
                return

            self.pw_node_id = streams[0][0]
            print(f"  PipeWire node: {self.pw_node_id}")
            GLib.idle_add(self._start_gstreamer)

        args = GLib.Variant('(osa{sv})', (
            self.session_path,
            '',
            {'handle_token': GLib.Variant('s', token)}
        ))

        self._call_portal('Start', args, on_response)
        return False

    def _start_gstreamer(self):
        print(f"Step 4: Starting GStreamer with node {self.pw_node_id}...")

        pipeline_str = f'''
            pipewiresrc path={self.pw_node_id} do-timestamp=true !
            videoconvert !
            video/x-raw,format=BGRA !
            appsink name=sink emit-signals=true max-buffers=1 drop=true
        '''

        try:
            self.pipeline = Gst.parse_launch(pipeline_str)
            sink = self.pipeline.get_by_name('sink')
            sink.connect('new-sample', self._on_new_sample)
            self.pipeline.set_state(Gst.State.PLAYING)
            print("\nCapturing! Press Ctrl+C to stop\n")
        except Exception as e:
            print(f"GStreamer error: {e}")
            import traceback
            traceback.print_exc()
            self.loop.quit()

        return False

    def run(self):
        print("Wayland Screen Capture")
        print("=" * 50)
        print()

        self.loop = GLib.MainLoop()
        GLib.idle_add(self._create_session)

        try:
            self.loop.run()
        except:
            pass
        finally:
            if self.pipeline:
                self.pipeline.set_state(Gst.State.NULL)
            if self.shm_mmap:
                self.shm_mmap.close()
            if self.shm_fd:
                os.close(self.shm_fd)
            if self.frame_count > 0 and self.start_time:
                elapsed = time.time() - self.start_time
                print(f"\nTotal: {self.frame_count} frames in {elapsed:.1f}s ({self.frame_count/elapsed:.1f} FPS)")

        return 0


if __name__ == '__main__':
    test = WaylandCaptureTest()
    sys.exit(test.run())
