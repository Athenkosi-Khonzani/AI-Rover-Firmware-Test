# =========================================================================
# remote.py - drive the mouse from another board (e.g. an ESP32) over UART
#
# Protocol: plain text, one command per line, ended with "\n" (or "\r\n").
#
#   F:100mm      forward 100 mm          (units: mm, cm, m - default mm)
#   B:50mm       backward 50 mm
#   L:90         turn left 90 degrees    (unit optional: 90, 90deg)
#   R:45deg      turn right 45 degrees
#   S            stop NOW (also aborts a move that is in progress)
#   ?            status (battery voltage)
#
# Replies (also one line each, ended with "\n"):
#   ACK F:100mm            command understood, starting
#   DONE F 99.6mm          finished, with what was actually achieved
#   DONE L 89.8deg
#   ERR <reason>           bad command, or the move was aborted / failed
#   OK ... / STOPPED       answers to ? and S
#
# Wiring (3.3 V logic on both boards, crossed TX/RX, shared GND):
#   ESP32 TX  ->  STM32 RX
#   ESP32 RX  <-  STM32 TX
#   ESP32 GND ->  STM32 GND
#
# Use on the mouse:  main.py =   import remote; remote.run()
# (copy motion.py and remote.py onto the UCT_MMOUSE drive too)
# =========================================================================

import uct_mouse
import motion

# Which STM32 UART to use:
#   1 = USART1 PB6/PB7  - exists in the stock firmware (programming header J2
#                         pins 7/8; shared with the ST-Link, so unplug it)
#   4 = UART4  PA0/PA1  - Front Aux J11 pins 14/16   } need a one-line firmware
#   3 = USART3 PD8/PD9  - Rear Aux J12 pins 9/10     } change + rebuild first
UART_ID = 1
BAUD = 115200

USE_ENCODER_TURNS = True        # True: turn_enc (encoders)   False: turn (gyro)
MAX_LINE = 64                   # longest command we accept (bytes)

_UNIT_MM = {"": 1.0, "mm": 1.0, "cm": 10.0, "m": 1000.0}


def open_uart(uart_id=UART_ID, baud=BAUD):
    from machine import UART
    return UART(uart_id, baud, rxbuf=256)       # 8N1


class Link:
    """Line-oriented wrapper around a UART (or anything with any/read/write)."""

    def __init__(self, uart):
        self.uart = uart
        self.buf = b""
        self.queue = []             # complete lines waiting to be executed
        self.stopped = False        # set when a stop command aborted a move

    def send(self, text):
        self.uart.write((text + "\n").encode())

    def _fill(self):
        n = self.uart.any()
        if n:
            data = self.uart.read(n)
            if data:
                self.buf += data
        # split complete lines
        while True:
            cut = -1
            for i in range(len(self.buf)):
                if self.buf[i] == 10 or self.buf[i] == 13:
                    cut = i
                    break
            if cut < 0:
                break
            raw = self.buf[:cut]
            self.buf = self.buf[cut + 1:]
            try:
                line = raw.decode().strip()
            except Exception:
                continue            # garbage bytes (e.g. a boot message)
            if line:
                self.queue.append(line)
        if len(self.buf) > MAX_LINE:    # no terminator in a long time: junk
            self.buf = b""

    def next_line(self):
        """Return the next complete command line, or None."""
        self._fill()
        if self.queue:
            return self.queue.pop(0)
        return None

    def stop_requested(self):
        """Called DURING a move: True if a stop command has arrived.

        A stop cancels the current move AND throws away every command that
        was still waiting in the queue (safer than carrying on).  Commands
        that arrive after the stop are handled normally."""
        self._fill()
        for i in range(len(self.queue)):
            head = self.queue[i].strip().upper()
            if head == "S" or head.startswith("S:") or head == "STOP":
                self.queue = self.queue[i + 1:]     # drop it and anything before
                self.stopped = True
                return True
        return False

    def finish_stop(self):
        """After a move: True (once) if it was ended by a stop command."""
        if self.stopped:
            self.stopped = False
            return True
        return False


# -------------------------------------------------------------------------
# parsing
# -------------------------------------------------------------------------
def parse(line):
    """'F:100mm' -> ('F', 100.0, 'mm').  Raises ValueError if malformed."""
    if ":" in line:
        head, arg = line.split(":", 1)
    else:
        head, arg = line, ""
    cmd = head.strip().upper()
    arg = arg.strip().lower()

    num = ""
    i = 0
    while i < len(arg) and arg[i] in "0123456789.-+":
        num += arg[i]
        i += 1
    unit = arg[i:].strip()
    value = float(num) if num else None      # float() raises ValueError itself
    return cmd, value, unit


# -------------------------------------------------------------------------
# executing
# -------------------------------------------------------------------------
def handle(line, link):
    try:
        cmd, value, unit = parse(line)
    except ValueError:
        link.send("ERR bad number in '%s'" % line)
        return

    if cmd in ("S", "STOP"):
        uct_mouse.set_motors(0, 0)
        link.send("STOPPED")
        return

    if cmd == "?":
        link.send("OK vbatt=%.2fV" % uct_mouse.get_vbatt())
        return

    if cmd in ("F", "B"):
        if value is None or unit not in _UNIT_MM:
            link.send("ERR use F:<number>[mm|cm|m]")
            return
        mm = value * _UNIT_MM[unit]
        if cmd == "B":
            mm = -mm
        link.send("ACK %s" % line)
        got_m, _ = motion.forward(mm / 1000.0,
                                  abort_check=link.stop_requested)
        got_mm = got_m * 1000.0
        if link.finish_stop():
            link.send("STOPPED")
        # abs() so B:50mm reports 50.0 rather than -50.0
        elif abs(got_mm - mm) > max(10.0, 0.1 * abs(mm)):
            link.send("ERR %s stopped early at %.1fmm of %.1fmm"
                      % (cmd, abs(got_mm), abs(mm)))
        else:
            link.send("DONE %s %.1fmm" % (cmd, abs(got_mm)))
        return

    if cmd in ("L", "R"):
        if value is None or unit not in ("", "deg", "d"):
            link.send("ERR use %s:<degrees>" % cmd)
            return
        angle = value if cmd == "L" else -value
        link.send("ACK %s" % line)
        f = motion.turn_enc if USE_ENCODER_TURNS else motion.turn
        got, _ = f(angle, abort_check=link.stop_requested)
        if link.finish_stop():
            link.send("STOPPED")
        elif abs(got - angle) > max(3.0, 0.1 * abs(angle)):
            link.send("ERR %s stopped early at %.1fdeg of %.1fdeg"
                      % (cmd, abs(got), abs(angle)))
        else:
            link.send("DONE %s %.1fdeg" % (cmd, abs(got)))
        return

    link.send("ERR unknown command '%s'" % line)


def run(uart=None):
    """Main loop: wait for lines from the UART and execute them."""
    if not motion.setup():
        return
    if uart is None:
        uart = open_uart()
    link = Link(uart)
    print("remote: listening on UART", UART_ID, "at", BAUD)
    link.send("READY")
    try:
        while True:
            line = link.next_line()
            if line is None:
                uct_mouse.delay_ms(5)       # keep the kernel's sensors updated
                continue
            print("remote: got", line)
            handle(line, link)
    finally:
        uct_mouse.set_motors(0, 0)
