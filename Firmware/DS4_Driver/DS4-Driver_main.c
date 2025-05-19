#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <linux/input.h>
#include <string.h>
#include <errno.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <dirent.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>

int get_dualshock4_fd() {
  const char *dir_path = "/dev/input/by-id";
  DIR *dir = opendir(dir_path);
  if (!dir) {
    perror("Failed to open /dev/input/by-id");
    return -1;
  }

  struct dirent *entry;
  char path[PATH_MAX];

  while ((entry = readdir(dir)) != NULL) {
    // Check for DualShock-like name and event (not js)
    if (strstr(entry->d_name, "Sony") || strstr(entry->d_name, "Wireless_Controller")) {
      if (strstr(entry->d_name, "event")) {
        snprintf(path, sizeof(path), "%s/%s", dir_path, entry->d_name);
        int fd = open(path, O_RDONLY);
        if (fd >= 0) {
          closedir(dir);
          return fd;
        } else {
          fprintf(stderr, "Failed to open %s: %s\n", path, strerror(errno));
        }
      }
    }
  }

  closedir(dir);
  fprintf(stderr, "DualShock 4 device not found in /dev/input/by-id\n");
  return -1;
}

#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <string.h>
#include <errno.h>

// Send X%d to /dev/ttyUSB0
int send_to_ttyusb0(char axis, int value) {
  // Configure serial port
  static struct termios tty;
  static int init = 1;
  static int fd = 0;

  if (init) {
    const char *device = "/dev/ttyUSB0";
    fd = open(device, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) {
      perror("Error opening /dev/ttyUSB0");
      return -1;
    }

    if (tcgetattr(fd, &tty) != 0) {
      perror("Error from tcgetattr");
      close(fd);
      return -1;
    }

    cfsetospeed(&tty, B9600); // Set baud rate (adjust if needed)
    cfsetispeed(&tty, B9600);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8; // 8-bit chars
    tty.c_iflag &= ~IGNBRK;                     // disable break processing
    tty.c_lflag = 0;                            // no signaling chars, no echo, no canonical processing
    tty.c_oflag = 0;                            // no remapping, no delays
    tty.c_cc[VMIN]  = 1;                        // read doesn't block
    tty.c_cc[VTIME] = 5;                        // 0.5 seconds read timeout

    tty.c_iflag &= ~(IXON | IXOFF | IXANY);     // shut off xon/xoff ctrl

    tty.c_cflag |= (CLOCAL | CREAD);            // ignore modem controls, enable reading
    tty.c_cflag &= ~(PARENB | PARODD);          // no parity
    tty.c_cflag &= ~CSTOPB;                     // 1 stop bit
    tty.c_cflag &= ~CRTSCTS;                    // no flow control

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
      perror("Error from tcsetattr");
      close(fd);
      return -1;
    }

    init = 0;
  }

  // Prepare string and write
  char buffer[32];
  snprintf(buffer, sizeof(buffer), "%c%d\n", axis, value);
  printf("%s", buffer);

  int n = write(fd, buffer, strlen(buffer));
  if (n < 0) {
    perror("Error writing to serial port");
    init = 1;
  }

  // close(fd);
  return (n > 0) ? 0 : -1;
}

#include <stdio.h>
#include <time.h>

#define LIMIT(from, val, to) if (val > to) val = to; else if (val < from) val = from;
#define DEADZONE(zone, val, invalid_mask, clear_mask) if (val > 0 && val < (zone) || val < 0 && val > -(zone)) {val = 0; invalid &= ~(clear_mask);}

// Returns milliseconds since epoch
unsigned long long millis() {
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);  // monotonic clock avoids time jumps
  return (ts.tv_sec * 1000ULL) + (ts.tv_nsec / 1000000ULL);
}

void loop_with_100ms_print() {
  unsigned long long last_print = 0;

  while (1) {
    unsigned long long now = millis();
    if (now - last_print >= 100) {
      last_print = now;
      printf("100ms passed: %llu ms\n", now);
    }
    // Optional: sleep a tiny bit to avoid full CPU usage
    struct timespec ts = {.tv_sec = 0, .tv_nsec = 1000000}; // 1ms
    nanosleep(&ts, NULL);
  }
}

int main() {
  const char *device = "/dev/input/eventX"; // replace with actual event file

  // Open device
  int fd = get_dualshock4_fd();
  if (fd < 0) {
    perror("Failed to open input device");
    return 1;
  }

  struct input_event ev;

  int last_val_LY = 128;
  int last_val_RX = 128;

  unsigned long long last_send = millis();

  printf("Listening to %s...\n", device);
  while (1) {
    static int vx =  0; // x movement
    static int vy =  0; // y movement
    static int vbli = 0; // ballast left  intake
    static int vblo = 0; // ballast left  outtake
    static int vbri = 0; // ballast right intake
    static int vbro = 0; // ballast right outtake

    static unsigned invalid = 0xF; // 1111 // bits represent chanels

    ssize_t n = read(fd, &ev, sizeof(ev));
    if (n == (ssize_t)sizeof(ev)) {
      if (ev.type == EV_ABS) {
        switch (ev.code) {
        case ABS_Y:  vx   = ev.value; break;
        case ABS_RX: vy   = ev.value; break;
        case ABS_Z:  vbli = ev.value; break;
        case ABS_RZ: vbri = ev.value; break;
        default:
          // printf("%d: %d\n", ev.code, ev.value);
          break;
        }
      } else if (ev.type == EV_KEY) {
        switch (ev.code) {
        case BTN_TL: vblo = ev.value ? 250 : 0; break;
        case BTN_TR: vbro = ev.value ? 250 : 0; break;
        default:
          printf("Button event: code=0x%X value=%d\n", ev.code, ev.value);
          break;
        }
      }
    } else if (n < 0) {
      perror("Read error");
      break;
    }

    if (millis() - last_send > 100) {
      last_send = millis();

      int ch_R = -vx + 128 + vy - 128;
      int ch_L = -vx + 128 - vy + 128;
      int ch_r = vbri - vbro;
      int ch_l = vbli - vblo;

      if (ch_R > 1000 || ch_R < -1000) printf("Please reconnect the controller\n");

      ch_R *= 2;
      ch_L *= 2;
      ch_r *= 2;
      ch_l *= 2;

      DEADZONE(20, ch_R, invalid, 1 << 0);
      DEADZONE(20, ch_L, invalid, 1 << 1);
      DEADZONE(20, ch_r, invalid, 1 << 2);
      DEADZONE(20, ch_l, invalid, 1 << 3);

      LIMIT(-200, ch_R, 200);
      LIMIT(-200, ch_L, 200);
      LIMIT(-200, ch_r, 200);
      LIMIT(-200, ch_l, 200);

      // printf("R: %d, L: %d, r: %d, l: %d\n", ch_R, ch_L, ch_r, ch_l);
      if (invalid) {
        printf("%d %d %d %d\n", invalid >> 3 & 1, invalid >> 2 & 1, invalid >> 1 & 1, invalid >> 0 & 1);
      }

      if (!invalid) {
        send_to_ttyusb0('R', ch_R);
        send_to_ttyusb0('L', ch_L);
        send_to_ttyusb0('r', ch_r);
        send_to_ttyusb0('l', ch_l);
      }
    }
  }

  close(fd);
  return 0;
}
