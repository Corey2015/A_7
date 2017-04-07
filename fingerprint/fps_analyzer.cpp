#define LOG_TAG "FingerprintSensor"
#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <getopt.h>
#include <fcntl.h>
#include <string.h>
#include <time.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/time.h>
#include <linux/types.h>
#include "fps_driver.h"
#include <sys/stat.h>
#include <math.h>
#include <limits.h>
#include <unistd.h>
#include <sys/time.h>
#include <cutils/properties.h>

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <linux/ioctl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#ifdef TAC_TIME_MEASUREMENTS
#include <sys/time.h>
#endif
#include "fpsensor.h"


#define __DFS747A__
////////////////////////////////////////////////////////////////////////////////
//
// Global variables
//
static volatile int poll_stop    = 0;
static volatile int recal_done   = 0;

#define DETECT_WINDOW_COLS    (8)
#define DETECT_WINDOW_ROWS    (8)
#define SWITCH_MODE_DELAY_US  (5000)

////////////////////////////////////////////////////////////////////////////////
//
// Detect Calibration
//

static int
fps_set_spi_speed(const int      fd,
                  const uint32_t speed)
{
    int status = 0;

    struct dfs747_ioc_transfer tr;
    tr.len    = speed;
    tr.opcode = DFS747_IOC_SET_CLKRATE;

    status = ioctl(fd, DFS747_IOC_MESSAGE(1), &tr);
    if (status < 0) {
        ALOGD("%s(): Calling ioctl() failed! status = %0d\n", __func__, status);
    }

    return status;
}


static double
get_elapsed_ms(const struct timeval *start,
                 const struct timeval *stop)
{
    double elapsed;

    if ((start == NULL) || (stop == NULL)) {
        return (double) -1;
    }

    elapsed = (double) (stop->tv_sec  * SEC + stop->tv_usec) -
              (double) (start->tv_sec * SEC + start->tv_usec);
    return elapsed / MSEC;
}

static int
save_bmp(const char     *path,
         const uint8_t  *img,
         const uint32_t size)
{
    uint8_t file_header[] = {
        0x42, 0x4D,              // 'B' 'M'
        0x36, 0x28, 0x00, 0x00,  // File size in bytes
        0x00, 0x00,              // Reserved
        0x00, 0x00,              // Reserved
        0x36, 0x04, 0x00, 0x00,  // Image offset in bytes
    };

    uint8_t info_header[] = {
        0x28, 0x00, 0x00, 0x00,  // Info header size in bytes
        0x90, 0x00, 0x00, 0x00,  // Image width in pixels
        0x40, 0x00, 0x00, 0x00,  // Image height in pixels
        0x01, 0x00,              // Number of color planes
        0x08, 0x00,              // Bits per pixel
        0x00, 0x00, 0x00, 0x00,  // Image size in bytes
        0x00, 0x00, 0x00, 0x00,  // Compression
        0xE5, 0x4C, 0x00, 0x00,  // X resolution
        0xE5, 0x4C, 0x00, 0x00,  // Y resolution
        0x00, 0x00, 0x00, 0x00,  // Number of colors
        0x00, 0x00, 0x00, 0x00   // Important colors
    };

    uint8_t color_table[256 * 4];

    FILE *fp = NULL;
    int   i;

    for (i = 0; i < 256; i++) {
        color_table[(i * 4) + 0] = i;
        color_table[(i * 4) + 1] = i;
        color_table[(i * 4) + 2] = i;
        color_table[(i * 4) + 3] = 0x00;
    }

    fp = fopen(path, "w");
    if (fp == NULL) {
        return -1;
    }

    fwrite(file_header, 1, sizeof(file_header), fp);
    fwrite(info_header, 1, sizeof(info_header), fp);
    fwrite(color_table, 1, sizeof(color_table), fp);
    fwrite(img, 1, size, fp);
    fclose(fp);

    return 0;
}

uint8_t find_otsu_th(const uint8_t  *img,
             const uint32_t size)
{
    uint8_t  max_th   = 0;
    double   max_otsu = 0.0;
    uint32_t nB       = 0;
    uint32_t sumB     = 0;
    uint32_t nF       = 0;
    uint32_t sumF     = 0;
    double   Wb       = 0.0;
    double   Wf       = 0.0;
    double   ub       = 0.0;
    double   uf       = 0.0;
    double   otsu     = 0.0;
    int      th;
    int      i;
    int      j;
    int      n;

    for (th = 0; th < 256; th++) {
        nB   = 0;
        sumB = 0;
        nF   = 0;
        sumF = 0;

        for (i = 2; i < (DFS747_SENSOR_ROWS - 2); i++) {
        for (j = 2; j < (DFS747_SENSOR_COLS - 2); j++) {
            n = i * DFS747_SENSOR_COLS + j;
            if (img[n] > th) {
                nF++;
                sumF += img[n];
            } else {
                nB++;
                sumB += img[n];
            }
        }}

        if ((nB == 0) || (nF == 0)) {
            continue;
        }

        Wb = (double) nB / (double) (DFS747_SENSOR_ROWS - 4) * (DFS747_SENSOR_COLS - 4);
        Wf = (double) nF / (double) (DFS747_SENSOR_ROWS - 4) * (DFS747_SENSOR_COLS - 4);

        ub = (double) sumB / (double) nB;
        uf = (double) sumF / (double) nF;

        otsu = Wb * Wf * (ub - uf) * (ub - uf);

        if (otsu > max_otsu) {
            max_otsu = otsu;
            max_th   = th;
        }
    }

    return max_th;
}

void find_pixel_range(const uint8_t  *img,
                 const uint32_t size,
                 uint8_t        *pix_min,
                 uint8_t        *pix_max)
{
    uint8_t min;
    uint8_t max;
    int     i;
    int     j;
    int     n;

    min = img[2 * DFS747_SENSOR_COLS + 2];
    max = img[2 * DFS747_SENSOR_COLS + 2];
    for (i = 2; i < DFS747_SENSOR_ROWS - 2; i++) {
    for (j = 2; j < DFS747_SENSOR_COLS - 2; j++) {
        n = i * DFS747_SENSOR_COLS + j;
        if (img[n] > max) max = img[n];
        if (img[n] < min) min = img[n];
    }}

    *pix_min = min;
    *pix_max = max;
}


////////////////////////////////////////////////////////////////////////////////
//
// Sensor Access
//



static int
fps_reset_sensor(const int      fd,
                 const uint32_t state)
{
    int status = 0;

    struct dfs747_ioc_transfer tr;
    tr.len    = state;
    tr.opcode = DFS747_IOC_RESET_SENSOR;

    status = ioctl(fd, DFS747_IOC_MESSAGE(1), &tr);
    if (status < 0) {
        ALOGD("%s(): Calling ioctl() failed! status = %0d\n", __func__, status);
    }

    return status;
}

int fps_multiple_read(const int     fd,
                  const uint8_t *addr,
                  uint8_t       *data,
                  const uint8_t len)
{
    int     status = 0;
    int     i;
    uint8_t *tx;
    uint8_t *rx;
    struct dfs747_ioc_transfer tr;

    if (data == NULL) {
        return -1;
    }

    tx = (uint8_t *) malloc(len);
    if (tx == NULL) {
        ALOGD("%s(): malloc() failed!\n", __func__);
        goto fps_multiple_read_end;
    }

    rx = (uint8_t *) malloc(len);
    if (rx == NULL) {
        ALOGD("%s(): malloc() failed!\n", __func__);
        goto fps_multiple_read_end;
    }

    for (i = 0; i < len; i++) {
        tx[i] = addr[i];
        rx[i] = 0x00;
    }

     tr.tx_buf = (unsigned long) tx;
     tr.rx_buf = (unsigned long) rx;
     tr.len    = len;
     tr.opcode = DFS747_IOC_REGISTER_MASS_READ;

    status = ioctl(fd, DFS747_IOC_MESSAGE(1), &tr);
    if (status < 0) {
        ALOGD("%s(): Calling ioctl() failed! status = %0d\n", __func__, status);
        goto fps_multiple_read_end;
    }

    for (i = 0; i < len; i++) {
        data[i] = rx[i];
    }

fps_multiple_read_end :

    if (tx) free(tx);
    if (rx) free(rx);

    return status;
}

int fps_multiple_write(const int     fd,
                   const uint8_t *addr,
                   const uint8_t *data,
                   const uint8_t len)
{
    int     status = 0;
    int     i;
    uint8_t *tx;

    struct dfs747_ioc_transfer tr;
    if ((addr == NULL) || (data == NULL)) {
        return -1;
    }

    tx = (uint8_t *) malloc(len * 2);
    if (tx == NULL) {
        ALOGD("%s(): malloc() failed!\n", __func__);
        goto fps_multiple_write_end;
    }

    for (i = 0; i < len; i++) {
        tx[(i * 2) + 0] = addr[i];
        tx[(i * 2) + 1] = data[i];
    }

     tr.tx_buf = (unsigned long) tx;
     tr.len    = (len * 2);
     tr.opcode = DFS747_IOC_REGISTER_MASS_WRITE,

    status = ioctl(fd, DFS747_IOC_MESSAGE(1), &tr);
    if (status < 0) {
        ALOGD("%s(): Calling ioctl() failed! status = %0d\n", __func__, status);
        goto fps_multiple_write_end;
    }


fps_multiple_write_end :

    if (tx) free(tx);

    return status;
}

int fps_get_one_image(const int      fd,
                  const uint32_t img_width,
                  const uint32_t img_height,
                  const uint32_t dummy_pix,
                  uint8_t        *img)
{
  int      status = 0;
  uint32_t img_size;
  uint8_t  tx[6];
  int      r;
  int      c;

  if (img == NULL) {
      return -1;
  }

  img_size = img_width * img_height + dummy_pix;

  tx[0] = img_width;
  tx[1] = img_height;
  tx[2] = dummy_pix;
  tx[3] = 0x00;
  tx[4] = 0x00;
  tx[5] = 0x00;

  // Execute get-image I/O control
  struct dfs747_ioc_transfer tr;
  tr.tx_buf = (unsigned long) tx;
  tr.rx_buf = (unsigned long) img;
  tr.len    = img_size;
  tr.opcode = DFS747_IOC_GET_ONE_IMG;


  status = ioctl(fd, DFS747_IOC_MESSAGE(1), &tr);
  if (status < 0) {
      ALOGD("%s(): Calling ioctl() failed! status = %0d\n", __func__, status);
      return status;
  }
  #if 0
    save_bmp("/data/system/users/0/fpdata/dolfa/before.bmp",img,img_size);

  for (c = 0; c < img_width; c++) {
      img[(DFS747_SENSOR_ROWS - 1) * img_width + c] = img[(DFS747_SENSOR_ROWS - 3) * img_width + c];
      img[(DFS747_SENSOR_ROWS - 2) * img_width + c] = img[(DFS747_SENSOR_ROWS - 3) * img_width + c];
  }


  for (c = 0; c < img_width; c++) {
      img[c] = img[img_width + c];
      //img[img_width + c] = img[2 * img_width + c];
  }

  save_bmp("/data/system/users/0/fpdata/dolfa/after.bmp",img,img_size);

  for (r = 0; r < img_height; r++) {
      img[r * img_width + 0] = img[r * img_width + 2];
      img[r * img_width + 1] = img[r * img_width + 2];
  }
#endif
  return status;
}

int fps_single_read(const int     fd,
                const uint8_t addr,
                uint8_t       *data)
{
    return fps_multiple_read(fd, &addr, data, 1);
}

int fps_single_write(const int     fd,
                 const uint8_t addr,
                 const uint8_t data)
{
    return fps_multiple_write(fd, &addr, &data, 1);
}

int fps_switch_mode(const int fd,
                const int mode_new,
                int       *mode_old)
{
  int     status = 0;
  uint8_t addr[3];
  uint8_t data[3];

  addr[0] = DFS747_REG_MISC_PWR_CTL_1;
  addr[1] = DFS747_REG_PWR_CTL_0;
  addr[2] = DFS747_REG_GBL_CTL;

  data[0] = 0x00;
  data[1] = 0x00;
  data[2] = 0x00;

  status = fps_multiple_read(fd, addr, data, sizeof(addr));
  if (status < 0) {
      return status;
  }

  if (mode_old != NULL) {
      if (data[2] & DFS747_ENABLE_DETECT) {
          *mode_old = DFS747_DETECT_MODE;
      } else if (data[1] == (DFS747_PWRDWN_ALL & ~DFS747_PWRDWN_BGR)) {
          *mode_old = DFS747_POWER_DOWN_MODE;
      } else {
          *mode_old = DFS747_IMAGE_MODE;
      }
  }

  switch (mode_new) {
      case DFS747_IMAGE_MODE : {
          data[0] &= ~DFS747_PWRDWN_OVT;
          data[1]  = (DFS747_PWRDWN_DET | DFS747_PWRDWN_OSC);
          data[2] &= ~DFS747_ENABLE_DETECT;
          break;
      }

      case DFS747_DETECT_MODE : {
          data[2] |= DFS747_ENABLE_DETECT;
          break;
      }

      case DFS747_POWER_DOWN_MODE : {
          data[0] |= DFS747_PWRDWN_OVT;
          data[1]  = DFS747_PWRDWN_ALL & ~DFS747_PWRDWN_BGR;
          data[2] &= ~DFS747_ENABLE_DETECT;
          break;
      }

      default : return -1;
  }

  status = fps_multiple_write(fd, addr, data, sizeof(addr));
  if (status < 0) {
      return status;
  }

  usleep(SWITCH_MODE_DELAY_US);

// Patrick 2017-04-01
#if 1
  switch (mode_new) {
      case DFS747_IMAGE_MODE :
          status = fps_single_write(fd, DFS747_REG_TEST_ANA, 0x00);
          break;

      case DFS747_DETECT_MODE :
          status = fps_single_write(fd, DFS747_REG_TEST_ANA, 0x03);
          break;
  }
#endif

  return status;
}

int fps_enable_tgen(const int fd,
                const int enable)
{
  int     status = 0;
  uint8_t addr;
  uint8_t data;

  addr = DFS747_REG_GBL_CTL;
  data = 0x00;

  status = fps_single_read(fd, addr, &data);
  if (status < 0) {
      return status;
  }

  if (enable != 0) {
      data |=  DFS747_ENABLE_TGEN;
  } else {
      data &= ~DFS747_ENABLE_TGEN;
  }

  status = fps_single_write(fd, addr, data);

  return status;
}

int dump_register(int dev_fd)
{
    int     status = 0;
    uint8_t addr[DFS747_REG_COUNT];
    uint8_t data[DFS747_REG_COUNT];
    int     i;

        for (i = 0; i < DFS747_REG_COUNT; i++) {
            addr[i] = i;
            data[i] = 0;
        }

        status = fps_multiple_read(dev_fd, addr, data, DFS747_REG_COUNT);
        if (status < 0) {
            return status;
        }

        for (i = 0; i < DFS747_REG_COUNT; i++) {
            ALOGD("    Addr = 0x%02X, Data = 0x%02X\n", addr[i], data[i]);
        }

    return status;
}

int fps_scan_detect_event(const int      dev_fd,
                      const uint8_t  detect_th,
                      const uint16_t cds_offset,
                      const double   sleep_us,
                      const int      int_enable,
                      const int      cal_detect)
{
  int            status = 0;
  uint8_t        event  = 0;
  uint8_t        addr[4];
  uint8_t        data[4];
  struct pollfd  poll_fps;

  // Disable and clear interrupts
  addr[0] = DFS747_REG_INT_CTL;
  addr[1] = DFS747_REG_INT_EVENT;

  data[0] = 0x00;
  data[1] = 0x00;

  status = fps_multiple_write(dev_fd, addr, data, 2);
  if (status < 0) {
      return status;
  }

  if (cal_detect > 0) {
      // Set up Detect Threshold and CDS offset
      addr[0] = DFS747_REG_DET_CDS_CTL_1;
      addr[1] = DFS747_REG_DET_CDS_CTL_0;
      addr[2] = DFS747_REG_V_DET_SEL;

      status = fps_multiple_read(dev_fd, addr, data, 3);
      if (status < 0) {
          return -1;
      }

      data[0] =  (uint8_t)  (cds_offset & 0x00FF);
      data[1] = ((uint8_t) ((cds_offset & 0x0100) >> 1)) | (data[1] & 0x7F);
      data[2] = detect_th & 0x3F;

      status = fps_multiple_write(dev_fd, addr, data, 3);
      if (status < 0) {
          return -1;
      }
  }

  if (int_enable > 0) {
      // Turn on Detect interrupt
      status = fps_single_write(dev_fd, DFS747_REG_INT_CTL, DFS747_DETECT_EVENT);
      if (status < 0) {
          return -1;
      }

      poll_fps.fd      = dev_fd;
      poll_fps.events  = POLLIN;
      poll_fps.revents = 0;
      status = poll(&poll_fps, 1, (int) (sleep_us / 1000));
      if (status < 0) {
          if ((poll_stop == 0) && (recal_done == 0)) {
              ALOGD("%s(): Calling poll() failed! status = %0d\n", __func__, status);
              return -1;
          }
      } else {
          ALOGD("%s(): poll_fps.revents = %0d\n", __func__, poll_fps.revents);
      }

      if (poll_fps.revents != 0) {
          status = fps_single_read(dev_fd, DFS747_REG_INT_EVENT, &event);
          if (status < 0) {
              return -1;
          }
          ALOGD("%s(): event = %02X\n", __func__, event);
      }

      // Turn off Detect interrupt
      status = fps_single_write(dev_fd, DFS747_REG_INT_CTL, 0x00);
      if (status < 0) {
          return -1;
      }
  } else {
      usleep((int) sleep_us);

      // Read if Detect interrupt happens
      status = fps_single_read(dev_fd, DFS747_REG_INT_EVENT, &event);
      if (status < 0) {
          return -1;
      }
  }

  status = fps_single_write(dev_fd, DFS747_REG_INT_EVENT, 0x00);
  if (status < 0) {
      return status;
  }

  return ((event & DFS747_DETECT_EVENT) != 0);
}

int
fps_search_detect_threshold(const int      fd,
                            const uint16_t cds_offset,
                            const uint8_t  upper,
                            const uint8_t  lower,
                            const double   sleep_us,
                            const uint32_t scan_limit,
                            const int      int_enable,
                            uint8_t        *detect_th)
{
  int      status = 0;
  uint8_t  det_upper;
  uint8_t  det_middle;
  uint8_t  det_lower;
  uint32_t scan_cnt;

  det_upper  = upper;
  det_middle = (upper + lower) / 2;
  det_lower  = lower;

  while ((det_upper - det_lower) > 1) {
      ALOGD("%s(): Detect Threshold range = 0x%02X : 0x%02X : 0x%02X\n", __func__,
             det_upper, det_middle, det_lower);

      status = fps_scan_detect_event(fd, det_middle, cds_offset, sleep_us, int_enable, 1);
      for (scan_cnt = 0; scan_cnt < scan_limit; scan_cnt++) {
          status = fps_scan_detect_event(fd, det_middle, cds_offset, sleep_us, int_enable, 0);

          // Finger-on detected
          if (status > 0) {
              break;
          }

          // Error
          if (status < 0) {
              return status;
          }
      }

      if (scan_cnt == scan_limit) {
          det_upper = det_middle;
      } else {
          det_lower = det_middle;
      }

      det_middle = (det_upper + det_lower) / 2;
  }

  *detect_th = det_upper;

  status = fps_single_write(fd, DFS747_REG_V_DET_SEL, *detect_th);
  if (status < 0) {
      return status;
  }

  if (*detect_th == upper) {
      ALOGD("%s(): Maximum Detect Threshold reached!\n", __func__);
  }

  if (*detect_th == lower) {
      ALOGD("%s(): Minimum Detect Threshold reached!\n", __func__);
  }

  return status;

}

int
fps_search_detect_window(const int      fd,
                         const uint8_t  col_scan_begin,
                         const uint8_t  col_scan_end,
                         const uint32_t repeat_times,
                         uint16_t       *extra_cds_offset,
                         uint8_t        *row_scan_begin,
                         uint8_t        *row_scan_end,
                         double         *win_avg,
                         double         *win_var)
{
  int      status        = 0;
  uint8_t  **raw_buf     = NULL;
  uint8_t  **img_buf     = NULL;
  uint8_t  *avg_img      = NULL;
  uint8_t  avg_frame     = 1;
  uint32_t img_width     = DFS747_SENSOR_COLS;
  uint32_t img_height    = DFS747_SENSOR_ROWS;
  uint32_t img_size      = DFS747_SENSOR_SIZE;
  double   pix           = 0.0;
  double   pix_sum       = 0.0;
  double   sqr_sum       = 0.0;
  double   avg           = 0.0;
  double   var           = 0.0;
  uint32_t col_scan_cnt  = 0;
  uint32_t row_scan_cnt  = 0;
  uint32_t win_scan_cnt  = 0;
  double   *all_wins_avg = NULL;
  double   *all_wins_var = NULL;
  uint8_t  pga_gain      = 0;
  int      t;
  int      f;
  int      w;
  int      rb;
  int      r;
  int      c;
  int      p;
  char     file[CHAR_CNT];

  col_scan_cnt = col_scan_end - col_scan_begin + 1;
  row_scan_cnt = col_scan_cnt;

// Patrick 2017-04-01
#if 0
  win_scan_cnt = img_height - 2 - col_scan_cnt + 1;
#else
  win_scan_cnt = img_height - 16 - col_scan_cnt + 1;
#endif

  // NOTE: Assume the sensor is in Image Mode before this call...

  // Switch CDS out as ADC singal source
  status = fps_single_write(fd, DFS747_REG_TEST_ANA, 0x03);

  if (status < 0) {
      return -1;
  }

  // Create image buffers to average later
  img_buf = (uint8_t **) malloc(sizeof(uint8_t *) * avg_frame);
  if (img_buf == NULL) {
      status = -1;
      goto fps_search_detect_window_error;
  }

  raw_buf = (uint8_t **) malloc(sizeof(uint8_t *) * avg_frame);
  if (raw_buf == NULL) {
      status = -1;
      goto fps_search_detect_window_error;
  }

  for (f = 0; f < avg_frame; f++) {
      raw_buf[f] = (uint8_t *) malloc(img_size + DFS747_DUMMY_PIXELS);
      if (raw_buf[f] == NULL) {
          status = -1;
          goto fps_search_detect_window_error;
      }
      img_buf[f] = &raw_buf[f][DFS747_DUMMY_PIXELS];
  }

  avg_img = (uint8_t *) malloc(img_size);
  if (avg_img == NULL) {
      status = -1;
      goto fps_search_detect_window_error;
  }

  all_wins_avg = (double *) malloc(sizeof(double) * win_scan_cnt);
  if (all_wins_avg == NULL) {
      status = -1;
      goto fps_search_detect_window_error;
  }

  all_wins_var = (double *) malloc(sizeof(double) * win_scan_cnt);
  if (all_wins_var == NULL) {
      status = -1;
      goto fps_search_detect_window_error;
  }

  for (w = 0; w < win_scan_cnt; w++) {
      all_wins_avg[w] = 0.0;
      all_wins_var[w] = 0.0;
  }

  for (t = 0; t < repeat_times; t++) {
      for (f = 0; f < avg_frame; f++) {
          // Clear all pending events
          status = fps_single_write(fd, DFS747_REG_INT_EVENT, 0x00);
          if (status < 0) {
              goto fps_search_detect_window_error;
          }

          // Turn on TGEN
          status = fps_enable_tgen(fd, 1);
          if (status < 0) {
              goto fps_search_detect_window_error;
          }

          // Get a frame
          status = fps_get_one_image(fd,
                                     img_width,
                                     img_height,
                                     DFS747_DUMMY_PIXELS,
                                     raw_buf[f]);
          if (status < 0) {
              goto fps_search_detect_window_error;
          }

          // Turn off TGEN
          status = fps_enable_tgen(fd, 0);
          if (status < 0) {
              goto fps_search_detect_window_error;
          }
      }

      // Average each frame
      for (p = 0; p < img_size; p++) {
          pix_sum = 0.0;
          for (f = 0; f < avg_frame; f++) {
              pix_sum += (double) img_buf[f][p];
          }

          avg_img[p] = (uint8_t) (pix_sum / avg_frame);
      }

      //sprintf(file, "detect_search.bmp");
      //status = save_bmp(file, avg_img, img_size);
      //if (status < 0) {
      //    goto fps_search_detect_window_error;
      //}

// Patrick 2017-04-01
#if 0
      for (rb = 1; rb < (1 + win_scan_cnt); rb++) {
#else
      for (rb = 9; rb < (9 + win_scan_cnt); rb++) {
#endif
          pix_sum = 0.0;
          sqr_sum = 0.0;
          for (r = rb; r < (rb + row_scan_cnt); r++) {
              for (c = col_scan_begin; c <= col_scan_end; c++) {
                  pix = (double) avg_img[r * DFS747_SENSOR_COLS + c];
                  pix_sum +=  pix;
                  sqr_sum += (pix * pix);
              }
          }

          avg =  pix_sum / (row_scan_cnt * col_scan_cnt);
          var = (sqr_sum / (row_scan_cnt * col_scan_cnt)) - (avg * avg);

// Patrick 2017-04-05
#if 0
          ALOGD("%s(): Row (Begin/End) = %0d/%0d, Avg = %0.3f, Var. = %0.3f\n",
                __func__, rb, (rb + row_scan_cnt - 1), avg, var);
#endif

// Patrick 2017-04-01
#if 0
          if (dev > all_wins_var[rb - 1]) {
              all_wins_var[rb - 1] = dev;
              all_wins_avg[rb - 1] = avg;
          }
#else
          if (var > all_wins_var[rb - 9]) {
              all_wins_var[rb - 9] = var;
              all_wins_avg[rb - 9] = avg;
          }
#endif
      }
  }


// Patrick 2017-04-01
#if 0
  for (rb = 1; rb < (1 + win_scan_cnt); rb++) {
      ALOGD("%s(): Row (Begin/End) %0d/%0d, Avg. = %0.3f, Var. = %0.3f\n",
            __func__, rb, (rb + row_scan_cnt - 1),
            all_wins_avg[rb - 1], all_wins_var[rb - 1]);
  }

  *row_scan_begin = 1;
  *row_scan_end   = 1 + row_scan_cnt - 1;
  *win_var        = all_wins_var[0];
#else
  for (rb = 9; rb < (9 + win_scan_cnt); rb++) {
      ALOGD("%s(): Row (Begin/End) %0d/%0d, Avg. = %0.3f, Var. = %0.3f\n",
            __func__, rb, (rb + row_scan_cnt - 1),
            all_wins_avg[rb - 9], all_wins_var[rb - 9]);
  }

  *row_scan_begin = 9;
  *row_scan_end   = 9 + row_scan_cnt - 1;
  // Patrick 2017-04-05
  #if 1
  *win_avg        = all_wins_avg[0];
  #endif
  *win_var        = all_wins_var[0];
#endif

  for (w = 1; w < win_scan_cnt; w++) {
// Patrick 2017-04-01
#if 0
      if ((all_wins_var[w] < *win_var) && (*win_var != 0)) {
#else
      if ((all_wins_var[w] > *win_var) && (*win_var != 0)) {
#endif
          *row_scan_begin = w + 1;
          *row_scan_end   = w + 1 + row_scan_cnt - 1;
// Patrick 2017-04-05
#if 1
          *win_avg        = all_wins_avg[w];
#endif
          *win_var        = all_wins_var[w];
      }
  }

  status = fps_single_read(fd, DFS747_REG_IMG_PGA1_CTL, &pga_gain);
  if (status < 0) {
      status = -1;
      goto fps_search_detect_window_error;
  }

  *extra_cds_offset = (int) (sqrt(*win_var)                               *  // ADC code deviation
                             1                                            /  // 4 sigmas
                             256 * 3.3 * 1000                             /  // ADC code -> mV
                             2 + 0.5);                                       // 2mV CDS offset per step
  ALOGD("%s(): extra_cds_offset = 0x%03X\n", __func__, *extra_cds_offset);

  if (*win_var >= 10.0) {
      ALOGD("%s(): Too bad to search a good detect window!\n", __func__);
      status = -1;
      goto fps_search_detect_window_error;
  }

  ALOGD("%s(): Sel. Row (Begin/End) = %0d/%0d, Avg. = %0.3f, Var. = %0.3f, Extra CDS Offset = 0x%03X\n",
        __func__, *row_scan_begin, *row_scan_end, *win_avg, *win_var, *extra_cds_offset);

fps_search_detect_window_error:

  // Switch PGA out as ADC signal source
  (void) fps_single_write(fd, DFS747_REG_TEST_ANA, 0x00);

  // Free allocated buffers
  if (avg_img) free(avg_img);
  for (f = 0; f < avg_frame; f++) {
      if (raw_buf[f]) free(raw_buf[f]);
  }
  if (raw_buf)      free(raw_buf);
  if (img_buf)      free(img_buf);
  if (all_wins_var) free(all_wins_var);
  if (all_wins_avg) free(all_wins_avg);

  return status;
}

int
fps_search_cds_offset(const int      fd,
                      const uint8_t  detect_th,
                      const uint16_t upper,
                      const uint16_t lower,
                      const double   sleep_us,
                      const uint32_t scan_limit,
                      const int      int_enable,
                      uint16_t       *cds_offset)
{
  int      status = 0;
  uint16_t cds_upper;
  uint16_t cds_middle;
  uint16_t cds_lower;
  uint32_t scan_cnt;
  uint8_t  addr[2];
  uint8_t  data[2];

  cds_upper  = upper;
  cds_middle = (upper + lower) / 2;
  cds_lower  = lower;

  while ((cds_upper - cds_lower) > 1) {
      ALOGD("%s(): CDS Offset range = 0x%03X : 0x%03X : 0x%03X\n", __func__,
             cds_upper, cds_middle, cds_lower);

      status = fps_scan_detect_event(fd, detect_th, cds_middle, sleep_us, int_enable, 1);
      for (scan_cnt = 0; scan_cnt < scan_limit; scan_cnt++) {
          status = fps_scan_detect_event(fd, detect_th, cds_middle, sleep_us, int_enable, 0);

          // Finger-on detected
          if (status > 0) {
              break;
          }

          // Error
          if (status < 0) {
              return status;
          }
      }

      if (scan_cnt == scan_limit) {
          cds_upper = cds_middle;
      } else {
          cds_lower = cds_middle;
      }

      cds_middle = (cds_upper + cds_lower) / 2;
  }

  *cds_offset = cds_upper;

  addr[0] = DFS747_REG_DET_CDS_CTL_0;
  addr[1] = DFS747_REG_DET_CDS_CTL_1;

  status = fps_multiple_read(fd, addr, data, 2);
  if (status < 0) {
      return status;
  }

  data[0] = (data[0] & 0x7F) | ((uint8_t) ((*cds_offset & 0x0100) >> 1));
  data[1] = (uint8_t) (*cds_offset & 0x00FF);

  status = fps_multiple_write(fd, addr, data, 2);
  if (status < 0) {
      return status;
  }

  if (*cds_offset == upper) {
      ALOGD("%s(): Maximum CDS Offset reached!\n", __func__);
  }

  if (*cds_offset == lower) {
      ALOGD("%s(): Minimum CDS Offset reached!\n", __func__);
  }

  return status;
}

int init_sensor(int dev_fd)
{
    int     status = 0;
    char    key    = 0;
    uint8_t addr[5];
    uint8_t data[5];
    int     i;

    //memset(bkg_img, 0x00, sizeof(bkg_img));

    ALOGD("    Initializing sensor...\n");

    status = fps_set_spi_speed(dev_fd, 8 * 1000 * 1000);
    if (status < 0) {
        return status;
    }

    // Reset sensor
    status = fps_reset_sensor(dev_fd, 0);
    usleep(20 * MSEC);
    status = fps_reset_sensor(dev_fd, 1);

// Patrick 2017-04-01
#if 1
    // Add a dummy write
    status = fps_single_write(dev_fd, DFS747_REG_INT_CTL, 0x00);
#endif

    // Disable and clear interrupts
    addr[0] = DFS747_REG_INT_CTL;
    addr[1] = DFS747_REG_INT_EVENT;

    data[0] = 0x00;
    data[1] = 0x00;

    status = fps_multiple_write(dev_fd, addr, data, 2);
    if (status < 0) {
        return status;
    }

    // Enter power down mode
    status = fps_switch_mode(dev_fd, DFS747_POWER_DOWN_MODE, NULL);
    if (status < 0) {
        return status;
    }

    status = fps_single_write(dev_fd, DFS747_REG_CPR_CTL, 0x00);
    if (status < 0) {
        return status;
    }
    usleep(10 * MSEC);

    status = fps_single_write(dev_fd, DFS747_REG_CPR_CTL, 0x08);
    if (status < 0) {
        return status;
    }
    usleep(10 * MSEC);

    status = fps_single_write(dev_fd, DFS747_REG_CPR_CTL, 0x04);

    if (status < 0) {
        return status;
    }
    usleep(10 * MSEC);

    status = fps_single_write(dev_fd, DFS747_REG_CPR_CTL, 0x0F);

    if (status < 0) {
        return status;
    }
    usleep(50 * MSEC);

    // Enter image mode
    status = fps_switch_mode(dev_fd, DFS747_IMAGE_MODE, NULL);
    if (status < 0) {
        return status;
    }

    // Settings for sensing enhancement
    addr[0] = DFS747_REG_GBL_CTL;
    addr[1] = DFS747_REG_MISC_PWR_CTL_1;
    addr[2] = DFS747_REG_ANA_I_SET_0;
    addr[3] = DFS747_REG_ANA_I_SET_1;
    addr[4] = DFS747_REG_SET_ANA_0;

    data[0] = 0x00;
    data[1] = 0x20;
    data[2] = 0xFC;
    data[3] = 0x03;
    data[4] = 0xFF;

    status = fps_multiple_write(dev_fd, addr, data, 5);
    if (status < 0) {
        return status;
    }
    ALOGD("init Done");
    return status;
}

int
reset_sensor(int dev_fd)
{
    int      status = 0;
    char     key    = 0;
    uint32_t delay  = 10 * MSEC;
    ALOGD("    Resetting sensor...\n");

    status = fps_reset_sensor(dev_fd, 0);
    if (status < 0) {
        return status;
    }

    usleep(delay);

    status = fps_reset_sensor(dev_fd, 1);
    if (status < 0) {
        return status;
    }

    ALOGD("    Done!\n");

    return status;
}
