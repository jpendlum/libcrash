/******************************************************************************
**  This is free software: you can redistribute it and/or modify
**  it under the terms of the GNU General Public License as published by
**  the Free Software Foundation, either version 3 of the License, or
**  (at your option) any later version.
**
**  This is distributed in the hope that it will be useful,
**  but WITHOUT ANY WARRANTY; without even the implied warranty of
**  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
**  GNU General Public License for more details.
**
**  You should have received a copy of the GNU General Public License
**  along with this code.  If not, see <http://www.gnu.org/licenses/>.
**
**
**
**  File:         libcrash.c
**  Author(s):    Jonathon Pendlum (jon.pendlum@gmail.com)
**  Description:  Library functions to use with the CRASH kernel module
**
******************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <time.h>
#include <unistd.h>
#include <libudev.h>
#include <sys/stat.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <string.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <libcrash.h>
#include <crash-kmod.h>

struct crash_plblock* crash_open(int plblock_id, xfer_type read_write_flag) {
  const char* dev = "/dev/crash";
  struct crash_plblock *plblock;

  if (read_fpga_status() != 1) {
    fprintf(stderr, "ERROR: FPGA not programmed\n");
    return 0;
  }

  plblock = malloc(sizeof(struct crash_plblock));
  if (plblock == 0) {
    fprintf(stderr, "ERROR: Failed to allocate memory for plblock - %s\n", strerror(errno));
    return 0;
  }

  plblock->id = plblock_id;
  plblock->read_write_flag = read_write_flag;
  plblock->dma_active = false;
  plblock->num_ring_buff = 0;
  plblock->ring_buff_len = 0;
  plblock->write_index = 0;
  plblock->read_index = 0;

  // Open crash device driver. This will allocate a DMA buffer among other things.
  plblock->fd = open(dev, O_RDWR|O_SYNC);
  if (plblock->fd < 0) {
    fprintf(stderr, "ERROR: Failed to open %s - %s\n", dev, strerror(errno));
    free(plblock);
    return 0;
  }

  if(ioctl(plblock->fd, CRASH_GET_DMA_PHYS_ADDR, &plblock->dma_phys_addr) < 0) {
    fprintf(stderr, "ERROR: Failed to get DMA buffer physical address\n");
    close(plblock->fd);
    free(plblock);
    return 0;
  }

  // mmap the DMA data buffer into user space
  plblock->dma_buff_len = DMA_BUFF_SIZE;
  plblock->dma_buff = (uint32_t*)mmap(NULL, plblock->dma_buff_len, PROT_READ|PROT_WRITE, MAP_SHARED, plblock->fd, MMAP_DMA_BUFF);
  if (plblock->dma_buff == MAP_FAILED) {
    fprintf(stderr, "ERROR: failed mmapping DMA buffer - %s\n", strerror(errno));
    close(plblock->fd);
    free(plblock);
    return 0;
  }

  // mmap the control registers into user space
  plblock->regs_len = REGS_TOTAL_ADDR_SPACE;
  plblock->regs = (uint32_t*)mmap(NULL, REGS_TOTAL_ADDR_SPACE, PROT_READ|PROT_WRITE, MAP_SHARED, plblock->fd, MMAP_REGS);
  if (plblock->regs == MAP_FAILED) {
    fprintf(stderr, "ERROR: failed mmapping control registers - %s\n", strerror(errno));
    munmap((void *)plblock->dma_buff,plblock->dma_buff_len);
    close(plblock->fd);
    free(plblock);
    return 0;
  }

  // Clear the DMA buffer
  memset((void *)plblock->dma_buff, 0, (uint32_t)(plblock->dma_buff_len));
  return plblock;
}

int crash_close(struct crash_plblock *plblock) {
  if (plblock == 0) {
    fprintf(stderr, "ERROR: Attempted to close non-existing plblock\n");
    return -1;
  }
  if (plblock->fd < 0) {
    fprintf(stderr, "ERROR: Attempted to close non-existing device\n");
    return -1;
  }
  munmap((void *)plblock->regs,plblock->regs_len);
  munmap((void *)plblock->dma_buff,plblock->dma_buff_len);
  close(plblock->fd);
  free(plblock);
  return 0;
}

int crash_single_dma(struct crash_plblock *plblock, uint plblock_id, uint num_words) {
  uint cmd;

  cmd = (1 << 31) + ((plblock_id & 0x7) << 23) + (num_words*WORD_SIZE & 0x7FFFFF);
  if (plblock->read_write_flag == READ) {
    if(ioctl(plblock->fd, CRASH_DMA_READ, cmd) < 0) {
      perror("ERROR: Failed DMA read");
      return -1;
    }
    if (crash_get_bit(plblock->regs,USRP_RX_FIFO_OVERFLOW)) {
      printf("O");
      // Clear overflow sticky bit
      crash_set_bit(plblock->regs,USRP_RX_FIFO_OVERFLOW_CLR);
      crash_clear_bit(plblock->regs,USRP_RX_FIFO_OVERFLOW_CLR);
    }
  } else if (plblock->read_write_flag == WRITE) {
    if(ioctl(plblock->fd, CRASH_DMA_WRITE, cmd) < 0) {
      perror("ERROR: Failed DMA write");
      return -1;
    }
    //if (crash_get_bit(plblock->regs,USRP_TX_FIFO_UNDERFLOW)) {
    //  printf("U");
    //  // Clear underflow sticky bit
    //  crash_set_bit(plblock->regs,USRP_TX_FIFO_UNDERFLOW_CLR);
    //  crash_clear_bit(plblock->regs,USRP_TX_FIFO_UNDERFLOW_CLR);
    //}
  }
  return 0;
}

// Continuous DMA into DMA buffer.
int crash_start_dma(struct crash_plblock *plblock, uint plblock_id, uint num_ring_buff, uint words_per_buff) {
  int i = 0;
  uint addr = 0;
  uint cmd = 0;

  if (plblock->dma_buff_len < num_ring_buff*words_per_buff*WORD_SIZE) {
    fprintf(stderr, "ERROR: Ring buffers bigger than DMA buffer size\n");
    return -1;
  }

  // CMD FIFOs are 64 entries deep
  if (num_ring_buff > 64) {
    fprintf(stderr, "ERROR: Too many ring buffers, must be less than or equal to 64\n");
    return -1;
  }

  // Setup ring buffers
  plblock->num_ring_buff = num_ring_buff;
  plblock->ring_buff_len = words_per_buff*WORD_SIZE;
  plblock->write_index = 0;
  plblock->read_index = 0;

  // Disable Xfer, Reset CMD fifo, reset Xfer count, and set autoload
  if (plblock->read_write_flag == READ) {
    crash_clear_bit(plblock->regs, DMA_S2MM_XFER_EN);
    crash_set_bit(plblock->regs, DMA_RESET_S2MM_CMD_FIFO);
    crash_clear_bit(plblock->regs, DMA_RESET_S2MM_CMD_FIFO);
    crash_set_bit(plblock->regs, DMA_CLEAR_S2MM_XFER_CNT);
    crash_clear_bit(plblock->regs, DMA_CLEAR_S2MM_XFER_CNT);
    crash_write_reg(plblock->regs, DMA_S2MM_CMD_FIFO_LOOP, (1 << plblock_id));
  } else {
    crash_clear_bit(plblock->regs, DMA_MM2S_XFER_EN);
    crash_set_bit(plblock->regs, DMA_RESET_MM2S_CMD_FIFO);
    crash_clear_bit(plblock->regs, DMA_RESET_MM2S_CMD_FIFO);
    crash_set_bit(plblock->regs, DMA_CLEAR_MM2S_XFER_CNT);
    crash_clear_bit(plblock->regs, DMA_CLEAR_MM2S_XFER_CNT);
    crash_write_reg(plblock->regs, DMA_MM2S_CMD_FIFO_LOOP, (1 << plblock_id));
  }

  // Load CMD fifo
  addr = plblock->dma_phys_addr;
  for (i = 0; i < num_ring_buff; i++) {
    cmd = (1 << 31) + ((plblock_id & 0x7) << 23) + (plblock->ring_buff_len & 0x7FFFFF);
    if (plblock->read_write_flag == READ) {
      crash_write_reg(plblock->regs, DMA_S2MM_CMD_ADDR, addr);
      crash_write_reg(plblock->regs, DMA_S2MM_CMD_DATA, cmd);
    } else {
      crash_write_reg(plblock->regs, DMA_MM2S_CMD_ADDR, addr);
      crash_write_reg(plblock->regs, DMA_MM2S_CMD_DATA, cmd);
    }
    addr += plblock->ring_buff_len;
  }

  // Start xfer
  if (plblock->read_write_flag == READ) {
    crash_set_bit(plblock->regs, DMA_S2MM_XFER_EN);
  } else {
    crash_set_bit(plblock->regs, DMA_MM2S_XFER_EN);
  }

  plblock->dma_active = true;
  return 0;
}

int crash_stop_dma(struct crash_plblock *plblock) {
  if (plblock->dma_active == false) {
    fprintf(stderr, "ERROR: DMA not active\n");
    return (-1);
  }

  // Disable Xfer, Reset CMD fifo, reset Xfer count, and set autoload
  if (plblock->read_write_flag == READ) {
    crash_clear_bit(plblock->regs, DMA_S2MM_XFER_EN);
    crash_set_bit(plblock->regs, DMA_RESET_S2MM_CMD_FIFO);
    crash_clear_bit(plblock->regs, DMA_RESET_S2MM_CMD_FIFO);
    crash_set_bit(plblock->regs, DMA_CLEAR_S2MM_XFER_CNT);
    crash_clear_bit(plblock->regs, DMA_CLEAR_S2MM_XFER_CNT);
    crash_write_reg(plblock->regs, DMA_S2MM_CMD_FIFO_LOOP, 0);
  } else {
    crash_clear_bit(plblock->regs, DMA_MM2S_XFER_EN);
    crash_set_bit(plblock->regs, DMA_RESET_MM2S_CMD_FIFO);
    crash_clear_bit(plblock->regs, DMA_RESET_MM2S_CMD_FIFO);
    crash_set_bit(plblock->regs, DMA_CLEAR_MM2S_XFER_CNT);
    crash_clear_bit(plblock->regs, DMA_CLEAR_MM2S_XFER_CNT);
    crash_write_reg(plblock->regs, DMA_MM2S_CMD_FIFO_LOOP, 0);
  }
  return 0;
}

// Returns a pointer to an offset in the DMA buffer and the number of words that can either be:
// 1) Read (if this is a S2MM DMA transfer) or
// 2) Written (if this is a MM2S DMA transfer)
struct dma_buff crash_get_dma_buffer(struct crash_plblock *plblock, uint requested_num_words) {
  struct dma_buff req_buff;
  uint count = 0;

  if (plblock->dma_active == false) {
    fprintf(stderr, "ERROR: DMA not active\n");
    req_buff.buff = 0;
    req_buff.num_words = 0;
    return (req_buff);
  }

  if (plblock->read_write_flag == READ) {
    count = crash_read_reg(plblock->regs, DMA_S2MM_XFER_CNT);
  } else {
    count = crash_read_reg(plblock->regs, DMA_MM2S_XFER_CNT);
  }

  // Ring buffer was overran
  if (count > plblock->num_ring_buff) {
    fprintf(stderr, "ERROR: Ring buffer overrun\n");
  }

  if (plblock->read_write_flag == READ) {
    plblock->write_index = (plblock->write_index + count*plblock->ring_buff_len) % plblock->num_ring_buff*plblock->ring_buff_len;
    // Write pointer equals read pointer, we cannot do anything
    if (plblock->read_index == plblock->write_index) {
      req_buff.buff = 0;
      req_buff.num_words = 0;
    // Write pointer behind read pointer, we can provide a buffer up to the end of the overall buffer
    } else if (plblock->read_index > plblock->write_index) {
      // Are we requesting more words than the length of the DMA buffer?
      if ((plblock->read_index + requested_num_words*WORD_SIZE) > plblock->num_ring_buff*plblock->ring_buff_len) {
        // We will have to give a smaller buffer
        req_buff.buff = &plblock->dma_buff[plblock->read_index];
        req_buff.num_words = (plblock->num_ring_buff*plblock->ring_buff_len - plblock->read_index)/WORD_SIZE;
        plblock->read_index = 0;
      } else {
        // We can get the full buffer size
        req_buff.buff = &plblock->dma_buff[plblock->read_index];
        req_buff.num_words = requested_num_words;
        plblock->read_index += requested_num_words*WORD_SIZE;
      }
    // Write pointer in front of read pointer, we can provide a buffer up to the write pointer
    } else {
      // Are we requesting more words than have been written?
      if ((plblock->read_index + requested_num_words*WORD_SIZE) > plblock->write_index) {
        req_buff.buff = &plblock->dma_buff[plblock->read_index];
        req_buff.num_words = (plblock->write_index - plblock->read_index)/WORD_SIZE;
        plblock->read_index = plblock->write_index;
      } else {
        // We can get the full buffer size
        req_buff.buff = &plblock->dma_buff[plblock->read_index];
        req_buff.num_words = requested_num_words;
        plblock->read_index += requested_num_words*WORD_SIZE;
      }
    }
  } else { // read_write_flag == WRITE, the role of read & write pointer are reversed
    plblock->read_index = (plblock->read_index + count*plblock->ring_buff_len) % plblock->num_ring_buff*plblock->ring_buff_len;
    // Write pointer equals read pointer, we cannot do anything
    if (plblock->read_index == plblock->write_index) {
      req_buff.buff = 0;
      req_buff.num_words = 0;
    // Read pointer behind write pointer, we can provide a buffer up to the end of the overall buffer
    } else if (plblock->write_index > plblock->read_index) {
      // Are we requesting more words than the length of the DMA buffer?
      if ((plblock->write_index + requested_num_words*WORD_SIZE) > plblock->num_ring_buff*plblock->ring_buff_len) {
        // We will have to give a smaller buffer
        req_buff.buff = &plblock->dma_buff[plblock->write_index];
        req_buff.num_words = (plblock->num_ring_buff*plblock->ring_buff_len - plblock->write_index)/WORD_SIZE;
        plblock->read_index = 0;
      } else {
        // We can get the full buffer size
        req_buff.buff = &plblock->dma_buff[plblock->write_index];
        req_buff.num_words = requested_num_words;
        plblock->write_index += requested_num_words*WORD_SIZE;
      }
    // Read pointer in front of write pointer, we can provide a buffer up to the read pointer
    } else {
      // Are we requesting more words than have been read?
      if ((plblock->write_index + requested_num_words*WORD_SIZE) > plblock->read_index) {
        req_buff.buff = &plblock->dma_buff[plblock->write_index];
        req_buff.num_words = (plblock->read_index - plblock->write_index)/WORD_SIZE;
        plblock->write_index = plblock->read_index;
      } else {
        // We can get the full buffer size
        req_buff.buff = &plblock->dma_buff[plblock->write_index];
        req_buff.num_words = requested_num_words;
        plblock->write_index += requested_num_words*WORD_SIZE;
      }
    }
  }

  return(req_buff);
}

int read_fpga_status() {
  struct udev *udev;
  struct udev_enumerate *enumerate;
  struct udev_list_entry *device;
  struct udev_device *dev;
  const char *path;
  long prog_done = 0;

  udev = udev_new();
  if (!udev) {
    fprintf(stderr, "ERROR: udev_new() failed\n");
    return -1;
  }

  // Enumerate devcfg
  enumerate = udev_enumerate_new(udev);
  udev_enumerate_add_match_sysname(enumerate, XDEV_CFG_NAME);
  udev_enumerate_scan_devices(enumerate);
  device = udev_enumerate_get_list_entry(enumerate);

  // Did not find a device, lets try a different name
  if (!device)
  {
    fprintf(stderr, "ERROR: Could not find FPGA configuration device!\n");
    return -1;
  }

  // List should have only one entry
  if (udev_list_entry_get_next(device) != 0)
  {
    fprintf(stderr, "ERROR: Found more than one FPGA configuration device!\n");
    return -1;
  }

  // Create udev device
  path = udev_list_entry_get_name(device);
  dev = udev_device_new_from_syspath(udev, path);

  prog_done = (int)atol(udev_device_get_sysattr_value(dev, "prog_done"));

  udev_enumerate_unref(enumerate);
  udev_unref(udev);

  return(prog_done);
}