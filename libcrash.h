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
**  File:         libcrash.h
**  Author(s):    Jonathon Pendlum (jon.pendlum@gmail.com)
**  Description:  Library functions to use with the CRASH kernel module
**
******************************************************************************/
#ifndef LIBCRASH_H
#define LIBCRASH_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Many other defines are included from crash-kmod.h
#define DMA_BUFF_SIZE           (1 << PAGE_ORDER) * getpagesize()
#define XDEV_CFG_NAME           "f8007000.ps7-dev-cfg"
#define WORD_SIZE               8

typedef enum {READ, WRITE} xfer_type;

// Fields in this struct should not be changed
struct crash_plblock {
  int                id;
  int                fd;
  xfer_type          read_write_flag;
  uint32_t volatile  *regs;
  uint               regs_len;
  uint32_t volatile  *dma_buff;
  uint32_t           dma_phys_addr;
  uint               dma_buff_len;
  // Ring buffer related
  bool               dma_active;
  uint               num_ring_buff;
  uint               ring_buff_len;
  uint               write_index;
  uint               read_index;
};

struct dma_buff {
  uint32_t volatile *buff;
  uint               num_words;
};

struct crash_plblock* crash_open(int plblock_id, xfer_type read_write_flag);
int crash_close(struct crash_plblock *plblock);
#define crash_read(plblock, plblock_id, num_words) crash_single_dma(plblock, plblock_id, num_words)
#define crash_write(plblock, plblock_id, num_words) crash_single_dma(plblock, plblock_id, num_words)
int crash_single_dma(struct crash_plblock *plblock, uint plblock_id, uint num_words);
int crash_start_dma(struct crash_plblock *plblock, uint plblock_id, uint num_ring_buff, uint words_per_buff);
int crash_stop_dma(struct crash_plblock *plblock);
struct dma_buff crash_get_dma_buffer(struct crash_plblock *plblock, uint requested_num_words);
int read_fpga_status();
#define crash_reset(plblock)              ioctl(plblock->fd, CRASH_RESET, 0)
#define crash_enable_s2mm_intr(plblock)   ioctl(plblock->fd, CRASH_SET_INTERRUPTS,  DMA_S2MM_INTERRUPT | ioctl(plblock->fd, CRASH_GET_INTERRUPTS, 0))
#define crash_disable_s2mm_intr(plblock)  ioctl(plblock->fd, CRASH_SET_INTERRUPTS, ~DMA_S2MM_INTERRUPT & ioctl(plblock->fd, CRASH_GET_INTERRUPTS, 0))
#define crash_enable_mm2s_intr(plblock)   ioctl(plblock->fd, CRASH_SET_INTERRUPTS,  DMA_MM2S_INTERRUPT | ioctl(plblock->fd, CRASH_GET_INTERRUPTS, 0))
#define crash_disable_mm2s_intr(plblock)  ioctl(plblock->fd, CRASH_SET_INTERRUPTS, ~DMA_MM2S_INTERRUPT & ioctl(plblock->fd, CRASH_GET_INTERRUPTS, 0))

#ifdef __cplusplus
}
#endif

#endif

