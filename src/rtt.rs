//! Implementation of the SEGGER RTT Protocol
//!
//! ---

#![no_main]
#![no_std]

//use core::option::Option;

use core::cell::RefCell;
use core::ptr::{write_volatile, read_volatile, copy_nonoverlapping};
use core::marker::Send;

use cortex_m::interrupt;
use cortex_m::interrupt::Mutex;
use cortex_m::asm;

use core::fmt::{self, Write};
use core::cmp;

#[repr(C)]
#[derive(Copy, Clone)]
struct RTTUp {
    name: *const u8,
    buffer: *const u8,
    size: u32,
    wr_off: u32,
    rd_off: u32,
    flags: u32
}

#[repr(C)]
#[derive(Copy, Clone)]
struct RTTDown {
    name: *const u8,
    buffer: *const u8,
    size: u32,
    wr_off: u32,
    rd_off: u32,
    flags: u32
}

#[repr(C)]
#[derive(Copy, Clone)]
struct RTTCB {
    id: [u8; 16],
    upBuffers: u32,
    downBuffers: u32,
    up: RTTUp,
    down: RTTDown
}

unsafe impl Send for RTTCB {}

impl RTTCB {

    pub const fn newConst() -> RTTCB {
        RTTCB {
            id: [0; 16],
            upBuffers: 1,
            downBuffers: 1,
            up: RTTUp {
                name: 0 as *const u8,
                buffer: 0 as *const u8,
                size: 16,
                wr_off: 0,
                rd_off: 0,
                flags: 0
            },
            down: RTTDown {
                name: 0 as *const u8,
                buffer: 0 as *const u8,
                size: 16,
                wr_off: 0,
                rd_off: 0,
                flags: 0
            }
        }
    }

    pub fn new(upBuf: &[u8;16], downBuf: &[u8;16]) -> RTTCB {
        RTTCB {
            id: [0; 16],
            upBuffers: 1,
            downBuffers: 1,
            up: RTTUp {
                name: "defaultUp".as_ptr(),
                buffer: upBuf.as_ptr(),
                size: upBuf.len() as u32,
                wr_off: 0,
                rd_off: 0,
                flags: 0
            },
            down: RTTDown {
                name: "defaultDown".as_ptr(),
                buffer: downBuf.as_ptr(),
                size: downBuf.len() as u32,
                wr_off: 0,
                rd_off: 0,
                flags: 0
            }
        }
    }
}

static RTTUpBuffer: Mutex<RefCell<[u8; 16]>> = Mutex::new(RefCell::new([0; 16]));
static RTTDownBuffer: Mutex<RefCell<[u8; 16]>> = Mutex::new(RefCell::new([0; 16]));

static RTTCb: Mutex<RefCell<RTTCB>> = Mutex::new(RefCell::new(RTTCB::newConst()));

pub struct RTT {
    chan: u32
}

pub fn get_chan(c: u32) -> RTT {
    RTT {
        chan: c
    }
}

pub fn init() {

    interrupt::free(|cs|
    {
        let upBuff = &(*RTTUpBuffer.borrow(cs).borrow());
        let downBuff = &(*RTTDownBuffer.borrow(cs).borrow());

        let mut cb = RTTCB::new(upBuff, downBuff);

        cb.id[0] = 'S' as u8;
        cb.id[1] = 'E' as u8;
        cb.id[2] = 'G' as u8;
        cb.id[3] = 'G' as u8;
        cb.id[4] = 'E' as u8;
        cb.id[5] = 'R' as u8;
        cb.id[6] = ' ' as u8;
        cb.id[7] = 'R' as u8;
        cb.id[8] = 'T' as u8;
        cb.id[9] = 'T' as u8;

        RTTCb.borrow(cs).replace(cb);
    });
}

impl fmt::Write for RTT {
    fn write_str(&mut self, s: &str) -> fmt::Result{

        interrupt::free(|cs|
        {
            let mut cb = RTTCb.borrow(cs).borrow_mut();

            let size = cb.up.size as usize;
            let wr_offset = unsafe { read_volatile(&cb.up.wr_off) } as usize;
            let rd_offset = unsafe { read_volatile(&cb.up.rd_off) } as usize;

            let mut space_left = 0;

            if wr_offset == rd_offset {
                space_left = size;
            } else if wr_offset > rd_offset {
                space_left = size - (wr_offset - rd_offset);
            } else {
                space_left = rd_offset - wr_offset;
            }

            // Get the wriable potions of the string based on
            // space remaining in the circular buffer
            let mut writeable_str = "";

            if space_left >= s.len() {
                writeable_str = s;
            } else {
                writeable_str = &s[..space_left]
            }

            let wrap_idx = (size - wr_offset) as usize;

            let mut left_str = "";
            let mut right_str = "";

            if wrap_idx >= writeable_str.len() {
                left_str = &writeable_str;
            } else {
                let strs = writeable_str.split_at(wrap_idx);
                left_str = strs.0;
                right_str = strs.1;
            }

            let mut upBuffer = &mut (*RTTUpBuffer.borrow(cs).borrow_mut());
            // Write Left String
            if left_str.len() > 0 {
                let dataPtr = &mut upBuffer[wr_offset];

                unsafe {
                    copy_nonoverlapping(left_str.as_ptr(), dataPtr as *mut u8, left_str.len());
                }
            }

            // Write Right String
            if right_str.len() > 0 {
                let dataPtr = &mut upBuffer[0];

                unsafe {
                    copy_nonoverlapping(right_str.as_ptr(), dataPtr as *mut u8, right_str.len());
                }
            }

            let offset_ref_mut = &mut cb.up.wr_off;
            let new_offset = (wr_offset + s.len()) % size;

            // Ensure all buffer writes occur before updating index
            asm::dmb();

            unsafe {
                write_volatile(offset_ref_mut, new_offset as u32);
            }

            asm::dmb();
        });

        Ok(())
    }

}

impl RTT {

    pub fn recv_bytes(&self, out_buff: &mut [u8]) -> usize {

        let mut copy_size: usize = 0 ;

        interrupt::free(|cs|
        {
            let down = &mut RTTCb.borrow(cs).borrow_mut().down;

            let buff_size = down.size as usize;
            let wr_offset = unsafe { read_volatile(&down.wr_off) } as usize;
            let rd_offset = unsafe { read_volatile(&down.rd_off) } as usize;

            if wr_offset != rd_offset {

                if rd_offset > wr_offset {
                    
                    let left_size = buff_size - rd_offset;

                    let right_size = wr_offset;

                    if right_size == 0 || out_buff.len() <= left_size {
                        // Only copy left bytes

                        let left_copy_size = cmp::min(left_size, out_buff.len());

                        let downBuffer = &(*RTTDownBuffer.borrow(cs).borrow_mut());
                        let dataPtr = &downBuffer[rd_offset];

                        unsafe {
                            copy_nonoverlapping(dataPtr as *const u8, out_buff.as_mut_ptr(), left_copy_size);
                        }

                        copy_size = left_copy_size;
                    } else {
                        // Copying entire left side, and some right side

                        let (mut out_buff_left, mut out_buff_right) = out_buff.split_at_mut(left_size);

                        let downBuffer = &(*RTTDownBuffer.borrow(cs).borrow_mut());
                        let dataPtr = &downBuffer[rd_offset];

                        unsafe {
                            copy_nonoverlapping(dataPtr as *const u8, out_buff_left.as_mut_ptr(), left_size);
                        }

                        let right_copy_size = cmp::min(right_size, out_buff_right.len());

                        let dataPtr = &downBuffer[0];

                        unsafe {
                            copy_nonoverlapping(dataPtr as *const u8, out_buff_right.as_mut_ptr(), right_copy_size);
                        }

                        copy_size = left_size + right_copy_size;
                    }
                } else {

                    let size = cmp::min(wr_offset - rd_offset, out_buff.len());

                    let downBuffer = &(*RTTDownBuffer.borrow(cs).borrow_mut());
                    let dataPtr = &downBuffer[rd_offset];

                    unsafe {
                        copy_nonoverlapping(dataPtr as *const u8, out_buff.as_mut_ptr(), size);
                    }

                    copy_size = size;
                }
            }

            asm::dmb();

            let new_rd_offset = ((rd_offset + copy_size) % buff_size) as u32;

            let rd_offset_mut = &mut down.rd_off;

            unsafe {
                write_volatile(rd_offset_mut, new_rd_offset);
            }

            asm::dmb();
        });

        copy_size
    }

    fn write_byte(&self, b: u8) {
        interrupt::free(|cs|
        {
            let mut cb = RTTCb.borrow(cs).borrow_mut();

            let offset_ref = &cb.up.wr_off;

            let offset = unsafe { read_volatile(offset_ref) };

            {
                let mut upBuffer = &mut (*RTTUpBuffer.borrow(cs).borrow_mut());

                let dataPtr = &mut upBuffer[offset as usize];

                unsafe {
                    write_volatile(dataPtr, b);
                }

                asm::dmb();
            }

            let new_offset = (offset + 1) % cb.up.size;

            unsafe {
                let offset_ref_mut = &mut cb.up.wr_off;
                write_volatile(offset_ref_mut, new_offset);
            }
        });
    }

    fn recv_byte(&self) -> (bool, u8) {

        let mut have_data = false;
        let mut byte_out: u8 = 0;

        interrupt::free(|cs|
        {
            let mut cb = RTTCb.borrow(cs).borrow_mut();

            let rd_offset = cb.down.rd_off;
            let wr_offset = cb.down.wr_off;

            if rd_offset != wr_offset {

                have_data = true;

                {
                    let rd_offset_ref = &cb.down.rd_off;

                    let rd_offset = unsafe { read_volatile(rd_offset_ref) };

                    let downBuffer = & (*RTTDownBuffer.borrow(cs).borrow_mut());

                    let dataPtr = &downBuffer[rd_offset as usize];

                    unsafe {
                        byte_out = read_volatile(dataPtr);
                    }
                }

                let rd_offset = (rd_offset + 1) % cb.down.size;

                let mut_rd_offset_ref = &mut cb.down.rd_off;

                unsafe {
                    write_volatile(mut_rd_offset_ref, rd_offset);
                }
            }
        });

        (have_data, byte_out)
    }
}

