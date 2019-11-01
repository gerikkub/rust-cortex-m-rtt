//! Implementation of the SEGGER RTT Protocol
//!
//! ---

#![no_main]
#![no_std]

//use core::option::Option;

use core::cell::RefCell;
use core::ptr::{write_volatile, read_volatile};
use core::marker::Send;

use cortex_m::interrupt;
use cortex_m::interrupt::Mutex;

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
                size: 256,
                wr_off: 0,
                rd_off: 0,
                flags: 0
            },
            down: RTTDown {
                name: 0 as *const u8,
                buffer: 0 as *const u8,
                size: 256,
                wr_off: 0,
                rd_off: 0,
                flags: 0
            }
        }
    }

    pub fn new(upBuf: &[u8;256], downBuf: &[u8;256]) -> RTTCB {
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

//static mut RTTUpBufferMem: [u8; 256] = [0; 256];
//static mut RTTDownBufferMem: [u8; 256] = [0; 256];

static RTTUpBuffer: Mutex<RefCell<[u8; 256]>> = Mutex::new(RefCell::new([0; 256]));
static RTTDownBuffer: Mutex<RefCell<[u8; 256]>> = Mutex::new(RefCell::new([0; 256]));

static RTTCb: Mutex<RefCell<RTTCB>> = Mutex::new(RefCell::new(RTTCB::newConst()));

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

pub fn write_byte(b: u8) {

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
        }

        let new_offset = (offset + 1) % cb.up.size;

        unsafe {
            let offset_ref_mut = &mut cb.up.wr_off;
            write_volatile(offset_ref_mut, new_offset);
        }
    });
}

pub fn recv_byte() -> (bool, u8) {

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

