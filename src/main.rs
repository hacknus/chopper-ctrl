//#![deny(unsafe_code)]
#![no_main]
#![no_std]
// For allocator
#![feature(lang_items)]
#![feature(alloc_error_handler)]

extern crate alloc;

use crate::commands::{extract_command, send_housekeeping, Commands};
use crate::devices::led::LED;
use crate::mav::Mav;
use crate::usb::{usb_init, usb_println, usb_read, USB_MESSAGE_LEN};
use crate::utils::MotorData;
use alloc::sync::Arc;
use arrform::{arrform, ArrForm};
use core::alloc::Layout;
use core::ptr;
use core::sync::atomic::{AtomicU32, Ordering};
use cortex_m::asm;
use cortex_m::peripheral::DWT;
use cortex_m_rt::exception;
use cortex_m_rt::{entry, ExceptionFrame};
use freertos_rust::*;
use panic_halt as _;
use stm32f4xx_hal::otg_fs::USB;
use stm32f4xx_hal::pac::CorePeripherals;
use stm32f4xx_hal::timer::{Channel, Channel1};
use stm32f4xx_hal::{
    interrupt,
    pac::{self, Interrupt},
    prelude::*,
};

mod commands;
mod devices;
mod mav;
mod pid;
mod usb;
mod utils;

#[global_allocator]
static GLOBAL: FreeRtosAllocator = FreeRtosAllocator;

// Global counter
static ENC_COUNT: AtomicU32 = AtomicU32::new(0);

const REV_PER_SECOND: f32 = 100.0; // 100 pulses per revolution

fn cycles_to_us(cycles: u32, sysclk_hz: u32) -> u64 {
    (cycles as u64).saturating_mul(1_000_000u64) / (sysclk_hz as u64)
}

use cortex_m_rt::pre_init;

const BOOTLOADER_MAGIC: u32 = 0xDEADBEEF;
const MAGIC_ADDR: *mut u32 = 0x2001_FFFC as *mut u32; // End of 128KB SRAM

pub fn jump_to_bootloader_via_reset() -> ! {
    unsafe {
        ptr::write_volatile(MAGIC_ADDR, BOOTLOADER_MAGIC);
        cortex_m::peripheral::SCB::sys_reset();
    }
}

#[pre_init]
unsafe fn check_bootloader_flag() {
    use cortex_m::peripheral::SCB;

    // this is required so the application automatically starts after flashing via DFU
    const FLASH_BASE: u32 = 0x08000000;

    // Only set if not already set (avoid overriding bootloader setup)
    let scb = &*SCB::ptr();
    if scb.vtor.read() != FLASH_BASE {
        scb.vtor.write(FLASH_BASE);
    }

    // Check magic value at start of reset handler
    if ptr::read_volatile(MAGIC_ADDR) == BOOTLOADER_MAGIC {
        // Clear magic value (like the assembly does)
        ptr::write_volatile(MAGIC_ADDR, 0);

        // Enable SYSCFG clock (RCC_APB2ENR = 0x40023844)
        ptr::write_volatile(0x40023844 as *mut u32, 0x00004000);

        // Map ROM at zero (SYSCFG_MEMRMP = 0x40013800)
        ptr::write_volatile(0x40013800 as *mut u32, 0x00000001);

        // Load bootloader stack pointer and reset vector
        let bootloader_base = 0x1FFF0000u32;
        let sp = ptr::read_volatile(bootloader_base as *const u32);
        let pc = ptr::read_volatile((bootloader_base + 4) as *const u32);

        // Use the transmute approach - it should work after memory remap
        cortex_m::asm::dsb();
        cortex_m::asm::isb();

        // Set stack pointer manually using register write
        core::arch::asm!("msr msp, {}", in(reg) sp);

        // Jump to bootloader
        let bootloader_entry: extern "C" fn() -> ! = core::mem::transmute(pc);
        bootloader_entry();
    }
}

#[entry]
fn main() -> ! {
    let mut dp = pac::Peripherals::take().unwrap();
    let mut core = CorePeripherals::take().unwrap();

    // Enable tracing & the cycle counter
    core.DCB.enable_trace();
    core.DWT.enable_cycle_counter();

    let rcc = dp.RCC.constrain();

    let clocks = rcc
        .cfgr
        .use_hse(12.MHz())
        .sysclk(96.MHz())
        .hclk(96.MHz())
        .require_pll48clk()
        .pclk1(24.MHz()) // APB1
        .pclk2(24.MHz()) // APB2
        .freeze();

    let mut delay = dp.TIM1.delay_us(&clocks);
    delay.delay(100.millis()); // apparently required for USB to set up properly...

    // initialize ports
    let gpioa = dp.GPIOA.split();
    let gpiob = dp.GPIOB.split();
    let gpioc = dp.GPIOC.split();

    // initialize pins
    let motor_pin = Channel1::new(gpioa.pa6);

    // initialize heater pwm
    let mut motor_pwm = dp.TIM3.pwm_hz(motor_pin, 1000.Hz(), &clocks);
    motor_pwm.set_duty(Channel::C1, 0);
    motor_pwm.enable(Channel::C1);

    let pwm_max_duty = motor_pwm.get_max_duty();

    // Configure PB6 as input with pull-up
    let mut enc_input = gpiob.pb6.into_pull_up_input();

    // Make it an interrupt source
    enc_input.make_interrupt_source(&mut dp.SYSCFG.constrain());
    enc_input.trigger_on_edge(&mut dp.EXTI, stm32f4xx_hal::gpio::Edge::Rising);
    enc_input.enable_interrupt(&mut dp.EXTI);

    // Enable the EXTI9_5 interrupt
    unsafe {
        cortex_m::peripheral::NVIC::unmask(pac::Interrupt::EXTI9_5);
    }

    // initialize leds
    let mut stat_led = LED::new(gpioc.pc1.into_push_pull_output());

    // initialize usb
    let usb = USB {
        usb_global: dp.OTG_FS_GLOBAL,
        usb_device: dp.OTG_FS_DEVICE,
        usb_pwrclk: dp.OTG_FS_PWRCLK,
        pin_dm: stm32f4xx_hal::gpio::alt::otg_fs::Dm::PA11(gpioa.pa11.into_alternate()),
        pin_dp: stm32f4xx_hal::gpio::alt::otg_fs::Dp::PA12(gpioa.pa12.into_alternate()),
        hclk: clocks.hclk(),
    };

    delay.delay(500.millis());

    unsafe {
        usb_init(usb);
        cortex_m::peripheral::NVIC::unmask(Interrupt::OTG_FS);
    }

    delay.delay(500.millis());

    // send the build version to the screen
    let version = env!("CARGO_PKG_VERSION");
    let name = env!("CARGO_PKG_NAME");
    let branch = option_env!("GIT_BRANCH").unwrap_or("(No Git Branch Found)");
    let commit = option_env!("GIT_HASH").unwrap_or("(No Git Hash Found)");
    let software_version = arrform!(
        128,
        "[REP] Boot Info: {} v{} on {} @ {}",
        name,
        version,
        branch,
        commit
    );

    usb_println(software_version.as_str());

    let motor_data = MotorData::default();
    let motor_data_container =
        Arc::new(Mutex::new(motor_data).expect("Failed to create motor 2 data guard mutex"));
    let motor_data_container_motor = motor_data_container.clone();
    let motor_data_container_usb = motor_data_container.clone();

    let motor_command_queue = Arc::new(Queue::new(10).unwrap());
    let motor_command_queue_motor = motor_command_queue.clone();
    let motor_command_queue_comm = motor_command_queue;

    stat_led.on();
    delay.delay(500.millis());
    stat_led.off();
    delay.delay(500.millis());
    stat_led.on();

    Task::new()
        .name("PID MOTOR TASK")
        .stack_size(1024)
        .priority(TaskPriority(3))
        .start(move || {
            let mut duty: u16 = 0;
            let mut pid = pid::PID::new();

            let sysclk_hz: u32 = 96_000_000;
            let mut velocity_mav = Mav::new();

            // Track pulses over 50ms windows
            let mut last_measurement_time = DWT::cycle_count();
            let mut last_enc_count = 0u32;
            let measurement_window_us = 50_000u64; // 50ms

            loop {
                let now_cycles = DWT::cycle_count();
                let elapsed_cycles = now_cycles.wrapping_sub(last_measurement_time);
                let elapsed_us = cycles_to_us(elapsed_cycles, sysclk_hz);

                // Only calculate speed every 50ms
                if elapsed_us >= measurement_window_us {
                    let current_enc_count = ENC_COUNT.load(Ordering::Relaxed);
                    let pulse_count = current_enc_count.wrapping_sub(last_enc_count);

                    // Speed = (pulses / 100 pulses_per_rev) / (time_in_seconds)
                    let actual_speed =
                        (pulse_count as f32 / REV_PER_SECOND) / (elapsed_us as f32 / 1_000_000.0);

                    velocity_mav.push(Some(actual_speed));

                    last_enc_count = current_enc_count;
                    last_measurement_time = now_cycles;

                    let now = FreeRtosUtils::get_tick_count() as u64;

                    if now > pid.prev_time + 5 {
                        let averaged_speed = velocity_mav.evaluate().unwrap_or(0.0);
                        if pid.enabled {
                            let pid_output = pid.get_value(averaged_speed, now);
                            duty = (pid_output.clamp(0.0, 1.0) * pwm_max_duty as f32) as u16;
                        } else {
                            duty = 0;
                        }

                        match motor_data_container_motor.lock(Duration::ms(1)) {
                            Ok(mut motor_data_temp) => {
                                motor_data_temp.target = pid.target;
                                motor_data_temp.actual = averaged_speed;
                                motor_data_temp.pwr = duty;
                                motor_data_temp.p = pid.p;
                                motor_data_temp.i = pid.i;
                                motor_data_temp.d = pid.d;
                                motor_data_temp.enc = current_enc_count;
                            }
                            Err(_) => {}
                        }
                    }

                    motor_pwm.set_duty(Channel::C1, duty);
                }

                if let Ok(cmd) = motor_command_queue_motor.receive(Duration::ms(1)) {
                    match cmd {
                        Commands::SetSpeed(speed) => pid.target = speed,
                        Commands::SetKp(kp) => pid.kp = kp,
                        Commands::SetKi(ki) => pid.ki = ki,
                        Commands::SetKd(kd) => pid.kd = kd,
                        Commands::Start => pid.enabled = true,
                        Commands::Reset => pid.i = 0.0,
                        Commands::Stop => {
                            pid.enabled = false;
                            motor_pwm.set_duty(Channel::C1, 0);
                        }
                    }
                }
            }
        })
        .unwrap();

    Task::new()
        .name("USB TASK")
        .stack_size(1024)
        .priority(TaskPriority(3))
        .start(move || {
            let mut hk = true;
            let mut hk_period = 100.0;
            let mut motor = MotorData::default();

            loop {
                // gather motor state
                match motor_data_container_usb.lock(Duration::ms(1)) {
                    Ok(motor_data_temp) => {
                        motor = motor_data_temp.clone();
                    }
                    Err(_) => {}
                }

                if hk {
                    send_housekeeping(
                        motor.enc,
                        motor.target,
                        motor.actual,
                        motor.p,
                        motor.i,
                        motor.d,
                        motor.pwr,
                        "",
                    );
                }

                let mut message_bytes = [0; USB_MESSAGE_LEN];
                if usb_read(&mut message_bytes) {
                    match core::str::from_utf8(&message_bytes) {
                        Ok(cmd) => {
                            extract_command(
                                cmd,
                                &motor_command_queue_comm,
                                &mut hk,
                                &mut hk_period,
                            );
                        }
                        Err(_) => {}
                    }
                }

                freertos_rust::CurrentTask::delay(Duration::ms(hk_period as u32 / 2));
                stat_led.toggle();
                freertos_rust::CurrentTask::delay(Duration::ms(hk_period as u32 / 2));
                stat_led.toggle();
            }
        })
        .unwrap();

    FreeRtosUtils::start_scheduler();
}

#[interrupt]
fn EXTI9_5() {
    unsafe {
        let exti = &(*pac::EXTI::ptr());
        exti.pr.write(|w| w.pr6().set_bit());
    }

    ENC_COUNT.fetch_add(1, Ordering::Relaxed);
}

#[exception]
#[allow(non_snake_case)]
unsafe fn DefaultHandler(_irqn: i16) {
    // custom default handler
    // irqn is negative for Cortex-M exceptions
    // irqn is positive for device specific (line IRQ)
    // panic!("Exception: {}", irqn);
}

#[exception]
#[allow(non_snake_case)]
unsafe fn HardFault(_ef: &ExceptionFrame) -> ! {
    loop {}
}

// define what happens in an Out Of Memory (OOM) condition
#[alloc_error_handler]
fn alloc_error(_layout: Layout) -> ! {
    asm::bkpt();
    loop {}
}

#[no_mangle]
#[allow(non_snake_case, unused_variables)]
fn vApplicationStackOverflowHook(pxTask: FreeRtosTaskHandle, pcTaskName: FreeRtosCharPtr) {
    asm::bkpt();
}
