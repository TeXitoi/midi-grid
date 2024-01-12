#![no_main]
#![no_std]

use keyberon::debounce::Debouncer;
use keyberon::layout::Event;
use keyberon::matrix::Matrix;
use panic_halt as _;
use rtic::app;
use stm32f1xx_hal::gpio::{EPin, Input, Output, PullUp, PushPull};
use stm32f1xx_hal::prelude::*;
use stm32f1xx_hal::usb::{self, Peripheral, UsbBus, UsbBusType};
use stm32f1xx_hal::{pac, timer};
use usb_device::bus::UsbBusAllocator;
use usb_device::class::UsbClass as _;
use usb_device::prelude::*;
use usbd_midi::data::byte::u7::U7;
use usbd_midi::data::midi::channel::Channel;
use usbd_midi::data::midi::message::Message;
use usbd_midi::data::midi::notes::Note;
use usbd_midi::data::usb::constants::USB_CLASS_NONE;
use usbd_midi::data::usb_midi::cable_number::CableNumber;
use usbd_midi::data::usb_midi::usb_midi_event_packet::UsbMidiEventPacket;
use usbd_midi::midi_device::MidiClass;

type UsbClass = MidiClass<'static, UsbBusType>;
type UsbDevice = usb_device::device::UsbDevice<'static, UsbBusType>;

#[app(device = stm32f1xx_hal::pac, peripherals = true)]
mod app {
    use super::*;

    #[shared]
    struct Shared {
        usb_dev: UsbDevice,
        usb_class: UsbClass,
    }
    #[local]
    struct Local {
        matrix: Matrix<EPin<Input<PullUp>>, EPin<Output<PushPull>>, 12, 5>,
        debouncer: Debouncer<[[bool; 12]; 5]>,
        timer: timer::CounterHz<pac::TIM3>,
    }

    #[init(local = [bus: Option<UsbBusAllocator<usb::UsbBusType>> = None])]
    fn init(mut c: init::Context) -> (Shared, Local, init::Monotonics) {
        let mut flash = c.device.FLASH.constrain();
        let rcc = c.device.RCC.constrain();

        // set 0x424C in DR10 for dfu on reset
        let bkp = rcc.bkp.constrain(c.device.BKP, &mut c.device.PWR);
        bkp.write_data_register_low(9, 0x424C);

        let clocks = rcc
            .cfgr
            .use_hse(8.MHz())
            .sysclk(72.MHz())
            .pclk1(36.MHz())
            .freeze(&mut flash.acr);

        let mut gpioa = c.device.GPIOA.split();
        let mut gpiob = c.device.GPIOB.split();
        let mut gpioc = c.device.GPIOC.split();

        // BluePill board has a pull-up resistor on the D+ line.
        // Pull the D+ pin down to send a RESET condition to the USB bus.
        let mut usb_dp = gpioa.pa12.into_push_pull_output(&mut gpioa.crh);
        usb_dp.set_low();
        cortex_m::asm::delay(clocks.sysclk().raw() / 100);

        let mut led = gpioc.pc13.into_push_pull_output(&mut gpioc.crh);
        led.set_high();

        let usb_dm = gpioa.pa11;
        let usb_dp = usb_dp.into_floating_input(&mut gpioa.crh);

        let usb = Peripheral {
            usb: c.device.USB,
            pin_dm: usb_dm,
            pin_dp: usb_dp,
        };

        *c.local.bus = Some(UsbBus::new(usb));
        let usb_bus = c.local.bus.as_ref().unwrap();

        let usb_class = MidiClass::new(&usb_bus, 1, 1).unwrap();
        let usb_dev = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0x16c0, 0x27dd))
            .product("MIDI Grid")
            .device_class(USB_CLASS_NONE)
            .build();

        let mut timer = c.device.TIM3.counter_hz(&clocks);
        timer.start(1.kHz()).unwrap();
        timer.listen(timer::Event::Update);

        let matrix = Matrix::new(
            [
                gpiob.pb12.into_pull_up_input(&mut gpiob.crh).erase(),
                gpiob.pb13.into_pull_up_input(&mut gpiob.crh).erase(),
                gpiob.pb14.into_pull_up_input(&mut gpiob.crh).erase(),
                gpiob.pb15.into_pull_up_input(&mut gpiob.crh).erase(),
                gpioa.pa8.into_pull_up_input(&mut gpioa.crh).erase(),
                gpioa.pa9.into_pull_up_input(&mut gpioa.crh).erase(),
                gpioa.pa10.into_pull_up_input(&mut gpioa.crh).erase(),
                gpiob.pb5.into_pull_up_input(&mut gpiob.crl).erase(),
                gpiob.pb6.into_pull_up_input(&mut gpiob.crl).erase(),
                gpiob.pb7.into_pull_up_input(&mut gpiob.crl).erase(),
                gpiob.pb8.into_pull_up_input(&mut gpiob.crh).erase(),
                gpiob.pb9.into_pull_up_input(&mut gpiob.crh).erase(),
            ],
            [
                gpiob.pb11.into_push_pull_output(&mut gpiob.crh).erase(),
                gpiob.pb10.into_push_pull_output(&mut gpiob.crh).erase(),
                gpiob.pb1.into_push_pull_output(&mut gpiob.crl).erase(),
                gpiob.pb0.into_push_pull_output(&mut gpiob.crl).erase(),
                gpioa.pa7.into_push_pull_output(&mut gpioa.crl).erase(),
            ],
        );

        (
            Shared { usb_dev, usb_class },
            Local {
                timer,
                debouncer: Debouncer::new([[false; 12]; 5], [[false; 12]; 5], 5),
                matrix: matrix.unwrap(),
            },
            init::Monotonics(),
        )
    }

    #[task(binds = USB_HP_CAN_TX, priority = 2, shared = [usb_dev, usb_class])]
    fn usb_tx(c: usb_tx::Context) {
        (c.shared.usb_dev, c.shared.usb_class)
            .lock(|usb_dev, usb_class| usb_poll(usb_dev, usb_class))
    }

    #[task(binds = USB_LP_CAN_RX0, priority = 2, shared = [usb_dev, usb_class])]
    fn usb_rx(c: usb_rx::Context) {
        (c.shared.usb_dev, c.shared.usb_class)
            .lock(|usb_dev, usb_class| usb_poll(usb_dev, usb_class))
    }

    #[task(binds = TIM3, priority = 1, shared = [usb_class], local = [matrix, debouncer, timer])]
    fn tick(mut c: tick::Context) {
        c.local.timer.clear_interrupt(timer::Event::Update);

        let state = c.local.matrix.get().unwrap();
        for event in c.local.debouncer.events(state) {
            let Some(midi_event) = midi_event(event, &state) else {
                continue;
            };
            while c
                .shared
                .usb_class
                .lock(|midi| midi.send_message(clone_midi_event(&midi_event)))
                .is_err()
            {}
        }
    }
}

fn clone_midi_event(e: &UsbMidiEventPacket) -> UsbMidiEventPacket {
    let mut res = core::mem::MaybeUninit::<UsbMidiEventPacket>::uninit();
    unsafe {
        core::ptr::copy_nonoverlapping(e, res.as_mut_ptr(), 1);
        res.assume_init()
    }
}

fn midi_event(event: Event, state: &[[bool; 12]; 5]) -> Option<UsbMidiEventPacket> {
    let midi @ (channel, note) = midi_from_coord(event.coord());
    let message = if event.is_press() {
        Message::NoteOn
    } else {
        if state
            .iter()
            .enumerate()
            .flat_map(|(y, xs)| {
                xs.iter()
                    .enumerate()
                    .filter_map(move |(x, b)| b.then_some((y as u8, x as u8)))
            })
            .map(midi_from_coord)
            .any(|m| m == midi)
        {
            return None;
        }
        Message::NoteOff
    };
    Some(UsbMidiEventPacket {
        cable_number: CableNumber::Cable0,
        message: message(channel, note, U7::MAX),
    })
}

fn midi_from_coord((y, x): (u8, u8)) -> (Channel, Note) {
    if (y, x) == (4, 11) {
        (Channel::Channel2, Note::C4)
    } else {
        (Channel::Channel1, unsafe {
            core::mem::transmute(127.min(41 + x * 2 + (4 - y) * 5))
        })
    }
}

fn usb_poll(usb_dev: &mut UsbDevice, usb_class: &mut UsbClass) {
    if usb_dev.poll(&mut [usb_class]) {
        usb_class.poll();
    }
}
