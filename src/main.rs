#![no_main]
#![no_std]

use core::convert::Infallible;
use embedded_hal::digital::v2::{InputPin, OutputPin};
use generic_array::typenum::{U12, U5};
use keyberon::debounce::Debouncer;
use keyberon::impl_heterogenous_array;
use keyberon::layout::Event;
use keyberon::matrix::{Matrix, PressedKeys};
use panic_halt as _;
use rtic::app;
use stm32f1xx_hal::gpio::{gpioa::*, gpiob::*, Input, Output, PullUp, PushPull};
use stm32f1xx_hal::prelude::*;
use stm32f1xx_hal::usb::{Peripheral, UsbBus, UsbBusType};
use stm32f1xx_hal::{pac, timer};
use usb_device::bus::UsbBusAllocator;
use usb_device::class::UsbClass as _;
use usb_device::prelude::*;
use usbd_midi::data::byte::u7::U7;
use usbd_midi::data::midi::channel::Channel;
use usbd_midi::data::midi::message::Message;
use usbd_midi::data::usb::constants::USB_CLASS_NONE;
use usbd_midi::data::usb_midi::cable_number::CableNumber;
use usbd_midi::data::usb_midi::usb_midi_event_packet::UsbMidiEventPacket;
use usbd_midi::midi_device::MidiClass;

type UsbClass = MidiClass<'static, UsbBusType>;
type UsbDevice = usb_device::device::UsbDevice<'static, UsbBusType>;

pub struct Cols(
    pub PB12<Input<PullUp>>,
    pub PB13<Input<PullUp>>,
    pub PB14<Input<PullUp>>,
    pub PB15<Input<PullUp>>,
    pub PA8<Input<PullUp>>,
    pub PA9<Input<PullUp>>,
    pub PA10<Input<PullUp>>,
    pub PB5<Input<PullUp>>,
    pub PB6<Input<PullUp>>,
    pub PB7<Input<PullUp>>,
    pub PB8<Input<PullUp>>,
    pub PB9<Input<PullUp>>,
);
impl_heterogenous_array! {
    Cols,
    dyn InputPin<Error = Infallible>,
    U12,
    [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11]
}

pub struct Rows(
    pub PB11<Output<PushPull>>,
    pub PB10<Output<PushPull>>,
    pub PB1<Output<PushPull>>,
    pub PB0<Output<PushPull>>,
    pub PA7<Output<PushPull>>,
);
impl_heterogenous_array! {
    Rows,
    dyn OutputPin<Error = Infallible>,
    U5,
    [0, 1, 2, 3, 4]
}

#[app(device = stm32f1xx_hal::pac, peripherals = true)]
const APP: () = {
    struct Resources {
        usb_dev: UsbDevice,
        usb_class: UsbClass,
        matrix: Matrix<Cols, Rows>,
        debouncer: Debouncer<PressedKeys<U5, U12>>,
        timer: timer::CountDownTimer<pac::TIM3>,
    }

    #[init]
    fn init(mut c: init::Context) -> init::LateResources {
        static mut USB_BUS: Option<UsbBusAllocator<UsbBusType>> = None;

        let mut flash = c.device.FLASH.constrain();
        let mut rcc = c.device.RCC.constrain();

        // set 0x424C in DR10 for dfu on reset
        let bkp = rcc
            .bkp
            .constrain(c.device.BKP, &mut rcc.apb1, &mut c.device.PWR);
        bkp.write_data_register_low(9, 0x424C);

        let clocks = rcc
            .cfgr
            .use_hse(8.mhz())
            .sysclk(72.mhz())
            .pclk1(36.mhz())
            .freeze(&mut flash.acr);

        let mut gpioa = c.device.GPIOA.split(&mut rcc.apb2);
        let mut gpiob = c.device.GPIOB.split(&mut rcc.apb2);
        let mut gpioc = c.device.GPIOC.split(&mut rcc.apb2);

        // BluePill board has a pull-up resistor on the D+ line.
        // Pull the D+ pin down to send a RESET condition to the USB bus.
        let mut usb_dp = gpioa.pa12.into_push_pull_output(&mut gpioa.crh);
        usb_dp.set_low().unwrap();
        cortex_m::asm::delay(clocks.sysclk().0 / 100);

        let mut led = gpioc.pc13.into_push_pull_output(&mut gpioc.crh);
        led.set_high().unwrap();

        let usb_dm = gpioa.pa11;
        let usb_dp = usb_dp.into_floating_input(&mut gpioa.crh);

        let usb = Peripheral {
            usb: c.device.USB,
            pin_dm: usb_dm,
            pin_dp: usb_dp,
        };

        *USB_BUS = Some(UsbBus::new(usb));
        let usb_bus = USB_BUS.as_ref().unwrap();

        let usb_class = MidiClass::new(&usb_bus, 1, 1).unwrap();
        let usb_dev = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0x16c0, 0x27dd))
            .product("MIDI Grid")
            .device_class(USB_CLASS_NONE)
            .build();

        let mut timer =
            timer::Timer::tim3(c.device.TIM3, &clocks, &mut rcc.apb1).start_count_down(1.khz());
        timer.listen(timer::Event::Update);

        let matrix = Matrix::new(
            Cols(
                gpiob.pb12.into_pull_up_input(&mut gpiob.crh),
                gpiob.pb13.into_pull_up_input(&mut gpiob.crh),
                gpiob.pb14.into_pull_up_input(&mut gpiob.crh),
                gpiob.pb15.into_pull_up_input(&mut gpiob.crh),
                gpioa.pa8.into_pull_up_input(&mut gpioa.crh),
                gpioa.pa9.into_pull_up_input(&mut gpioa.crh),
                gpioa.pa10.into_pull_up_input(&mut gpioa.crh),
                gpiob.pb5.into_pull_up_input(&mut gpiob.crl),
                gpiob.pb6.into_pull_up_input(&mut gpiob.crl),
                gpiob.pb7.into_pull_up_input(&mut gpiob.crl),
                gpiob.pb8.into_pull_up_input(&mut gpiob.crh),
                gpiob.pb9.into_pull_up_input(&mut gpiob.crh),
            ),
            Rows(
                gpiob.pb11.into_push_pull_output(&mut gpiob.crh),
                gpiob.pb10.into_push_pull_output(&mut gpiob.crh),
                gpiob.pb1.into_push_pull_output(&mut gpiob.crl),
                gpiob.pb0.into_push_pull_output(&mut gpiob.crl),
                gpioa.pa7.into_push_pull_output(&mut gpioa.crl),
            ),
        );

        init::LateResources {
            usb_dev,
            usb_class,
            timer,
            debouncer: Debouncer::new(PressedKeys::default(), PressedKeys::default(), 5),
            matrix: matrix.unwrap(),
        }
    }

    #[task(binds = USB_HP_CAN_TX, priority = 2, resources = [usb_dev, usb_class])]
    fn usb_tx(mut c: usb_tx::Context) {
        usb_poll(&mut c.resources.usb_dev, &mut c.resources.usb_class);
    }

    #[task(binds = USB_LP_CAN_RX0, priority = 2, resources = [usb_dev, usb_class])]
    fn usb_rx(mut c: usb_rx::Context) {
        usb_poll(&mut c.resources.usb_dev, &mut c.resources.usb_class);
    }

    #[task(binds = TIM3, priority = 1, resources = [usb_class, matrix, debouncer, timer])]
    fn tick(mut c: tick::Context) {
        c.resources.timer.clear_update_interrupt_flag();

        for event in c
            .resources
            .debouncer
            .events(c.resources.matrix.get().unwrap())
        {
            while let Err(_) = c
                .resources
                .usb_class
                .lock(|midi| midi.send_message(midi_event(event)))
            {}
        }
    }
};

fn midi_event(event: Event) -> UsbMidiEventPacket {
    let (y, x) = event.coord();
    let note = 127.min(42 + x * 3 + (5 - y) * 2);
    let note = unsafe { core::mem::transmute(note) };
    let message = if event.is_press() {
        Message::NoteOn
    } else {
        Message::NoteOff
    };
    UsbMidiEventPacket {
        cable_number: CableNumber::Cable0,
        message: message(Channel::Channel1, note, U7::MAX),
    }
}

fn usb_poll(usb_dev: &mut UsbDevice, usb_class: &mut UsbClass) {
    if usb_dev.poll(&mut [usb_class]) {
        usb_class.poll();
    }
}
