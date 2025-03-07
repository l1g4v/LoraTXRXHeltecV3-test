#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![feature(impl_trait_in_assoc_type)]

//global allocator
extern crate alloc;
use alloc::{format, string::String};
use esp_alloc as _;
use esp_backtrace as _;

use core::{ptr::addr_of_mut, str};

use embassy_executor::Spawner;
use embassy_time::{Delay, Duration, Timer};
use embedded_hal_bus::spi::ExclusiveDevice;
use esp_hal::{
    self,
    clock::CpuClock,
    cpu_control::{CpuControl, Stack},
    dma::{DmaRxBuf, DmaTxBuf},
    dma_buffers,
    gpio::{Input, Level, Output, Pull},
    i2c::master::{Config as I2cConfig, I2c},
    spi::{
        master::{Config, Spi, SpiDmaBus},
        Mode,
    },
    time::RateExtU32,
    timer::{timg::TimerGroup, AnyTimer},
    uart::UartRx,
    Async, Blocking,
};
use esp_hal_embassy::{main, Executor};

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use log::{error, info};

//OLED
use embedded_graphics::{
    mono_font::{ascii::FONT_6X10, MonoTextStyleBuilder},
    pixelcolor::BinaryColor,
    prelude::*,
    text::{Baseline, Text},
};
use ssd1306::{mode::BufferedGraphicsMode, prelude::*, I2CDisplayInterface, Ssd1306};

//LoRa
use lora_phy::sx126x::{Sx1262, Sx126x, TcxoCtrlVoltage};
use lora_phy::{
    iv::GenericSx126xInterfaceVariant,
    mod_traits::{IrqState, RadioKind},
};
use lora_phy::{mod_params::*, sx126x};
use static_cell::StaticCell;

const LORA_FREQUENCY_IN_HZ: u32 = 915_000_000;
const LORA_BANDWIDTH: Bandwidth = Bandwidth::_250KHz;
const LORA_SPREADING_FACTOR: SpreadingFactor = SpreadingFactor::_9;
const LORA_CODING_RATE: CodingRate = CodingRate::_4_7;
const LORA_PREAMBLE_LENGTH: u16 = 8;
const LORA_TX_POWER: i32 = 22;
const LORA_BUFFER_SIZE: usize = 100;
const LORA_BOOST_RX: bool = true;

//CORE1
static mut CORE1_STACK: Stack<4096> = Stack::new();
static EXECUTOR1: StaticCell<Executor> = StaticCell::new();

static TX_CHANNEL_DATA: Channel<CriticalSectionRawMutex, ([u8; LORA_BUFFER_SIZE], usize), 1> =
    Channel::new();
static RX_CHANNEL_DATA: Channel<CriticalSectionRawMutex, (i16, i16, String), 1> = Channel::new();

#[main]
async fn main(_spawner: Spawner) {
    esp_println::logger::init_logger_from_env();

    let mut config = esp_hal::Config::default();
    config.cpu_clock = CpuClock::max();
    let peripherals = esp_hal::init(config);
    esp_alloc::heap_allocator!(72 * 1024);

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    let timer0: AnyTimer = timg0.timer0.into();
    let timer1: AnyTimer = timg0.timer1.into();
    esp_hal_embassy::init([timer0, timer1]);

    //threading
    let mut cpu_control = CpuControl::new(peripherals.CPU_CTRL);

    //I2C configuration
    let sda = peripherals.GPIO17;
    let scl = peripherals.GPIO18;
    let config = I2cConfig::default().with_frequency(RateExtU32::kHz(400));
    let i2c = I2c::new(peripherals.I2C0, config)
        .unwrap()
        .with_scl(scl)
        .with_sda(sda); //I2cDriver::new(i2c, sda, scl, &config)?;

    //toggle pin 21
    let mut rstoled = Output::new(peripherals.GPIO21, Level::High);
    rstoled.set_low();
    rstoled.set_high();
    let interface = I2CDisplayInterface::new(i2c);

    //OLED
    let mut display = Ssd1306::new(interface, DisplaySize128x64, DisplayRotation::Rotate0)
        .into_buffered_graphics_mode();
    display.init().unwrap();

    let text_style = MonoTextStyleBuilder::new()
        .font(&FONT_6X10)
        .text_color(BinaryColor::On)
        .build();

    Text::with_baseline(
        "Starting radio...",
        Point::zero(),
        text_style,
        Baseline::Top,
    )
    .draw(&mut display)
    .unwrap();

    display.flush().unwrap();

    //SPI configuration
    let sclk = peripherals.GPIO9;
    let miso = peripherals.GPIO11;
    let mosi = peripherals.GPIO10;
    let cs = peripherals.GPIO8;
    let dma_channel = peripherals.DMA_CH0;

    let (rx_buffer, rx_descriptors, tx_buffer, tx_descriptors) = dma_buffers!(32000);
    let dma_rx_buf = DmaRxBuf::new(rx_descriptors, rx_buffer).unwrap();
    let dma_tx_buf = DmaTxBuf::new(tx_descriptors, tx_buffer).unwrap();

    let spi = Spi::new(
        peripherals.SPI2,
        Config::default()
            .with_frequency(RateExtU32::kHz(100))
            .with_mode(Mode::_0),
    )
    .unwrap()
    .with_sck(sclk)
    .with_mosi(mosi)
    .with_miso(miso)
    .with_dma(dma_channel)
    .with_buffers(dma_rx_buf, dma_tx_buf)
    .into_async();

    //LoRa
    let nss = Output::new(cs, Level::High);
    let reset = Output::new(peripherals.GPIO12, Level::High);
    let dio1 = Input::new(peripherals.GPIO14, Pull::None);
    let busy = Input::new(peripherals.GPIO13, Pull::None);

    let spi = ExclusiveDevice::new(spi, nss, Delay).unwrap();
    let iv = GenericSx126xInterfaceVariant::new(reset, dio1, busy, None, None).unwrap();

    let _guard = cpu_control
        .start_app_core(unsafe { &mut *addr_of_mut!(CORE1_STACK) }, move || {
            let executor1 = EXECUTOR1.init(Executor::new());
            executor1.run(|spawner| {
                let _ = spawner.spawn(radio(spi, iv));
            });
        })
        .unwrap();

    let config = esp_hal::uart::Config::default().with_baudrate(115200);
    let uart_rx = UartRx::new(peripherals.UART0, config).unwrap().into_async();
    Timer::after(Duration::from_millis(100)).await;
    _spawner.spawn(oled_task(display)).ok();
    _spawner.spawn(uart_task(uart_rx)).ok();

    loop {
        Timer::after(Duration::from_nanos(1)).await;
    }
}

#[embassy_executor::task]
async fn uart_task(mut uart_rx: UartRx<'static, Async>) {
    info!(
        "Starting uart_task() on core {}",
        esp_hal::Cpu::current() as usize
    );
    let mut buffer = [0u8; LORA_BUFFER_SIZE];
    TX_CHANNEL_DATA.send((buffer, 1)).await;
    loop {
        let len = uart_rx.read_async(&mut buffer).await.unwrap();
        if len > 0 {
            let x = buffer.clone();
            TX_CHANNEL_DATA.send((x, len)).await;
            info!("UART RX = {:?}", &buffer[..len]);
        }
    }
}

#[embassy_executor::task]
async fn oled_task(
    mut display: Ssd1306<
        I2CInterface<I2c<'static, Blocking>>,
        DisplaySize128x64,
        BufferedGraphicsMode<DisplaySize128x64>,
    >,
) {
    info!(
        "Starting oled_task() on core {}",
        esp_hal::Cpu::current() as usize
    );
    let text_style = MonoTextStyleBuilder::new()
        .font(&FONT_6X10)
        .text_color(BinaryColor::On)
        .build();
    display.clear_buffer();
    display.flush().unwrap();
    Text::with_baseline(
        "Ready, waiting for RX...",
        Point::new(0, 0),
        text_style,
        Baseline::Top,
    )
    .draw(&mut display)
    .unwrap();
    display.flush().unwrap();
    loop {
        let (rssi, snr, msg) = RX_CHANNEL_DATA.receive().await;
        display.clear_buffer();
        display.flush().unwrap();
        Text::with_baseline(
            "Last message stadistics:",
            Point::zero(),
            text_style,
            Baseline::Top,
        )
        .draw(&mut display)
        .unwrap();
        Text::with_baseline(
            format!("RSSI: {}dBm", rssi).as_str(),
            Point::new(0, 16),
            text_style,
            Baseline::Top,
        )
        .draw(&mut display)
        .unwrap();
        Text::with_baseline(
            format!("SNR: {}dB", snr).as_str(),
            Point::new(0, 32),
            text_style,
            Baseline::Top,
        )
        .draw(&mut display)
        .unwrap();
        Text::with_baseline(
            format!("MSG: {}", msg.as_str()).as_str(),
            Point::new(0, 48),
            text_style,
            Baseline::Top,
        )
        .draw(&mut display)
        .unwrap();
        display.flush().unwrap();
    }
}

#[embassy_executor::task]
async fn radio(
    spi: ExclusiveDevice<SpiDmaBus<'static, Async>, Output<'static>, Delay>,
    iv: GenericSx126xInterfaceVariant<Output<'static>, Input<'static>>,
) {
    info!(
        "Starting radio() on core {}",
        esp_hal::Cpu::current() as usize
    );

    let config = sx126x::Config {
        chip: Sx1262,
        tcxo_ctrl: Some(TcxoCtrlVoltage::Ctrl1V7),
        use_dcdc: true,
        rx_boost: LORA_BOOST_RX,
    };
    let mut modem = Sx126x::new(spi, iv, config);

    let mut receiving_buffer = [00u8; LORA_BUFFER_SIZE];

    let mdltn_params = {
        match modem.create_modulation_params(
            LORA_SPREADING_FACTOR,
            LORA_BANDWIDTH,
            LORA_CODING_RATE,
            LORA_FREQUENCY_IN_HZ,
        ) {
            Ok(mp) => mp,
            Err(err) => {
                info!("Radio error = {:?}", err);
                return;
            }
        }
    };
    let base_param = modem
        .create_packet_params(LORA_PREAMBLE_LENGTH, false, 0, true, false, &mdltn_params)
        .unwrap();
    modem.reset(&mut Delay).await.unwrap();
    modem.init_lora(true).await.unwrap();
    modem.set_modulation_params(&mdltn_params).await.unwrap();
    modem.set_packet_params(&base_param).await.unwrap();
    modem
        .set_tx_power_and_ramp_time(LORA_TX_POWER, Some(&mdltn_params), false)
        .await
        .unwrap();
    modem.set_channel(LORA_FREQUENCY_IN_HZ).await.unwrap();
    modem.set_standby().await.unwrap();
    modem
        .set_irq_params(Some(RadioMode::Standby))
        .await
        .unwrap();
    info!("Radio initialized");
    loop {
        let tx_chn_avb = TX_CHANNEL_DATA.is_full();
        if tx_chn_avb {
            info!("Switching to TX mode");
            let (data, len) = TX_CHANNEL_DATA.receive().await;
            let tx_params = modem
                .create_packet_params(
                    LORA_PREAMBLE_LENGTH,
                    false,
                    len as u8,
                    true,
                    false,
                    &mdltn_params,
                )
                .unwrap();
            modem.set_packet_params(&tx_params).await.unwrap();
            modem.set_payload(&data[..len]).await.unwrap();
            modem
                .set_irq_params(Some(RadioMode::Transmit))
                .await
                .unwrap();
            modem
                .set_tx_power_and_ramp_time(LORA_TX_POWER, Some(&mdltn_params), true)
                .await
                .unwrap();
            modem.do_tx().await.unwrap();
            modem.await_irq().await.unwrap();

            match modem
                .process_irq_event(RadioMode::Transmit, None, true)
                .await
            {
                Ok(Some(IrqState::Done | IrqState::PreambleReceived)) => {
                    info!("TX done");
                }
                Ok(None) => {}
                Err(err) => {
                    error!("TX error = {:?}", err);
                }
            }
            info!("Switching to RX mode");
            modem
                .set_irq_params(Some(RadioMode::Standby))
                .await
                .unwrap();
            modem.set_packet_params(&base_param).await.unwrap();
            modem
                .set_irq_params(RadioMode::Receive(RxMode::Continuous).into())
                .await
                .unwrap();
            modem
                .set_tx_power_and_ramp_time(LORA_TX_POWER, Some(&mdltn_params), false)
                .await
                .unwrap();
            let _ = modem.do_rx(RxMode::Continuous).await;
        } else {
            //non-blocking receive
            match modem
                .get_rx_payload(&base_param, &mut receiving_buffer)
                .await
            {
                Ok(len) => {
                    if len > 0 {
                        let recv_str = str::from_utf8(&receiving_buffer[..len as usize]).unwrap();
                        let recv_str = String::from(recv_str);
                        info!("RX = {:?}", recv_str);
                        let packet_status = modem.get_rx_packet_status().await.unwrap();
                        let rssi = packet_status.rssi;
                        let snr = packet_status.snr;
                        RX_CHANNEL_DATA.send((rssi, snr, recv_str)).await;
                        //TODO: search alternatives to reseting the modem in order to clear the recv buffer
                        let mut del = embassy_time::Delay;
                        modem.reset(&mut del).await.unwrap();
                        modem.init_lora(true).await.unwrap();
                        modem.set_modulation_params(&mdltn_params).await.unwrap();
                        modem.set_packet_params(&base_param).await.unwrap();
                        modem
                            .set_irq_params(Some(RadioMode::Standby))
                            .await
                            .unwrap();
                        modem
                            .set_irq_params(RadioMode::Receive(RxMode::Continuous).into())
                            .await
                            .unwrap();
                        modem
                            .set_tx_power_and_ramp_time(LORA_TX_POWER, Some(&mdltn_params), false)
                            .await
                            .unwrap();
                        let _ = modem.do_rx(RxMode::Continuous).await;
                    }
                }
                Err(err) => {
                    error!("RX error = {:?}", err);
                }
            }
        }
    }
}
