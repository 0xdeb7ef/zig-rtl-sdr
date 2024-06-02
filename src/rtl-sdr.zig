//! A wrapper around librtlsdr

const std = @import("std");
const c = @cImport(@cInclude("rtl-sdr.h"));

pub fn getDeviceCount() u32 {
    return c.rtlsdr_get_device_count();
}

pub fn getDeviceName(index: u32) [:0]const u8 {
    return std.mem.span(c.rtlsdr_get_device_name(index));
}

pub const Callback: type = ?*const fn ([*c]u8, u32, ?*anyopaque) callconv(.C) void;

/// Get USB device strings.
pub fn getDeviceUsbStrings(index: u32) !struct {
    manufact: [256:0]u8,
    product: [256:0]u8,
    serial: [256:0]u8,
} {
    var manufact: [256:0]u8 = undefined;
    var product: [256:0]u8 = undefined;
    var serial: [256:0]u8 = undefined;

    const err_no = c.rtlsdr_get_device_usb_strings(
        index,
        &manufact,
        &product,
        &serial,
    );

    if (err_no != 0) {
        return error.UnspecifiedError;
    }

    return .{
        .manufact = manufact,
        .product = product,
        .serial = serial,
    };
}

/// Get device index by USB serial string descriptor.
pub fn getIndexBySerial(serial: [:0]const u8) error{
    NameIsNull,
    NoDevicesFound,
    NoMatchingDeviceFound,
}!u32 {
    const index = c.rtlsdr_get_index_by_serial(serial);

    return switch (index) {
        -1 => error.NameIsNull, // this error will never be reached
        -2 => error.NoDevicesFound,
        -3 => error.NoMatchingDeviceFound,
        else => @intCast(index),
    };
}

const Tuner = enum(c_int) {
    RTLSDR_TUNER_UNKNOWN = 0,
    RTLSDR_TUNER_E4000,
    RTLSDR_TUNER_FC0012,
    RTLSDR_TUNER_FC0013,
    RTLSDR_TUNER_FC2580,
    RTLSDR_TUNER_R820T,
    RTLSDR_TUNER_R828D,
};

pub const Device = struct {
    ptr: *?*c.rtlsdr_dev_t,
    allocator: std.mem.Allocator,

    pub fn init(allocator: std.mem.Allocator, index: u32) !Device {
        const dev = try allocator.create(?*c.rtlsdr_dev_t);
        errdefer allocator.destroy(dev);

        const err_no = c.rtlsdr_open(dev, index);

        if (err_no != 0) {
            return error.NoMatchingDeviceFound;
        }

        return Device{
            .ptr = dev,
            .allocator = allocator,
        };
    }

    pub fn deinit(dev: Device) void {
        _ = c.rtlsdr_close(dev.ptr.*);
        dev.allocator.destroy(dev.ptr);
    }

    /// Set crystal oscillator frequencies used for the RTL2832 and the tuner IC.
    ///
    /// Usually both ICs use the same clock. Changing the clock may make sense if
    /// you are applying an external clock to the tuner or to compensate the
    /// frequency (and samplerate) error caused by the original (cheap) crystal.
    ///
    /// NOTE: Call this function only if you fully understand the implications.
    pub fn setXtalFreq(
        dev: Device,
        /// frequency value used to clock the RTL2832 in Hz
        rtl_freq: u32,
        /// frequency value used to clock the tuner IC in Hz
        tuner_freq: u32,
    ) error{FrequencyError}!void {
        const err_no = c.rtlsdr_set_xtal_freq(dev.ptr.*, rtl_freq, tuner_freq);

        if (err_no != 0) {
            return error.FrequencyError;
        }
    }

    /// Get crystal oscillator frequencies used for the RTL2832 and the tuner IC.
    ///
    /// Usually both ICs use the same clock.
    pub fn getXtalFreq(dev: Device) error{FrequencyError}!struct {
        rtl_freq: u32,
        tuner_freq: u32,
    } {
        var rtl_freq: u32 = undefined;
        var tuner_freq: u32 = undefined;
        const err_no = c.rtlsdr_get_xtal_freq(dev.ptr.*, &rtl_freq, &tuner_freq);

        if (err_no != 0) {
            return error.FrequencyError;
        }

        return .{
            .rtl_freq = rtl_freq,
            .tuner_freq = tuner_freq,
        };
    }

    /// Get USB device strings.
    pub fn getDeviceStrings(dev: Device) !struct {
        manufact: [256:0]u8,
        product: [256:0]u8,
        serial: [256:0]u8,
    } {
        var manufact: [256:0]u8 = undefined;
        var product: [256:0]u8 = undefined;
        var serial: [256:0]u8 = undefined;

        const err_no = c.rtlsdr_get_usb_strings(
            dev.ptr.*,
            &manufact,
            &product,
            &serial,
        );

        if (err_no != 0) {
            return error.UnspecifiedError;
        }

        return .{
            .manufact = manufact,
            .product = product,
            .serial = serial,
        };
    }

    /// Write the device EEPROM
    pub fn writeEEPROM(
        dev: Device,
        /// buffer of data to be written
        data: []u8,
        /// address where the data should be written
        offset: u8,
    ) void {
        _ = dev;
        _ = data;
        _ = offset;

        @compileError("=== DANGER ===");
        // will not implement right now as I have no way of testing this safely
    }

    /// Read the device EEPROM
    pub fn readEEPROM(
        dev: Device,
        /// address where the data should be read from
        offset: u8,
        /// length of the data
        comptime len: u16,
    ) error{
        InvalidDeviceHandle,
        EEPROMSizeExceeded,
        NoEEPROMFound,
    }![len]u8 {
        var data: [len]u8 = undefined;
        const err_no = c.rtlsdr_read_eeprom(dev.ptr.*, &data, offset, len);

        return switch (err_no) {
            -1 => error.InvalidDeviceHandle,
            -2 => error.EEPROMSizeExceeded,
            -3 => error.NoEEPROMFound,
            else => data,
        };
    }

    /// Set the device frequency.
    pub fn setCenterFreq(
        dev: Device,
        /// frequency in Hz
        freq: u32,
    ) error{FrequencyError}!void {
        const err_no = c.rtlsdr_set_center_freq(dev.ptr.*, freq);

        if (err_no != 0) {
            return error.FrequencyError;
        }
    }

    /// Get actual frequency the device is tuned to (Hz).
    pub fn getCenterFreq(dev: Device) error{FrequencyError}!u32 {
        const freq = c.rtlsdr_get_center_freq(dev.ptr.*);

        if (freq == 0) {
            return error.FrequencyError;
        }

        return freq;
    }

    /// Set the frequency correction value for the device.
    pub fn setFreqCorrection(
        dev: Device,
        /// correction value in parts per million (ppm)
        ppm: i32,
    ) error{PPMError}!void {
        const err_no = c.rtlsdr_set_freq_correction(dev.ptr.*, ppm);

        if (err_no != 0) {
            return error.PPMError;
        }
    }

    /// Get actual frequency correction value of the device (ppm).
    pub fn getFreqCorrection(dev: Device) i32 {
        return c.rtlsdr_get_freq_correction(dev.ptr.*);
    }

    /// Get the tuner type.
    pub fn getTunerType(dev: Device) Tuner {
        return @enumFromInt(c.rtlsdr_get_tuner_type(dev.ptr.*));
    }

    /// Get a list of gains supported by the tuner (tenth of a dB).
    ///
    /// NOTE: returned slice must be freed by caller
    pub fn getTunerGains(dev: Device) ![]i32 {
        const gains = try dev.allocator.alloc(i32, try getNumberOfTunerGains(dev));

        const err_no = c.rtlsdr_get_tuner_gains(dev.ptr.*, gains.ptr);

        if (err_no <= 0) {
            return error.TunerGainError;
        }

        return gains;
    }

    fn getNumberOfTunerGains(dev: Device) error{TunerGainError}!usize {
        const err_no = c.rtlsdr_get_tuner_gains(dev.ptr.*, null);

        if (err_no <= 0) {
            return error.TunerGainError;
        }

        return @intCast(err_no);
    }

    /// Set the gain for the device.
    /// Manual gain mode must be enabled for this to work.
    ///
    /// Valid gain values (in tenths of a dB) for the E4000 tuner:
    /// -10, 15, 40, 65, 90, 115, 140, 165, 190,
    /// 215, 240, 290, 340, 420, 430, 450, 470, 490
    ///
    /// Valid gain values may be queried with `getTunerGains()` function.
    pub fn setTunerGain(
        dev: Device,
        /// gain in tenths of a dB, 115 means 11.5 dB
        gain: i32,
    ) error{TunerGainError}!void {
        const err_no = c.rtlsdr_set_tuner_gain(dev.ptr.*, gain);

        if (err_no != 0) {
            return error.TunerGainError;
        }
    }

    /// Set the bandwidth for the device.
    pub fn setTunerBandwidth(
        dev: Device,
        /// bandwidth in Hz. Zero means automatic BW selection
        bw: u32,
    ) error{TunerBandwidthError}!void {
        const err_no = c.rtlsdr_set_tuner_bandwidth(dev.ptr.*, bw);

        if (err_no != 0) {
            return error.TunerBandwidthError;
        }
    }

    /// Get actual gain the device is configured to (tenth of a dB).
    pub fn getTunerGain(dev: Device) error{TunerGainError}!i32 {
        const gain = c.rtlsdr_get_tuner_gain(dev.ptr.*);

        if (gain == 0) {
            return error.TunerGainError;
        }

        return gain;
    }

    /// Set the intermediate frequency gain for the device (tenth of a dB).
    pub fn setTunerIFGain(
        dev: Device,
        /// intermediate frequency gain stage number (1 to 6 for E4000)
        stage: i32,
        /// gain in tenths of a dB, -30 means -3.0 dB
        gain: i32,
    ) error{TunerGainError}!void {
        const err_no = c.rtlsdr_set_tuner_if_gain(dev.ptr.*, stage, gain);

        if (err_no != 0) {
            return error.TunerGainError;
        }
    }

    /// Set the gain mode (automatic/manual) for the device.
    /// Manual gain mode must be enabled for the gain setter function to work.
    pub fn setTunerGainMode(dev: Device, manual: bool) error{TunerGainError}!void {
        const err_no = c.rtlsdr_set_tuner_gain_mode(dev.ptr.*, @intFromBool(manual));

        if (err_no != 0) {
            return error.TunerGainError;
        }
    }

    /// Set the sample rate for the device, also selects the baseband filters
    /// according to the requested sample rate for tuners where this is possible.
    pub fn setSampleRate(
        dev: Device,
        /// the sample rate to be set, possible values are:
        /// * 225001 - 300000 Hz
        /// * 900001 - 3200000 Hz
        ///
        /// sample loss is to be expected for rates > 2400000
        rate: u32,
    ) error{SampleRateError}!void {
        const err_no = c.rtlsdr_set_sample_rate(dev.ptr.*, rate);

        if (err_no != 0) {
            return error.SampleRateError;
        }
    }

    /// Get actual sample rate the device is configured to (Hz).
    pub fn getSampleRate(dev: Device) error{SampleRateError}!u32 {
        const sample_rate = c.rtlsdr_get_sample_rate(dev.ptr.*);

        if (sample_rate == 0) {
            return error.SampleRateError;
        }

        return sample_rate;
    }

    /// Enable test mode that returns an 8 bit counter instead of the samples.
    /// The counter is generated inside the RTL2832.
    pub fn setTestMode(dev: Device, on: bool) error{TestModeError}!void {
        const err_no = c.rtlsdr_set_testmode(dev.ptr.*, @intFromBool(on));

        if (err_no != 0) {
            return error.TestModeError;
        }
    }

    /// Enable or disable the internal digital AGC of the RTL2832.
    pub fn setAGCMode(dev: Device, on: bool) error{AGCError}!void {
        const err_no = c.rtlsdr_set_agc_mode(dev.ptr.*, @intFromBool(on));

        if (err_no != 0) {
            return error.AGCError;
        }
    }

    /// Enable or disable the direct sampling mode. When enabled, the IF mode
    /// of the RTL2832 is activated, and rtlsdr_set_center_freq() will control
    /// the IF-frequency of the DDC, which can be used to tune from 0 to 28.8 MHz
    /// (xtal frequency of the RTL2832).
    pub fn setDirectSampling(dev: Device, on: bool) error{DirectSamplingError}!void {
        const err_no = c.rtlsdr_set_direct_sampling(dev.ptr.*, @intFromBool(on));

        if (err_no != 0) {
            return error.DirectSamplingError;
        }
    }

    /// Get state of the direct sampling mode.
    pub fn getDirectSampling(dev: Device) error{DirectSamplingError}!enum {
        Disabled,
        IADC,
        QADC,
    } {
        const err_no = c.rtlsdr_get_direct_sampling(dev.ptr.*);

        return switch (err_no) {
            0 => .Disabled,
            1 => .IADC,
            2 => .QADC,
            else => error.DirectSamplingError,
        };
    }

    /// Enable or disable offset tuning for zero-IF tuners, which allows to avoid
    /// problems caused by the DC offset of the ADCs and 1/f noise.
    pub fn setOffsetTuning(dev: Device, on: bool) error{OffsetTuningError}!void {
        const err_no = c.rtlsdr_set_offset_tuning(dev.ptr.*, @intFromBool(on));

        if (err_no != 0) {
            return error.OffsetTuningError;
        }
    }

    /// Get state of the offset tuning mode.
    pub fn getOffsetTuning(dev: Device) error{OffsetTuningError}!bool {
        const err_no = c.rtlsdr_get_offset_tuning(dev.ptr.*);

        return switch (err_no) {
            0 => false,
            1 => true,
            else => error.OffsetTuningError,
        };
    }

    // streaming functions

    /// IMPORTANT: Run this functions before reading any samples.
    pub fn resetBuffer(dev: Device) void {
        _ = c.rtlsdr_reset_buffer(dev.ptr.*);
    }

    /// Read samples from the device synchronously.
    pub fn readSync(
        dev: Device,
        /// buffer to read the samples into
        ///
        /// size must be a multiple of 512,
        /// should be a multiple of 16384 (URB size),
        /// suggested default buffer length = (16 * 32 * 512)
        buf: []u8,
    ) !void {
        if (buf.len % 512 != 0) {
            return error.SizeNotMultipleOf512;
        }

        var read: c_int = undefined;
        const err_no = c.rtlsdr_read_sync(dev.ptr.*, buf.ptr, @intCast(buf.len), &read);

        if (err_no < 0) {
            return error.SyncReadError;
        }
    }

    /// Read samples from the device asynchronously. This function will block until
    /// it is being canceled using rtlsdr_cancel_async()
    ///
    /// NOTE: This function is deprecated and is subject for removal.
    // pub fn waitAsync(dev: Device, cb: void, ctx: void) void {
    //     _ = dev;
    //     _ = cb;
    //     _ = ctx;

    //     @compileError("deprecated");
    // }

    /// Read samples from the device asynchronously. This function will block until
    /// it is being canceled using `cancelAsync()`
    pub fn readAsync(
        dev: Device,
        /// callback function to return received samples
        cb: Callback,
        /// user specific context to pass via the callback function
        ctx: ?*anyopaque,
        /// optional buffer count, buf_num * buf_len = overall buffer size
        ///
        /// set to 0 for default buffer count (15)
        buf_num: u32,
        /// optional buffer length, must be multiple of 512,
        /// should be a multiple of 16384 (URB size), set to 0
        /// for default buffer length (16 * 32 * 512)
        buf_len: u32,
    ) !void {
        const err_no = c.rtlsdr_read_async(dev.ptr.*, cb, ctx, buf_num, buf_len);

        if (err_no != 0) {
            return error.AsyncReadError;
        }
    }

    /// Cancel all pending asynchronous operations on the device.
    pub fn cancelAsync(dev: Device) error{AsyncError}!void {
        const err_no = c.rtlsdr_cancel_async(dev.ptr.*);

        if (err_no != 0) {
            return error.AsyncError;
        }
    }

    /// Enable or disable the bias tee on GPIO PIN 0.
    pub fn setBiasTee(dev: Device, on: bool) error{DeviceNotInitialized}!void {
        const err_no = c.rtlsdr_set_bias_tee(dev.ptr.*, @intFromBool(on));

        if (err_no == -1) {
            return error.DeviceNotInitialized;
        }
    }

    /// Enable or disable the bias tee on the given GPIO pin.
    pub fn setBiasTeeGPIO(
        dev: Device,
        /// the gpio pin to configure as a Bias T control
        gpio: i32,
        on: bool,
    ) error{DeviceNotInitialized}!void {
        const err_no = c.rtlsdr_set_bias_tee_gpio(dev.ptr.*, gpio, @intFromBool(on));

        if (err_no == -1) {
            return error.DeviceNotInitialized;
        }
    }
};
