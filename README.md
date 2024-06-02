# zig-rtl-sdr

This is a Zig wrapper around [rtl-sdr](https://github.com/osmocom/rtl-sdr)

## Example

### `build.zig`

```zig
...

const rtlsdr_package = b.dependency("rtl-sdr", .{
    .target = target,
    .optimize = optimize,
});

const rtlsdr = rtlsdr_package.module("rtl-sdr");
exe.root_module.addImport("rtl-sdr", rtlsdr);

...
```

### `main.zig`

```zig
const std = @import("std");
const rtl = @import("rtl-sdr");

pub fn main() !void {
    // IO
    const stdout_file = std.io.getStdOut().writer();
    var bw = std.io.bufferedWriter(stdout_file);
    const stdout = bw.writer();

    // GPA
    var gpa = std.heap.GeneralPurposeAllocator(.{}){};
    const allocator = gpa.allocator();
    defer _ = gpa.deinit();

    // initialize the rtl device at index 0
    const dev = try rtl.Device.init(allocator, 0);
    defer dev.deinit(); // don't forget to deinit

    // get device info
    const info = try dev.getDeviceStrings();
    try stdout.print("Manufacturer: {s}\nProduct: {s}\nSerial: {s}\n", .{
        info.manufact,
        info.product,
        info.serial,
    });
    try bw.flush();

    // setting up device params
    try dev.setSampleRate(2.4e6);
    try dev.setCenterFreq(92.5e6);
    try dev.setTunerGainMode(false);
    dev.resetBuffer(); // important to run before reading any samples

    // creating a buffer to store the samples
    var buffer: [512]u8 = undefined;

    // hack to get slices working
    var z: usize = 0;
    _ = &z;

    // reading 512 samples in sync mode
    const read = try dev.readSync(buffer[z..]);

    try stdout.print("Buffer[{d}]: {any}\n", .{read, buffer});
    try bw.flush();
}
```
