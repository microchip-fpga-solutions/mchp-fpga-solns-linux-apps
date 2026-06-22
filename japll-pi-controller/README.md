# PolarFire SoC JAPLL PI Controller Application

This application adjusts the frequency of the TSU (Time Stamp Unit) clock on
PolarFire SoC kits as per the master clock, using PPB offsets from incoming PTP
packets.

## Supported Kits

| Kit | Clock Source | Config Value |
|-----|-------------|--------------|
| PolarFire SoC Video Kit | Transceiver JAPLL | `xcvr` |
| PolarFire SoC Motor Control Kit | CCC (Clock Conditioning Circuitry) | `ccc` |

## Pre-requisites

The japll-pi-controller application relies on the linuxptp application v3.1.1.
The linuxptp application should either be part of the downloaded WIC image, or may be
downloaded and compiled directly on the target. To confirm whether linuxptp is installed,
following command can be tried:

```text
ptp4l -v
```

## Configuration

To modify the configuration, open japll-pi.cfg:

```text
root@mpfs:/opt/microchip/japll-pi-controller# vim /opt/microchip/japll-pi-controller/configs/japll-pi.cfg
```

### Selecting the Clock Source (Kit Selection)

The `clock_source` parameter in `configs/japll-pi.cfg` determines which clock
hardware path is used. Set it according to your target kit:

```text
clock_source            ccc            # For Motor Control Kit (CCC clock)
clock_source            xcvr           # For Video Kit (Transceiver JAPLL)
```

Example config files are provided for each kit:
- `configs/japll-pi-video-kit.cfg.example` - Video Kit configuration
- `configs/japll-pi-motor-ctrl-kit.cfg.example` - Motor Control Kit configuration

To use an example config, copy it over the active config file:

```text
cp configs/japll-pi-motor-ctrl-kit.cfg.example configs/japll-pi.cfg
```

### Tuning Parameters

```text
     clock_source       ccc/xcvr            -> select clock source for the target kit
     k_proportional     0.9                 -> proportional constant
     k_integral         0.9                 -> integral constant
     delta_time         0.125               -> time frame between two packets
     set_point          0                   -> ideal value for tuning ppb/freq
     pi_enable          1                   -> enable PI controller
     japll_wr_enable    1                   -> enable write operation for tuned PPB value to JAPLL registers
     board_ref_clk_freq 125.00              -> External reference clock from which the TSU clock will be derived
     eth_interface      eth1                -> ethernet driver interface for input to ptp4l command
     ptp4l_config       ./configs/gPTP.cfg  -> filename along with path for input to ptp4l command
```

Note: The delta_time parameter is inversely proportional to the incoming packets per second.

## Running the Application

The following steps need to be performed to launch the application:

```text
root@mpfs:~# cd /opt/microchip/japll-pi-controller

root@mpfs:/opt/microchip/japll-pi-controller# ./japll-pi
```

## Terminating the Application

Once launched, the PI Controller application will run indefinitely, unless it is
interrupted using a ctrl+c, in which case, it will perform a graceful exit.

## Re-build

If needed, the application can be rebuilt by issuing `make clean` and then
`make` command from the same directory.

```text
root@mpfs:/opt/microchip/japll-pi-controller# make clean && make
```
