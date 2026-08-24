# Cameras

OptiTrack Prime 13W — **legacy Prime series, not PrimeX**.
The Linux SDK docs only name X-series; legacy support is undocumented
but verified working on this hardware.

| Property    | Value                                      |
|-------------|--------------------------------------------|
| Resolution  | 1280 x 1024, mono, global shutter          |
| Frame rate  | 120 Hz                                     |
| FOV         | 82 deg H x 70 deg V (ultra-wide)           |
| Interface   | Gigabit Ethernet, PoE                      |
| Illumination| 850 nm IR ring, SetIntensity 0-15          |
| Lens        | Manual focus, lockable ring                |

## Inventory

| Serial | Status                                          |
|--------|-------------------------------------------------|
| 33277  | In use                                          |
| 33659  | In use                                          |
| 33661  | In use                                          |
| 33663  | In use                                          |
| 33275  | Set aside — defocused, needs lens adjustment    |
| 33272  | Available, unlabeled                            |
| others | Available, unlabeled (9 total on hand)          |

Refocus 33275 with a variance-of-Laplacian metric rather than judging
sharpness by eye.

## Network

Cameras self-assign link-local addresses and broadcast discovery on
UDP 13013. Host NIC `enp6s0` is link-local only and dedicated to the
camera network; internet comes over Wi-Fi.

    sudo tcpdump -i enp6s0 -n port 13013   # confirm a camera is alive
