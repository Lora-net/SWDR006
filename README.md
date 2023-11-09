# SWDR006 - LR11xx Sidewalk Driver for nRF52840

This repository contains the software driver that enables the [LR11xx family](https://www.semtech.com/products/wireless-rf/lora-edge) of silicon to support the Sidewalk protocol when paired with the [Nordic nRF52840 MCU](https://www.nordicsemi.com/Products/Development-hardware/nrf52840-dk) and nRF Connect SDK. The driver is offered in binary form, as a static library which implements the "Platform Abstraction Layer" interfaces necessary to support LoRa or FSK connectivity. The static library contains within it a complete implementation of Semtech's SWDR001 (LR11xx Driver), which can be used to access other features of the LR11xx silicon, such as WIFI and GNSS scanning and ranging.

# Repository Organization
The repository contains the following components:  
* ``libsid_pal_radio_lr11xx_impl.a``: The static library containing Platform Abstraction Layer and Sidewlk-specific driver functions.  In addition, a 2nd copy of the library is provided with zephyr [``LOG_RUNTIME_FILTERING``](https://docs.zephyrproject.org/latest/kconfig.html#CONFIG_LOG_RUNTIME_FILTERING) enabled.   If the zephyr shell (aka CLI) is enabled, the static library with ``LOG_RUNTIME_FILTERING`` will be linked in, otherwise without zephyr shell, the other static library is linked in.    Additionally, a copy of this static library is provided with ``CONFIG_LOG=n`` for release builds.
* ``libsmtc_lr11xx_swdr001_impl.a`` containing a subset of [SWDR001](https://github.com/Lora-net/SWDR001) driver files from semtech.
* ``nRF52840_LR11xx_driver_v010000.diff``: file which is used to patch the Nordic NCS codebase to support linking of the provided Libsid_pal_radio_lr11xx_impl.a static library.
* ``wifi_lr11xx`` and ``gnss_lr11xx``: Examples demonstrating WIFI (or GNSS) scan and uplink. These examples are available only after successful application of the ``nRF52840_LR11xx_driver_v010000.diff'``. 
* GNSS and WIFI interface declarations: These C header file(s) contain function prototypes necessary to invoke the GNSS and WIFI scan driver functions. 
* AWS sub-directory: This sub-directory contains the AWS lambda implementation that enables re-assembly of the packet stream in AWS IOT Core.

# Supported Semtech Silicon
 This repository supports the LR11xx family of Semtech silicon. This includes LR11110, LR1120 and LR1121. SWDR006 is compatible with LR11xx microcode versions 0x0308 (LR1110), and 0x0102 for LR1120 and LR1121, This repository is built with [v2.3.0 of SWDR001](https://github.com/Lora-net/SWDR001). There is a high-degree of compatibility between LR1110, LR1120 and LR1121 which minimizes product integration effort and application software development time. Where functionality is common between the various transceivers, a driver compiled for LR1110 is fully capable of driving LR1120 and LR1121. This is why in the instructions below, build flags often reference ''LR1110'' exclusively, even if the instructions pertain to LR11xx in general. 

# How to Add LR11xx to Nordic's nRF Connect SDK Sidewalk Extension

 1. If you havent ever used sidewalk before, first run ``template_subghz`` with sx1262 shield.
 2. Verify sidewalk revision in the west manifest, as shown in the [nordic instructions](https://nrfconnect.github.io/sdk-sidewalk/setting_up_sidewalk_environment/setting_up_sdk.html) for setting up sidewalk SDK.
```
$ west list | grep sidewalk
sidewalk     sidewalk                     9a2e912e703625f11b73889678bfdf1792f3c266 https://github.com/nrfconnect/sdk-sidewalk
```
this patch is only for this revision

 3. to add LR1110:
     * with your working directory in cloned nordic repo, at top level directory, i.e. ``~/ncs/<version>/sidewalk``:
     * ``patch -p1 < ../nRF52840_LR11xx_driver_v010000.diff``
     *  parent directory path `..` assumes you put the diff file there, otherwise you can specify the full path to its location.
     * copy radio driver libraries ``lib*.a`` into sidewalk project to ``~/ncs/<version>/sidewalk/lib/lora_fsk/``  There are two libraries provided, one with ``LOG_RUNTIME_FILTERING`` enabled and other without.
 4. if you're interested in WiFi scan, see the readme in ``sidewalk/samples/wifi_lr1110``
 5. or, if you're interested in GNSS scan, see the readme in ``sidewalk/samples/gnss_lr1110``
 6. or, if you're only interested in sidewalk connectivity with LR11xx, you can use ``template_subghz`` as provided by nordic, built with ``west build -b nrf52840dk_nrf52840 -- -DRADIO=LR1110``

# Operational Considerations
While conducting validation test and field trials of this repository, several different flavors error traces were observed. Root-cause analysis of these errors has determined that are not indicative of any observable anomalous behavior. The specific error text and an accompanying explanation are included below.
 ```
 <err> lr1110_hal: rdwr start wait_on_busy 0113
 <err> lr1110_radio: disable_irq fail
 <err> lr1110_radio: enable_irq fail
 <wrn> lr110_radio: irq 30
 ```
On rare occasion, the sidewalk MAC layer might attempt an operation on the LR11xx chip while its in sleep mode, then the ``wait_on_busy`` will be seen in the logs. This doesn't impact any correct functioning of sidewalk, and is strictly for diagnostics.

The ``irq 30`` message indicates that an interrupt bit was set by the transceiver chip, but was not serviced by the radio irq handler function.  This doesn't impact any correct functioning of sidewalk, and is strictly for diagnostics.

# Support
Support for Sidewalk on Nordic nRF52840 is described here:  https://devzone.nordicsemi.com.

Support for LR11xx silicon is described here: https://semtech.my.site.com/ldp/ldp_support. 

# GNSS Performance Evaluation Notice

The included GNSS example source code is provided solely to demonstrate the GNSS scan functionality under ideal conditions. 
The source code and GNSS scan results are not representative of the optimal configuration or performance characteristics 
of the silicon. The LR11xx product family is flexible and can be emobodied and configured in a multitude of ways to realize 
various trade-offs regarding
performance, battery life, PCB size, cost, etc. The GNSS example included in this release and the corresponding evaluation 
kits are designed & configured by the included source code in a default capacity which is sufficient to demonstrate functional
GNSS scan capability only. Care must be taken if/when attempting to assess performance characterstics of GNSS scan functionality 
and we strongly encourage those conducting such analysis to contact Semtech via the provided support channels so that we can 
ensure appropriate configuration settings are employed for a given performance evaluation use-case.
