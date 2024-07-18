# LR11xx support and LoRa Basics Modem for sdk-sidewalk v2.6.1

## Getting started

Before getting started, make sure you have a proper nRF Connect SDK development environment.
Follow the official
[Installation guide](https://developer.nordicsemi.com/nRF_Connect_SDK/doc/latest/nrf/installation/install_ncs.html).

### Initialization

The first step is to initialize the workspace folder (``my-workspace``) where
the ``example-application`` and all nRF Connect SDK modules will be cloned. Run the following
command:

```shell
# initialize my-workspace for the ncs-example-application (main branch)
west init -m https://github.com/Lora-net/SWDR006 --mr main my-workspace

# update nRF Connect SDK modules
cd my-workspace
west update
```
### configuring project for LR11xx
edit the project's ``.conf`` (default is ``prj.conf``)
```
CONFIG_SIDEWALK_LINK_MASK_LORA=y # run sidewalk in CSS, or..
CONFIG_SIDEWALK_LINK_MASK_FSK=y # run sidewalk in FSK
CONFIG_RADIO_LR11XX=y  # use LR11xx instead of sx126x
CONFIG_RADIO_TCXO=y  # set to n if using crystal on LR11xx instead of TCXO
```
for the building & running  ``template_lbm`` application, see the readme in ``template_lbm`` subdirectory  

### Available example applications:
all example apps are built using ``west build -b <board> -- -DOVERLAY_CONFIG=foobar.conf``.  The app to build is selected by adding ``-- -DOVERLAY_CONFIG=foobar.conf``
 * the full build command, for example: ``west build -b nrf52840dk_nrf52840 -- -DOVERLAY_CONFIG=overlay-nav3sid.conf``
#### apps provided by Nordic:
from the directory ``samples/sid_end_device`` enables LR11xx in ``prj.conf`` -->
* hello
   * ``-DOVERLAY_CONFIG=overlay-hello.conf``
* sensor_monitoring
   * ``-DOVERLAY_CONFIG=overlay-demo.conf``
* dut, aka ``sid_dut``
   * ``-DOVERLAY_CONFIG=overlay-dut.conf``
#### apps added by Semtech:
from the directory ``samples/lbm_sid_end_device`` -->
* lora basics modem (lorawan end device), running in sidewalk environment
   * ``-DOVERLAY_CONFIG=overlay-lbm.conf``
* NAV3 running simultanously with sidewalk (aka NAV3 in bypass mode):
  * ``-DOVERLAY_CONFIG=overlay-nav3sid.conf``
* NAV3 running in lora basics modem:
  * `-DOVERLAY_CONFIG=overlay-nav3lbm.conf``
 * LR11xx firmware update, and almanac erase:
   * build in ``samples/SWTL001``directory


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
