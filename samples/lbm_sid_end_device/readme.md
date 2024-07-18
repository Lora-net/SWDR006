button 4 toggles between operating LBM vs sidewalk  
button 1 in sidewalk operation, sends uplink, the same as original sid_end_device  

During LBM operation, the four LEDs on the NRF52840-DK are not driven.

## lorawan build
lorawan credentials are declared in ``example_options.h``  
``west build -b nrf52840dk_nrf52840 -- -DOVERLAY_CONFIG=overlay-lbm.conf``

## NAV3 on sidewalk build
``west build -b nrf52840dk_nrf52840 -- -DOVERLAY_CONFIG=overlay-nav3sid.conf``

## NAV3 on LBM build
lorawan credentials are declared in ``example_options.h``  
``west build -b nrf52840dk_nrf52840 -- -DOVERLAY_CONFIG=overlay-lbm-nav3lbm.conf``


