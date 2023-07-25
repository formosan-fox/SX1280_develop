# Change log

## Note on formating
Extracted from http://keepachangelog.com/en/0.3.0/

Date format is YYYY-MM-DD
A new entry is added for each release plus one entry on top to track unreleased change
Classification is:
- Added for new feature
- Changed for change in existing functionnality
- Deprecated for feature to be removed in next release
- Removed for feature removed by the current release
- Fixed for bug fixes
- Security for correction and/or feature improving system/user security

## [v1.6] 2019-02-15

See *v1.6-rc1*.

## [v1.6-rc1] 2019-02-13

### Changed
 - In ranging demo, the Slave does not get its antenna diversity configuration from Master anymore
 - In ranging demo, the channel is always updated even if antenna diversity is both
     - The kit does not attempt ranging with the same channel on both antennas
     - This allows to have a kit in single antenna and another on in both antenna and keep them synchronized

## [v1.5.2] 2018-10-15

### Fixed
 - Fix freeze when entering *Utilities* page (#3)

## [v1.5.1] 2018-10-03

### Changed
 - Update sx1280 driver to v1.3.1

## [v1.5] 2018-09-28

### Changed
 - Modify the aglorithm for ranging correction with addition of per rssi correction and new algorithm depending on SF, BW
 - Antenna naming is changed to reflect the PCB name

### Fixed
 - Fix the ranging slave freezing on reception of wrong address from master

## [v1.4.1.1] 2018-09-05

### Fixed
 - Fix bug in ranging demo freezing at sf5 bw 1600 (#2)

## [v1.4.1] 2018-07-31

### Changed
 - Modify display of version information in utility page to indicate version number and date

## [v1.4] 2018-07-31

### Added
 - Compile time configuration mechanism to select activation of GPS or PROXIMITY
 - LNA and AGC control of the sx1280 depending on demo running
 - PER demo with Frequency Hopping

### Changed
 - Modify software version displaying to include the version number and version date
 - SX1280 driver on v1.3
 - Ranging demo use driver in polling mode instead of interrupt mode

### Removed
 - PER demo without Frequency Hopping

### Fixed
 - Fix ping pong demo with using a timeouted Rx window instead of rx continuous

## [v1.3] 2017-08-17

### Added
 - Add a factory reset feature that reset the eeprom and reboot the board
 - The factory reset functionnality is callable by pressing unser button during boot time

### Fixed
 - The factory reset existing button now calls the new factory reset feature, and not only the eeprom factory reset

## [v1.2] 2017-08-17

### Changed
 - SX1280 driver on v1.0.2
 - Use polling mode in PER, PING-PONG and some test demo applications

## [v1.1] 2017-06-23

### Changed
- SX1280 driver on v1.0.1
- The project do not compile unused display drivers anymore (but the display drivers are still in the project)
- A post-build command build also a bin file for the project

### Fixed
- Correct bug with ranging slave hang after some ranging exchanges

## [v1.0] 2017-05-30

### Added
- First commit synchronized with Mbed

### Changed
- Modification of readme to indicate the install procedure

### Deprecated

### Removed

### Fixed
- Fix bug in ranging demo with low distance estimation

### Security

