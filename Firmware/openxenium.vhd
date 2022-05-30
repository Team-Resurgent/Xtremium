-- Design Name: openxenium
-- Module Name: openxenium - Behavioral
-- Project Name: OpenXenium QPI. Open Source Xenius modchip CPLD replacement project
-- Target Devices: XC3S{50,200}A-VQ100
--
-- Revision 0.01 (2019/09/20) - File Created - Ryan Wendland
-- Revision 2022/04/04 - Use SOIC8 16MiB Flash Chips in QPI Mode - Michael Saga
--
-- Additional Comments:
--
-- OpenXenium is free software: you can redistribute it and/or modify
-- it under the terms of the GNU General Public License as published by
-- the Free Software Foundation, either version 3 of the License, or
-- (at your option) any later version.
--
-- This program is distributed in the hope that it will be useful,
-- but WITHOUT ANY WARRANTY; without even the implied warranty of
-- MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
-- GNU General Public License for more details.
--
-- You should have received a copy of the GNU General Public License
-- along with this program. If not, see <https://www.gnu.org/licenses/>.
--
----------------------------------------------------------------------------------
--
--
--**XENIUM BANK SELECTION**
--Bank selection is controlled by the lower nibble of IO address 0x00EF.
--A20,A19,A18 are masked address lines to the flash memory.
--'X' means the address line is not masked by the CPLD for banking purposes.
--
--REGISTER 0x00EF Bank Commands:
--BANK # NAME                 A20|A19|A18 OFFSET   SIZE   ADDRESS           NOTE
--     0 (TSOP)                X |X |X    N/A      N/A    N/A               This prevents the CPLD from driving any pins for TSOP boot.
--     1 XeniumOS (loader)     1 |1 |0    0x180000 256KiB 0x180000-0x1BFFFF This is the default boot bank & contains XeniumOS's Cromwell bootloader.
--     2 XeniumOS              1 |0 |X    0x100000 512KiB 0x100000-0x17FFFF Contains XeniumOS.
--     3 BANK1 (USER BIOS)     0 |0 |0    0x000000 256KiB 0x200000-0x23FFFF
--     4 BANK2 (USER BIOS)     0 |0 |1    0x040000 256KiB 0x240000-0x27FFFF
--     5 BANK3 (USER BIOS)     0 |1 |0    0x080000 256KiB 0x280000-0x2BFFFF
--     6 BANK4 (USER BIOS)     0 |1 |1    0x0C0000 256KiB 0x2C0000-0x2FFFFF
--     7 BANK1 (USER BIOS)     0 |0 |X    0x000000 512KiB 0x200000-0x27FFFF
--     8 BANK2 (USER BIOS)     0 |1 |X    0x080000 512KiB 0x280000-0x2FFFFF
--     9 BANK1 (USER BIOS)     0 |X |X    0x000000 1MiB   0x200000-0x2FFFFF
--    10 RECOVERY              1 |1 |1    0x1C0000 256KiB 0x1C0000-0x1FFFFF See NOTE 1.
--    11 (No address masking)  X |X |X    N/A      16MiB  0x000000-0xFFFFFF This maps all of flash memory into the LPC MMIO window (0xFF000000-0xFFFFFFFF).
--    12 (U2 QPI Chip Select)  X |X |X    N/A      N/A    N/A               This is the default selected chip.
--    13 (U3 QPI Chip Select)  X |X |X    N/A      N/A    N/A
--    14 (U4 QPI Chip Select)  X |X |X    N/A      N/A    N/A
--    15 (U2 QPI Chip Select)  X |X |X    N/A      N/A    N/A
--
--NOTE 1: The RECOVERY bank can also be activated by the physical switch on the Xenium. This sets bank 10 on power-up.
--This bank also contains non-volatile storage of settings & an EEPROM backup in the smaller sectors at the end of the flash memory.
--The memory map is shown below:
--     (1C0000 to 1DFFFF PROTECTED AREA 128kbyte recovery bios)
--     (1E0000 to 1FBFFF Additional XeniumOS Data)
--     (1FC000 to 1FFFFF Contains eeprom backup, XeniumOS settings)
--
--
--**XENIUM CONTROL WRITE/READ REGISTERS**
--Bits marked 'X' either have no function or an unknown function.
--
--**0x00EF WRITE:**
--X,SCK,CS,MOSI,BANK[3:0]
--
--**0x00EF READ:**
--RECOVERY (Active Low),BUSY,MISO2 (Header Pin 4),MISO1 (Header Pin 1),BANK[3:0]
--
--**0x00EE WRITE:**
--X,X,X,X,X,B,G,R (DEFAULT LED ON POWER UP IS RED)
--
--**0x00EE READ:**
--Returns 0xAA for OpenXenium QPI (OpenXenium & Genuine Xenium return 0x55)
--

LIBRARY IEEE;
USE IEEE.STD_LOGIC_1164.ALL;
USE IEEE.STD_LOGIC_UNSIGNED.ALL;
ENTITY openxenium IS
   PORT (
      HEADER_MISO1 : IN STD_LOGIC;
      HEADER_MISO2 : IN STD_LOGIC;
      HEADER_CS : OUT STD_LOGIC;
      HEADER_SCK : OUT STD_LOGIC;
      HEADER_MOSI : OUT STD_LOGIC;

      MOSFET_LED_R : OUT STD_LOGIC;
      MOSFET_LED_G : OUT STD_LOGIC;
      MOSFET_LED_B : OUT STD_LOGIC;
      MOSFET_D0 : OUT STD_LOGIC;
      MOSFET_A20M : OUT STD_LOGIC;

      QPI_IO : INOUT STD_LOGIC_VECTOR (3 DOWNTO 0);
      QPI_CS : OUT STD_LOGIC_VECTOR (2 DOWNTO 0);
      QPI_CLK : OUT STD_LOGIC;

      LPC_LAD : INOUT STD_LOGIC_VECTOR (3 DOWNTO 0);
      LPC_CLK : IN STD_LOGIC;
      LPC_RST : IN STD_LOGIC;

      SWITCH_RECOVER : IN STD_LOGIC -- Recovery is active low and requires an external pull-up resistor to 3.3V.
   );
END openxenium;

ARCHITECTURE Behavioral OF openxenium IS

   TYPE LPC_STATE_MACHINE IS (
   WAIT_START,
   CYCTYPE_DIR,
   ADDRESS,
   WRITE_DATA0,
   WRITE_DATA1,
   READ_DATA0,
   READ_DATA1,
   TAR1,
   TAR2,
   SYNCING,
   SYNC_COMPLETE,
   TAR_EXIT
   );

   TYPE CYC_TYPE IS (
   IO_READ,
   IO_WRITE,
   MEM_READ,
   MEM_WRITE
   );

   SIGNAL LPC_CURRENT_STATE : LPC_STATE_MACHINE := WAIT_START;
   SIGNAL CYCLE_TYPE : CYC_TYPE := IO_READ;
   SIGNAL LPC_CYCLE_ACTIVE : STD_LOGIC;
   SIGNAL LPC_CYCLE_WRITE : STD_LOGIC;
   SIGNAL LPC_CYCLE_MEM : STD_LOGIC;
   SIGNAL LPC_ADDRESS : STD_LOGIC_VECTOR (23 DOWNTO 0); -- LPC address width submitted on the bus is actually 32 bits, but we only need 24.
   SIGNAL LPC_BUFFER : STD_LOGIC_VECTOR (7 DOWNTO 0); -- Generic byte buffer

   --IO READ/WRITE REGISTERS VISIBLE TO THE LPC BUS
   --BITS MARKED 'X' HAVE AN UNKNOWN FUNCTION OR ARE UNUSED.
   --Bit masks are all shown upper nibble first.
   CONSTANT XENIUM_00EC : STD_LOGIC_VECTOR (15 DOWNTO 0) := x"00EC"; -- QPI Bitbang Control Register
   CONSTANT XENIUM_00ED : STD_LOGIC_VECTOR (15 DOWNTO 0) := x"00ED"; -- QPI Bitbang Data Register
   CONSTANT XENIUM_00EE : STD_LOGIC_VECTOR (15 DOWNTO 0) := x"00EE"; -- RGB LED Control Register
   CONSTANT XENIUM_00EF : STD_LOGIC_VECTOR (15 DOWNTO 0) := x"00EF"; -- SPI and Banking Control Register
   CONSTANT REG_00EE_READ : STD_LOGIC_VECTOR (7 DOWNTO 0) := x"AA"; -- OpenXenium QPI (OpenXenium & Genuine Xenium return 0x55)
   SIGNAL REG_00EC : STD_LOGIC_VECTOR (7 DOWNTO 0) := x"00"; -- X,X,X,X,IN,CLK,CS,BBIO
   SIGNAL REG_00ED : STD_LOGIC_VECTOR (7 DOWNTO 0) := x"00"; -- IN[7:4],OUT[3:0]
   SIGNAL REG_00EE_WRITE : STD_LOGIC_VECTOR (7 DOWNTO 0) := x"01"; -- X,X,X,X,X,B,G,R (Red is default LED colour on power-up)
   SIGNAL REG_00EF_WRITE : STD_LOGIC_VECTOR (7 DOWNTO 0) := x"01"; -- X,SCK,CS,MOSI,BANK[3:0]
   SIGNAL REG_00EF_READ : STD_LOGIC_VECTOR (7 DOWNTO 0); -- RECOVERY (Active Low),BUSY,MISO2 (Header Pin 4),MISO1 (Header Pin 1),BANK[3:0]
   SIGNAL SWITCH_RECOVER_LATCH : STD_LOGIC := '0';

   --QPI READ/WRITE REGISTERS FOR FLASH MEMORY
   --QPI Instructions (W25Q128JV-DTR Rev C section 6.1.4)
   CONSTANT QPI_INST_INIT : STD_LOGIC_VECTOR (7 DOWNTO 0) := x"38"; -- Enter QPI Mode (W25Q128JV-DTR Rev C section 8.2.38)
   CONSTANT QPI_INST_READ : STD_LOGIC_VECTOR (7 DOWNTO 0) := x"0B"; -- Fast Read in QPI Mode (W25Q128JV-DTR Rev C section 8.2.7)
   CONSTANT QPI_INST_RSR1 : STD_LOGIC_VECTOR (7 DOWNTO 0) := x"05"; -- Read Status Register-1 (W25Q128JV-DTR Rev C section 8.2.4)
   --Write Protect Features (W25Q128JV-DTR Rev C section 6.2.1 paragraph 2)
   CONSTANT QPI_INST_WR_EN : STD_LOGIC_VECTOR (7 DOWNTO 0) := x"06"; -- Write Enable (W25Q128JV-DTR Rev C section 8.2.1)
   CONSTANT QPI_INST_WR_DI : STD_LOGIC_VECTOR (7 DOWNTO 0) := x"04"; -- Write Disable (W25Q128JV-DTR Rev C section 8.2.3)
   CONSTANT QPI_INST_WRITE : STD_LOGIC_VECTOR (7 DOWNTO 0) := x"02"; -- Page Program (W25Q128JV-DTR Rev C section 8.2.16)
   CONSTANT QPI_INST_ERASE_4K : STD_LOGIC_VECTOR (7 DOWNTO 0) := x"20"; -- 4KiB Sector Erase (W25Q128JV-DTR Rev C section 8.2.18)
   CONSTANT QPI_INST_ERASE_32K : STD_LOGIC_VECTOR (7 DOWNTO 0) := x"52"; -- 32KiB Block Erase (W25Q128JV-DTR Rev C section 8.2.19)
   CONSTANT QPI_INST_ERASE_64K : STD_LOGIC_VECTOR (7 DOWNTO 0) := x"D8"; -- 64KiB Block Erase (W25Q128JV-DTR Rev C section 8.2.20)
   SIGNAL QPI_BUFFER : STD_LOGIC_VECTOR (11 DOWNTO 0) := (OTHERS => '0'); -- 12-bit shift register (4-bit output)
   SIGNAL QPI_CHIP : STD_LOGIC_VECTOR (1 DOWNTO 0) := "00"; -- Chip Selection (when BANK[3:0] = "11XX")
   TYPE QPI_EN_TYPE IS (
   QPI_EN_OFF,
   QPI_EN_OUT,
   QPI_EN_IN,
   QPI_EN_INIT
   );
   SIGNAL QPI_EN : QPI_EN_TYPE := QPI_EN_OFF;
   SIGNAL QPI_EN_INIT_LATCH : STD_LOGIC := '0';

   --SOFTWARE DATA PROTECTION (SDP) COMMAND SEQUENCE
   CONSTANT SDP_TICK_ADDR : STD_LOGIC_VECTOR (15 DOWNTO 0) := x"AAAA";
   CONSTANT SDP_TICK_DATA : STD_LOGIC_VECTOR (7 DOWNTO 0) := x"AA";
   CONSTANT SDP_TOCK_ADDR : STD_LOGIC_VECTOR (15 DOWNTO 0) := x"5555";
   CONSTANT SDP_TOCK_DATA : STD_LOGIC_VECTOR (7 DOWNTO 0) := x"55";
   CONSTANT SDP_ID_ENTRY_DATA : STD_LOGIC_VECTOR (7 DOWNTO 0) := x"90";
   CONSTANT SDP_ID_EXIT_DATA : STD_LOGIC_VECTOR (7 DOWNTO 0) := x"F0";
   CONSTANT SDP_ID_VENDOR : STD_LOGIC_VECTOR (7 DOWNTO 0) := x"01";
   CONSTANT SDP_ID_DEVICE : STD_LOGIC_VECTOR (7 DOWNTO 0) := x"C4";
   CONSTANT SDP_WRITE_DATA : STD_LOGIC_VECTOR (7 DOWNTO 0) := x"A0";
   CONSTANT SDP_ERASE_DATA : STD_LOGIC_VECTOR (7 DOWNTO 0) := x"80";
   CONSTANT SDP_ERASE_SECTOR_DATA : STD_LOGIC_VECTOR (7 DOWNTO 0) := x"30";
   SIGNAL SDP_ID_EN : STD_LOGIC := '0';
   SIGNAL SDP_WR_EN : STD_LOGIC := '0';
   SIGNAL SDP_WR_ERASE : STD_LOGIC := '0';
   SIGNAL SDP_WR_BUSY : STD_LOGIC := '0';
   SIGNAL SDP_WR_BUSY_BIT : STD_LOGIC := '0';
   SIGNAL SDP_WR_BUSY_TOGGLE : STD_LOGIC := '0';
   SIGNAL SDP_COUNT : INTEGER RANGE 0 TO 4 := 0;

   --TSOPBOOT IS SET TO '1' WHEN YOU REQUEST TO BOOT FROM TSOP.
   --THIS SIGNAL PREVENTS THE CPLD FROM DRIVING ANY PINS, EVEN ON RESET.
   --Only a power cycle will reset this signal.
   SIGNAL TSOPBOOT : STD_LOGIC := '0';

   --D0LEVEL is inverted and connected to MOSFET_D0.
   --This signal allows to latch/release D0 (and LFRAME).
   SIGNAL D0LEVEL : STD_LOGIC := '0';

   --A20MLEVEL is inverted and connected to MOSFET_A20M.
   --On reset (A20MLEVEL is '0'), the signal restricts write access to flash memory,
   --as well as access to the rest of flash memory (bottom 15MiB), and the A20M# pin on the CPU is asserted.
   --On read at IO address 0x00EE, the A20M# pin on the CPU is deasserted (A20MLEVEL is '1') until next reset,
   --and all of flash memory is available to the CPU.
   --By default on power-up (A20MLEVEL is '1'), the set signal allows for hotswapping and programming,
   --and simply cannot assert the A20M# pin until next reset.
   SIGNAL A20MLEVEL : STD_LOGIC := '1';

   --GENERIC COUNTER USED TO TRACK ADDRESS AND SYNC COUNTERS.
   SIGNAL COUNT : INTEGER RANGE 0 TO 7;

BEGIN
   --ASSIGN THE IO TO SIGNALS BASED ON REQUIRED BEHAVIOUR
   HEADER_CS <= REG_00EF_WRITE(5);
   HEADER_SCK <= REG_00EF_WRITE(6);
   HEADER_MOSI <= REG_00EF_WRITE(4);

   MOSFET_LED_R <= '0' WHEN TSOPBOOT = '1' ELSE
                   REG_00EE_WRITE(0);
   MOSFET_LED_G <= '0' WHEN TSOPBOOT = '1' ELSE
                   REG_00EE_WRITE(1);
   MOSFET_LED_B <= '0' WHEN TSOPBOOT = '1' ELSE
                   REG_00EE_WRITE(2);

   QPI_IO <= REG_00ED(3 DOWNTO 0) WHEN QPI_EN = QPI_EN_OUT AND REG_00EC(0) = '1' ELSE
             QPI_BUFFER(11 DOWNTO 8) WHEN QPI_EN = QPI_EN_OUT ELSE
             "ZZZ" & QPI_BUFFER(11) WHEN QPI_EN = QPI_EN_INIT ELSE
             "ZZZZ";
   QPI_CS <= "000" WHEN QPI_EN = QPI_EN_INIT ELSE
             "111" WHEN QPI_EN = QPI_EN_OFF ELSE
             "011" WHEN QPI_CHIP = "10" ELSE -- Chip U4 (when BANK[3:0] = x"E")
             "101" WHEN QPI_CHIP = "01" ELSE -- Chip U3 (when BANK[3:0] = x"D")
             "110"; -- Chip U2 (when BANK[3:0] = x"C" OR x"F")
   QPI_CLK <= REG_00EC(2) WHEN REG_00EC(0) = '1' ELSE
              '0' WHEN QPI_EN = QPI_EN_OFF ELSE NOT LPC_CLK; -- SPI Mode 0

   --LAD lines can be either input or output
   --The output values depend on variable states of the LPC transaction
   --Refer to the Intel LPC Specification Rev 1.1
   LPC_LAD <= "0000" WHEN LPC_CURRENT_STATE = SYNC_COMPLETE ELSE
              "0101" WHEN LPC_CURRENT_STATE = SYNCING ELSE
              "1111" WHEN LPC_CURRENT_STATE = TAR2 ELSE
              "1111" WHEN LPC_CURRENT_STATE = TAR_EXIT ELSE
              LPC_BUFFER(3 DOWNTO 0) WHEN LPC_CURRENT_STATE = READ_DATA0 ELSE -- This happens lower nibble first! (Refer to Intel LPC spec)
              LPC_BUFFER(7 DOWNTO 4) WHEN LPC_CURRENT_STATE = READ_DATA1 ELSE
              "ZZZZ";

   LPC_CYCLE_ACTIVE <= '1' WHEN LPC_CURRENT_STATE /= WAIT_START AND LPC_CURRENT_STATE /= CYCTYPE_DIR ELSE
                       '0';
   LPC_CYCLE_WRITE <= '1' WHEN CYCLE_TYPE = IO_WRITE OR CYCLE_TYPE = MEM_WRITE ELSE
                      '0';
   LPC_CYCLE_MEM <= '1' WHEN CYCLE_TYPE = MEM_READ OR CYCLE_TYPE = MEM_WRITE ELSE
                    '0';

   --The D0 pad has the following behaviour:
   ---Held low on boot to ensure the Xbox boots from the LPC bus, then released after a few memory reads.
   ---When soldered and bridged to the LFRAME pad, this will simulate LPC transaction aborts for a v1.6 Xbox mainboard.
   ---Released when booting from TSOP.
   --NOTE: MOSFET_D0 is an output to a MOSFET driver. '0' turns off the MOSFET, leaving the pad floating,
   --and '1' turns on the MOSFET, forcing the pad to ground. This is why D0LEVEL is inverted before mapping it.
   MOSFET_D0 <= '0' WHEN TSOPBOOT = '1' ELSE
                '1' WHEN LPC_CYCLE_MEM = '1' ELSE
                NOT D0LEVEL;

   --The A20M# pad controls the A20M# pin on the CPU, intended to bypass the hidden 1BL ROM in the MCPX X3 southbridge.
   --NOTE: MOSFET_A20M is an output to a MOSFET driver. '0' turns off the MOSFET, leaving the pad floating,
   --and '1' turns on the MOSFET, forcing the pad to ground. A20MLEVEL is inverted and connected to MOSFET_A20M,
   --hence MOSFET is on at reset (not at power-up), asserting the A20M# pin.
   MOSFET_A20M <= '0' WHEN TSOPBOOT = '1' ELSE
                  NOT A20MLEVEL;

   REG_00EF_READ <= SWITCH_RECOVER & SDP_WR_BUSY & HEADER_MISO2 & HEADER_MISO1 & REG_00EF_WRITE(3 DOWNTO 0);

PROCESS (LPC_CLK, LPC_RST) BEGIN
   IF LPC_RST = '0' AND QPI_EN_INIT_LATCH = '1' THEN
      --LPC_RST goes low during boot up or hard reset.
      --Hold D0 low to boot from the LPC bus, only if not booting from TSOP.
      D0LEVEL <= TSOPBOOT;
      --Assert the A20M# pin on the CPU, only if not booting from TSOP.
      A20MLEVEL <= TSOPBOOT;
      SDP_ID_EN <= '0';
      SDP_WR_EN <= '0';
      SDP_WR_ERASE <= '0';
      SDP_WR_BUSY <= '0';
      SDP_COUNT <= 0;
      QPI_EN <= QPI_EN_OFF;
      CYCLE_TYPE <= IO_READ;
      LPC_CURRENT_STATE <= WAIT_START;
   ELSIF rising_edge(LPC_CLK) THEN
      IF QPI_EN_INIT_LATCH = '0' THEN
         QPI_BUFFER <= QPI_BUFFER(10 DOWNTO 0) & '0';
      ELSE
         QPI_BUFFER <= QPI_BUFFER(7 DOWNTO 0) & "0000";
      END IF;
      CASE LPC_CURRENT_STATE IS
      WHEN WAIT_START =>
         CYCLE_TYPE <= IO_READ;
         IF QPI_EN_INIT_LATCH = '0' THEN
            IF QPI_EN = QPI_EN_OFF THEN
               QPI_EN <= QPI_EN_INIT;
               QPI_BUFFER(11 DOWNTO 4) <= QPI_INST_INIT;
               COUNT <= 7;
            ELSE
               IF COUNT = 0 THEN
                  QPI_EN <= QPI_EN_OFF;
                  QPI_EN_INIT_LATCH <= '1';
               ELSE
                  COUNT <= COUNT - 1;
               END IF;
            END IF;
         ELSIF TSOPBOOT = '0' AND LPC_LAD = "0000" THEN
            LPC_CURRENT_STATE <= CYCTYPE_DIR;
         END IF;
      WHEN CYCTYPE_DIR =>
         IF LPC_LAD(3 DOWNTO 1) = "000" THEN
            CYCLE_TYPE <= IO_READ;
            COUNT <= 3;
            LPC_CURRENT_STATE <= ADDRESS;
         ELSIF LPC_LAD(3 DOWNTO 1) = "001" THEN
            CYCLE_TYPE <= IO_WRITE;
            COUNT <= 3;
            LPC_CURRENT_STATE <= ADDRESS;
         ELSIF LPC_LAD(3 DOWNTO 1) = "010" AND REG_00EC(0) = '0' THEN
            CYCLE_TYPE <= MEM_READ;
            COUNT <= 7;
            LPC_CURRENT_STATE <= ADDRESS;
            SDP_COUNT <= 0;
         ELSIF LPC_LAD(3 DOWNTO 1) = "011" AND REG_00EC(0) = '0' THEN
            CYCLE_TYPE <= MEM_WRITE;
            COUNT <= 7;
            LPC_CURRENT_STATE <= ADDRESS;
            IF SDP_WR_EN = '1' THEN
               QPI_BUFFER(11 DOWNTO 4) <= QPI_INST_WR_EN;
               QPI_EN <= QPI_EN_OUT;
            END IF;
         ELSE
            LPC_CURRENT_STATE <= WAIT_START; -- Unsupported, reset state machine.
         END IF;

      --ADDRESS GATHERING
      WHEN ADDRESS =>
         COUNT <= COUNT - 1;
         IF COUNT = 7 THEN
            -- Set recovery bank on power-up if switch is activated.
            IF SWITCH_RECOVER_LATCH = '0' THEN
               SWITCH_RECOVER_LATCH <= '1';
               IF SWITCH_RECOVER = '0' THEN
                  REG_00EF_WRITE(3 DOWNTO 0) <= x"A";
               END IF;
            END IF;
         ELSIF COUNT = 6 THEN
            IF SDP_WR_EN = '1' THEN
               QPI_EN <= QPI_EN_OFF;
            END IF;
            IF SDP_WR_BUSY = '1' THEN
               QPI_BUFFER(11 DOWNTO 4) <= QPI_INST_RSR1;
               QPI_EN <= QPI_EN_OUT;
            ELSIF CYCLE_TYPE = MEM_WRITE THEN
               IF SDP_WR_ERASE = '1' AND REG_00EF_WRITE(3 DOWNTO 0) = x"B" THEN
                  QPI_BUFFER(7 DOWNTO 0) <= QPI_INST_ERASE_4K;
               ELSIF SDP_WR_ERASE = '1' THEN
                  QPI_BUFFER(7 DOWNTO 0) <= QPI_INST_ERASE_64K;
               ELSE
                  QPI_BUFFER(7 DOWNTO 0) <= QPI_INST_WRITE;
               END IF;
            ELSE
               QPI_BUFFER(7 DOWNTO 0) <= QPI_INST_READ;
            END IF;
         ELSIF COUNT = 5 THEN
            IF SDP_WR_EN = '1' OR SDP_WR_BUSY = '1' OR (SDP_ID_EN = '0' AND CYCLE_TYPE = MEM_READ) THEN
               QPI_EN <= QPI_EN_OUT;
            END IF;
            --BANK SELECTION
            IF REG_00EF_WRITE(3 DOWNTO 0) = x"1" OR
               REG_00EF_WRITE(3 DOWNTO 0) = x"2" OR
               REG_00EF_WRITE(3 DOWNTO 0) = x"A" THEN
               QPI_BUFFER(3 DOWNTO 0) <= x"1";
            ELSIF REG_00EF_WRITE(3 DOWNTO 0) = x"B" OR
                  REG_00EF_WRITE(3 DOWNTO 2) = "11" THEN
               -- Do no bank selection (aka. address masking) and map all of flash memory.
               IF A20MLEVEL = '0' THEN
                  -- Wrap-around top 1MiB of flash memory until the A20M# pin on the CPU is deasserted.
                  QPI_BUFFER(3 DOWNTO 0) <= x"F";
               ELSE
                  QPI_BUFFER(3 DOWNTO 0) <= LPC_LAD;
               END IF;
            ELSE
               QPI_BUFFER(3 DOWNTO 0) <= x"2";
            END IF;
            LPC_ADDRESS(23 DOWNTO 20) <= LPC_LAD;
         ELSIF COUNT = 4 THEN
            IF SDP_WR_BUSY = '1' THEN
               QPI_EN <= QPI_EN_IN;
            END IF;
            --BANK SELECTION
            IF REG_00EF_WRITE(3 DOWNTO 0) = x"1" OR
               REG_00EF_WRITE(3 DOWNTO 0) = x"5" THEN
               QPI_BUFFER(3 DOWNTO 0) <= "10" & LPC_LAD(1 DOWNTO 0);
            ELSIF REG_00EF_WRITE(3 DOWNTO 0) = x"2" OR
                  REG_00EF_WRITE(3 DOWNTO 0) = x"7" THEN
               QPI_BUFFER(3 DOWNTO 0) <= '0' & LPC_LAD(2 DOWNTO 0);
            ELSIF REG_00EF_WRITE(3 DOWNTO 0) = x"3" THEN
               QPI_BUFFER(3 DOWNTO 0) <= "00" & LPC_LAD(1 DOWNTO 0);
            ELSIF REG_00EF_WRITE(3 DOWNTO 0) = x"4" THEN
               QPI_BUFFER(3 DOWNTO 0) <= "01" & LPC_LAD(1 DOWNTO 0);
            ELSIF REG_00EF_WRITE(3 DOWNTO 0) = x"6" OR
                  REG_00EF_WRITE(3 DOWNTO 0) = x"A" THEN
               QPI_BUFFER(3 DOWNTO 0) <= "11" & LPC_LAD(1 DOWNTO 0);
            ELSIF REG_00EF_WRITE(3 DOWNTO 0) = x"8" THEN
               QPI_BUFFER(3 DOWNTO 0) <= '1' & LPC_LAD(2 DOWNTO 0);
            ELSE
               QPI_BUFFER(3 DOWNTO 0) <= LPC_LAD;
            END IF;
            LPC_ADDRESS(19 DOWNTO 16) <= LPC_LAD;
         ELSIF COUNT = 3 THEN
            IF LPC_CYCLE_MEM = '1' THEN
               IF SDP_WR_BUSY = '1' THEN
                  LPC_BUFFER(7 DOWNTO 4) <= QPI_IO;
               END IF;
               QPI_BUFFER(3 DOWNTO 0) <= LPC_LAD;
            END IF;
            LPC_ADDRESS(15 DOWNTO 12) <= LPC_LAD;
         ELSIF COUNT = 2 THEN
            IF LPC_CYCLE_MEM = '1' THEN
               IF SDP_WR_BUSY = '1' THEN
                  SDP_WR_BUSY_BIT <= QPI_IO(0);
                  LPC_BUFFER(3 DOWNTO 0) <= QPI_IO;
                  QPI_EN <= QPI_EN_OFF;
               END IF;
               QPI_BUFFER(3 DOWNTO 0) <= LPC_LAD;
            END IF;
            LPC_ADDRESS(11 DOWNTO 8) <= LPC_LAD;
         ELSIF COUNT = 1 THEN
            IF LPC_CYCLE_MEM = '1' THEN
               QPI_BUFFER(3 DOWNTO 0) <= LPC_LAD;
            END IF;
            LPC_ADDRESS(7 DOWNTO 4) <= LPC_LAD;
         ELSIF COUNT = 0 THEN
            IF LPC_CYCLE_MEM = '1' THEN
               QPI_BUFFER(3 DOWNTO 0) <= LPC_LAD;
            END IF;
            LPC_ADDRESS(3 DOWNTO 0) <= LPC_LAD;
            IF CYCLE_TYPE = MEM_READ THEN
               LPC_CURRENT_STATE <= TAR1;
            ELSIF CYCLE_TYPE = MEM_WRITE THEN
               LPC_CURRENT_STATE <= WRITE_DATA0;
            ELSIF LPC_ADDRESS(15 DOWNTO 4) & LPC_LAD(3 DOWNTO 2) = XENIUM_00EC(15 DOWNTO 2) THEN
               IF CYCLE_TYPE = IO_READ THEN
                  LPC_CURRENT_STATE <= TAR1;
               ELSIF CYCLE_TYPE = IO_WRITE THEN
                  LPC_CURRENT_STATE <= WRITE_DATA0;
               END IF;
            ELSE
               LPC_CURRENT_STATE <= WAIT_START; -- Unsupported, reset state machine.
            END IF;
         END IF;

      --MEMORY OR IO WRITES
      WHEN WRITE_DATA0 =>
         LPC_BUFFER(3 DOWNTO 0) <= LPC_LAD; -- This happens lower nibble first! (Refer to Intel LPC spec)
         LPC_CURRENT_STATE <= WRITE_DATA1;
      WHEN WRITE_DATA1 =>
         LPC_BUFFER(7 DOWNTO 4) <= LPC_LAD;
         IF CYCLE_TYPE = MEM_WRITE THEN
            QPI_BUFFER(7 DOWNTO 4) <= LPC_LAD;
            QPI_BUFFER(3 DOWNTO 0) <= LPC_BUFFER(3 DOWNTO 0);
            IF SDP_WR_ERASE = '1' AND LPC_LAD & LPC_BUFFER(3 DOWNTO 0) /= SDP_ERASE_SECTOR_DATA THEN --x"30"
               SDP_WR_ERASE <= '0';
               SDP_WR_EN <= '0';
               QPI_EN <= QPI_EN_OFF;
            END IF;
         END IF;
         LPC_CURRENT_STATE <= TAR1;

      --MEMORY OR IO READS
      WHEN READ_DATA0 =>
         LPC_CURRENT_STATE <= READ_DATA1;
      WHEN READ_DATA1 =>
         LPC_CURRENT_STATE <= TAR_EXIT;

      --TURN BUS AROUND (HOST TO PERIPHERAL)
      WHEN TAR1 =>
         IF CYCLE_TYPE = MEM_WRITE AND SDP_WR_ERASE = '1' THEN
            QPI_EN <= QPI_EN_OFF;
         END IF;
         LPC_CURRENT_STATE <= TAR2;
      WHEN TAR2 =>
         LPC_CURRENT_STATE <= SYNCING;
         COUNT <= 4;

      --SYNCING STAGE
      WHEN SYNCING =>
         COUNT <= COUNT - 1;
         IF COUNT = 4 THEN
            IF CYCLE_TYPE = IO_READ THEN
               CASE LPC_ADDRESS(15 DOWNTO 0) IS
               WHEN XENIUM_00EC =>
                  LPC_BUFFER <= REG_00EC;
               WHEN XENIUM_00ED =>
                  LPC_BUFFER <= REG_00ED;
               WHEN XENIUM_00EE =>
                  LPC_BUFFER <= REG_00EE_READ;
                  -- Deassert the A20M# pin on the CPU.
                  A20MLEVEL <= '1';
               WHEN XENIUM_00EF =>
                  IF REG_00EF_WRITE(3 DOWNTO 0) = x"B" THEN
                     LPC_BUFFER(7 DOWNTO 4) <= REG_00EF_READ(7 DOWNTO 4);
                     LPC_BUFFER(3 DOWNTO 0) <= "11" & QPI_CHIP;
                  ELSE
                     LPC_BUFFER <= REG_00EF_READ;
                  END IF;
               WHEN OTHERS =>
                  LPC_BUFFER <= REG_00EE_READ;
               END CASE;
               LPC_CURRENT_STATE <= SYNC_COMPLETE;
            ELSIF CYCLE_TYPE = IO_WRITE THEN
               CASE LPC_ADDRESS(15 DOWNTO 0) IS
               WHEN XENIUM_00EC =>
                  IF REG_00EC(0) = '1' AND LPC_BUFFER(0) = '0' THEN -- BBIO (off)
                     REG_00EC <= x"00";
                     REG_00ED <= x"00";
                     QPI_EN <= QPI_EN_OFF;
                  ELSIF LPC_BUFFER(0) = '1' THEN -- BBIO
                     IF LPC_BUFFER(1) = '1' THEN -- CS
                        IF LPC_BUFFER(3) = '1' THEN -- IN
                           QPI_EN <= QPI_EN_IN;
                           REG_00ED(7 DOWNTO 4) <= QPI_IO;
                        ELSE
                           QPI_EN <= QPI_EN_OUT;
                        END IF;
                     ELSE
                        QPI_EN <= QPI_EN_OFF;
                        REG_00ED(7 DOWNTO 4) <= QPI_IO;
                     END IF;
                     REG_00EC(3 DOWNTO 0) <= LPC_BUFFER(3 DOWNTO 0);
                  END IF;
               WHEN XENIUM_00ED =>
                  IF REG_00EC(0) = '1' THEN -- BBIO
                     REG_00ED(3 DOWNTO 0) <= LPC_BUFFER(3 DOWNTO 0);
                  END IF;
               WHEN XENIUM_00EE =>
                  REG_00EE_WRITE <= LPC_BUFFER;
               WHEN XENIUM_00EF =>
                  REG_00EF_WRITE(7 DOWNTO 4) <= LPC_BUFFER(7 DOWNTO 4);
                  IF LPC_BUFFER(3 DOWNTO 0) = x"0" THEN
                     -- Bank 0 will disable state machine and release D0 & A20M# to boot from TSOP after reset.
                     TSOPBOOT <= '1';
                  ELSIF LPC_BUFFER(3 DOWNTO 2) = "11" THEN
                     IF LPC_BUFFER(1 DOWNTO 0) /= QPI_CHIP THEN
                        QPI_CHIP <= LPC_BUFFER(1 DOWNTO 0);
                        SDP_WR_BUSY <= '1';
                     END IF;
                  ELSE
                     REG_00EF_WRITE(3 DOWNTO 0) <= LPC_BUFFER(3 DOWNTO 0);
                  END IF;
               WHEN OTHERS =>
               END CASE;
               LPC_CURRENT_STATE <= SYNC_COMPLETE;
            ELSIF SDP_WR_BUSY = '1' THEN
               SDP_WR_BUSY <= SDP_WR_BUSY_BIT;
               IF CYCLE_TYPE = MEM_READ THEN
                  LPC_BUFFER(0) <= SDP_WR_BUSY_TOGGLE;
                  SDP_WR_BUSY_TOGGLE <= NOT SDP_WR_BUSY_TOGGLE;
               END IF;
               LPC_CURRENT_STATE <= SYNC_COMPLETE;
            ELSIF CYCLE_TYPE = MEM_READ THEN
               IF SDP_ID_EN = '1' THEN
                  IF LPC_ADDRESS(1) = '0' THEN
                     LPC_BUFFER <= SDP_ID_VENDOR;
                  ELSE
                     LPC_BUFFER <= SDP_ID_DEVICE;
                  END IF;
                  LPC_CURRENT_STATE <= SYNC_COMPLETE;
               END IF;
            ELSIF CYCLE_TYPE = MEM_WRITE THEN
               IF SDP_WR_EN = '1' THEN
                  SDP_WR_BUSY <= '1';
                  QPI_EN <= QPI_EN_OFF;
               ELSIF SDP_ID_EN = '1' THEN
                  IF LPC_BUFFER = SDP_ID_EXIT_DATA THEN --x"F0"
                     SDP_ID_EN <= '0';
                  END IF;
               ELSE
                  CASE LPC_ADDRESS(15 DOWNTO 0) IS
                  WHEN SDP_TICK_ADDR => --x"AAAA"
                     IF (SDP_COUNT = 0 OR SDP_COUNT = 3) AND LPC_BUFFER = SDP_TICK_DATA THEN --x"AA"
                        SDP_COUNT <= SDP_COUNT + 1;
                     ELSIF SDP_COUNT = 2 AND LPC_BUFFER = SDP_ID_ENTRY_DATA THEN --x"90"
                        SDP_ID_EN <= '1';
                        SDP_COUNT <= 0;
                     ELSIF SDP_COUNT = 2 AND LPC_BUFFER = SDP_WRITE_DATA THEN --x"A0"
                        SDP_WR_EN <= A20MLEVEL;
                        SDP_COUNT <= 0;
                     ELSIF SDP_COUNT = 2 AND LPC_BUFFER = SDP_ERASE_DATA THEN --x"80"
                        SDP_COUNT <= 3;
                     ELSE
                        SDP_COUNT <= 0;
                     END IF;
                  WHEN SDP_TOCK_ADDR => --x"5555"
                     IF (SDP_COUNT = 1 OR SDP_COUNT = 4) AND LPC_BUFFER = SDP_TOCK_DATA THEN --x"55"
                        IF SDP_COUNT = 4 THEN
                           SDP_WR_EN <= A20MLEVEL;
                           SDP_WR_ERASE <= A20MLEVEL;
                        END IF;
                        SDP_COUNT <= SDP_COUNT + 1;
                     ELSE
                        SDP_COUNT <= 0;
                     END IF;
                  WHEN OTHERS =>
                     SDP_COUNT <= 0;
                  END CASE;
               END IF;
               LPC_CURRENT_STATE <= SYNC_COMPLETE;
            END IF;
         ELSIF COUNT = 3 AND CYCLE_TYPE /= MEM_READ THEN
            LPC_CURRENT_STATE <= SYNC_COMPLETE;
         ELSIF COUNT = 2 THEN
            QPI_EN <= QPI_EN_IN;
         ELSIF COUNT = 1 THEN
            LPC_BUFFER(7 DOWNTO 4) <= QPI_IO;
         ELSIF COUNT = 0 THEN
            LPC_BUFFER(3 DOWNTO 0) <= QPI_IO;
            QPI_EN <= QPI_EN_OFF;
            LPC_CURRENT_STATE <= SYNC_COMPLETE;
         END IF;
      WHEN SYNC_COMPLETE =>
         IF SDP_WR_BUSY = '1' THEN
            SDP_WR_EN <= '0';
            SDP_WR_ERASE <= '0';
         END IF;
         IF LPC_CYCLE_WRITE = '0' THEN
            LPC_CURRENT_STATE <= READ_DATA0;
         ELSE
            LPC_CURRENT_STATE <= TAR_EXIT;
         END IF;

      --TURN BUS AROUND (PERIPHERAL TO HOST)
      WHEN TAR_EXIT =>
         --D0 is held low until a few memory reads.
         --This ensures that the Xbox is booting from the LPC bus.
         --Genuine Xenium arbitrarily releases after the 5th read at address 0x74.
         IF LPC_ADDRESS(7 DOWNTO 0) = x"74" THEN
            D0LEVEL <= '1';
         END IF;
         LPC_CURRENT_STATE <= WAIT_START;
      END CASE;
   END IF;
END PROCESS;
END Behavioral;
