-- Design Name: openxenium
-- Module Name: openxenium - Behavioral
-- Project Name: OpenXenium QPI. Open Source Xenius modchip CPLD replacement project
-- Target Devices: XC3S{50,200}A-VQ100
--
-- Revision 2019/09/20 - File Created - Ryan Wendland
-- Revision 2022/04/04 - Use SOIC8 16MiB SPI flash chips in QPI mode - Michael Saga
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
--    10 RECOVER               1 |1 |1    0x1C0000 256KiB 0x1C0000-0x1FFFFF See NOTE 1.
--    11 (QPI chip select)     X |X |X    N/A      16MiB  0x000000-0xFFFFFF See NOTE 2.
--    12 (QPI chip U2/CS0)     X |X |X    N/A      N/A    N/A               This is the default selected chip.
--    13 (QPI chip U3/CS1)     X |X |X    N/A      N/A    N/A
--    14 (QPI chip U4/CS2)     X |X |X    N/A      N/A    N/A
--    15 (QPI chip CS3)        X |X |X    N/A      N/A    N/A               See NOTE 3.
--
--NOTE 1: The RECOVER bank (10/0x0A) can be set on power-up by the physical switch on the Xenium.
--This bank also contains non-volatile storage of settings & an EEPROM backup in the smaller sectors at the end of the flash memory.
--The memory map is shown below:
--     (0x1C0000 to 0x1DFFFF PROTECTED AREA 128KiB recovery bootloader)
--     (0x1E0000 to 0x1FBFFF Additional XeniumOS Data)
--     (0x1FC000 to 0x1FFFFF Contains EEPROM backup & XeniumOS settings)
--
--NOTE 2: The QPI chip select bank (11/0x0B) when set, returns the currently selected chip when read back.
--This bank also maps all of flash memory into the LPC MMIO window (0xFF000000-0xFFFFFFFF).
--
--NOTE 3: This is an auxiliary QPI chip select & can be used for an external chip or a QPI bus peripheral, useful in QPI bitbang mode.
--
--
--**XENIUM CONTROL WRITE/READ REGISTERS**
--Bits marked 'X' either have no function or an unknown function.
--
--**0x00EF WRITE:**
--X,SCK,CS,MOSI,BANK[3:0]
--
--**0x00EF READ:**
--RECOVER (Active Low),BUSY,MISO2 (Header Pin 4),MISO1 (Header Pin 1),BANK[3:0]
--
--**0x00EE WRITE:**
--X,X,X,X,X,B,G,R (DEFAULT LED ON POWER UP IS RED)
--
--**0x00EE READ:**
--Genuine Xenium return 0x55
--

LIBRARY IEEE;
USE IEEE.STD_LOGIC_1164.ALL;
USE IEEE.NUMERIC_STD.ALL;

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
      QPI_CS : OUT STD_LOGIC_VECTOR (3 DOWNTO 0);
      QPI_CLK : OUT STD_LOGIC;

      LPC_LAD : INOUT STD_LOGIC_VECTOR (3 DOWNTO 0);
      LPC_CLK : IN STD_LOGIC;
      LPC_RST : IN STD_LOGIC;
      LPC_LFRAME : IN STD_LOGIC;

      FTDI_RXD : OUT STD_LOGIC;

      SWITCH_RECOVER : IN STD_LOGIC -- RECOVER pin is active low and requires an external pull-up resistor to 3.3V.
   );
END openxenium;

ARCHITECTURE Behavioral OF openxenium IS
   COMPONENT dcm33to96 IS
      PORT (
         CLKIN_IN : IN STD_LOGIC;
         CLKFX_OUT : OUT STD_LOGIC;
         CLKIN_IBUFG_OUT : OUT STD_LOGIC;
         CLK0_OUT : OUT STD_LOGIC
      );
   END COMPONENT;
   SIGNAL CLK33 : STD_LOGIC; -- LPC clock @ 33.3MHz
   SIGNAL CLK96 : STD_LOGIC; -- dcm33to96 @ 96MHz
   SIGNAL CTR33 : UNSIGNED (63 DOWNTO 0) := (OTHERS => '0'); -- Monotonic 64-bit TSC of LPC clock @ 33.3MHz
   SIGNAL CTR96 : UNSIGNED (63 DOWNTO 0) := (OTHERS => '0'); -- Monotonic 64-bit TSC of dcm33to96 @ 96MHz
   SIGNAL CTR : UNSIGNED (63 DOWNTO 0) := (OTHERS => '0');
   TYPE U64_U8_MAP_TYPE IS ARRAY (0 TO 7) OF UNSIGNED (7 DOWNTO 0);
   CONSTANT CTR_U8LE_MAP : U64_U8_MAP_TYPE := (
   0 => CTR(7 DOWNTO 0),
   1 => CTR(15 DOWNTO 8),
   2 => CTR(23 DOWNTO 16),
   3 => CTR(31 DOWNTO 24),
   4 => CTR(39 DOWNTO 32),
   5 => CTR(47 DOWNTO 40),
   6 => CTR(55 DOWNTO 48),
   7 => CTR(63 DOWNTO 56)
   );

   COMPONENT uart_tx IS
      PORT (
         TX_CLK : IN STD_LOGIC;
         TX_BYTE : IN STD_LOGIC_VECTOR (7 DOWNTO 0);
         TX_START : IN STD_LOGIC;
         TX_IDLE : OUT STD_LOGIC;
         TX : OUT STD_LOGIC
      );
   END COMPONENT;
   SIGNAL TX_CLK : STD_LOGIC;
   SIGNAL TX_BYTE : STD_LOGIC_VECTOR (7 DOWNTO 0);
   SIGNAL TX_START : STD_LOGIC := '0';
   SIGNAL TX_IDLE : STD_LOGIC;

   TYPE BUS_ARB_TYPE IS (
   PENDING,
   DETECT,
   MASTER,
   NOT_FOUND
   );
   SIGNAL LPC_SIO_UART_ARB : BUS_ARB_TYPE := PENDING;
   TYPE LPC_HAS_LFRAME_TYPE IS (
   PENDING,
   YES,
   NO
   );
   SIGNAL LPC_HAS_LFRAME : LPC_HAS_LFRAME_TYPE := PENDING;
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
   SIGNAL LPC_CURRENT_STATE : LPC_STATE_MACHINE := WAIT_START;
   TYPE CYC_TYPE IS (
   IO_READ,
   IO_WRITE,
   MEM_READ,
   MEM_WRITE
   );
   SIGNAL CYCLE_TYPE : CYC_TYPE := IO_READ;
   SIGNAL LPC_CYCLE_ACTIVE : STD_LOGIC;
   SIGNAL LPC_CYCLE_WRITE : STD_LOGIC;
   SIGNAL LPC_CYCLE_MEM : STD_LOGIC;
   SIGNAL LPC_CYCLE_UART : STD_LOGIC;
   SIGNAL LPC_ADDRESS : STD_LOGIC_VECTOR (23 DOWNTO 0); -- LPC address width submitted on the bus is actually 32 bits, but we only need 24.
   SIGNAL LPC_BUFFER : STD_LOGIC_VECTOR (7 DOWNTO 0); -- Generic byte buffer

   --IO READ/WRITE REGISTERS VISIBLE TO THE LPC BUS
   --BITS MARKED 'X' HAVE AN UNKNOWN FUNCTION OR ARE UNUSED.
   --Bit masks are all shown upper nibble first.
   CONSTANT XENIUM_00EC : STD_LOGIC_VECTOR (15 DOWNTO 0) := x"00EC"; -- QPI Bitbang Control Register (Available when RECOVER pin remains active low)
   CONSTANT XENIUM_00ED : STD_LOGIC_VECTOR (15 DOWNTO 0) := x"00ED"; -- QPI Bitbang Data Register (Available when RECOVER pin remains active low)
   CONSTANT XENIUM_00EE : STD_LOGIC_VECTOR (15 DOWNTO 0) := x"00EE"; -- RGB LED Control Register
   CONSTANT XENIUM_00EF : STD_LOGIC_VECTOR (15 DOWNTO 0) := x"00EF"; -- SPI Bitbang and Banking Control Register
   CONSTANT XENIUM_03F8 : STD_LOGIC_VECTOR (15 DOWNTO 0) := x"03F8"; -- UART to FTDI @ 3 megabaud via 96MHz DCM
   CONSTANT REG_00EE_READ : STD_LOGIC_VECTOR (7 DOWNTO 0) := x"55"; -- Genuine Xenium
   SIGNAL REG_00EC : STD_LOGIC_VECTOR (7 DOWNTO 0) := x"00"; -- X,X,X,X,IN,CLK,CS,BBIO
   SIGNAL REG_00ED : STD_LOGIC_VECTOR (7 DOWNTO 0) := x"00"; -- IN[7:4],OUT[3:0]
   SIGNAL REG_00EE_WRITE : STD_LOGIC_VECTOR (7 DOWNTO 0) := x"01"; -- X,X,X,X,X,B,G,R (Red is default LED colour on power-up)
   SIGNAL REG_00EF_WRITE : STD_LOGIC_VECTOR (7 DOWNTO 0) := x"01"; -- X,SCK,CS,MOSI,BANK[3:0]
   SIGNAL REG_00EF_READ : STD_LOGIC_VECTOR (7 DOWNTO 0); -- RECOVER (Active Low),BUSY,MISO2 (Header Pin 4),MISO1 (Header Pin 1),BANK[3:0]
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
   SIGNAL QPI_CHIP : STD_LOGIC_VECTOR (1 DOWNTO 0) := "00"; -- Chip Selector (when BANK[3:0] = "11XX")
   TYPE QPI_EN_TYPE IS (
   QPI_EN_OFF,
   QPI_EN_OUT,
   QPI_EN_IN,
   QPI_EN_INIT
   );
   SIGNAL QPI_EN : QPI_EN_TYPE := QPI_EN_OFF;
   SIGNAL QPI_EN_INIT_AGAIN : STD_LOGIC := '0';
   SIGNAL QPI_EN_INIT_LATCH : STD_LOGIC := '0';
   SIGNAL QPI_BUSY : STD_LOGIC := '0';
   SIGNAL QPI_BUSY_TOGGLE : STD_LOGIC := '0';
   SIGNAL QPI_LPC_MUTEX : STD_LOGIC; -- If set, another FSM other than the LPC FSM is taking over the QPI bus.

   --SOFTWARE DATA PROTECTION (SDP) COMMAND SEQUENCE
   CONSTANT SDP_TICK_ADDR : STD_LOGIC_VECTOR (11 DOWNTO 0) := x"AAA";
   CONSTANT SDP_TICK_DATA : STD_LOGIC_VECTOR (7 DOWNTO 0) := x"AA";
   CONSTANT SDP_TOCK_ADDR : STD_LOGIC_VECTOR (11 DOWNTO 0) := x"555";
   CONSTANT SDP_TOCK_DATA : STD_LOGIC_VECTOR (7 DOWNTO 0) := x"55";
   CONSTANT SDP_RESET_DATA : STD_LOGIC_VECTOR (7 DOWNTO 0) := x"F0";
   CONSTANT SDP_ID_DATA : STD_LOGIC_VECTOR (7 DOWNTO 0) := x"90";
   CONSTANT SDP_ID_VENDOR : STD_LOGIC_VECTOR (7 DOWNTO 0) := x"01";
   CONSTANT SDP_ID_DEVICE : STD_LOGIC_VECTOR (7 DOWNTO 0) := x"C4";
   CONSTANT SDP_CFI_DATA : STD_LOGIC_VECTOR (7 DOWNTO 0) := x"98";
   CONSTANT SDP_TSC_DATA : STD_LOGIC_VECTOR (7 DOWNTO 0) := x"99";
   CONSTANT SDP_WRITE_DATA : STD_LOGIC_VECTOR (7 DOWNTO 0) := x"A0";
   CONSTANT SDP_ERASE_DATA : STD_LOGIC_VECTOR (7 DOWNTO 0) := x"80";
   CONSTANT SDP_ERASE_SECTOR_DATA : STD_LOGIC_VECTOR (7 DOWNTO 0) := x"30";
   TYPE SDP_CFI_ROM_TYPE IS ARRAY (16#10# TO 16#7F#) OF STD_LOGIC_VECTOR (7 DOWNTO 0);
   CONSTANT SDP_CFI_ROM : SDP_CFI_ROM_TYPE := (
   -- Common Flash Interface (S29AL016J Rev Q section 9)
   16#10# => x"51", -- 20h
   16#11# => x"52", -- 22h
   16#12# => x"59", -- 24h
   16#13# => x"02", -- 26h
   16#14# => x"00", -- 28h
   16#15# => x"40", -- 2Ah
   16#16# => x"00", -- 2Ch
   16#17# => x"00", -- 2Eh
   16#18# => x"00", -- 30h
   16#19# => x"00", -- 32h
   16#1A# => x"00", -- 34h
   16#1B# => x"27", -- 36h
   16#1C# => x"36", -- 38h
   16#1D# => x"00", -- 3Ah
   16#1E# => x"00", -- 3Ch
   16#1F# => x"03", -- 3Eh
   16#20# => x"00", -- 40h
   16#21# => x"09", -- 42h
   16#22# => x"00", -- 44h
   16#23# => x"05", -- 46h
   16#24# => x"00", -- 48h
   16#25# => x"04", -- 4Ah
   16#26# => x"00", -- 4Ch
   16#27# => x"15", -- 4Eh
   16#28# => x"02", -- 50h
   16#29# => x"00", -- 52h
   16#2A# => x"00", -- 54h
   16#2B# => x"00", -- 56h
   16#2C# => x"04", -- 58h
   16#2D# => x"00", -- 5Ah
   16#2E# => x"00", -- 5Ch
   16#2F# => x"40", -- 5Eh
   16#30# => x"00", -- 60h
   16#31# => x"01", -- 62h
   16#32# => x"00", -- 64h
   16#33# => x"20", -- 66h
   16#34# => x"00", -- 68h
   16#35# => x"00", -- 6Ah
   16#36# => x"00", -- 6Ch
   16#37# => x"80", -- 6Eh
   16#38# => x"00", -- 70h
   16#39# => x"1E", -- 72h
   16#3A# => x"00", -- 74h
   16#3B# => x"00", -- 76h
   16#3C# => x"01", -- 78h
   16#3D# => x"00", -- 7Ah
   16#3E# => x"00", -- 7Ch
   16#3F# => x"00", -- 7Eh
   16#40# => x"50", -- 80h
   16#41# => x"52", -- 82h
   16#42# => x"49", -- 84h
   16#43# => x"31", -- 86h
   16#44# => x"33", -- 88h
   16#45# => x"0C", -- 8Ah
   16#46# => x"02", -- 8Ch
   16#47# => x"01", -- 8Eh
   16#48# => x"01", -- 90h
   16#49# => x"04", -- 92h
   16#4A# => x"00", -- 94h
   16#4B# => x"00", -- 96h
   16#4C# => x"00", -- 98h
   16#4D# => x"00", -- 9Ah
   16#4E# => x"00", -- 9Ch
   16#4F# => x"03", -- 9Eh
   16#50# => x"00", -- A0h
   16#51# => x"00", -- A2h
   16#52# => x"00", -- A4h
   16#53# => x"00", -- A6h
   16#54# => x"00", -- A8h
   16#55# => x"00", -- AAh
   16#56# => x"00", -- ACh
   16#57# => x"00", -- AEh
   16#58# => x"00", -- B0h
   16#59# => x"00", -- B2h
   16#5A# => x"00", -- B4h
   16#5B# => x"00", -- B6h
   16#5C# => x"00", -- B8h
   16#5D# => x"00", -- BAh
   16#5E# => x"00", -- BCh
   16#5F# => x"06", -- BEh
   16#60# => x"00", -- C0h
   16#61# => x"09", -- C2h
   16#62# => x"00", -- C4h
   16#63# => x"05", -- C6h
   16#64# => x"00", -- C8h
   16#65# => x"04", -- CAh
   16#66# => x"00", -- CCh
   16#67# => x"15", -- CEh
   16#68# => x"02", -- D0h
   16#69# => x"00", -- D2h
   16#6A# => x"00", -- D4h
   16#6B# => x"00", -- D6h
   16#6C# => x"04", -- D8h
   16#6D# => x"00", -- DAh
   16#6E# => x"00", -- DCh
   16#6F# => x"40", -- DEh
   16#70# => x"00", -- E0h
   16#71# => x"01", -- E2h
   16#72# => x"00", -- E4h
   16#73# => x"20", -- E6h
   16#74# => x"00", -- E8h
   16#75# => x"00", -- EAh
   16#76# => x"00", -- ECh
   16#77# => x"80", -- EEh
   16#78# => x"00", -- F0h
   16#79# => x"1E", -- F2h
   16#7A# => x"00", -- F4h
   16#7B# => x"00", -- F6h
   16#7C# => x"01", -- F8h
   16#7D# => x"00", -- FAh
   16#7E# => x"00", -- FCh
   16#7F# => x"00"  -- FEh
   );
   TYPE SDP_READ_TYPE IS (
   SDP_READ_OFF,
   SDP_READ_ID,
   SDP_READ_CFI,
   SDP_READ_TSC
   );
   SIGNAL SDP_READ : SDP_READ_TYPE := SDP_READ_OFF;
   TYPE SDP_WRITE_TYPE IS (
   SDP_WRITE_OFF,
   SDP_WRITE_EN,
   SDP_WRITE_ERASE
   );
   SIGNAL SDP_WRITE : SDP_WRITE_TYPE := SDP_WRITE_OFF;
   SIGNAL SDP_COUNT : INTEGER RANGE 0 TO 4 := 0;
   SIGNAL ERASE_END : STD_LOGIC := '0';
   TYPE ERASE_END_STATE_MACHINE IS (
   START,
   WR_EN,
   ERASE,
   BUSY,
   STOP
   );
   SIGNAL ERASE_END_CURRENT_STATE : ERASE_END_STATE_MACHINE := START;
   SIGNAL ERASE_END_SECTOR : UNSIGNED (3 DOWNTO 0) := x"0";
   SIGNAL ERASE_END_COUNT : UNSIGNED (3 DOWNTO 0) := x"0";
   SIGNAL ERASE_END_ITER : INTEGER RANGE 0 TO 8 := 0;

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
   DCM33TO96_INST : dcm33to96 PORT MAP (
      CLKIN_IN => LPC_CLK,
      CLKFX_OUT => CLK96,
      CLKIN_IBUFG_OUT => CLK33,
      CLK0_OUT => OPEN
   );
   UART_TX_INST : uart_tx PORT MAP (
      TX_CLK => TX_CLK,
      TX_BYTE => TX_BYTE,
      TX_START => TX_START,
      TX_IDLE => TX_IDLE,
      TX => FTDI_RXD
   );
   TX_CLK <= CTR96(4); -- 3 megabaud

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
   QPI_CS <= "0000" WHEN QPI_EN = QPI_EN_INIT ELSE
             "1111" WHEN QPI_EN = QPI_EN_OFF ELSE
             "0111" WHEN QPI_CHIP = "11" ELSE -- CS3 (when BANK[3:0] = x"F")
             "1011" WHEN QPI_CHIP = "10" ELSE -- U4/CS2 (when BANK[3:0] = x"E")
             "1101" WHEN QPI_CHIP = "01" ELSE -- U3/CS1 (when BANK[3:0] = x"D")
             "1110"; -- U2/CS0 (when BANK[3:0] = x"C")
   QPI_CLK <= REG_00EC(2) WHEN REG_00EC(0) = '1' ELSE
              '0' WHEN QPI_EN = QPI_EN_OFF ELSE NOT CLK33; -- SPI Mode 0

   QPI_LPC_MUTEX <= '1' WHEN ERASE_END = '1' ELSE
                    '0';

   --LAD lines can be either input or output
   --The output values depend on variable states of the LPC transaction
   --Refer to the Intel LPC Specification Rev 1.1
   LPC_LAD <= "ZZZZ" WHEN LPC_CYCLE_UART = '1' AND (LPC_SIO_UART_ARB = DETECT OR LPC_SIO_UART_ARB = MASTER) ELSE
              "0000" WHEN LPC_CURRENT_STATE = SYNC_COMPLETE ELSE
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
   LPC_CYCLE_UART <= '1' WHEN LPC_CYCLE_ACTIVE = '1' AND LPC_CYCLE_MEM = '0' AND LPC_ADDRESS(15 DOWNTO 3) = XENIUM_03F8(15 DOWNTO 3) ELSE
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

   REG_00EF_READ <= SWITCH_RECOVER & QPI_BUSY & HEADER_MISO2 & HEADER_MISO1 & REG_00EF_WRITE(3 DOWNTO 0);

PROCESS (CLK96) BEGIN
   IF rising_edge(CLK96) THEN
      CTR96 <= CTR96 + 1;
   END IF;
END PROCESS;
PROCESS (CLK33) BEGIN
   IF rising_edge(CLK33) THEN
      CTR33 <= CTR33 + 1;
   END IF;
END PROCESS;
PROCESS (CLK33, LPC_RST, QPI_EN_INIT_LATCH, TSOPBOOT) BEGIN
   IF LPC_RST = '0' AND QPI_EN_INIT_LATCH = '1' THEN
      --LPC_RST goes low during boot up or hard reset.
      --Hold D0 low to boot from the LPC bus, only if not booting from TSOP.
      D0LEVEL <= TSOPBOOT;
      --Assert the A20M# pin on the CPU, only if not booting from TSOP.
      A20MLEVEL <= TSOPBOOT;
      SDP_READ <= SDP_READ_OFF;
      SDP_WRITE <= SDP_WRITE_OFF;
      SDP_COUNT <= 0;
      ERASE_END <= '0';
      ERASE_END_CURRENT_STATE <= START;
      REG_00EC <= x"00";
      REG_00ED <= x"00";
      QPI_BUSY <= '0';
      QPI_BUSY_TOGGLE <= '0';
      QPI_EN <= QPI_EN_OFF;
      IF LPC_LFRAME = '1' THEN
         LPC_HAS_LFRAME <= PENDING;
      ELSE
         LPC_HAS_LFRAME <= NO;
      END IF;
      LPC_SIO_UART_ARB <= PENDING;
      CYCLE_TYPE <= IO_READ;
      LPC_CURRENT_STATE <= WAIT_START;
   ELSIF rising_edge(CLK33) THEN
      IF QPI_EN_INIT_LATCH = '0' THEN
         QPI_BUFFER <= QPI_BUFFER(10 DOWNTO 0) & '0';
      ELSE
         QPI_BUFFER <= QPI_BUFFER(7 DOWNTO 0) & "0000";
      END IF;
      IF QPI_EN_INIT_LATCH = '1' AND LPC_SIO_UART_ARB = DETECT AND LPC_CYCLE_UART = '1' AND LPC_LFRAME = '0' AND LPC_LAD = "1111" THEN
         LPC_SIO_UART_ARB <= NOT_FOUND; -- SuperIO on LPC bus not found by host & we become bus master for UART cycles.
         LPC_CURRENT_STATE <= WAIT_START;
      ELSE
      IF QPI_EN_INIT_LATCH = '1' AND LPC_HAS_LFRAME = PENDING AND LPC_CYCLE_MEM = '1' THEN
         -- Detect if LFRAME is bridged to the D0 pad & is not driven by host.
         IF LPC_LFRAME = '1' THEN
            LPC_HAS_LFRAME <= YES;
         ELSE
            LPC_HAS_LFRAME <= NO;
         END IF;
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
                  IF QPI_EN_INIT_AGAIN = '0' THEN
                     QPI_EN_INIT_AGAIN <= '1';
                  ELSE
                     QPI_EN_INIT_AGAIN <= '0';
                     QPI_EN_INIT_LATCH <= '1';
                     -- Set RECOVER bank on power-up if switch is activated.
                     IF SWITCH_RECOVER_LATCH = '0' THEN
                        SWITCH_RECOVER_LATCH <= '1';
                        IF SWITCH_RECOVER = '0' THEN
                           REG_00EF_WRITE(3 DOWNTO 0) <= x"A";
                        END IF;
                     END IF;
                  END IF;
               ELSE
                  COUNT <= COUNT - 1;
               END IF;
            END IF;
         ELSIF TSOPBOOT = '0' AND LPC_LAD = "0000" THEN
            LPC_CURRENT_STATE <= CYCTYPE_DIR;
         END IF;
      WHEN CYCTYPE_DIR =>
         LPC_ADDRESS <= (OTHERS => '0');
         IF LPC_SIO_UART_ARB = DETECT THEN
            LPC_SIO_UART_ARB <= MASTER; -- SuperIO on LPC bus replied before a bus abort from host & is bus master for UART cycles.
         END IF;
         IF LPC_LAD(3 DOWNTO 1) = "000" THEN
            CYCLE_TYPE <= IO_READ;
            COUNT <= 3;
            LPC_CURRENT_STATE <= ADDRESS;
         ELSIF LPC_LAD(3 DOWNTO 1) = "001" THEN
            CYCLE_TYPE <= IO_WRITE;
            COUNT <= 3;
            LPC_CURRENT_STATE <= ADDRESS;
         ELSIF LPC_LAD(3 DOWNTO 1) = "010" THEN
            CYCLE_TYPE <= MEM_READ;
            COUNT <= 7;
            LPC_CURRENT_STATE <= ADDRESS;
            SDP_COUNT <= 0;
         ELSIF LPC_LAD(3 DOWNTO 1) = "011" THEN
            CYCLE_TYPE <= MEM_WRITE;
            COUNT <= 7;
            LPC_CURRENT_STATE <= ADDRESS;
            IF SDP_WRITE /= SDP_WRITE_OFF THEN
               QPI_BUFFER(11 DOWNTO 4) <= QPI_INST_WR_EN;
               QPI_EN <= QPI_EN_OUT;
            END IF;
         ELSE
            LPC_CURRENT_STATE <= WAIT_START; -- Unsupported, reset state machine.
         END IF;

      --ADDRESS GATHERING
      WHEN ADDRESS =>
         COUNT <= COUNT - 1;
         IF LPC_CYCLE_MEM = '1' AND (QPI_LPC_MUTEX = '1' OR REG_00EC(0) = '1') THEN
            IF COUNT = 0 THEN
               IF QPI_LPC_MUTEX = '1' AND CYCLE_TYPE = MEM_READ THEN
                  LPC_CURRENT_STATE <= TAR1;
               ELSE
                  LPC_CURRENT_STATE <= WAIT_START; -- Memory transactions are disabled, reset state machine.
               END IF;
            END IF;
         ELSIF COUNT = 6 THEN
            IF SDP_WRITE /= SDP_WRITE_OFF THEN
               QPI_EN <= QPI_EN_OFF;
            END IF;
            IF QPI_BUSY = '1' THEN
               QPI_BUFFER(11 DOWNTO 4) <= QPI_INST_RSR1;
               QPI_EN <= QPI_EN_OUT;
            ELSIF CYCLE_TYPE = MEM_WRITE THEN
               IF SDP_WRITE = SDP_WRITE_ERASE THEN
                  IF REG_00EF_WRITE(3 DOWNTO 0) = x"B" THEN
                     QPI_BUFFER(7 DOWNTO 0) <= QPI_INST_ERASE_4K;
                  ELSE
                     QPI_BUFFER(7 DOWNTO 0) <= QPI_INST_ERASE_64K;
                  END IF;
               ELSE
                  QPI_BUFFER(7 DOWNTO 0) <= QPI_INST_WRITE;
               END IF;
            ELSE
               QPI_BUFFER(7 DOWNTO 0) <= QPI_INST_READ;
            END IF;
         ELSIF COUNT = 5 THEN
            IF QPI_BUSY = '1' OR SDP_WRITE /= SDP_WRITE_OFF OR (SDP_READ = SDP_READ_OFF AND CYCLE_TYPE = MEM_READ) THEN
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
            IF QPI_BUSY = '1' THEN
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
               IF QPI_BUSY = '1' THEN
                  LPC_BUFFER(7 DOWNTO 4) <= QPI_IO;
               END IF;
               QPI_BUFFER(3 DOWNTO 0) <= LPC_LAD;
            END IF;
            LPC_ADDRESS(15 DOWNTO 12) <= LPC_LAD;
         ELSIF COUNT = 2 THEN
            IF LPC_CYCLE_MEM = '1' THEN
               IF QPI_BUSY = '1' THEN
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
            ELSIF LPC_ADDRESS(15 DOWNTO 4) & LPC_LAD(3 DOWNTO 1) = XENIUM_00EE(15 DOWNTO 1) OR
                 (LPC_ADDRESS(15 DOWNTO 4) & LPC_LAD(3 DOWNTO 2) = XENIUM_00EC(15 DOWNTO 2) AND (REG_00EC(0) = '1' OR SWITCH_RECOVER = '0')) OR
                 (LPC_ADDRESS(15 DOWNTO 4) & LPC_LAD(3) = XENIUM_03F8(15 DOWNTO 3) AND LPC_HAS_LFRAME /= PENDING) THEN
               IF LPC_ADDRESS(15 DOWNTO 4) & LPC_LAD(3) = XENIUM_03F8(15 DOWNTO 3) AND LPC_SIO_UART_ARB = PENDING THEN
                  IF LPC_HAS_LFRAME = YES THEN
                     LPC_SIO_UART_ARB <= DETECT; -- Listen for an LPC bus abort from host for us to become bus master for UART cycles.
                  ELSE
                     LPC_SIO_UART_ARB <= NOT_FOUND; -- We are bus master for UART cycles.
                  END IF;
               END IF;
               IF LPC_CYCLE_WRITE = '0' THEN
                  LPC_CURRENT_STATE <= TAR1;
               ELSE
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
         IF CYCLE_TYPE = MEM_WRITE AND QPI_LPC_MUTEX = '0' THEN
            QPI_BUFFER(7 DOWNTO 4) <= LPC_LAD;
            QPI_BUFFER(3 DOWNTO 0) <= LPC_BUFFER(3 DOWNTO 0);
            IF SDP_WRITE = SDP_WRITE_ERASE THEN
               IF LPC_LAD & LPC_BUFFER(3 DOWNTO 0) /= SDP_ERASE_SECTOR_DATA THEN --x"30"
                  SDP_WRITE <= SDP_WRITE_OFF;
                  QPI_EN <= QPI_EN_OFF;
               ELSIF REG_00EF_WRITE(3 DOWNTO 0) = x"A" AND LPC_ADDRESS(17 DOWNTO 16) = "11" THEN
                  SDP_WRITE <= SDP_WRITE_OFF;
                  QPI_EN <= QPI_EN_OFF;
                  ERASE_END_SECTOR <= UNSIGNED(LPC_ADDRESS(15 DOWNTO 12));
                  ERASE_END <= '1';
                  QPI_BUSY <= '1';
               END IF;
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
         IF CYCLE_TYPE = MEM_WRITE AND SDP_WRITE = SDP_WRITE_ERASE THEN
            QPI_EN <= QPI_EN_OFF;
         END IF;
         LPC_CURRENT_STATE <= TAR2;
      WHEN TAR2 =>
         IF CYCLE_TYPE = MEM_READ AND SDP_READ = SDP_READ_TSC AND LPC_ADDRESS(2 DOWNTO 0) = o"0" THEN
            IF LPC_ADDRESS(3) = '0' THEN
               CTR <= CTR33; -- LPC
            ELSE
               CTR <= CTR96; -- DCM
            END IF;
         END IF;
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
               WHEN XENIUM_03F8(15 DOWNTO 3) & o"5" => --x"03FD"
                  LPC_BUFFER <= "00" & TX_IDLE & '0' & x"0"; -- LSR: THRE (bit 5)
               WHEN OTHERS =>
                  LPC_BUFFER <= x"00";
               END CASE;
               LPC_CURRENT_STATE <= SYNC_COMPLETE;
            ELSIF CYCLE_TYPE = IO_WRITE THEN
               CASE LPC_ADDRESS(15 DOWNTO 0) IS
               WHEN XENIUM_00EC =>
                  IF QPI_LPC_MUTEX = '0' THEN
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
                  END IF;
               WHEN XENIUM_00ED =>
                  IF QPI_LPC_MUTEX = '0' THEN
                     IF REG_00EC(0) = '1' THEN -- BBIO
                        REG_00ED(3 DOWNTO 0) <= LPC_BUFFER(3 DOWNTO 0);
                     END IF;
                  END IF;
               WHEN XENIUM_00EE =>
                  REG_00EE_WRITE <= LPC_BUFFER;
               WHEN XENIUM_00EF =>
                  REG_00EF_WRITE(7 DOWNTO 4) <= LPC_BUFFER(7 DOWNTO 4);
                  IF LPC_BUFFER(3 DOWNTO 0) = x"0" THEN
                     -- Bank 0 will disable state machine and release D0 & A20M# to boot from TSOP after reset.
                     TSOPBOOT <= '1';
                  ELSIF QPI_LPC_MUTEX = '0' THEN
                     IF LPC_BUFFER(3 DOWNTO 2) = "11" THEN
                        IF QPI_CHIP /= LPC_BUFFER(1 DOWNTO 0) AND SDP_READ = SDP_READ_OFF THEN
                           QPI_CHIP <= LPC_BUFFER(1 DOWNTO 0);
                           QPI_BUSY <= '1';
                        END IF;
                     ELSE
                        REG_00EF_WRITE(3 DOWNTO 0) <= LPC_BUFFER(3 DOWNTO 0);
                     END IF;
                  END IF;
               WHEN XENIUM_03F8 =>
                  IF TX_START = '0' AND TX_IDLE = '1' THEN
                     TX_START <= '1';
                     TX_BYTE <= LPC_BUFFER;
                  END IF;
               WHEN OTHERS =>
               END CASE;
               LPC_CURRENT_STATE <= SYNC_COMPLETE;
            ELSIF QPI_BUSY = '1' THEN
               IF QPI_LPC_MUTEX = '0' THEN
                  QPI_BUSY <= LPC_BUFFER(0);
               ELSIF ERASE_END = '1' AND ERASE_END_CURRENT_STATE = STOP THEN
                  ERASE_END <= '0';
                  ERASE_END_CURRENT_STATE <= START;
                  QPI_BUSY <= '0';
               END IF;
               IF CYCLE_TYPE = MEM_READ THEN
                  LPC_BUFFER(0) <= QPI_BUSY_TOGGLE;
                  QPI_BUSY_TOGGLE <= NOT QPI_BUSY_TOGGLE;
               END IF;
               LPC_CURRENT_STATE <= SYNC_COMPLETE;
            ELSIF CYCLE_TYPE = MEM_READ THEN
               IF SDP_READ = SDP_READ_ID THEN
                  IF LPC_ADDRESS(1) = '0' THEN
                     LPC_BUFFER <= SDP_ID_VENDOR;
                  ELSE
                     LPC_BUFFER <= SDP_ID_DEVICE;
                  END IF;
                  LPC_CURRENT_STATE <= SYNC_COMPLETE;
               ELSIF SDP_READ = SDP_READ_CFI THEN
                  IF UNSIGNED(LPC_ADDRESS(7 DOWNTO 1)) < x"10" THEN
                     LPC_BUFFER <= x"FF";
                  ELSE
                     LPC_BUFFER <= SDP_CFI_ROM(TO_INTEGER(UNSIGNED(LPC_ADDRESS(7 DOWNTO 1)) - x"10"));
                  END IF;
                  LPC_CURRENT_STATE <= SYNC_COMPLETE;
               ELSIF SDP_READ = SDP_READ_TSC THEN
                  LPC_BUFFER <= STD_LOGIC_VECTOR(CTR_U8LE_MAP(TO_INTEGER(UNSIGNED(LPC_ADDRESS(2 DOWNTO 0)))));
                  LPC_CURRENT_STATE <= SYNC_COMPLETE;
               END IF;
            ELSIF CYCLE_TYPE = MEM_WRITE THEN
               IF SDP_WRITE /= SDP_WRITE_OFF THEN
                  QPI_BUSY <= '1';
                  QPI_EN <= QPI_EN_OFF;
               ELSIF SDP_READ /= SDP_READ_OFF THEN
                  IF LPC_BUFFER = SDP_RESET_DATA THEN --x"F0"
                     SDP_READ <= SDP_READ_OFF;
                  END IF;
               ELSIF LPC_ADDRESS(7 DOWNTO 0) = SDP_TICK_ADDR(7 DOWNTO 0) AND LPC_BUFFER = SDP_CFI_DATA THEN --x"AA" AND x"98"
                  SDP_READ <= SDP_READ_CFI;
               ELSIF LPC_ADDRESS(7 DOWNTO 0) = SDP_TICK_ADDR(7 DOWNTO 0) AND LPC_BUFFER = SDP_TSC_DATA THEN --x"AA" AND x"99"
                  SDP_READ <= SDP_READ_TSC;
               ELSE
                  CASE LPC_ADDRESS(11 DOWNTO 0) IS
                  WHEN SDP_TICK_ADDR => --x"AAA"
                     IF (SDP_COUNT = 0 OR SDP_COUNT = 3) AND LPC_BUFFER = SDP_TICK_DATA THEN --x"AA"
                        SDP_COUNT <= SDP_COUNT + 1;
                     ELSIF SDP_COUNT = 2 AND LPC_BUFFER = SDP_ID_DATA THEN --x"90"
                        SDP_READ <= SDP_READ_ID;
                        SDP_COUNT <= 0;
                     ELSIF SDP_COUNT = 2 AND LPC_BUFFER = SDP_WRITE_DATA THEN --x"A0"
                        IF A20MLEVEL = '1' THEN
                           SDP_WRITE <= SDP_WRITE_EN;
                        END IF;
                        SDP_COUNT <= 0;
                     ELSIF SDP_COUNT = 2 AND LPC_BUFFER = SDP_ERASE_DATA THEN --x"80"
                        SDP_COUNT <= 3;
                     ELSE
                        SDP_COUNT <= 0;
                     END IF;
                  WHEN SDP_TOCK_ADDR => --x"555"
                     IF SDP_COUNT = 1 AND LPC_BUFFER = SDP_TOCK_DATA THEN --x"55"
                        SDP_COUNT <= SDP_COUNT + 1;
                     ELSIF SDP_COUNT = 4 AND LPC_BUFFER = SDP_TOCK_DATA THEN --x"55"
                        IF A20MLEVEL = '1' THEN
                           SDP_WRITE <= SDP_WRITE_ERASE;
                        END IF;
                        SDP_COUNT <= 0;
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
         IF QPI_BUSY = '1' THEN
            SDP_WRITE <= SDP_WRITE_OFF;
         END IF;
         IF TX_START = '1' THEN
            TX_START <= '0';
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
      IF ERASE_END = '1' THEN
         ERASE_END_ITER <= ERASE_END_ITER - 1;
         CASE ERASE_END_CURRENT_STATE IS
         WHEN START =>
            IF REG_00EF_WRITE(3 DOWNTO 0) = x"A" THEN
               IF ERASE_END_SECTOR <= x"7" THEN
                  -- 8 4KiB sector erase @ 0x1F0000-0x1F7FFF
                  ERASE_END_COUNT <= x"7" - ERASE_END_SECTOR;
               ELSIF ERASE_END_SECTOR <= x"9" THEN
                  -- 2 4KiB sector erase @ 0x1F8000-0x1F9FFF
                  ERASE_END_COUNT <= x"9" - ERASE_END_SECTOR;
               ELSIF ERASE_END_SECTOR <= x"B" THEN
                  -- 2 4KiB sector erase @ 0x1FA000-0x1FBFFF
                  ERASE_END_COUNT <= x"B" - ERASE_END_SECTOR;
               ELSE
                  -- 4 4KiB sector erase @ 0x1FC000-0x1FFFFF
                  ERASE_END_COUNT <= x"F" - ERASE_END_SECTOR;
               END IF;
               ERASE_END_ITER <= 2;
               ERASE_END_CURRENT_STATE <= WR_EN;
            ELSE
               ERASE_END <= '0';
               QPI_BUSY <= '0';
            END IF;
         WHEN WR_EN =>
            IF ERASE_END_ITER = 2 THEN
               QPI_BUFFER(11 DOWNTO 4) <= QPI_INST_WR_EN;
               QPI_EN <= QPI_EN_OUT;
            ELSIF ERASE_END_ITER = 0 THEN
               QPI_EN <= QPI_EN_OFF;
               ERASE_END_ITER <= 8;
               ERASE_END_CURRENT_STATE <= ERASE;
            END IF;
         WHEN ERASE =>
            IF ERASE_END_ITER = 8 THEN
               QPI_BUFFER(11 DOWNTO 4) <= QPI_INST_ERASE_4K;
               QPI_EN <= QPI_EN_OUT;
            ELSIF ERASE_END_ITER = 7 THEN
               QPI_BUFFER(7 DOWNTO 0) <= x"1F";
            ELSIF ERASE_END_ITER = 6 THEN
               QPI_BUFFER(3 DOWNTO 0) <= STD_LOGIC_VECTOR(ERASE_END_SECTOR);
            ELSIF ERASE_END_ITER = 5 THEN
               QPI_BUFFER(3 DOWNTO 0) <= x"0";
            ELSIF ERASE_END_ITER = 4 THEN
               QPI_BUFFER(3 DOWNTO 0) <= x"0";
            ELSIF ERASE_END_ITER = 3 THEN
               QPI_BUFFER(3 DOWNTO 0) <= x"0";
            ELSIF ERASE_END_ITER = 0 THEN
               QPI_EN <= QPI_EN_OFF;
               ERASE_END_ITER <= 4;
               ERASE_END_CURRENT_STATE <= BUSY;
            END IF;
         WHEN BUSY =>
            IF ERASE_END_ITER = 4 THEN
               QPI_BUFFER(11 DOWNTO 4) <= QPI_INST_RSR1;
               QPI_EN <= QPI_EN_OUT;
            ELSIF ERASE_END_ITER = 2 THEN
               QPI_EN <= QPI_EN_IN;
            ELSIF ERASE_END_ITER = 0 THEN
               QPI_EN <= QPI_EN_OFF;
               IF QPI_IO(0) = '1' THEN
                  ERASE_END_ITER <= 4;
               ELSE
                  IF ERASE_END_COUNT > 0 THEN
                     ERASE_END_COUNT <= ERASE_END_COUNT - 1;
                     ERASE_END_SECTOR <= ERASE_END_SECTOR + 1;
                     ERASE_END_ITER <= 2;
                     ERASE_END_CURRENT_STATE <= WR_EN;
                  ELSE
                     ERASE_END_CURRENT_STATE <= STOP;
                  END IF;
               END IF;
            END IF;
         WHEN STOP =>
         WHEN OTHERS =>
         END CASE;
      END IF;
   END IF;
END PROCESS;
END Behavioral;
