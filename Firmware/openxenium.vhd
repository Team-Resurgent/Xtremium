-- Design Name: openxenium
-- Module Name: openxenium - Behavioral
-- Project Name: OpenXenium QPI. Open Source Xenius modchip CPLD replacement project
-- Target Devices: XC95144XL-TQ100
--
-- Revision 0.01 (2019/09/20) - File Created - Ryan Wendland
-- Revision 2022/04/04 - Use SOIC-8 16MiB Flash Banks in QPI Mode - Michael Saga
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
-- along with this program. If not, see <http://www.gnu.org/licenses/>.
--
----------------------------------------------------------------------------------
--
--
--**BANK SELECTION**
--Bank selection is controlled by the lower nibble of address REG_00EF.
--A20,A19,A18 are address lines to the parallel flash memory.
--lines marked X means it is not forced by the CPLD for banking purposes.
--This is how is works:
--
--REGISTER 0xEF Bank Commands:
--BANK NAME                  DATA BYTE    A20|A19|A18 ADDRESS OFFSET
--TSOP                       XXXX 0000     X |X |X    N/A.     (This locks up the Xenium to force it to boot from TSOP.)
--XeniumOS(c.well loader)    XXXX 0001     1 |1 |0    0x180000 (This is the default boot state. Contains Cromwell bootloader)
--XeniumOS                   XXXX 0010     1 |0 |X    0x100000 (This is a 512kb bank and contains XeniumOS)
--BANK1 (USER BIOS 256kB)    XXXX 0011     0 |0 |0    0x000000
--BANK2 (USER BIOS 256kB)    XXXX 0100     0 |0 |1    0x040000
--BANK3 (USER BIOS 256kB)    XXXX 0101     0 |1 |0    0x080000
--BANK4 (USER BIOS 256kB)    XXXX 0110     0 |1 |1    0x0C0000
--BANK1 (USER BIOS 512kB)    XXXX 0111     0 |0 |X    0x000000
--BANK2 (USER BIOS 512kB)    XXXX 1000     0 |1 |X    0x080000
--BANK1 (USER BIOS 1MB)      XXXX 1001     0 |X |X    0x000000
--RECOVERY (NOTE 1)          XXXX 1010     1 |1 |1    0x1C0000
--
--
--NOTE 1: The RECOVERY bank can also be actived by the physical switch on the Xenium. This forces bank ten (0b1010) on power up.
--This bank also contains non-volatile storage of settings an EEPROM backup in the smaller sectors at the end of the flash memory.
--The memory map is shown below:
--     (1C0000 to 1DFFFF PROTECTED AREA 128kbyte recovery bios)
--     (1E0000 to 1FBFFF Additional XeniumOS Data)
--     (1FC000 to 1FFFFF Contains eeprom backup, XeniumOS settings)
--
--
--**XENIUM CONTROL WRITE/READ REGISTERS**
--Bits marked 'X' either have no function or an unknown function.
--
--**0xEF WRITE:**
--X,SCK,CS,MOSI,BANK[3:0]
--
--**0xEF READ:**
--RECOVERY (Active Low),X,MISO2 (Header Pin 4),MISO1 (Header Pin 1),BANK[3:0]
--
--**0xEE WRITE:**
--X,X,X,X,X,B,G,R (DEFAULT LED ON POWER UP IS RED)
--
--**0xEE READ:**
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
      HEADER_LED_R : OUT STD_LOGIC;
      HEADER_LED_G : OUT STD_LOGIC;
      HEADER_LED_B : OUT STD_LOGIC;

      QPI_IO : INOUT STD_LOGIC_VECTOR (3 DOWNTO 0);
      QPI_CS : OUT STD_LOGIC_VECTOR (2 DOWNTO 0);

      LPC_LAD : INOUT STD_LOGIC_VECTOR (3 DOWNTO 0);
      LPC_CLK : IN STD_LOGIC;
      LPC_RST : IN STD_LOGIC;

      XENIUM_RECOVERY : IN STD_LOGIC; -- Recovery is active low and requires an external pull-up resistor to 3.3V.
      XENIUM_D0 : OUT STD_LOGIC
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

   SIGNAL LPC_ADDRESS : STD_LOGIC_VECTOR (23 DOWNTO 0); -- LPC address width submitted on the bus is actually 32 bits, but we only need 24.

   SIGNAL LPC_STATE_LED_EN : STD_LOGIC; -- Overlay LPC bus activity on the LED.
   SIGNAL LPC_STATE_ACTIVE : STD_LOGIC;
   SIGNAL LPC_STATE_WRITE : STD_LOGIC;
   SIGNAL LPC_STATE_IO : STD_LOGIC;

   --XENIUM IO REGISTERS. BITS MARKED 'X' HAVE AN UNKNOWN FUNCTION OR ARE UNUSED. NEEDS MORE RE.
   --Bit masks are all shown upper nibble first.

   --IO READ/WRITE REGISTERS
   CONSTANT XENIUM_00EE : STD_LOGIC_VECTOR (15 DOWNTO 0) := x"00EE"; -- RGB LED Control Register
   CONSTANT XENIUM_00EF : STD_LOGIC_VECTOR (15 DOWNTO 0) := x"00EF"; -- SPI and Banking Control Register
   CONSTANT REG_00EE_READ : STD_LOGIC_VECTOR (7 DOWNTO 0) := x"AA"; -- OpenXenium QPI (OpenXenium & Genuine Xenium return 0x55)
   SIGNAL REG_00EE_WRITE : STD_LOGIC_VECTOR (7 DOWNTO 0) := "00000001"; -- X,X,X,X,X,B,G,R (Red is default LED colour on power-up)
   SIGNAL REG_00EF_WRITE : STD_LOGIC_VECTOR (7 DOWNTO 0) := "00000001"; -- X,SCK,CS,MOSI,BANK[3:0]
   SIGNAL REG_00EF_READ : STD_LOGIC_VECTOR (7 DOWNTO 0) := "01010101"; -- RECOVERY (Active Low),X,MISO2 (Header Pin 4),MISO1 (Header Pin 1),BANK[3:0]
   SIGNAL BYTEBUFFER : STD_LOGIC_VECTOR (7 DOWNTO 0); -- Generic byte buffer

   --QPI READ/WRITE REGISTERS FOR FLASH MEMORY
   -- QPI Instructions (W25Q128JV-DTR Rev C section 6.1.4)
   CONSTANT QPI_INST_READ : STD_LOGIC_VECTOR (7 DOWNTO 0) := x"EB"; -- Fast Read Quad I/O in QPI Mode (W25Q128JV-DTR Rev C section 8.2.14)
   CONSTANT QPI_INST_WR_EN : STD_LOGIC_VECTOR (7 DOWNTO 0) := x"06"; -- Write Enable (W25Q128JV-DTR Rev C section 8.2.1)
   CONSTANT QPI_INST_WR_DI : STD_LOGIC_VECTOR (7 DOWNTO 0) := x"04"; -- Write Disable (W25Q128JV-DTR Rev C section 8.2.3)
   CONSTANT QPI_INST_WRITE : STD_LOGIC_VECTOR (7 DOWNTO 0) := x"02"; -- Page Program (W25Q128JV-DTR Rev C section 8.2.16)
   CONSTANT QPI_INST_ERASE : STD_LOGIC_VECTOR (7 DOWNTO 0) := x"20"; -- 4KiB Sector Erase (W25Q128JV-DTR Rev C section 8.2.18)
   SIGNAL QPI_BUFFER : STD_LOGIC_VECTOR (11 DOWNTO 0) := (OTHERS => '0'); -- 12-bit output shift register
   SIGNAL QPI_EN_OUT : STD_LOGIC := '0';
   SIGNAL QPI_EN_IN : STD_LOGIC := '0';

   --TSOPBOOT IS SET TO '1' WHEN YOU REQUEST TO BOOT FROM TSOP. THIS PREVENTS THE CPLD FROM DRIVING D0.
   --D0LEVEL is inverted and connected to the D0 output pad. This allows the CPLD to latch/release the D0/LFRAME signal.
--   SIGNAL TSOPBOOT : STD_LOGIC := '0';
   SIGNAL D0LEVEL : STD_LOGIC := '0';

   --GENERIC COUNTER USED TO TRACK ADDRESS AND SYNC COUNTERS.
   SIGNAL COUNT : INTEGER RANGE 0 TO 7;

BEGIN
   --ASSIGN THE IO TO SIGNALS BASED ON REQUIRED BEHAVIOUR
   HEADER_CS <= REG_00EF_WRITE(5);
   HEADER_SCK <= REG_00EF_WRITE(6);
   HEADER_MOSI <= REG_00EF_WRITE(4);

   HEADER_LED_R <= '1' WHEN LPC_STATE_LED_EN = '1' AND LPC_STATE_ACTIVE = '1' AND LPC_STATE_WRITE = '0' ELSE
                   '0' WHEN LPC_STATE_LED_EN = '1' ELSE
                   REG_00EE_WRITE(0);
   HEADER_LED_G <= '1' WHEN LPC_STATE_LED_EN = '1' AND LPC_STATE_ACTIVE = '1' AND LPC_STATE_WRITE = '1' ELSE
                   '0' WHEN LPC_STATE_LED_EN = '1' ELSE
                   REG_00EE_WRITE(1);
   HEADER_LED_B <= '1' WHEN LPC_STATE_LED_EN = '1' AND LPC_STATE_ACTIVE = '1' AND LPC_STATE_IO = '1' ELSE
                   '0' WHEN LPC_STATE_LED_EN = '1' ELSE
                   REG_00EE_WRITE(2);

   QPI_IO <= QPI_BUFFER(11 DOWNTO 8) WHEN QPI_EN_OUT = '1' AND QPI_EN_IN = '0' ELSE
             "ZZZZ";

   QPI_CS <= "111" WHEN QPI_EN_OUT = '0' AND QPI_EN_IN = '0' ELSE
             "011" WHEN REG_00EF_WRITE(3 DOWNTO 0) = x"3" ELSE -- Bank 3 (U4)
             "101" WHEN REG_00EF_WRITE(3 DOWNTO 0) = x"2" ELSE -- Bank 2 (U3)
             "110"; -- Bank 1 (U2)

   --LAD lines can be either input or output
   --The output values depend on variable states of the LPC transaction
   --Refer to the Intel LPC Specification Rev 1.1
   LPC_LAD <= "0000" WHEN LPC_CURRENT_STATE = SYNC_COMPLETE ELSE
              "0101" WHEN LPC_CURRENT_STATE = SYNCING ELSE
              "1111" WHEN LPC_CURRENT_STATE = TAR2 ELSE
              "1111" WHEN LPC_CURRENT_STATE = TAR_EXIT ELSE
              BYTEBUFFER(3 DOWNTO 0) WHEN LPC_CURRENT_STATE = READ_DATA0 ELSE -- This happens lower nibble first! (Refer to Intel LPC spec)
              BYTEBUFFER(7 DOWNTO 4) WHEN LPC_CURRENT_STATE = READ_DATA1 ELSE
              "ZZZZ";

   LPC_STATE_LED_EN <= '1' WHEN REG_00EE_WRITE(2 DOWNTO 0) = "000" ELSE
                       '0';
   LPC_STATE_ACTIVE <= '1' WHEN LPC_CURRENT_STATE /= WAIT_START AND LPC_CURRENT_STATE /= CYCTYPE_DIR ELSE
                       '0';
   LPC_STATE_WRITE <= '1' WHEN CYCLE_TYPE = IO_WRITE OR CYCLE_TYPE = MEM_WRITE ELSE
                      '0';
   LPC_STATE_IO <= '1' WHEN CYCLE_TYPE = IO_READ OR CYCLE_TYPE = IO_WRITE ELSE
                   '0';

   --D0 has the following behaviour
   --Held low on boot to ensure it boots from the LPC then released when definitely booting from modchip.
   --When soldered to LFRAME it will simulate LPC transaction aborts for 1.6.
   --Released for TSOP booting.
   --NOTE: XENIUM_D0 is an output to a mosfet driver. '0' turns off the MOSFET releasing D0
   --and a value of '1' turns on the MOSFET forcing it to ground. This is why I invert D0LEVEL before mapping it.
   XENIUM_D0 <= --'0' WHEN TSOPBOOT = '1' ELSE
                '1' WHEN CYCLE_TYPE = MEM_READ ELSE
                '1' WHEN CYCLE_TYPE = MEM_WRITE ELSE
                NOT D0LEVEL;

   REG_00EF_READ <= XENIUM_RECOVERY & '0' & HEADER_MISO2 & HEADER_MISO1 & REG_00EF_WRITE(3 DOWNTO 0);

PROCESS (LPC_CLK, LPC_RST) BEGIN
   IF LPC_RST = '0' THEN
      --LPC_RST goes low during boot up or hard reset.
      --We need to set D0 only if not TSOP booting.
      D0LEVEL <= '0';--TSOPBOOT;
      REG_00EE_WRITE(2 DOWNTO 0) <= "000";
      QPI_EN_OUT <= '0';
      QPI_EN_IN <= '0';
      CYCLE_TYPE <= IO_READ;
      LPC_CURRENT_STATE <= WAIT_START;
   ELSIF rising_edge(LPC_CLK) THEN
      QPI_BUFFER <= QPI_BUFFER(7 DOWNTO 0) & "0000";
      CASE LPC_CURRENT_STATE IS
         WHEN WAIT_START =>
            CYCLE_TYPE <= IO_READ;
            IF LPC_LAD = "0000" --AND TSOPBOOT = '0' 
            THEN
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
            ELSIF LPC_LAD(3 DOWNTO 1) = "010" THEN
               CYCLE_TYPE <= MEM_READ;
               COUNT <= 7;
               LPC_CURRENT_STATE <= ADDRESS;
            ELSIF LPC_LAD(3 DOWNTO 1) = "011" THEN
               CYCLE_TYPE <= MEM_WRITE;
               COUNT <= 7;
               LPC_CURRENT_STATE <= ADDRESS;
               -- Write Protect Features (W25Q128JV-DTR Rev C section 6.2.1 paragraph 2)
               QPI_EN_OUT <= '1';
               IF XENIUM_RECOVERY = '0' THEN
                  QPI_BUFFER(11 DOWNTO 4) <= QPI_INST_WR_EN;
               ELSE
                  QPI_BUFFER(11 DOWNTO 4) <= QPI_INST_WR_DI;
               END IF;
            ELSE
               LPC_CURRENT_STATE <= WAIT_START; -- Unsupported, reset state machine.
            END IF;

         --ADDRESS GATHERING
         WHEN ADDRESS =>
            IF COUNT = 6 THEN
               QPI_EN_OUT <= '0';
               IF CYCLE_TYPE = MEM_WRITE THEN
                  QPI_BUFFER(7 DOWNTO 0) <= QPI_INST_WRITE;
               ELSE
                  QPI_BUFFER(7 DOWNTO 0) <= QPI_INST_READ;
               END IF;
            ELSIF COUNT = 5 THEN
               QPI_EN_OUT <= '1';
               QPI_BUFFER(3 DOWNTO 0) <= LPC_LAD;
               LPC_ADDRESS(23 DOWNTO 20) <= LPC_LAD;
            ELSIF COUNT = 4 THEN
               QPI_BUFFER(3 DOWNTO 0) <= LPC_LAD;
               LPC_ADDRESS(19 DOWNTO 16) <= LPC_LAD;
               --BANK CONTROL
               -- Set recovery bank if switch is activated
--               IF XENIUM_RECOVERY = '0' --AND TSOPBOOT = '0' 
--               AND D0LEVEL = '0' THEN
--                  REG_00EF_WRITE(3 DOWNTO 0) <= "1010";
--               END IF;
--               CASE REG_00EF_WRITE(3 DOWNTO 0) IS
--                  WHEN "0001" =>
--                     LPC_ADDRESS(23 DOWNTO 21) <= "111";
--                     LPC_ADDRESS(20 DOWNTO 18) <= "110"; --256kb bank
--                  WHEN "0010" =>
--                     LPC_ADDRESS(23 DOWNTO 21) <= "111";
--                     LPC_ADDRESS(20 DOWNTO 19) <= "10"; --512kb bank
--                  WHEN "0011" =>
--                     LPC_ADDRESS(23 DOWNTO 21) <= "111";
--                     LPC_ADDRESS(20 DOWNTO 18) <= "000"; --256kb bank
--                  WHEN "0100" =>
--                     LPC_ADDRESS(23 DOWNTO 21) <= "111";
--                     LPC_ADDRESS(20 DOWNTO 18) <= "001"; --256kb bank
--                  WHEN "0101" =>
--                     LPC_ADDRESS(23 DOWNTO 21) <= "111";
--                     LPC_ADDRESS(20 DOWNTO 18) <= "010"; --256kb bank
--                  WHEN "0110" =>
--                     LPC_ADDRESS(23 DOWNTO 21) <= "111";
--                     LPC_ADDRESS(20 DOWNTO 18) <= "011"; --256kb bank
--                  WHEN "0111" =>
--                     LPC_ADDRESS(23 DOWNTO 21) <= "111";
--                     LPC_ADDRESS(20 DOWNTO 19) <= "00"; --512kb bank
--                  WHEN "1000" =>
--                     LPC_ADDRESS(23 DOWNTO 21) <= "111";
--                     LPC_ADDRESS(20 DOWNTO 19) <= "01"; --512kb bank
--                  WHEN "1001" =>
--                     LPC_ADDRESS(23 DOWNTO 21) <= "111";
--                     LPC_ADDRESS(20) <= '0'; --1mb bank
--                  WHEN "1010" =>
--                     LPC_ADDRESS(23 DOWNTO 21) <= "111";
--                     LPC_ADDRESS(20 DOWNTO 18) <= "111"; --256kb bank
----                  WHEN "0000" =>
                     --Bank zero will disable modchip and release D0 and reset state machine.
----                     LPC_CURRENT_STATE <= WAIT_START;
----                     TSOPBOOT <= '1';
--                  WHEN OTHERS =>
--               END CASE;
            ELSIF COUNT = 3 THEN
               QPI_BUFFER(3 DOWNTO 0) <= LPC_LAD;
               LPC_ADDRESS(15 DOWNTO 12) <= LPC_LAD;
            ELSIF COUNT = 2 THEN
               QPI_BUFFER(3 DOWNTO 0) <= LPC_LAD;
               LPC_ADDRESS(11 DOWNTO 8) <= LPC_LAD;
            ELSIF COUNT = 1 THEN
               QPI_BUFFER(3 DOWNTO 0) <= LPC_LAD;
               LPC_ADDRESS(7 DOWNTO 4) <= LPC_LAD;
            ELSIF COUNT = 0 THEN
               QPI_BUFFER(3 DOWNTO 0) <= LPC_LAD;
               LPC_ADDRESS(3 DOWNTO 0) <= LPC_LAD;
               IF CYCLE_TYPE = MEM_READ THEN
                  LPC_CURRENT_STATE <= TAR1;
               ELSIF CYCLE_TYPE = MEM_WRITE THEN
                  LPC_CURRENT_STATE <= WRITE_DATA0;
               ELSIF LPC_ADDRESS(15 DOWNTO 4) & LPC_LAD(3 DOWNTO 1) = XENIUM_00EE(15 DOWNTO 1) THEN
                  -- Respond to our LPC IO address register, which may be either 0x00EE or 0x00EF.
                  IF CYCLE_TYPE = IO_READ THEN
                     LPC_CURRENT_STATE <= TAR1;
                  ELSIF CYCLE_TYPE = IO_WRITE THEN
                     LPC_CURRENT_STATE <= WRITE_DATA0;
                  END IF;
               ELSE
                  LPC_CURRENT_STATE <= WAIT_START; -- Unsupported, reset state machine.
               END IF;
            END IF;
            COUNT <= COUNT - 1;

         --MEMORY OR IO WRITES
         WHEN WRITE_DATA0 =>
            BYTEBUFFER(3 DOWNTO 0) <= LPC_LAD; -- This happens lower nibble first! (Refer to Intel LPC spec)
            LPC_CURRENT_STATE <= WRITE_DATA1;
         WHEN WRITE_DATA1 =>
            IF CYCLE_TYPE = MEM_WRITE THEN
               QPI_BUFFER(7 DOWNTO 4) <= LPC_LAD;
               QPI_BUFFER(3 DOWNTO 0) <= BYTEBUFFER(3 DOWNTO 0);
            END IF;
            BYTEBUFFER(7 DOWNTO 4) <= LPC_LAD;
            LPC_CURRENT_STATE <= TAR1;

         --MEMORY OR IO READS
         WHEN READ_DATA0 =>
            LPC_CURRENT_STATE <= READ_DATA1;
         WHEN READ_DATA1 =>
            LPC_CURRENT_STATE <= TAR_EXIT;

         --TURN BUS AROUND (HOST TO PERIPHERAL)
         WHEN TAR1 =>
            LPC_CURRENT_STATE <= TAR2;
         WHEN TAR2 =>
            LPC_CURRENT_STATE <= SYNCING;
            COUNT <= 5;

         --SYNCING STAGE
         WHEN SYNCING =>
            COUNT <= COUNT - 1;
            IF COUNT = 4 THEN
               IF CYCLE_TYPE = IO_READ THEN
                  IF LPC_ADDRESS(15 DOWNTO 0) = XENIUM_00EF THEN
                     BYTEBUFFER <= REG_00EF_READ;
                  ELSE
                     BYTEBUFFER <= REG_00EE_READ;
                  END IF;
                  LPC_CURRENT_STATE <= SYNC_COMPLETE;
               ELSIF CYCLE_TYPE = IO_WRITE THEN
                  IF LPC_ADDRESS(15 DOWNTO 0) = XENIUM_00EF THEN
                     REG_00EF_WRITE(7 DOWNTO 4) <= BYTEBUFFER(7 DOWNTO 4);
                     IF BYTEBUFFER(3 DOWNTO 0) /= x"0" THEN
                        -- Must be a valid bank between 1 to 3.
                        IF BYTEBUFFER(3 DOWNTO 0) = x"2" OR BYTEBUFFER(3 DOWNTO 0) = x"3" THEN
                           REG_00EF_WRITE(3 DOWNTO 0) <= BYTEBUFFER(3 DOWNTO 0);
                        ELSE
                           REG_00EF_WRITE(3 DOWNTO 0) <= x"1";
                        END IF;
                     END IF;
                  ELSE
                     REG_00EE_WRITE <= BYTEBUFFER;
                  END IF;
                  LPC_CURRENT_STATE <= SYNC_COMPLETE;
               ELSIF CYCLE_TYPE = MEM_WRITE THEN
                  QPI_EN_OUT <= '0';
               END IF;
            ELSIF COUNT = 3 THEN
               IF CYCLE_TYPE = MEM_READ THEN
                  QPI_EN_OUT <= '0';
                  QPI_EN_IN <= '1';
               END IF;
            ELSIF COUNT = 2 THEN
               IF CYCLE_TYPE = MEM_READ THEN
                  BYTEBUFFER(7 DOWNTO 4) <= QPI_IO;
               END IF;
            ELSIF COUNT = 1 THEN
               IF CYCLE_TYPE = MEM_READ THEN
                  BYTEBUFFER(3 DOWNTO 0) <= QPI_IO;
               END IF;
            ELSIF COUNT = 0 THEN
               IF CYCLE_TYPE = MEM_READ THEN
                  QPI_EN_IN <= '0';
               END IF;
               LPC_CURRENT_STATE <= SYNC_COMPLETE;
            END IF;
         WHEN SYNC_COMPLETE =>
            IF CYCLE_TYPE = MEM_READ OR CYCLE_TYPE = IO_READ THEN
               LPC_CURRENT_STATE <= READ_DATA0;
            ELSE
               LPC_CURRENT_STATE <= TAR_EXIT;
            END IF;

         --TURN BUS AROUND (PERIPHERAL TO HOST)
         WHEN TAR_EXIT =>
            --D0 is held low until a few memory reads
            --This ensures it is booting from the modchip. Genuine Xenium arbitrarily
            --releases after the 5th read. This is always address 0x74.
            IF LPC_ADDRESS(7 DOWNTO 0) = x"74" THEN
               D0LEVEL <= '1';
            END IF;
            LPC_CURRENT_STATE <= WAIT_START;
      END CASE;
   END IF;
END PROCESS;
END Behavioral;
