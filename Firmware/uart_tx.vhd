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

LIBRARY IEEE;
USE IEEE.STD_LOGIC_1164.ALL;

ENTITY uart_tx IS
   PORT (
      TX_CLK : IN STD_LOGIC;
      TX_BYTE : IN STD_LOGIC_VECTOR (7 DOWNTO 0);
      TX_READ : IN STD_LOGIC;
      TX_READY : OUT STD_LOGIC;
      TX : OUT STD_LOGIC
   );
END uart_tx;

ARCHITECTURE ARCH_UART_TX OF uart_tx IS
   SIGNAL CLK_BAUD : STD_LOGIC;

   TYPE BYTE_STATE_MACHINE IS (
   READ,
   XMIT
   );
   SIGNAL BYTE_CURRENT_STATE : BYTE_STATE_MACHINE := READ;
   SIGNAL BYTE_START : STD_LOGIC := '0';
   SIGNAL BYTE_READ : STD_LOGIC;
   SIGNAL BYTE_IN : STD_LOGIC_VECTOR (7 DOWNTO 0);
   SIGNAL BYTE_OUT : STD_LOGIC_VECTOR (7 DOWNTO 0);

   TYPE TX_STATE_MACHINE IS (
   START,
   BIT0,
   BIT1,
   BIT2,
   BIT3,
   BIT4,
   BIT5,
   BIT6,
   BIT7,
   STOP
   );
   SIGNAL TX_CURRENT_STATE : TX_STATE_MACHINE := START;
   SIGNAL TX_NEXT_STATE : TX_STATE_MACHINE;
   SIGNAL TX_RST : STD_LOGIC;
BEGIN
   CLK_BAUD <= TX_CLK;

   TX_READY <= NOT BYTE_START AND NOT BYTE_READ;

   TX <= TX_RST WHEN TX_CURRENT_STATE = START ELSE -- Start bit
         BYTE_OUT(0) WHEN TX_CURRENT_STATE = BIT0 ELSE -- LSB
         BYTE_OUT(1) WHEN TX_CURRENT_STATE = BIT1 ELSE
         BYTE_OUT(2) WHEN TX_CURRENT_STATE = BIT2 ELSE
         BYTE_OUT(3) WHEN TX_CURRENT_STATE = BIT3 ELSE
         BYTE_OUT(4) WHEN TX_CURRENT_STATE = BIT4 ELSE
         BYTE_OUT(5) WHEN TX_CURRENT_STATE = BIT5 ELSE
         BYTE_OUT(6) WHEN TX_CURRENT_STATE = BIT6 ELSE
         BYTE_OUT(7) WHEN TX_CURRENT_STATE = BIT7 ELSE -- MSB
         '1'; -- Stop bit

   TX_NEXT_STATE <= BIT0 WHEN TX_CURRENT_STATE = START ELSE
                    BIT1 WHEN TX_CURRENT_STATE = BIT0 ELSE
                    BIT2 WHEN TX_CURRENT_STATE = BIT1 ELSE
                    BIT3 WHEN TX_CURRENT_STATE = BIT2 ELSE
                    BIT4 WHEN TX_CURRENT_STATE = BIT3 ELSE
                    BIT5 WHEN TX_CURRENT_STATE = BIT4 ELSE
                    BIT6 WHEN TX_CURRENT_STATE = BIT5 ELSE
                    BIT7 WHEN TX_CURRENT_STATE = BIT6 ELSE
                    STOP WHEN TX_CURRENT_STATE = BIT7 ELSE
                    STOP;

PROCESS (BYTE_START, BYTE_READ, TX_READ) BEGIN
   IF BYTE_START = '1' AND BYTE_READ = '1' THEN
      BYTE_START <= '0';
   ELSIF rising_edge(TX_READ) THEN
      BYTE_START <= '1';
      BYTE_IN <= TX_BYTE;
   END IF;
END PROCESS;
PROCESS (CLK_BAUD) BEGIN
   IF rising_edge(CLK_BAUD) THEN
      CASE BYTE_CURRENT_STATE IS
      WHEN READ =>
         IF BYTE_START = '1' THEN
            BYTE_READ <= '1';
            BYTE_OUT <= BYTE_IN;
            BYTE_CURRENT_STATE <= XMIT;
            TX_RST <= '0';
         ELSE
            BYTE_READ <= '0';
            TX_RST <= '1';
         END IF;
      WHEN XMIT =>
         BYTE_READ <= '0';
         IF TX_NEXT_STATE = STOP THEN
            BYTE_CURRENT_STATE <= READ;
            TX_RST <= '1';
         ELSE
            TX_RST <= '0';
         END IF;
      WHEN OTHERS =>
         BYTE_READ <= '0';
         TX_RST <= '1';
      END CASE;
   END IF;
END PROCESS;
PROCESS (TX_RST, CLK_BAUD) BEGIN
   IF TX_RST = '1' THEN
      TX_CURRENT_STATE <= START;
   ELSIF rising_edge(CLK_BAUD) THEN
      TX_CURRENT_STATE <= TX_NEXT_STATE;
   END IF;
END PROCESS;
END ARCH_UART_TX;
