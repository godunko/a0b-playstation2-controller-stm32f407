--
--  Copyright (C) 2024, Vadim Godunko
--
--  SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
--

--  Driver of the PlayStation 2 Dual Shock controller.
--
--  This driver is intended to be used on STM32F407 MCU family. It uses
--  SPI1, TIM7, EXTI? peripherals, and installs corresponding interrupt
--  handlers.
--
--  Connection:
--    PS2 Controller       STM32F407
--      Data          -->   MISO PA6
--      Command       -->   MOSI PA7
--      Attention     -->  (NSS) PA4
--      Clock         -->   SCK  PA5
--      Acknoledge    -->  (ACK) PA3

with A0B.Types; use A0B.Types;

package Driver is

   procedure Initialize;

   type Unsigned_8_Array is array (Positive range <>) of Unsigned_8;

   --  type Acknowledge is record
   --     Acknowledge_Delay    : Natural;
   --     Acknowledge_Duration : Natural;
   --  end record;

   --  type Acknowledge_Array is array (Positive range <>) of Acknowledge;

   procedure Send
     (Buffer          : in out Unsigned_8_Array;
      Adaptive_Length : Boolean;
      Last            : out Natural);

end Driver;
