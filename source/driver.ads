--
--  Copyright (C) 2024, Vadim Godunko
--
--  SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
--

--  Driver of the PlayStation 2 Dual Shock controller.
--
--  This driver is intended to be used on STM32F407 MCU family. It uses
--  SPI2, TIM7, EXTI11 peripherals, and installs corresponding interrupt
--  handlers.
--
--  Connection:
--    PS2 Controller        STM32F407
--      Data          -->   MISO PB14
--      Command       -->   MOSI PB15
--      Attention     -->  (NSS) PB12
--      Clock         -->   SCK  PB13
--      Acknoledge    -->  (ACK) PB11

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
