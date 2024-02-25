--
--  Copyright (C) 2024, Vadim Godunko
--
--  SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
--

--  Communitation channel driver for STM32F407 MCU family. MCU must run at
--  168 MHz, otherwise prescaler and timings need to be modified, see package
--  body.
--
--  This driver uses SPI2, TIM7, EXTI11 peripherals, and installs
--  corresponding interrupt handlers.
--
--  Connection:
--    PS2 Controller        STM32F407
--      1 Data        -->   MISO PB14
--      2 Command     -->   MOSI PB15
--      6 Attention   -->  (NSS) PB12
--      7 Clock       -->   SCK  PB13
--      9 Acknoledge  -->  (ACK) PB11

package A0B.PlayStation2_Controllers.Async.STM32F407 is

   type STM32F407_Communication_Channel is
     limited new Communication.Abstract_Communication_Channel with private;

private

   type STM32F407_Communication_Channel is
     limited new Communication.Abstract_Communication_Channel with record
      null;
   end record;

   overriding procedure Initialize
     (Self   : in out STM32F407_Communication_Channel);
      --  Input  : not null access constant Communication.Buffer;
      --  Output : not null access Communication.Buffer);

   overriding procedure Finalize
     (Self : in out STM32F407_Communication_Channel);

   overriding procedure Exchange
     (Self   : in out STM32F407_Communication_Channel;
      Buffer : aliased in out Communication.Buffer;
      Status : aliased out Communication.Status);

end A0B.PlayStation2_Controllers.Async.STM32F407;