--
--  Copyright (C) 2024, Vadim Godunko
--
--  SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
--

--  It is expected that STM32F407 MCU is runnung at 168MHz.
--  SPI2 peripheral is on APB1 bus and clocked at 42 MHz.
--  TIM7 peripheral is on APB1 bus and clocked at 2*42 = 84 MHz.
--
--  Use of 128 prescaler for SPI result communication at 328.125 KBit/s,
--  and it should be less that 500 KBit/s.
--
--  Timer is used to delays after lowering of NSS and starting of
--  transmission (20 usec), as well as to detect communication timeout
--  (100 usec). Timer's prescaler value is fixed to 0, it means that it
--  counts at 84 MHz. Autoreload values are set to 1_680 (for 20 usec
--  delay), and to 8_400 (for 100 usec timeout).
--
--  XXX Reset of peripheral need to be removed, because some peripherals
--  can be used for other purposes too.
--
--  XXX EXTI interrputs handler takes interrupts for EXTI10 .. EXTI15. It is
--  need to be dispatched in some way to be able to use other EXTIs for other
--  purposes.

pragma Ada_2022;

with Ada.Interrupts.Names;

with A0B.ARMv7M.CMSIS;
with A0B.SVD.STM32F407.DBG;        use A0B.SVD.STM32F407.DBG;
with A0B.SVD.STM32F407.EXTI;       use A0B.SVD.STM32F407.EXTI;
with A0B.SVD.STM32F407.GPIO;       use A0B.SVD.STM32F407.GPIO;
with A0B.SVD.STM32F407.RCC;        use A0B.SVD.STM32F407.RCC;
with A0B.SVD.STM32F407.SPI;        use A0B.SVD.STM32F407.SPI;
with A0B.SVD.STM32F407.SYSCFG;     use A0B.SVD.STM32F407.SYSCFG;
with A0B.SVD.STM32F407.TIM;        use A0B.SVD.STM32F407.TIM;

--  with Ada.Text_IO;
--  with Ada.Real_Time;
--  with System.BB.Threads;

package body A0B.PlayStation2_Controllers.Async.STM32F407 is

   Prescale_TIM_PSC : constant := 0;
   Delay_TIM_ARR    : constant := 1_680;
   Timeout_TIM_ARR  : constant := 8_400;
   --  Values of timer's configuration, see description at the top of the file.

   EXTI_Id          : constant := 11;
   NSS_Id           : constant := 12;
   --  Pin numbers.

   Buffer    : access Communication.Buffer;
   Status    : access Communication.Status;
   Current   : A0B.Types.Unsigned_8 := 0;
   Remaining : A0B.Types.Unsigned_8 := 0;
   Done      : Ada.Synchronous_Task_Control.Suspension_Object;

   protected Handler is

      procedure EXTI_Handler
        with Attach_Handler => Ada.Interrupts.Names.EXTI15_10_Interrupt;

      procedure TIM_Handler
        with Attach_Handler => Ada.Interrupts.Names.TIM7_Interrupt;

      procedure SPI_Handler
        with Attach_Handler => Ada.Interrupts.Names.SPI2_Interrupt;

   end Handler;

   --------------
   -- Exchange --
   --------------

   overriding procedure Exchange
     (Self   : in out STM32F407_Communication_Channel;
      Buffer : aliased in out Communication.Buffer;
      Status : aliased out Communication.Status)
   is
      use type A0B.Types.Unsigned_8;

   begin
      STM32F407.Buffer    := Buffer'Unchecked_Access;
      STM32F407.Status    := Status'Unchecked_Access;
      STM32F407.Current   := 0;
      STM32F407.Remaining := 3;
      --  Minimum amount of data to be transmitted is three bytes. Actual
      --  amount is send by the controller in 4 least significant bits of the
      --  second byte.

      GPIOB_Periph.BSRR.BR.Arr (NSS_Id) := True;  --  Clear bit, NCS

      Ada.Synchronous_Task_Control.Set_False (Done);

      --  Start delay timer

      TIM7_Periph.ARR.ARR := Delay_TIM_ARR;
      TIM7_Periph.EGR.UG  := True;
      TIM7_Periph.CR1.CEN := True;

--  Ada.Text_IO.Put_Line (">>> Until True");
      Ada.Synchronous_Task_Control.Suspend_Until_True (Done);
--  Ada.Text_IO.Put_Line ("<<< Until True");

--        while SPI2_Periph.SR.BSY loop
--           null;
--        end loop;

      GPIOB_Periph.BSRR.BS.Arr (NSS_Id) := True;  --  Set bit, NCS

      Status.Last      := STM32F407.Current - 1;
      STM32F407.Buffer := null;
      STM32F407.Status := null;
   end Exchange;

   --------------
   -- Finalize --
   --------------

   overriding procedure Finalize
     (Self : in out STM32F407_Communication_Channel) is
   begin
      null;
   end Finalize;

   -------------
   -- Handler --
   -------------

   protected body Handler is

      ------------------
      -- EXTI_Handler --
      ------------------

      procedure EXTI_Handler is
      begin
         --  Pending state must be cleared by software. Also, disable
         --  interrupt, otherwise it might be triggered when number of bytes
         --  is less than expected by the controller.
         --
         --  XXX Is it wrong expectation and actual length of the data should
         --  be computed from the second received byte?

         EXTI_Periph.PR.PR.Arr (EXTI_Id)  := True;
         EXTI_Periph.IMR.MR.Arr (EXTI_Id) := False;

         --  Disable timeout.

         TIM7_Periph.CR1.CEN := False;

         --  Enable TXE interrupt to initiate transfer of the next byte.

         SPI2_Periph.CR2.TXEIE := True;
      end EXTI_Handler;

      -----------------
      -- SPI_Handler --
      -----------------

      procedure SPI_Handler is
         use type A0B.Types.Unsigned_8;

      begin
         if SPI2_Periph.CR2.TXEIE
           and then SPI2_Periph.SR.TXE
         then
            --  Initiate transfer of the byte.

            --  Current           := @ + 1;
            Remaining         := @ - 1;
            SPI2_Periph.DR.DR := DR_DR_Field (Buffer (Current));

            --  Disable TXE and enable RXNE interrupt.

            SPI2_Periph.CR2.TXEIE  := False;
            SPI2_Periph.CR2.RXNEIE := True;
         end if;

         if SPI2_Periph.SR.RXNE then
            --  Byte has been received, store it in the buffer.

            Buffer (Current) :=
              A0B.Types.Unsigned_8 (SPI2_Periph.DR.DR);
            Current          := @ + 1;

            --  Disable SPI.RXNE interrupt

            SPI2_Periph.CR2.RXNEIE := False;

            --  Update amount of remaining bytes after receive of the
            --  3 bytes header.

            if Current = 3 then
               Remaining := Payload_Length (Buffer.all) * 2;
               --  Payload length is specified in 2-byte words.
            end if;

            if Remaining /= 0 then
               --  Enable interrupt on raising edge of the Acknowledge signal

               EXTI_Periph.PR.PR.Arr (EXTI_Id)  := True;
               EXTI_Periph.IMR.MR.Arr (EXTI_Id) := True;

               --  Configure timer for timeout interval and start it.

               TIM7_Periph.ARR.ARR := Timeout_TIM_ARR;
               TIM7_Periph.EGR.UG  := True;
               TIM7_Periph.CR1.CEN := True;

            else
               --  Last byte has been processed.

               Status.No_Acknowledge := False;
               Ada.Synchronous_Task_Control.Set_True (Done);
            end if;
         end if;
      end SPI_Handler;

      -----------------
      -- TIM_Handler --
      -----------------

      procedure TIM_Handler is
         use type A0B.Types.Unsigned_16;

      begin
         TIM7_Periph.SR.UIF := False;
         --  UIF flag should be reset by software.

         if TIM7_Periph.ARR.ARR = Delay_TIM_ARR then
            --  "Attention" delay is done, enable SPI.TXE interrupt to start
            --  transfer of the first byte.

            SPI2_Periph.CR2.TXEIE := True;

         elsif TIM7_Periph.ARR.ARR = Timeout_TIM_ARR then
            --  Acknoledge timeout, disable EXTI.

            EXTI_Periph.IMR.MR.Arr (EXTI_Id) := False;

            Status.No_Acknowledge := True;
            Ada.Synchronous_Task_Control.Set_True (Done);

         else
            raise Program_Error;
         end if;
      end TIM_Handler;

   end Handler;

   ----------------
   -- Initialize --
   ----------------

   overriding procedure Initialize
     (Self : in out STM32F407_Communication_Channel)
--        Input  : not null access constant Communication.Buffer;
--        Output : not null access Communication.Buffer);

   is

      procedure Enable_Peripheral_Clocks;
      --  Enables clocks of GPIOB, SYSCFG, SPI2 and TIM7 peripherals.

      procedure Reset_Peripherals;
      --  Reset GPIOB, SYSCFG, SPI2 and TIM7 periferals.

      procedure Configure_PIO;
      procedure Configure_SPI;
      procedure Configure_TIM;
      procedure Configure_EXTI;
      --  Configure peripherals.

      --------------------
      -- Configure_EXTI --
      --------------------

      procedure Configure_EXTI is
      begin
         SYSCFG_Periph.EXTICR3.EXTI.Arr (EXTI_Id) := 2#0001#;
         --  Select the source input for the EXTI3 - PB11

         EXTI_Periph.RTSR.TR.Arr (EXTI_Id) := True;
         --  Rising trigger enabled (for Event and Interrupt) for input line
         EXTI_Periph.FTSR.TR.Arr (EXTI_Id) := False;
         --  Falling trigger disabled (for Event and Interrupt) for input line
      end Configure_EXTI;

      -------------------
      -- Configure_PIO --
      -------------------

      procedure Configure_PIO is
      begin
         --  PB11, ACK

         GPIOB_Periph.MODER.Arr (EXTI_Id)   := 2#00#;
         --  General purpose input mode
         GPIOB_Periph.OSPEEDR.Arr (EXTI_Id) := 2#11#;  --  Very high speed
         GPIOB_Periph.PUPDR.Arr (EXTI_Id)   := 2#01#;  --  Pull up

         --  PB12, NSS

         GPIOB_Periph.MODER.Arr (NSS_Id)   := 2#01#;
         --  General purpose output mode
         GPIOB_Periph.OSPEEDR.Arr (NSS_Id) := 2#11#;
         GPIOB_Periph.BSRR.BS.Arr (NSS_Id) := True;   --  Set NSS

         --  PB13, SCK

         GPIOB_Periph.MODER.Arr (13)   := 2#10#;    --  Alternate function mode
         GPIOB_Periph.OSPEEDR.Arr (13) := 2#11#;    --  Very high speed
         GPIOB_Periph.AFRH.Arr (13)    := 2#0101#;  --  Alternate function 5

         --  PB14, MISO

         GPIOB_Periph.MODER.Arr (14)   := 2#10#;    --  Alternate function mode
         GPIOB_Periph.OSPEEDR.Arr (14) := 2#11#;    --  Very high speed
         GPIOB_Periph.AFRH.Arr (14)    := 2#0101#;  --  Alternate function 5
         GPIOB_Periph.PUPDR.Arr (14)   := 2#01#;    --  Pull up

         --  PB15, MOSI

         GPIOB_Periph.MODER.Arr (15)   := 2#10#;    --  Alternate function mode
         GPIOB_Periph.OSPEEDR.Arr (15) := 2#11#;    --  Very high speed
         GPIOB_Periph.AFRH.Arr (15)    := 2#0101#;  --  Alternate function 5
      end Configure_PIO;

      -------------------
      -- Configure_SPI --
      -------------------

      procedure Configure_SPI is
      begin
         SPI2_Periph.CR1.SPE := False;

         SPI2_Periph.CR1 :=
           (CPHA     => True,
            --  The second clock transition is the first data capture edge
            CPOL     => True,   --  CK to 1 when idle
            MSTR     => True,
            BR       => 7,      --  fPCLK/126
            SPE      => False,
            LSBFIRST => True,   --  LSB transmitted first
            SSI      => True,
            SSM      => False,  --  Software slave management disabled
            RXONLY   => False,
            DFF      => False,
            CRCNEXT  => False,
            CRCEN    => False,  --  CRC calculation disabled
            BIDIOE   => False,
            BIDIMODE => False,
            others   => <>);

         SPI2_Periph.CR2 :=
           (RXDMAEN => False,
            TXDMAEN => False,
            SSOE    => True,
            --  SS output is enabled in master mode and when the cell is
            --  enabled.
            FRF     => False,  --  SPI Motorola mode
            ERRIE   => False,
            RXNEIE  => False,
            TXEIE   => False,
            others  => <>);

         SPI2_Periph.I2SCFGR.I2SMOD := False;

         SPI2_Periph.CR1.SPE := True;
      end Configure_SPI;

      -------------------
      -- Configure_TIM --
      -------------------

      procedure Configure_TIM is
      begin
         TIM7_Periph.CR1 :=
           (CEN    => False,  --  Counter disabled
            UDIS   => False,  --  UEV enabled
            URS    => True,
            --  Only counter overflow/underflow generates an update interrupt
            --  or DMA request if enabled.
            OPM    => True,
            --  Counter stops counting at the next update event (clearing the
            --  CEN bit).
            ARPE   => False,
            others => <>);

         TIM7_Periph.PSC :=
           (PSC    => Prescale_TIM_PSC,
            others => <>);

         TIM7_Periph.DIER :=
           (UIE    => True,   --  Update interrupt enabled.
            UDE    => False,  --  Update DMA request disabled.
            others => <>);

         --  Stop timer on debug breakpoint

         DBG_Periph.DBGMCU_APB1_FZ.DBG_TIM7_STOP := True;
      end Configure_TIM;

      ------------------------------
      -- Enable_Peripheral_Clocks --
      ------------------------------

      procedure Enable_Peripheral_Clocks is
      begin
         RCC_Periph.AHB1ENR.GPIOBEN  := True;
         RCC_Periph.APB2ENR.SYSCFGEN := True;
         RCC_Periph.APB1ENR.SPI2EN   := True;
         RCC_Periph.APB1ENR.TIM7EN   := True;
         A0B.ARMv7M.CMSIS.Data_Synchronization_Barrier;
         --  Wait till completion of all writes, see STM32F407 Errata sheet
         --  2.2.13 Delay after an RCC peripheral clock enabling
      end Enable_Peripheral_Clocks;

      -----------------------
      -- Reset_Peripherals --
      -----------------------

      procedure Reset_Peripherals is
      begin
         --  Set reset bit.

         RCC_Periph.AHB1RSTR.GPIOBRST  := True;
         RCC_Periph.APB2RSTR.SYSCFGRST := True;
         RCC_Periph.APB1RSTR.SPI2RST   := True;
         RCC_Periph.APB1RSTR.TIM7RST   := True;
         A0B.ARMv7M.CMSIS.Data_Synchronization_Barrier;

         --  Clear reset bit.

         RCC_Periph.AHB1RSTR.GPIOBRST  := False;
         RCC_Periph.APB2RSTR.SYSCFGRST := False;
         RCC_Periph.APB1RSTR.SPI2RST   := False;
         RCC_Periph.APB1RSTR.TIM7RST   := False;
         A0B.ARMv7M.CMSIS.Data_Synchronization_Barrier;
      end Reset_Peripherals;

   begin
      --  Enable peripheral clocks.

      Enable_Peripheral_Clocks;

      --  Reset peripherals. It is useful for debugging.

      Reset_Peripherals;

      --  Configure peripherals.

      Configure_SPI;
      Configure_PIO;
      Configure_TIM;
      Configure_EXTI;
   end Initialize;

end A0B.PlayStation2_Controllers.Async.STM32F407;