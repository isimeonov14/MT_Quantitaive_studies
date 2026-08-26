//
// Copyright (c) 2010-2026 Antmicro
//
// This file is licensed under the MIT License.
// Full license text is available in 'licenses/MIT.txt'.
//

using System;
using System.Collections.Generic;
using System.Linq;

using Antmicro.Renode.Core;
using Antmicro.Renode.Core.Structure;
using Antmicro.Renode.Core.Structure.Registers;
using Antmicro.Renode.Logging;
using Antmicro.Renode.Peripherals.Bus;
using Antmicro.Renode.Peripherals.Miscellaneous;

namespace Antmicro.Renode.Peripherals.SPI
{
    public class NRF5340_SPI_CS : NullRegistrationPointPeripheralContainer<ISPIPeripheral>, IDoubleWordPeripheral, IProvidesRegisterCollection<DoubleWordRegisterCollection>, IKnownSize, INRFEventProvider
    {
        public NRF5340_SPI_CS(IMachine machine, bool easyDMA = false) : base(machine)
        {
            this.machine = machine;
            sysbus = machine.GetSystemBus(this);
            this.easyDMA = easyDMA;

            IRQ = new GPIO();

            receiveFifo = new Queue<byte>();
            RegistersCollection = new DoubleWordRegisterCollection(this);
            DefineRegisters();
            Reset();
        }

        public override void Reset()
        {
            receiveFifo.Clear();
            enabled = false;
            transactionInProgress = false;
            RegistersCollection.Reset();
            UpdateInterrupts();
        }

        public uint ReadDoubleWord(long offset)
        {
            return RegistersCollection.Read(offset);
        }

        public void WriteDoubleWord(long offset, uint value)
        {
            RegistersCollection.Write(offset, value);
        }

        // When true, the attached SPI slave is expected to finalize transactions
        // on an external chip-select GPIO edge rather than on TASKS_STOP / disable.
        public bool ExternalCSControlsTransactionBoundary { get; set; } = true;


        public GPIO IRQ { get; }

        public DoubleWordRegisterCollection RegistersCollection { get; }

        public long Size => 0x1000;

        public event Action<uint> EventTriggered;

        private void UpdateInterrupts()
        {
            var status = false;

            if(easyDMA)
            {
                status |= stoppedPending.Value && stoppedEnabled.Value;
                status |= startedPending.Value && startedEnabled.Value;
                status |= endPending.Value && endEnabled.Value;
                status |= endRxPending.Value && endRxEnabled.Value;
                status |= endTxPending.Value && endTxEnabled.Value;
            }
            else
            {
                status |= readyEnabled.Value && readyPending.Value;
            }
            status &= enabled;

            this.Log(LogLevel.Noisy, "Setting IRQ to {0}", status);
            IRQ.Set(status);
        }

        private void DefineRegisters()
        {
            Registers.EventsReady.Define(this)
                .WithFlag(0, out readyPending, name: "EVENTS_READY")
                .WithReservedBits(1, 31)
                .WithWriteCallback((_, __) => UpdateInterrupts())
            ;

            if(easyDMA)
            {
                Registers.EnableInterrupt.Define(this)
                    .WithReservedBits(0, 1)
                    .WithFlag(1, out stoppedEnabled, FieldMode.Read | FieldMode.Set, name: "STOPPED")
                    .WithReservedBits(2, 2)
                    .WithFlag(4, out endRxEnabled, FieldMode.Read | FieldMode.Set, name: "ENDRX")
                    .WithReservedBits(5, 1)
                    .WithFlag(6, out endEnabled, FieldMode.Read | FieldMode.Set, name: "END")
                    .WithReservedBits(7, 1)
                    .WithFlag(8, out endTxEnabled, FieldMode.Read | FieldMode.Set, name: "ENDTX")
                    .WithReservedBits(9, 10)
                    .WithFlag(19, out startedEnabled, FieldMode.Read | FieldMode.Set, name: "STARTED")
                    .WithReservedBits(20, 12)
                    .WithWriteCallback((_, __) => UpdateInterrupts())
                ;

                Registers.DisableInterrupt.Define(this)
                    .WithReservedBits(0, 1)
                    .WithFlag(1, name: "STOPPED",
                        valueProviderCallback: _ => stoppedEnabled.Value,
                        writeCallback: (_, val) => { if(val) stoppedEnabled.Value = false; })
                    .WithReservedBits(2, 2)
                    .WithFlag(4, name: "ENDRX",
                        valueProviderCallback: _ => endRxEnabled.Value,
                        writeCallback: (_, val) => { if(val) endRxEnabled.Value = false; })
                    .WithReservedBits(5, 1)
                    .WithFlag(6, name: "END",
                        valueProviderCallback: _ => endEnabled.Value,
                        writeCallback: (_, val) => { if(val) endEnabled.Value = false; })
                    .WithReservedBits(7, 1)
                    .WithFlag(8, name: "ENDTX",
                        valueProviderCallback: _ => endTxEnabled.Value,
                        writeCallback: (_, val) => { if(val) endTxEnabled.Value = false; })
                    .WithReservedBits(9, 10)
                    .WithFlag(19, name: "STARTED",
                        valueProviderCallback: _ => startedEnabled.Value,
                        writeCallback: (_, val) => { if(val) startedEnabled.Value = false; })
                    .WithReservedBits(20, 12)
                    .WithWriteCallback((_, __) => UpdateInterrupts())
                ;

                Registers.TxListType.Define(this)
                    .WithTaggedFlag("LIST - List type", 0)
                    .WithReservedBits(1, 31)
                ;

                Registers.RxListType.Define(this)
                    .WithTaggedFlag("LIST - List type", 0)
                    .WithReservedBits(1, 31)
                ;
            }
            else
            {
                Registers.EnableInterrupt.Define(this)
                    .WithReservedBits(0, 2)
                    .WithFlag(2, out readyEnabled, FieldMode.Read | FieldMode.Set, name: "READY")
                    .WithReservedBits(3, 3)
                    .WithFlag(6, out endEnabled, FieldMode.Read | FieldMode.Set, name: "END")
                    .WithReservedBits(7, 25)
                    .WithWriteCallback((_, __) => UpdateInterrupts())
                ;

                Registers.DisableInterrupt.Define(this)
                    .WithReservedBits(0, 2)
                    .WithFlag(2, name: "READY",
                        valueProviderCallback: _ => readyEnabled.Value,
                        writeCallback: (_, val) => { if(val) readyEnabled.Value = false; })
                    .WithReservedBits(3, 3)
                    .WithFlag(6, name: "END",
                        writeCallback: (_, val) => { if(val) endEnabled.Value = false; })
                    .WithReservedBits(7, 25)
                    .WithWriteCallback((_, __) => UpdateInterrupts())
                ;
            }

            Registers.Enable.Define(this)
                .WithValueField(0, 4,
                    valueProviderCallback: _ => enabled ? 1 : 0u,
                    writeCallback: (_, val) =>
                    {
                        switch(val)
                        {
                        case 0:
                            // disabled
                            if(!ExternalCSControlsTransactionBoundary)
                            {
                                FinishCurrentTransaction();
                            }
                            enabled = false;
                            break;

                        case 1:
                            // enabled, standard mode
                            if(!easyDMA)
                            {
                                enabled = true;
                            }
                            break;

                        case 7:
                            // enabled, easyDMA mode
                            if(easyDMA)
                            {
                                enabled = true;
                            }
                            break;

                        default:
                            this.Log(LogLevel.Warning, "Unhandled enable value: 0x{0:X}", val);
                            return;
                        }

                        UpdateInterrupts();
                    })
                .WithReservedBits(4, 28)
            ;

            if(easyDMA)
            {
                Registers.PinSelectSCK.Define(this)
                    .WithValueField(0, 32, out pinSelectSCK, name: "PSEL.SCK");

                Registers.PinSelectMOSI.Define(this)
                    .WithValueField(0, 32, out pinSelectMOSI, name: "PSEL.MOSI");

                Registers.PinSelectMISO.Define(this)
                    .WithValueField(0, 32, out pinSelectMISO, name: "PSEL.MISO");

                Registers.Frequency.Define(this)
                    .WithValueField(0, 32, out frequency, name: "FREQUENCY");

                Registers.Configuration.Define(this)
                    .WithValueField(0, 32, out configuration, name: "CONFIG");

                Registers.Configuration2.Define(this)
                    .WithValueField(0, 32, out configuration2, name: "CONFIG2");
            }

            Registers.TransmitBuffer.Define(this)
                // the documentation says this field is readable, so we use the automatic
                // underlying backing field to return the previously written value
                .WithValueField(0, 8, name: "TXD", writeCallback: (_, val) => SendData((byte)val))
                .WithReservedBits(8, 24)
            ;

            Registers.ReceiveBuffer.Define(this)
                .WithValueField(0, 8, FieldMode.Read, name: "RXD",
                    valueProviderCallback: _ =>
                    {
                        lock(receiveFifo)
                        {
                            if(receiveFifo.Count == 0)
                            {
                                this.Log(LogLevel.Warning, "Tried to read from an empty buffer");
                                return 0;
                            }

                            var result = receiveFifo.Dequeue();

                            // some new byte moved to the head
                            // of the queue - let's generate the
                            // READY event
                            if(receiveFifo.Count > 0)
                            {
                                readyPending.Value = true;
                                EventTriggered?.Invoke((uint)Registers.EventsReady);
                                UpdateInterrupts();
                            }
                            return result;
                        }
                    })
                .WithReservedBits(8, 24)
            ;

            if(easyDMA)
            {
                Registers.TasksStart.Define(this)
                    .WithFlag(0, FieldMode.Write, name: "TASKS_START", writeCallback: (_, val) =>
                    {
                        if(val)
                        {
                            transactionInProgress = true;
                            machine.LocalTimeSource.ExecuteInNearestSyncedState(__ => ExecuteTransaction());
                        }
                    })
                    .WithReservedBits(1, 31)
                ;

                Registers.TasksStop.Define(this)
                    .WithFlag(0, FieldMode.Write, name: "TASKS_STOP", writeCallback: (_, val) =>
                    {
                        if(val)
                        {
                            if(!ExternalCSControlsTransactionBoundary)
                            {
                                FinishCurrentTransaction();
                            }
                            stoppedPending.Value = true;
                            EventTriggered?.Invoke((uint)Registers.EventsStopped);
                            UpdateInterrupts();
                        }
                    })
                    .WithReservedBits(1, 31)
                ;

                Registers.EventsStopped.Define(this)
                    .WithFlag(0, out stoppedPending, name: "EVENTS_STOPPED")
                    .WithReservedBits(1, 31)
                    .WithWriteCallback((_, __) => UpdateInterrupts())
                ;

                Registers.EventsStarted.Define(this)
                    .WithFlag(0, out startedPending, name: "EVENTS_STARTED")
                    .WithReservedBits(1, 31)
                    .WithWriteCallback((_, __) => UpdateInterrupts())
                ;

                Registers.EventsEnd.Define(this)
                    .WithFlag(0, out endPending, name: "EVENTS_END")
                    .WithReservedBits(1, 31)
                    .WithWriteCallback((_, __) => UpdateInterrupts())
                ;

                Registers.EventsEndTx.Define(this)
                    .WithFlag(0, out endTxPending, name: "EVENTS_ENDTX")
                    .WithReservedBits(1, 31)
                    .WithWriteCallback((_, __) => UpdateInterrupts())
                ;

                Registers.EventsEndRx.Define(this)
                    .WithFlag(0, out endRxPending, name: "EVENTS_ENDRX")
                    .WithReservedBits(1, 31)
                    .WithWriteCallback((_, __) => UpdateInterrupts())
                ;

                Registers.RxDataPointer.Define(this)
                    .WithValueField(0, 32, out rxDataPointer, name: "PTR")
                ;

                Registers.RxMaxDataCount.Define(this)
                    .WithValueField(0, 16, out rxMaxDataCount, name: "MAXCNT")
                    .WithReservedBits(16, 16)
                ;

                Registers.RxTransferredDataAmount.Define(this)
                    .WithValueField(0, 16, out rxTransferredDataAmount, FieldMode.Read, name: "AMOUNT")
                    .WithReservedBits(16, 16)
                ;

                Registers.TxDataPointer.Define(this)
                    .WithValueField(0, 32, out txDataPointer, name: "PTR")
                ;

                Registers.TxMaxDataCount.Define(this)
                    .WithValueField(0, 16, out txMaxDataCount, name: "MAXCNT")
                    .WithReservedBits(16, 16)
                ;

                Registers.TxTransferredDataAmount.Define(this)
                    .WithValueField(0, 16, out txTransferredDataAmount, FieldMode.Read, name: "AMOUNT")
                    .WithReservedBits(16, 16)
                ;

                Registers.ORC.Define(this)
                    .WithValueField(0, 8, out orcByte, name: "ORC")
                    .WithReservedBits(8, 24)
                ;
            }
        }

        private void ExecuteTransaction()
        {
            var rxLength = (int)rxMaxDataCount.Value;
            var txLength = (int)txMaxDataCount.Value;
            var receivedBytes = new byte[rxLength];

            startedPending.Value = true;
            EventTriggered?.Invoke((uint)Registers.EventsStarted);

            if(RegisteredPeripheral == null)
            {
                this.Log(LogLevel.Warning, "Issued a transaction, but no device is connected - returning dummy bytes");
            }
            else
            {
                this.Log(LogLevel.Debug,
                    "Starting SPI EasyDMA transaction: TX.PTR=0x{0:X8}, TX.MAXCNT={1}, RX.PTR=0x{2:X8}, RX.MAXCNT={3}, ORC=0x{4:X2}",
                    txDataPointer.Value,
                    txLength,
                    rxDataPointer.Value,
                    rxLength,
                    orcByte.Value);


                var bytesToSend = sysbus.ReadBytes(txDataPointer.Value, txLength);
                if(rxLength > txLength)
                {
                    bytesToSend = bytesToSend.Concat(Enumerable.Repeat((byte)orcByte.Value, rxLength - txLength)).ToArray();
                }

                var counter = 0;
                foreach(var b in bytesToSend)
                {
                    var result = RegisteredPeripheral.Transmit(b);
                    if(counter < rxLength)
                    {
                        receivedBytes[counter] = result;
                    }
                    counter++;
                }
            }

            if(rxLength > 0)
            {
                sysbus.WriteBytes(receivedBytes, rxDataPointer.Value);
            }

            txTransferredDataAmount.Value = (ulong)txLength;
            rxTransferredDataAmount.Value = (ulong)rxLength;

            endTxPending.Value = true;
            EventTriggered?.Invoke((uint)Registers.EventsEndTx);
            endRxPending.Value = true;
            EventTriggered?.Invoke((uint)Registers.EventsEndRx);
            endPending.Value = true;
            EventTriggered?.Invoke((uint)Registers.EventsEnd);
            UpdateInterrupts();
        }

        private void FinishCurrentTransaction()
        {
            if(!transactionInProgress)
            {
                return;
            }

            RegisteredPeripheral?.FinishTransmission();
            transactionInProgress = false;
        }

        private void SendData(byte b)
        {
            if(!enabled)
            {
                this.Log(LogLevel.Warning, "Trying to send data, but the controller is disabled");
                return;
            }

            if(RegisteredPeripheral == null)
            {
                this.Log(LogLevel.Warning, "No device connected");
                return;
            }

            lock(receiveFifo)
            {
                if(receiveFifo.Count == ReceiveBufferSize)
                {
                    this.Log(LogLevel.Warning, "Buffers full, ignoring data");
                    return;
                }

                // there is no need to queue transmitted bytes - let's send them right away
                var result = RegisteredPeripheral.Transmit(b);

                receiveFifo.Enqueue(result);

                // the READY event is generated
                // only when the head
                // of the queue changes
                if(receiveFifo.Count == 1)
                {
                    readyPending.Value = true;
                    EventTriggered?.Invoke((uint)Registers.EventsReady);
                    UpdateInterrupts();
                }
            }
        }

        private bool enabled;
        private bool transactionInProgress;

        private IValueRegisterField orcByte;
        private IValueRegisterField pinSelectSCK;
        private IValueRegisterField pinSelectMOSI;
        private IValueRegisterField pinSelectMISO;
        private IValueRegisterField frequency;
        private IValueRegisterField configuration;
        private IValueRegisterField configuration2;
        private IValueRegisterField rxTransferredDataAmount;
        private IValueRegisterField txTransferredDataAmount;
        private IValueRegisterField rxMaxDataCount;
        private IValueRegisterField txMaxDataCount;
        private IValueRegisterField rxDataPointer;
        private IFlagRegisterField endRxPending;

        private IValueRegisterField txDataPointer;
        private IFlagRegisterField endTxPending;

        private IFlagRegisterField endTxEnabled;
        private IFlagRegisterField endPending;

        private IFlagRegisterField endEnabled;
        private IFlagRegisterField readyEnabled;

        private IFlagRegisterField readyPending;
        private IFlagRegisterField startedEnabled;

        private IFlagRegisterField startedPending;
        private IFlagRegisterField stoppedEnabled;
        private IFlagRegisterField stoppedPending;

        private IFlagRegisterField endRxEnabled;
        private readonly IBusController sysbus;

        private readonly Queue<byte> receiveFifo;
        private readonly IMachine machine;
        private readonly bool easyDMA;

        // RXD is double buffered
        private const int ReceiveBufferSize = 2;

        private enum Registers
        {
            TasksStart = 0x10,
            TasksStop = 0x14,
            EventsStopped = 0x104,
            EventsReady = 0x108,
            EventsEndRx = 0x110,
            EventsEnd = 0x118,
            EventsEndTx = 0x120,
            EventsStarted = 0x14C,
            EnableInterrupt = 0x304,
            DisableInterrupt = 0x308,
            Enable = 0x500,
            PinSelectSCK = 0x508,
            PinSelectMOSI = 0x50C,
            PinSelectMISO = 0x510,
            ReceiveBuffer = 0x518,
            TransmitBuffer = 0x51C,
            Frequency = 0x524,
            RxDataPointer = 0x534,
            RxMaxDataCount = 0x538,
            RxTransferredDataAmount = 0x53C,
            RxListType = 0x540,
            TxDataPointer = 0x544,
            TxMaxDataCount = 0x548,
            TxTransferredDataAmount = 0x54C,
            TxListType = 0x550,
            Configuration = 0x554,
            Configuration2 = 0x560,
            ORC = 0x5C0,
        }
    }
}