//
// Copyright (c) 2024-2026 Skaaltec S.r.l.
//
// All rights reserved. This file is part of the SmartVNS firmware test
// infrastructure and is provided under a proprietary license.
// Unauthorized copying, modification, distribution, or use is
// strictly prohibited.
//

//
// High-coverage Renode model for the ST LSM6DSV16BX IMU.
//
// This model follows the structure of the provided LSM6DSO example, but is adapted to
// the LSM6DSV16BX register map and feature set.
//
// Scope:
// - SPI register interface with GPIO-driven chip select
// - Main-page register map and embedded-function register bank access
// - Accelerometer, gyroscope, temperature, Qvar, timestamp and FIFO data paths
// - FIFO modes, watermark/full/overrun flags, INT1/INT2 routing, BDU-like static output behavior
// - TDM/Qvar/embedded-function configuration register storage
// - RESD feeding for acceleration / angular rate / temperature
//
// Intentionally simplified:
// - No electrical I2C / I3C / TDM serial bus implementation
// - No full MLC/FSM/SFLP algorithm emulation; related registers are exposed/stored
// - Triggered FIFO modes are approximated to their closest streaming behavior
//

using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;

using Antmicro.Renode.Core;
using Antmicro.Renode.Logging;
using Antmicro.Renode.Peripherals.Sensor;
using Antmicro.Renode.Peripherals.SPI;
using Antmicro.Renode.Time;
using Antmicro.Renode.Utilities.RESD;

namespace Antmicro.Renode.Peripherals.Sensors
{
    public class LSM6DSV16BX_SmartVNS : ISPIPeripheral, IGPIOReceiver, ITemperatureSensor, IUnderstandRESD
    {
        public LSM6DSV16BX_SmartVNS(IMachine machine)
        {
            this.machine = machine;

            Interrupt1 = new GPIO();
            Interrupt2 = new GPIO();

            mainRegisters = new byte[RegisterSpaceSize];
            embeddedRegisters = new byte[RegisterSpaceSize];
            pageMemory = new byte[PageMemorySize];
            fifoQueue = new Queue<FifoFrame>();

            Reset();
        }

        public void Reset()
        {
            StopAllStreamsAndThreads();

            defaultAccelerationX = 0m;
            defaultAccelerationY = 0m;
            defaultAccelerationZ = 1m;
            defaultAngularRateX = 0m;
            defaultAngularRateY = 0m;
            defaultAngularRateZ = 0m;
            deterministicAccelerationSequence = deterministicSequenceStart;

            ResetRegistersAndState(preservePinCtrlAndIfCfg: false, preserveChipSelectState: false);

            currentAccelerationSample = new Vector3Sample(defaultAccelerationX, defaultAccelerationY, defaultAccelerationZ);
            currentGyroscopeSample = new Vector3Sample(defaultAngularRateX, defaultAngularRateY, defaultAngularRateZ);
            RestartDefaultFeedersIfNeeded();
        }

        public bool ChipSelectActiveLow { get; set; } = true;

        public byte DeselectedTransmitValue { get; set; } = 0xFF;

        public bool RandomizeDefaultSamples { get; set; }

        public decimal RandomSampleFullScaleFraction { get; set; } = 0.8m;

        public int RandomSeed
        {
            get => randomSeed;
            set
            {
                randomSeed = value;
                lock(randomLock)
                {
                    randomGenerator = new Random(randomSeed);
                }
            }
        }

        public bool DeterministicSequenceSamples { get; set; }

        public ushort DeterministicSequenceStart
        {
            get => deterministicSequenceStart;
            set
            {
                deterministicSequenceStart = value;
                deterministicAccelerationSequence = value;
            }
        }

        public ushort DeterministicSequenceStep { get; set; } = 1;

        /// <summary>
        /// Load application-facing raw IMU samples from a CSV file and replay
        /// them through the modeled FIFO at the sensor's configured internal
        /// rate. For a 120 Hz source trace and a 480 Hz sensor rate, each
        /// source sample is held for four sensor ticks. This lets the
        /// production firmware's normal 4:1 FIFO averaging reconstruct the
        /// original 120 Hz sequence.
        ///
        /// Expected CSV columns:
        /// sys_time,gyro_x,gyro_y,gyro_z,acc_x,acc_y,acc_z,movement_events
        /// </summary>
        public void LoadRawImuCsv(string path, uint sourceRateHz = 120, bool loop = false)
        {
            if(sourceRateHz == 0)
            {
                throw new ArgumentException("sourceRateHz must be greater than zero.", nameof(sourceRateHz));
            }

            var lines = File.ReadAllLines(path);
            if(lines.Length < 2)
            {
                throw new ArgumentException("Raw IMU CSV does not contain data rows.", nameof(path));
            }

            var header = lines[0].Split(',');
            var columnIndex = new Dictionary<string, int>(StringComparer.OrdinalIgnoreCase);
            for(var i = 0; i < header.Length; i++)
            {
                columnIndex[header[i].Trim()] = i;
            }

            var required = new[]
            {
                "gyro_x", "gyro_y", "gyro_z",
                "acc_x", "acc_y", "acc_z"
            };

            foreach(var name in required)
            {
                if(!columnIndex.ContainsKey(name))
                {
                    throw new ArgumentException($"Raw IMU CSV is missing required column '{name}'.", nameof(path));
                }
            }

            var samples = new List<RawImuReplaySample>(lines.Length - 1);
            for(var lineNumber = 1; lineNumber < lines.Length; lineNumber++)
            {
                var line = lines[lineNumber].Trim();
                if(string.IsNullOrWhiteSpace(line))
                {
                    continue;
                }

                var fields = line.Split(',');
                try
                {
                    var sample = new RawImuReplaySample(
                        ParseInt16(fields, columnIndex["gyro_x"]),
                        ParseInt16(fields, columnIndex["gyro_y"]),
                        ParseInt16(fields, columnIndex["gyro_z"]),
                        ParseInt16(fields, columnIndex["acc_x"]),
                        ParseInt16(fields, columnIndex["acc_y"]),
                        ParseInt16(fields, columnIndex["acc_z"]),
                        columnIndex.ContainsKey("movement_events")
                            ? ParseByte(fields, columnIndex["movement_events"])
                            : (byte)0
                    );
                    samples.Add(sample);
                }
                catch(Exception e)
                {
                    throw new FormatException($"Failed to parse raw IMU CSV line {lineNumber + 1}: {line}", e);
                }
            }

            if(samples.Count == 0)
            {
                throw new ArgumentException("Raw IMU CSV contains no valid samples.", nameof(path));
            }

            rawImuReplaySamples = samples.ToArray();
            rawImuReplaySourceRateHz = sourceRateHz;
            rawImuReplayLoop = loop;
            rawImuReplayIndex = 0;
            rawImuReplayHoldCounter = 0;
            rawImuReplayFinishedLogged = false;

            this.Log(LogLevel.Info,
                "Loaded {0} raw IMU samples from '{1}' at source rate {2} Hz (loop={3}).",
                rawImuReplaySamples.Length, path, rawImuReplaySourceRateHz, rawImuReplayLoop);

            RestartDefaultFeedersIfNeeded();
        }

        public void StopRawImuReplay()
        {
            rawImuReplaySamples = null;
            rawImuReplayIndex = 0;
            rawImuReplayHoldCounter = 0;
            rawImuReplayFinishedLogged = false;

            rawImuReplayFeederThread?.Stop();
            rawImuReplayFeederThread = null;

            RestartDefaultFeedersIfNeeded();
        }

        public int RawImuReplaySampleCount => rawImuReplaySamples == null ? 0 : rawImuReplaySamples.Length;

        public int RawImuReplaySampleIndex => rawImuReplayIndex;

        private static short ParseInt16(string[] fields, int index)
        {
            return short.Parse(fields[index].Trim(), NumberStyles.Integer, CultureInfo.InvariantCulture);
        }

        private static byte ParseByte(string[] fields, int index)
        {
            return byte.Parse(fields[index].Trim(), NumberStyles.Integer, CultureInfo.InvariantCulture);
        }

        public GPIO Interrupt1 { get; }

        public GPIO Interrupt2 { get; }

        public decimal Temperature { get; set; }

        public decimal DefaultAccelerationX
        {
            get => defaultAccelerationX;
            set
            {
                defaultAccelerationX = value;
                currentAccelerationSample.X = value;
            }
        }

        public decimal DefaultAccelerationY
        {
            get => defaultAccelerationY;
            set
            {
                defaultAccelerationY = value;
                currentAccelerationSample.Y = value;
            }
        }

        public decimal DefaultAccelerationZ
        {
            get => defaultAccelerationZ;
            set
            {
                defaultAccelerationZ = value;
                currentAccelerationSample.Z = value;
            }
        }

        public decimal DefaultAngularRateX
        {
            get => defaultAngularRateX;
            set
            {
                defaultAngularRateX = value;
                currentGyroscopeSample.X = value;
            }
        }

        public decimal DefaultAngularRateY
        {
            get => defaultAngularRateY;
            set
            {
                defaultAngularRateY = value;
                currentGyroscopeSample.Y = value;
            }
        }

        public decimal DefaultAngularRateZ
        {
            get => defaultAngularRateZ;
            set
            {
                defaultAngularRateZ = value;
                currentGyroscopeSample.Z = value;
            }
        }

        public void FeedAccelerationSample(decimal x, decimal y, decimal z, uint repeat = 1)
        {
            currentAccelerationSample = new Vector3Sample(x, y, z);
            var frame = FifoFrame.FromVector(FifoTag.AccelerometerNC, currentAccelerationSample,
                GetAccelerometerSensitivityLsbPerG(GetAccelerometerFullScaleG()), VectorPayloadOrder.ZYX);
            for(var i = 0u; i < repeat; i++)
            {
                TryPushFrameToFifo(frame);
                MaybeBatchTimestampFrame();
            }
            UpdateInterrupts();
        }

        public void FeedAngularRateSample(decimal x, decimal y, decimal z, uint repeat = 1)
        {
            currentGyroscopeSample = new Vector3Sample(x, y, z);
            var frame = FifoFrame.FromVector(FifoTag.GyroscopeNC, currentGyroscopeSample,
                GetGyroscopeSensitivityLsbPerDps(), VectorPayloadOrder.XYZ);
            for(var i = 0u; i < repeat; i++)
            {
                TryPushFrameToFifo(frame);
                MaybeBatchTimestampFrame();
            }
            UpdateInterrupts();
        }

        public void FeedQvarSample(short rawValue, uint repeat = 1)
        {
            qvarRawValue = rawValue;
            qvarValid = true;
            var frame = FifoFrame.FromInt16(FifoTag.Qvar, rawValue);
            for(var i = 0u; i < repeat; i++)
            {
                TryPushFrameToFifo(frame);
                MaybeBatchTimestampFrame();
            }
            UpdateInterrupts();
        }

        public void FeedQvarSampleFromMillivolts(decimal milliVolts, uint repeat = 1)
        {
            // Table 5: 78 LSB / mV typ.
            var scaled = (short)Math.Max(short.MinValue, Math.Min(short.MaxValue, decimal.ToInt32(decimal.Round(milliVolts * 78m, 0))));
            FeedQvarSample(scaled, repeat);
        }

        public void FeedTemperatureSample(decimal temperatureCelsius, uint repeat = 1)
        {
            Temperature = temperatureCelsius;
            var frame = FifoFrame.FromInt16(FifoTag.Temperature, GetScaledTemperatureShort());
            for(var i = 0u; i < repeat; i++)
            {
                TryPushFrameToFifo(frame);
                MaybeBatchTimestampFrame();
            }
            UpdateInterrupts();
        }

        public void SetStepCounter(ushort value, bool batchToFifo = false)
        {
            stepCounter = value;
            if(batchToFifo)
            {
                TryPushFrameToFifo(FifoFrame.FromUInt16(FifoTag.StepCounter, stepCounter));
            }
            UpdateInterrupts();
        }

        public void FeedAccelerationSamplesFromRESD(string path, uint channel = 0, ulong startTime = 0,
            RESDStreamSampleOffset sampleOffsetType = RESDStreamSampleOffset.Specified, long sampleOffsetTime = 0)
        {
            accelerometerResdStream = this.CreateRESDStream<AccelerationSample>(path, channel, sampleOffsetType, sampleOffsetTime);
            accelerometerResdStartTime = startTime;
            RestartAccelerationFeeder();
        }

        public void FeedAngularRateSamplesFromRESD(string path, uint channel = 0, ulong startTime = 0,
            RESDStreamSampleOffset sampleOffsetType = RESDStreamSampleOffset.Specified, long sampleOffsetTime = 0)
        {
            gyroResdStream = this.CreateRESDStream<AngularRateSample>(path, channel, sampleOffsetType, sampleOffsetTime);
            gyroResdStartTime = startTime;
            RestartGyroFeeder();
        }

        public void FeedTemperatureSamplesFromRESD(string path, uint channel)
        {
            temperatureResdStream = this.CreateRESDStream<TemperatureSample>(path, channel);
        }

        public void OnGPIO(int number, bool value)
        {
            if(number != 0)
            {
                return;
            }

            var selected = ChipSelectActiveLow ? !value : value;
            if(selected == chipSelected)
            {
                return;
            }

            chipSelected = selected;
            if(!chipSelected)
            {
                FinishTransmission();
            }
        }

        public void FinishTransmission()
        {
            commandInProgress = CommandTypes.None;
            currentAddress = 0;
        }

        public byte Transmit(byte data)
        {
            if(!chipSelected)
            {
                return DeselectedTransmitValue;
            }

            switch(commandInProgress)
            {
            case CommandTypes.None:
                currentAddress = (byte)(data & 0x7F);
                commandInProgress = ((data & 0x80) != 0) ? CommandTypes.Read : CommandTypes.Write;
                this.Log(LogLevel.Noisy, "SPI {0} transaction begins at register 0x{1:X2}", commandInProgress, currentAddress);
                return 0;

            case CommandTypes.Read:
                var readValue = ReadRegister(currentAddress);
                TryIncrementAddress();
                return readValue;

            case CommandTypes.Write:
                WriteRegister(currentAddress, data);
                TryIncrementAddress();
                return 0;

            default:
                return 0;
            }
        }

        [OnRESDSample(SampleType.Acceleration)]
        [BeforeRESDSample(SampleType.Acceleration)]
        private void HandleAccelerationSample(AccelerationSample sample, TimeInterval _)
        {
            if(sample != null)
            {
                var x = (decimal)sample.AccelerationX / 1e6m;
                var y = (decimal)sample.AccelerationY / 1e6m;
                var z = (decimal)sample.AccelerationZ / 1e6m;
                FeedAccelerationSample(x, y, z);
            }
            else
            {
                FeedAccelerationSample(DefaultAccelerationX, DefaultAccelerationY, DefaultAccelerationZ);
            }
        }

        [AfterRESDSample(SampleType.Acceleration)]
        private void HandleAccelerationSampleEnded(AccelerationSample _, TimeInterval __)
        {
            RestartAccelerationFeeder();
        }

        [OnRESDSample(SampleType.AngularRate)]
        [BeforeRESDSample(SampleType.AngularRate)]
        private void HandleAngularRateSample(AngularRateSample sample, TimeInterval _)
        {
            if(sample != null)
            {
                // RESD angular-rate samples are in 1e-5 rad/s in the provided example.
                var x = RadiansToDegrees * (decimal)sample.AngularRateX / 1e5m;
                var y = RadiansToDegrees * (decimal)sample.AngularRateY / 1e5m;
                var z = RadiansToDegrees * (decimal)sample.AngularRateZ / 1e5m;
                FeedAngularRateSample(x, y, z);
            }
            else
            {
                FeedAngularRateSample(DefaultAngularRateX, DefaultAngularRateY, DefaultAngularRateZ);
            }
        }

        [AfterRESDSample(SampleType.AngularRate)]
        private void HandleAngularRateSampleEnded(AngularRateSample _, TimeInterval __)
        {
            RestartGyroFeeder();
        }

        [OnRESDSample(SampleType.Temperature)]
        [BeforeRESDSample(SampleType.Temperature)]
        private void HandleTemperatureSample(TemperatureSample sample, TimeInterval _)
        {
            if(sample != null)
            {
                FeedTemperatureSample((decimal)sample.Temperature / 1e3m);
            }
        }

        private byte ReadRegister(byte address)
        {
            var embedded = EmbeddedAccessEnabled && address != (byte)MainRegisters.FuncCfgAccess;
            var value = embedded ? ReadEmbeddedRegister(address) : ReadMainRegister(address);
            this.Log(LogLevel.Noisy, "Read {0} register 0x{1:X2} -> 0x{2:X2}", embedded ? "embedded" : "main", address, value);
            return value;
        }

        private void WriteRegister(byte address, byte value)
        {
            var embedded = EmbeddedAccessEnabled && address != (byte)MainRegisters.FuncCfgAccess;
            this.Log(LogLevel.Noisy, "Write {0} register 0x{1:X2} <- 0x{2:X2}", embedded ? "embedded" : "main", address, value);
            if(embedded)
            {
                WriteEmbeddedRegister(address, value);
                return;
            }
            WriteMainRegister(address, value);
        }

        private byte ReadMainRegister(byte address)
        {
            switch((MainRegisters)address)
            {
            case MainRegisters.WhoAmI:
                return 0x71;

            case MainRegisters.OutTempL:
                TryUpdateCurrentTemperatureSample();
                return (byte)(GetScaledTemperatureShort() & 0xFF);
            case MainRegisters.OutTempH:
                return (byte)((GetScaledTemperatureShort() >> 8) & 0xFF);

            case MainRegisters.OutXLG:
            case MainRegisters.OutXHG:
            case MainRegisters.OutYLG:
            case MainRegisters.OutYHG:
            case MainRegisters.OutZLG:
            case MainRegisters.OutZHG:
                return GetGyroscopeOutputByte((MainRegisters)address);

            case MainRegisters.OutZLA:
            case MainRegisters.OutZHA:
            case MainRegisters.OutYLA:
            case MainRegisters.OutYHA:
            case MainRegisters.OutXLA:
            case MainRegisters.OutXHA:
                return GetAccelerometerOutputByte((MainRegisters)address, dualChannel: false);

            case MainRegisters.UiOutZLDualC:
            case MainRegisters.UiOutZHDualC:
            case MainRegisters.UiOutYLDualC:
            case MainRegisters.UiOutYHDualC:
            case MainRegisters.UiOutXLDualC:
            case MainRegisters.UiOutXHDualC:
                return GetAccelerometerOutputByte((MainRegisters)address, dualChannel: true);

            case MainRegisters.AhQvarOutL:
                return (byte)(qvarRawValue & 0xFF);
            case MainRegisters.AhQvarOutH:
                return (byte)((qvarRawValue >> 8) & 0xFF);

            case MainRegisters.CtrlStatus:
                // 0: host controls config; 1: FSM owns subset of CTRL regs.
                return (byte)(((embeddedRegisters[(int)EmbeddedRegisters.FuncCfgAccess] & 0x08) != 0) ? 0x04 : 0x00);

            case MainRegisters.FifoStatus1:
                return (byte)(fifoQueue.Count & 0xFF);
            case MainRegisters.FifoStatus2:
                return ReadFifoStatus2();

            case MainRegisters.AllIntSrc:
                return allInterruptSource;
            case MainRegisters.StatusReg:
                return ReadStatusRegister();

            case MainRegisters.Timestamp0:
            case MainRegisters.Timestamp1:
            case MainRegisters.Timestamp2:
            case MainRegisters.Timestamp3:
                return GetTimestampByte(address - (byte)MainRegisters.Timestamp0);

            case MainRegisters.WakeUpSrc:
                return wakeUpSource;
            case MainRegisters.TapSrc:
                return tapSource;
            case MainRegisters.D6DSrc:
                return d6dSource;
            case MainRegisters.EmbFuncStatusMainpage:
                return embeddedFunctionStatusMainpage;
            case MainRegisters.FsmStatusMainpage:
                return fsmStatusMainpage;
            case MainRegisters.MlcStatusMainpage:
                return mlcStatusMainpage;
            case MainRegisters.InternalFreqFine:
                return 0x00;

            case MainRegisters.FifoDataOutTag:
                DequeueFifoFrame();
                return ComposeFifoTagByte(lastDequeuedFrame.Tag);
            case MainRegisters.FifoDataOutByte0:
            case MainRegisters.FifoDataOutByte1:
            case MainRegisters.FifoDataOutByte2:
            case MainRegisters.FifoDataOutByte3:
            case MainRegisters.FifoDataOutByte4:
            case MainRegisters.FifoDataOutByte5:
                return GetFifoDataByte(address - (byte)MainRegisters.FifoDataOutByte0);


            default:
                return mainRegisters[address];
            }
        }

        private void WriteMainRegister(byte address, byte value)
        {
            switch((MainRegisters)address)
            {
            case MainRegisters.FuncCfgAccess:
                mainRegisters[address] = (byte)(value & 0x8C);
                if((value & 0x04) != 0)
                {
                    PerformGlobalReset();
                }
                return;

            case MainRegisters.PinCtrl:
                mainRegisters[address] = (byte)(value & 0xE3);
                return;

            case MainRegisters.IfCfg:
                mainRegisters[address] = (byte)(value & 0xFD);
                UpdateInterrupts();
                return;

            case MainRegisters.FifoCtrl1:
                mainRegisters[address] = value;
                HandleMainRegisterSideEffects((MainRegisters)address, value);
                return;
            case MainRegisters.FifoCtrl2:
                mainRegisters[address] = (byte)(value & 0xD7);
                HandleMainRegisterSideEffects((MainRegisters)address, mainRegisters[address]);
                return;
            case MainRegisters.FifoCtrl3:
                mainRegisters[address] = value;
                HandleMainRegisterSideEffects((MainRegisters)address, value);
                return;
            case MainRegisters.FifoCtrl4:
                mainRegisters[address] = (byte)(value & 0xF7);
                HandleMainRegisterSideEffects((MainRegisters)address, mainRegisters[address]);
                return;
            case MainRegisters.CounterBdrReg1:
                mainRegisters[address] = (byte)(value & 0x67);
                HandleMainRegisterSideEffects((MainRegisters)address, mainRegisters[address]);
                return;
            case MainRegisters.CounterBdrReg2:
            case MainRegisters.Int1Ctrl:
            case MainRegisters.Int2Ctrl:
            case MainRegisters.Ctrl1:
            case MainRegisters.Ctrl2:
            case MainRegisters.Ctrl5:
            case MainRegisters.Ctrl6:
            case MainRegisters.FunctionsEnable:
            case MainRegisters.InactivityDur:
            case MainRegisters.InactivityThs:
            case MainRegisters.TapCfg0:
            case MainRegisters.TapCfg1:
            case MainRegisters.TapCfg2:
            case MainRegisters.TapThs6D:
            case MainRegisters.TapDur:
            case MainRegisters.WakeUpThs:
            case MainRegisters.WakeUpDur:
            case MainRegisters.FreeFall:
            case MainRegisters.Md1Cfg:
            case MainRegisters.Md2Cfg:
            case MainRegisters.EmbFuncCfg:
            case MainRegisters.TdmCfg0:
            case MainRegisters.TdmCfg1:
            case MainRegisters.TdmCfg2:
            case MainRegisters.ZOfsUsr:
            case MainRegisters.YOfsUsr:
            case MainRegisters.XOfsUsr:
                mainRegisters[address] = value;
                HandleMainRegisterSideEffects((MainRegisters)address, value);
                return;
            case MainRegisters.Ctrl4:
                mainRegisters[address] = (byte)(value & 0x1E);
                HandleMainRegisterSideEffects((MainRegisters)address, mainRegisters[address]);
                return;
            case MainRegisters.Ctrl7:
                mainRegisters[address] = (byte)(value & 0xFD);
                HandleMainRegisterSideEffects((MainRegisters)address, mainRegisters[address]);
                return;
            case MainRegisters.Ctrl8:
            case MainRegisters.Ctrl9:
                mainRegisters[address] = (byte)(value & 0xFB);
                HandleMainRegisterSideEffects((MainRegisters)address, mainRegisters[address]);
                return;
            case MainRegisters.Ctrl10:
                mainRegisters[address] = (byte)(value & 0x7F);
                HandleMainRegisterSideEffects((MainRegisters)address, mainRegisters[address]);
                return;

            case MainRegisters.Ctrl3:
                mainRegisters[address] = (byte)(value & 0xC4);
                if((value & 0x80) != 0)
                {
                    this.Log(LogLevel.Noisy, "CTRL3.BOOT requested; approximating boot reload and auto-clearing the bit.");
                    mainRegisters[address] &= 0x7F;
                }
                if((value & 0x01) != 0)
                {
                    PerformSoftwareReset();
                }
                return;

            default:
                mainRegisters[address] = value;
                return;
            }
        }

        private byte ReadEmbeddedRegister(byte address)
        {
            switch((EmbeddedRegisters)address)
            {
            case EmbeddedRegisters.PageValue:
                return ReadPageValue();
            case EmbeddedRegisters.EmbFuncExecStatus:
                return 0x00;
            case EmbeddedRegisters.EmbFuncStatus:
                return embeddedRegisters[(int)EmbeddedRegisters.EmbFuncStatus];
            case EmbeddedRegisters.FsmStatus:
                return fsmStatus;
            case EmbeddedRegisters.MlcStatus:
                return mlcStatus;
            case EmbeddedRegisters.FsmOuts1:
            case EmbeddedRegisters.FsmOuts2:
            case EmbeddedRegisters.FsmOuts3:
            case EmbeddedRegisters.FsmOuts4:
            case EmbeddedRegisters.FsmOuts5:
            case EmbeddedRegisters.FsmOuts6:
            case EmbeddedRegisters.FsmOuts7:
            case EmbeddedRegisters.FsmOuts8:
            case EmbeddedRegisters.Mlc1Src:
            case EmbeddedRegisters.Mlc2Src:
            case EmbeddedRegisters.Mlc3Src:
            case EmbeddedRegisters.Mlc4Src:
                return embeddedRegisters[address];
            case EmbeddedRegisters.StepCounterL:
                return (byte)(stepCounter & 0xFF);
            case EmbeddedRegisters.StepCounterH:
                return (byte)((stepCounter >> 8) & 0xFF);
            case EmbeddedRegisters.EmbFuncSrc:
                return embeddedRegisters[address];
            default:
                return embeddedRegisters[address];
            }
        }

        private void WriteEmbeddedRegister(byte address, byte value)
        {
            switch((EmbeddedRegisters)address)
            {
            case EmbeddedRegisters.PageValue:
                WritePageValue(value);
                return;
            case EmbeddedRegisters.PageAddress:
                embeddedRegisters[address] = value;
                return;
            case EmbeddedRegisters.PageSel:
                embeddedRegisters[address] = (byte)(value & 0xF0);
                return;
            case EmbeddedRegisters.PageRw:
                embeddedRegisters[address] = (byte)(value & 0xE0);
                return;
            case EmbeddedRegisters.EmbFuncEnA:
            case EmbeddedRegisters.EmbFuncEnB:
            case EmbeddedRegisters.EmbFuncInt1:
            case EmbeddedRegisters.FsmInt1:
            case EmbeddedRegisters.MlcInt1:
            case EmbeddedRegisters.EmbFuncInt2:
            case EmbeddedRegisters.FsmInt2:
            case EmbeddedRegisters.MlcInt2:
            case EmbeddedRegisters.EmbFuncFifoEnA:
            case EmbeddedRegisters.EmbFuncFifoEnB:
            case EmbeddedRegisters.FsmEnable:
            case EmbeddedRegisters.FsmLongCounterL:
            case EmbeddedRegisters.FsmLongCounterH:
            case EmbeddedRegisters.IntAckMask:
            case EmbeddedRegisters.FsmOdr:
            case EmbeddedRegisters.MlcOdr:
            case EmbeddedRegisters.EmbFuncInitA:
            case EmbeddedRegisters.EmbFuncInitB:
                embeddedRegisters[address] = value;
                if((EmbeddedRegisters)address == EmbeddedRegisters.EmbFuncInitA && (value & 0x01) != 0)
                {
                    // SFLP_GAME_INIT: keep the bit latched in storage, but no functional model.
                }
                return;
            case EmbeddedRegisters.SflpOdr:
                embeddedRegisters[address] = (byte)((value & 0x38) | 0x43);
                return;
            default:
                embeddedRegisters[address] = value;
                return;
            }
        }

        private void HandleMainRegisterSideEffects(MainRegisters register, byte value)
        {
            switch(register)
            {
            case MainRegisters.Ctrl1:
            case MainRegisters.Ctrl2:
            case MainRegisters.FifoCtrl3:
            case MainRegisters.FunctionsEnable:
            case MainRegisters.CounterBdrReg1:
                counterBatchEvents = 0;
                counterBatchEventLatched = false;
                timestampBatchCounter = 0;
                RestartDefaultFeedersIfNeeded();
                goto case MainRegisters.Int1Ctrl;

            case MainRegisters.Ctrl4:
            case MainRegisters.Ctrl7:
            case MainRegisters.Int1Ctrl:
            case MainRegisters.Int2Ctrl:
            case MainRegisters.Md1Cfg:
            case MainRegisters.Md2Cfg:
            case MainRegisters.IfCfg:
                UpdateInterrupts();
                break;

            case MainRegisters.CounterBdrReg2:
                counterBatchEvents = 0;
                counterBatchEventLatched = false;
                timestampBatchCounter = 0;
                UpdateInterrupts();
                break;

            case MainRegisters.Ctrl3:
                UpdateInterrupts();
                break;

            case MainRegisters.FifoCtrl1:
                UpdateInterrupts();
                break;

            case MainRegisters.FifoCtrl4:
                timestampBatchCounter = 0;
                if(GetFifoMode() == FifoMode.Bypass)
                {
                    fifoQueue.Clear();
                    previousFifoOverrunStatus = false;
                    lastDequeuedFrame = FifoFrame.Empty;
                    fifoTagCounter = 0;
                }
                UpdateInterrupts();
                break;
            }
        }

        private void TryIncrementAddress()
        {
            if(currentAddress == (byte)MainRegisters.FifoDataOutByte5)
            {
                currentAddress = (byte)MainRegisters.FifoDataOutTag;
                return;
            }

            if((mainRegisters[(int)MainRegisters.Ctrl3] & 0x04) == 0)
            {
                return;
            }

            currentAddress = (byte)((currentAddress + 1) & 0x7F);
        }

        private void TryPushFrameToFifo(FifoFrame frame)
        {
            KeepLatestFrame(frame);

            if(!ShouldBatchFrame(frame.Tag))
            {
                return;
            }

            var mode = GetFifoMode();
            if(mode == FifoMode.Bypass)
            {
                return;
            }

            if(StopOnWatermarkEnabled && IsFifoWatermarkReached)
            {
                UpdateInterrupts();
                return;
            }

            if(mode == FifoMode.Continuous)
            {
                if(fifoQueue.Count >= MaxFifoWords)
                {
                    fifoQueue.Dequeue();
                    previousFifoOverrunStatus = true;
                }
                fifoQueue.Enqueue(frame);
            }
            else
            {
                if(fifoQueue.Count >= MaxFifoWords)
                {
                    previousFifoOverrunStatus = true;
                    UpdateInterrupts();
                    return;
                }
                fifoQueue.Enqueue(frame);
            }

            UpdateCounterBdr(frame.Tag);
            UpdateInterrupts();
        }

        private void KeepLatestFrame(FifoFrame frame)
        {
            switch(frame.Tag)
            {
            case FifoTag.AccelerometerNC:
            case FifoTag.AccelerometerDualC:
                latestAccelerationFrame = frame;
                break;
            case FifoTag.GyroscopeNC:
                latestGyroscopeFrame = frame;
                break;
            case FifoTag.Temperature:
                latestTemperatureFrame = frame;
                break;
            case FifoTag.Qvar:
                latestQvarFrame = frame;
                break;
            case FifoTag.StepCounter:
                latestStepCounterFrame = frame;
                break;
            case FifoTag.Timestamp:
                latestTimestampFrame = frame;
                break;
            }
        }

        private bool ShouldBatchFrame(FifoTag tag)
        {
            switch(tag)
            {
            case FifoTag.AccelerometerNC:
                return GetBatchDataRateXL() != 0 && IsAccelerometerPoweredOn;
            case FifoTag.GyroscopeNC:
                return GetBatchDataRateGY() != 0 && IsGyroscopePoweredOn;
            case FifoTag.Qvar:
                return (mainRegisters[(int)MainRegisters.CounterBdrReg1] & 0x04) != 0 && IsQvarEnabled;
            case FifoTag.Temperature:
                return ((mainRegisters[(int)MainRegisters.FifoCtrl4] >> 4) & 0x03) != 0 && IsTemperatureDataReady;
            case FifoTag.StepCounter:
                return false;
            case FifoTag.Timestamp:
                return TimestampEnabled && GetTimestampBatchDecimation() != 0;
            case FifoTag.AccelerometerDualC:
                var dualFromIf = (mainRegisters[(int)MainRegisters.EmbFuncCfg] & 0x80) != 0;
                var dualFromFsm = (mainRegisters[(int)MainRegisters.FifoCtrl2] & 0x01) != 0;
                return dualFromIf || dualFromFsm;
            default:
                return false;
            }
        }

        private void DequeueFifoFrame()
        {
            if(fifoQueue.Count == 0)
            {
                lastDequeuedFrame = FifoFrame.Empty;
                return;
            }

            lastDequeuedFrame = fifoQueue.Dequeue();
            fifoTagCounter = (byte)((fifoTagCounter + 1) & 0x03);
            UpdateInterrupts();
        }

        private byte ComposeFifoTagByte(FifoTag tag)
        {
            if(tag == FifoTag.Empty)
            {
                return 0x00;
            }

            return (byte)(((byte)tag << 3) | ((fifoTagCounter & 0x03) << 1));
        }

        private byte GetFifoDataByte(int index)
        {
            if(lastDequeuedFrame.Tag == FifoTag.Empty)
            {
                return 0;
            }

            var bytes = lastDequeuedFrame.GetPayloadBytes();
            if(index < 0 || index >= bytes.Length)
            {
                return 0;
            }
            return bytes[index];
        }

        private byte ReadFifoStatus2()
        {
            var diffFifo8 = (fifoQueue.Count >> 8) & 0x1;
            var fifoOvrLatched = previousFifoOverrunStatus ? 1 : 0;
            var counterBdr = counterBatchEventLatched ? 1 : 0;
            var fifoFullIa = IsFifoFull ? 1 : 0;
            var fifoOvrIa = previousFifoOverrunStatus ? 1 : 0;
            var fifoWtmIa = IsFifoWatermarkReached ? 1 : 0;

            var value = (byte)(diffFifo8
                | (fifoOvrLatched << 3)
                | (counterBdr << 4)
                | (fifoFullIa << 5)
                | (fifoOvrIa << 6)
                | (fifoWtmIa << 7));

            counterBatchEventLatched = false;
            previousFifoOverrunStatus = false;
            UpdateInterrupts();
            return value;
        }

        private byte ReadStatusRegister()
        {
            var value = (byte)0;
            if(timestampOverflowLatched)
            {
                value |= 0x80;
            }
            if(IsQvarDataAvailable)
            {
                value |= 0x08;
            }
            if(IsTemperatureDataReady)
            {
                value |= 0x04;
            }
            if(IsGyroscopeDataReady)
            {
                value |= 0x02;
            }
            if(IsAccelerometerDataReady)
            {
                value |= 0x01;
            }
            return value;
        }

        private void UpdateCounterBdr(FifoTag tag)
        {
            var triggerGyro = ((mainRegisters[(int)MainRegisters.CounterBdrReg1] >> 5) & 0x03) == 0x01;
            var matched = (!triggerGyro && tag == FifoTag.AccelerometerNC) || (triggerGyro && tag == FifoTag.GyroscopeNC);
            if(!matched)
            {
                return;
            }

            var threshold = GetCounterBdrThreshold();
            if(threshold == 0)
            {
                return;
            }

            counterBatchEvents++;
            if(counterBatchEvents >= threshold)
            {
                counterBatchEvents = 0;
                counterBatchEventLatched = true;
            }
        }

        private void UpdateInterrupts()
        {
            var int1Requested = false;
            var int2Requested = false;

            // INT1_CTRL (0Dh)
            var int1Ctrl = mainRegisters[(int)MainRegisters.Int1Ctrl];
            if((int1Ctrl & 0x01) != 0 && IsAccelerometerDataReady)
            {
                int1Requested = true;
            }
            if((int1Ctrl & 0x02) != 0 && IsGyroscopeDataReady)
            {
                int1Requested = true;
            }
            if((int1Ctrl & 0x08) != 0 && IsFifoWatermarkReached)
            {
                int1Requested = true;
            }
            if((int1Ctrl & 0x10) != 0 && previousFifoOverrunStatus)
            {
                int1Requested = true;
            }
            if((int1Ctrl & 0x20) != 0 && IsFifoFull)
            {
                int1Requested = true;
            }
            if((int1Ctrl & 0x40) != 0 && counterBatchEventLatched)
            {
                int1Requested = true;
            }

            // INT2_CTRL (0Eh)
            var int2Ctrl = mainRegisters[(int)MainRegisters.Int2Ctrl];
            if((int2Ctrl & 0x01) != 0 && IsAccelerometerDataReady)
            {
                int2Requested = true;
            }
            if((int2Ctrl & 0x02) != 0 && IsGyroscopeDataReady)
            {
                int2Requested = true;
            }
            if((int2Ctrl & 0x08) != 0 && IsFifoWatermarkReached)
            {
                int2Requested = true;
            }
            if((int2Ctrl & 0x10) != 0 && previousFifoOverrunStatus)
            {
                int2Requested = true;
            }
            if((int2Ctrl & 0x20) != 0 && IsFifoFull)
            {
                int2Requested = true;
            }
            if((int2Ctrl & 0x40) != 0 && counterBatchEventLatched)
            {
                int2Requested = true;
            }
            if((int2Ctrl & 0x80) != 0 && embeddedFunctionEndOperation)
            {
                int2Requested = true;
            }

            // MD1_CFG / MD2_CFG event routing.
            if((mainRegisters[(int)MainRegisters.Md1Cfg] & 0x7F) != 0 && (allInterruptSource != 0 || embeddedFunctionStatusMainpage != 0))
            {
                int1Requested = true;
            }
            if((mainRegisters[(int)MainRegisters.Md2Cfg] & 0xFF) != 0 && (allInterruptSource != 0 || embeddedFunctionStatusMainpage != 0 || timestampOverflowLatched))
            {
                int2Requested = true;
            }

            // CTRL4/CTRL7 routes.
            if((mainRegisters[(int)MainRegisters.Ctrl4] & 0x04) != 0 && IsTemperatureDataReady)
            {
                int2Requested = true;
            }
            if((mainRegisters[(int)MainRegisters.Ctrl7] & 0x40) != 0 && IsQvarDataAvailable)
            {
                int2Requested = true;
            }

            // CTRL4.INT2_on_INT1 reroutes selected INT2 sources to INT1.
            if((mainRegisters[(int)MainRegisters.Ctrl4] & 0x10) != 0)
            {
                if((mainRegisters[(int)MainRegisters.Ctrl4] & 0x04) != 0 && IsTemperatureDataReady)
                {
                    int1Requested = true;
                }
                if((mainRegisters[(int)MainRegisters.Ctrl7] & 0x40) != 0 && IsQvarDataAvailable)
                {
                    int1Requested = true;
                }
                if((int2Ctrl & 0x80) != 0 && embeddedFunctionEndOperation)
                {
                    int1Requested = true;
                }
                if((mainRegisters[(int)MainRegisters.Md2Cfg] & 0x01) != 0 && timestampOverflowLatched)
                {
                    int1Requested = true;
                }
            }

            var activeLow = (mainRegisters[(int)MainRegisters.IfCfg] & 0x10) != 0;
            Interrupt1.Set(activeLow ? !int1Requested : int1Requested);
            Interrupt2.Set(activeLow ? !int2Requested : int2Requested);
        }

        private byte GetGyroscopeOutputByte(MainRegisters register)
        {
            var fs = GetGyroscopeFullScaleDps();
            var sensitivity = 1000m / GetGyroscopeSensitivityMilliDpsPerLsb(fs);

            switch(register)
            {
            case MainRegisters.OutXLG:
                return GetScaledAxisByte(currentGyroscopeSample.X, sensitivity, upperByte: false);
            case MainRegisters.OutXHG:
                return GetScaledAxisByte(currentGyroscopeSample.X, sensitivity, upperByte: true);
            case MainRegisters.OutYLG:
                return GetScaledAxisByte(currentGyroscopeSample.Y, sensitivity, upperByte: false);
            case MainRegisters.OutYHG:
                return GetScaledAxisByte(currentGyroscopeSample.Y, sensitivity, upperByte: true);
            case MainRegisters.OutZLG:
                return GetScaledAxisByte(currentGyroscopeSample.Z, sensitivity, upperByte: false);
            case MainRegisters.OutZHG:
                return GetScaledAxisByte(currentGyroscopeSample.Z, sensitivity, upperByte: true);
            default:
                return 0;
            }
        }

        private byte GetAccelerometerOutputByte(MainRegisters register, bool dualChannel)
        {
            var fs = dualChannel ? 16m : GetAccelerometerFullScaleG();
            var sensitivity = GetAccelerometerSensitivityLsbPerG(fs);

            decimal z = currentAccelerationSample.Z;
            decimal y = currentAccelerationSample.Y;
            decimal x = currentAccelerationSample.X;

            if(!dualChannel && UserOffsetOnOutput)
            {
                x -= GetUserOffsetG(mainRegisters[(int)MainRegisters.XOfsUsr]);
                y -= GetUserOffsetG(mainRegisters[(int)MainRegisters.YOfsUsr]);
                z -= GetUserOffsetG(mainRegisters[(int)MainRegisters.ZOfsUsr]);
            }

            switch(register)
            {
            case MainRegisters.OutZLA:
            case MainRegisters.UiOutZLDualC:
                return GetScaledAxisByte(z, sensitivity, upperByte: false);
            case MainRegisters.OutZHA:
            case MainRegisters.UiOutZHDualC:
                return GetScaledAxisByte(z, sensitivity, upperByte: true);
            case MainRegisters.OutYLA:
            case MainRegisters.UiOutYLDualC:
                return GetScaledAxisByte(y, sensitivity, upperByte: false);
            case MainRegisters.OutYHA:
            case MainRegisters.UiOutYHDualC:
                return GetScaledAxisByte(y, sensitivity, upperByte: true);
            case MainRegisters.OutXLA:
            case MainRegisters.UiOutXLDualC:
                return GetScaledAxisByte(x, sensitivity, upperByte: false);
            case MainRegisters.OutXHA:
            case MainRegisters.UiOutXHDualC:
                return GetScaledAxisByte(x, sensitivity, upperByte: true);
            default:
                return 0;
            }
        }

        private byte GetTimestampByte(int byteIndex)
        {
            var timestamp = GetCurrentTimestampValue();
            return (byte)((timestamp >> (byteIndex * 8)) & 0xFF);
        }

        private void SyncCpuTime()
        {
            if(machine.SystemBus.TryGetCurrentCPU(out var cpu))
            {
                cpu.SyncTime();
            }
        }

        private void TryUpdateCurrentTemperatureSample()
        {
            if(temperatureResdStream == null)
            {
                return;
            }

            SyncCpuTime();
            var currentTimestamp = machine.ClockSource.CurrentValue.TotalNanoseconds;
            if(temperatureResdStream.TryGetSample(currentTimestamp, out var sample) == RESDStreamStatus.OK)
            {
                Temperature = (decimal)sample.Temperature / 1e3m;
            }
        }

        private short GetScaledTemperatureShort()
        {
            return SaturateToInt16((Temperature - 25m) * 256m);
        }

        private decimal GetAccelerometerFullScaleG()
        {
            switch(mainRegisters[(int)MainRegisters.Ctrl8] & 0x03)
            {
            case 0x0:
                return 2m;
            case 0x1:
                return 4m;
            case 0x2:
                return 8m;
            case 0x3:
                return 16m;
            default:
                return 2m;
            }
        }

        private decimal GetAccelerometerSensitivityLsbPerG(decimal fs)
        {
            switch((int)fs)
            {
            case 2:
                return 1000m / 0.061m;
            case 4:
                return 1000m / 0.122m;
            case 8:
                return 1000m / 0.244m;
            case 16:
                return 1000m / 0.488m;
            default:
                return 1000m / 0.061m;
            }
        }

        private int GetGyroscopeFullScaleDps()
        {
            switch(mainRegisters[(int)MainRegisters.Ctrl6] & 0x0F)
            {
            case 0x0:
                return 125;
            case 0x1:
                return 250;
            case 0x2:
                return 500;
            case 0x3:
                return 1000;
            case 0x4:
                return 2000;
            case 0xC:
                return 4000;
            default:
                return 125;
            }
        }

        private decimal GetGyroscopeSensitivityMilliDpsPerLsb(int fs)
        {
            switch(fs)
            {
            case 125:
                return 4.375m;
            case 250:
                return 8.75m;
            case 500:
                return 17.5m;
            case 1000:
                return 35m;
            case 2000:
                return 70m;
            case 4000:
                return 140m;
            default:
                return 4.375m;
            }
        }

        private decimal GetGyroscopeSensitivityLsbPerDps()
        {
            return 1000m / GetGyroscopeSensitivityMilliDpsPerLsb(GetGyroscopeFullScaleDps());
        }

        private decimal ClampDecimal(decimal value, decimal min, decimal max)
        {
            if(value < min)
            {
                return min;
            }

            if(value > max)
            {
                return max;
            }

            return value;
        }

        private decimal NextRandomDecimal(decimal min, decimal max)
        {
            lock(randomLock)
            {
                return min + ((decimal)randomGenerator.NextDouble() * (max - min));
            }
        }

        private decimal GetRandomRangeScale()
        {
            return ClampDecimal(RandomSampleFullScaleFraction, 0m, 1m);
        }

        private Vector3Sample CreateRandomAccelerationSample()
        {
            var max = GetAccelerometerFullScaleG() * GetRandomRangeScale();

            return new Vector3Sample(
                NextRandomDecimal(-max, max),
                NextRandomDecimal(-max, max),
                NextRandomDecimal(-max, max)
            );
        }

        private Vector3Sample CreateRandomAngularRateSample()
        {
            var max = (decimal)GetGyroscopeFullScaleDps() * GetRandomRangeScale();

            return new Vector3Sample(
                NextRandomDecimal(-max, max),
                NextRandomDecimal(-max, max),
                NextRandomDecimal(-max, max)
            );
        }

        private short NextDeterministicAccelerationRaw()
        {
            var value = deterministicAccelerationSequence;
            deterministicAccelerationSequence = (ushort)(deterministicAccelerationSequence + DeterministicSequenceStep);
            return unchecked((short)value);
        }

        private decimal AccelerometerRawToG(short raw)
        {
            var sensitivity = GetAccelerometerSensitivityLsbPerG(GetAccelerometerFullScaleG());

            if(sensitivity == 0m)
            {
                return 0m;
            }

            return raw / sensitivity;
        }

        private decimal GetUserOffsetG(byte registerValue)
        {
            var signed = unchecked((sbyte)registerValue);
            var weight = ((mainRegisters[(int)MainRegisters.Ctrl9] & 0x02) != 0) ? (1m / 64m) : (1m / 1024m);
            return signed * weight;
        }

        private byte GetScaledAxisByte(decimal value, decimal sensitivity, bool upperByte)
        {
            var shortValue = SaturateToInt16(value * sensitivity);
            return upperByte ? (byte)((shortValue >> 8) & 0xFF) : (byte)(shortValue & 0xFF);
        }

        private byte ReadPageValue()
        {
            var pageIndex = GetPageMemoryIndex();
            var value = pageMemory[pageIndex];
            if((embeddedRegisters[(int)EmbeddedRegisters.PageRw] & 0x20) != 0)
            {
                embeddedRegisters[(int)EmbeddedRegisters.PageAddress]++;
            }
            return value;
        }

        private void WritePageValue(byte value)
        {
            var pageIndex = GetPageMemoryIndex();
            pageMemory[pageIndex] = value;
            if((embeddedRegisters[(int)EmbeddedRegisters.PageRw] & 0x40) != 0)
            {
                embeddedRegisters[(int)EmbeddedRegisters.PageAddress]++;
            }
        }

        private int GetPageMemoryIndex()
        {
            var page = (embeddedRegisters[(int)EmbeddedRegisters.PageSel] >> 4) & 0x0F;
            var address = embeddedRegisters[(int)EmbeddedRegisters.PageAddress];
            return ((page << 8) | address) % PageMemorySize;
        }

        private ushort GetCounterBdrThreshold()
        {
            var high = (ushort)(mainRegisters[(int)MainRegisters.CounterBdrReg1] & 0x03);
            var low = mainRegisters[(int)MainRegisters.CounterBdrReg2];
            return (ushort)((high << 8) | low);
        }

        private int GetFifoWatermark()
        {
            return mainRegisters[(int)MainRegisters.FifoCtrl1];
        }

        private int GetSamplesAddedPerOdr()
        {
            var count = 0;
            if(ShouldBatchFrame(FifoTag.AccelerometerNC))
            {
                count++;
            }
            if(ShouldBatchFrame(FifoTag.GyroscopeNC))
            {
                count++;
            }
            if(ShouldBatchFrame(FifoTag.Temperature))
            {
                count++;
            }
            if(ShouldBatchFrame(FifoTag.Qvar))
            {
                count++;
            }
            if(ShouldBatchFrame(FifoTag.Timestamp))
            {
                count++;
            }
            if(count == 0)
            {
                count = 1;
            }
            return count;
        }

        private byte GetBatchDataRateXL()
        {
            return (byte)(mainRegisters[(int)MainRegisters.FifoCtrl3] & 0x0F);
        }

        private byte GetBatchDataRateGY()
        {
            return (byte)((mainRegisters[(int)MainRegisters.FifoCtrl3] >> 4) & 0x0F);
        }

        private FifoMode GetFifoMode()
        {
            var rawMode = mainRegisters[(int)MainRegisters.FifoCtrl4] & 0x07;
            switch(rawMode)
            {
            case 0x0:
                return FifoMode.Bypass;
            case 0x1:
            case 0x3:
            case 0x7:
                return FifoMode.StopWhenFull;
            case 0x2:
            case 0x4:
            case 0x6:
                return FifoMode.Continuous;
            default:
                return FifoMode.Bypass;
            }
        }

        private uint GetFrequencyFromBatchDataRate(byte value)
        {
            return GetFrequencyFromDataRateCode(value);
        }

        private uint GetFrequencyFromAccelerometerOdr()
        {
            return GetFrequencyFromDataRateCode((byte)(mainRegisters[(int)MainRegisters.Ctrl1] & 0x0F));
        }

        private uint GetFrequencyFromGyroscopeOdr()
        {
            var odr = (byte)(mainRegisters[(int)MainRegisters.Ctrl2] & 0x0F);
            if(odr == 0x1)
            {
                return 0;
            }
            return GetFrequencyFromDataRateCode(odr);
        }

        private uint GetFrequencyFromDataRateCode(byte value)
        {
            switch(value)
            {
            case 0x0:
                return 0;
            case 0x1:
                return 2;
            case 0x2:
                return 8;
            case 0x3:
                return 15;
            case 0x4:
                return 30;
            case 0x5:
                return 60;
            case 0x6:
                return 120;
            case 0x7:
                return 240;
            case 0x8:
                return 480;
            case 0x9:
                return 960;
            case 0xA:
                return 1920;
            case 0xB:
                return 3840;
            case 0xC:
                return 7680;
            default:
                return 0;
            }
        }

        private uint GetAccelerationFeedFrequency()
        {
            if(!IsAccelerometerPoweredOn)
            {
                return 0;
            }
            var batchRate = GetBatchDataRateXL();
            return batchRate != 0 ? GetFrequencyFromBatchDataRate(batchRate) : GetFrequencyFromAccelerometerOdr();
        }

        private uint GetGyroscopeFeedFrequency()
        {
            if(!IsGyroscopePoweredOn)
            {
                return 0;
            }
            var batchRate = GetBatchDataRateGY();
            return batchRate != 0 ? GetFrequencyFromBatchDataRate(batchRate) : GetFrequencyFromGyroscopeOdr();
        }

        private IManagedThread CreateRawImuReplayFeeder()
        {
            if(rawImuReplaySamples == null || rawImuReplaySamples.Length == 0)
            {
                return null;
            }

            var accelerationFrequency = GetAccelerationFeedFrequency();
            var gyroscopeFrequency = GetGyroscopeFeedFrequency();

            // Wait until the production firmware has enabled both sensor
            // streams. Register configuration occurs in several SPI writes.
            if(accelerationFrequency == 0 || gyroscopeFrequency == 0)
            {
                return null;
            }

            if(accelerationFrequency != gyroscopeFrequency)
            {
                this.Log(LogLevel.Error,
                    "Raw IMU replay requires equal accelerometer and gyroscope rates; got XL={0} Hz GY={1} Hz.",
                    accelerationFrequency, gyroscopeFrequency);
                return null;
            }

            if(accelerationFrequency % rawImuReplaySourceRateHz != 0)
            {
                this.Log(LogLevel.Error,
                    "Raw IMU replay source rate {0} Hz does not divide configured sensor rate {1} Hz.",
                    rawImuReplaySourceRateHz, accelerationFrequency);
                return null;
            }

            rawImuReplayHoldTicks = accelerationFrequency / rawImuReplaySourceRateHz;
            if(rawImuReplayHoldTicks == 0)
            {
                rawImuReplayHoldTicks = 1;
            }

            this.Log(LogLevel.Info,
                "Starting raw IMU replay: sensor_rate={0} Hz source_rate={1} Hz hold_ticks={2}.",
                accelerationFrequency, rawImuReplaySourceRateHz, rawImuReplayHoldTicks);

            var feeder = machine.ObtainManagedThread(
                FeedNextRawImuReplayTick,
                accelerationFrequency,
                name: "lsm6dsv16bx-raw-imu-replay",
                owner: this
            );
            feeder.Start();
            return feeder;
        }

        private void FeedNextRawImuReplayTick()
        {
            // Do not consume the input trace while the production firmware is
            // still configuring the FIFO. This preserves sample 0 as the first
            // value visible to the firmware once continuous FIFO batching is
            // enabled.
            if(GetFifoMode() == FifoMode.Bypass
                || !ShouldBatchFrame(FifoTag.AccelerometerNC)
                || !ShouldBatchFrame(FifoTag.GyroscopeNC))
            {
                return;
            }

            if(rawImuReplaySamples == null || rawImuReplaySamples.Length == 0)
            {
                return;
            }

            if(rawImuReplayIndex >= rawImuReplaySamples.Length)
            {
                if(rawImuReplayLoop)
                {
                    rawImuReplayIndex = 0;
                    rawImuReplayHoldCounter = 0;
                }
                else
                {
                    if(!rawImuReplayFinishedLogged)
                    {
                        this.Log(LogLevel.Info, "Raw IMU replay finished after {0} source samples.",
                            rawImuReplaySamples.Length);
                        rawImuReplayFinishedLogged = true;
                    }
                    return;
                }
            }

            var sample = rawImuReplaySamples[rawImuReplayIndex];

            // Keep the FIFO payload ordering identical to the existing model:
            // gyro uses XYZ while accelerometer FIFO payloads use ZYX.
            TryPushFrameToFifo(FifoFrame.FromRawVector(
                FifoTag.GyroscopeNC,
                sample.GyroX, sample.GyroY, sample.GyroZ,
                VectorPayloadOrder.XYZ));
            MaybeBatchTimestampFrame();

            TryPushFrameToFifo(FifoFrame.FromRawVector(
                FifoTag.AccelerometerNC,
                sample.AccX, sample.AccY, sample.AccZ,
                VectorPayloadOrder.ZYX));
            MaybeBatchTimestampFrame();

            UpdateInterrupts();

            rawImuReplayHoldCounter++;
            if(rawImuReplayHoldCounter >= rawImuReplayHoldTicks)
            {
                rawImuReplayHoldCounter = 0;
                rawImuReplayIndex++;
            }
        }

        private void RestartRawImuReplayFeeder()
        {
            rawImuReplayFeederThread?.Stop();
            rawImuReplayFeederThread = null;

            // Reset the trace whenever the production firmware reconfigures
            // the sensor so each run begins from the same deterministic input.
            rawImuReplayIndex = 0;
            rawImuReplayHoldCounter = 0;
            rawImuReplayFinishedLogged = false;

            rawImuReplayFeederThread = CreateRawImuReplayFeeder();
        }

        private IManagedThread CreateAccelerationDefaultSampleFeeder()
        {
            var frequency = GetAccelerationFeedFrequency();
            if(frequency == 0)
            {
                return null;
            }

            return CreateDefaultSampleFeeder(
                () =>
                {
                    if(DeterministicSequenceSamples)
                    {
                        var rawX = NextDeterministicAccelerationRaw();
                        FeedAccelerationSample(AccelerometerRawToG(rawX), 0m, 0m);
                    }
                    else if(RandomizeDefaultSamples)
                    {
                        var sample = CreateRandomAccelerationSample();
                        FeedAccelerationSample(sample.X, sample.Y, sample.Z);
                    }
                    else
                    {
                        FeedAccelerationSample(DefaultAccelerationX, DefaultAccelerationY, DefaultAccelerationZ);
                    }
                },
                frequency,
                "lsm6dsv16bx-accel-default-feeder"
            );
        }

        private IManagedThread CreateAngularRateDefaultSampleFeeder()
        {
            var frequency = GetGyroscopeFeedFrequency();
            if(frequency == 0)
            {
                return null;
            }

            return CreateDefaultSampleFeeder(
                () =>
                {
                    if(RandomizeDefaultSamples)
                    {
                        var sample = CreateRandomAngularRateSample();
                        FeedAngularRateSample(sample.X, sample.Y, sample.Z);
                    }
                    else
                    {
                        FeedAngularRateSample(DefaultAngularRateX, DefaultAngularRateY, DefaultAngularRateZ);
                    }
                },
                frequency,
                "lsm6dsv16bx-gyro-default-feeder"
            );
        }

        private IManagedThread CreateDefaultSampleFeeder(Action action, uint frequency, string name)
        {
            var feeder = machine.ObtainManagedThread(action, frequency, name: name, owner: this);
            action();
            feeder.Start();
            return feeder;
        }

        private void RestartAccelerationFeeder()
        {
            accelerometerFeederThread?.Stop();
            accelerometerFeederThread = null;

            var frequency = GetAccelerationFeedFrequency();
            if(frequency == 0)
            {
                return;
            }

            if(accelerometerResdStream != null)
            {
                accelerometerFeederThread = accelerometerResdStream.StartSampleFeedThread(this, frequency,
                    startTime: accelerometerResdStartTime);
                return;
            }

            accelerometerFeederThread = CreateAccelerationDefaultSampleFeeder();
        }

        private void RestartGyroFeeder()
        {
            gyroFeederThread?.Stop();
            gyroFeederThread = null;

            var frequency = GetGyroscopeFeedFrequency();
            if(frequency == 0)
            {
                return;
            }

            if(gyroResdStream != null)
            {
                gyroFeederThread = gyroResdStream.StartSampleFeedThread(this, frequency, startTime: gyroResdStartTime);
                return;
            }

            gyroFeederThread = CreateAngularRateDefaultSampleFeeder();
        }

        private void RestartDefaultFeedersIfNeeded()
        {
            if(rawImuReplaySamples != null && rawImuReplaySamples.Length > 0)
            {
                accelerometerFeederThread?.Stop();
                accelerometerFeederThread = null;
                gyroFeederThread?.Stop();
                gyroFeederThread = null;
                RestartRawImuReplayFeeder();
                return;
            }

            rawImuReplayFeederThread?.Stop();
            rawImuReplayFeederThread = null;

            RestartAccelerationFeeder();
            RestartGyroFeeder();
        }

        private void StopFeederThreads()
        {
            accelerometerFeederThread?.Stop();
            accelerometerFeederThread = null;
            gyroFeederThread?.Stop();
            gyroFeederThread = null;
            rawImuReplayFeederThread?.Stop();
            rawImuReplayFeederThread = null;
        }

        private void StopAllStreamsAndThreads()
        {
            StopFeederThreads();

            accelerometerResdStream?.Dispose();
            accelerometerResdStream = null;
            accelerometerResdStartTime = 0;
            gyroResdStream?.Dispose();
            gyroResdStream = null;
            gyroResdStartTime = 0;
            temperatureResdStream?.Dispose();
            temperatureResdStream = null;
        }

        private void ResetRegistersAndState(bool preservePinCtrlAndIfCfg, bool preserveChipSelectState)
        {
            var preservedPinCtrl = mainRegisters[(int)MainRegisters.PinCtrl];
            var preservedIfCfg = mainRegisters[(int)MainRegisters.IfCfg];
            var preservedChipSelected = chipSelected;

            Array.Clear(mainRegisters, 0, mainRegisters.Length);
            Array.Clear(embeddedRegisters, 0, embeddedRegisters.Length);
            Array.Clear(pageMemory, 0, pageMemory.Length);
            fifoQueue.Clear();

            LoadResetValues();
            if(preservePinCtrlAndIfCfg)
            {
                mainRegisters[(int)MainRegisters.PinCtrl] = preservedPinCtrl;
                mainRegisters[(int)MainRegisters.IfCfg] = preservedIfCfg;
            }

            chipSelected = preserveChipSelectState ? preservedChipSelected : false;
            currentAddress = 0;
            commandInProgress = CommandTypes.None;
            previousFifoOverrunStatus = false;
            counterBatchEventLatched = false;
            counterBatchEvents = 0;
            timestampOverflowLatched = false;
            embeddedFunctionEndOperation = false;
            lastTimestampValue = 0;
            timestampBatchCounter = 0;
            lastDequeuedFrame = FifoFrame.Empty;
            latestAccelerationFrame = FifoFrame.Empty;
            latestGyroscopeFrame = FifoFrame.Empty;
            latestTemperatureFrame = FifoFrame.Empty;
            latestQvarFrame = FifoFrame.Empty;
            latestStepCounterFrame = FifoFrame.Empty;
            latestTimestampFrame = FifoFrame.Empty;
            fifoTagCounter = 0;
            stepCounter = 0;
            Temperature = 25m;
            qvarRawValue = 0;
            qvarValid = true;
            currentAccelerationSample = new Vector3Sample(defaultAccelerationX, defaultAccelerationY, defaultAccelerationZ);
            currentGyroscopeSample = new Vector3Sample(defaultAngularRateX, defaultAngularRateY, defaultAngularRateZ);

            UpdateInterrupts();
        }

        private void PerformGlobalReset()
        {
            this.Log(LogLevel.Noisy, "FUNC_CFG_ACCESS.SW_POR requested; performing global reset.");
            StopFeederThreads();
            ResetRegistersAndState(preservePinCtrlAndIfCfg: false, preserveChipSelectState: true);
            RestartDefaultFeedersIfNeeded();
        }

        private void PerformSoftwareReset()
        {
            this.Log(LogLevel.Noisy, "CTRL3.SW_RESET requested; resetting software-resettable registers.");
            StopFeederThreads();
            ResetRegistersAndState(preservePinCtrlAndIfCfg: true, preserveChipSelectState: true);
            RestartDefaultFeedersIfNeeded();
        }

        private void LoadResetValues()
        {
            // Main-page reset defaults from the LSM6DSV16BX datasheet.
            mainRegisters[(int)MainRegisters.FuncCfgAccess] = 0x00;
            mainRegisters[(int)MainRegisters.PinCtrl] = 0x23;
            mainRegisters[(int)MainRegisters.IfCfg] = 0x00;
            mainRegisters[(int)MainRegisters.FifoCtrl1] = 0x00;
            mainRegisters[(int)MainRegisters.FifoCtrl2] = 0x00;
            mainRegisters[(int)MainRegisters.FifoCtrl3] = 0x00;
            mainRegisters[(int)MainRegisters.FifoCtrl4] = 0x00;
            mainRegisters[(int)MainRegisters.CounterBdrReg1] = 0x00;
            mainRegisters[(int)MainRegisters.CounterBdrReg2] = 0x00;
            mainRegisters[(int)MainRegisters.Int1Ctrl] = 0x00;
            mainRegisters[(int)MainRegisters.Int2Ctrl] = 0x00;
            mainRegisters[(int)MainRegisters.Ctrl3] = 0x44;
            mainRegisters[(int)MainRegisters.FunctionsEnable] = 0x00;
            mainRegisters[(int)MainRegisters.InactivityDur] = 0x04;
            mainRegisters[(int)MainRegisters.TdmCfg0] = 0x80;
            mainRegisters[(int)MainRegisters.TdmCfg1] = 0xE0;
            mainRegisters[(int)MainRegisters.TdmCfg2] = 0x01;

            // Embedded-function registers touched by the driver.
            embeddedRegisters[(int)EmbeddedRegisters.PageSel] = 0x10;
            embeddedRegisters[(int)EmbeddedRegisters.PageRw] = 0x00;
            embeddedRegisters[(int)EmbeddedRegisters.EmbFuncEnA] = 0x00;
            embeddedRegisters[(int)EmbeddedRegisters.EmbFuncEnB] = 0x00;
            embeddedRegisters[(int)EmbeddedRegisters.EmbFuncFifoEnA] = 0x00;
            embeddedRegisters[(int)EmbeddedRegisters.EmbFuncFifoEnB] = 0x00;
            embeddedRegisters[(int)EmbeddedRegisters.SflpOdr] = 0x43;
            embeddedRegisters[(int)EmbeddedRegisters.FsmOdr] = 0x00;
            embeddedRegisters[(int)EmbeddedRegisters.MlcOdr] = 0x00;
            embeddedRegisters[(int)EmbeddedRegisters.EmbFuncInitA] = 0x00;
            embeddedRegisters[(int)EmbeddedRegisters.EmbFuncInitB] = 0x00;
        }

        private bool EmbeddedAccessEnabled => (mainRegisters[(int)MainRegisters.FuncCfgAccess] & 0x80) != 0;

        private bool TimestampEnabled => (mainRegisters[(int)MainRegisters.FunctionsEnable] & 0x40) != 0;

        private bool UserOffsetOnOutput => (mainRegisters[(int)MainRegisters.Ctrl9] & 0x01) != 0;

        private bool IsAccelerometerPoweredOn => (mainRegisters[(int)MainRegisters.Ctrl1] & 0x0F) != 0;

        private bool IsGyroscopePoweredOn
        {
            get
            {
                var odr = (byte)(mainRegisters[(int)MainRegisters.Ctrl2] & 0x0F);
                return odr != 0 && odr != 0x1;
            }
        }

        private bool IsQvarEnabled => (mainRegisters[(int)MainRegisters.Ctrl7] & 0x80) != 0;

        private bool IsAccelerometerDataReady => IsAccelerometerPoweredOn;

        private bool IsGyroscopeDataReady => IsGyroscopePoweredOn;

        private bool IsTemperatureDataReady => IsAccelerometerPoweredOn || IsGyroscopePoweredOn;

        private bool IsQvarDataAvailable => IsQvarEnabled && qvarValid;

        private bool StopOnWatermarkEnabled => (mainRegisters[(int)MainRegisters.FifoCtrl2] & 0x80) != 0;

        private bool IsFifoWatermarkReached => GetFifoWatermark() != 0 && fifoQueue.Count >= GetFifoWatermark();

        private bool IsFifoFull => fifoQueue.Count >= MaxFifoWords;

        private readonly IMachine machine;
        private readonly byte[] mainRegisters;
        private readonly byte[] embeddedRegisters;
        private readonly byte[] pageMemory;
        private readonly Queue<FifoFrame> fifoQueue;

        private bool chipSelected;
        private byte currentAddress;
        private CommandTypes commandInProgress;
        private bool previousFifoOverrunStatus;
        private bool counterBatchEventLatched;
        private ushort counterBatchEvents;
        private bool timestampOverflowLatched;
        private bool embeddedFunctionEndOperation;
        private byte fifoTagCounter;
        private ushort stepCounter;
        private short qvarRawValue;
        private bool qvarValid;
        private uint lastTimestampValue;
        private int timestampBatchCounter;
        private ulong accelerometerResdStartTime;
        private ulong gyroResdStartTime;
        private readonly object randomLock = new object();
        private Random randomGenerator = new Random(0x5EED);
        private int randomSeed = 0x5EED;
        private ushort deterministicSequenceStart = 1;
        private ushort deterministicAccelerationSequence = 1;

        private byte allInterruptSource;
        private byte wakeUpSource;
        private byte tapSource;
        private byte d6dSource;
        private byte embeddedFunctionStatusMainpage;
        private byte fsmStatusMainpage;
        private byte mlcStatusMainpage;
        private byte fsmStatus;
        private byte mlcStatus;

        private decimal defaultAccelerationX;
        private decimal defaultAccelerationY;
        private decimal defaultAccelerationZ;
        private decimal defaultAngularRateX;
        private decimal defaultAngularRateY;
        private decimal defaultAngularRateZ;

        private Vector3Sample currentAccelerationSample;
        private Vector3Sample currentGyroscopeSample;

        private FifoFrame lastDequeuedFrame;
        private FifoFrame latestAccelerationFrame;
        private FifoFrame latestGyroscopeFrame;
        private FifoFrame latestTemperatureFrame;
        private FifoFrame latestQvarFrame;
        private FifoFrame latestStepCounterFrame;
        private FifoFrame latestTimestampFrame;

        private RESDStream<AccelerationSample> accelerometerResdStream;
        private RESDStream<AngularRateSample> gyroResdStream;
        private RESDStream<TemperatureSample> temperatureResdStream;
        private IManagedThread accelerometerFeederThread;
        private IManagedThread gyroFeederThread;
        private IManagedThread rawImuReplayFeederThread;

        private RawImuReplaySample[] rawImuReplaySamples;
        private uint rawImuReplaySourceRateHz = 120;
        private bool rawImuReplayLoop;
        private int rawImuReplayIndex;
        private uint rawImuReplayHoldCounter;
        private uint rawImuReplayHoldTicks = 1;
        private bool rawImuReplayFinishedLogged;

        private const int RegisterSpaceSize = 0x80;
        private const int PageMemorySize = 0x1000;
        private const int MaxFifoWords = 0x1FF;
        private const long TimestampResolutionNanoseconds = 21750;
        private const decimal RadiansToDegrees = 180m / (decimal)Math.PI;

        private enum CommandTypes
        {
            Write = 0,
            Read = 1,
            None,
        }

        private enum FifoMode
        {
            Bypass,
            StopWhenFull,
            Continuous,
        }

        private enum VectorPayloadOrder
        {
            XYZ,
            ZYX,
        }

        private enum FifoTag : byte
        {
            Empty = 0x00,
            GyroscopeNC = 0x01,
            AccelerometerNC = 0x02,
            Temperature = 0x03,
            Timestamp = 0x04,
            CfgChange = 0x05,
            AccelerometerNC_T_2 = 0x06,
            AccelerometerNC_T_1 = 0x07,
            Accelerometer2xC = 0x08,
            Accelerometer3xC = 0x09,
            GyroscopeNC_T_2 = 0x0A,
            GyroscopeNC_T_1 = 0x0B,
            Gyroscope2xC = 0x0C,
            Gyroscope3xC = 0x0D,
            StepCounter = 0x12,
            SflpGameRotationVector = 0x13,
            SflpGyroscopeBias = 0x16,
            SflpGravityVector = 0x17,
            MlcResult = 0x1A,
            MlcFilter = 0x1B,
            MlcFeature = 0x1C,
            AccelerometerDualC = 0x1D,
            Qvar = 0x1F,
        }

        private enum MainRegisters : byte
        {
            FuncCfgAccess = 0x01,
            PinCtrl = 0x02,
            IfCfg = 0x03,
            FifoCtrl1 = 0x07,
            FifoCtrl2 = 0x08,
            FifoCtrl3 = 0x09,
            FifoCtrl4 = 0x0A,
            CounterBdrReg1 = 0x0B,
            CounterBdrReg2 = 0x0C,
            Int1Ctrl = 0x0D,
            Int2Ctrl = 0x0E,
            WhoAmI = 0x0F,
            Ctrl1 = 0x10,
            Ctrl2 = 0x11,
            Ctrl3 = 0x12,
            Ctrl4 = 0x13,
            Ctrl5 = 0x14,
            Ctrl6 = 0x15,
            Ctrl7 = 0x16,
            Ctrl8 = 0x17,
            Ctrl9 = 0x18,
            Ctrl10 = 0x19,
            CtrlStatus = 0x1A,
            FifoStatus1 = 0x1B,
            FifoStatus2 = 0x1C,
            AllIntSrc = 0x1D,
            StatusReg = 0x1E,
            OutTempL = 0x20,
            OutTempH = 0x21,
            OutXLG = 0x22,
            OutXHG = 0x23,
            OutYLG = 0x24,
            OutYHG = 0x25,
            OutZLG = 0x26,
            OutZHG = 0x27,
            OutZLA = 0x28,
            OutZHA = 0x29,
            OutYLA = 0x2A,
            OutYHA = 0x2B,
            OutXLA = 0x2C,
            OutXHA = 0x2D,
            UiOutZLDualC = 0x34,
            UiOutZHDualC = 0x35,
            UiOutYLDualC = 0x36,
            UiOutYHDualC = 0x37,
            UiOutXLDualC = 0x38,
            UiOutXHDualC = 0x39,
            AhQvarOutL = 0x3A,
            AhQvarOutH = 0x3B,
            Timestamp0 = 0x40,
            Timestamp1 = 0x41,
            Timestamp2 = 0x42,
            Timestamp3 = 0x43,
            WakeUpSrc = 0x45,
            TapSrc = 0x46,
            D6DSrc = 0x47,
            EmbFuncStatusMainpage = 0x49,
            FsmStatusMainpage = 0x4A,
            MlcStatusMainpage = 0x4B,
            InternalFreqFine = 0x4F,
            FunctionsEnable = 0x50,
            InactivityDur = 0x54,
            InactivityThs = 0x55,
            TapCfg0 = 0x56,
            TapCfg1 = 0x57,
            TapCfg2 = 0x58,
            TapThs6D = 0x59,
            TapDur = 0x5A,
            WakeUpThs = 0x5B,
            WakeUpDur = 0x5C,
            FreeFall = 0x5D,
            Md1Cfg = 0x5E,
            Md2Cfg = 0x5F,
            EmbFuncCfg = 0x63,
            TdmCfg0 = 0x6C,
            TdmCfg1 = 0x6D,
            TdmCfg2 = 0x6E,
            ZOfsUsr = 0x73,
            YOfsUsr = 0x74,
            XOfsUsr = 0x75,
            FifoDataOutTag = 0x78,
            FifoDataOutByte0 = 0x79,
            FifoDataOutByte1 = 0x7A,
            FifoDataOutByte2 = 0x7B,
            FifoDataOutByte3 = 0x7C,
            FifoDataOutByte4 = 0x7D,
            FifoDataOutByte5 = 0x7E,
        }

        private enum EmbeddedRegisters : byte
        {
            FuncCfgAccess = 0x01,
            PageSel = 0x02,
            EmbFuncEnA = 0x04,
            EmbFuncEnB = 0x05,
            EmbFuncExecStatus = 0x07,
            PageAddress = 0x08,
            PageValue = 0x09,
            EmbFuncInt1 = 0x0A,
            FsmInt1 = 0x0B,
            MlcInt1 = 0x0D,
            EmbFuncInt2 = 0x0E,
            FsmInt2 = 0x0F,
            MlcInt2 = 0x11,
            EmbFuncStatus = 0x12,
            FsmStatus = 0x13,
            MlcStatus = 0x15,
            PageRw = 0x17,
            EmbFuncFifoEnA = 0x44,
            EmbFuncFifoEnB = 0x45,
            FsmEnable = 0x46,
            FsmLongCounterL = 0x48,
            FsmLongCounterH = 0x49,
            IntAckMask = 0x4B,
            FsmOuts1 = 0x4C,
            FsmOuts2 = 0x4D,
            FsmOuts3 = 0x4E,
            FsmOuts4 = 0x4F,
            FsmOuts5 = 0x50,
            FsmOuts6 = 0x51,
            FsmOuts7 = 0x52,
            FsmOuts8 = 0x53,
            SflpOdr = 0x5E,
            FsmOdr = 0x5F,
            MlcOdr = 0x60,
            StepCounterL = 0x62,
            StepCounterH = 0x63,
            EmbFuncSrc = 0x64,
            EmbFuncInitA = 0x66,
            EmbFuncInitB = 0x67,
            Mlc1Src = 0x70,
            Mlc2Src = 0x71,
            Mlc3Src = 0x72,
            Mlc4Src = 0x73,
        }

        private struct RawImuReplaySample
        {
            public RawImuReplaySample(short gyroX, short gyroY, short gyroZ,
                short accX, short accY, short accZ, byte movementEvent)
            {
                GyroX = gyroX;
                GyroY = gyroY;
                GyroZ = gyroZ;
                AccX = accX;
                AccY = accY;
                AccZ = accZ;
                MovementEvent = movementEvent;
            }

            public readonly short GyroX;
            public readonly short GyroY;
            public readonly short GyroZ;
            public readonly short AccX;
            public readonly short AccY;
            public readonly short AccZ;
            public readonly byte MovementEvent;
        }

        private struct Vector3Sample
        {
            public Vector3Sample(decimal x, decimal y, decimal z)
            {
                X = x;
                Y = y;
                Z = z;
            }

            public decimal X;
            public decimal Y;
            public decimal Z;
        }

        private struct FifoFrame
        {
            public FifoFrame(FifoTag tag, byte[] payload)
            {
                Tag = tag;
                Payload = new byte[6];
                if(payload != null)
                {
                    Array.Copy(payload, Payload, Math.Min(Payload.Length, payload.Length));
                }
            }

            public static FifoFrame FromVector(FifoTag tag, Vector3Sample vector, decimal sensitivityLsbPerUnit, VectorPayloadOrder order)
            {
                return new FifoFrame(tag, PackVector(vector, sensitivityLsbPerUnit, order));
            }

            public static FifoFrame FromRawVector(FifoTag tag, short x, short y, short z, VectorPayloadOrder order)
            {
                return order == VectorPayloadOrder.ZYX
                    ? new FifoFrame(tag, Pack3(z, y, x))
                    : new FifoFrame(tag, Pack3(x, y, z));
            }

            public static FifoFrame FromInt16(FifoTag tag, short scalar)
            {
                return new FifoFrame(tag, PackScalar((ushort)scalar));
            }

            public static FifoFrame FromUInt16(FifoTag tag, ushort scalar)
            {
                return new FifoFrame(tag, PackScalar(scalar));
            }

            public static FifoFrame FromUInt32(FifoTag tag, uint scalar)
            {
                return new FifoFrame(tag, new byte[]
                {
                    (byte)(scalar & 0xFF),
                    (byte)((scalar >> 8) & 0xFF),
                    (byte)((scalar >> 16) & 0xFF),
                    (byte)((scalar >> 24) & 0xFF),
                    0, 0,
                });
            }

            public byte[] GetPayloadBytes()
            {
                return Payload;
            }

            public static readonly FifoFrame Empty = new FifoFrame(FifoTag.Empty, new byte[6]);

            public readonly FifoTag Tag;
            private readonly byte[] Payload;

            private static byte[] PackVector(Vector3Sample vector, decimal sensitivityLsbPerUnit, VectorPayloadOrder order)
            {
                return order == VectorPayloadOrder.ZYX
                    ? Pack3(
                        Scale(vector.Z, sensitivityLsbPerUnit),
                        Scale(vector.Y, sensitivityLsbPerUnit),
                        Scale(vector.X, sensitivityLsbPerUnit))
                    : Pack3(
                        Scale(vector.X, sensitivityLsbPerUnit),
                        Scale(vector.Y, sensitivityLsbPerUnit),
                        Scale(vector.Z, sensitivityLsbPerUnit));
            }

            private static short Scale(decimal value, decimal sensitivity)
            {
                return SaturateToInt16(value * sensitivity);
            }

            private static byte[] PackScalar(ushort scalar)
            {
                return new byte[]
                {
                    (byte)(scalar & 0xFF),
                    (byte)((scalar >> 8) & 0xFF),
                    0, 0, 0, 0,
                };
            }

            private static byte[] Pack3(short a, short b, short c)
            {
                return new byte[]
                {
                    (byte)(a & 0xFF), (byte)((a >> 8) & 0xFF),
                    (byte)(b & 0xFF), (byte)((b >> 8) & 0xFF),
                    (byte)(c & 0xFF), (byte)((c >> 8) & 0xFF),
                };
            }
        }

        private static short SaturateToInt16(decimal value)
        {
            var scaled = decimal.ToInt64(decimal.Round(value, 0));
            if(scaled > short.MaxValue)
            {
                scaled = short.MaxValue;
            }
            else if(scaled < short.MinValue)
            {
                scaled = short.MinValue;
            }
            return (short)scaled;
        }

        private uint GetCurrentTimestampValue()
        {
            if(!TimestampEnabled)
            {
                return 0;
            }

            SyncCpuTime();
            var currentTimestamp = (uint)(machine.ClockSource.CurrentValue.TotalNanoseconds / TimestampResolutionNanoseconds);
            if(currentTimestamp < lastTimestampValue)
            {
                timestampOverflowLatched = true;
            }
            lastTimestampValue = currentTimestamp;
            return currentTimestamp;
        }

        private int GetTimestampBatchDecimation()
        {
            switch((mainRegisters[(int)MainRegisters.FifoCtrl4] >> 6) & 0x03)
            {
            case 0x1:
                return 1;
            case 0x2:
                return 8;
            case 0x3:
                return 32;
            default:
                return 0;
            }
        }

        private void MaybeBatchTimestampFrame()
        {
            var decimation = GetTimestampBatchDecimation();
            if(!TimestampEnabled || decimation == 0 || GetFifoMode() == FifoMode.Bypass)
            {
                return;
            }

            timestampBatchCounter++;
            if(timestampBatchCounter < decimation)
            {
                return;
            }

            timestampBatchCounter = 0;
            var frame = FifoFrame.FromUInt32(FifoTag.Timestamp, GetCurrentTimestampValue());
            TryPushFrameToFifo(frame);
        }
    }
}
