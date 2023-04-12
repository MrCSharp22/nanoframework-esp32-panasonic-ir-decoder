#define SerialOutput
//#define VSDebugOutput

using System;
using System.Device.Gpio;
using System.IO.Ports;
using System.Text;
using System.Threading;

using nanoFramework.Hardware.Esp32;
using nanoFramework.Hardware.Esp32.Rmt;

namespace Panasonic_IR_Decoder
{
	public static class Program
	{
		private const int RxChannelPinNumber = 25;

		private const int NumberOfMemoryBlocks = 7;
		private const int BufferSize = 1000;

		private const ushort HeaderPulse = 3456;
		private const ushort HeaderSpace = 1728;
		private const ushort Pulse = 432;
		private const ushort ZeroSpace = 432;
		private const ushort OneSpace = 1296;
		private const ushort PauseSpace = 10000;
		private const float Tolerance = 0.3f; // percentage


#if SerialOutput
		private static SerialPort serialPort;
#endif

		public static void Main()
		{
#if SerialOutput
			Configuration.SetPinFunction(32, DeviceFunction.COM2_TX);
			Configuration.SetPinFunction(33, DeviceFunction.COM2_RX);

			serialPort = new SerialPort("COM2")
			{
				BaudRate = 9600,
				Parity = Parity.None,
				DataBits = 8,
				StopBits = StopBits.One,
				Handshake = Handshake.None,
			};

			serialPort.Open();

			serialPort.WriteLine("IR DECODER UTIL BOOT COMPLETED.");
#endif
			var gpioController = new GpioController();
			var rxChannelPin = gpioController.OpenPin(RxChannelPinNumber, PinMode.InputPullUp);

			var rxChannelSettings = new ReceiverChannelSettings(pinNumber: RxChannelPinNumber)
			{
				NumberOfMemoryBlocks = NumberOfMemoryBlocks,
				BufferSize = BufferSize,

				EnableFilter = true,
				FilterThreshold = 156,

				ReceiveTimeout = TimeSpan.FromSeconds(2),
				IdleThreshold = 12_000,
			};

			using var rxChannel = new ReceiverChannel(rxChannelSettings);
			rxChannel.Start(clearBuffer: true);

			WriteStdOut($"RMT RX Channel: {rxChannelSettings.Channel} with {rxChannelSettings.NumberOfMemoryBlocks} memory blocks (holding {rxChannel.NumberOfMemoryBlocks * 64} rmt items)");

			while (true)
			{
				var commands = rxChannel.GetAllItems();
				if (commands is null)
				{
					continue;
				}
				else if (commands.Length != 220)
				{
					WriteStdOut($"Invalid/Unknown command received. Length: {commands.Length}");

					continue;
				}

				try
				{
					PrintSummary(commands);

					//PrintTimings(commands);
					//PrintAsCSharpDefinition(commands);

					//PrintPanasonicIRCodeAsBinary(commands);
					AnalyzePanasonicIRCrc(commands);
				}
				catch (Exception ex)
				{
					WriteStdOut(ex.Message);
				}
			}
		}

		private static void PrintSummary(RmtCommand[] commands)
		{
			var stringBuilder = new StringBuilder();

			stringBuilder.AppendLine("New command recorded...");
			stringBuilder.AppendLine($"Raw Length: {commands.Length}");
			stringBuilder.AppendLine($"Data Bytes: {(commands.Length - 4) / 8}");
			stringBuilder.AppendLine();

#if SerialOutput
			serialPort.WriteLine(stringBuilder.ToString());
#else
			Console.WriteLine(stringBuilder.ToString()); 
#endif
		}

		private static void PrintTimings(RmtCommand[] commands)
		{
			var stringBuilder = new StringBuilder();
			stringBuilder.AppendLine("(Timing in Pairs)");

			foreach (var command in commands)
			{
				stringBuilder.AppendLine($"{(command.Level0 ? '+' : '-')}{command.Duration0},{(command.Level1 ? '+' : '-')}{command.Duration1},");
			}

			stringBuilder.AppendLine();

#if SerialOutput
			serialPort.WriteLine(stringBuilder.ToString());
#else
			Console.WriteLine(stringBuilder.ToString()); 
#endif
		}

		private static void PrintAsCSharpDefinition(RmtCommand[] commands)
		{
			var stringBuilder = new StringBuilder();
			stringBuilder.AppendLine("C# Definition: ");
			stringBuilder.Append("var commands = new RmtCommand[] {");

			foreach (var command in commands)
			{
				stringBuilder.Append("new RmtCommand(");
				stringBuilder.Append(command.Duration0);
				stringBuilder.Append(",");
				stringBuilder.Append(command.Level0.ToString().ToLower());
				stringBuilder.Append(",");
				stringBuilder.Append(command.Duration1);
				stringBuilder.Append(",");
				stringBuilder.Append(command.Level1.ToString().ToLower());
				stringBuilder.Append("),");
				stringBuilder.Append("\r\n");
			}

			stringBuilder.AppendLine("};");
			stringBuilder.AppendLine();

#if SerialOutput
			serialPort.WriteLine(stringBuilder.ToString());
#else
			Console.WriteLine(stringBuilder.ToString()); 
#endif
		}

		private static void PrintPanasonicIRCodeAsBinary(RmtCommand[] commands)
		{
			var stringBuffer = new StringBuilder();
			stringBuffer.AppendLine();
			stringBuffer.Append("Raw Binary = ");

			int numOfBitsPrinted = 0;

			// skip the first command as it is the header
			// skip last command as it is the end mark
			// this will also skip frame separators
			for (var i = 1; i < commands.Length - 1; i++)
			{
				var command = commands[i];

				if (numOfBitsPrinted % 8 == 0)
					stringBuffer.Append(" ");

				if (IsInRange(command.Duration0, HeaderPulse, Tolerance)) // header, skip
				{
					continue;
				}

				if (IsInRange(command.Duration1, PauseSpace, Tolerance)) // pause command, skip
				{
					continue;
				}

				if (IsInRange(command.Duration1, ZeroSpace, Tolerance)) // 0
				{
					stringBuffer.Append("0");
					numOfBitsPrinted++;
				}
				else
				{
					stringBuffer.Append("1");
					numOfBitsPrinted++;
				}
			}

			stringBuffer.AppendLine();

#if SerialOutput
			serialPort.WriteLine(stringBuffer.ToString());
#else
			Console.WriteLine(stringBuffer.ToString()); 
#endif
		}

		private static void AnalyzePanasonicIRCrc(RmtCommand[] commands)
		{
			var stringBuffer = new StringBuilder();
			stringBuffer.AppendLine();

			// we need to ignore the first command + frame seperators
			var frame1StartIndex = 1;
			var frame1LengthInBits = 64; // 8 bytes
			var frameSeparator1Index = frame1StartIndex + frame1LengthInBits;
			var frameSeparator2Index = frameSeparator1Index + 1;
			var frame2StartIndex = frameSeparator2Index + 1;
			var frame2LengthInBits = 144; // 18 bytes
			var crcIndex = frame2StartIndex + 144;
			var byteSize = 8;

			var frame1 = commands.ToByteArray(frame1StartIndex, frame1LengthInBits);
			var frame2 = commands.ToByteArray(frame2StartIndex, frame2LengthInBits);
			var crc = commands.ToByteArray(crcIndex, byteSize)[0];
			var row = 0;

			stringBuffer.AppendLine("Frame 1:");
			stringBuffer.AppendLine("== | === RAW === | === HEX ===");
			foreach (var b in frame1)
			{
				stringBuffer.AppendLine($"{row.ToString("D2")} | {b.AsStringRepresentation()}  |     0x{b.ToString("X2")}   ");
				row++;
			}

			stringBuffer.AppendLine("Frame 2:");
			stringBuffer.AppendLine("=== RAW === | === HEX === | === LSB8 === | === LSB8 HEX ===");
			foreach (var b in frame2)
			{
				stringBuffer.AppendLine($"{row.ToString("D2")} | {b.AsStringRepresentation()}  |     0x{b.ToString("X2")}    | {b.Reverse().AsStringRepresentation()}   |    0x{b.Reverse().ToString("X2")}   ");
				row++;
			}

			// crc (make it appear as part of frame 2)
			stringBuffer.AppendLine($"{row.ToString("D2")} | {crc.AsStringRepresentation()}  |     0x{crc.ToString("X2")}    | {crc.Reverse().AsStringRepresentation()}   |    0x{crc.Reverse().ToString("X2")}      <<< CRC");

#if SerialOutput
			serialPort.WriteLine(stringBuffer.ToString());
#else
			Console.WriteLine(stringBuffer.ToString());
#endif
		}

		private static byte[] ToByteArray(this RmtCommand[] commands, int start, int length)
		{
			if (length < 8)
				throw new ArgumentOutOfRangeException();

			var bitsInByte = 8;
			var bytesCount = length / bitsInByte;
			var byteArray = new byte[bytesCount];
			byte bitMask = 0x80; // MSB

			for (var commandIndex = start; commandIndex < start + length;)
			{
				byte b = 0x00;
				for (var bitIndex = 0; bitIndex < bitsInByte; bitIndex++)
				{
					var command = commands[commandIndex];
					if (command.Duration1.IsInRange(OneSpace, Tolerance)) // bit value = 1
					{
						b |= (byte)(bitMask >> bitIndex);
					}

					commandIndex++;
				}

				byteArray[((commandIndex - start) / bitsInByte) - 1] = b;
			}

			return byteArray;
		}

		private static bool IsInRange(this ushort val, int expected, float tolerance = 0)
		{
			var min = expected * (1 - tolerance);
			var max = expected * (1 + tolerance);
			return min <= val && val <= max;
		}

		private static byte Reverse(this byte val)
		{
			byte b = 0x00;
			var mask = 0x80;
			for (var i = 0; i < 8; i++)
			{
				if ((int)(val & (mask >> i)) > 0)
				{
					b |= (byte)(mask >> (7 - i));
				}
			}

			return b;
		}

		private static string AsStringRepresentation(this byte b)
		{
			var mask = 0x80; //MSB
			var result = string.Empty;
			for (var i = 0; i < 8; i++)
			{
				if ((int)(b & mask) > 0)
				{
					result += "1";
				}
				else
				{
					result += "0";
				}

				if (mask == 0x10)
					result += " ";

				mask >>= 1;
			}

			return result;
		}

		private static string Reverse(this string value)
		{
			var arr = value.ToCharArray();
			var reversed = new char[arr.Length];
			var lastIndex = arr.Length - 1;
			while (lastIndex >= 0)
			{
				reversed[arr.Length - 1 - lastIndex] = arr[lastIndex];
				lastIndex--;
			}

			return new string(reversed);
		}

		private static void WriteStdOut(string str)
		{
#if SerialOutput
			serialPort.WriteLine(str);
#else
			Console.WriteLine(str); 
#endif
		}
	}
}
