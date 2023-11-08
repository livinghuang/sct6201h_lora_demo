function decodeUplink(input) {
  // Get the first byte from the bytes array
  const firstByte = input.bytes[0];

  // Calculate the battery voltage based on the first byte
  const batteryVoltage = (firstByte / 254) * 1.2 + 3.0;
  const batteryVoltageNumber = parseFloat(batteryVoltage.toFixed(2));

  // Create a Uint8Array view of the bytes array starting from index 1
  const uint8Array = new Uint8Array(input.bytes.slice(1));

  // Extract temperature and humidity bytes (assuming it's at specific offsets) and interpret as 32-bit floats
  const hdc1080TemperatureBytes = uint8Array.slice(0, 4);
  const humidityBytes = uint8Array.slice(4, 8);
  const bmp280TemperatureBytes = uint8Array.slice(8, 12);
  const pressureBytes = uint8Array.slice(12, 16);
  
  const hdc1080_temperature = new DataView(hdc1080TemperatureBytes.buffer).getFloat32(0, true);
  const hdc1080_humidity = new DataView(humidityBytes.buffer).getFloat32(0, true);
  const bmp280_internal_temperature = new DataView(bmp280TemperatureBytes.buffer).getFloat32(0, true);
  const bmp280_pressure = new DataView(pressureBytes.buffer).getFloat32(0, true);

  return {
    data: {
      battery_voltage: batteryVoltageNumber,
      hdc1080_data: {
        hdc1080_temperature,
        hdc1080_humidity,
      },
      bmp280_data: {
        bmp280_internal_temperature,
        bmp280_pressure,
      },
    },
  };
}
