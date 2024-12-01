// Define the size of the lookup table
#define TABLE_SIZE 201

// Arrays to hold the PWM and Force values
int pwm_values[TABLE_SIZE] = {
  1100, 1104, 1108, 1112, 1116, 1120, 1124, 1128, 1132, 1136,
  1140, 1144, 1148, 1152, 1156, 1160, 1164, 1168, 1172, 1176,
  1180, 1184, 1188, 1192, 1196, 1200, 1204, 1208, 1212, 1216,
  1220, 1224, 1228, 1232, 1236, 1240, 1244, 1248, 1252, 1256,
  1260, 1264, 1268, 1272, 1276, 1280, 1284, 1288, 1292, 1296,
  1300, 1304, 1308, 1312, 1316, 1320, 1324, 1328, 1332, 1336,
  1340, 1344, 1348, 1352, 1356, 1360, 1364, 1368, 1372, 1376,
  1380, 1384, 1388, 1392, 1396, 1400, 1404, 1408, 1412, 1416,
  1420, 1424, 1428, 1432, 1436, 1440, 1444, 1448, 1452, 1456,
  1460, 1464, 1500, 1536, 1540, 1544, 1548, 1552, 1556, 1560, 
  1564, 1568, 1572, 1576, 1580, 1584, 1588, 1592, 1596, 1600, 
  1604, 1608, 1612, 1616, 1620, 1624, 1628, 1632, 1636, 1640, 
  1644, 1648, 1652, 1656, 1660, 1664, 1668, 1672, 1676, 1680, 
  1684, 1688, 1692, 1696, 1700, 1704, 1708, 1712, 1716, 1720, 
  1724, 1728, 1732, 1736, 1740, 1744, 1748, 1752, 1756, 1760, 
  1764, 1768, 1772, 1776, 1780, 1784, 1788, 1792, 1796, 1800, 
  1804, 1808, 1812, 1816, 1820, 1824, 1828, 1832, 1836, 1840, 
  1844, 1848, 1852, 1856, 1860, 1864, 1868, 1872, 1876, 1880, 
  1884, 1888, 1892, 1896, 1900
};

float force_values[TABLE_SIZE] = {
  -3.52, -3.50, -3.49, -3.45, -3.40, -3.36, -3.29, -3.25, -3.19, -3.14,
  -3.10, -3.06, -3.00, -2.94, -2.88, -2.85, -2.78, -2.76, -2.69, -2.64,
  -2.59, -2.53, -2.49, -2.45, -2.41, -2.35, -2.34, -2.26, -2.20, -2.18,
  -2.12, -2.05, -2.03, -1.99, -1.91, -1.89, -1.82, -1.76, -1.72, -1.68,
  -1.63, -1.58, -1.56, -1.52, -1.48, -1.44, -1.40, -1.37, -1.32, -1.28,
  -1.24, -1.19, -1.17, -1.12, -1.09, -1.05, -1.02, -0.98, -0.95, -0.92,
  -0.88, -0.85, -0.81, -0.77, -0.74, -0.70, -0.68, -0.65, -0.62, -0.59,
  -0.55, -0.52, -0.49, -0.46, -0.43, -0.40, -0.37, -0.35, -0.32, -0.29,
  -0.26, -0.24, -0.21, -0.19, -0.16, -0.15, -0.12, -0.10, -0.08, -0.07,
  -0.05, -0.03, 0.00, 0.05, 0.06, 0.08,
  0.10, 0.12, 0.15, 0.18, 0.20, 0.23, 0.26, 0.29, 0.33, 0.36,
  0.39, 0.43, 0.46, 0.50, 0.53, 0.58, 0.62, 0.64, 0.69, 0.73,
  0.77, 0.83, 0.85, 0.89, 0.92, 0.97, 1.00, 1.05, 1.09, 1.14,
  1.20, 1.23, 1.28, 1.32, 1.37, 1.41, 1.46, 1.51, 1.55, 1.61,
  1.65, 1.71, 1.76, 1.81, 1.85, 1.91, 1.96, 2.00, 2.09, 2.12,
  2.16, 2.25, 2.27, 2.34, 2.43, 2.50, 2.56, 2.64, 2.66, 2.76,
  2.78, 2.88, 2.93, 2.99, 3.05, 3.13, 3.19, 3.23, 3.32, 3.36,
  3.42, 3.49, 3.57, 3.62, 3.69, 3.77, 3.84, 3.92, 3.98, 4.03,
  4.11, 4.15, 4.21, 4.30, 4.38, 4.42, 4.51, 4.53, 4.52
};

// Function to interpolate the PWM value based on the Force input
int interpolatePwmValue(float force) {
  // Iterate through the force values to find the interval containing the input force
  for (int i = 0; i < TABLE_SIZE - 1; i++) {
    if (force_values[i] <= force && force <= force_values[i + 1]) {
      // Linear interpolation formula
      float pwm = pwm_values[i] + (pwm_values[i + 1] - pwm_values[i]) * ((force - force_values[i]) / (force_values[i + 1] - force_values[i]));
      
      // Add limit check for PWM values
      if (pwm < 1300 || pwm > 1700) {
        return -1; // Consider out of bounds if PWM is not in the range [1300, 1700]
      }

      return (int)pwm; // Return the interpolated PWM value
    }
  }
  // If the force is out of bounds, return -1
  return -1;
}

void setup() {
  Serial.begin(9600);
  Serial.println("Enter force value:");
}

void loop() {
  // Check if data is available in the Serial buffer
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n'); // Read the input until newline
    input.trim(); // Remove any extra whitespace
    float force_input = input.toFloat(); // Convert input to float

    // Find the corresponding PWM value using interpolation
    int pwm = interpolatePwmValue(force_input);
    if (pwm != -1) {
      Serial.print("Force: ");
      Serial.print(force_input);
      Serial.print(" -> PWM: ");
      Serial.println(pwm);
    } else {
      Serial.println("Force value out of bounds or PWM value out of acceptable range (1300-1700).");
    }

    // Prompt for the next input
    Serial.println("Enter another force value:");
  }

  delay(500); // Small delay to avoid overwhelming the serial port
}
