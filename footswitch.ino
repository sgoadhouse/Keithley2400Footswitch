
/* 
 DESCRIPTION
 ====================
 Reads data from Keithley 2400 SourceMeter instrument and sends 
 to keyboard when footswitch is pressed. So can focus on probing and
 actual measurement value goes into a spreadsheet or some other document.

 IMPORTANT: Be sure to have the correct Board (Sparkfun Pro Micro) AND
 Processor (5V, 16 MHz) selected in the Arduino IDE. I once had 3.3V, 8 MHz
 processor selected by mistake and keyboard access did not work.

 Board: Sparkfun Pro Micro
 Processor: ATmega32U4 (5V, 16 MHz) 
      
 
 */
 
// Include the ClickButton library found here :
// https://code.google.com/p/clickbutton/
#include <ClickButton.h>

#include <Keyboard.h>
#include <stdlib.h>  // for dtostre() / dtostrf()

#define PULLUP_TIP_PIN 5
#define PULLUP_RING_PIN 3
#define FOOT_TIP_PIN 4
#define FOOT_RING_PIN 2

#define LED_PIN 17  // RX LED can be controlled directly

// set to 0 for no debugging. 
// set to 1 for minimal debug output to Serial console
// set to higher values for even more debugging output
#define DEBUG (0)  

//
// NOTE: Search for "CHANGE ME" for places where can change the output format
//

// If want to Sweep Current, change MODE_SWEEP to a positive, non-0 value and it will sweep that many measurements
// If want to Sweep Voltage, change MODE_SWEEP to a negative, non-0 value and it will sweep that many measurements
//@@@#define MODE_SWEEP (100)
#define MODE_SWEEP (0)
//@@@#define MODE_SWEEP (-100)

// Starting load current 
#define MODE_LOAD_CURRENT_START (0.000)

// Incrementing load current 
//@@@#define MODE_LOAD_CURRENT_INCR (0.010)
#define MODE_LOAD_CURRENT_INCR (0)

// Starting voltage 
//@@@#define MODE_LOAD_VOLTAGE_START (2.70)
#define MODE_LOAD_VOLTAGE_START (0.00)

// Incrementing voltage (make a negative value for decrementing)
//@@@#define MODE_LOAD_VOLTAGE_INCR (-0.010)
#define MODE_LOAD_VOLTAGE_INCR (0)

// If sending a second measurement, afterwards, move left twice and down once to go to a new line on Excel spreadsheet
#define MODE_LEFT_LEFT_DOWN (1) 

// Instantiate ClickButton objects 
ClickButton footswitch_tip = ClickButton (FOOT_TIP_PIN, LOW);
ClickButton footswitch_ring = ClickButton (FOOT_RING_PIN, LOW);



boolean stringComplete = false;  // whether the string is complete

const int LINE_BUFFER_SIZE = 200; // max line length is one less than this
const float DATA_NaN = +9.91e37; // Not-A-Number according to Keithley User Manual

boolean readMeasurement = false; // true if reading measurement data

float sourceCurrent = MODE_LOAD_CURRENT_START;
float sourceVoltage = MODE_LOAD_VOLTAGE_START;
int sweepCount = 0;
int keyNumCount = 0;

// Modes for decodeMeasurement. _AUTO looks at the Status to see what is being measured
const int UNITS_UNKN=-1;
const int UNITS_AUTO=0;
const int UNITS_VOLT=1;
const int UNITS_CURR=2;

//void keyMeasurementString (String text) {
//    Keyboard.begin(); // start keyboard control
//    Keyboard.print(text);
//    Keyboard.write(KEY_RETURN); // "press" the return key
//    Keyboard.end(); // stop keyboard control
//}

// Send the Undo keyboard sequence to undo the last entry.
// Depending on the program being used, this may need to be some other sequence
void keyMeasurementUndo (void) {
    Keyboard.begin(); // start keyboard control
    Keyboard.press(KEY_LEFT_GUI); // press and hold Command key
    Keyboard.write('z');
    Keyboard.release(KEY_LEFT_GUI);
    Keyboard.end(); // stop keyboard control
    // Update keyNumCount
    keyNumCount = (keyNumCount > 0) ? (keyNumCount - 1) : 0;
}

// Send the Undo keyboard sequence to undo the last entry.
// Depending on the program being used, this may need to be some other sequence
void keyMeasurementLeftDown (int leftCnt=2) {
    Keyboard.begin(); // start keyboard control
    for (int i=0; i<leftCnt; i++) {
      Keyboard.press(KEY_LEFT_ARROW); // press and release Left Arrow
      Keyboard.release(KEY_LEFT_ARROW);      
      // Update keyNumCount
      keyNumCount = (keyNumCount > 0) ? (keyNumCount - 1) : 0;
    }
    Keyboard.press(KEY_DOWN_ARROW); // press and release Down Arrow
    Keyboard.release(KEY_DOWN_ARROW);
    Keyboard.end(); // stop keyboard control
}

void keyMeasurementStr (char *text) {
    Keyboard.begin(); // start keyboard control
    Keyboard.print(text);
    Keyboard.write(KEY_RETURN); // "press" the return key [in Excel, can change direction of next cell]
    Keyboard.end(); // stop keyboard control
    // Update keyNumCount
    keyNumCount = keyNumCount + 1;
}

// Determine the unit prefix and return it as well as the data shifted until it is >= 1
char unitPrefix (float &data) {
  const char prefix[] = {' ', 'm', 'u', 'n', 'p', 'f'};
  int idx=0;

  // Be sure to not go past the end of prefix[]
  while ( (abs(data) < 1) && (idx < (sizeof(prefix)-1)) ) {
    data = data * 1000;
    idx = idx + 1;  
  }

  return prefix[idx];
}

void keyMeasurement (float data, int units) {
    char datastr[32];

    // CHANGE ME: Use "if (0)" to output with units and "if (1)" to output in scientific notation
    if (0) {
      // If want to just return the value using scientific notation:
      keyMeasurementStr(dtostre(data, datastr, 6, DTOSTR_UPPERCASE)); 
    } else {
      // If want to return the value with units:
      char prf = unitPrefix(data);

      // Need to prefix string with a single space so that Excel will except it
      datastr[0] = ' ';

      // convert data to a string
      dtostrf(data, -12, 6, &datastr[1]);

      // Look for the space after the converted number 
      // being careful to not go past the end of the string
      int idx = 1; // start after the leading space
      while(datastr[idx] != ' ' 
            && datastr[idx] != '\t' 
            && datastr[idx] != '\0') {
         idx++; 
      }
      datastr[idx++] = ' ';
      datastr[idx++] = prf;
      if (units == UNITS_CURR) {
       datastr[idx++] = 'A';
      } else {
       datastr[idx++] = 'V';      
      }

      // terminate the string
      datastr[idx++] = '\0';

      // Send it to the computer
      keyMeasurementStr(datastr); 
    }

}

// From: Hoylen http://stackoverflow.com/a/23391677
//
int read_line(char* buffer, int bufsize)
{
  for (int index = 0; index < bufsize; index++) {
    // Wait until characters are available
    while (Serial1.available() == 0) {
    }

    char ch = Serial1.read(); // read next character
    if (DEBUG >= 1) Serial.print(ch); // echo it back: useful with the serial monitor (optional)

    if (ch == '\n') {
      buffer[index] = 0; // end of line reached: null terminate string
      return index; // success: return length of string (zero if string is empty)
    }

    buffer[index] = ch; // Append character to buffer
  }

  // Reached end of buffer, but have not seen the end-of-line yet.
  // Discard the rest of the line (safer than returning a partial line).

  char ch;
  do {
    // Wait until characters are available - @@@ SHOULD HAVE TIMEOUT
    while (Serial1.available() == 0) {
    }
    ch = Serial1.read(); // read next character (and discard it)
    if (DEBUG >= 1) Serial.print(ch); // echo it back
  } while (ch != '\n');

  buffer[0] = 0; // set buffer to empty string even though it should not be used
  return -1; // error: return negative one to indicate the input was too long
}

void decodeMeasurement(char *line, int &measureType, float &volt, float &curr) {
  char *strp = line;
  boolean stop = false;
  const int NUMBER_DATA = 5;
  float data[NUMBER_DATA];
  int index=0;

  measureType = UNITS_UNKN; // set to the UNKNOWN type in case of error

  while (!stop && (index < NUMBER_DATA)) {
    // step through characters until find a comma or the end of the line/string
    while(*strp != '\0' && *strp != ',' && *strp != '\n' && *strp != '\r') {
      strp++;
    }

    if (*strp != ',') {
      // either end of the string or the line, so make this the last iteration of loop
      stop = true;
    }

    // null terminate the sub-string and advance to the next character
    *(strp++) = '\0';

    // convert sub-string to a float
    data[index++] = atof(line);
    {
      char datastr[20];
      if (DEBUG >= 2) Serial.println(dtostre(data[index-1], datastr, 7, DTOSTR_UPPERCASE)); // output to console for debug
    }
    
    // set line to new location of strp
    line = strp;
  }

  if (index != NUMBER_DATA) {
    // wrong number of values, so return NaN
    volt = DATA_NaN;
    curr = DATA_NaN;
    return;
  }

  // Should have received 5 values in scientific notation and seperated by commas, like so:
  //
  // +1.884154E-01,+0.000000E+00,+9.910000E+37,+5.933816E+03,+3.584600E+04
  //
  // VOLTAGE,      CURRENT,      RESISTANCE,   TIMESTAMP,    STATUS
  //
  // TIMESTAMP is a rolling number since the instrument was turned on - we ignore it here
  // STATUS is actually a 24-bit binary value with each bit having a meaning

  // Check the status. 
  // If mode is _AUTO and Voltage is being measured, return voltage (first data). 
  // If mode is _AUTO and current is being measured, return current (second data).
  // Otherwise, return the requested value and always return which value is being returned.
  unsigned long status = (unsigned long) data[4];

  if (DEBUG >= 3) {
    // Decode the Status bits and output to console for debug
    Serial.print("Status = ");
    Serial.println(status);
  
    // From the Keithley 2400 User Manual:
    //
    // The significance of each status bit is explained as follows:
    // Bit 0 (OFLO) — Set to 1 if measurement was made while in over-range. 
    // Bit 1 (Filter) — Set to 1 if measurement was made with the filter enabled. 
    // Bit 2 (Front/Rear) — Set to 1 if FRONT terminals are selected.
    // Bit 3 (Compliance) — Set to 1 if in real compliance.
    // Bit 4 (OVP) — Set to 1 if the over voltage protection limit was reached. 
    // Bit 5 (Math) — Set to 1 if math expression (calc1) is enabled.
    // Bit 6 (Null) — Set to 1 if Null is enabled.
    // Bit 7 (Limits) — Set to 1 if a limit test (calc2) is enabled.
    // Bits 8 and 9 (Limit Results) — Provides limit test results (see grading and sorting modes below).
    // Bit 10 (Auto-ohms) — Set to 1 if auto-ohms enabled. 
    // Bit 11 (V-Meas) — Set to 1 if V-Measure is enabled. 
    // Bit 12 (I-Meas) — Set to 1 if I-Measure is enabled. 
    // Bit 13 (Ω-Meas) — Set to 1 if Ω-Measure is enabled. 
    // Bit 14 (V-Sour) — Set to 1 if V-Source used.
    // Bit 15 (I-Sour) — Set to 1 if I-Source used.
    // Bit 16 (Range Compliance) — Set to 1 if in range compliance.
    // Bit 17 (Offset Compensation) — Set to 1 if Offset Compensated Ohms is enabled.
    // Bit 18 — Contact check failure (see Appendix F).
    // Bits 19, 20 and 21 (Limit Results) — Provides limit test results (see grading and sorting modes below). 
    // Bit 22 (Remote Sense) — Set to 1 if 4-wire remote sense selected.
    // Bit 23 (Pulse Mode) — Set to 1 if in the Pulse Mode.
    if (status & 0x000001L) Serial.println("Status - OVER-RANGE");
    if (status & 0x000002L) Serial.println("Status - FILTER ON"); else Serial.println("Status - FILTER off");
    if (status & 0x000004L) Serial.println("Status - FRONT Terminals"); else Serial.println("Status - REAR Terminals");
    if (status & 0x000008L) Serial.println("Status - REAL compliance"); else Serial.println("Status - Range compliance");
    if (status & 0x000010L) Serial.println("Status - OVER-VOLTAGE!");
    if (status & 0x000800L) Serial.println("Status - Voltage Measurement");
    if (status & 0x001000L) Serial.println("Status - Current Measurement");
    if (status & 0x004000L) Serial.println("Status - Voltage SOURCED");
    if (status & 0x008000L) Serial.println("Status - Current SOURCED");
  }

  volt = data[0]; // return voltage value
  curr = data[1]; // return current value
  
  if (status & 0x001000L) {
    // Current Measurement enabled
    measureType = UNITS_CURR; // let caller know that current was measured
  } else {
    // Current Measurement is not enabled so default to voltage
    measureType = UNITS_VOLT; // let caller know that voltage was (likely) measured
  }

}


void setupInstrument() {
  // Send commands to the instrument to set it up for taking measurements
  //
  // Setup instrument to handle "\n" line endings
  //
  // Set AUTO ON mode so Keithley goes to idle after measurements
  Serial1.print(":SENS:VOLT:NPLC 10\n"); // set to high accuracy for ALL measurements - not just voltage
  Serial1.print(":SOUR:CLE:AUTO ON\n"); // Set AUTO ON mode so Keithley goes to idle after measurements
  //@@@Serial1.print(":SOUR:CLE:AUTO OFF\n"); // Set AUTO OFF mode so Keithley stays enabled after measurement
  //@@@Serial1.print(":ARM:COUN 6\n"); // set ARM Count = 6
  Serial1.print(":ARM:COUN 1\n"); // set ARM Count = 1
  Serial1.print(":ARM:SOUR IMM\n"); // set ARM Source = Immediate
  //@@@Serial1.print(":ARM:SOUR TIM\n"); // set ARM Source = Timer
  //@@@Serial1.print(":ARM:TIM 0.010\n"); // set ARM Timer to 10 ms
  Serial1.print(":TRIG:COUN 1\n"); // set Trigger Count = 1
  Serial1.print(":TRIG:SOUR IMM\n"); // set Trigger Source = Immediate
  Serial1.print(":SENS:AVER:TCON REP\n"); // Repeating Filter mode
  //@@@Serial1.print(":SENS:AVER:COUNT 16\n"); // Average over 16 readings
  Serial1.print(":SENS:AVER:COUNT 10\n"); // Average over 10 readings
  //@@@Serial1.print(":SENS:AVER ON\n"); // Enable Filter mode - should see FILT on display
  Serial1.print(":SENS:AVER OFF\n"); // Disable Filter mode - let user manually enable if desired
  Serial1.print(":SENS:CURR:RANG:AUTO ON\n"); // Enable Auto Range Mode for Current Measurement 
  Serial1.print(":SENS:VOLT:RANG:AUTO ON\n"); // Enable Auto Range Mode for Voltage Measurement 
  Serial1.print(":SOUR:DEL 0.25\n"); // Delay 0.25 seconds after enable Source and before Measuring
  //@@@Serial1.print(":SOUR:DEL:AUTO ON\n"); // Auto Delay after enable Source and before Measuring

  if (MODE_LOAD_CURRENT_INCR != 0) {
    Serial1.print(":SOUR:CURR:RANG:AUTO ON\n");
  }

  if (MODE_LOAD_VOLTAGE_INCR != 0) {
    Serial1.print(":SOUR:VOLT:RANG:AUTO ON\n");
  }

  // Put instrument back in front panel input mode
  Serial1.print(":SYSTem:LOCal\n");
  
}


void setup() {

  // Setup the external pull-up pins as outputs to drive the pull-ups :
  pinMode(PULLUP_TIP_PIN,OUTPUT);
  pinMode(PULLUP_RING_PIN,OUTPUT);
  digitalWrite(PULLUP_TIP_PIN, HIGH);
  digitalWrite(PULLUP_RING_PIN, HIGH);

  // Setup the footswitches
  // (These are default if not set, but changeable for convenience)
  footswitch_tip.debounceTime   = 20;   // Debounce timer in ms
  footswitch_tip.multiclickTime = 250;  // Time limit for multi clicks
  footswitch_tip.longClickTime  = 1000; // Time until long clicks register
  footswitch_ring.debounceTime   = 20;   // Debounce timer in ms
  footswitch_ring.multiclickTime = 250;  // Time limit for multi clicks
  footswitch_ring.longClickTime  = 1000; // Time until long clicks register

  //Setup the LED :
  pinMode(LED_PIN,OUTPUT);

  // initialize serial with instrument:
  //@@@Serial1.begin(57600);
  Serial1.begin(19200);

  // initialize serial console
  if (DEBUG >= 1) Serial.begin(57600);

  // Send serial commands to Instrument to initialize it
  // 
  // NOTE: This requires the Keithley 2400 to be on with
  // the serial cable properly installed before powering the 
  // Arduino + footswitch. If power up in the wrong order, 
  // Keithley will not be set up properly. So simply power 
  // cycle the Arduino and make sure see the REM remote indicator
  // light up on the Keithley screen which indicates that it 
  // received its set up commands.
  setupInstrument();

  // Init sourceCurrent
  sourceCurrent = MODE_LOAD_CURRENT_START;

  // Init sourceVoltage
  sourceVoltage = MODE_LOAD_VOLTAGE_START;

  // Init sweepCount
  sweepCount = 0;

  // Init keyNumCount
  keyNumCount = 0;

  if (DEBUG >= 1) {
    while (!Serial) {
      ; // wait for serial port to connect. Needed for native USB port only
    }
  }
  
}

// When detect the falling edge of the footswitch, 
// tell the Keithley 2400 instrument to enable its 
// output, read 16 values, average them and return. 
// Then send this value to the keyboard to be input
// into a spreadsheet or what ever document has the 
// keyboard focus.
void loop() {
  char line[LINE_BUFFER_SIZE];
   
  // Update the Bounce instance :
  footswitch_tip.Update();
  footswitch_ring.Update();

  // If not reading a measurement, check the footswitch for single or double clicks
  if ( !readMeasurement ) {
    // Indicate NOT reading measurement
    digitalWrite(LED_PIN, LOW ); // RX LED ON

    // if sweepCount > 0, then auto change the source and take another reading
    if (sweepCount > 0) {
      readMeasurement = true;
      stringComplete = false;
      // Keithley appears to have a bug that requires turning the Filter off and then on to put it back in repeating mode.
      // So send commands before each reading to turn off and then on the filter. 
      //@@@Serial1.print(":SENS:AVER OFF\n"); 
      //@@@Serial1.print(":SENS:AVER ON\n"); 

      // Advance the current load, if desired
      if (MODE_LOAD_CURRENT_INCR != 0) {
        if (sourceCurrent < 0.00) {
          // Seems to mess up the first voltage measurement if sourceCurrent is 0.00, so skip it
          //@@@Serial1.print(":SOUR:CURR:RANG:AUTO ON\n");
          Serial1.print(":SOUR:CURR ");
          Serial1.print(sourceCurrent,7);
          Serial1.print("\n");
          if (DEBUG >= 1) { Serial.print(":SOUR:CURR "); Serial.print(sourceCurrent,7); Serial.print("\n");}
        }
        sourceCurrent = sourceCurrent - MODE_LOAD_CURRENT_INCR; // adjust load current
      }

      // Advance the voltage, if desired
      if (MODE_LOAD_VOLTAGE_INCR != 0) {
        //@@@if (sourceCurrent < 0.00) {
          //@@@@ Seems to mess up the first voltage measurement if sourceCurrent is 0.00, so skip it
          //@@@Serial1.print(":SOUR:VOLT:RANG:AUTO ON\n");
          Serial1.print(":SOUR:VOLT ");
          Serial1.print(sourceVoltage,6);
          Serial1.print("\n");
          if (DEBUG >= 1) { Serial.print(":SOUR:VOLT "); Serial.print(sourceVoltage,6); Serial.print("\n");}
        //@@@}
        sourceVoltage = sourceVoltage + MODE_LOAD_VOLTAGE_INCR; // adjust voltage
      }

      sweepCount = sweepCount - 1; // decrement sweep counter      
      Serial1.print(":READ?\n"); // tell Instrument to enable and measure
    } else {  
      // If see footswitch pressed, handle it
      switch ( footswitch_tip.clicks ) {
        case 1:
          readMeasurement = true;
          stringComplete = false;
          // Keithley appears to have a bug that requires turning the Filter off and then on to put it back in repeating mode.
          // So send commands before each reading to turn off and then on the filter. 
          //@@@Serial1.print(":SENS:AVER OFF\n"); 
          //@@@Serial1.print(":SENS:AVER ON\n"); 
  
          if (MODE_SWEEP > 0) {
            // start a current sweep of MODE_SWEEP measurements
            sweepCount = MODE_SWEEP;
            sourceCurrent = MODE_LOAD_CURRENT_START;
            if (DEBUG >= 1) { Serial.print("Starting Current Sweep of "); Serial.print(sweepCount); Serial.print(" steps\n");}
          } else if (MODE_SWEEP < 0) {
            // start a voltage sweep of MODE_SWEEP measurements
            sweepCount = -1*MODE_SWEEP;
            sourceVoltage = MODE_LOAD_VOLTAGE_START;
            if (DEBUG >= 1) { Serial.print("Starting Voltage Sweep of "); Serial.print(sweepCount); Serial.print(" steps\n");}
          }
          
          // Advance the current load, if desired
          if (MODE_LOAD_CURRENT_INCR != 0) {
            if (sourceCurrent < 0.00) {
              // Seems to mess up the first voltage measurement if sourceCurrent is 0.00, so skip it
              //@@@Serial1.print(":SOUR:CURR:RANG:AUTO ON\n");
              Serial1.print(":SOUR:CURR ");
              Serial1.print(sourceCurrent,7);
              Serial1.print("\n");
              if (DEBUG >= 1) { Serial.print(":SOUR:CURR "); Serial.print(sourceCurrent,7); Serial.print("\n");}
            }
            sourceCurrent = sourceCurrent - MODE_LOAD_CURRENT_INCR; // adjust load current
          }

          // Advance the voltage, if desired
          if (MODE_LOAD_VOLTAGE_INCR != 0) {
            //@@@if (sourceCurrent < 0.00) {
              //@@@@ Seems to mess up the first voltage measurement if sourceCurrent is 0.00, so skip it
              //@@@Serial1.print(":SOUR:VOLT:RANG:AUTO ON\n");
              Serial1.print(":SOUR:VOLT ");
              Serial1.print(sourceVoltage,6);
              Serial1.print("\n");
              if (DEBUG >= 1) { Serial.print(":SOUR:VOLT "); Serial.print(sourceVoltage,6); Serial.print("\n");}
            //@@@}
            sourceVoltage = sourceVoltage + MODE_LOAD_VOLTAGE_INCR; // adjust voltage
          }

          Serial1.print(":READ?\n"); // tell Instrument to enable and measure
          break;
  
        case 2: // double-click: undo the last entry
          keyMeasurementUndo(); 
          break;
  
        case 3: // triple-click: go to the next Excel line
          keyMeasurementLeftDown(keyNumCount); 
          break;
  
        default:
          // Either no click or too many clicks to have a defined action, so ignore 
          break;
      } 
    }
  } else {
    // Reading a Measurement, so handle the serial comm.
    if ( !stringComplete ) {
      // Indicate if reading measurement
      digitalWrite(LED_PIN, HIGH ); // RX LED OFF  

      if (read_line(line, sizeof(line)) < 0) {
        if (DEBUG >= 1) Serial.println("Error: line too long");
        return; // skip command processing and try again on next iteration of loop
      }
      stringComplete = true;

    } else {
      // string is complete, so process
      stringComplete = false;
      readMeasurement = false;
   
      // Indicate string is complete (@@@ DO WE NEED THIS?)
      digitalWrite(LED_PIN, LOW ); // RX LED ON

      int measureType;
      float volt, curr, data;
      
      decodeMeasurement(line, measureType, volt, curr);

      // copy the measured value to data
      if (measureType == UNITS_CURR) {
        data = curr;
      } else {
        data = volt;
      }

      if (DEBUG >= 1) {
        char datastr[20];
        Serial.print("Data = ");
        Serial.println(dtostre(data, datastr, 6, DTOSTR_UPPERCASE)); // output to console for debug
        Serial.print("Data = ");
        Serial.println(dtostrf(data, 9, 6, datastr)); // output to console for debug
      }
  
      // Send received data out to the keyboard to be entered into a spreadsheet or other document
      keyMeasurement(data, measureType);

      // CHANGE ME: Set to "if (0)" if do not want the second value output
      if (0) {
        // Return the other data value as well
        if (measureType == UNITS_VOLT) {
          if (curr < 1e37) { // first check that the returned value is not invalid
            keyMeasurement(curr, UNITS_CURR);
            if (MODE_LEFT_LEFT_DOWN) keyMeasurementLeftDown(); // move cursor down and to the left twice to reposition for next two values
          }
        } else if (measureType == UNITS_CURR) {
          if (volt < 1e37) { // first check that the returned value is not invalid
            keyMeasurement(volt, UNITS_VOLT);
            if (MODE_LEFT_LEFT_DOWN) keyMeasurementLeftDown(); // move cursor down and to the left twice to reposition for next two values
          }
        }
      }
  
      // Put instrument back in front panel input mode if not doing a sweep
      if (sweepCount <= 0) Serial1.print(":SYSTem:LOCal\n");

    }
  }
}


