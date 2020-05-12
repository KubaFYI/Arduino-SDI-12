/*
 * Example L:  SDI-12 slave for Zephyr testing
 * 
 * A quick sketch based mostly on example H of the EnviroDIY's SDI-12 library.
 * Written to provide a slave device enabling testing of the Zephyr's SDI-12 
 * master library.
 * 
 * The slave behaves differently depending on its address which allows the same
 * slave device to be used for testing with various conditions.
 *
 */


#include <SDI12.h>

#define DATA_PIN 14         // The pin of the SDI-12 data bus
#define POWER_PIN -1       // The sensor power pin (or -1 if not switching power)

enum crc_e {
  CORRECT,
  INCORRECT,
  NONE
};

char sensorAddress = '0';
int state = 0;

int values_number;
int wait_seconds;
int service_req_wait;
int meas_init_time;
int every_second;
int no_response;

bool service_req_sent;
crc_e crc_status;

#define CHAR_BUFFER_SIZE 100

char crc[4];
char char_buffer[CHAR_BUFFER_SIZE];
String buffer;

#define WAIT 0
#define INITIATE_CONCURRENT 1
#define INITIATE_MEASUREMENT 2

// Create object by which to communicate with the SDI-12 bus on SDIPIN
SDI12 slaveSDI12(DATA_PIN);

void pollSensor(float* measurementValues) {
  measurementValues[0] =  1.11;
  measurementValues[1] = -2.22;
  measurementValues[2] =  3.33;
  measurementValues[3] = -4.44;
  measurementValues[4] =  5.55;
  measurementValues[5] = -6.66;
  measurementValues[6] =  7.77;
  measurementValues[7] = -8.88;
  measurementValues[8] =  9.99;
}

void sdi_12_calc_crc_ascii(char *cmd, uint8_t cmd_len, char* crc_str)
{
  uint16_t crc_num = 0;
  int idx, jdx;

  for ( idx=0; idx<cmd_len; idx++ ){
    crc_num ^= cmd[idx];
    for ( jdx=0; jdx<8; jdx++ ){
      if ( (crc_num & 0x0001) == 0x0001){
        crc_num >>= 1;
        crc_num ^= 0xA001;
      } else {
        crc_num >>= 1;
      }
    }
  }

  crc_str[0] = 0x40 | (crc_num >> 12);
  crc_str[1] = 0x40 | ((crc_num >> 6) & 0x3F);
  crc_str[2] = 0x40 | (crc_num & 0x3F);
  crc_str[3] = '\0';
}


void parseSdi12Cmd(String command, String* dValues) {
/* Ingests a command from an SDI-12 master, sends the applicable response, and
 * (when applicable) sets a flag to initiate a measurement
 */
  // First char of command is always either (a) the address of the device being
  // probed OR (b) a '?' for address query.
  // Do nothing if this command is addressed to a different device
  Serial.println("parseSdi12Cmd");
  if (command.charAt(0) != sensorAddress && command.charAt(0) != '?') {
    Serial.println("exit parseSdi12Cmd -> bad start");
    return;
  }

  if ( every_second != -1 ) {
    if ( every_second == 0 ) {
      Serial.printf("processing (every second resp)\n");
      every_second = 1;
    } else {
      Serial.printf("not processing (every second resp)\n");
      every_second = 0;
      return;
    }
  }

  no_response = 0;

  // If execution reaches this point, the slave should respond with something in
  // the form:   <address><responseStr><Carriage Return><Line Feed>
  // The following if-switch-case block determines what to put into <responseStr>,
  // and the full response will be constructed afterward. For '?!' (address query)
  // or 'a!' (acknowledge active) commands, responseStr is blank so section is skipped
  String responseStr = "";
  if (command.length() > 1) {
    switch (command.charAt(1)) {
    case '!':
        responseStr = "";
        break;
    case 'I':
        // Identify command
        // Slave should respond with ID message: 2-char SDI-12 version + 8-char
        // company name + 6-char sensor model + 3-char sensor version + 0-13 char S/N
        responseStr = "11ZEPHYRIO0000011.0OtherInfo"; // Substitute proper ID String here
        break;
      case 'C':
        Serial.printf("Concurrent measurement not supported\n");
        responseStr = "00000";  // 9 values ready in 21 sec; Substitue sensor-specific values here
        // It is not preferred for the actual measurement to occur in this subfunction,
        // because doing to would hold the main program hostage until the measurement
        // is complete.  Instead, we'll just set a flag and handle the measurement elsewhere.
        break;
      case 'M':
        // Initiate measurement command
        service_req_sent = false;
        switch (sensorAddress) {
        case '0':
          values_number = 3;
          wait_seconds = 0;
          service_req_wait = -1;
          every_second = -1;
          no_response = 0;
          crc_status = command.charAt(2) == 'C' ? CORRECT : NONE;
          break;
        case '1':
          values_number = 3;
          wait_seconds = 10;
          service_req_wait = -1;
          every_second = -1;
          no_response = 0;
          crc_status = command.charAt(2) == 'C' ? CORRECT : NONE;
          break;
        case '2':
          values_number = 9;
          wait_seconds = 0;
          service_req_wait = -1;
          every_second = -1;
          no_response = 0;
          crc_status = command.charAt(2) == 'C' ? CORRECT : NONE;
          break;
        case '3':
          values_number = 3;
          wait_seconds = 10;
          service_req_wait = 5;
          every_second = -1;
          no_response = 0;
          crc_status = command.charAt(2) == 'C' ? INCORRECT : NONE;
          break;
        case '4':
          values_number = 3;
          wait_seconds = 0;
          service_req_wait = -1;
          every_second = -1;
          no_response = 0;
          crc_status = command.charAt(2) == 'C' ? CORRECT : NONE;
          break;
        case '5':
          values_number = 3;
          wait_seconds = 0;
          service_req_wait = -1;
          every_second = 1;
          no_response = 0;
          crc_status = command.charAt(2) == 'C' ? CORRECT : NONE;
          break;
        case '6':
          no_response = 1;
          crc_status = command.charAt(2) == 'C' ? CORRECT : NONE;
          break;
        default:
          values_number = 1;
          wait_seconds = 0;
          service_req_wait = -1;
          every_second = -1;
          no_response = 0;
          crc_status = command.charAt(2) == 'C' ? CORRECT : NONE;
          break;

        }

        sprintf(char_buffer, "%03d%01d", wait_seconds, values_number);
        responseStr = String(char_buffer);

        // It is not preferred for the actual measurement to occur in this subfunction,
        // because doing to would hold the main program hostage until the measurement
        // is complete.  Instead, we'll just set a flag and handle the measurement elsewhere.
        // It is preferred though not required that the slave send a service request upon
        // completion of the measurement.  This should be handled in the main loop().
        state = INITIATE_MEASUREMENT;
        break;
        // NOTE: "aM1...9!" commands may be added by duplicating this case and adding
        //       additional states to the state flag

      case 'D':
        // Send data command
        // Slave should respond with a String of values
        // Values to be returned must be split into Strings of 35 characters or fewer
        // (75 or fewer for concurrent).  The number following "D" in the SDI-12 command
        // specifies which String to send
        responseStr = dValues[(int)command.charAt(2)-48];
        break;
      case 'A':
        // Change address command
        // Slave should respond with blank message (just the [new] address + <CR> + <LF>)
        sensorAddress = command.charAt(2);
        break;
      default:
        // Mostly for debugging; send back UNKN if unexpected command received
        responseStr = "UNKN";
        break;
    }
  }

  // Issue the response speficied in the switch-case structure above.
  buffer = String(sensorAddress) + responseStr;
  buffer.toCharArray(char_buffer, CHAR_BUFFER_SIZE);

  if (command.charAt(1) == 'M' ||
      command.charAt(1) == 'D') {
    if (crc_status == CORRECT) {
      sdi_12_calc_crc_ascii(char_buffer, strlen(char_buffer), crc);
    } else if (crc_status == INCORRECT) {
      strcpy(crc, "ABC");
    } else {
      strcpy(crc, "");
    }
  } else {
    strcpy(crc, "");
  }
  String resp_str = buffer + String(crc) + "\r\n";

  delay(5);

  if(no_response) {
    Serial.printf("intentionally sending no response");
  } else {
    Serial.printf("resp: %s\n", resp_str.c_str());
    slaveSDI12.sendResponse(resp_str);
  }

  Serial.printf("Processed %s\n", command.c_str());
}


void formatOutputSDI(float* measurementValues, String* dValues,
                     unsigned int maxChar, int maxValInD, int valsToSend) {
/* Ingests an array of floats and produces Strings in SDI-12 output format */

  dValues[0] = "";
  int j = 0;

  // upper limit on i should be number of elements in measurementValues
  for (int i=0; i<valsToSend; i++) {
    // Read float value "i" as a String with 6 deceimal digits
    // (NOTE: SDI-12 specifies max of 7 digits per value; we can only use 6
    //  decimal place precision if integer part is one digit)
    String valStr = String(measurementValues[i],2);
    // Explictly add implied + sign if non-negative
    if (valStr.charAt(0) != '-') { valStr = '+' + valStr; }
    // Append dValues[j] if it will not exceed 35 (aM!) or 75 (aC!) characters
    if (dValues[j].length() + valStr.length() < maxChar) { dValues[j] += valStr; }
    // Start a new dValues "line" if appending would exceed 35/75 characters
    else { dValues[++j] = valStr; }
  }

  // Fill rest of dValues with blank strings
  while (j<9) { dValues[++j] = ""; }
}

void setup() {
  slaveSDI12.begin();
  delay(500);
  slaveSDI12.forceListen();  // sets SDIPIN as input to prepare for incoming message
  Serial.begin(9600);
  delay(500);
  Serial.print("Starting...\n");
  every_second = -1;
}

void loop() {

  static float measurementValues[9]; // 9 floats to hold simulated sensor data
  static String dValues[10];  // 10 String objects to hold the responses to aD0!-aD9! commands
  static String commandReceived = "";  // String object to hold the incoming command

  // If a byte is available, an SDI message is queued up. Read in the entire message
  // before proceding.  It may be more robust to add a single character per loop()
  // iteration to a static char buffer; however, the SDI-12 spec requires a precise
  // response time, and this method is invariant to the remaining loop() contents.
  int avail = slaveSDI12.available();
  if (avail < 0) { slaveSDI12.clearBuffer(); } // Buffer is full; clear
  else if (avail > 0) {
    Serial.printf("avail: %d\n", avail);
    for(int a = 0; a < avail; a++){
      char charReceived = slaveSDI12.read();
      Serial.printf("char: 0x%x (%c)\n", charReceived, charReceived);
      // Character '!' indicates the end of an SDI-12 command; if the current
      // character is '!', stop listening and respond to the command
      if (charReceived == '\x0a') {
        // Command string is completed; do something with it
        Serial.printf("rcv: %s\n", commandReceived.c_str());
        parseSdi12Cmd(commandReceived, dValues);
        // Clear command string to reset for next command
        commandReceived = "";
        // '!' should be the last available character anyway, but exit the "for" loop
        // just in case there are any stray characters
        slaveSDI12.clearBuffer();
        break;
      }
      // If the current character is anything but '!', it is part of the command
      // string.  Append the commandReceived String object.
      else{
        // Append command string with new character
        commandReceived += String(charReceived);
      }
    }
  }

  // For aM! and aC! commands, parseSdi12Cmd will modify "state" to indicate that
  // a measurement should be taken
  switch (state) {
    case WAIT:
      if ( millis() - meas_init_time > wait_seconds * 1000 ) {
        formatOutputSDI(measurementValues, dValues, 75, 3, values_number);
      } else if ( service_req_wait > 0 && !service_req_sent &&
                  millis() - meas_init_time > service_req_wait * 1000) {
        Serial.printf("Sending Service request!");
        formatOutputSDI(measurementValues, dValues, 75, 3, values_number);
        slaveSDI12.sendResponse(String(sensorAddress) + "\r\n");
        delay(25+10);
        service_req_sent = true;
      }
      break;
    case INITIATE_CONCURRENT:
    case INITIATE_MEASUREMENT:
      pollSensor(measurementValues);
      meas_init_time = millis();
      state = WAIT;
      formatOutputSDI(measurementValues, dValues, 75, 3, 0);
      break;
  }
}