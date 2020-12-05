/* Bluetooth config */

// Throttle 2 controller service UUID string: 8877FB19-E00B-40BE-905E-ACB42C39E6B8
static uint8_t t2cs_uuid_bytes[16u] = {0x88,0x77,0xFB,0x19,0xE0,0x0B,0x40,0xBE,0x90,0x5E,0xAC,0xB4,0x2C,0x39,0xE6,0xB8};
// Throttle value char UUID string: 5A57F691-C0B9-45DD-BDF1-279681212C29
static uint8_t t2ctc_uuid_bytes[16u] = {0x5A,0x57,0xF6,0x91,0xC0,0xB9,0x45,0xDD,0xBD,0xF1,0x27,0x96,0x81,0x21,0x2C,0x29};
// Armed status UUID string: 28913A56-5701-4B27-85DB-50985F224847
static uint8_t t2cac_uuid_bytes[16u] = {0x28,0x91,0x3A,0x56,0x57,0x01,0x4B,0x27,0x85,0xDB,0x50,0x98,0x5F,0x22,0x48,0x47};

BLEService        t2cs = BLEService(t2cs_uuid_bytes);
BLECharacteristic t2ctc = BLECharacteristic(t2ctc_uuid_bytes);
BLECharacteristic t2cac = BLECharacteristic(t2cac_uuid_bytes);

BLEDfu bledfu;

uint16_t bps = 0;

void setupBle() {
  //while ( !Serial ) delay(10);   // for nrf52840 with native usb

  Serial.println("Setting up Ble");

  Bluefruit.begin();

  // Set the advertised device name (keep it short!)
  Bluefruit.setName("OpenPPG Controller");
  // To be consistent OTA DFU should be added first if it exists
  bledfu.begin();

  // Set the connect/disconnect callback handlers
  Bluefruit.Periph.setConnectCallback(connect_callback);
  Bluefruit.Periph.setDisconnectCallback(disconnect_callback);

  // Configure and Start the Device Information Service
  Serial.println("Configuring the Device Information Service");
  bledis.setManufacturer("OpenPPG");
  bledis.setModel("Controller V2");
  bledis.begin();

  // Start the BLE Battery Service and set it to 0 by default
  Serial.println("Configuring the Battery Service");
  blebas.begin();
  blebas.write(100); //TODO set to battery

  // Setup the Throttle service using
  // BLEService and BLECharacteristic classes
  Serial.println("Configuring the Throttle Service");
  setupThrottleBle();

  // Setup the advertising packet(s)
  Serial.println("Setting up the advertising payload(s)");
  startAdv();

  Serial.println("Ready - Advertising");
}


void startAdv(void) {
  // Advertising packet
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();

  // Include HRM Service UUID
  Bluefruit.Advertising.addService(t2cs);

  // Include Name
  Bluefruit.Advertising.addName();

  /* Start Advertising
   * - Enable auto advertising if disconnected
   * - Interval:  fast mode = 20 ms, slow mode = 152.5 ms
   * - Timeout for fast mode is 30 seconds
   * - Start(timeout) with timeout = 0 will advertise forever (until connected)
   *
   * For recommended advertising interval
   * https://developer.apple.com/library/content/qa/qa1931/_index.html
   */
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244);    // in unit of 0.625 ms
  Bluefruit.Advertising.setFastTimeout(30);      // number of seconds in fast mode
  Bluefruit.Advertising.start(0);                // 0 = Don't stop advertising after n seconds
}

void setupThrottleBle(void) {
  t2cs.begin();

  // Note: You must call .begin() on the BLEService before calling .begin() on
  // any characteristic(s) within that service definition.. Calling .begin() on
  // a BLECharacteristic will cause it to be added to the last BLEService that
  // was 'begin()'ed!

  t2ctc.setProperties(CHR_PROPS_NOTIFY);
  t2ctc.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  t2ctc.setFixedLen(sizeof(STR_BLE_TCTRL2REC_MSG));
  t2ctc.setCccdWriteCallback(cccd_callback);  // Optionally capture CCCD updates
  t2ctc.begin();
  // uint8_t hrmdata[2] = { 0b00000110, 0x40 }; // Set the characteristic to use 8-bit values, with the sensor connected and detected
  t2ctc.write((uint8_t*)&controlData, sizeof(STR_BLE_TCTRL2REC_MSG));
}

void connect_callback(uint16_t conn_handle) {
  // Get the reference to current connection
  BLEConnection* connection = Bluefruit.Connection(conn_handle);

  char central_name[32] = { 0 };
  connection->getPeerName(central_name, sizeof(central_name));

  Serial.print("Connected to ");
  Serial.println(central_name);
}

/**
 * Callback invoked when a connection is dropped
 * @param conn_handle connection where this event happens
 * @param reason is a BLE_HCI_STATUS_CODE which can be found in ble_hci.h
 */
void disconnect_callback(uint16_t conn_handle, uint8_t reason) {
  (void) conn_handle;
  (void) reason;

  Serial.print("Disconnected, reason = 0x"); Serial.println(reason, HEX);
  Serial.println("Advertising!");
}

void cccd_callback(uint16_t conn_hdl, BLECharacteristic* chr, uint16_t cccd_value) {
    // Display the raw request packet
    Serial.print("CCCD Updated: ");
    //Serial.printBuffer(request->data, request->len);
    Serial.println(cccd_value);

    // Check the characteristic this CCCD update is associated with in case
    // this handler is used for multiple CCCD records.
    if (chr->uuid == t2ctc.uuid) {
        if (chr->notifyEnabled(conn_hdl)) {
            Serial.println("Measurement 'Notify' enabled");
        } else {
            Serial.println(" Measurement 'Notify' disabled");
        }
    }
}

void throttleBleLoop(STR_BLE_TCTRL2REC_MSG data) {
  Serial.print("sending ");
  Serial.println(data.throttlePercent);
  if ( Bluefruit.connected() ) {
    // Note: We use .notify instead of .write!
    // If it is connected but CCCD is not enabled
    // The characteristic's value is still updated although notification is not sent
    if ( t2ctc.notify((uint8_t*)&data, sizeof(STR_BLE_TCTRL2REC_MSG)) ) {
      Serial.println("Data updated");
    } else {
      Serial.println("ERROR: Notify not set in the CCCD or not connected!");
    }
  } else {
    // Serial.println("NC");
  }
}
