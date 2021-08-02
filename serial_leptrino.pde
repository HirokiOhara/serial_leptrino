import processing.serial.*; //<>// //<>//

PrintWriter file;
Serial myPort;
int time; // [msec]
float[] ratedValueList = new float[6];
boolean calib;
float[] calibValueList = new float[6];
/* res_: represent Variables of Response */
byte res_bcc = 0x00;
int res_id = 0;
int res_state = 0;
byte[] res_data = new byte[0];
/* cc_: Control Character */
byte[] cc_start = new byte[]{0x10, 0x02};  // DEL STX
byte[] cc_end = new byte[]{0x10, 0x03};    // DEL ETX
/* Command List */
byte[] checkProductInfo = {0x04, (byte)0xFF, 0x2A, 0x00};
byte[] checkRatedValue = {0x04, (byte)0xFF, 0x2B, 0x00};
byte[] checkDigitalFilter = {0x04, (byte)0xFF, (byte)0xB6, 0x00};
byte[] getHandShake = {0x04, (byte)0xFF, 0x30, 0x00};
byte[] getData = {0x04, (byte)0xFF, 0x32, 0x00};
byte[] stopData = {0x04, (byte)0xFF, 0x33, 0x00};
byte filter = 0x00;
// OFF  : 0x00;
// 10Hz : 0x01;
// 30Hz : 0x02;
// 100Hz: 0x03;
byte[] setDigitalFilter = {0x08, (byte)0xFF, (byte)0xA6, filter, 0x00, 0x00, 0x00};

void setup() {
  myPort = portOpen();
  file = createWriter("test.csv");
  time = 1000;
  sendMsg(myPort, checkRatedValue);
  //sendMsg(myPort, checkProductInfo);
}

void draw() {
}

void mousePressed() {
  calib = true;
  sendMsg(myPort, getData);
  delay(time);  // ここで間隔を調整
  sendMsg(myPort, stopData);
  file.flush();
}


// Called when data is available.
void serialEvent(Serial p) {
  byte inByte = (byte)p.read();
  if (res_state == 0) {
    if (inByte == cc_start[0]) {
      res_state += 1;
    }
  } else if (res_state == 1) {
    if (inByte == cc_start[1]) {
      res_state += 1;
    }
  } else if (res_state == 2) {
    res_data = new byte[inByte];
    res_data[res_id] = inByte;
    res_bcc ^= inByte;
    res_id += 1;
    res_state += 1;
  } else if (res_state == 3) {
    res_data[res_id] = inByte;
    res_bcc ^= inByte;
    res_id += 1;
    if (res_id == res_data.length)
      res_state += 1;
  } else if (res_state == 4) {
    if (inByte == cc_end[0]) {
      res_state += 1;
    }
  } else if (res_state == 5) {
    if (inByte == cc_end[1]) {
      res_bcc ^= inByte;
      res_state += 1;
    }
  } else if (res_state == 6) {
    if (res_bcc == inByte) {
      println("> bcc of " + hex(res_data[2]) + " --- greate!");
      calForce(res_data);
      for (int i = 0; i < res_data.length; i++) {
        file.print(hex(res_data[i]));
      }
      file.print("\n");
    } else
      println("> bcc of " + hex(res_data[2]) + " --- error");
    res_bcc = 0x00;
    res_id = 0;
    res_state = 0;
  }
}



/* User's defined functions below */

Serial portOpen() {
  // List all the available serial ports:
  printArray(Serial.list());
  // Open the port you are using at the rate you want:
  return new Serial(this, Serial.list()[5], 460800);
}

void sendMsg(Serial p, byte[] cmd) {
  byte bcc = 0x00;
  int msg_id = 0;
  int msg_length = cc_start.length + cmd.length + cc_end.length + 1;
  byte[] msg_lst = new byte[msg_length];
  for (int i = 0; i < cc_start.length; i++) {
    msg_lst[i] = cc_start[i];
    msg_id += 1;
  }
  for (int i = 0; i < cmd.length; i++) {
    msg_lst[msg_id] = cmd[i];
    bcc ^= cmd[i];
    msg_id += 1;
  }
  for (int i = 0; i < cc_end.length; i++) {
    msg_lst[msg_id] = cc_end[i];
    if (cc_end[i] != 0x10)
      bcc ^= cc_end[i];
    msg_id += 1;
  }
  msg_lst[msg_id] = bcc;
  p.write(msg_lst);
}


void calForce(byte[] res) {
  byte cmd_type = res[2];
  if (cmd_type == 0x2B) {
    for (int i = 0; i < 6; i++) {
      String bi = binary(res[7 + 4*i]) + binary(res[6 + 4*i]) + binary(res[5 + 4*i]) + binary(res[4 + 4*i]);
      Integer sign = int(bi.substring(0, 1));
      String power = str(unbinary(bi.substring(1, 9)) - 127);
      String sig = bi.substring(9);
      float F = unbinary(str(int(float(1+ "." + sig) * pow(10, float(power)))));
      if (sign == 0) {
        //println(F);
      } else {
        F -= F;
        //println(F);
      }
      ratedValueList[i] = F;
    }
  } else if (cmd_type == 0x30 || res_data[0] == 0x14 && cmd_type == 0x32) {
    for (int j = 0; j < 6; j++) {
      String bi = binary(res[5 + 2*j]) + binary(res[4 + 2*j]);
      float F = unbinary(bi);
      if (int(bi.substring(0, 1)) == 1) {
        float comp = unbinary("10000000000000000");
        F = comp - F + 1;
      }
      if (abs(F) <= 10000) {
        if (calib) {
          calibValueList[j] = ratedValueList[j] * F/10000;
        }
        println(ratedValueList[j] * F/10000 - calibValueList[j]);
      } else
        println("error");
    }
    if (calib) {
      calib = false;
    }
  }
}
