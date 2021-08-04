/* //<>// //<>//
 Designed by Hiroki OHARA
 MIT license
 
 From lefty's view,
 X-axis: to the beyond
 Y-axis: to the down side
 Z-axis: to the right side */


import processing.serial.*;

PrintWriter file;
Serial myPort;
int time; // [msec]
int counter;  // output.csvのインデックス用
float[] rated_value_list = new float[6];
boolean init;
float[] init_value_list = new float[6];
float[] calculated_list = new float[6];
boolean running = false;
/*
 利き手に合わせて"lefty"の値を変更する */
boolean lefty = false;
/*
 Check the connection between PC and sensor */
boolean connecting = false;
/*
 res_: represent Variables of Response */
byte res_bcc = 0x00;
int res_id = 0;
int res_state = 0;
byte[] res_data = new byte[0];
/*
 cc_: Control Character */
byte[] ccStart = new byte[]{0x10, 0x02};  // DEL STX
byte[] ccEnd = new byte[]{0x10, 0x03};    // DEL ETX
/*
 Command List */
byte[] checkProductInfo = {0x04, (byte)0xFF, 0x2A, 0x00};
byte[] checkRatedValue = {0x04, (byte)0xFF, 0x2B, 0x00};
byte[] checkDigitalFilter = {0x04, (byte)0xFF, (byte)0xB6, 0x00};
byte[] getHandShake = {0x04, (byte)0xFF, 0x30, 0x00};
byte[] getData = {0x04, (byte)0xFF, 0x32, 0x00};
byte[] stopData = {0x04, (byte)0xFF, 0x33, 0x00};
/* 
 OFF  : 0x00
 10Hz : 0x01
 30Hz : 0x02
 100Hz: 0x03 */
byte filter = 0x00;
byte[] setDigitalFilter = {0x08, (byte)0xFF, (byte)0xA6, filter, 0x00, 0x00, 0x00};


void setup() {
  if (Serial.list().length >= 5)
    connecting = true;
  if (connecting) {
    myPort = portOpen();
    myPort.clear();
    sendMsg(myPort, checkRatedValue);
    //sendMsg(myPort, checkProductInfo);
  }
  file = createWriter("test.csv");
  time = 5000;


  /* For Graphic Animation */
  size(400, 400, P3D);
  background(195, 0, 16);
  if (lefty)
    camera(70.0, 35.0, 120.0, width/2, height/2, 0, 0.0, 1.0, 0.0);
  else
    camera(width - 70.0, 35.0, 120.0, width/2, height/2, 0, 0.0, 1.0, 0.0);
  frameRate(60);
}

void draw() {
  background(255);
  translate(width/2, height/2, 0);
  fill(255, 255, 255);
  stroke(255);
  box(300, 0, 300);
  stroke(0);
  sphereDetail(12, 6);
  stroke(230);
  noFill();
  sphere(100);

  pushMatrix();
  block();
  popMatrix();
  pushMatrix();
  xy();
  popMatrix();
  pushMatrix();
  yz();
  popMatrix();
  pushMatrix();
  zx();
  popMatrix();
}


void mousePressed() {
  if (!running && connecting) {
    running = true;
    thread("SubThread");
  }
  file.flush();
}


void SubThread() {
  while (running) {
    float cur = millis();
    init = true;
    counter = 0;
    sendMsg(myPort, getData);
    while (millis() - cur < time);
    sendMsg(myPort, stopData);
    running = false;
  }
}

/*
 Called when data is available.　*/
void serialEvent(Serial p) {
  byte inByte = (byte)p.read();
  switch(res_state) {
  case 0:
    if (inByte == ccStart[0]) {
      res_state += 1;
    }
    break;
  case 1:
    if (inByte == ccStart[1]) {
      res_state += 1;
    }
    break;
  case 2:
    res_data = new byte[inByte];
    res_data[res_id] = inByte;
    res_bcc ^= inByte;
    res_id += 1;
    res_state += 1;
    break;
  case 3:
    res_data[res_id] = inByte;
    res_bcc ^= inByte;
    res_id += 1;
    if (res_id == res_data.length)
      res_state += 1;
    break;
  case 4:
    if (inByte == ccEnd[0]) {
      res_state += 1;
    }
    break;
  case 5:
    if (inByte == ccEnd[1]) {
      res_bcc ^= inByte;
      res_state += 1;
    }
    break;
  case 6:
    if (res_bcc == inByte) {
      println("> bcc of " + hex(res_data[2]) + " --- greate!");
      calForce(res_data);
    } else
      println("> bcc of " + hex(res_data[2]) + " --- error");
    res_bcc = 0x00;
    res_id = 0;
    res_state = 0;
    break;
  }
}



/*
 User's defined functions below */

Serial portOpen() {
  // List all the available serial ports:
  printArray(Serial.list());
  // Open the port you are using at the rate you want:
  return new Serial(this, Serial.list()[5], 460800);
}


void sendMsg(Serial p, byte[] cmd) {
  byte bcc = 0x00;
  int msg_id = 0;
  int msg_length = ccStart.length + cmd.length + ccEnd.length + 1;
  byte[] msg_lst = new byte[msg_length];
  for (int i = 0; i < ccStart.length; i++) {
    msg_lst[i] = ccStart[i];
    msg_id += 1;
  }
  for (int i = 0; i < cmd.length; i++) {
    msg_lst[msg_id] = cmd[i];
    bcc ^= cmd[i];
    msg_id += 1;
  }
  for (int i = 0; i < ccEnd.length; i++) {
    msg_lst[msg_id] = ccEnd[i];
    if (ccEnd[i] != 0x10)
      bcc ^= ccEnd[i];
    msg_id += 1;
  }
  msg_lst[msg_id] = bcc;
  p.write(msg_lst);
}


void calForce(byte[] res) {
  byte cmd_type = res[2];
  switch(cmd_type) {
  case 0x2B:
    for (int i = 0; i < 6; i++) {
      String bi = binary(res[7 + 4*i]) + binary(res[6 + 4*i]) + binary(res[5 + 4*i]) + binary(res[4 + 4*i]);
      Integer sign = int(bi.substring(0, 1));
      String power = str(unbinary(bi.substring(1, 9)) - 127);
      String sig = bi.substring(9);
      float F = unbinary(str(int(float(1+ "." + sig) * pow(10, float(power)))));
      /* 
       sign=1は負の数 */
      if (sign == 1) {
        F -= F;
      }
      rated_value_list[i] = F;
    }
    break;
  case 0x30:
    String output_single = str(counter);
    for (int j = 0; j < 6; j++) {
      String bi = binary(res[5 + 2*j]) + binary(res[4 + 2*j]);
      float F = unbinary(bi);
      if (int(bi.substring(0, 1)) == 1) {  // Two's Complement
        float comp = unbinary("10000000000000000"); // 17 bits
        F = - (comp - F + 1);
      }
      if (abs(F) <= 10000) {
        if (init) {
          init_value_list[j] = rated_value_list[j] * F/10000;
        }
        //println(rated_value_list[j] * F/10000 - init_value_list[j]);
        calculated_list[j] = rated_value_list[j] * F/10000 - init_value_list[j];
        output_single +=  "," + calculated_list[j];
      } else
        println("error");
    }
    file.println(output_single);
    counter += 1;
    if (init) {
      init = false;
    }
    break;
  case 0x32:
    if (res_data[0] == 0x14) {
      String output_serial = str(counter);
      for (int j = 0; j < 6; j++) {
        String bi = binary(res[5 + 2*j]) + binary(res[4 + 2*j]);
        float F = unbinary(bi);
        if (int(bi.substring(0, 1)) == 1) {  // Two's Complement
          float comp = unbinary("10000000000000000"); // 17 bits
          F = - (comp - F + 1);
        }
        if (abs(F) <= 10000) {
          if (init) {
            init_value_list[j] = rated_value_list[j] * F/10000;
          }
          //println(rated_value_list[j] * F/10000 - init_value_list[j]);
          calculated_list[j] = rated_value_list[j] * F/10000 - init_value_list[j];
          output_serial +=  "," + calculated_list[j];
        } else
          println("error");
      }
      file.println(output_serial);
      counter += 1;
      if (init) {
        init = false;
      }
    }
    break;
  }
}

void xy() {
  rotateZ(- 100 * calculated_list[3]*PI/180.0);
  sphereDetail(10);
  stroke(200);
  noFill();
  //noStroke();
  //fill(255, 0, 0);

  pushMatrix();
  translate(-100 * sqrt(3) / 2, 50, 0);
  sphere(10);
  popMatrix();

  pushMatrix();
  translate(50, -100 * sqrt(3) / 2, 0);
  sphere(10);
  popMatrix();

  pushMatrix();
  translate(-100 * sqrt(3) / 2, -50, 0);
  sphere(10);
  popMatrix();

  pushMatrix();
  translate(-50, -100 * sqrt(3) / 2, 0);
  sphere(10);
  popMatrix();

  pushMatrix();
  translate(100 * sqrt(3) / 2, -50, 0);
  sphere(10);
  popMatrix();

  pushMatrix();
  translate(-50, 100 * sqrt(3) / 2, 0);
  sphere(10);
  popMatrix();

  pushMatrix();
  translate(100 * sqrt(3) / 2, 50, 0);
  sphere(10);
  popMatrix();

  pushMatrix();
  translate(50, 100 * sqrt(3) / 2, 0);
  sphere(10);
  popMatrix();

  pushMatrix();
  translate(100, 0, 0);
  sphere(10);
  popMatrix();

  pushMatrix();
  translate(-100, 0, 0);
  sphere(10);
  popMatrix();

  pushMatrix();
  translate(0, 100, 0);
  sphere(10);
  popMatrix();

  pushMatrix();
  translate(0, -100, 0);
  sphere(10);
  popMatrix();
}

void yz() {
  rotateX(- 100 * calculated_list[4]*PI/180.0);
  sphereDetail(10);
  stroke(255, 95, 0);
  noFill();
  //noStroke();
  //fill(0, 255, 0);

  pushMatrix();
  translate(0, -100 * sqrt(3) / 2, 50);
  sphere(10);
  popMatrix();

  pushMatrix();
  translate(0, 50, -100 * sqrt(3) / 2);
  sphere(10);
  popMatrix();

  pushMatrix();
  translate(0, -100 * sqrt(3) / 2, -50);
  sphere(10);
  popMatrix();

  pushMatrix();
  translate(0, -50, -100 * sqrt(3) / 2);
  sphere(10);
  popMatrix();

  pushMatrix();
  translate(0, 100 * sqrt(3) / 2, -50);
  sphere(10);
  popMatrix();

  pushMatrix();
  translate(0, -50, 100 * sqrt(3) / 2);
  sphere(10);
  popMatrix();

  pushMatrix();
  translate(0, 100 * sqrt(3) / 2, 50);
  sphere(10);
  popMatrix();

  pushMatrix();
  translate(0, 50, 100 * sqrt(3) / 2);
  sphere(10);
  popMatrix();

  pushMatrix();
  translate(0, 100, 0);
  sphere(10);
  popMatrix();

  pushMatrix();
  translate(0, -100, 0);
  sphere(10);
  popMatrix();

  pushMatrix();
  translate(0, 0, 100);
  sphere(10);
  popMatrix();

  pushMatrix();
  translate(0, 0, -100);
  sphere(10);
  popMatrix();
}

void zx() {
  rotateY(100 * calculated_list[5]*PI/180.0);
  sphereDetail(10);
  stroke(100);
  noFill();

  pushMatrix();
  translate(-100 * sqrt(3) / 2, 0, 50);
  sphere(10);
  popMatrix();

  pushMatrix();
  translate(50, 0, -100 * sqrt(3) / 2);
  sphere(10);
  popMatrix();

  pushMatrix();
  translate(-100 * sqrt(3) / 2, 0, -50);
  sphere(10);
  popMatrix();

  pushMatrix();
  translate(-50, 0, -100 * sqrt(3) / 2);
  sphere(10);
  popMatrix();

  pushMatrix();
  translate(100 * sqrt(3) / 2, 0, -50);
  sphere(10);
  popMatrix();

  pushMatrix();
  translate(-50, 0, 100 * sqrt(3) / 2);
  sphere(10);
  popMatrix();

  pushMatrix();
  translate(100 * sqrt(3) / 2, 0, 50);
  sphere(10);
  popMatrix();

  pushMatrix();
  translate(50, 0, 100 * sqrt(3) / 2);
  sphere(10);
  popMatrix();

  pushMatrix();
  translate(100, 0, 0);
  sphere(10);
  popMatrix();

  pushMatrix();
  translate(-100, 0, 0);
  sphere(10);
  popMatrix();

  pushMatrix();
  translate(0, 0, 100);
  sphere(10);
  popMatrix();

  pushMatrix();
  translate(0, 0, -100);
  sphere(10);
  popMatrix();
}

void block() {
  fill(0);
  stroke(64);

  pushMatrix();
  translate(calculated_list[1], 0, 0);
  box(10, 10, 10);
  popMatrix();

  pushMatrix();
  translate(0, -calculated_list[2], 0);
  box(10, 10, 10);
  popMatrix();

  pushMatrix();
  translate(0, 0, calculated_list[0]);
  box(10, 10, 10);
  popMatrix();
}
